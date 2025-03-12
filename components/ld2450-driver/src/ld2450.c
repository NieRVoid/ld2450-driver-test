/**
 * @file ld2450.c
 * @brief Core implementation for HLK-LD2450 radar sensor driver
 * 
 * This file contains the main implementation of the HLK-LD2450 driver,
 * including initialization, data handling, and UART communication.
 * 
 * @copyright Copyright (c) 2025 NieRVoid
 * @license MIT License
 */

#include <string.h>
#include <math.h>
#include "esp_log.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/uart.h"
#include "esp_timer.h"
#include "ld2450.h"
#include "ld2450_private.h"

/* Define M_PI if not already defined */
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static const char *TAG = "ld2450";

/* Global driver context */
ld2450_context_t g_ld2450_ctx = {0};

/* Pattern to detect data frame header */
static const uint8_t DATA_FRAME_PATTERN[] = {0xAA, 0xFF};
static const int DATA_FRAME_PATTERN_LEN = sizeof(DATA_FRAME_PATTERN);

/* Static function declarations */
static void ld2450_uart_task(void *pvParameters);
static esp_err_t ld2450_uart_init(const ld2450_config_t *config);
static esp_err_t ld2450_uart_deinit(void);
static esp_err_t ld2450_handle_uart_data(uint8_t *data, size_t len);
static esp_err_t ld2450_reset_cmd_parser(void);
static esp_err_t ld2450_reset_data_parser(void);
static esp_err_t ld2450_parse_cmd_byte(uint8_t byte);
static esp_err_t ld2450_parse_data_byte(uint8_t byte);
static void ld2450_swap_data_buffers(void);

esp_err_t ld2450_init(const ld2450_config_t *config)
{
    ESP_RETURN_ON_FALSE(config != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid config");
    ESP_RETURN_ON_FALSE(g_ld2450_ctx.state == LD2450_STATE_UNINITIALIZED, ESP_ERR_INVALID_STATE, 
                        TAG, "Driver already initialized");
    
    /* Clear the context */
    memset(&g_ld2450_ctx, 0, sizeof(ld2450_context_t));
    
    /* Store configuration */
    g_ld2450_ctx.uart_port = config->uart_port;
    g_ld2450_ctx.uart_baud_rate = config->uart_baud_rate;
    
    /* Initialize buffers */
    g_ld2450_ctx.active_buffer = g_ld2450_ctx.data_buffer_a;
    g_ld2450_ctx.processing_buffer = g_ld2450_ctx.data_buffer_b;
    
    /* Create synchronization primitives */
    g_ld2450_ctx.uart_mutex = xSemaphoreCreateMutex();
    if (g_ld2450_ctx.uart_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create UART mutex");
        return ESP_ERR_NO_MEM;
    }
    
    g_ld2450_ctx.cmd_sem = xSemaphoreCreateBinary();
    if (g_ld2450_ctx.cmd_sem == NULL) {
        vSemaphoreDelete(g_ld2450_ctx.uart_mutex);
        ESP_LOGE(TAG, "Failed to create command semaphore");
        return ESP_ERR_NO_MEM;
    }

    // Initialize the semaphore to ensure it's in the correct state
    xSemaphoreGive(g_ld2450_ctx.cmd_sem);
    xSemaphoreTake(g_ld2450_ctx.cmd_sem, 0); // Reset to taken state
    
    /* Initialize UART */
    esp_err_t ret = ld2450_uart_init(config);
    if (ret != ESP_OK) {
        vSemaphoreDelete(g_ld2450_ctx.uart_mutex);
        vSemaphoreDelete(g_ld2450_ctx.cmd_sem);
        ESP_LOGE(TAG, "Failed to initialize UART: %s", esp_err_to_name(ret));
        return ret;
    }
    
    /* Reset parsers */
    ld2450_reset_cmd_parser();
    ld2450_reset_data_parser();
    
    /* Initialize state */
    g_ld2450_ctx.state = LD2450_STATE_IDLE;
    g_ld2450_ctx.config_mode = false;
    
    ESP_LOGI(TAG, "LD2450 driver initialized");
    return ESP_OK;
}

esp_err_t ld2450_deinit(void)
{
    ESP_RETURN_ON_FALSE(g_ld2450_ctx.state != LD2450_STATE_UNINITIALIZED, ESP_ERR_INVALID_STATE,
                        TAG, "Driver not initialized");
    ESP_RETURN_ON_FALSE(g_ld2450_ctx.state != LD2450_STATE_DEINITIALIZING, ESP_ERR_INVALID_STATE,
                        TAG, "Driver already deinitializing");
    
    /* Set state to deinitializing to prevent further operations */
    g_ld2450_ctx.state = LD2450_STATE_DEINITIALIZING;
    
    /* If in config mode, exit it */
    if (g_ld2450_ctx.config_mode) {
        ld2450_exit_config_mode();
    }
    
    /* Deinitialize UART */
    esp_err_t ret = ld2450_uart_deinit();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to deinitialize UART: %s", esp_err_to_name(ret));
        /* Continue with deinitialization despite errors */
    }
    
    /* Free synchronization primitives */
    if (g_ld2450_ctx.uart_mutex != NULL) {
        vSemaphoreDelete(g_ld2450_ctx.uart_mutex);
        g_ld2450_ctx.uart_mutex = NULL;
    }
    
    if (g_ld2450_ctx.cmd_sem != NULL) {
        vSemaphoreDelete(g_ld2450_ctx.cmd_sem);
        g_ld2450_ctx.cmd_sem = NULL;
    }
    
    /* Reset state */
    g_ld2450_ctx.state = LD2450_STATE_UNINITIALIZED;
    g_ld2450_ctx.data_callback = NULL;
    g_ld2450_ctx.user_ctx = NULL;
    
    ESP_LOGI(TAG, "LD2450 driver deinitialized");
    return ESP_OK;
}

esp_err_t ld2450_register_data_callback(ld2450_data_callback_t callback, void *user_ctx)
{
    ESP_RETURN_ON_FALSE(g_ld2450_ctx.state != LD2450_STATE_UNINITIALIZED, ESP_ERR_INVALID_STATE,
                        TAG, "Driver not initialized");
    ESP_RETURN_ON_FALSE(g_ld2450_ctx.state != LD2450_STATE_DEINITIALIZING, ESP_ERR_INVALID_STATE,
                        TAG, "Driver deinitializing");
    ESP_RETURN_ON_FALSE(callback != NULL, ESP_ERR_INVALID_ARG, TAG, "Callback is NULL");
    
    /* Register callback */
    g_ld2450_ctx.data_callback = callback;
    g_ld2450_ctx.user_ctx = user_ctx;
    
    return ESP_OK;
}

esp_err_t ld2450_enter_config_mode(void)
{
    ESP_RETURN_ON_FALSE(g_ld2450_ctx.state == LD2450_STATE_IDLE, ESP_ERR_INVALID_STATE,
                        TAG, "Driver not in idle state");
    
    /* If already in config mode, just return success */
    if (g_ld2450_ctx.config_mode) {
        return ESP_OK;
    }
    
    ESP_LOGI(TAG, "Entering configuration mode");
    
    /* Prepare enable configuration command: 0x00FF with value 0x0001 */
    uint8_t cmd_value[2] = {0x01, 0x00}; /* 0x0001 in little-endian */
    uint8_t ack_data[LD2450_MAX_RETURN_VALUE_SIZE];
    uint16_t ack_len = 0;
    
    /* Send command and wait for ACK */
    esp_err_t ret = ld2450_send_command(LD2450_CMD_ENABLE_CONFIG, cmd_value, 
                                      sizeof(cmd_value), ack_data, &ack_len, 
                                      LD2450_DEFAULT_TIMEOUT_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send enable config command: %s", esp_err_to_name(ret));
        return ret;
    }
    
    /* Check ACK status (first 2 bytes) */
    if (ack_len < 2 || (ack_data[0] != 0 || ack_data[1] != 0)) {
        ESP_LOGE(TAG, "Enable config command failed with status: 0x%02x%02x", 
                 ack_data[1], ack_data[0]);
        return ESP_FAIL;
    }
    
    /* Update state */
    g_ld2450_ctx.config_mode = true;
    g_ld2450_ctx.state = LD2450_STATE_CONFIG;
    
    ESP_LOGI(TAG, "Entered configuration mode");
    return ESP_OK;
}

esp_err_t ld2450_exit_config_mode(void)
{
    ESP_RETURN_ON_FALSE(g_ld2450_ctx.state == LD2450_STATE_CONFIG, ESP_ERR_INVALID_STATE,
                        TAG, "Driver not in config state");
    
    /* If not in config mode, just return success */
    if (!g_ld2450_ctx.config_mode) {
        g_ld2450_ctx.state = LD2450_STATE_IDLE;
        return ESP_OK;
    }
    
    ESP_LOGI(TAG, "Exiting configuration mode");
    
    /* End configuration command: 0x00FE with no value */
    uint8_t ack_data[LD2450_MAX_RETURN_VALUE_SIZE];
    uint16_t ack_len = 0;
    
    /* Send command and wait for ACK */
    esp_err_t ret = ld2450_send_command(LD2450_CMD_END_CONFIG, NULL, 
                                      0, ack_data, &ack_len, 
                                      LD2450_DEFAULT_TIMEOUT_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send end config command: %s", esp_err_to_name(ret));
        return ret;
    }
    
    /* Check ACK status (first 2 bytes) */
    if (ack_len < 2 || (ack_data[0] != 0 || ack_data[1] != 0)) {
        ESP_LOGE(TAG, "End config command failed with status: 0x%02x%02x", 
                 ack_data[1], ack_data[0]);
        return ESP_FAIL;
    }
    
    /* Update state */
    g_ld2450_ctx.config_mode = false;
    g_ld2450_ctx.state = LD2450_STATE_IDLE;
    
    ESP_LOGI(TAG, "Exited configuration mode");
    return ESP_OK;
}

esp_err_t ld2450_send_command(uint16_t cmd_word, const uint8_t *cmd_value, 
                            uint16_t cmd_value_len, uint8_t *ack_data,
                            uint16_t *ack_len, uint32_t timeout_ms)
{
    ESP_RETURN_ON_FALSE(g_ld2450_ctx.state != LD2450_STATE_UNINITIALIZED, ESP_ERR_INVALID_STATE,
                        TAG, "Driver not initialized");
    ESP_RETURN_ON_FALSE(g_ld2450_ctx.state != LD2450_STATE_DEINITIALIZING, ESP_ERR_INVALID_STATE,
                        TAG, "Driver deinitializing");
    ESP_RETURN_ON_FALSE(cmd_value_len <= LD2450_MAX_CMD_VALUE_SIZE, ESP_ERR_INVALID_ARG,
                        TAG, "Command value too large");
    
    esp_err_t ret = ESP_OK;

    // Clear any pending data in RX buffer
    uart_flush_input(g_ld2450_ctx.uart_port);

    // Reset command parser before sending a new command
    ld2450_reset_cmd_parser();

    // Add a small delay before sending the command
    vTaskDelay(pdMS_TO_TICKS(10));
    
    /* Take UART mutex to prevent concurrent command execution */
    if (xSemaphoreTake(g_ld2450_ctx.uart_mutex, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take UART mutex");
        return ESP_ERR_TIMEOUT;
    }
    
    /* Reset command parser */
    ld2450_reset_cmd_parser();
    
    /* Prepare command frame */
    size_t frame_len = 0;
    uint8_t *frame = g_ld2450_ctx.cmd_tx_buffer;
    
    /* Add frame header */
    memcpy(frame + frame_len, LD2450_CMD_FRAME_HEADER, LD2450_FRAME_HEADER_SIZE);
    frame_len += LD2450_FRAME_HEADER_SIZE;
    
    /* Add data length (command word + value length) */
    uint16_t data_len = LD2450_CMD_WORD_SIZE + cmd_value_len;
    frame[frame_len++] = data_len & 0xFF;         /* Low byte */
    frame[frame_len++] = (data_len >> 8) & 0xFF;  /* High byte */
    
    /* Add command word */
    frame[frame_len++] = cmd_word & 0xFF;         /* Low byte */
    frame[frame_len++] = (cmd_word >> 8) & 0xFF;  /* High byte */
    
    /* Add command value if present */
    if (cmd_value != NULL && cmd_value_len > 0) {
        memcpy(frame + frame_len, cmd_value, cmd_value_len);
        frame_len += cmd_value_len;
    }
    
    /* Add frame tail */
    memcpy(frame + frame_len, LD2450_CMD_FRAME_TAIL, LD2450_FRAME_TAIL_SIZE);
    frame_len += LD2450_FRAME_TAIL_SIZE;
    
    /* Update state */
    g_ld2450_ctx.state = LD2450_STATE_COMMAND;
    g_ld2450_ctx.cmd_word = cmd_word;
    
    /* Send command frame */
    ESP_LOGD(TAG, "Sending command 0x%04x", cmd_word);
    int written = uart_write_bytes(g_ld2450_ctx.uart_port, (const char *)frame, frame_len);
    if (written != frame_len) {
        ESP_LOGE(TAG, "Failed to send command frame");
        ret = ESP_FAIL;
        goto cleanup;
    }
    
    /* Wait for ACK */
    g_ld2450_ctx.state = LD2450_STATE_WAITING_ACK;
    if (xSemaphoreTake(g_ld2450_ctx.cmd_sem, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        ESP_LOGE(TAG, "Timeout waiting for ACK");
        ret = ESP_ERR_TIMEOUT;
        goto cleanup;
    }
    
    /* Check last error */
    if (g_ld2450_ctx.last_error != ESP_OK) {
        ESP_LOGE(TAG, "Command failed: %s", esp_err_to_name(g_ld2450_ctx.last_error));
        ret = g_ld2450_ctx.last_error;
        goto cleanup;
    }
    
    /* Copy ACK data if requested */
    if (ack_data != NULL && ack_len != NULL) {
        /* Skip the first 2 bytes which are the command word */
        size_t copy_len = g_ld2450_ctx.last_ack_length > LD2450_MAX_RETURN_VALUE_SIZE ?
                         LD2450_MAX_RETURN_VALUE_SIZE : g_ld2450_ctx.last_ack_length;
        memcpy(ack_data, g_ld2450_ctx.last_ack_data, copy_len);
        *ack_len = copy_len;
    }
    
cleanup:
    /* Restore state */
    if (g_ld2450_ctx.state == LD2450_STATE_COMMAND || 
        g_ld2450_ctx.state == LD2450_STATE_WAITING_ACK) {
        if (g_ld2450_ctx.config_mode) {
            g_ld2450_ctx.state = LD2450_STATE_CONFIG;
        } else {
            g_ld2450_ctx.state = LD2450_STATE_IDLE;
        }
    }
    
    /* Release UART mutex */
    xSemaphoreGive(g_ld2450_ctx.uart_mutex);
    
    return ret;
}

esp_err_t ld2450_process_data_frame(const uint8_t *buffer, size_t length)
{
    ESP_RETURN_ON_FALSE(buffer != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid buffer");
    ESP_RETURN_ON_FALSE(length >= LD2450_DATA_FRAME_HEADER_SIZE + LD2450_DATA_FRAME_TAIL_SIZE,
                        ESP_ERR_INVALID_ARG, TAG, "Buffer too small");
    
    /* Verify frame header (AA FF) */
    if (buffer[0] != 0xAA || buffer[1] != 0xFF) {
        ESP_LOGW(TAG, "Invalid data frame header");
        return ESP_ERR_INVALID_STATE;
    }
    
    /* Verify frame type (03 00) */
    if (buffer[2] != 0x03 || buffer[3] != 0x00) {
        ESP_LOGW(TAG, "Invalid data frame type");
        return ESP_ERR_INVALID_STATE;
    }
    
    /* Verify frame tail (55 CC) */
    if (buffer[length - 2] != 0x55 || buffer[length - 1] != 0xCC) {
        ESP_LOGW(TAG, "Invalid data frame tail");
        return ESP_ERR_INVALID_STATE;
    }
    
    /* Process targets */
    size_t target_count = 0;
    const uint8_t *target_data = buffer + LD2450_DATA_FRAME_HEADER_SIZE;
    
    for (int i = 0; i < LD2450_MAX_TARGETS; i++) {
        /* Check if target is active (non-zero data) */
        bool active = false;
        for (int j = 0; j < LD2450_TARGET_DATA_SIZE; j++) {
            if (target_data[i * LD2450_TARGET_DATA_SIZE + j] != 0) {
                active = true;
                break;
            }
        }
        
        g_ld2450_ctx.targets[i].active = active;
        
        if (active) {
            /* Extract x-coordinate (signed int16) */
            uint16_t x_raw = target_data[i * LD2450_TARGET_DATA_SIZE] | 
                           (target_data[i * LD2450_TARGET_DATA_SIZE + 1] << 8);
            
            /* Extract y-coordinate (signed int16) */
            uint16_t y_raw = target_data[i * LD2450_TARGET_DATA_SIZE + 2] | 
                           (target_data[i * LD2450_TARGET_DATA_SIZE + 3] << 8);
            
            /* Extract speed (signed int16) */
            uint16_t speed_raw = target_data[i * LD2450_TARGET_DATA_SIZE + 4] | 
                               (target_data[i * LD2450_TARGET_DATA_SIZE + 5] << 8);
            
            /* Extract distance resolution */
            uint16_t resolution = target_data[i * LD2450_TARGET_DATA_SIZE + 6] | 
                                (target_data[i * LD2450_TARGET_DATA_SIZE + 7] << 8);
            
            /* Adjust values based on protocol */
            /* For x and y: MSB indicates sign (1 positive, 0 negative) */
            /* For speed: MSB indicates direction (1 positive, 0 negative) */
            int16_t x = (x_raw & 0x8000) ? (x_raw & 0x7FFF) : -(x_raw & 0x7FFF);
            int16_t y = (y_raw & 0x8000) ? (y_raw & 0x7FFF) : -(y_raw & 0x7FFF);
            int16_t speed = (speed_raw & 0x8000) ? (speed_raw & 0x7FFF) : -(speed_raw & 0x7FFF);
            
            /* Store target data */
            g_ld2450_ctx.targets[i].x = x;
            g_ld2450_ctx.targets[i].y = y;
            g_ld2450_ctx.targets[i].speed = speed;
            g_ld2450_ctx.targets[i].resolution = resolution;
            
            target_count++;
        }
    }
    
    /* Update active target count */
    g_ld2450_ctx.active_target_count = target_count;
    
    /* Call user callback if registered */
    if (g_ld2450_ctx.data_callback != NULL) {
        g_ld2450_ctx.data_callback(g_ld2450_ctx.targets, 
                                 g_ld2450_ctx.active_target_count, 
                                 g_ld2450_ctx.user_ctx);
    }
    
    return ESP_OK;
}

static esp_err_t ld2450_uart_init(const ld2450_config_t *config)
{
    /* Configure UART parameters */
    uart_config_t uart_config = {
        .baud_rate = config->uart_baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
        .rx_flow_ctrl_thresh = 122, // Add flow control threshold
    };
    
    /* Install UART driver with event queue */
    esp_err_t ret = uart_driver_install(config->uart_port, 
                                    //   2 * LD2450_MAX_ACK_FRAME_SIZE, /* RX buffer size */
                                      256,
                                      0,                            /* TX buffer size (0 = no buffer) */
                                      10,                           /* Queue size */
                                      &g_ld2450_ctx.uart_queue,     /* Queue handle */
                                      0);                           /* Interrupt allocation flags */
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install UART driver");
        return ret;
    }
    
    /* Set UART parameters */
    ret = uart_param_config(config->uart_port, &uart_config);
    if (ret != ESP_OK) {
        uart_driver_delete(config->uart_port);
        ESP_LOGE(TAG, "Failed to configure UART parameters");
        return ret;
    }
    
    /* Set pins */
    ret = uart_set_pin(config->uart_port, 
                     config->uart_tx_pin, 
                     config->uart_rx_pin,
                     UART_PIN_NO_CHANGE, 
                     UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) {
        uart_driver_delete(config->uart_port);
        ESP_LOGE(TAG, "Failed to set UART pins");
        return ret;
    }
    
    /* Set pattern detection for data frames */
    ret = uart_enable_pattern_det_baud_intr(config->uart_port, 
                                          DATA_FRAME_PATTERN[0], 
                                          DATA_FRAME_PATTERN_LEN, 
                                          1,                      /* Min pattern length */
                                          0,                      /* Pattern match timeout */
                                          0);                     /* Post idle timeout */
    if (ret != ESP_OK) {
        uart_driver_delete(config->uart_port);
        ESP_LOGE(TAG, "Failed to enable pattern detection");
        return ret;
    }

    // Add a small delay to ensure pattern detection is properly set up
    vTaskDelay(pdMS_TO_TICKS(10));
    
    /* Create UART processing task */
    BaseType_t task_ret = xTaskCreate(ld2450_uart_task, 
                                    "ld2450_uart", 
                                    4096,              /* Stack size */
                                    NULL, 
                                    5,                /* Priority */
                                    &g_ld2450_ctx.uart_task);
    if (task_ret != pdPASS) {
        uart_driver_delete(config->uart_port);
        ESP_LOGE(TAG, "Failed to create UART task");
        return ESP_ERR_NO_MEM;
    }
    
    return ESP_OK;
}

static esp_err_t ld2450_uart_deinit(void)
{
    /* Notify task to exit */
    if (g_ld2450_ctx.uart_task != NULL) {
        /* Send task termination event */
        uart_event_t event;
        event.type = UART_EVENT_MAX; /* Use max event as termination signal */
        if (g_ld2450_ctx.uart_queue != NULL) {
            xQueueSend(g_ld2450_ctx.uart_queue, &event, 0);
            
            /* Wait for task to terminate */
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        
        /* Delete task if it's still running */
        vTaskDelete(g_ld2450_ctx.uart_task);
        g_ld2450_ctx.uart_task = NULL;
    }
    
    /* Delete UART driver */
    if (uart_is_driver_installed(g_ld2450_ctx.uart_port)) {
        return uart_driver_delete(g_ld2450_ctx.uart_port);
    }
    
    return ESP_OK;
}

static void ld2450_uart_task(void *pvParameters)
{
    uart_event_t event;
    uint8_t *dtmp = (uint8_t *)malloc(LD2450_MAX_ACK_FRAME_SIZE);
    
    if (dtmp == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for UART task");
        vTaskDelete(NULL);
        return;
    }
    
    while (1) {
        /* Wait for UART event */
        if (xQueueReceive(g_ld2450_ctx.uart_queue, &event, portMAX_DELAY)) {
            /* Break on termination event */
            if (event.type == UART_EVENT_MAX) {
                break;
            }
            
            switch (event.type) {
                case UART_DATA:
                    /* Process incoming data */
                    {
                        int len = uart_read_bytes(g_ld2450_ctx.uart_port, dtmp, 
                                               event.size, LD2450_UART_TIMEOUT_TICKS);
                        if (len > 0) {
                            ld2450_handle_uart_data(dtmp, len);
                        }
                    }
                    break;
                    
                case UART_PATTERN_DET:
                    /* Pattern detected (data frame header) */
                    {
                        int pos = uart_pattern_pop_pos(g_ld2450_ctx.uart_port);
                        
                        if (pos != -1) {
                            /* Only reset data parser if we're not in command mode */
                            if (g_ld2450_ctx.state == LD2450_STATE_IDLE) {
                                ld2450_reset_data_parser();
                            }
                            
                            /* Read and discard data before pattern */
                            if (pos > 0) {
                                uart_read_bytes(g_ld2450_ctx.uart_port, dtmp, pos, LD2450_UART_TIMEOUT_TICKS);
                            }
                        } else {
                            ESP_LOGW(TAG, "Pattern position not found");
                        }
                    }
                    break;
                    
                case UART_FIFO_OVF:
                case UART_BUFFER_FULL:
                    /* Handle overflow by flushing the buffer */
                    uart_flush_input(g_ld2450_ctx.uart_port);
                    xQueueReset(g_ld2450_ctx.uart_queue);
                    ESP_LOGW(TAG, "UART buffer overflow");
                    break;
                    
                default:
                    break;
            }
        }
    }
    
    free(dtmp);
    vTaskDelete(NULL);
}

static esp_err_t ld2450_handle_uart_data(uint8_t *data, size_t len)
{
    esp_err_t ret = ESP_OK;
    
    /* Process each byte */
    for (size_t i = 0; i < len; i++) {
        /* Route the byte to the appropriate parser based on the current state */
        if (g_ld2450_ctx.state == LD2450_STATE_COMMAND || 
            g_ld2450_ctx.state == LD2450_STATE_WAITING_ACK ||
            g_ld2450_ctx.state == LD2450_STATE_CONFIG) {
            /* Command mode - parse for ACKs */
            ret = ld2450_parse_cmd_byte(data[i]);
        } else if (g_ld2450_ctx.state == LD2450_STATE_IDLE) {
            /* Normal mode - parse for data frames */
            ret = ld2450_parse_data_byte(data[i]);
        }
        
        /* Handle errors */
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Error parsing byte: %s", esp_err_to_name(ret));
            /* Continue parsing despite errors */
        }
    }
    
    return ESP_OK;
}

static esp_err_t ld2450_reset_cmd_parser(void)
{
    g_ld2450_ctx.parse_state = LD2450_PARSE_WAIT_HEADER;
    g_ld2450_ctx.bytes_received = 0;
    g_ld2450_ctx.data_length = 0;
    g_ld2450_ctx.cmd_word = 0;
    return ESP_OK;
}

static esp_err_t ld2450_reset_data_parser(void)
{
    g_ld2450_ctx.data_parse_state = LD2450_DATA_PARSE_WAIT_HEADER;
    g_ld2450_ctx.data_bytes_received = 0;
    g_ld2450_ctx.target_index = 0;
    return ESP_OK;
}

static esp_err_t ld2450_parse_cmd_byte(uint8_t byte)
{
    /* Store byte in the receive buffer */
    if (g_ld2450_ctx.bytes_received < LD2450_MAX_ACK_FRAME_SIZE) {
        g_ld2450_ctx.cmd_rx_buffer[g_ld2450_ctx.bytes_received++] = byte;
    } else {
        /* Buffer overflow, reset parser */
        ESP_LOGW(TAG, "Command receive buffer overflow");
        ld2450_reset_cmd_parser();
        return ESP_ERR_NO_MEM;
    }
    
    /* Process byte based on current parse state */
    switch (g_ld2450_ctx.parse_state) {
        case LD2450_PARSE_WAIT_HEADER:
            /* Check if we've received the complete header */
            if (g_ld2450_ctx.bytes_received == LD2450_FRAME_HEADER_SIZE) {
                /* Verify header */
                if (memcmp(g_ld2450_ctx.cmd_rx_buffer, LD2450_CMD_FRAME_HEADER, 
                           LD2450_FRAME_HEADER_SIZE) == 0) {
                    /* Header is valid, move to next state */
                    g_ld2450_ctx.parse_state = LD2450_PARSE_WAIT_LENGTH;
                } else {
                    /* Invalid header, reset parser */
                    ld2450_reset_cmd_parser();
                    return ESP_ERR_INVALID_RESPONSE;
                }
            }
            break;
            
        case LD2450_PARSE_WAIT_LENGTH:
            /* Check if we've received the complete length field */
            if (g_ld2450_ctx.bytes_received == LD2450_FRAME_HEADER_SIZE + 
                                             LD2450_DATA_LENGTH_SIZE) {
                /* Extract data length */
                g_ld2450_ctx.data_length = g_ld2450_ctx.cmd_rx_buffer[LD2450_FRAME_HEADER_SIZE] | 
                                         (g_ld2450_ctx.cmd_rx_buffer[LD2450_FRAME_HEADER_SIZE + 1] << 8);
                
                /* Verify data length */
                if (g_ld2450_ctx.data_length > 0 && 
                    g_ld2450_ctx.data_length <= LD2450_MAX_RETURN_VALUE_SIZE + 2) {
                    /* Length is valid, move to next state */
                    g_ld2450_ctx.parse_state = LD2450_PARSE_WAIT_DATA;
                } else {
                    /* Invalid length, reset parser */
                    ESP_LOGW(TAG, "Invalid data length: %d", g_ld2450_ctx.data_length);
                    ld2450_reset_cmd_parser();
                    return ESP_ERR_INVALID_SIZE;
                }
            }
            break;
            
        case LD2450_PARSE_WAIT_DATA:
            /* Check if we've received all data */
            if (g_ld2450_ctx.bytes_received == LD2450_FRAME_HEADER_SIZE + 
                                             LD2450_DATA_LENGTH_SIZE + 
                                             g_ld2450_ctx.data_length) {
                /* Move to tail state */
                g_ld2450_ctx.parse_state = LD2450_PARSE_WAIT_TAIL;
                
                /* Extract command word from ACK data */
                g_ld2450_ctx.cmd_word = g_ld2450_ctx.cmd_rx_buffer[LD2450_FRAME_HEADER_SIZE + 
                                                                LD2450_DATA_LENGTH_SIZE] | 
                                     (g_ld2450_ctx.cmd_rx_buffer[LD2450_FRAME_HEADER_SIZE + 
                                                                LD2450_DATA_LENGTH_SIZE + 1] << 8);
            }
            break;
            
        case LD2450_PARSE_WAIT_TAIL:
            /* Check if we've received the complete tail */
            if (g_ld2450_ctx.bytes_received == LD2450_FRAME_HEADER_SIZE + 
                                             LD2450_DATA_LENGTH_SIZE + 
                                             g_ld2450_ctx.data_length + 
                                             LD2450_FRAME_TAIL_SIZE) {
                /* Verify tail */
                if (memcmp(&g_ld2450_ctx.cmd_rx_buffer[g_ld2450_ctx.bytes_received - 
                                                    LD2450_FRAME_TAIL_SIZE], 
                           LD2450_CMD_FRAME_TAIL, LD2450_FRAME_TAIL_SIZE) == 0) {
                    /* Complete frame received */
                    ESP_LOGD(TAG, "Received ACK for command 0x%04x", g_ld2450_ctx.cmd_word);
                    
                    /* Extract ACK data (skip command word) */
                    size_t ack_data_len = g_ld2450_ctx.data_length - LD2450_CMD_WORD_SIZE;
                    if (ack_data_len > 0 && ack_data_len <= LD2450_MAX_RETURN_VALUE_SIZE) {
                        memcpy(g_ld2450_ctx.last_ack_data, 
                               &g_ld2450_ctx.cmd_rx_buffer[LD2450_FRAME_HEADER_SIZE + 
                                                        LD2450_DATA_LENGTH_SIZE + 
                                                        LD2450_CMD_WORD_SIZE], 
                               ack_data_len);
                        g_ld2450_ctx.last_ack_length = ack_data_len;
                        
                        /* Extract ACK status (first 2 bytes) */
                        if (ack_data_len >= 2) {
                            g_ld2450_ctx.last_ack_status = g_ld2450_ctx.last_ack_data[0] | 
                                                         (g_ld2450_ctx.last_ack_data[1] << 8);
                            
                            /* Check status */
                            if (g_ld2450_ctx.last_ack_status == LD2450_ACK_SUCCESS) {
                                g_ld2450_ctx.last_error = ESP_OK;
                            } else {
                                g_ld2450_ctx.last_error = ESP_FAIL;
                                ESP_LOGW(TAG, "Command 0x%04x failed with status 0x%04x", 
                                         g_ld2450_ctx.cmd_word, g_ld2450_ctx.last_ack_status);
                            }
                        } else {
                            ESP_LOGW(TAG, "ACK data too short to contain status");
                            g_ld2450_ctx.last_error = ESP_ERR_INVALID_RESPONSE;
                        }
                    } else {
                        ESP_LOGW(TAG, "Invalid ACK data length: %d", ack_data_len);
                        g_ld2450_ctx.last_error = ESP_ERR_INVALID_SIZE;
                    }
                    
                    /* Notify waiting thread */
                    xSemaphoreGive(g_ld2450_ctx.cmd_sem);
                    
                    /* Reset parser */
                    ld2450_reset_cmd_parser();
                } else {
                    /* Invalid tail, reset parser */
                    ESP_LOGW(TAG, "Invalid frame tail");
                    ld2450_reset_cmd_parser();
                    return ESP_ERR_INVALID_RESPONSE;
                }
            }
            break;
    }
    
    return ESP_OK;
}

static esp_err_t ld2450_parse_data_byte(uint8_t byte)
{
    /* Store byte in the active buffer */
    if (g_ld2450_ctx.data_bytes_received < LD2450_MAX_DATA_FRAME_SIZE) {
        g_ld2450_ctx.active_buffer[g_ld2450_ctx.data_bytes_received++] = byte;
    } else {
        /* Buffer overflow, reset parser */
        ESP_LOGW(TAG, "Data receive buffer overflow");
        ld2450_reset_data_parser();
        return ESP_ERR_NO_MEM;
    }
    
    /* Process byte based on current parse state */
    switch (g_ld2450_ctx.data_parse_state) {
        case LD2450_DATA_PARSE_WAIT_HEADER:
            /* Check for header sequence (AA FF) */
            if (g_ld2450_ctx.data_bytes_received == 2) {
                if (g_ld2450_ctx.active_buffer[0] == 0xAA && 
                    g_ld2450_ctx.active_buffer[1] == 0xFF) {
                    /* Header found, move to next state */
                    g_ld2450_ctx.data_parse_state = LD2450_DATA_PARSE_WAIT_TYPE;
                } else {
                    /* Invalid header, reset parser */
                    ld2450_reset_data_parser();
                    return ESP_ERR_INVALID_RESPONSE;
                }
            }
            break;
            
        case LD2450_DATA_PARSE_WAIT_TYPE:
            /* Check for data frame type (03 00) */
            if (g_ld2450_ctx.data_bytes_received == 4) {
                if (g_ld2450_ctx.active_buffer[2] == 0x03 && 
                    g_ld2450_ctx.active_buffer[3] == 0x00) {
                    /* Type is valid, move to next state */
                    g_ld2450_ctx.data_parse_state = LD2450_DATA_PARSE_WAIT_TARGETS;
                    g_ld2450_ctx.target_index = 0;
                } else {
                    /* Invalid type, reset parser */
                    ld2450_reset_data_parser();
                    return ESP_ERR_INVALID_RESPONSE;
                }
            }
            break;
            
        case LD2450_DATA_PARSE_WAIT_TARGETS:
            /* Check if we've received all target data */
            size_t expected_target_bytes = LD2450_MAX_TARGETS * LD2450_TARGET_DATA_SIZE;
            if (g_ld2450_ctx.data_bytes_received == LD2450_DATA_FRAME_HEADER_SIZE + 
                                                 expected_target_bytes) {
                /* All target data received, move to tail state */
                g_ld2450_ctx.data_parse_state = LD2450_DATA_PARSE_WAIT_TAIL;
            }
            break;
            
        case LD2450_DATA_PARSE_WAIT_TAIL:
            /* Check for tail sequence (55 CC) */
            if (g_ld2450_ctx.data_bytes_received == LD2450_DATA_FRAME_HEADER_SIZE + 
                                                 (LD2450_MAX_TARGETS * LD2450_TARGET_DATA_SIZE) + 
                                                 LD2450_DATA_FRAME_TAIL_SIZE) {
                /* Check tail bytes */
                size_t tail_offset = g_ld2450_ctx.data_bytes_received - LD2450_DATA_FRAME_TAIL_SIZE;
                if (g_ld2450_ctx.active_buffer[tail_offset] == 0x55 && 
                    g_ld2450_ctx.active_buffer[tail_offset + 1] == 0xCC) {
                    /* Complete frame received, swap buffers and process data */
                    ld2450_swap_data_buffers();
                    
                    /* Process the data frame in the processing buffer */
                    ld2450_process_data_frame(g_ld2450_ctx.processing_buffer, 
                                           g_ld2450_ctx.data_bytes_received);
                    
                    /* Reset parser for next frame */
                    ld2450_reset_data_parser();
                } else {
                    /* Invalid tail, reset parser */
                    ld2450_reset_data_parser();
                    return ESP_ERR_INVALID_RESPONSE;
                }
            }
            break;
    }
    
    return ESP_OK;
}

static void ld2450_swap_data_buffers(void)
{
    /* Atomically swap the active and processing buffers */
    uint8_t *temp = g_ld2450_ctx.active_buffer;
    g_ld2450_ctx.active_buffer = g_ld2450_ctx.processing_buffer;
    g_ld2450_ctx.processing_buffer = temp;
}

/* Implementation of command APIs */

esp_err_t ld2450_set_tracking_mode(bool multi_target)
{
    ESP_RETURN_ON_FALSE(g_ld2450_ctx.state == LD2450_STATE_CONFIG, ESP_ERR_INVALID_STATE,
                        TAG, "Not in configuration mode");
    
    uint16_t cmd = multi_target ? LD2450_CMD_MULTI_TARGET : LD2450_CMD_SINGLE_TARGET;
    uint8_t ack_data[LD2450_MAX_RETURN_VALUE_SIZE];
    uint16_t ack_len = 0;
    
    /* Send command and wait for ACK */
    esp_err_t ret = ld2450_send_command(cmd, NULL, 0, ack_data, &ack_len, 
                                      LD2450_DEFAULT_TIMEOUT_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set tracking mode: %s", esp_err_to_name(ret));
        return ret;
    }
    
    /* Check ACK status (first 2 bytes) */
    if (ack_len < 2 || (ack_data[0] != 0 || ack_data[1] != 0)) {
        ESP_LOGE(TAG, "Set tracking mode command failed with status: 0x%02x%02x", 
                 ack_data[1], ack_data[0]);
        return ESP_FAIL;
    }
    
    return ESP_OK;
}

esp_err_t ld2450_get_tracking_mode(bool *multi_target)
{
    ESP_RETURN_ON_FALSE(g_ld2450_ctx.state == LD2450_STATE_CONFIG, ESP_ERR_INVALID_STATE,
                        TAG, "Not in configuration mode");
    ESP_RETURN_ON_FALSE(multi_target != NULL, ESP_ERR_INVALID_ARG,
                        TAG, "multi_target is NULL");
    
    uint8_t ack_data[LD2450_MAX_RETURN_VALUE_SIZE];
    uint16_t ack_len = 0;
    
    /* Send query command and wait for ACK */
    esp_err_t ret = ld2450_send_command(LD2450_CMD_QUERY_TARGET_MODE, NULL, 0, 
                                      ack_data, &ack_len, LD2450_DEFAULT_TIMEOUT_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to query tracking mode: %s", esp_err_to_name(ret));
        return ret;
    }
    
    /* Check ACK status (first 2 bytes) */
    if (ack_len < 4 || (ack_data[0] != 0 || ack_data[1] != 0)) {
        ESP_LOGE(TAG, "Query tracking mode command failed with status: 0x%02x%02x", 
                 ack_data[1], ack_data[0]);
        return ESP_FAIL;
    }
    
    /* Extract tracking mode (bytes 2-3) */
    uint16_t mode = ack_data[2] | (ack_data[3] << 8);
    *multi_target = (mode == 0x0002); /* 0x0001 = single, 0x0002 = multi */
    
    return ESP_OK;
}

esp_err_t ld2450_get_firmware_version(ld2450_firmware_version_t *version)
{
    ESP_RETURN_ON_FALSE(g_ld2450_ctx.state == LD2450_STATE_CONFIG, ESP_ERR_INVALID_STATE,
                        TAG, "Not in configuration mode");
    ESP_RETURN_ON_FALSE(version != NULL, ESP_ERR_INVALID_ARG, TAG, "version is NULL");
    
    uint8_t ack_data[LD2450_MAX_RETURN_VALUE_SIZE];
    uint16_t ack_len = 0;
    
    /* Send command and wait for ACK */
    esp_err_t ret = ld2450_send_command(LD2450_CMD_READ_FW_VERSION, NULL, 0, 
                                      ack_data, &ack_len, LD2450_DEFAULT_TIMEOUT_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read firmware version: %s", esp_err_to_name(ret));
        return ret;
    }
    
    /* Check ACK status (first 2 bytes) */
    if (ack_len < 8 || (ack_data[0] != 0 || ack_data[1] != 0)) {
        ESP_LOGE(TAG, "Read firmware version command failed with status: 0x%02x%02x", 
                 ack_data[1], ack_data[0]);
        return ESP_FAIL;
    }
    
    /* Skip firmware type (2 bytes) */
    /* Extract main version (bytes 4-5) */
    version->main_version = ack_data[4] | (ack_data[5] << 8);
    
    /* Extract sub-version (bytes 6-9) */
    version->sub_version = ack_data[6] | 
                         (ack_data[7] << 8) | 
                         (ack_data[8] << 16) | 
                         (ack_data[9] << 24);
    
    return ESP_OK;
}

esp_err_t ld2450_set_baud_rate(ld2450_baud_rate_t baud_rate)
{
    ESP_RETURN_ON_FALSE(g_ld2450_ctx.state == LD2450_STATE_CONFIG, ESP_ERR_INVALID_STATE,
                        TAG, "Not in configuration mode");
    ESP_RETURN_ON_FALSE(baud_rate >= LD2450_BAUD_RATE_9600 && 
                      baud_rate <= LD2450_BAUD_RATE_460800, ESP_ERR_INVALID_ARG,
                      TAG, "Invalid baud rate");
    
    uint8_t cmd_value[2];
    cmd_value[0] = baud_rate & 0xFF;
    cmd_value[1] = (baud_rate >> 8) & 0xFF;
    
    uint8_t ack_data[LD2450_MAX_RETURN_VALUE_SIZE];
    uint16_t ack_len = 0;
    
    /* Send command and wait for ACK */
    esp_err_t ret = ld2450_send_command(LD2450_CMD_SET_BAUD_RATE, cmd_value, 
                                      sizeof(cmd_value), ack_data, &ack_len, 
                                      LD2450_DEFAULT_TIMEOUT_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set baud rate: %s", esp_err_to_name(ret));
        return ret;
    }
    
    /* Check ACK status (first 2 bytes) */
    if (ack_len < 2 || (ack_data[0] != 0 || ack_data[1] != 0)) {
        ESP_LOGE(TAG, "Set baud rate command failed with status: 0x%02x%02x", 
                 ack_data[1], ack_data[0]);
        return ESP_FAIL;
    }
    
    return ESP_OK;
}

esp_err_t ld2450_restore_factory_settings(void)
{
    ESP_RETURN_ON_FALSE(g_ld2450_ctx.state == LD2450_STATE_CONFIG, ESP_ERR_INVALID_STATE,
                        TAG, "Not in configuration mode");
    
    uint8_t ack_data[LD2450_MAX_RETURN_VALUE_SIZE];
    uint16_t ack_len = 0;
    
    /* Send command and wait for ACK */
    esp_err_t ret = ld2450_send_command(LD2450_CMD_RESTORE_FACTORY, NULL, 0, 
                                      ack_data, &ack_len, LD2450_DEFAULT_TIMEOUT_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to restore factory settings: %s", esp_err_to_name(ret));
        return ret;
    }
    
    /* Check ACK status (first 2 bytes) */
    if (ack_len < 2 || (ack_data[0] != 0 || ack_data[1] != 0)) {
        ESP_LOGE(TAG, "Restore factory settings command failed with status: 0x%02x%02x", 
                 ack_data[1], ack_data[0]);
        return ESP_FAIL;
    }
    
    return ESP_OK;
}

esp_err_t ld2450_restart(void)
{
    ESP_RETURN_ON_FALSE(g_ld2450_ctx.state == LD2450_STATE_CONFIG, ESP_ERR_INVALID_STATE,
                        TAG, "Not in configuration mode");
    
    uint8_t ack_data[LD2450_MAX_RETURN_VALUE_SIZE];
    uint16_t ack_len = 0;
    
    /* Send command and wait for ACK */
    esp_err_t ret = ld2450_send_command(LD2450_CMD_RESTART, NULL, 0, 
                                      ack_data, &ack_len, LD2450_DEFAULT_TIMEOUT_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to restart module: %s", esp_err_to_name(ret));
        return ret;
    }
    
    /* Check ACK status (first 2 bytes) */
    if (ack_len < 2 || (ack_data[0] != 0 || ack_data[1] != 0)) {
        ESP_LOGE(TAG, "Restart command failed with status: 0x%02x%02x", 
                 ack_data[1], ack_data[0]);
        return ESP_FAIL;
    }
    
    /* Update state */
    g_ld2450_ctx.config_mode = false;
    g_ld2450_ctx.state = LD2450_STATE_IDLE;
    
    return ESP_OK;
}

esp_err_t ld2450_set_bluetooth(bool enable)
{
    ESP_RETURN_ON_FALSE(g_ld2450_ctx.state == LD2450_STATE_CONFIG, ESP_ERR_INVALID_STATE,
                        TAG, "Not in configuration mode");
    
    uint8_t cmd_value[2];
    cmd_value[0] = enable ? 0x01 : 0x00;
    cmd_value[1] = 0x00;
    
    uint8_t ack_data[LD2450_MAX_RETURN_VALUE_SIZE];
    uint16_t ack_len = 0;
    
    /* Send command and wait for ACK */
    esp_err_t ret = ld2450_send_command(LD2450_CMD_BLUETOOTH, cmd_value, 
                                      sizeof(cmd_value), ack_data, &ack_len, 
                                      LD2450_DEFAULT_TIMEOUT_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set bluetooth: %s", esp_err_to_name(ret));
        return ret;
    }
    
    /* Check ACK status (first 2 bytes) */
    if (ack_len < 2 || (ack_data[0] != 0 || ack_data[1] != 0)) {
        ESP_LOGE(TAG, "Set bluetooth command failed with status: 0x%02x%02x", 
                 ack_data[1], ack_data[0]);
        return ESP_FAIL;
    }
    
    return ESP_OK;
}

esp_err_t ld2450_get_mac_address(uint8_t mac[6])
{
    ESP_RETURN_ON_FALSE(g_ld2450_ctx.state == LD2450_STATE_CONFIG, ESP_ERR_INVALID_STATE,
                        TAG, "Not in configuration mode");
    ESP_RETURN_ON_FALSE(mac != NULL, ESP_ERR_INVALID_ARG, TAG, "mac is NULL");
    
    uint8_t cmd_value[2] = {0x01, 0x00}; /* 0x0001 in little-endian */
    uint8_t ack_data[LD2450_MAX_RETURN_VALUE_SIZE];
    uint16_t ack_len = 0;
    
    /* Send command and wait for ACK */
    esp_err_t ret = ld2450_send_command(LD2450_CMD_GET_MAC, cmd_value, 
                                      sizeof(cmd_value), ack_data, &ack_len, 
                                      LD2450_DEFAULT_TIMEOUT_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get MAC address: %s", esp_err_to_name(ret));
        return ret;
    }
    
    /* Check ACK status (first 2 bytes) */
    if (ack_len < 8 || (ack_data[0] != 0 || ack_data[1] != 0)) {
        ESP_LOGE(TAG, "Get MAC address command failed with status: 0x%02x%02x", 
                 ack_data[1], ack_data[0]);
        return ESP_FAIL;
    }
    
    /* Extract MAC address (bytes 3-8) */
    memcpy(mac, &ack_data[2], 6);
    
    return ESP_OK;
}

esp_err_t ld2450_get_region_filter(ld2450_region_filter_mode_t *mode, 
                                 int16_t regions[][4], 
                                 size_t *region_count)
{
    ESP_RETURN_ON_FALSE(g_ld2450_ctx.state == LD2450_STATE_CONFIG, ESP_ERR_INVALID_STATE,
                        TAG, "Not in configuration mode");
    ESP_RETURN_ON_FALSE(mode != NULL, ESP_ERR_INVALID_ARG, TAG, "mode is NULL");
    ESP_RETURN_ON_FALSE(regions != NULL, ESP_ERR_INVALID_ARG, TAG, "regions is NULL");
    ESP_RETURN_ON_FALSE(region_count != NULL, ESP_ERR_INVALID_ARG, TAG, "region_count is NULL");
    
    uint8_t ack_data[LD2450_MAX_RETURN_VALUE_SIZE];
    uint16_t ack_len = 0;
    
    /* Send command and wait for ACK */
    esp_err_t ret = ld2450_send_command(LD2450_CMD_QUERY_REGION, NULL, 0, 
                                      ack_data, &ack_len, LD2450_DEFAULT_TIMEOUT_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to query region filter: %s", esp_err_to_name(ret));
        return ret;
    }
    
    /* Check ACK status (first 2 bytes) */
    if (ack_len < 4 || (ack_data[0] != 0 || ack_data[1] != 0)) {
        ESP_LOGE(TAG, "Query region filter command failed with status: 0x%02x%02x", 
                 ack_data[1], ack_data[0]);
        return ESP_FAIL;
    }
    
    /* Extract region filtering mode (bytes 2-3) */
    *mode = (ld2450_region_filter_mode_t)(ack_data[2] | (ack_data[3] << 8));
    
    /* Count active regions and extract coordinates */
    *region_count = 0;
    
    /* Parse each region (up to 3) */
    for (size_t i = 0; i < 3; i++) {
        /* Each region has 4 coordinates (x1, y1, x2, y2) each 2 bytes */
        size_t region_offset = 4 + (i * 8);
        
        /* Check if we have enough data */
        if (ack_len < region_offset + 8) {
            break;
        }
        
        /* Check if region is active (non-zero coordinates) */
        bool active = false;
        for (size_t j = 0; j < 8; j++) {
            if (ack_data[region_offset + j] != 0) {
                active = true;
                break;
            }
        }
        
        if (active) {
            /* Extract the coordinates */
            for (size_t j = 0; j < 4; j++) {
                regions[*region_count][j] = ack_data[region_offset + (j * 2)] |
                                          (ack_data[region_offset + (j * 2) + 1] << 8);
            }
            
            (*region_count)++;
        }
    }
    
    return ESP_OK;
}

esp_err_t ld2450_set_region_filter(ld2450_region_filter_mode_t mode,
                                 const int16_t regions[][4],
                                 size_t region_count)
{
    ESP_RETURN_ON_FALSE(g_ld2450_ctx.state == LD2450_STATE_CONFIG, ESP_ERR_INVALID_STATE,
                        TAG, "Not in configuration mode");
    ESP_RETURN_ON_FALSE(mode <= LD2450_REGION_FILTER_EXCLUDE, ESP_ERR_INVALID_ARG,
                        TAG, "Invalid filter mode");
    ESP_RETURN_ON_FALSE(region_count <= 3, ESP_ERR_INVALID_ARG, 
                        TAG, "Too many regions (max 3)");
    ESP_RETURN_ON_FALSE(mode == LD2450_REGION_FILTER_DISABLED || 
                      (regions != NULL && region_count > 0), ESP_ERR_INVALID_ARG,
                      TAG, "Regions required when filtering is enabled");
    
    /* Prepare command value */
    uint8_t cmd_value[26]; /* 2 bytes mode + 24 bytes region coordinates (3 regions * 4 coords * 2 bytes) */
    memset(cmd_value, 0, sizeof(cmd_value));
    
    /* Set filtering mode */
    cmd_value[0] = mode & 0xFF;
    cmd_value[1] = (mode >> 8) & 0xFF;
    
    /* Set region coordinates */
    for (size_t i = 0; i < region_count && i < 3; i++) {
        for (size_t j = 0; j < 4; j++) {
            size_t offset = 2 + (i * 8) + (j * 2);
            cmd_value[offset] = regions[i][j] & 0xFF;
            cmd_value[offset + 1] = (regions[i][j] >> 8) & 0xFF;
        }
    }
    
    uint8_t ack_data[LD2450_MAX_RETURN_VALUE_SIZE];
    uint16_t ack_len = 0;
    
    /* Send command and wait for ACK */
    esp_err_t ret = ld2450_send_command(LD2450_CMD_SET_REGION, cmd_value, 
                                      sizeof(cmd_value), ack_data, &ack_len, 
                                      LD2450_DEFAULT_TIMEOUT_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set region filter: %s", esp_err_to_name(ret));
        return ret;
    }
    
    /* Check ACK status (first 2 bytes) */
    if (ack_len < 2 || (ack_data[0] != 0 || ack_data[1] != 0)) {
        ESP_LOGE(TAG, "Set region filter command failed with status: 0x%02x%02x", 
                 ack_data[1], ack_data[0]);
        return ESP_FAIL;
    }
    
    return ESP_OK;
}