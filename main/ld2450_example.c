/**
 * @file main.c
 * @brief Basic example for using the HLK-LD2450 mmWave radar sensor driver
 * 
 * This example demonstrates:
 * 1. Driver initialization
 * 2. Target detection and data processing
 * 3. Configuration of the radar module
 * 4. JSON formatting of target data
 * 
 * @copyright Copyright (c) 2025 NieRVoid
 * @license MIT License
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_check.h"
#include "ld2450.h"

static const char *TAG = "ld2450_example";

/* GPIO configuration for UART */
#define UART_NUM        UART_NUM_1
#define TXD_PIN         GPIO_NUM_17
#define RXD_PIN         GPIO_NUM_16
#define UART_BAUD_RATE  256000

/* Buffer for JSON data */
#define JSON_BUFFER_SIZE 512
static char json_buffer[JSON_BUFFER_SIZE];

/* Semaphore to signal new data */
static SemaphoreHandle_t data_semaphore = NULL;

/* Latest target data */
static ld2450_target_t latest_targets[LD2450_MAX_TARGETS];
static size_t latest_target_count = 0;

/**
 * @brief Callback function for target data
 */
static void target_data_callback(const ld2450_target_t targets[], size_t count, void *user_ctx)
{
    /* Make a copy of the target data */
    memcpy(latest_targets, targets, sizeof(ld2450_target_t) * LD2450_MAX_TARGETS);
    latest_target_count = count;
    
    /* Signal the main task that new data is available */
    xSemaphoreGive(data_semaphore);
}

/**
 * @brief Display target information
 */
static void display_target_info(const ld2450_target_t *target, int id)
{
    if (!target->active) {
        return;
    }
    
    /* Calculate additional information */
    float distance_m = ld2450_calculate_distance_mm(target) / 1000.0f;
    float angle_deg = ld2450_calculate_angle_degrees(target);
    float speed_mps = ld2450_calculate_speed_mps(target);
    
    /* Display target information */
    ESP_LOGI(TAG, "Target %d: x=%d mm, y=%d mm, speed=%.2f m/s, distance=%.2f m, angle=%.1f°",
             id, target->x, target->y, speed_mps, distance_m, angle_deg);
}

/**
 * @brief Initialize the LD2450 radar module
 */
static esp_err_t init_radar(void)
{
    /* Configure the radar */
    ld2450_config_t config = {
        .uart_port = UART_NUM,
        .uart_baud_rate = UART_BAUD_RATE,
        .uart_tx_pin = TXD_PIN,
        .uart_rx_pin = RXD_PIN
    };
    
    /* Initialize the driver */
    esp_err_t ret = ld2450_init(&config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize LD2450 driver: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Add delay to allow radar to stabilize
    vTaskDelay(pdMS_TO_TICKS(2000)); 
    
    /* Register data callback */
    ret = ld2450_register_data_callback(target_data_callback, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register data callback: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "LD2450 radar initialized successfully");
    return ESP_OK;
}

/**
 * @brief Configure radar settings
 */
static esp_err_t configure_radar(void)
{
    esp_err_t ret;
    char version_str[32];
    
    /* Get firmware version */
    ret = ld2450_cmd_get_firmware_info(version_str, sizeof(version_str));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get firmware info: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "Firmware version: %s", version_str);
    
    /* Configure for multi-target tracking */
    ret = ld2450_cmd_configure_tracking(true);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure tracking mode: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "Configured for multi-target tracking");
    
    /* Optional: Set up detection zones */
    if (false) { // Change to true to enable region filtering
        /* Define a rectangular detection zone in front of the radar */
        /* Coordinates in mm: [x1, y1, x2, y2] for each region */
        int16_t regions[1][4] = {
            {-2000, 0, 2000, 4000}  /* 4m x 4m area in front of the radar */
        };
        
        /* Set detection zone to only include targets in defined regions */
        ret = ld2450_cmd_set_detection_zones(LD2450_REGION_FILTER_INCLUDE, regions, 1);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to set detection zones: %s", esp_err_to_name(ret));
            return ret;
        }
        ESP_LOGI(TAG, "Detection zone configured");
    }
    
    return ESP_OK;
}

/**
 * @brief Process new target data
 */
static void process_target_data(void)
{
    size_t json_len = 0;
    
    /* Count active targets */
    size_t active_count = 0;
    for (size_t i = 0; i < latest_target_count; i++) {
        if (latest_targets[i].active) {
            active_count++;
        }
    }
    
    /* Display information about detected targets */
    ESP_LOGI(TAG, "Detected %u active targets", active_count);
    
    /* Process each target */
    for (size_t i = 0; i < latest_target_count; i++) {
        if (latest_targets[i].active) {
            display_target_info(&latest_targets[i], i);
        }
    }
    
    /* Convert targets to JSON for integration with other systems */
    esp_err_t ret = ld2450_cmd_targets_to_json(latest_targets, latest_target_count, 
                                             json_buffer, JSON_BUFFER_SIZE, &json_len);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "JSON data: %s", json_buffer);
    } else {
        ESP_LOGE(TAG, "Failed to convert targets to JSON: %s", esp_err_to_name(ret));
    }
    
    /* Calculate statistics if multiple targets active */
    if (active_count > 1) {
        int16_t avg_x, avg_y, avg_speed;
        ret = ld2450_calculate_target_statistics(latest_targets, latest_target_count, 
                                              &avg_x, &avg_y, &avg_speed);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Average position: (%d, %d) mm, Average speed: %d cm/s", 
                    avg_x, avg_y, avg_speed);
        }
    }
    
    /* Find closest target to reference point (0,0) */
    if (active_count > 0) {
        size_t closest_idx;
        ret = ld2450_find_closest_target(latest_targets, latest_target_count, 0, 0, &closest_idx);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Closest target to origin is target %u", closest_idx);
        }
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Starting LD2450 radar example");
    
    /* Create semaphore for data signaling */
    data_semaphore = xSemaphoreCreateBinary();
    if (data_semaphore == NULL) {
        ESP_LOGE(TAG, "Failed to create semaphore");
        return;
    }
    
    /* Initialize radar module */
    esp_err_t ret = init_radar();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Initialization failed");
        vSemaphoreDelete(data_semaphore);
        return;
    }
    
    /* Configure radar settings */
    ret = configure_radar();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Configuration failed");
        ld2450_deinit();
        vSemaphoreDelete(data_semaphore);
        return;
    }
    
    ESP_LOGI(TAG, "LD2450 radar ready, waiting for targets...");
    
    /* Main loop - wait for and process target data */
    while (true) {
        /* Wait for new data (with 5 second timeout) */
        if (xSemaphoreTake(data_semaphore, pdMS_TO_TICKS(5000)) == pdTRUE) {
            /* Process the target data */
            process_target_data();
        } else {
            ESP_LOGW(TAG, "No targets detected for 5 seconds");
        }
    }
    
    /* Cleanup (never reached in this example) */
    ld2450_deinit();
    vSemaphoreDelete(data_semaphore);
}