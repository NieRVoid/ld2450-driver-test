/**
 * @file ld2450.c
 * @brief Implementation of the HLK-LD2450 radar driver for ESP-IDF
 *
 * Driver for interfacing with the HLK-LD2450 24GHz radar sensor using ESP32-IDF.
 *
 * @copyright Copyright (c) 2025 NieRVoid
 * @license MIT License
 */

 #include "ld2450.h"
 #include "protocol.h"
 
 #include "esp_log.h"
 #include "esp_check.h"
 #include "freertos/FreeRTOS.h"
 #include "freertos/task.h"
 #include "freertos/queue.h"
 #include "freertos/semphr.h"
 #include "freertos/event_groups.h"
 #include "driver/uart.h"
 #include "esp_timer.h"
 #include <string.h>
 #include <limits.h>
 
 static const char *TAG = "ld2450";
 
 ESP_EVENT_DEFINE_BASE(LD2450_EVENTS);
 
 // Event group bit definitions
 #define LD2450_EVENT_STOP_REQUEST   (1 << 0)  // Request to stop the task
 #define LD2450_EVENT_TASK_STOPPED   (1 << 1)  // Task has stopped
 
 // Task notification values
 #define LD2450_NOTIFICATION_STOP    1  // Stop task notification value
 
 /**
  * @brief Driver state structure
  */
 typedef struct {
     bool initialized;                      /*!< Initialization flag */
     bool config_mode;                      /*!< Configuration mode flag */
     uart_port_t uart_port;                 /*!< UART port */
     SemaphoreHandle_t uart_mutex;          /*!< UART mutex for thread safety */
     QueueHandle_t data_queue;              /*!< Queue for radar data frames */
     TaskHandle_t receiver_task;            /*!< Receiver task handle */
     ld2450_data_callback_t data_callback;  /*!< Data callback function */
     void *callback_user_data;              /*!< User data for callback */
     ld2450_frame_data_t latest_data;       /*!< Most recent frame data */
     bool event_loop_enabled;               /*!< Event loop flag */
     EventGroupHandle_t event_group;        /*!< Event group for synchronization */
 } ld2450_driver_t;
 
 // Driver instance (singleton)
 static ld2450_driver_t s_driver = {
     .initialized = false,
     .config_mode = false,
     .uart_port = UART_NUM_MAX,
     .uart_mutex = NULL,
     .data_queue = NULL,
     .receiver_task = NULL,
     .data_callback = NULL,
     .callback_user_data = NULL,
     .event_loop_enabled = false,
     .event_group = NULL
 };
 
 // Forward declarations for internal functions
 static void ld2450_receiver_task(void *arg);
 static esp_err_t ld2450_send_command(ld2450_cmd_t cmd, const void *data, size_t data_len);
 static esp_err_t ld2450_receive_response(ld2450_cmd_t cmd, void *response, size_t *response_len, uint32_t timeout_ms);
 static esp_err_t ld2450_process_data_frame(uint8_t *data, size_t len, ld2450_frame_data_t *frame);
 
 /**
  * @brief Convert driver baud rate enum to actual baudrate value
  */
 static uint32_t ld2450_baud_to_value(ld2450_baud_rate_t baud_rate)
 {
     switch (baud_rate) {
         case LD2450_BAUD_RATE_9600:   return 9600;
         case LD2450_BAUD_RATE_19200:  return 19200;
         case LD2450_BAUD_RATE_38400:  return 38400;
         case LD2450_BAUD_RATE_57600:  return 57600;
         case LD2450_BAUD_RATE_115200: return 115200;
         case LD2450_BAUD_RATE_230400: return 230400;
         case LD2450_BAUD_RATE_256000: return 256000;
         case LD2450_BAUD_RATE_460800: return 460800;
         default:                       return 256000; // Default
     }
 }
 
 /**
  * @brief Convert driver baud rate enum to protocol baud rate index
  */
 static uint16_t ld2450_baud_to_index(ld2450_baud_rate_t baud_rate)
 {
     switch (baud_rate) {
         case LD2450_BAUD_RATE_9600:   return LD2450_BAUD_9600;
         case LD2450_BAUD_RATE_19200:  return LD2450_BAUD_19200;
         case LD2450_BAUD_RATE_38400:  return LD2450_BAUD_38400;
         case LD2450_BAUD_RATE_57600:  return LD2450_BAUD_57600;
         case LD2450_BAUD_RATE_115200: return LD2450_BAUD_115200;
         case LD2450_BAUD_RATE_230400: return LD2450_BAUD_230400;
         case LD2450_BAUD_RATE_256000: return LD2450_BAUD_256000;
         case LD2450_BAUD_RATE_460800: return LD2450_BAUD_460800;
         default:                       return LD2450_BAUD_256000; // Default
     }
 }
 
 /**
  * @brief Convert driver tracking mode enum to protocol tracking mode
  */
 static uint16_t ld2450_tracking_mode_to_protocol(ld2450_tracking_mode_t mode)
 {
     return (mode == LD2450_MODE_SINGLE_TARGET) ? LD2450_TRACK_SINGLE : LD2450_TRACK_MULTI;
 }
 
 /**
  * @brief Convert protocol tracking mode to driver tracking mode enum
  */
 static ld2450_tracking_mode_t ld2450_protocol_to_tracking_mode(uint16_t mode)
 {
     return (mode == LD2450_TRACK_SINGLE) ? LD2450_MODE_SINGLE_TARGET : LD2450_MODE_MULTI_TARGET;
 }
 
 /**
  * @brief Convert driver filter mode enum to protocol filter type
  */
 static uint16_t ld2450_filter_mode_to_protocol(ld2450_filter_mode_t mode)
 {
     switch (mode) {
         case LD2450_FILTER_NONE:         return LD2450_FILTER_DISABLE;
         case LD2450_FILTER_INSIDE_REGION: return LD2450_FILTER_INSIDE;
         case LD2450_FILTER_OUTSIDE_REGION: return LD2450_FILTER_OUTSIDE;
         default:                         return LD2450_FILTER_DISABLE;
     }
 }
 
 /**
  * @brief Convert protocol filter type to driver filter mode enum
  */
 static ld2450_filter_mode_t ld2450_protocol_to_filter_mode(uint16_t type)
 {
     switch (type) {
         case LD2450_FILTER_DISABLE: return LD2450_FILTER_NONE;
         case LD2450_FILTER_INSIDE:  return LD2450_FILTER_INSIDE_REGION;
         case LD2450_FILTER_OUTSIDE: return LD2450_FILTER_OUTSIDE_REGION;
         default:                    return LD2450_FILTER_NONE;
     }
 }
 
 /**
  * @brief Safely take the UART mutex with shutdown check
  * 
  * @param timeout_ms Timeout in milliseconds
  * @return true if mutex was successfully taken, false if not
  */
 static bool safe_mutex_take(uint32_t timeout_ms) {
     // Check if we're shutting down
     if (s_driver.event_group != NULL) {
         EventBits_t bits = xEventGroupGetBits(s_driver.event_group);
         if ((bits & LD2450_EVENT_STOP_REQUEST) != 0) {
             return false;
         }
     }
     
     // Check if mutex exists and can be taken
     if (s_driver.uart_mutex != NULL && 
         xSemaphoreTake(s_driver.uart_mutex, pdMS_TO_TICKS(timeout_ms)) == pdTRUE) {
         return true;
     }
     return false;
 }
 
 /**
  * @brief Safely give the UART mutex
  */
 static void safe_mutex_give(void) {
     if (s_driver.uart_mutex != NULL) {
         xSemaphoreGive(s_driver.uart_mutex);
     }
 }
 
 /**
  * @brief Check if task should stop
  * 
  * @return true if task should stop, false otherwise
  */
 static bool should_task_stop(void) {
     if (s_driver.event_group != NULL) {
         EventBits_t bits = xEventGroupGetBits(s_driver.event_group);
         if ((bits & LD2450_EVENT_STOP_REQUEST) != 0) {
             return true;
         }
     }
     return false;
 }
 
 esp_err_t ld2450_init(const ld2450_config_t *config)
 {
     // Check if already initialized
     if (s_driver.initialized) {
         ESP_LOGW(TAG, "Driver already initialized");
         return ESP_ERR_INVALID_STATE;
     }
     
     ESP_RETURN_ON_FALSE(config != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid config");
     
     // Create event group for task synchronization
     s_driver.event_group = xEventGroupCreate();
     ESP_RETURN_ON_FALSE(s_driver.event_group != NULL, ESP_ERR_NO_MEM, TAG, 
                         "Failed to create event group");
     
     // Clear any existing bits
     xEventGroupClearBits(s_driver.event_group, 0xFF);
     
     // Create mutex for UART access
     s_driver.uart_mutex = xSemaphoreCreateMutex();
     ESP_RETURN_ON_FALSE(s_driver.uart_mutex != NULL, ESP_ERR_NO_MEM, TAG, "Failed to create mutex");
     
     // Create data queue
     s_driver.data_queue = xQueueCreate(5, sizeof(ld2450_frame_data_t));
     ESP_RETURN_ON_FALSE(s_driver.data_queue != NULL, ESP_ERR_NO_MEM, TAG, "Failed to create data queue");
     
     // Initialize UART
     uart_config_t uart_config = {
         .baud_rate = ld2450_baud_to_value(config->baud_rate),
         .data_bits = UART_DATA_8_BITS,
         .parity    = UART_PARITY_DISABLE,
         .stop_bits = UART_STOP_BITS_1,
         .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
         .source_clk = UART_SCLK_DEFAULT,
     };
     
     ESP_RETURN_ON_ERROR(
         uart_driver_install(config->uart_port, config->rx_buffer_size, config->tx_buffer_size, 0, NULL, 0),
         TAG, "Failed to install UART driver");
         
     ESP_RETURN_ON_ERROR(
         uart_param_config(config->uart_port, &uart_config),
         TAG, "Failed to configure UART");
         
     ESP_RETURN_ON_ERROR(
         uart_set_pin(config->uart_port, config->tx_pin, config->rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE),
         TAG, "Failed to set UART pins");
     
     // Store configuration
     s_driver.uart_port = config->uart_port;
     s_driver.event_loop_enabled = config->event_loop_enabled;
     
     // Create receiver task
     BaseType_t task_created = xTaskCreate(
         ld2450_receiver_task,
         "ld2450_rx",
         config->task_stack_size,
         NULL,
         config->task_priority,
         &s_driver.receiver_task
     );
     
     if (task_created != pdTRUE) {
         // Clean up on failure
         if (s_driver.event_group != NULL) {
             vEventGroupDelete(s_driver.event_group);
             s_driver.event_group = NULL;
         }
         uart_driver_delete(config->uart_port);
         vSemaphoreDelete(s_driver.uart_mutex);
         vQueueDelete(s_driver.data_queue);
         s_driver.uart_mutex = NULL;
         s_driver.data_queue = NULL;
         return ESP_ERR_NO_MEM;
     }
     
     // Mark as initialized
     s_driver.initialized = true;
     ESP_LOGI(TAG, "Driver initialized on UART%d (TX:%d, RX:%d, Baud:%u)",
              config->uart_port, config->tx_pin, config->rx_pin, 
              ld2450_baud_to_value(config->baud_rate));
     
     return ESP_OK;
 }
 
 esp_err_t ld2450_deinit(void)
 {
     if (!s_driver.initialized) {
         ESP_LOGW(TAG, "Driver not initialized");
         return ESP_ERR_LD2450_NOT_INITIALIZED;
     }
     
     ESP_LOGI(TAG, "Deinitializing driver");
     
     // Step 1: Request task to stop via event group
     if (s_driver.receiver_task != NULL && s_driver.event_group != NULL) {
         // Set stop request bit
         xEventGroupSetBits(s_driver.event_group, LD2450_EVENT_STOP_REQUEST);
         
         // Send notification to wake up task if it's blocked on UART read
         xTaskNotify(s_driver.receiver_task, LD2450_NOTIFICATION_STOP, eSetValueWithOverwrite);
         
         // Step 2: Wait for task to exit with timeout (500ms)
         EventBits_t bits = xEventGroupWaitBits(
             s_driver.event_group,          // Event group
             LD2450_EVENT_TASK_STOPPED,     // Bits to wait for
             pdFALSE,                       // Don't clear on exit
             pdTRUE,                        // Wait for all bits
             pdMS_TO_TICKS(500)             // Timeout
         );
         
         // Check if task signaled it stopped
         if ((bits & LD2450_EVENT_TASK_STOPPED) == 0) {
             // Task didn't exit cleanly within timeout
             ESP_LOGW(TAG, "Receiver task did not exit cleanly, deleting forcefully");
             vTaskDelete(s_driver.receiver_task);
         } else {
             ESP_LOGI(TAG, "Receiver task exited cleanly");
         }
         
         s_driver.receiver_task = NULL;
     }
     
     // Step 3: Flush and delete UART driver (safe now that task is stopped)
     if (s_driver.initialized) {
         uart_flush(s_driver.uart_port);
         esp_err_t err = uart_driver_delete(s_driver.uart_port);
         if (err != ESP_OK) {
             ESP_LOGE(TAG, "Error deleting UART driver: %s", esp_err_to_name(err));
             // Continue with deinitialization anyway
         }
     }
     
     // Step 4: Delete synchronization primitives
     if (s_driver.uart_mutex != NULL) {
         vSemaphoreDelete(s_driver.uart_mutex);
         s_driver.uart_mutex = NULL;
     }
     
     if (s_driver.data_queue != NULL) {
         vQueueDelete(s_driver.data_queue);
         s_driver.data_queue = NULL;
     }
     
     if (s_driver.event_group != NULL) {
         vEventGroupDelete(s_driver.event_group);
         s_driver.event_group = NULL;
     }
     
     // Step 5: Reset driver state
     s_driver.initialized = false;
     s_driver.config_mode = false;
     s_driver.data_callback = NULL;
     s_driver.callback_user_data = NULL;
     s_driver.uart_port = UART_NUM_MAX;
     
     ESP_LOGI(TAG, "Driver deinitialized successfully");
     return ESP_OK;
 }
 
 /**
  * @brief Enable configuration mode
  */
  esp_err_t ld2450_enable_config(void)
  {
      if (!s_driver.initialized) {
          return ESP_ERR_LD2450_NOT_INITIALIZED;
      }
      
      ESP_LOGI(TAG, "Enabling configuration mode");
      
      uint16_t cmd_value = 0x0001;
      esp_err_t err = ld2450_send_command(LD2450_CMD_ENABLE_CONFIG, &cmd_value, sizeof(cmd_value));
      if (err != ESP_OK) {
          ESP_LOGE(TAG, "Failed to send enable config command: %s", esp_err_to_name(err));
          return err;
      }
      
      // Response: [status(2)][protocol_version(2)][buffer_size(2)]
      uint8_t resp_buffer[6];
      size_t resp_len = sizeof(resp_buffer);
      
      err = ld2450_receive_response(LD2450_CMD_ENABLE_CONFIG, resp_buffer, &resp_len, 1000);
      if (err != ESP_OK) {
          ESP_LOGE(TAG, "Failed to receive enable config response: %s", esp_err_to_name(err));
          return err;
      }
      
      // Check status
      uint16_t status = resp_buffer[0] | (resp_buffer[1] << 8);
      if (status != LD2450_ACK_SUCCESS) {
          ESP_LOGE(TAG, "Enable config command failed with status: %d", status);
          return ESP_ERR_LD2450_COMMAND_FAILED;
      }
      
      // Extract protocol version and buffer size
      uint16_t protocol_version = resp_buffer[2] | (resp_buffer[3] << 8);
      uint16_t buffer_size = resp_buffer[4] | (resp_buffer[5] << 8);
      ESP_LOGI(TAG, "Configuration mode enabled, protocol version: 0x%04x, buffer size: %u", 
               protocol_version, buffer_size);
      
      s_driver.config_mode = true;
      return ESP_OK;
  }
  
  /**
   * @brief End configuration mode
   */
  esp_err_t ld2450_end_config(void)
  {
      if (!s_driver.initialized) {
          return ESP_ERR_LD2450_NOT_INITIALIZED;
      }
      
      if (!s_driver.config_mode) {
          ESP_LOGW(TAG, "Configuration mode not enabled");
          return ESP_OK;
      }
      
      ESP_LOGI(TAG, "Ending configuration mode");
      
      esp_err_t err = ld2450_send_command(LD2450_CMD_END_CONFIG, NULL, 0);
      if (err != ESP_OK) {
          ESP_LOGE(TAG, "Failed to send end config command: %s", esp_err_to_name(err));
          return err;
      }
      
      // Response: [status(2)]
      uint8_t resp_buffer[2];
      size_t resp_len = sizeof(resp_buffer);
      
      err = ld2450_receive_response(LD2450_CMD_END_CONFIG, resp_buffer, &resp_len, 1000);
      if (err != ESP_OK) {
          ESP_LOGE(TAG, "Failed to receive end config response: %s", esp_err_to_name(err));
          return err;
      }
      
      // Check status
      uint16_t status = resp_buffer[0] | (resp_buffer[1] << 8);
      if (status != LD2450_ACK_SUCCESS) {
          ESP_LOGE(TAG, "End config command failed with status: %d", status);
          return ESP_ERR_LD2450_COMMAND_FAILED;
      }
      
      s_driver.config_mode = false;
      ESP_LOGI(TAG, "Configuration mode ended");
      return ESP_OK;
  }
  
  /**
   * @brief Set target tracking mode
   */
  esp_err_t ld2450_set_tracking_mode(ld2450_tracking_mode_t mode)
  {
      if (!s_driver.initialized) {
          return ESP_ERR_LD2450_NOT_INITIALIZED;
      }
      
      if (!s_driver.config_mode) {
          ESP_LOGE(TAG, "Configuration mode not enabled");
          return ESP_ERR_LD2450_CONFIG_LOCKED;
      }
      
      ld2450_cmd_t cmd = (mode == LD2450_MODE_SINGLE_TARGET) ? 
                         LD2450_CMD_SINGLE_TARGET : LD2450_CMD_MULTI_TARGET;
      
      ESP_LOGI(TAG, "Setting tracking mode to %s", 
               (mode == LD2450_MODE_SINGLE_TARGET) ? "single-target" : "multi-target");
      
      esp_err_t err = ld2450_send_command(cmd, NULL, 0);
      if (err != ESP_OK) {
          ESP_LOGE(TAG, "Failed to send tracking mode command: %s", esp_err_to_name(err));
          return err;
      }
      
      // Response: [status(2)]
      uint8_t resp_buffer[2];
      size_t resp_len = sizeof(resp_buffer);
      
      err = ld2450_receive_response(cmd, resp_buffer, &resp_len, 1000);
      if (err != ESP_OK) {
          ESP_LOGE(TAG, "Failed to receive tracking mode response: %s", esp_err_to_name(err));
          return err;
      }
      
      // Check status
      uint16_t status = resp_buffer[0] | (resp_buffer[1] << 8);
      if (status != LD2450_ACK_SUCCESS) {
          ESP_LOGE(TAG, "Set tracking mode command failed with status: %d", status);
          return ESP_ERR_LD2450_COMMAND_FAILED;
      }
      
      ESP_LOGI(TAG, "Tracking mode set successfully");
      return ESP_OK;
  }
  
  /**
   * @brief Get current tracking mode
   */
  esp_err_t ld2450_get_tracking_mode(ld2450_tracking_mode_t *mode)
  {
      if (!s_driver.initialized) {
          return ESP_ERR_LD2450_NOT_INITIALIZED;
      }
      
      if (!s_driver.config_mode) {
          ESP_LOGE(TAG, "Configuration mode not enabled");
          return ESP_ERR_LD2450_CONFIG_LOCKED;
      }
      
      if (mode == NULL) {
          return ESP_ERR_INVALID_ARG;
      }
      
      ESP_LOGI(TAG, "Querying tracking mode");
      
      esp_err_t err = ld2450_send_command(LD2450_CMD_QUERY_TRACK_MODE, NULL, 0);
      if (err != ESP_OK) {
          ESP_LOGE(TAG, "Failed to send query tracking mode command: %s", esp_err_to_name(err));
          return err;
      }
      
      // Response: [status(2)][mode(2)]
      uint8_t resp_buffer[4];
      size_t resp_len = sizeof(resp_buffer);
      
      err = ld2450_receive_response(LD2450_CMD_QUERY_TRACK_MODE, resp_buffer, &resp_len, 1000);
      if (err != ESP_OK) {
          ESP_LOGE(TAG, "Failed to receive tracking mode query response: %s", esp_err_to_name(err));
          return err;
      }
      
      // Check status
      uint16_t status = resp_buffer[0] | (resp_buffer[1] << 8);
      if (status != LD2450_ACK_SUCCESS) {
          ESP_LOGE(TAG, "Query tracking mode command failed with status: %d", status);
          return ESP_ERR_LD2450_COMMAND_FAILED;
      }
      
      // Extract mode
      uint16_t track_mode = resp_buffer[2] | (resp_buffer[3] << 8);
      *mode = ld2450_protocol_to_tracking_mode(track_mode);
      
      ESP_LOGI(TAG, "Current tracking mode: %s", 
               (*mode == LD2450_MODE_SINGLE_TARGET) ? "single-target" : "multi-target");
      
      return ESP_OK;
  }
  
  /**
   * @brief Read firmware version from radar
   */
   esp_err_t ld2450_get_firmware_version(ld2450_firmware_version_t *version)
   {
       if (!s_driver.initialized) {
           return ESP_ERR_LD2450_NOT_INITIALIZED;
       }
       
       if (!s_driver.config_mode) {
           ESP_LOGE(TAG, "Configuration mode not enabled");
           return ESP_ERR_LD2450_CONFIG_LOCKED;
       }
       
       if (version == NULL) {
           return ESP_ERR_INVALID_ARG;
       }
       
       ESP_LOGI(TAG, "Reading firmware version");
       
       esp_err_t err = ld2450_send_command(LD2450_CMD_READ_FW_VERSION, NULL, 0);
       if (err != ESP_OK) {
           ESP_LOGE(TAG, "Failed to send read firmware version command: %s", esp_err_to_name(err));
           return err;
       }
       
       // Response: [status(2)][fw_type(2)][main_version(2)][sub_version(4)]
       uint8_t resp_buffer[10];
       size_t resp_len = sizeof(resp_buffer);
       
       err = ld2450_receive_response(LD2450_CMD_READ_FW_VERSION, resp_buffer, &resp_len, 1000);
       if (err != ESP_OK) {
           ESP_LOGE(TAG, "Failed to receive firmware version response: %s", esp_err_to_name(err));
           return err;
       }
       
       // Check status
       uint16_t status = resp_buffer[0] | (resp_buffer[1] << 8);
       if (status != LD2450_ACK_SUCCESS) {
           ESP_LOGE(TAG, "Read firmware version command failed with status: %d", status);
           return ESP_ERR_LD2450_COMMAND_FAILED;
       }
       
       // Use the protocol structure to process raw data
       ld2450_protocol_firmware_version_t proto_version;
       
       // Extract protocol data
       proto_version.fw_type = resp_buffer[2] | (resp_buffer[3] << 8);
       proto_version.major_version = resp_buffer[4] | (resp_buffer[5] << 8);
       proto_version.sub_version = resp_buffer[6] | (resp_buffer[7] << 8) | 
                                   (resp_buffer[8] << 16) | (resp_buffer[9] << 24);
       
       // Convert protocol data to public API structure
       version->main_version = proto_version.major_version;
       version->sub_version = proto_version.sub_version;
       
       // Format version string (e.g., "V1.02.22062416")
       snprintf(version->version_str, sizeof(version->version_str), "V%d.%02lX.%08lX",
                (version->main_version >> 8) & 0xFF,
                version->main_version & 0xFF,
                version->sub_version);
       
       ESP_LOGI(TAG, "Firmware version: %s", version->version_str);
       
       return ESP_OK;
   }
  
  /**
   * @brief Set serial port baud rate
   */
  esp_err_t ld2450_set_baud_rate(ld2450_baud_rate_t baud_rate)
  {
      if (!s_driver.initialized) {
          return ESP_ERR_LD2450_NOT_INITIALIZED;
      }
      
      if (!s_driver.config_mode) {
          ESP_LOGE(TAG, "Configuration mode not enabled");
          return ESP_ERR_LD2450_CONFIG_LOCKED;
      }
      
      uint16_t baud_index = ld2450_baud_to_index(baud_rate);
      ESP_LOGI(TAG, "Setting baud rate to %u (index: 0x%04x)", 
               ld2450_baud_to_value(baud_rate), baud_index);
      
      esp_err_t err = ld2450_send_command(LD2450_CMD_SET_BAUD_RATE, &baud_index, sizeof(baud_index));
      if (err != ESP_OK) {
          ESP_LOGE(TAG, "Failed to send set baud rate command: %s", esp_err_to_name(err));
          return err;
      }
      
      // Response: [status(2)]
      uint8_t resp_buffer[2];
      size_t resp_len = sizeof(resp_buffer);
      
      err = ld2450_receive_response(LD2450_CMD_SET_BAUD_RATE, resp_buffer, &resp_len, 1000);
      if (err != ESP_OK) {
          ESP_LOGE(TAG, "Failed to receive set baud rate response: %s", esp_err_to_name(err));
          return err;
      }
      
      // Check status
      uint16_t status = resp_buffer[0] | (resp_buffer[1] << 8);
      if (status != LD2450_ACK_SUCCESS) {
          ESP_LOGE(TAG, "Set baud rate command failed with status: %d", status);
          return ESP_ERR_LD2450_COMMAND_FAILED;
      }
      
      ESP_LOGI(TAG, "Baud rate set successfully (will take effect after restart)");
      return ESP_OK;
  }
  
  /**
   * @brief Helper function to send a command to the radar
   */
   static esp_err_t ld2450_send_command(ld2450_cmd_t cmd, const void *data, size_t data_len)
   {
       if (!s_driver.initialized) {
           return ESP_ERR_LD2450_NOT_INITIALIZED;
       }
       
       if (data_len > 0 && data == NULL) {
           return ESP_ERR_INVALID_ARG;
       }
       
       // Calculate total command size
       size_t cmd_data_len = 2 + data_len; // Command word (2 bytes) + data
       size_t total_len = LD2450_HEADER_LEN + 2 + cmd_data_len + LD2450_TRAILER_LEN;
       
       if (total_len > LD2450_MAX_CMD_LEN) {
           ESP_LOGE(TAG, "Command too large (%u bytes)", total_len);
           return ESP_ERR_INVALID_ARG;
       }
       
       uint8_t cmd_buffer[LD2450_MAX_CMD_LEN];
       size_t pos = 0;
       
       // Frame header
       static const uint8_t header[] = LD2450_FRAME_HEADER;
       memcpy(&cmd_buffer[pos], header, LD2450_HEADER_LEN);
       pos += LD2450_HEADER_LEN;
       
       // Data length (2 bytes, little-endian)
       cmd_buffer[pos++] = cmd_data_len & 0xFF;
       cmd_buffer[pos++] = (cmd_data_len >> 8) & 0xFF;
       
       // Command word (2 bytes, little-endian)
       cmd_buffer[pos++] = cmd & 0xFF;
       cmd_buffer[pos++] = (cmd >> 8) & 0xFF;
       
       // Command data (if any)
       if (data_len > 0) {
          memcpy(&cmd_buffer[pos], data, data_len);
          pos += data_len;
      }
      
      // Frame trailer
      static const uint8_t trailer[] = LD2450_FRAME_TRAILER;
      memcpy(&cmd_buffer[pos], trailer, LD2450_TRAILER_LEN);
      pos += LD2450_TRAILER_LEN;
      
      // Check for shutdown request before accessing resources
      if (should_task_stop()) {
          ESP_LOGW(TAG, "Send command aborted due to shutdown request");
          return ESP_ERR_INVALID_STATE;
      }
      
      // Take UART mutex for thread safety
      if (!safe_mutex_take(100)) {
          ESP_LOGE(TAG, "Failed to acquire UART mutex or shutdown in progress");
          return ESP_ERR_TIMEOUT;
      }
      
      // Flush input buffer to discard any pending data
      uart_flush_input(s_driver.uart_port);
      
      // Send command
      int written = uart_write_bytes(s_driver.uart_port, cmd_buffer, pos);
      safe_mutex_give();
      
      if (written != pos) {
          ESP_LOGE(TAG, "UART write error: %d/%u bytes written", written, pos);
          return ESP_ERR_LD2450_COMM_ERROR;
      }
      
      // Print sent command for debugging
      ESP_LOGD(TAG, "Command sent: cmd=0x%04x, len=%u", cmd, pos);
      ESP_LOG_BUFFER_HEX_LEVEL(TAG, cmd_buffer, pos, ESP_LOG_DEBUG);
      
      return ESP_OK;
  }
  
  /**
   * @brief Helper function to receive a command response from the radar
   */
  static esp_err_t ld2450_receive_response(ld2450_cmd_t cmd, void *response, size_t *response_len, uint32_t timeout_ms)
  {
      if (!s_driver.initialized) {
          return ESP_ERR_LD2450_NOT_INITIALIZED;
      }
      
      if (response == NULL || response_len == NULL) {
          return ESP_ERR_INVALID_ARG;
      }
      
      // Check for shutdown request before accessing resources
      if (should_task_stop()) {
          ESP_LOGW(TAG, "Receive response aborted due to shutdown request");
          return ESP_ERR_INVALID_STATE;
      }
      
      uint8_t rx_buffer[LD2450_MAX_RESP_LEN];
      size_t expected_resp_len = *response_len;
      
      // Take UART mutex for thread safety
      if (!safe_mutex_take(100)) {
          ESP_LOGE(TAG, "Failed to acquire UART mutex or shutdown in progress");
          return ESP_ERR_TIMEOUT;
      }
      
      // Define response parsing state variables
      ld2450_rx_state_t state = LD2450_STATE_WAIT_HEADER;
      uint16_t data_length = 0;
      size_t bytes_read = 0;
      size_t data_bytes_read = 0;
      uint64_t start_time = esp_timer_get_time() / 1000;  // Convert to ms
      
      // Buffer to store the expected header pattern
      static const uint8_t expected_header[] = LD2450_FRAME_HEADER;
      static const uint8_t expected_trailer[] = LD2450_FRAME_TRAILER;
      uint8_t header_match_pos = 0;
      uint8_t trailer_match_pos = 0;
      
      // Response data buffer
      uint8_t resp_data[LD2450_MAX_RESP_LEN];
      
      // Read and parse response
      while ((esp_timer_get_time() / 1000) - start_time < timeout_ms) {
          // Check for shutdown request during long operations
          if (should_task_stop()) {
              safe_mutex_give();
              ESP_LOGW(TAG, "Receive response aborted due to shutdown request");
              return ESP_ERR_INVALID_STATE;
          }
          
          size_t available_bytes;
          ESP_ERROR_CHECK(uart_get_buffered_data_len(s_driver.uart_port, &available_bytes));
          
          if (available_bytes == 0) {
              // Temporarily release mutex during waiting
              safe_mutex_give();
              vTaskDelay(pdMS_TO_TICKS(10));  // Short delay to avoid busy waiting
              
              // Re-acquire mutex
              if (!safe_mutex_take(10)) {
                  ESP_LOGE(TAG, "Failed to reacquire UART mutex or shutdown in progress");
                  return ESP_ERR_TIMEOUT;
              }
              continue;
          }
          
          // Read one byte at a time to handle state machine
          uint8_t byte;
          int read_bytes = uart_read_bytes(s_driver.uart_port, &byte, 1, pdMS_TO_TICKS(10));
          if (read_bytes != 1) {
              continue;
          }
          
          // Process byte according to current state
          switch (state) {
              case LD2450_STATE_WAIT_HEADER:
                  // Match header byte by byte
                  if (byte == expected_header[header_match_pos]) {
                      header_match_pos++;
                      if (header_match_pos == LD2450_HEADER_LEN) {
                          // Header fully matched, move to length field
                          state = LD2450_STATE_READ_LENGTH;
                          bytes_read = 0;
                      }
                  } else {
                      // Mismatch, reset header matching
                      header_match_pos = 0;
                  }
                  break;
                  
              case LD2450_STATE_READ_LENGTH:
                  // Read 2-byte data length field
                  rx_buffer[bytes_read++] = byte;
                  if (bytes_read == 2) {
                      // Extract data length
                      data_length = rx_buffer[0] | (rx_buffer[1] << 8);
                      bytes_read = 0;
                      data_bytes_read = 0;
                      state = LD2450_STATE_READ_DATA;
                      
                      // Check for reasonable data length
                      if (data_length > LD2450_MAX_RESP_LEN - LD2450_HEADER_LEN - LD2450_TRAILER_LEN) {
                          ESP_LOGE(TAG, "Response data length too large: %u", data_length);
                          safe_mutex_give();
                          return ESP_ERR_LD2450_INVALID_RESPONSE;
                      }
                  }
                  break;
                  
              case LD2450_STATE_READ_DATA:
                  // Read data bytes
                  if (data_bytes_read < data_length) {
                      resp_data[data_bytes_read++] = byte;
                  }
                  
                  if (data_bytes_read == data_length) {
                      state = LD2450_STATE_WAIT_TRAILER;
                      trailer_match_pos = 0;
                  }
                  break;
                  
              case LD2450_STATE_WAIT_TRAILER:
                  // Match trailer byte by byte
                  if (byte == expected_trailer[trailer_match_pos]) {
                      trailer_match_pos++;
                      if (trailer_match_pos == LD2450_TRAILER_LEN) {
                          // Trailer fully matched, response is complete
                          state = LD2450_STATE_COMPLETE;
                      }
                  } else {
                      // Mismatch, reset trailer matching
                      trailer_match_pos = 0;
                  }
                  break;
                  
              case LD2450_STATE_COMPLETE:
                  // Should not receive more bytes after complete
                  ESP_LOGW(TAG, "Received unexpected byte after response complete: 0x%02x", byte);
                  break;
          }
          
          if (state == LD2450_STATE_COMPLETE) {
              break;
          }
      }
      
      // Release mutex
      safe_mutex_give();
      
      // Check for timeout
      if (state != LD2450_STATE_COMPLETE) {
          ESP_LOGE(TAG, "Timeout waiting for response, state=%d", state);
          return ESP_ERR_LD2450_TIMEOUT;
      }
      
      // Validate response
      if (data_length < 4) {  // Minimum: CMD(2) + ACK Status(2)
          ESP_LOGE(TAG, "Response too short: %u bytes", data_length);
          return ESP_ERR_LD2450_INVALID_RESPONSE;
      }
      
      // Check if response contains the expected command (first 2 bytes of data)
      uint16_t resp_cmd = resp_data[0] | (resp_data[1] << 8);
      if (resp_cmd != (cmd | 0x0100)) {  // Response command is original command | 0x0100
          ESP_LOGE(TAG, "Response command mismatch: expected 0x%04x, got 0x%04x", 
                   (cmd | 0x0100), resp_cmd);
          return ESP_ERR_LD2450_INVALID_RESPONSE;
      }
      
      // Extract response data (after the command bytes)
      size_t actual_resp_len = data_length - 2;  // Subtract command word length
      if (actual_resp_len > expected_resp_len) {
          ESP_LOGW(TAG, "Response data truncated: %u > %u bytes", 
                   actual_resp_len, expected_resp_len);
          actual_resp_len = expected_resp_len;
      }
      
      // Copy response data to output buffer
      memcpy(response, &resp_data[2], actual_resp_len);
      *response_len = actual_resp_len;
      
      // Print received response for debugging
      ESP_LOGD(TAG, "Response received: cmd=0x%04x, len=%u", resp_cmd, data_length);
      ESP_LOG_BUFFER_HEX_LEVEL(TAG, resp_data, data_length, ESP_LOG_DEBUG);
      
      return ESP_OK;
  }
  
  /**
   * @brief Restore factory settings
   */
   esp_err_t ld2450_restore_factory_settings(void)
   {
       if (!s_driver.initialized) {
           return ESP_ERR_LD2450_NOT_INITIALIZED;
       }
       
       if (!s_driver.config_mode) {
           ESP_LOGE(TAG, "Configuration mode not enabled");
           return ESP_ERR_LD2450_CONFIG_LOCKED;
       }
       
       ESP_LOGI(TAG, "Restoring factory settings");
       
       esp_err_t err = ld2450_send_command(LD2450_CMD_RESTORE_FACTORY, NULL, 0);
       if (err != ESP_OK) {
           ESP_LOGE(TAG, "Failed to send restore factory settings command: %s", esp_err_to_name(err));
           return err;
       }
       
       // Response: [status(2)]
       uint8_t resp_buffer[2];
       size_t resp_len = sizeof(resp_buffer);
       
       err = ld2450_receive_response(LD2450_CMD_RESTORE_FACTORY, resp_buffer, &resp_len, 1000);
       if (err != ESP_OK) {
           ESP_LOGE(TAG, "Failed to receive restore factory settings response: %s", esp_err_to_name(err));
           return err;
       }
       
       // Check status
       uint16_t status = resp_buffer[0] | (resp_buffer[1] << 8);
       if (status != LD2450_ACK_SUCCESS) {
           ESP_LOGE(TAG, "Restore factory settings command failed with status: %d", status);
           return ESP_ERR_LD2450_COMMAND_FAILED;
       }
       
       ESP_LOGI(TAG, "Factory settings restored successfully (will take effect after restart)");
       return ESP_OK;
   }
   
   /**
    * @brief Restart the radar module
    */
   esp_err_t ld2450_restart(void)
   {
       if (!s_driver.initialized) {
           return ESP_ERR_LD2450_NOT_INITIALIZED;
       }
       
       if (!s_driver.config_mode) {
           ESP_LOGE(TAG, "Configuration mode not enabled");
           return ESP_ERR_LD2450_CONFIG_LOCKED;
       }
       
       ESP_LOGI(TAG, "Restarting radar module");
       
       esp_err_t err = ld2450_send_command(LD2450_CMD_RESTART_MODULE, NULL, 0);
       if (err != ESP_OK) {
           ESP_LOGE(TAG, "Failed to send restart command: %s", esp_err_to_name(err));
           return err;
       }
       
       // Response: [status(2)]
       uint8_t resp_buffer[2];
       size_t resp_len = sizeof(resp_buffer);
       
       err = ld2450_receive_response(LD2450_CMD_RESTART_MODULE, resp_buffer, &resp_len, 1000);
       if (err != ESP_OK) {
           ESP_LOGE(TAG, "Failed to receive restart response: %s", esp_err_to_name(err));
           return err;
       }
       
       // Check status
       uint16_t status = resp_buffer[0] | (resp_buffer[1] << 8);
       if (status != LD2450_ACK_SUCCESS) {
           ESP_LOGE(TAG, "Restart command failed with status: %d", status);
           return ESP_ERR_LD2450_COMMAND_FAILED;
       }
       
       ESP_LOGI(TAG, "Restart command sent successfully");
     
       // Wait for the module to restart
       vTaskDelay(pdMS_TO_TICKS(2000));
       
       // Reset driver state since configuration mode is exited after restart
       s_driver.config_mode = false;
       
       return ESP_OK;
   }
   
   /**
    * @brief Configure Bluetooth settings
    */
   esp_err_t ld2450_set_bluetooth(ld2450_bt_mode_t mode)
   {
       if (!s_driver.initialized) {
           return ESP_ERR_LD2450_NOT_INITIALIZED;
       }
       
       if (!s_driver.config_mode) {
           ESP_LOGE(TAG, "Configuration mode not enabled");
           return ESP_ERR_LD2450_CONFIG_LOCKED;
       }
       
       uint16_t bt_value = (mode == LD2450_BT_ON) ? LD2450_BT_ENABLE : LD2450_BT_DISABLE;
       ESP_LOGI(TAG, "Setting Bluetooth to %s", (mode == LD2450_BT_ON) ? "ON" : "OFF");
       
       esp_err_t err = ld2450_send_command(LD2450_CMD_BLUETOOTH_SETTINGS, &bt_value, sizeof(bt_value));
       if (err != ESP_OK) {
           ESP_LOGE(TAG, "Failed to send Bluetooth settings command: %s", esp_err_to_name(err));
           return err;
       }
       
       // Response: [status(2)]
       uint8_t resp_buffer[2];
       size_t resp_len = sizeof(resp_buffer);
       
       err = ld2450_receive_response(LD2450_CMD_BLUETOOTH_SETTINGS, resp_buffer, &resp_len, 1000);
       if (err != ESP_OK) {
           ESP_LOGE(TAG, "Failed to receive Bluetooth settings response: %s", esp_err_to_name(err));
           return err;
       }
       
       // Check status
       uint16_t status = resp_buffer[0] | (resp_buffer[1] << 8);
       if (status != LD2450_ACK_SUCCESS) {
           ESP_LOGE(TAG, "Set Bluetooth settings command failed with status: %d", status);
           return ESP_ERR_LD2450_COMMAND_FAILED;
       }
       
       ESP_LOGI(TAG, "Bluetooth settings changed successfully (will take effect after restart)");
       return ESP_OK;
   }
   
   /**
    * @brief Get radar MAC address
    */
   esp_err_t ld2450_get_mac_address(uint8_t mac_addr[6])
   {
       if (!s_driver.initialized) {
           return ESP_ERR_LD2450_NOT_INITIALIZED;
       }
       
       if (!s_driver.config_mode) {
           ESP_LOGE(TAG, "Configuration mode not enabled");
           return ESP_ERR_LD2450_CONFIG_LOCKED;
       }
       
       if (mac_addr == NULL) {
           return ESP_ERR_INVALID_ARG;
       }
       
       // Command value is 0x0001 for this command
       uint16_t cmd_value = 0x0001;
       ESP_LOGI(TAG, "Reading MAC address");
       
       esp_err_t err = ld2450_send_command(LD2450_CMD_GET_MAC_ADDRESS, &cmd_value, sizeof(cmd_value));
       if (err != ESP_OK) {
           ESP_LOGE(TAG, "Failed to send get MAC address command: %s", esp_err_to_name(err));
           return err;
       }
       
       // Response: [status(2)][fixed_type(1)][mac_addr(6)]
       uint8_t resp_buffer[8];
       size_t resp_len = sizeof(resp_buffer);
       
       err = ld2450_receive_response(LD2450_CMD_GET_MAC_ADDRESS, resp_buffer, &resp_len, 1000);
       if (err != ESP_OK) {
           ESP_LOGE(TAG, "Failed to receive MAC address response: %s", esp_err_to_name(err));
           return err;
       }
       
       // Check status
       uint16_t status = resp_buffer[0] | (resp_buffer[1] << 8);
       if (status != LD2450_ACK_SUCCESS) {
           ESP_LOGE(TAG, "Get MAC address command failed with status: %d", status);
           return ESP_ERR_LD2450_COMMAND_FAILED;
       }
       
       // Extract MAC address (skip first byte which is the fixed type)
       memcpy(mac_addr, &resp_buffer[2], 6);
       
       ESP_LOGI(TAG, "MAC address: %02X:%02X:%02X:%02X:%02X:%02X",
                mac_addr[0], mac_addr[1], mac_addr[2],
                mac_addr[3], mac_addr[4], mac_addr[5]);
       
       return ESP_OK;
   }
   
   /**
    * @brief Configure region filtering
    */
   esp_err_t ld2450_set_region_filter(const ld2450_region_filter_t *filter)
   {
       if (!s_driver.initialized) {
           return ESP_ERR_LD2450_NOT_INITIALIZED;
       }
       
       if (!s_driver.config_mode) {
           ESP_LOGE(TAG, "Configuration mode not enabled");
           return ESP_ERR_LD2450_CONFIG_LOCKED;
       }
       
       if (filter == NULL) {
           return ESP_ERR_INVALID_ARG;
       }
       
       ESP_LOGI(TAG, "Setting region filter mode to %d", filter->mode);
       
       // Prepare data buffer for region filter command
       uint8_t filter_data[26];  // 2 bytes for type + 24 bytes for region coordinates
       size_t pos = 0;
       
       // Region filter type
       uint16_t filter_type = ld2450_filter_mode_to_protocol(filter->mode);
       filter_data[pos++] = filter_type & 0xFF;
       filter_data[pos++] = (filter_type >> 8) & 0xFF;
       
       // Region 1 coordinates
       filter_data[pos++] = filter->region1_p1.x & 0xFF;
       filter_data[pos++] = (filter->region1_p1.x >> 8) & 0xFF;
       filter_data[pos++] = filter->region1_p1.y & 0xFF;
       filter_data[pos++] = (filter->region1_p1.y >> 8) & 0xFF;
       filter_data[pos++] = filter->region1_p2.x & 0xFF;
       filter_data[pos++] = (filter->region1_p2.x >> 8) & 0xFF;
       filter_data[pos++] = filter->region1_p2.y & 0xFF;
       filter_data[pos++] = (filter->region1_p2.y >> 8) & 0xFF;
       
       // Region 2 coordinates
       filter_data[pos++] = filter->region2_p1.x & 0xFF;
       filter_data[pos++] = (filter->region2_p1.x >> 8) & 0xFF;
       filter_data[pos++] = filter->region2_p1.y & 0xFF;
       filter_data[pos++] = (filter->region2_p1.y >> 8) & 0xFF;
       filter_data[pos++] = filter->region2_p2.x & 0xFF;
       filter_data[pos++] = (filter->region2_p2.x >> 8) & 0xFF;
       filter_data[pos++] = filter->region2_p2.y & 0xFF;
       filter_data[pos++] = (filter->region2_p2.y >> 8) & 0xFF;
       
       // Region 3 coordinates
       filter_data[pos++] = filter->region3_p1.x & 0xFF;
       filter_data[pos++] = (filter->region3_p1.x >> 8) & 0xFF;
       filter_data[pos++] = filter->region3_p1.y & 0xFF;
       filter_data[pos++] = (filter->region3_p1.y >> 8) & 0xFF;
       filter_data[pos++] = filter->region3_p2.x & 0xFF;
       filter_data[pos++] = (filter->region3_p2.x >> 8) & 0xFF;
       filter_data[pos++] = filter->region3_p2.y & 0xFF;
       filter_data[pos++] = (filter->region3_p2.y >> 8) & 0xFF;
       
       // Send command with filter configuration
       esp_err_t err = ld2450_send_command(LD2450_CMD_SET_REGION_FILTER, filter_data, sizeof(filter_data));
       if (err != ESP_OK) {
           ESP_LOGE(TAG, "Failed to send set region filter command: %s", esp_err_to_name(err));
           return err;
       }
       
       // Response: [status(2)]
       uint8_t resp_buffer[2];
       size_t resp_len = sizeof(resp_buffer);
       
       err = ld2450_receive_response(LD2450_CMD_SET_REGION_FILTER, resp_buffer, &resp_len, 1000);
       if (err != ESP_OK) {
           ESP_LOGE(TAG, "Failed to receive set region filter response: %s", esp_err_to_name(err));
           return err;
       }
       
       // Check status
       uint16_t status = resp_buffer[0] | (resp_buffer[1] << 8);
       if (status != LD2450_ACK_SUCCESS) {
           ESP_LOGE(TAG, "Set region filter command failed with status: %d", status);
           return ESP_ERR_LD2450_COMMAND_FAILED;
       }
       
       ESP_LOGI(TAG, "Region filter set successfully");
       return ESP_OK;
   }
   
   /**
    * @brief Get current region filter configuration
    */
   esp_err_t ld2450_get_region_filter(ld2450_region_filter_t *filter)
   {
       if (!s_driver.initialized) {
           return ESP_ERR_LD2450_NOT_INITIALIZED;
       }
       
       if (!s_driver.config_mode) {
           ESP_LOGE(TAG, "Configuration mode not enabled");
           return ESP_ERR_LD2450_CONFIG_LOCKED;
       }
       
       if (filter == NULL) {
           return ESP_ERR_INVALID_ARG;
       }
       
       ESP_LOGI(TAG, "Querying region filter configuration");
       
       esp_err_t err = ld2450_send_command(LD2450_CMD_QUERY_REGION_FILTER, NULL, 0);
       if (err != ESP_OK) {
           ESP_LOGE(TAG, "Failed to send query region filter command: %s", esp_err_to_name(err));
           return err;
       }
       
       // Response: [status(2)][filter_type(2)][region_coordinates(24)]
       uint8_t resp_buffer[28];
       size_t resp_len = sizeof(resp_buffer);
       
       err = ld2450_receive_response(LD2450_CMD_QUERY_REGION_FILTER, resp_buffer, &resp_len, 1000);
       if (err != ESP_OK) {
           ESP_LOGE(TAG, "Failed to receive query region filter response: %s", esp_err_to_name(err));
           return err;
       }
       
       // Check status
       uint16_t status = resp_buffer[0] | (resp_buffer[1] << 8);
       if (status != LD2450_ACK_SUCCESS) {
          ESP_LOGE(TAG, "Query region filter command failed with status: %d", status);
          return ESP_ERR_LD2450_COMMAND_FAILED;
      }
      
      // Extract filter type
      uint16_t filter_type = resp_buffer[2] | (resp_buffer[3] << 8);
      filter->mode = ld2450_protocol_to_filter_mode(filter_type);
      
      // Extract region coordinates
      size_t pos = 4;
      
      // Region 1
      filter->region1_p1.x = resp_buffer[pos] | (resp_buffer[pos+1] << 8);
      pos += 2;
      filter->region1_p1.y = resp_buffer[pos] | (resp_buffer[pos+1] << 8);
      pos += 2;
      filter->region1_p2.x = resp_buffer[pos] | (resp_buffer[pos+1] << 8);
      pos += 2;
      filter->region1_p2.y = resp_buffer[pos] | (resp_buffer[pos+1] << 8);
      pos += 2;
      
      // Region 2
      filter->region2_p1.x = resp_buffer[pos] | (resp_buffer[pos+1] << 8);
      pos += 2;
      filter->region2_p1.y = resp_buffer[pos] | (resp_buffer[pos+1] << 8);
      pos += 2;
      filter->region2_p2.x = resp_buffer[pos] | (resp_buffer[pos+1] << 8);
      pos += 2;
      filter->region2_p2.y = resp_buffer[pos] | (resp_buffer[pos+1] << 8);
      pos += 2;
      
      // Region 3
      filter->region3_p1.x = resp_buffer[pos] | (resp_buffer[pos+1] << 8);
      pos += 2;
      filter->region3_p1.y = resp_buffer[pos] | (resp_buffer[pos+1] << 8);
      pos += 2;
      filter->region3_p2.x = resp_buffer[pos] | (resp_buffer[pos+1] << 8);
      pos += 2;
      filter->region3_p2.y = resp_buffer[pos] | (resp_buffer[pos+1] << 8);
      
      ESP_LOGI(TAG, "Region filter: mode=%d", filter->mode);
      ESP_LOGI(TAG, "  Region 1: (%d,%d) to (%d,%d)", 
               filter->region1_p1.x, filter->region1_p1.y, 
               filter->region1_p2.x, filter->region1_p2.y);
      ESP_LOGI(TAG, "  Region 2: (%d,%d) to (%d,%d)", 
               filter->region2_p1.x, filter->region2_p1.y, 
               filter->region2_p2.x, filter->region2_p2.y);
      ESP_LOGI(TAG, "  Region 3: (%d,%d) to (%d,%d)", 
               filter->region3_p1.x, filter->region3_p1.y, 
               filter->region3_p2.x, filter->region3_p2.y);
      
      return ESP_OK;
  }
  
  /**
   * @brief Process a received data frame
   */
   static esp_err_t ld2450_process_data_frame(uint8_t *data, size_t len, ld2450_frame_data_t *frame)
   {
       if (data == NULL || frame == NULL) {
           return ESP_ERR_INVALID_ARG;
       }
       
       // Check for minimum frame length
       // Header(4) + Target1(8) + Target2(8) + Target3(8) + Trailer(2) = 30 bytes
       if (len < 30) {
           ESP_LOGW(TAG, "Data frame too short: %u bytes", len);
           return ESP_ERR_LD2450_INVALID_RESPONSE;
       }
       
       // Verify data frame header (AA FF 03 00)
       if (data[0] != 0xAA || data[1] != 0xFF || data[2] != 0x03 || data[3] != 0x00) {
           ESP_LOGW(TAG, "Invalid data frame header: %02X %02X %02X %02X", 
                    data[0], data[1], data[2], data[3]);
           return ESP_ERR_LD2450_INVALID_RESPONSE;
       }
       
       // Verify data frame trailer (55 CC)
       if (data[len-2] != 0x55 || data[len-1] != 0xCC) {
           ESP_LOGW(TAG, "Invalid data frame trailer: %02X %02X", 
                    data[len-2], data[len-1]);
           return ESP_ERR_LD2450_INVALID_RESPONSE;
       }
       
       // Process target data (8 bytes per target)
       size_t pos = 4; // Skip header
       bool at_least_one_target = false;
       
       // Process up to 3 targets
       for (int i = 0; i < 3; i++) {
           // Check if we have data for this target
           bool is_zero = true;
           for (int j = 0; j < 8; j++) {
               if (data[pos + j] != 0x00) {
                   is_zero = false;
                   break;
               }
           }
           
           if (is_zero) {
               // No target data (all zeros)
               frame->targets[i].present = false;
               frame->targets[i].x = 0;
               frame->targets[i].y = 0;
               frame->targets[i].speed = 0;
               frame->targets[i].resolution = 0;
           } else {
               // Extract target data
               frame->targets[i].present = true;
               at_least_one_target = true;
               
               // X coordinate (signed int16)
               int16_t x_raw = data[pos] | (data[pos+1] << 8);
               frame->targets[i].x = x_raw;
               pos += 2;
               
               // Y coordinate (signed int16)
               int16_t y_raw = data[pos] | (data[pos+1] << 8);
               frame->targets[i].y = y_raw;
               pos += 2;
               
               // Speed (signed int16, cm/s)
               int16_t speed_raw = data[pos] | (data[pos+1] << 8);
               frame->targets[i].speed = speed_raw;
               pos += 2;
               
               // Resolution (unsigned int16, mm)
               uint16_t resolution = data[pos] | (data[pos+1] << 8);
               frame->targets[i].resolution = resolution;
               pos += 2;
           }
       }
       
       // Set timestamp
       frame->timestamp = esp_timer_get_time() / 1000; // Convert us to ms
       
       return at_least_one_target ? ESP_OK : ESP_ERR_NOT_FOUND;
   }
   
   /**
    * @brief Get latest target data (non-blocking)
    */
   esp_err_t ld2450_get_data(ld2450_frame_data_t *data)
   {
       if (!s_driver.initialized) {
           return ESP_ERR_LD2450_NOT_INITIALIZED;
       }
       
       if (data == NULL) {
           return ESP_ERR_INVALID_ARG;
       }
       
       // Take mutex to ensure thread safety when accessing latest_data
       if (!safe_mutex_take(100)) {
           ESP_LOGW(TAG, "Failed to acquire mutex for get_data or shutdown in progress");
           return ESP_ERR_TIMEOUT;
       }
       
       // Copy latest frame data
       memcpy(data, &s_driver.latest_data, sizeof(ld2450_frame_data_t));
       
       // Release mutex
       safe_mutex_give();
       
       // Check if we have any active targets
       bool has_targets = false;
       for (int i = 0; i < 3; i++) {
           if (data->targets[i].present) {
               has_targets = true;
               break;
           }
       }
       
       return has_targets ? ESP_OK : ESP_ERR_NOT_FOUND;
   }
   
   /**
    * @brief Wait for new target data (blocking)
    */
   esp_err_t ld2450_wait_for_data(ld2450_frame_data_t *data, uint32_t timeout_ms)
   {
       if (!s_driver.initialized) {
           return ESP_ERR_LD2450_NOT_INITIALIZED;
       }
       
       if (data == NULL) {
           return ESP_ERR_INVALID_ARG;
       }
       
       // Wait for new data to arrive in the queue
       if (s_driver.data_queue == NULL) {
           return ESP_ERR_INVALID_STATE;
       }
       
       // Check for shutdown request before waiting
       if (should_task_stop()) {
           ESP_LOGW(TAG, "Wait for data aborted due to shutdown request");
           return ESP_ERR_INVALID_STATE;
       }
       
       if (xQueueReceive(s_driver.data_queue, data, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
           return ESP_ERR_TIMEOUT;
       }
       
       // Check if we have any active targets
       bool has_targets = false;
       for (int i = 0; i < 3; i++) {
           if (data->targets[i].present) {
               has_targets = true;
               break;
           }
       }
       
       return has_targets ? ESP_OK : ESP_ERR_NOT_FOUND;
   }
   
   /**
    * @brief Register a callback for target data
    */
   esp_err_t ld2450_register_data_callback(ld2450_data_callback_t callback, void *user_data)
   {
       if (!s_driver.initialized) {
           return ESP_ERR_LD2450_NOT_INITIALIZED;
       }
       
       // Take mutex to ensure thread safety
       if (!safe_mutex_take(100)) {
           ESP_LOGW(TAG, "Failed to acquire mutex for register_data_callback or shutdown in progress");
           return ESP_ERR_TIMEOUT;
       }
       
       // Update callback
       s_driver.data_callback = callback;
       s_driver.callback_user_data = user_data;
       
       // Release mutex
       safe_mutex_give();
       
       return ESP_OK;
   }
   
   /**
    * @brief Register an event handler for LD2450 events
    */
   esp_err_t ld2450_register_event_handler(esp_event_handler_t event_handler, void *user_data)
   {
       if (!s_driver.initialized) {
           return ESP_ERR_LD2450_NOT_INITIALIZED;
       }
       
       if (!s_driver.event_loop_enabled) {
           ESP_LOGE(TAG, "Event loop not enabled in driver configuration");
           return ESP_ERR_NOT_SUPPORTED;
       }
       
       // Register event handler with default event loop
       return esp_event_handler_register(LD2450_EVENTS, ESP_EVENT_ANY_ID, 
                                        event_handler, user_data);
   }
   
   /**
    * @brief Convert error code to string
    */
   const char* ld2450_err_to_str(esp_err_t err)
   {
       switch (err) {
           case ESP_OK:
               return "Success";
           case ESP_ERR_LD2450_TIMEOUT:
               return "Communication timeout";
           case ESP_ERR_LD2450_COMM_ERROR:
               return "Communication error";
           case ESP_ERR_LD2450_INVALID_RESPONSE:
               return "Invalid response received";
           case ESP_ERR_LD2450_COMMAND_FAILED:
               return "Command failed (ACK failure)";
           case ESP_ERR_LD2450_NOT_INITIALIZED:
               return "Driver not initialized";
           case ESP_ERR_LD2450_CONFIG_LOCKED:
               return "Configuration mode not enabled";
           case ESP_ERR_NOT_FOUND:
               return "No targets detected";
           default:
               return esp_err_to_name(err);
       }
   }
   
   /**
    * @brief Receiver task to handle incoming data from the radar
    */
   static void ld2450_receiver_task(void *arg)
   {
       if (!s_driver.initialized) {
           vTaskDelete(NULL);
           return;
       }
       
       ESP_LOGI(TAG, "Receiver task started");
       
       // Buffer for incoming data
       uint8_t rx_buffer[128];
       
       // Data frame parsing state
       ld2450_data_rx_state_t data_state = LD2450_DATA_STATE_WAIT_HEADER;
       uint8_t header_match_pos = 0;
       uint8_t trailer_match_pos = 0;
       uint8_t data_frame[64]; // Buffer to store complete data frame
       size_t data_frame_pos = 0;
       
       // Expected patterns
       static const uint8_t data_header[] = LD2450_DATA_FRAME_HEADER;
       static const uint8_t data_trailer[] = LD2450_DATA_FRAME_TRAILER;
       
       // Target presence tracking for events
       bool had_targets_previously = false;
       
       // Standard frame size: Header(4) + Target1(8) + Target2(8) + Target3(8) + Trailer(2) = 30 bytes
       const size_t standard_frame_size = 30;
       
       // Task notification value storage
       uint32_t notification_value = 0;
       
       while (true) {
           // Check for stop request via event group (non-blocking)
           EventBits_t bits = xEventGroupGetBits(s_driver.event_group);
           if ((bits & LD2450_EVENT_STOP_REQUEST) != 0) {
               ESP_LOGI(TAG, "Received stop request, exiting task");
               break;
           }
           
           // Check for task notification (with 10ms timeout)
           if (xTaskNotifyWait(0, ULONG_MAX, &notification_value, pdMS_TO_TICKS(10)) == pdTRUE) {
               // Process notification
               if (notification_value == LD2450_NOTIFICATION_STOP) {
                   ESP_LOGI(TAG, "Received stop notification, exiting task");
                   break;
               }
           }
           
           // Check how many bytes are available
           size_t available_bytes;
           if (uart_get_buffered_data_len(s_driver.uart_port, &available_bytes) != ESP_OK) {
               ESP_LOGE(TAG, "Failed to get UART buffered data length");
               vTaskDelay(pdMS_TO_TICKS(10));
               continue;
           }
           
           if (available_bytes == 0) {
               vTaskDelay(pdMS_TO_TICKS(10));
               continue;
           }
           
           // Read available data (limit to buffer size)
           size_t bytes_to_read = (available_bytes > sizeof(rx_buffer)) ? 
                                 sizeof(rx_buffer) : available_bytes;
           int bytes_read = uart_read_bytes(s_driver.uart_port, rx_buffer, 
                                           bytes_to_read, pdMS_TO_TICKS(20));
           
           if (bytes_read <= 0) {
               continue;
           }
           
           // Process each byte
           for (int i = 0; i < bytes_read; i++) {
               uint8_t byte = rx_buffer[i];
               
               // State machine for data frame parsing
               switch (data_state) {
                   case LD2450_DATA_STATE_WAIT_HEADER:
                       // Match header byte by byte
                       if (byte == data_header[header_match_pos]) {
                           // Store byte in the data frame
                           data_frame[data_frame_pos++] = byte;
                           header_match_pos++;
                           
                           if (header_match_pos == LD2450_DATA_HEADER_LEN) {
                               // Header fully matched, move to reading targets data
                               data_state = LD2450_DATA_STATE_READ_TARGETS;
                               header_match_pos = 0;
                           }
                       } else {
                           // Reset on mismatch
                           header_match_pos = 0;
                           data_frame_pos = 0;
                           
                           // If this byte matches the first header byte, keep it
                           if (byte == data_header[0]) {
                               data_frame[data_frame_pos++] = byte;
                               header_match_pos = 1;
                           }
                       }
                       break;
                       
                   case LD2450_DATA_STATE_READ_TARGETS:
                       // Store byte in data frame
                       data_frame[data_frame_pos++] = byte;
                       
                       // Check if we have all target data (24 bytes = 3 targets × 8 bytes)
                       if (data_frame_pos == LD2450_DATA_HEADER_LEN + 24) {
                           data_state = LD2450_DATA_STATE_WAIT_TRAILER;
                       }
                       break;
                       
                   case LD2450_DATA_STATE_WAIT_TRAILER:
                       // Store byte in data frame
                       data_frame[data_frame_pos++] = byte;
                       
                       // Match trailer bytes
                       if (byte == data_trailer[trailer_match_pos]) {
                           trailer_match_pos++;
                           
                           if (trailer_match_pos == LD2450_DATA_TRAILER_LEN) {
                               // Trailer fully matched, frame is complete
                               data_state = LD2450_DATA_STATE_COMPLETE;
                           }
                       } else {
                           // Reset on trailer mismatch
                           trailer_match_pos = 0;
                           
                           // Check if this byte might be the start of a new trailer
                           if (byte == data_trailer[0]) {
                               trailer_match_pos = 1;
                           }
                       }
                       break;
                       
                   case LD2450_DATA_STATE_COMPLETE:
                       // We shouldn't reach here, but if we do, start over
                       data_state = LD2450_DATA_STATE_WAIT_HEADER;
                       header_match_pos = 0;
                       trailer_match_pos = 0;
                       data_frame_pos = 0;
                       break;
               }
               
               // Process completed frame
               if (data_state == LD2450_DATA_STATE_COMPLETE) {
                   // Check for stop request before processing
                   if (should_task_stop()) {
                    ESP_LOGI(TAG, "Stop request detected while processing frame, exiting task");
                    goto task_exit;
                }
                
                // Sanity check frame size
                if (data_frame_pos != standard_frame_size) {
                    ESP_LOGW(TAG, "Unexpected data frame size: %u bytes", data_frame_pos);
                } else {
                    // Process data frame
                    ld2450_frame_data_t frame;
                    esp_err_t result = ld2450_process_data_frame(data_frame, data_frame_pos, &frame);
                    
                    if (result == ESP_OK) {
                        // Take mutex for thread safety
                        if (safe_mutex_take(10)) {
                            // Update latest data
                            memcpy(&s_driver.latest_data, &frame, sizeof(frame));
                            
                            // Get callback reference while holding mutex
                            ld2450_data_callback_t callback = s_driver.data_callback;
                            void* callback_data = s_driver.callback_user_data;
                            
                            // Release mutex before calling callback
                            safe_mutex_give();
                            
                            // Call user callback if registered
                            if (callback != NULL) {
                                callback(&frame, callback_data);
                            }
                        } else {
                            // If mutex take failed, could be shutdown in progress
                            if (should_task_stop()) {
                                ESP_LOGI(TAG, "Stop request detected, exiting task");
                                goto task_exit;
                            }
                        }
                        
                        // Add to data queue for wait_for_data function (non-blocking)
                        if (s_driver.data_queue != NULL) {
                            xQueueSendToBack(s_driver.data_queue, &frame, 0);
                        }
                        
                        // Send events if enabled
                        if (s_driver.event_loop_enabled) {
                            bool has_targets = false;
                            for (int j = 0; j < 3; j++) {
                                if (frame.targets[j].present) {
                                    has_targets = true;
                                    break;
                                }
                            }
                            
                            // Target state change events
                            if (has_targets && !had_targets_previously) {
                                esp_event_post(LD2450_EVENTS, LD2450_EVENT_TARGET_DETECTED,
                                              &frame, sizeof(frame), 0);
                            } else if (!has_targets && had_targets_previously) {
                                esp_event_post(LD2450_EVENTS, LD2450_EVENT_TARGET_LOST,
                                              NULL, 0, 0);
                            }
                            
                            had_targets_previously = has_targets;
                        }
                    } else if (result != ESP_ERR_NOT_FOUND) {
                        // Log error only if it's not a "no targets" error
                        ESP_LOGW(TAG, "Failed to process data frame: %s", ld2450_err_to_str(result));
                    }
                }
                
                // Reset for next frame
                data_state = LD2450_DATA_STATE_WAIT_HEADER;
                header_match_pos = 0;
                trailer_match_pos = 0;
                data_frame_pos = 0;
            }
            
            // Prevent buffer overflow
            if (data_frame_pos >= sizeof(data_frame)) {
                ESP_LOGW(TAG, "Data frame buffer overflow, resetting parser");
                data_state = LD2450_DATA_STATE_WAIT_HEADER;
                header_match_pos = 0;
                trailer_match_pos = 0;
                data_frame_pos = 0;
            }
        }
    }
    
task_exit:
    // Signal that task is stopping
    if (s_driver.event_group != NULL) {
        xEventGroupSetBits(s_driver.event_group, LD2450_EVENT_TASK_STOPPED);
    }
    ESP_LOGI(TAG, "Receiver task exiting cleanly");
    vTaskDelete(NULL);
}