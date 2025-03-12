/**
 * @file basic_operation.c
 * @brief Basic example for the HLK-LD2450 driver component
 *
 * This example demonstrates how to initialize the driver, register a callback for
 * target detection events, and display information about detected targets.
 * 
 * @author NieRVoid
 * @date 2025-03-12
 * @license MIT
 */

 #include <stdio.h>
 #include "esp_log.h"
 #include "freertos/FreeRTOS.h"
 #include "freertos/task.h"
 #include "esp_system.h"
 #include "ld2450.h"
 
 static const char *TAG = "ld2450_example";
 
 /**
  * @brief Callback function for target detection events
  * 
  * This function is called when new target data is available from the radar
  * 
  * @param frame Pointer to the frame containing target data
  * @param user_ctx User context pointer (not used in this example)
  */
 void target_detection_callback(const ld2450_frame_t *frame, void *user_ctx)
 {
     ESP_LOGI(TAG, "Target frame received: %d targets detected", frame->count);
     
     // Process each valid target
     for (uint8_t i = 0; i < frame->count; i++) {
         const ld2450_target_t *target = &frame->targets[i];
         
         // Log target information
         ESP_LOGI(TAG, "Target %d: position (x=%d, y=%d) mm, speed=%d cm/s, "
                      "distance=%.1f mm, angle=%.1f°",
                  i + 1, target->x, target->y, target->speed, 
                  target->distance, target->angle);
     }
     
     // Log an empty line for better readability in the console
     if (frame->count > 0) {
         printf("\n");
     }
 }
 
 void app_main(void)
 {
     ESP_LOGI(TAG, "Starting HLK-LD2450 basic example");
     
     // Initialize the LD2450 driver with default configuration
     ld2450_config_t config = LD2450_DEFAULT_CONFIG();
     
     // Customize configuration if needed
     // config.uart_port = UART_NUM_1;    // Change UART port
     // config.uart_rx_pin = 9;           // Change RX pin
     // config.uart_tx_pin = 10;          // Change TX pin
     
     // Initialize the driver
     esp_err_t ret = ld2450_init(&config);
     if (ret != ESP_OK) {
         ESP_LOGE(TAG, "Failed to initialize LD2450 driver: %s", esp_err_to_name(ret));
         return;
     }
     
     // Register callback for target detection events
     ret = ld2450_register_target_callback(target_detection_callback, NULL);
     if (ret != ESP_OK) {
         ESP_LOGE(TAG, "Failed to register callback: %s", esp_err_to_name(ret));
         return;
     }
     
     // Get firmware version
     ld2450_firmware_version_t version;
     ret = ld2450_get_firmware_version(&version);
     if (ret == ESP_OK) {
         ESP_LOGI(TAG, "Radar firmware version: %s", version.version_string);
     } else {
         ESP_LOGW(TAG, "Failed to read firmware version: %s", esp_err_to_name(ret));
     }
     
     // Set tracking mode to multi-target (default, but explicitly set for demonstration)
     ret = ld2450_set_tracking_mode(LD2450_MODE_MULTI_TARGET);
     if (ret != ESP_OK) {
         ESP_LOGW(TAG, "Failed to set tracking mode: %s", esp_err_to_name(ret));
     }
     
     ESP_LOGI(TAG, "LD2450 radar initialized successfully");
     ESP_LOGI(TAG, "Waiting for target detection events...");
     
     // Main loop - data processing is handled by the driver in the background
     while (1) {
         // This example doesn't need to do anything in the main loop
         // The target_detection_callback will be called when targets are detected
         vTaskDelay(pdMS_TO_TICKS(1000));
     }
     
     // This code is unreachable but shown for completeness
     ld2450_deinit();
 }