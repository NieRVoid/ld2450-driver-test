/**
 * @file main.c
 * @brief Basic operation example for the HLK-LD2450 radar driver
 * 
 * This example initializes the LD2450 driver, registers a callback to receive 
 * target data, and displays the results every 3 seconds.
 * 
 * @author NieRVoid
 * @date 2025-03-12
 * @license MIT
 */

 #include <stdio.h>
 #include <string.h>
 #include "freertos/FreeRTOS.h"
 #include "freertos/task.h"
 #include "esp_log.h"
 #include "esp_system.h"
 #include "ld2450.h"
 
 static const char *TAG = "ld2450_example";
 
 // Shared buffer for radar data to be displayed in main loop
 static ld2450_frame_t latest_frame = {0};
 static SemaphoreHandle_t data_mutex = NULL;
 static bool new_data_available = false;
 
 /**
  * @brief Callback function for radar data
  * 
  * This function is called by the driver when new target data is received.
  * It stores the data in a shared buffer for display in the main loop.
  */
 static void radar_data_callback(const ld2450_frame_t *frame, void *user_ctx)
 {
     if (xSemaphoreTake(data_mutex, 0) == pdTRUE) {
         // Copy the frame data to our buffer
         memcpy(&latest_frame, frame, sizeof(ld2450_frame_t));
         new_data_available = true;
         xSemaphoreGive(data_mutex);
     }
 }
 
 /**
  * @brief Display radar target information
  * 
  * Prints the current target information to the console
  */
 static void display_radar_data(void)
 {
     if (xSemaphoreTake(data_mutex, portMAX_DELAY) == pdTRUE) {
         if (new_data_available) {
             ESP_LOGI(TAG, "----- Radar Data -----");
             
             if (latest_frame.count == 0) {
                 ESP_LOGI(TAG, "No targets detected");
             } else {
                 ESP_LOGI(TAG, "Detected %d targets:", latest_frame.count);
                 
                 for (int i = 0; i < latest_frame.count; i++) {
                     const ld2450_target_t *target = &latest_frame.targets[i];
                     
                     ESP_LOGI(TAG, "Target %d:", i + 1);
                     ESP_LOGI(TAG, "  Position: X=%d mm, Y=%d mm", target->x, target->y);
                     ESP_LOGI(TAG, "  Polar: Distance=%.2f mm, Angle=%.2f°", 
                              target->distance, target->angle);
                     ESP_LOGI(TAG, "  Speed: %d cm/s", target->speed);
                     ESP_LOGI(TAG, "  Resolution: %u mm", target->resolution);
                 }
             }
             
             new_data_available = false;
         }
         xSemaphoreGive(data_mutex);
     }
 }
 
 /**
  * @brief Configure the radar with some basic settings
  * 
  * This function demonstrates how to use configuration functions.
  * We'll get the firmware version and set the tracking mode.
  */
 static void configure_radar(void)
 {
     esp_err_t ret;
     ld2450_firmware_version_t version;
     ld2450_tracking_mode_t mode;
     
     // Get firmware version
     ret = ld2450_get_firmware_version(&version);
     if (ret == ESP_OK) {
         ESP_LOGI(TAG, "Radar firmware version: %s", version.version_string);
     } else {
         ESP_LOGW(TAG, "Failed to get firmware version: %s", esp_err_to_name(ret));
     }
     
     // Get current tracking mode
     ret = ld2450_get_tracking_mode(&mode);
     if (ret == ESP_OK) {
         ESP_LOGI(TAG, "Current tracking mode: %s", 
                  mode == LD2450_MODE_SINGLE_TARGET ? "Single Target" : "Multi Target");
     } else {
         ESP_LOGW(TAG, "Failed to get tracking mode: %s", esp_err_to_name(ret));
     }
     
     // Set multi-target tracking mode
     ret = ld2450_set_tracking_mode(LD2450_MODE_MULTI_TARGET);
     if (ret == ESP_OK) {
         ESP_LOGI(TAG, "Set tracking mode to Multi Target");
     } else {
         ESP_LOGW(TAG, "Failed to set tracking mode: %s", esp_err_to_name(ret));
     }
 }
 
 void app_main(void)
 {
     ESP_LOGI(TAG, "HLK-LD2450 Basic Example");
     
     // Create mutex for thread-safe access to shared data
     data_mutex = xSemaphoreCreateMutex();
     if (!data_mutex) {
         ESP_LOGE(TAG, "Failed to create mutex");
         return;
     }
     
     // Initialize the LD2450 driver with default configuration
     ld2450_config_t config = LD2450_DEFAULT_CONFIG();
     
     // You can modify the default configuration here if needed
     // config.uart_port = UART_NUM_1;
     // config.uart_rx_pin = 5;
     // config.uart_tx_pin = 6;
     // config.uart_baud_rate = 115200;
     
     ESP_LOGI(TAG, "Initializing LD2450 driver...");
     esp_err_t ret = ld2450_init(&config);
     if (ret != ESP_OK) {
         ESP_LOGE(TAG, "Failed to initialize LD2450 driver: %s", esp_err_to_name(ret));
         return;
     }
     
     // Register callback for target data
     ret = ld2450_register_target_callback(radar_data_callback, NULL);
     if (ret != ESP_OK) {
         ESP_LOGE(TAG, "Failed to register callback: %s", esp_err_to_name(ret));
         return;
     }
     
     // Configure radar with example settings
     vTaskDelay(pdMS_TO_TICKS(1000));  // Give the radar some time to initialize
    //  configure_radar();
     
     // Main loop - display radar data every 3 seconds
     while (1) {
         display_radar_data();
         vTaskDelay(pdMS_TO_TICKS(3000));  // 3 second delay
     }
     
     // The following code will never be reached in this example
     ld2450_deinit();
     vSemaphoreDelete(data_mutex);
 }