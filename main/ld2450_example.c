/**
 * @file main.c
 * @brief Basic example of using the LD2450 radar sensor driver
 * 
 * This example demonstrates how to initialize the LD2450 driver,
 * configure the sensor, and receive target detection data.
 * 
 * @copyright Copyright (c) 2025 NieRVoid
 * @license MIT License
 */

 #include <stdio.h>
 #include <string.h>
 #include "freertos/FreeRTOS.h"
 #include "freertos/task.h"
 #include "esp_log.h"
 #include "esp_system.h"
 #include "esp_timer.h"
 #include "nvs_flash.h"
 #include "ld2450.h"
 
 static const char *TAG = "ld2450_example";
 
 /* UART port and pins for LD2450 radar */
 #define LD2450_UART_PORT      UART_NUM_1
 #define LD2450_UART_TX_PIN    GPIO_NUM_17
 #define LD2450_UART_RX_PIN    GPIO_NUM_18
 #define LD2450_UART_BAUD_RATE 256000
 
 /* Settings for radar sensor */
 #define USE_MULTI_TARGET_MODE true
 #define PRINT_INTERVAL_MS     500
 
 /* Radar data callback function */
 static void radar_data_callback(const ld2450_target_t targets[], 
                               size_t active_targets, 
                               void *user_ctx);
 
 void app_main(void)
 {
     /* Initialize NVS */
     esp_err_t ret = nvs_flash_init();
     if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
         ESP_ERROR_CHECK(nvs_flash_erase());
         ret = nvs_flash_init();
     }
     ESP_ERROR_CHECK(ret);
     
     ESP_LOGI(TAG, "Starting LD2450 basic example...");
     
     /* Configure the LD2450 driver */
     ld2450_config_t radar_config = {
         .uart_port = LD2450_UART_PORT,
         .uart_tx_pin = LD2450_UART_TX_PIN,
         .uart_rx_pin = LD2450_UART_RX_PIN,
         .uart_baud_rate = LD2450_UART_BAUD_RATE,
     };
     
     /* Initialize the LD2450 driver */
     ESP_LOGI(TAG, "Initializing LD2450 radar driver...");
     ret = ld2450_init(&radar_config);
     if (ret != ESP_OK) {
         ESP_LOGE(TAG, "Failed to initialize LD2450 driver: %s", esp_err_to_name(ret));
         return;
     }
     
     /* Register data callback */
     ret = ld2450_register_data_callback(radar_data_callback, NULL);
     if (ret != ESP_OK) {
         ESP_LOGE(TAG, "Failed to register data callback: %s", esp_err_to_name(ret));
         goto cleanup;
     }
     
     /* Configure the radar sensor */
     ESP_LOGI(TAG, "Configuring LD2450 radar...");
     ret = ld2450_cmd_configure_tracking(USE_MULTI_TARGET_MODE);
     if (ret != ESP_OK) {
         ESP_LOGE(TAG, "Failed to configure tracking mode: %s", esp_err_to_name(ret));
         goto cleanup;
     }
     
     /* Get firmware version */
     char version_str[16] = {0};
     ret = ld2450_cmd_get_firmware_info(version_str, sizeof(version_str));
     if (ret == ESP_OK) {
         ESP_LOGI(TAG, "LD2450 firmware version: %s", version_str);
     } else {
         ESP_LOGW(TAG, "Failed to get firmware version: %s", esp_err_to_name(ret));
     }
     
     ESP_LOGI(TAG, "LD2450 radar initialized and configured!");
     ESP_LOGI(TAG, "Waiting for radar data...");
     
     /* Main loop - just sleep and let the callback handle data */
     while (1) {
         vTaskDelay(pdMS_TO_TICKS(1000));
     }
     
 cleanup:
     /* Cleanup on error */
     ld2450_deinit();
     ESP_LOGE(TAG, "Example terminated");
 }
 
 /* Target data timestamp for output rate control */
 static int64_t last_print_time = 0;
 
 /* JSON buffer for formatting target data */
 static char json_buffer[512];
 
 /**
  * @brief Callback function for radar data
  * 
  * This function is called by the LD2450 driver when new target
  * data is received from the radar sensor.
  * 
  * @param targets Array of detected targets
  * @param active_targets Number of active targets
  * @param user_ctx User context pointer (unused in this example)
  */
 static void radar_data_callback(const ld2450_target_t targets[], 
                               size_t active_targets, 
                               void *user_ctx)
 {
     /* Get current time */
     int64_t now = esp_timer_get_time() / 1000; /* Convert to ms */
     
     /* Only print data at specified intervals to avoid flooding the console */
     if (now - last_print_time < PRINT_INTERVAL_MS) {
         return;
     }
     last_print_time = now;
     
     /* Skip if no active targets */
     if (active_targets == 0) {
         ESP_LOGI(TAG, "No targets detected");
         return;
     }
     
     /* Convert target data to JSON for display */
     size_t json_len = 0;
     esp_err_t ret = ld2450_cmd_targets_to_json(targets, LD2450_MAX_TARGETS, 
                                              json_buffer, sizeof(json_buffer), 
                                              &json_len);
     if (ret != ESP_OK) {
         ESP_LOGE(TAG, "Failed to format targets as JSON: %s", esp_err_to_name(ret));
         return;
     }
     
     /* Print JSON data */
     ESP_LOGI(TAG, "Radar targets: %s", json_buffer);
     
     /* Also print individual target data in more readable format */
     ESP_LOGI(TAG, "Detected %u active targets:", active_targets);
     for (size_t i = 0; i < LD2450_MAX_TARGETS; i++) {
         if (targets[i].active) {
             /* Calculate additional metrics */
             float distance_m = sqrtf(targets[i].x * targets[i].x + 
                                     targets[i].y * targets[i].y) / 1000.0f;
             float speed_mps = targets[i].speed / 100.0f;
             
             ESP_LOGI(TAG, "  Target %u: x=%dmm, y=%dmm, distance=%.2fm, speed=%.2fm/s",
                      i, targets[i].x, targets[i].y, distance_m, speed_mps);
         }
     }
 }