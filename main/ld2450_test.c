#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "ld2450.h"

static const char *TAG = "LD2450_TEST";

// Button configuration
#define BOOT_BUTTON_GPIO          0
#define BUTTON_DEBOUNCE_TIME_MS   50
#define DOUBLE_PRESS_INTERVAL_MS  500

// Global variables for button state and program control
static volatile bool g_exit_app = false;
static int64_t g_last_button_press = 0;
static int64_t g_last_display_time = 0;
static const int64_t DISPLAY_INTERVAL_US = 3000000; // 3 seconds in microseconds

// GPIO interrupt handler
static void IRAM_ATTR button_isr_handler(void* arg) {
    int64_t current_time = esp_timer_get_time();
    int64_t diff = current_time - g_last_button_press;
    
    // Debounce check
    if (diff < BUTTON_DEBOUNCE_TIME_MS * 1000) {
        return;
    }
    
    // Check for double press
    if (diff < DOUBLE_PRESS_INTERVAL_MS * 1000) {
        g_exit_app = true;
    }
    
    g_last_button_press = current_time;
}

// Utility function to print buffer as hex dump
void print_hex_dump(const char* prefix, const uint8_t* data, size_t len) {
    if (!data || len == 0) {
        ESP_LOGI(TAG, "%s: <empty buffer>", prefix);
        return;
    }
    
    ESP_LOGI(TAG, "%s (%u bytes):", prefix, len);
    
    char line[128];
    char *ptr = line;
    
    for (size_t i = 0; i < len; i++) {
        if (i % 16 == 0) {
            if (i > 0) {
                *ptr = '\0';
                ESP_LOGI(TAG, "%s", line);
            }
            ptr = line;
            ptr += sprintf(ptr, "  %04x: ", (unsigned int)i);
        }
        
        ptr += sprintf(ptr, "%02x ", data[i]);
        
        // Add extra space after 8 bytes for readability
        if ((i % 16 == 7)) {
            ptr += sprintf(ptr, " ");
        }
    }
    
    // Print any remaining bytes
    if (ptr != line) {
        *ptr = '\0';
        ESP_LOGI(TAG, "%s", line);
    }
}

// Function to retrieve and print error debug info from driver
void print_error_debug_info(void) {
    uint8_t debug_buffer[256];
    size_t debug_len = 0;
    
    esp_err_t ret = ld2450_get_last_error_data(debug_buffer, sizeof(debug_buffer), &debug_len);
    if (ret == ESP_OK && debug_len > 0) {
        print_hex_dump("Last error data", debug_buffer, debug_len);
    } else {
        ESP_LOGI(TAG, "No error debug data available");
    }
}

// Callback function to handle radar detection data
static void radar_data_callback(const ld2450_frame_t *frame, void *user_ctx) {
    if (!frame) return;
    
    int64_t current_time = esp_timer_get_time();
    
    // Only print radar data every 3 seconds
    if (current_time - g_last_display_time >= DISPLAY_INTERVAL_US) {
        ESP_LOGI(TAG, "---------- Radar Data Frame ----------");
        ESP_LOGI(TAG, "Timestamp: %lld ms", frame->timestamp / 1000);
        ESP_LOGI(TAG, "Number of targets detected: %d", frame->count);
        
        for (int i = 0; i < frame->count; i++) {
            const ld2450_target_t *target = &frame->targets[i];
            if (target->valid) {
                ESP_LOGI(TAG, "Target #%d:", i + 1);
                ESP_LOGI(TAG, "  Position:   (%d, %d) mm", target->x, target->y);
                ESP_LOGI(TAG, "  Distance:   %.2f mm", target->distance);
                ESP_LOGI(TAG, "  Angle:      %.1f°", target->angle);
                ESP_LOGI(TAG, "  Speed:      %d cm/s", target->speed);
                ESP_LOGI(TAG, "  Resolution: %d mm", target->resolution);
            }
        }
        ESP_LOGI(TAG, "--------------------------------------\n");
        
        // Update last display time
        g_last_display_time = current_time;
    }
}

void print_mac_address(uint8_t mac[6]) {
    ESP_LOGI(TAG, "MAC Address: %02X:%02X:%02X:%02X:%02X:%02X", 
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

void print_region_filter(ld2450_filter_type_t type, ld2450_region_t regions[3]) {
    const char *type_str = "Unknown";
    switch (type) {
        case LD2450_FILTER_DISABLED: type_str = "Disabled"; break;
        case LD2450_FILTER_INCLUDE_ONLY: type_str = "Include Only"; break;
        case LD2450_FILTER_EXCLUDE: type_str = "Exclude"; break;
    }
    
    ESP_LOGI(TAG, "Region Filter: %s", type_str);
    if (type != LD2450_FILTER_DISABLED) {
        for (int i = 0; i < 3; i++) {
            ESP_LOGI(TAG, "  Region %d: (%d,%d) to (%d,%d) mm", 
                    i+1, regions[i].x1, regions[i].y1, regions[i].x2, regions[i].y2);
        }
    }
}

void app_main(void) {
    ESP_LOGI(TAG, "Starting LD2450 Radar Test");
    
    // Configure GPIO for boot button
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BOOT_BUTTON_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
    };
    gpio_config(&io_conf);
    
    // Install GPIO ISR handler
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BOOT_BUTTON_GPIO, button_isr_handler, NULL);
    
    // Initialize LD2450 with default configuration
    ld2450_config_t config = LD2450_DEFAULT_CONFIG();
    
    // Print configuration settings
    ESP_LOGI(TAG, "Initializing LD2450 with:");
    ESP_LOGI(TAG, "  UART Port: %d", config.uart_port);
    ESP_LOGI(TAG, "  RX Pin: GPIO%d", config.uart_rx_pin);
    ESP_LOGI(TAG, "  TX Pin: GPIO%d", config.uart_tx_pin);
    ESP_LOGI(TAG, "  Baud Rate: %lu", config.uart_baud_rate);
    ESP_LOGI(TAG, "  Auto Processing: %s", config.auto_processing ? "Enabled" : "Disabled");
    
    esp_err_t ret = ld2450_init(&config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize LD2450! Error: %s", esp_err_to_name(ret));
        print_error_debug_info();
        return;
    }
    ESP_LOGI(TAG, "LD2450 initialized successfully");
    
    // Register callback for target data
    ret = ld2450_register_target_callback(radar_data_callback, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register callback! Error: %s", esp_err_to_name(ret));
        print_error_debug_info();
        return;
    }
    
    // Configure the radar
    ESP_LOGI(TAG, "Configuring radar settings...");
    
    // 1. Enable Bluetooth
    ret = ld2450_set_bluetooth(true);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable Bluetooth! Error: %s", esp_err_to_name(ret));
        print_error_debug_info();
    } else {
        ESP_LOGI(TAG, "Bluetooth enabled successfully");
    }
    
    // 2. Set tracking mode to multi-target
    ret = ld2450_set_tracking_mode(LD2450_MODE_MULTI_TARGET);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set tracking mode! Error: %s", esp_err_to_name(ret));
        print_error_debug_info();
    } else {
        ESP_LOGI(TAG, "Tracking mode set to multi-target");
    }
    
    // 3. Disable area filtering
    ld2450_region_t empty_regions[3] = {0};
    ret = ld2450_set_region_filter(LD2450_FILTER_DISABLED, empty_regions);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to disable area filtering! Error: %s", esp_err_to_name(ret));
        print_error_debug_info();
    } else {
        ESP_LOGI(TAG, "Area filtering disabled");
    }
    
    // 4. Restart radar module
    ESP_LOGI(TAG, "Restarting radar module...");
    ret = ld2450_restart_module();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to restart module! Error: %s", esp_err_to_name(ret));
        print_error_debug_info();
    }
    
    // Wait for the module to restart
    vTaskDelay(pdMS_TO_TICKS(3000));
    ESP_LOGI(TAG, "Radar module restarted");
    
    // 5. Get configuration information
    ESP_LOGI(TAG, "\n======== RADAR CONFIGURATION ========");
    
    // Get firmware version
    ld2450_firmware_version_t version;
    ret = ld2450_get_firmware_version(&version);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Firmware: %s (Main: %" PRIu16 ", Sub: %" PRIu32 ")", 
                 version.version_string, version.main_version, version.sub_version);
    } else {
        ESP_LOGE(TAG, "Failed to get firmware version! Error: %s", esp_err_to_name(ret));
        print_error_debug_info();
    }
    
    // Get MAC address
    uint8_t mac[6];
    ret = ld2450_get_mac_address(mac);
    if (ret == ESP_OK) {
        print_mac_address(mac);
    } else {
        ESP_LOGE(TAG, "Failed to get MAC address! Error: %s", esp_err_to_name(ret));
        print_error_debug_info();
    }
    
    // Get tracking mode
    ld2450_tracking_mode_t mode;
    ret = ld2450_get_tracking_mode(&mode);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Tracking Mode: %s", 
                 mode == LD2450_MODE_SINGLE_TARGET ? "Single Target" : "Multi Target");
    } else {
        ESP_LOGE(TAG, "Failed to get tracking mode! Error: %s", esp_err_to_name(ret));
        print_error_debug_info();
    }
    
    // Get region filter configuration
    ld2450_filter_type_t filter_type;
    ld2450_region_t regions[3];
    ret = ld2450_get_region_filter(&filter_type, regions);
    if (ret == ESP_OK) {
        print_region_filter(filter_type, regions);
    } else {
        ESP_LOGE(TAG, "Failed to get region filter! Error: %s", esp_err_to_name(ret));
        print_error_debug_info();
    }
    
    ESP_LOGI(TAG, "===================================\n");
    
    // Initialize last display time
    g_last_display_time = esp_timer_get_time();
    
    // Wait for and process radar data
    ESP_LOGI(TAG, "Waiting for radar detection data...");
    ESP_LOGI(TAG, "(Press boot button twice quickly to exit)");
    
    // Main loop - the callback will handle incoming data
    while (!g_exit_app) {
        vTaskDelay(pdMS_TO_TICKS(100)); // Check exit flag every 100ms
    }
    
    // Clean up before exiting
    ESP_LOGI(TAG, "Exiting program...");
    gpio_isr_handler_remove(BOOT_BUTTON_GPIO);
    ld2450_deinit();
    ESP_LOGI(TAG, "Cleanup complete, goodbye!");
}
