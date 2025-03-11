/**
 * @file ld2450.h
 * @brief HLK-LD2450 radar sensor driver for ESP32-IDF
 *
 * This file provides the public API for interfacing with the HLK-LD2450
 * 24GHz radar sensor module using ESP32-IDF v5.4. It allows configuration,
 * control, and data retrieval from the sensor.
 *
 * @copyright Copyright (c) 2025 NieRVoid
 * @license MIT License
 */

#ifndef LD2450_H_
#define LD2450_H_

#include "esp_err.h"
#include "driver/uart.h"
#include "esp_event.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** 
 * @brief Error codes specific to LD2450 driver
 */
#define ESP_ERR_LD2450_BASE                  0x10000
#define ESP_ERR_LD2450_TIMEOUT               (ESP_ERR_LD2450_BASE + 1)  /*!< Timeout waiting for response */
#define ESP_ERR_LD2450_COMM_ERROR            (ESP_ERR_LD2450_BASE + 2)  /*!< Communication error */
#define ESP_ERR_LD2450_INVALID_RESPONSE      (ESP_ERR_LD2450_BASE + 3)  /*!< Invalid response received */
#define ESP_ERR_LD2450_COMMAND_FAILED        (ESP_ERR_LD2450_BASE + 4)  /*!< Command failed (ACK failure) */
#define ESP_ERR_LD2450_NOT_INITIALIZED       (ESP_ERR_LD2450_BASE + 5)  /*!< Driver not initialized */
#define ESP_ERR_LD2450_INVALID_ARG           (ESP_ERR_LD2450_BASE + 6)  /*!< Invalid argument */
#define ESP_ERR_LD2450_CONFIG_LOCKED         (ESP_ERR_LD2450_BASE + 7)  /*!< Configuration mode not enabled */

/**
 * @brief LD2450 event base
 */
ESP_EVENT_DECLARE_BASE(LD2450_EVENTS);

/**
 * @brief LD2450 events dispatched via event loop
 */
typedef enum {
    LD2450_EVENT_TARGET_DETECTED,    /*!< Target(s) detected */
    LD2450_EVENT_TARGET_LOST,        /*!< Target(s) lost */
    LD2450_EVENT_COMM_ERROR,         /*!< Communication error */
    LD2450_EVENT_CONFIG_CHANGED,     /*!< Configuration changed */
} ld2450_event_t;

/**
 * @brief Target tracking modes
 */
typedef enum {
    LD2450_MODE_SINGLE_TARGET,       /*!< Single target tracking mode */
    LD2450_MODE_MULTI_TARGET         /*!< Multi-target tracking mode (default) */
} ld2450_tracking_mode_t;

/**
 * @brief Baud rate settings for the radar
 */
typedef enum {
    LD2450_BAUD_RATE_9600,
    LD2450_BAUD_RATE_19200,
    LD2450_BAUD_RATE_38400,
    LD2450_BAUD_RATE_57600,
    LD2450_BAUD_RATE_115200,
    LD2450_BAUD_RATE_230400,
    LD2450_BAUD_RATE_256000,         /*!< Default */
    LD2450_BAUD_RATE_460800
} ld2450_baud_rate_t;

/**
 * @brief Region filter modes
 */
typedef enum {
    LD2450_FILTER_NONE,              /*!< No region filtering */
    LD2450_FILTER_INSIDE_REGION,     /*!< Only detect targets inside region */
    LD2450_FILTER_OUTSIDE_REGION     /*!< Only detect targets outside region */
} ld2450_filter_mode_t;

/**
 * @brief Bluetooth mode settings
 */
typedef enum {
    LD2450_BT_OFF = 0,               /*!< Bluetooth disabled */
    LD2450_BT_ON = 1                 /*!< Bluetooth enabled (default) */
} ld2450_bt_mode_t;

/**
 * @brief Target data structure
 */
 typedef struct {
    int16_t x;           /*!< X coordinate in mm */
    int16_t y;           /*!< Y coordinate in mm */
    int16_t speed;       /*!< Speed in cm/s, positive is approaching, negative is receding */
    uint16_t resolution; /*!< Distance resolution in mm */
    bool present;        /*!< Whether target is present */
} ld2450_target_data_t;

/**
 * @brief Full radar data frame containing all targets
 */
typedef struct {
    ld2450_target_data_t targets[3]; /*!< Data for up to 3 targets */
    uint32_t timestamp;              /*!< Timestamp when data was received */
} ld2450_frame_data_t;

/**
 * @brief Coordinates for region filtering
 */
typedef struct {
    int16_t x;           /*!< X coordinate in mm */
    int16_t y;           /*!< Y coordinate in mm */
} ld2450_point_t;

/**
 * @brief Region filter configuration
 */
typedef struct {
    ld2450_filter_mode_t mode;      /*!< Region filtering mode */
    ld2450_point_t region1_p1;      /*!< Region 1, first point */
    ld2450_point_t region1_p2;      /*!< Region 1, second point (diagonal) */
    ld2450_point_t region2_p1;      /*!< Region 2, first point */
    ld2450_point_t region2_p2;      /*!< Region 2, second point (diagonal) */
    ld2450_point_t region3_p1;      /*!< Region 3, first point */
    ld2450_point_t region3_p2;      /*!< Region 3, second point (diagonal) */
} ld2450_region_filter_t;

/**
 * @brief Firmware version information
 */
typedef struct {
    uint16_t main_version;          /*!< Main version number */
    uint32_t sub_version;           /*!< Sub-version number */
    char version_str[16];           /*!< Version string (e.g., "V1.02.22062416") */
} ld2450_firmware_version_t;

/**
 * @brief Callback function for receiving target data
 */
typedef void (*ld2450_data_callback_t)(const ld2450_frame_data_t *data, void *user_data);

/**
 * @brief LD2450 driver configuration
 */
typedef struct {
    uart_port_t uart_port;          /*!< UART port number */
    int tx_pin;                     /*!< UART TX pin */
    int rx_pin;                     /*!< UART RX pin */
    ld2450_baud_rate_t baud_rate;   /*!< UART baud rate */
    size_t rx_buffer_size;          /*!< UART RX buffer size */
    size_t tx_buffer_size;          /*!< UART TX buffer size */
    int task_priority;              /*!< Receiver task priority */
    size_t task_stack_size;         /*!< Receiver task stack size */
    bool event_loop_enabled;        /*!< Whether to use event loop */
} ld2450_config_t;

/**
 * @brief Default configuration for LD2450 driver
 */
#define LD2450_DEFAULT_CONFIG() { \
    .uart_port = UART_NUM_1, \
    .tx_pin = 17, \
    .rx_pin = 16, \
    .baud_rate = LD2450_BAUD_RATE_256000, \
    .rx_buffer_size = 1024, \
    .tx_buffer_size = 256, \
    .task_priority = 5, \
    .task_stack_size = 4096, \
    .event_loop_enabled = true, \
}

/**
 * @brief Initialize the LD2450 radar driver
 * 
 * @param config Pointer to driver configuration
 * @return esp_err_t ESP_OK on success, or an error code
 */
esp_err_t ld2450_init(const ld2450_config_t *config);

/**
 * @brief Deinitialize the LD2450 radar driver and release resources
 * 
 * @return esp_err_t ESP_OK on success, or an error code
 */
esp_err_t ld2450_deinit(void);

/**
 * @brief Enable configuration mode
 * 
 * This must be called before any other configuration commands.
 * 
 * @return esp_err_t ESP_OK on success, or an error code
 */
esp_err_t ld2450_enable_config(void);

/**
 * @brief End configuration mode
 * 
 * This must be called after configuration commands to return to normal operation.
 * 
 * @return esp_err_t ESP_OK on success, or an error code
 */
esp_err_t ld2450_end_config(void);

/**
 * @brief Set target tracking mode
 * 
 * @param mode Tracking mode (single or multi-target)
 * @return esp_err_t ESP_OK on success, or an error code
 */
esp_err_t ld2450_set_tracking_mode(ld2450_tracking_mode_t mode);

/**
 * @brief Get current tracking mode
 * 
 * @param mode Pointer to store the tracking mode
 * @return esp_err_t ESP_OK on success, or an error code
 */
esp_err_t ld2450_get_tracking_mode(ld2450_tracking_mode_t *mode);

/**
 * @brief Read firmware version from radar
 * 
 * @param version Pointer to store the firmware version
 * @return esp_err_t ESP_OK on success, or an error code
 */
esp_err_t ld2450_get_firmware_version(ld2450_firmware_version_t *version);

/**
 * @brief Set serial port baud rate
 * 
 * This change requires a module restart to take effect.
 * 
 * @param baud_rate New baud rate
 * @return esp_err_t ESP_OK on success, or an error code
 */
esp_err_t ld2450_set_baud_rate(ld2450_baud_rate_t baud_rate);

/**
 * @brief Restore factory settings
 * 
 * This command resets all settings to factory defaults.
 * 
 * @return esp_err_t ESP_OK on success, or an error code
 */
esp_err_t ld2450_restore_factory_settings(void);

/**
 * @brief Restart the radar module
 * 
 * @return esp_err_t ESP_OK on success, or an error code
 */
esp_err_t ld2450_restart(void);

/**
 * @brief Configure Bluetooth settings
 * 
 * @param mode Bluetooth mode (on/off)
 * @return esp_err_t ESP_OK on success, or an error code
 */
esp_err_t ld2450_set_bluetooth(ld2450_bt_mode_t mode);

/**
 * @brief Get radar MAC address
 * 
 * @param mac_addr Buffer to store the 6-byte MAC address
 * @return esp_err_t ESP_OK on success, or an error code
 */
esp_err_t ld2450_get_mac_address(uint8_t mac_addr[6]);

/**
 * @brief Configure region filtering
 * 
 * @param filter Pointer to region filter configuration
 * @return esp_err_t ESP_OK on success, or an error code
 */
esp_err_t ld2450_set_region_filter(const ld2450_region_filter_t *filter);

/**
 * @brief Get current region filter configuration
 * 
 * @param filter Pointer to store region filter configuration
 * @return esp_err_t ESP_OK on success, or an error code
 */
esp_err_t ld2450_get_region_filter(ld2450_region_filter_t *filter);

/**
 * @brief Get latest target data (non-blocking)
 * 
 * Returns immediately with the most recent data.
 * 
 * @param data Pointer to store radar frame data
 * @return esp_err_t ESP_OK on success, or an error code
 */
esp_err_t ld2450_get_data(ld2450_frame_data_t *data);

/**
 * @brief Wait for new target data (blocking)
 * 
 * Blocks until new data is available or timeout occurs.
 * 
 * @param data Pointer to store radar frame data
 * @param timeout_ms Timeout in milliseconds
 * @return esp_err_t ESP_OK on success, or an error code
 */
esp_err_t ld2450_wait_for_data(ld2450_frame_data_t *data, uint32_t timeout_ms);

/**
 * @brief Register a callback for target data
 * 
 * @param callback Function to call when new data arrives
 * @param user_data User data to pass to callback
 * @return esp_err_t ESP_OK on success, or an error code
 */
esp_err_t ld2450_register_data_callback(ld2450_data_callback_t callback, void *user_data);

/**
 * @brief Register an event handler for LD2450 events
 * 
 * @param event_handler Event handler function
 * @param user_data User data to pass to event handler
 * @return esp_err_t ESP_OK on success, or an error code
 */
esp_err_t ld2450_register_event_handler(esp_event_handler_t event_handler, void *user_data);

/**
 * @brief Convert error code to string
 * 
 * @param err Error code
 * @return const char* Error message string
 */
const char* ld2450_err_to_str(esp_err_t err);

#ifdef __cplusplus
}
#endif

#endif /* LD2450_H_ */