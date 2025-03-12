/**
 * @file ld2450.h
 * @brief Public API for HLK-LD2450 radar sensor driver
 * 
 * This header provides the public API for interacting with HLK-LD2450 
 * mmWave radar sensor modules. The driver supports both basic and
 * advanced functionality including:
 * - Target detection and tracking (single and multi-target modes)
 * - Configuration of detection zones (include/exclude regions)
 * - Bluetooth control
 * - Firmware information retrieval
 * 
 * @copyright Copyright (c) 2025 NieRVoid
 * @license MIT License
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include "esp_err.h"
#include "driver/uart.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Maximum number of targets that can be tracked simultaneously
 */
#define LD2450_MAX_TARGETS 3

/**
 * @brief Target tracking data from the radar sensor
 */
typedef struct {
    bool active;            /*!< Whether this target is active (detected) */
    int16_t x;              /*!< X-coordinate in millimeters */
    int16_t y;              /*!< Y-coordinate in millimeters */
    int16_t speed;          /*!< Speed in cm/s (positive=approaching, negative=receding) */
    uint16_t resolution;    /*!< Resolution/quality of detection (lower is better) */
} ld2450_target_t;

/**
 * @brief Callback function type for target data updates
 */
typedef void (*ld2450_data_callback_t)(const ld2450_target_t targets[], 
                                    size_t count, void *user_ctx);

/**
 * @brief Supported baud rates for UART communication
 */
typedef enum {
    LD2450_BAUD_RATE_9600   = 0x0001, /*!< 9600 bps */
    LD2450_BAUD_RATE_19200  = 0x0002, /*!< 19200 bps */
    LD2450_BAUD_RATE_38400  = 0x0003, /*!< 38400 bps */
    LD2450_BAUD_RATE_57600  = 0x0004, /*!< 57600 bps */
    LD2450_BAUD_RATE_115200 = 0x0005, /*!< 115200 bps */
    LD2450_BAUD_RATE_230400 = 0x0006, /*!< 230400 bps */
    LD2450_BAUD_RATE_256000 = 0x0007, /*!< 256000 bps (default) */
    LD2450_BAUD_RATE_460800 = 0x0008, /*!< 460800 bps */
} ld2450_baud_rate_t;

/**
 * @brief Region filtering modes
 */
typedef enum {
    LD2450_REGION_FILTER_DISABLED = 0x0000, /*!< No region filtering */
    LD2450_REGION_FILTER_INCLUDE  = 0x0001, /*!< Only detect targets inside defined regions */
    LD2450_REGION_FILTER_EXCLUDE  = 0x0002, /*!< Only detect targets outside defined regions */
} ld2450_region_filter_mode_t;

/**
 * @brief Firmware version structure
 */
typedef struct {
    uint16_t main_version;  /*!< Main firmware version */
    uint32_t sub_version;   /*!< Sub-version */
} ld2450_firmware_version_t;

/**
 * @brief Configuration structure for driver initialization
 */
typedef struct {
    uart_port_t uart_port;      /*!< UART port number */
    uint32_t uart_baud_rate;    /*!< UART baud rate */
    int uart_tx_pin;            /*!< UART TX pin */
    int uart_rx_pin;            /*!< UART RX pin */
} ld2450_config_t;

/**
 * @brief Configuration parameters for combined operations
 */
typedef struct {
    /* Tracking mode configuration */
    bool update_tracking;         /*!< Whether to update tracking mode */
    bool multi_target;            /*!< Multi-target mode if update_tracking is true */
    
    /* Region filtering configuration */
    bool update_regions;          /*!< Whether to update region filtering */
    ld2450_region_filter_mode_t region_mode;  /*!< Region filtering mode if update_regions is true */
    int16_t regions[3][4];        /*!< Region coordinates [x1, y1, x2, y2] if update_regions is true */
    size_t region_count;          /*!< Number of regions if update_regions is true */
    
    /* Baud rate configuration */
    bool update_baud_rate;        /*!< Whether to update baud rate */
    ld2450_baud_rate_t baud_rate; /*!< Baud rate if update_baud_rate is true */
    
    /* Bluetooth configuration */
    bool update_bluetooth;        /*!< Whether to update Bluetooth status */
    bool bluetooth_enabled;       /*!< Bluetooth enabled if update_bluetooth is true */
    
    /* Firmware information (read-only) */
    uint16_t firmware_main;       /*!< Main firmware version */
    uint32_t firmware_sub;        /*!< Sub-version */
} ld2450_config_params_t;

/* Basic Driver API */

/**
 * @brief Initialize the LD2450 radar driver
 * 
 * @param[in] config Driver configuration
 * @return esp_err_t
 *         - ESP_OK: Success
 *         - ESP_ERR_INVALID_ARG: Invalid configuration
 *         - ESP_ERR_INVALID_STATE: Driver already initialized
 *         - ESP_ERR_NO_MEM: Memory allocation failed
 */
esp_err_t ld2450_init(const ld2450_config_t *config);

/**
 * @brief Deinitialize the LD2450 radar driver
 * 
 * @return esp_err_t
 *         - ESP_OK: Success
 *         - ESP_ERR_INVALID_STATE: Driver not initialized
 */
esp_err_t ld2450_deinit(void);

/**
 * @brief Register a callback for target data updates
 * 
 * @param[in] callback Callback function
 * @param[in] user_ctx User context to pass to the callback
 * @return esp_err_t
 *         - ESP_OK: Success
 *         - ESP_ERR_INVALID_STATE: Driver not initialized
 *         - ESP_ERR_INVALID_ARG: Invalid callback
 */
esp_err_t ld2450_register_data_callback(ld2450_data_callback_t callback, void *user_ctx);

/* Configuration Mode API */

/**
 * @brief Enter configuration mode
 * 
 * @return esp_err_t
 *         - ESP_OK: Success
 *         - ESP_ERR_INVALID_STATE: Not in idle state
 */
esp_err_t ld2450_enter_config_mode(void);

/**
 * @brief Exit configuration mode
 * 
 * @return esp_err_t
 *         - ESP_OK: Success
 *         - ESP_ERR_INVALID_STATE: Not in configuration mode
 */
esp_err_t ld2450_exit_config_mode(void);

/**
 * @brief Set the tracking mode
 * 
 * @param[in] multi_target true for multi-target mode, false for single-target mode
 * @return esp_err_t
 *         - ESP_OK: Success
 *         - ESP_ERR_INVALID_STATE: Not in configuration mode
 */
esp_err_t ld2450_set_tracking_mode(bool multi_target);

/**
 * @brief Get the current tracking mode
 * 
 * @param[out] multi_target Will be set to true for multi-target mode, false for single-target mode
 * @return esp_err_t
 *         - ESP_OK: Success
 *         - ESP_ERR_INVALID_STATE: Not in configuration mode
 *         - ESP_ERR_INVALID_ARG: Invalid pointer
 */
esp_err_t ld2450_get_tracking_mode(bool *multi_target);

/**
 * @brief Get the firmware version
 * 
 * @param[out] version Structure to store firmware version information
 * @return esp_err_t
 *         - ESP_OK: Success
 *         - ESP_ERR_INVALID_STATE: Not in configuration mode
 *         - ESP_ERR_INVALID_ARG: Invalid pointer
 */
esp_err_t ld2450_get_firmware_version(ld2450_firmware_version_t *version);

/**
 * @brief Set the UART baud rate
 * 
 * @param[in] baud_rate Baud rate to set
 * @return esp_err_t
 *         - ESP_OK: Success
 *         - ESP_ERR_INVALID_STATE: Not in configuration mode
 *         - ESP_ERR_INVALID_ARG: Invalid baud rate
 */
esp_err_t ld2450_set_baud_rate(ld2450_baud_rate_t baud_rate);

/**
 * @brief Restore factory settings
 * 
 * @return esp_err_t
 *         - ESP_OK: Success
 *         - ESP_ERR_INVALID_STATE: Not in configuration mode
 */
esp_err_t ld2450_restore_factory_settings(void);

/**
 * @brief Restart the radar module
 * 
 * @return esp_err_t
 *         - ESP_OK: Success
 *         - ESP_ERR_INVALID_STATE: Not in configuration mode
 */
esp_err_t ld2450_restart(void);

/**
 * @brief Enable or disable Bluetooth
 * 
 * @param[in] enable true to enable Bluetooth, false to disable
 * @return esp_err_t
 *         - ESP_OK: Success
 *         - ESP_ERR_INVALID_STATE: Not in configuration mode
 */
esp_err_t ld2450_set_bluetooth(bool enable);

/**
 * @brief Get the Bluetooth MAC address
 * 
 * @param[out] mac Buffer to store the 6-byte MAC address
 * @return esp_err_t
 *         - ESP_OK: Success
 *         - ESP_ERR_INVALID_STATE: Not in configuration mode
 *         - ESP_ERR_INVALID_ARG: Invalid pointer
 */
esp_err_t ld2450_get_mac_address(uint8_t mac[6]);

/**
 * @brief Get the region filter configuration
 * 
 * @param[out] mode Filter mode
 * @param[out] regions Array to store region coordinates [x1, y1, x2, y2]
 * @param[out] region_count Number of regions
 * @return esp_err_t
 *         - ESP_OK: Success
 *         - ESP_ERR_INVALID_STATE: Not in configuration mode
 *         - ESP_ERR_INVALID_ARG: Invalid pointer
 */
esp_err_t ld2450_get_region_filter(ld2450_region_filter_mode_t *mode, 
                                 int16_t regions[][4], 
                                 size_t *region_count);

/**
 * @brief Set the region filter configuration
 * 
 * @param[in] mode Filter mode
 * @param[in] regions Array of region coordinates [x1, y1, x2, y2]
 * @param[in] region_count Number of regions (0-3)
 * @return esp_err_t
 *         - ESP_OK: Success
 *         - ESP_ERR_INVALID_STATE: Not in configuration mode
 *         - ESP_ERR_INVALID_ARG: Invalid arguments
 */
esp_err_t ld2450_set_region_filter(ld2450_region_filter_mode_t mode,
                                 const int16_t regions[][4],
                                 size_t region_count);

/* Advanced Commands API */

/**
 * @brief Update the baud rate and handle ESP32 UART reconfiguration
 * 
 * @param[in] baud_rate New baud rate
 * @return esp_err_t
 *         - ESP_OK: Success
 *         - ESP_ERR_INVALID_ARG: Invalid baud rate
 */
esp_err_t ld2450_cmd_update_baud_rate(ld2450_baud_rate_t baud_rate);

/**
 * @brief Perform factory reset and restart
 * 
 * @return esp_err_t
 *         - ESP_OK: Success
 */
esp_err_t ld2450_cmd_factory_reset_and_restart(void);

/**
 * @brief Configure tracking mode with automatic mode switching
 * 
 * @param[in] multi_target true for multi-target, false for single-target
 * @return esp_err_t
 *         - ESP_OK: Success
 */
esp_err_t ld2450_cmd_configure_tracking(bool multi_target);

/**
 * @brief Get firmware version as a string
 * 
 * @param[out] version_str Buffer to store version string
 * @param[in] str_size Size of the buffer
 * @return esp_err_t
 *         - ESP_OK: Success
 *         - ESP_ERR_INVALID_ARG: Invalid arguments
 */
esp_err_t ld2450_cmd_get_firmware_info(char *version_str, size_t str_size);

/**
 * @brief Get Bluetooth MAC address as a formatted string
 * 
 * @param[out] mac_str Buffer to store MAC string (format: XX:XX:XX:XX:XX:XX)
 * @param[in] str_size Size of the buffer (minimum 18 bytes)
 * @return esp_err_t
 *         - ESP_OK: Success
 *         - ESP_ERR_INVALID_ARG: Invalid arguments
 */
esp_err_t ld2450_cmd_get_bluetooth_mac(char *mac_str, size_t str_size);

/**
 * @brief Configure Bluetooth functionality
 * 
 * @param[in] enable true to enable Bluetooth, false to disable
 * @return esp_err_t
 *         - ESP_OK: Success
 */
esp_err_t ld2450_cmd_configure_bluetooth(bool enable);

/**
 * @brief Set up detection zones with automatic mode switching
 * 
 * @param[in] mode Filter mode
 * @param[in] regions Array of region coordinates [x1, y1, x2, y2]
 * @param[in] region_count Number of regions (0-3)
 * @return esp_err_t
 *         - ESP_OK: Success
 *         - ESP_ERR_INVALID_ARG: Invalid arguments
 */
esp_err_t ld2450_cmd_set_detection_zones(ld2450_region_filter_mode_t mode,
                                      const int16_t regions[][4],
                                      size_t region_count);

/**
 * @brief Get current detection zones configuration
 * 
 * @param[out] mode Filter mode
 * @param[out] regions Array to store region coordinates [x1, y1, x2, y2]
 * @param[out] region_count Number of regions
 * @return esp_err_t
 *         - ESP_OK: Success
 *         - ESP_ERR_INVALID_ARG: Invalid pointers
 */
esp_err_t ld2450_cmd_get_detection_zones(ld2450_region_filter_mode_t *mode,
                                      int16_t regions[][4],
                                      size_t *region_count);

/**
 * @brief Create a circular zone as a rectangular approximation
 * 
 * @param[in] center_x X-coordinate of center (mm)
 * @param[in] center_y Y-coordinate of center (mm)
 * @param[in] radius Radius in mm
 * @param[in] include Whether to use as include or exclude zone (informational only)
 * @param[out] region_out Output region coordinates [x1, y1, x2, y2]
 * @return esp_err_t
 *         - ESP_OK: Success
 *         - ESP_ERR_INVALID_ARG: Invalid pointer
 */
esp_err_t ld2450_cmd_create_circular_zone(int16_t center_x, int16_t center_y,
                                       uint16_t radius, bool include,
                                       int16_t region_out[4]);

/**
 * @brief Check if a point lies within a circle
 * 
 * @param[in] center_x X-coordinate of circle center (mm)
 * @param[in] center_y Y-coordinate of circle center (mm)
 * @param[in] radius Circle radius in mm
 * @param[in] point_x X-coordinate of point to check (mm)
 * @param[in] point_y Y-coordinate of point to check (mm)
 * @param[out] result Will be set to true if point is in circle, false otherwise
 * @return esp_err_t
 *         - ESP_OK: Success
 *         - ESP_ERR_INVALID_ARG: Invalid pointer
 */
esp_err_t ld2450_cmd_is_point_in_circle(int16_t center_x, int16_t center_y,
                                      uint16_t radius, int16_t point_x,
                                      int16_t point_y, bool *result);

/**
 * @brief Convert target data to JSON format
 * 
 * @param[in] targets Array of target data
 * @param[in] count Number of targets
 * @param[out] json_buffer Buffer to store JSON string
 * @param[in] buffer_size Size of the buffer
 * @param[out] output_len Actual length of the JSON output
 * @return esp_err_t
 *         - ESP_OK: Success
 *         - ESP_ERR_INVALID_ARG: Invalid arguments
 *         - ESP_ERR_NO_MEM: Buffer too small
 */
esp_err_t ld2450_cmd_targets_to_json(const ld2450_target_t targets[],
                                   size_t count, char *json_buffer,
                                   size_t buffer_size, size_t *output_len);

/**
 * @brief Configure multiple settings in one operation
 * 
 * @param[in] params Configuration parameters
 * @return esp_err_t
 *         - ESP_OK: Success
 *         - ESP_ERR_INVALID_ARG: Invalid arguments
 */
esp_err_t ld2450_cmd_configure_and_restart(ld2450_config_params_t *params);

/**
 * @brief Get all radar parameters
 * 
 * @param[out] params Configuration parameters structure to fill
 * @return esp_err_t
 *         - ESP_OK: Success
 *         - ESP_ERR_INVALID_ARG: Invalid arguments
 */
esp_err_t ld2450_cmd_get_all_parameters(ld2450_config_params_t *params);

/* Utility Functions */

/**
 * @brief Calculate target speed in meters per second
 * 
 * @param[in] target Target structure
 * @return float Speed in meters per second
 */
float ld2450_calculate_speed_mps(const ld2450_target_t *target);

/**
 * @brief Calculate distance from radar to target
 * 
 * @param[in] target Target structure
 * @return uint32_t Distance in millimeters
 */
uint32_t ld2450_calculate_distance_mm(const ld2450_target_t *target);

/**
 * @brief Calculate angle to target
 * 
 * @param[in] target Target structure
 * @return float Angle in degrees (0-359)
 */
float ld2450_calculate_angle_degrees(const ld2450_target_t *target);

/**
 * @brief Calculate target statistics from parsed data
 * 
 * @param[in] targets Array of target data
 * @param[in] count Number of targets
 * @param[out] avg_x Average X coordinate (can be NULL)
 * @param[out] avg_y Average Y coordinate (can be NULL)
 * @param[out] avg_speed Average speed (can be NULL)
 * @return esp_err_t
 *         - ESP_OK: Success
 *         - ESP_ERR_INVALID_ARG: Invalid targets array
 *         - ESP_ERR_NOT_FOUND: No active targets
 */
esp_err_t ld2450_calculate_target_statistics(const ld2450_target_t targets[], size_t count,
                                          int16_t *avg_x, int16_t *avg_y, int16_t *avg_speed);

/**
 * @brief Find the closest target to reference coordinates
 * 
 * @param[in] targets Array of target data
 * @param[in] count Number of targets
 * @param[in] ref_x Reference X coordinate
 * @param[in] ref_y Reference Y coordinate
 * @param[out] closest_idx Index of closest target
 * @return esp_err_t
 *         - ESP_OK: Success
 *         - ESP_ERR_INVALID_ARG: Invalid arguments
 *         - ESP_ERR_NOT_FOUND: No active targets
 */
esp_err_t ld2450_find_closest_target(const ld2450_target_t targets[], size_t count,
                                  int16_t ref_x, int16_t ref_y, size_t *closest_idx);

/**
 * @brief Find the target with highest signal quality
 * 
 * @param[in] targets Array of target data
 * @param[in] count Number of targets
 * @param[out] best_idx Index of highest quality target
 * @return esp_err_t
 *         - ESP_OK: Success
 *         - ESP_ERR_INVALID_ARG: Invalid arguments
 *         - ESP_ERR_NOT_FOUND: No active targets
 */
esp_err_t ld2450_find_best_quality_target(const ld2450_target_t targets[], size_t count,
                                       size_t *best_idx);

#ifdef __cplusplus
}
#endif