/**
 * @file ld2450_commands.c
 * @brief Command implementation for HLK-LD2450 radar sensor
 * 
 * This file contains advanced command implementations for the HLK-LD2450 
 * radar sensor, including region configuration, tracking mode control, 
 * and other enhanced features.
 * 
 * @copyright Copyright (c) 2025 NieRVoid
 * @license MIT License
 */

#include <string.h>
#include <math.h>
#include "esp_timer.h"
#include <inttypes.h>
#include "esp_log.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "ld2450.h"
#include "ld2450_private.h"

static const char *TAG = "ld2450_cmd";

/**
 * @brief Internal helper for entering config mode
 * 
 * Ensures the radar is in configuration mode before executing commands
 * 
 * @return ESP_OK on success
 */
static esp_err_t ensure_config_mode(void)
{
    if (g_ld2450_ctx.config_mode) {
        return ESP_OK;
    }
    
    return ld2450_enter_config_mode();
}

/**
 * @brief Convert baud rate enum to actual baud rate value
 * 
 * @param baud_enum Baud rate enumeration value
 * @return uint32_t Actual baud rate in bps
 */
static uint32_t baud_enum_to_value(ld2450_baud_rate_t baud_enum)
{
    switch (baud_enum) {
        case LD2450_BAUD_RATE_9600:   return 9600;
        case LD2450_BAUD_RATE_19200:  return 19200;
        case LD2450_BAUD_RATE_38400:  return 38400;
        case LD2450_BAUD_RATE_57600:  return 57600;
        case LD2450_BAUD_RATE_115200: return 115200;
        case LD2450_BAUD_RATE_230400: return 230400;
        case LD2450_BAUD_RATE_256000: return 256000;
        case LD2450_BAUD_RATE_460800: return 460800;
        default:                      return 256000; /* Default baud rate */
    }
}

/**
 * @brief Update UART baud rate
 * 
 * Updates the ESP32 UART baud rate after changing radar baud rate.
 * 
 * @param baud_rate New baud rate value
 * @return ESP_OK on success
 */
static esp_err_t update_uart_baud_rate(uint32_t baud_rate)
{
    /* Update the configured baud rate */
    g_ld2450_ctx.uart_baud_rate = baud_rate;
    
    /* Update UART driver parameters */
    uart_config_t uart_config = {
        .baud_rate = baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    
    return uart_param_config(g_ld2450_ctx.uart_port, &uart_config);
}

/**
 * @brief Check if region filtering configuration is valid
 * 
 * Validates region coordinates and count.
 * 
 * @param mode Region filtering mode
 * @param regions Array of region coordinates
 * @param region_count Number of regions
 * @return ESP_OK if valid
 */
static esp_err_t validate_region_config(ld2450_region_filter_mode_t mode,
                                     const int16_t regions[][4],
                                     size_t region_count)
{
    /* Check mode value */
    if (mode > LD2450_REGION_FILTER_EXCLUDE) {
        ESP_LOGE(TAG, "Invalid region filter mode: %d", mode);
        return ESP_ERR_INVALID_ARG;
    }
    
    /* If disabled, no further checks needed */
    if (mode == LD2450_REGION_FILTER_DISABLED) {
        return ESP_OK;
    }
    
    /* Check region count */
    if (region_count == 0 || region_count > 3) {
        ESP_LOGE(TAG, "Invalid region count: %u (must be 1-3)", region_count);
        return ESP_ERR_INVALID_ARG;
    }
    
    /* Check if regions pointer is valid */
    if (regions == NULL) {
        ESP_LOGE(TAG, "Regions array is NULL");
        return ESP_ERR_INVALID_ARG;
    }
    
    /* Check each region for validity */
    for (size_t i = 0; i < region_count; i++) {
        /* Each region should form a valid rectangle */
        if (regions[i][0] > regions[i][2] || regions[i][1] > regions[i][3]) {
            ESP_LOGE(TAG, "Invalid region dimensions for region %u", i);
            return ESP_ERR_INVALID_ARG;
        }
        
        /* No need to check int16_t range as it's enforced by the type */
    }
    
    return ESP_OK;
}

/* Public Command Implementations */

esp_err_t ld2450_cmd_update_baud_rate(ld2450_baud_rate_t baud_rate)
{
    ESP_RETURN_ON_FALSE(baud_rate >= LD2450_BAUD_RATE_9600 && 
                      baud_rate <= LD2450_BAUD_RATE_460800, 
                      ESP_ERR_INVALID_ARG, TAG, "Invalid baud rate");
    
    esp_err_t ret;
    uint32_t new_baud_value = baud_enum_to_value(baud_rate);
    
    /* Enter config mode if not already in it */
    ret = ensure_config_mode();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enter configuration mode: %s", esp_err_to_name(ret));
        return ret;
    }
    
    /* Set the baud rate on the radar */
    ret = ld2450_set_baud_rate(baud_rate);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set baud rate: %s", esp_err_to_name(ret));
        return ret;
    }
    
    /* Restart the radar to apply the new baud rate */
    ret = ld2450_restart();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to restart radar: %s", esp_err_to_name(ret));
        return ret;
    }
    
    /* Delay to allow the radar to restart with the new baud rate */
    vTaskDelay(pdMS_TO_TICKS(500));
    
    /* Update ESP32 UART baud rate */
    ret = update_uart_baud_rate(new_baud_value);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to update ESP32 UART baud rate: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "Baud rate updated to %" PRIu32 " bps", new_baud_value);
    
    return ESP_OK;
}

esp_err_t ld2450_cmd_factory_reset_and_restart(void)
{
    esp_err_t ret;
    
    /* Enter config mode if not already in it */
    ret = ensure_config_mode();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enter configuration mode: %s", esp_err_to_name(ret));
        return ret;
    }
    
    /* Restore factory settings */
    ret = ld2450_restore_factory_settings();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to restore factory settings: %s", esp_err_to_name(ret));
        return ret;
    }
    
    /* Restart the radar to apply changes */
    ret = ld2450_restart();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to restart radar: %s", esp_err_to_name(ret));
        return ret;
    }
    
    /* Delay to allow restart */
    vTaskDelay(pdMS_TO_TICKS(500));
    
    /* Update UART baud rate to factory default (256000) */
    ret = update_uart_baud_rate(256000);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to update ESP32 UART baud rate: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "Factory reset and restart completed");
    
    return ESP_OK;
}

esp_err_t ld2450_cmd_configure_tracking(bool multi_target)
{
    esp_err_t ret;
    
    /* Enter config mode if not already in it */
    ret = ensure_config_mode();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enter configuration mode: %s", esp_err_to_name(ret));
        return ret;
    }
    
    /* Set tracking mode */
    ret = ld2450_set_tracking_mode(multi_target);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure tracking mode: %s", esp_err_to_name(ret));
        return ret;
    }
    
    /* Exit config mode */
    ret = ld2450_exit_config_mode();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to exit configuration mode: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "Tracking mode set to %s", multi_target ? "multi-target" : "single-target");
    
    return ESP_OK;
}

esp_err_t ld2450_cmd_get_firmware_info(char *version_str, size_t str_size)
{
    ESP_RETURN_ON_FALSE(version_str != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid version string buffer");
    ESP_RETURN_ON_FALSE(str_size > 0, ESP_ERR_INVALID_ARG, TAG, "Invalid buffer size");
    
    esp_err_t ret;
    ld2450_firmware_version_t version;
    
    /* Enter config mode if not already in it */
    ret = ensure_config_mode();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enter configuration mode: %s", esp_err_to_name(ret));
        return ret;
    }
    
    /* Get firmware version */
    ret = ld2450_get_firmware_version(&version);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get firmware version: %s", esp_err_to_name(ret));
        return ret;
    }
    
    /* Format version string */
    snprintf(version_str, str_size, "v%u.%" PRIu32, version.main_version, version.sub_version);
    
    /* Exit config mode */
    ret = ld2450_exit_config_mode();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to exit configuration mode: %s", esp_err_to_name(ret));
        return ret;
    }
    
    /* Log firmware version retrieval success */
    ESP_LOGI(TAG, "Firmware version retrieved successfully: %s", version_str);
    
    return ESP_OK;
}

esp_err_t ld2450_cmd_get_bluetooth_mac(char *mac_str, size_t str_size)
{
    ESP_RETURN_ON_FALSE(mac_str != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid MAC string buffer");
    ESP_RETURN_ON_FALSE(str_size >= 18, ESP_ERR_INVALID_ARG, TAG, "Buffer too small for MAC address");
    
    esp_err_t ret;
    uint8_t mac[6];
    
    /* Enter config mode if not already in it */
    ret = ensure_config_mode();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enter configuration mode: %s", esp_err_to_name(ret));
        return ret;
    }
    
    /* Get MAC address */
    ret = ld2450_get_mac_address(mac);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get MAC address: %s", esp_err_to_name(ret));
        return ret;
    }
    
    /* Format MAC string as XX:XX:XX:XX:XX:XX */
    snprintf(mac_str, str_size, "%02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    
    /* Exit config mode */
    ret = ld2450_exit_config_mode();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to exit configuration mode: %s", esp_err_to_name(ret));
        return ret;
    }
    
    return ESP_OK;
}

esp_err_t ld2450_cmd_configure_bluetooth(bool enable)
{
    esp_err_t ret;
    
    /* Enter config mode if not already in it */
    ret = ensure_config_mode();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enter configuration mode: %s", esp_err_to_name(ret));
        return ret;
    }
    
    /* Configure Bluetooth */
    ret = ld2450_set_bluetooth(enable);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure Bluetooth: %s", esp_err_to_name(ret));
        return ret;
    }
    
    /* Restart the radar to apply changes */
    ret = ld2450_restart();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to restart radar: %s", esp_err_to_name(ret));
        return ret;
    }
    
    /* Delay to allow restart */
    vTaskDelay(pdMS_TO_TICKS(500));
    
    ESP_LOGI(TAG, "Bluetooth %s and radar restarted", enable ? "enabled" : "disabled");
    
    return ESP_OK;
}

esp_err_t ld2450_cmd_set_detection_zones(ld2450_region_filter_mode_t mode,
                                      const int16_t regions[][4],
                                      size_t region_count)
{
    /* Validate parameters */
    esp_err_t ret = validate_region_config(mode, regions, region_count);
    if (ret != ESP_OK) {
        return ret;
    }
    
    /* Enter config mode if not already in it */
    ret = ensure_config_mode();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enter configuration mode: %s", esp_err_to_name(ret));
        return ret;
    }
    
    /* Set region filter */
    ret = ld2450_set_region_filter(mode, regions, region_count);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set region filter: %s", esp_err_to_name(ret));
        return ret;
    }
    
    /* Exit config mode */
    ret = ld2450_exit_config_mode();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to exit configuration mode: %s", esp_err_to_name(ret));
        return ret;
    }
    
    const char *mode_str;
    switch (mode) {
        case LD2450_REGION_FILTER_DISABLED: mode_str = "disabled"; break;
        case LD2450_REGION_FILTER_INCLUDE: mode_str = "include-only"; break;
        case LD2450_REGION_FILTER_EXCLUDE: mode_str = "exclude"; break;
        default: mode_str = "unknown"; break;
    }
    
    ESP_LOGI(TAG, "Detection zones set to %s mode with %zu regions", mode_str, region_count);
    
    return ESP_OK;
}

esp_err_t ld2450_cmd_get_detection_zones(ld2450_region_filter_mode_t *mode,
                                      int16_t regions[][4],
                                      size_t *region_count)
{
    ESP_RETURN_ON_FALSE(mode != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid mode pointer");
    ESP_RETURN_ON_FALSE(regions != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid regions array");
    ESP_RETURN_ON_FALSE(region_count != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid region_count pointer");
    
    esp_err_t ret;
    
    /* Enter config mode if not already in it */
    ret = ensure_config_mode();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enter configuration mode: %s", esp_err_to_name(ret));
        return ret;
    }
    
    /* Get region filter settings */
    ret = ld2450_get_region_filter(mode, regions, region_count);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get region filter: %s", esp_err_to_name(ret));
        return ret;
    }
    
    /* Exit config mode */
    ret = ld2450_exit_config_mode();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to exit configuration mode: %s", esp_err_to_name(ret));
        return ret;
    }
    
    return ESP_OK;
}

esp_err_t ld2450_cmd_create_circular_zone(int16_t center_x, int16_t center_y,
                                       uint16_t radius, bool include,
                                       int16_t region_out[4])
{
    ESP_RETURN_ON_FALSE(region_out != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid region output buffer");
    
    /* Calculate coordinates for bounding box around circle */
    int16_t x_min = center_x - radius;
    int16_t x_max = center_x + radius;
    int16_t y_min = center_y - radius;
    int16_t y_max = center_y + radius;
    
    /* Store in region format (x1, y1, x2, y2) */
    region_out[0] = x_min;
    region_out[1] = y_min;
    region_out[2] = x_max;
    region_out[3] = y_max;
    
    return ESP_OK;
}

esp_err_t ld2450_cmd_is_point_in_circle(int16_t center_x, int16_t center_y,
                                      uint16_t radius, int16_t point_x,
                                      int16_t point_y, bool *result)
{
    ESP_RETURN_ON_FALSE(result != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid result pointer");
    
    /* Calculate squared distance from point to center */
    int32_t dx = point_x - center_x;
    int32_t dy = point_y - center_y;
    int32_t distance_sq = dx * dx + dy * dy;
    
    /* Compare with squared radius */
    int32_t radius_sq = (int32_t)radius * radius;
    
    *result = (distance_sq <= radius_sq);
    
    return ESP_OK;
}

esp_err_t ld2450_cmd_targets_to_json(const ld2450_target_t targets[],
                                   size_t count, char *json_buffer,
                                   size_t buffer_size, size_t *output_len)
{
    ESP_RETURN_ON_FALSE(targets != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid targets array");
    ESP_RETURN_ON_FALSE(json_buffer != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid JSON buffer");
    ESP_RETURN_ON_FALSE(output_len != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid output_len pointer");
    ESP_RETURN_ON_FALSE(buffer_size > 0, ESP_ERR_INVALID_ARG, TAG, "Invalid buffer size");
    
    /* Start JSON array */
    int written = snprintf(json_buffer, buffer_size, 
        "{\"timestamp_ms\":%" PRIu64 ",\"targets\":[", 
        (uint64_t)(esp_timer_get_time() / 1000));
                        
    if (written < 0 || written >= buffer_size) {
        return ESP_ERR_NO_MEM;
    }
    
    char *cur_pos = json_buffer + written;
    size_t remaining = buffer_size - written;
    
    /* Format each active target */
    size_t active_count = 0;
    for (size_t i = 0; i < count; i++) {
        if (targets[i].active) {
            /* Add comma between items */
            if (active_count > 0) {
                written = snprintf(cur_pos, remaining, ",");
                if (written < 0 || written >= remaining) {
                    return ESP_ERR_NO_MEM;
                }
                cur_pos += written;
                remaining -= written;
            }
            
            /* Calculate additional information */
            float distance = sqrtf(targets[i].x * targets[i].x + 
                                  targets[i].y * targets[i].y) / 1000.0f; /* m */
            
            float angle = atan2f(targets[i].x, targets[i].y) * 180.0f / M_PI;
            if (angle < 0) {
                angle += 360.0f;
            }
            
            float speed_mps = targets[i].speed / 100.0f; /* m/s */
            
            /* Format target data with additional information */
            written = snprintf(cur_pos, remaining, 
                            "{\"id\":%zu,"
                             "\"x_mm\":%d,"
                             "\"y_mm\":%d,"
                             "\"speed_cmps\":%d,"
                             "\"distance_m\":%.2f,"
                             "\"angle_deg\":%.1f,"
                             "\"speed_mps\":%.2f,"
                             "\"resolution\":%u}",
                            i, 
                            targets[i].x, 
                            targets[i].y, 
                            targets[i].speed,
                            distance,
                            angle,
                            speed_mps,
                            targets[i].resolution);
            
            if (written < 0 || written >= remaining) {
                return ESP_ERR_NO_MEM;
            }
            
            cur_pos += written;
            remaining -= written;
            active_count++;
        }
    }
    
    /* Close JSON array and object */
    written = snprintf(cur_pos, remaining, "]}");
    if (written < 0 || written >= remaining) {
        return ESP_ERR_NO_MEM;
    }
    
    cur_pos += written;
    *output_len = cur_pos - json_buffer;
    
    return ESP_OK;
}

esp_err_t ld2450_cmd_configure_and_restart(ld2450_config_params_t *params)
{
    ESP_RETURN_ON_FALSE(params != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid params");
    
    esp_err_t ret;
    bool needs_restart = false;
    
    /* Enter config mode if not already in it */
    ret = ensure_config_mode();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enter configuration mode: %s", esp_err_to_name(ret));
        return ret;
    }
    
    /* Configure tracking mode if requested */
    if (params->update_tracking) {
        ret = ld2450_set_tracking_mode(params->multi_target);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to set tracking mode: %s", esp_err_to_name(ret));
            return ret;
        }
        needs_restart = true;
    }
    
    /* Configure Bluetooth if requested */
    if (params->update_bluetooth) {
        ret = ld2450_set_bluetooth(params->bluetooth_enabled);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to configure Bluetooth: %s", esp_err_to_name(ret));
            return ret;
        }
        needs_restart = true;
    }
    
    /* Configure baud rate if requested */
    if (params->update_baud_rate) {
        ret = ld2450_set_baud_rate(params->baud_rate);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to set baud rate: %s", esp_err_to_name(ret));
            return ret;
        }
        needs_restart = true;
    }
    
    /* Configure region filtering if requested */
    if (params->update_regions) {
        ret = ld2450_set_region_filter(params->region_mode, params->regions, params->region_count);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to set region filter: %s", esp_err_to_name(ret));
            return ret;
        }
        needs_restart = true;
    }
    
    /* Restart the radar if any changes were made */
    if (needs_restart) {
        ret = ld2450_restart();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to restart radar: %s", esp_err_to_name(ret));
            return ret;
        }
        
        /* Delay to allow restart */
        vTaskDelay(pdMS_TO_TICKS(500));
        
        /* Update local UART baud rate if it was changed */
        if (params->update_baud_rate) {
            uint32_t baud_value = baud_enum_to_value(params->baud_rate);
            ret = update_uart_baud_rate(baud_value);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to update ESP32 UART baud rate: %s", esp_err_to_name(ret));
                return ret;
            }
        }
        
        ESP_LOGI(TAG, "Radar configuration updated and device restarted");
    } else {
        /* No changes, just exit config mode */
        ret = ld2450_exit_config_mode();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to exit configuration mode: %s", esp_err_to_name(ret));
            return ret;
        }
        
        ESP_LOGI(TAG, "No configuration changes made");
    }
    
    return ESP_OK;
}

esp_err_t ld2450_cmd_get_all_parameters(ld2450_config_params_t *params)
{
    ESP_RETURN_ON_FALSE(params != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid params");
    
    esp_err_t ret;
    bool tracking_mode;
    
    /* Initialize params */
    memset(params, 0, sizeof(ld2450_config_params_t));
    
    /* Enter config mode if not already in it */
    ret = ensure_config_mode();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enter configuration mode: %s", esp_err_to_name(ret));
        return ret;
    }
    
    /* Get tracking mode */
    ret = ld2450_get_tracking_mode(&tracking_mode);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get tracking mode: %s", esp_err_to_name(ret));
        return ret;
    }
    params->multi_target = tracking_mode;
    params->update_tracking = true;
    
    /* Get region filter settings */
    ret = ld2450_get_region_filter(&params->region_mode, params->regions, &params->region_count);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get region filter: %s", esp_err_to_name(ret));
        /* Continue despite error */
    } else {
        params->update_regions = true;
    }
    
    /* We can't directly query the current baud rate or Bluetooth settings */
    params->update_baud_rate = false;
    params->update_bluetooth = false;
    
    /* Get firmware version */
    ld2450_firmware_version_t version;
    ret = ld2450_get_firmware_version(&version);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get firmware version: %s", esp_err_to_name(ret));
        /* Continue despite error */
    } else {
        params->firmware_main = version.main_version;
        params->firmware_sub = version.sub_version;
    }
    
    /* Exit config mode */
    ret = ld2450_exit_config_mode();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to exit configuration mode: %s", esp_err_to_name(ret));
        return ret;
    }
    
    return ESP_OK;
}