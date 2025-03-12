/**
 * @file ld2450_parser.c
 * @brief Parser implementation for HLK-LD2450 radar sensor data
 * 
 * This file contains specialized functions for parsing data frames from 
 * the HLK-LD2450 radar sensor with optimized performance for real-time 
 * applications.
 * 
 * @copyright Copyright (c) 2025 NieRVoid
 * @license MIT License
 */

#include <string.h>
#include <math.h>
#include "esp_log.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "ld2450.h"
#include "ld2450_private.h"

/* Define M_PI if not already defined */
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static const char *TAG = "ld2450_parser";

/**
 * @brief Parse a target data segment from a data frame
 * 
 * Extracts coordinate, speed, and resolution information for a single target
 * from the data frame.
 * 
 * @param data Pointer to target data segment (8 bytes)
 * @param target Pointer to target structure to fill
 * @return bool True if target is active (non-zero data)
 */
static bool parse_target_data(const uint8_t *data, ld2450_target_t *target)
{
    /* Check if this target is active (non-zero data) */
    bool active = false;
    for (int i = 0; i < LD2450_TARGET_DATA_SIZE; i++) {
        if (data[i] != 0) {
            active = true;
            break;
        }
    }
    
    /* If not active, just mark as inactive and return */
    if (!active) {
        target->active = false;
        return false;
    }
    
    /* Extract raw values (little-endian) */
    uint16_t x_raw = data[0] | (data[1] << 8);
    uint16_t y_raw = data[2] | (data[3] << 8);
    uint16_t speed_raw = data[4] | (data[5] << 8);
    uint16_t resolution = data[6] | (data[7] << 8);
    
    /**
     * Adjust values based on protocol:
     * - For x and y: MSB indicates sign (1 positive, 0 negative)
     * - For speed: MSB indicates direction (1 positive, 0 negative)
     */
    int16_t x = (x_raw & 0x8000) ? (x_raw & 0x7FFF) : -(x_raw & 0x7FFF);
    int16_t y = (y_raw & 0x8000) ? (y_raw & 0x7FFF) : -(y_raw & 0x7FFF);
    int16_t speed = (speed_raw & 0x8000) ? (speed_raw & 0x7FFF) : -(speed_raw & 0x7FFF);
    
    /* Store parsed values */
    target->x = x;
    target->y = y;
    target->speed = speed;
    target->resolution = resolution;
    target->active = true;
    
    return true;
}

/**
 * @brief Validate data frame header format
 * 
 * Checks if the provided buffer contains a valid data frame header.
 * 
 * @param buffer Pointer to buffer containing potential header
 * @param size Buffer size
 * @return bool True if valid header is found
 */
bool ld2450_validate_data_frame_header(const uint8_t *buffer, size_t size)
{
    if (buffer == NULL || size < 4) {
        return false;
    }
    
    /* Data frame header should be AA FF 03 00 */
    return (buffer[0] == 0xAA && 
            buffer[1] == 0xFF && 
            buffer[2] == 0x03 && 
            buffer[3] == 0x00);
}

/**
 * @brief Validate data frame tail format
 * 
 * Checks if the provided buffer position contains a valid data frame tail.
 * 
 * @param buffer Pointer to buffer containing potential tail
 * @return bool True if valid tail is found
 */
bool ld2450_validate_data_frame_tail(const uint8_t *buffer)
{
    if (buffer == NULL) {
        return false;
    }
    
    /* Data frame tail should be 55 CC */
    return (buffer[0] == 0x55 && buffer[1] == 0xCC);
}

/**
 * @brief Parse a complete data frame buffer
 * 
 * Processes a full data frame and extracts target information.
 * This is an optimized version of the function in ld2450.c.
 * 
 * @param buffer Pointer to data frame buffer
 * @param length Buffer length
 * @param targets Array to store target data (must accommodate LD2450_MAX_TARGETS)
 * @param target_count Pointer to store number of active targets
 * @return esp_err_t ESP_OK on success
 */
esp_err_t ld2450_parse_data_frame(const uint8_t *buffer, size_t length,
                                ld2450_target_t targets[], size_t *target_count)
{
    ESP_RETURN_ON_FALSE(buffer != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid buffer");
    ESP_RETURN_ON_FALSE(targets != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid targets array");
    ESP_RETURN_ON_FALSE(target_count != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid target_count pointer");
    ESP_RETURN_ON_FALSE(length >= LD2450_DATA_FRAME_HEADER_SIZE + 
                      (LD2450_MAX_TARGETS * LD2450_TARGET_DATA_SIZE) + 
                      LD2450_DATA_FRAME_TAIL_SIZE, ESP_ERR_INVALID_SIZE, TAG, "Buffer too small");
    
    /* Validate data frame header */
    if (!ld2450_validate_data_frame_header(buffer, length)) {
        ESP_LOGW(TAG, "Invalid data frame header");
        return ESP_ERR_INVALID_STATE;
    }
    
    /* Validate data frame tail */
    const uint8_t *tail_pos = buffer + LD2450_DATA_FRAME_HEADER_SIZE + 
                            (LD2450_MAX_TARGETS * LD2450_TARGET_DATA_SIZE);
    if (!ld2450_validate_data_frame_tail(tail_pos)) {
        ESP_LOGW(TAG, "Invalid data frame tail");
        return ESP_ERR_INVALID_STATE;
    }
    
    /* Process target data */
    size_t active_count = 0;
    const uint8_t *target_data = buffer + LD2450_DATA_FRAME_HEADER_SIZE;
    
    for (int i = 0; i < LD2450_MAX_TARGETS; i++) {
        /* Parse target data segment */
        if (parse_target_data(target_data + (i * LD2450_TARGET_DATA_SIZE), &targets[i])) {
            active_count++;
        }
    }
    
    /* Update target count */
    *target_count = active_count;
    
    return ESP_OK;
}

/**
 * @brief Find a data frame in a buffer
 * 
 * Scans a buffer for a valid data frame and returns its position.
 * Useful for recovering synchronization after errors.
 * 
 * @param buffer Pointer to buffer to scan
 * @param length Buffer length
 * @param frame_pos Pointer to store position of frame start if found
 * @return esp_err_t ESP_OK if frame found, ESP_ERR_NOT_FOUND otherwise
 */
esp_err_t ld2450_find_data_frame(const uint8_t *buffer, size_t length, size_t *frame_pos)
{
    ESP_RETURN_ON_FALSE(buffer != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid buffer");
    ESP_RETURN_ON_FALSE(frame_pos != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid frame_pos pointer");
    
    /* Minimum length to consider */
    if (length < 4) {
        return ESP_ERR_NOT_FOUND;
    }
    
    /* Scan for header pattern */
    for (size_t i = 0; i <= length - 4; i++) {
        if (buffer[i] == 0xAA && 
            buffer[i+1] == 0xFF && 
            buffer[i+2] == 0x03 && 
            buffer[i+3] == 0x00) {
            
            *frame_pos = i;
            return ESP_OK;
        }
    }
    
    return ESP_ERR_NOT_FOUND;
}

/**
 * @brief Parse command frame header
 * 
 * Validates the command frame header format.
 * 
 * @param buffer Pointer to buffer containing header
 * @param size Buffer size
 * @return bool True if valid header is found
 */
bool ld2450_parse_cmd_frame_header(const uint8_t *buffer, size_t size)
{
    if (buffer == NULL || size < LD2450_FRAME_HEADER_SIZE) {
        return false;
    }
    
    /* Command frame header should be FD FC FB FA */
    return (buffer[0] == 0xFD && 
            buffer[1] == 0xFC && 
            buffer[2] == 0xFB && 
            buffer[3] == 0xFA);
}

/**
 * @brief Parse command frame tail
 * 
 * Validates the command frame tail format.
 * 
 * @param buffer Pointer to buffer containing tail
 * @param size Buffer size
 * @return bool True if valid tail is found
 */
bool ld2450_parse_cmd_frame_tail(const uint8_t *buffer, size_t size)
{
    if (buffer == NULL || size < LD2450_FRAME_TAIL_SIZE) {
        return false;
    }
    
    /* Command frame tail should be 04 03 02 01 */
    return (buffer[0] == 0x04 && 
            buffer[1] == 0x03 && 
            buffer[2] == 0x02 && 
            buffer[3] == 0x01);
}

/**
 * @brief Parse ACK data from response frame
 * 
 * Extracts and validates ACK data from a command response frame.
 * 
 * @param buffer Pointer to complete frame buffer
 * @param length Buffer length
 * @param cmd Expected command word
 * @param ack_data Buffer to store ACK data
 * @param ack_len Pointer to store ACK data length
 * @return esp_err_t ESP_OK on success
 */
esp_err_t ld2450_parse_ack_frame(const uint8_t *buffer, size_t length, uint16_t cmd,
                               uint8_t *ack_data, uint16_t *ack_len)
{
    ESP_RETURN_ON_FALSE(buffer != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid buffer");
    ESP_RETURN_ON_FALSE(ack_data != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid ack_data buffer");
    ESP_RETURN_ON_FALSE(ack_len != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid ack_len pointer");
    
    /* Check minimum frame size */
    if (length < LD2450_FRAME_HEADER_SIZE + LD2450_DATA_LENGTH_SIZE + 
               LD2450_CMD_WORD_SIZE + LD2450_FRAME_TAIL_SIZE) {
        ESP_LOGW(TAG, "ACK frame too small");
        return ESP_ERR_INVALID_SIZE;
    }
    
    /* Validate header */
    if (!ld2450_parse_cmd_frame_header(buffer, length)) {
        ESP_LOGW(TAG, "Invalid ACK frame header");
        return ESP_ERR_INVALID_STATE;
    }
    
    /* Extract data length */
    uint16_t data_length = buffer[LD2450_FRAME_HEADER_SIZE] | 
                         (buffer[LD2450_FRAME_HEADER_SIZE + 1] << 8);
    
    /* Validate total frame length */
    if (length < LD2450_FRAME_HEADER_SIZE + LD2450_DATA_LENGTH_SIZE + 
               data_length + LD2450_FRAME_TAIL_SIZE) {
        ESP_LOGW(TAG, "ACK frame incomplete");
        return ESP_ERR_INVALID_SIZE;
    }
    
    /* Extract command word */
    uint16_t received_cmd = buffer[LD2450_FRAME_HEADER_SIZE + LD2450_DATA_LENGTH_SIZE] | 
                          (buffer[LD2450_FRAME_HEADER_SIZE + LD2450_DATA_LENGTH_SIZE + 1] << 8);
    
    /* Check if received command matches expected command */
    if ((received_cmd & 0x00FF) != (cmd & 0x00FF)) {
        ESP_LOGW(TAG, "ACK command mismatch: expected 0x%04x, got 0x%04x", cmd, received_cmd);
        return ESP_ERR_INVALID_RESPONSE;
    }
    
    /* Calculate ACK data length */
    uint16_t received_ack_len = data_length - LD2450_CMD_WORD_SIZE;
    if (received_ack_len > LD2450_MAX_RETURN_VALUE_SIZE) {
        ESP_LOGW(TAG, "ACK data too long: %d bytes", received_ack_len);
        received_ack_len = LD2450_MAX_RETURN_VALUE_SIZE;
    }
    
    /* Copy ACK data */
    if (received_ack_len > 0) {
        memcpy(ack_data, 
               &buffer[LD2450_FRAME_HEADER_SIZE + LD2450_DATA_LENGTH_SIZE + LD2450_CMD_WORD_SIZE], 
               received_ack_len);
    }
    
    /* Validate tail */
    const uint8_t *tail_pos = buffer + LD2450_FRAME_HEADER_SIZE + 
                            LD2450_DATA_LENGTH_SIZE + data_length;
    if (!ld2450_parse_cmd_frame_tail(tail_pos, LD2450_FRAME_TAIL_SIZE)) {
        ESP_LOGW(TAG, "Invalid ACK frame tail");
        return ESP_ERR_INVALID_STATE;
    }
    
    /* Update ACK length */
    *ack_len = received_ack_len;
    
    return ESP_OK;
}

/**
 * @brief Extract target statistics from parsed data
 * 
 * Calculates statistics like average position or speed from 
 * multiple target readings. Useful for filtering and tracking.
 * 
 * @param targets Array of target data
 * @param count Number of targets
 * @param avg_x Pointer to store average X coordinate
 * @param avg_y Pointer to store average Y coordinate
 * @param avg_speed Pointer to store average speed
 * @return esp_err_t ESP_OK on success
 */
esp_err_t ld2450_calculate_target_statistics(const ld2450_target_t targets[], size_t count,
    int16_t *avg_x, int16_t *avg_y, int16_t *avg_speed)
{
ESP_RETURN_ON_FALSE(targets != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid targets array");

/* Count active targets */
size_t active_count = 0;
int32_t sum_x = 0;
int32_t sum_y = 0;
int32_t sum_speed = 0;

for (size_t i = 0; i < count; i++) {
if (targets[i].active) {
sum_x += targets[i].x;
sum_y += targets[i].y;
sum_speed += targets[i].speed;
active_count++;
}
}

/* Check if we have any active targets */
if (active_count == 0) {
if (avg_x) *avg_x = 0;
if (avg_y) *avg_y = 0;
if (avg_speed) *avg_speed = 0;
return ESP_ERR_NOT_FOUND;
}

/* Calculate averages */
if (avg_x) *avg_x = (int16_t)(sum_x / active_count);
if (avg_y) *avg_y = (int16_t)(sum_y / active_count);
if (avg_speed) *avg_speed = (int16_t)(sum_speed / active_count);

return ESP_OK;
}

/**
* @brief Find the closest target to reference coordinates
* 
* Identifies the target that is closest to the specified reference point.
* 
* @param targets Array of target data
* @param count Number of targets
* @param ref_x Reference X coordinate
* @param ref_y Reference Y coordinate
* @param closest_idx Pointer to store index of closest target
* @return esp_err_t ESP_OK on success
*/
esp_err_t ld2450_find_closest_target(const ld2450_target_t targets[], size_t count,
int16_t ref_x, int16_t ref_y, size_t *closest_idx)
{
ESP_RETURN_ON_FALSE(targets != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid targets array");
ESP_RETURN_ON_FALSE(closest_idx != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid closest_idx pointer");

int32_t min_distance_sq = INT32_MAX;
bool found = false;

for (size_t i = 0; i < count; i++) {
if (targets[i].active) {
/* Calculate squared distance to avoid sqrt for performance */
int32_t dx = targets[i].x - ref_x;
int32_t dy = targets[i].y - ref_y;
int32_t distance_sq = dx * dx + dy * dy;

if (distance_sq < min_distance_sq) {
min_distance_sq = distance_sq;
*closest_idx = i;
found = true;
}
}
}

return found ? ESP_OK : ESP_ERR_NOT_FOUND;
}

/**
* @brief Extract target with highest signal quality
* 
* In HLK-LD2450, the resolution can be used as an indicator of signal quality.
* This function finds the target with the best quality.
* 
* @param targets Array of target data
* @param count Number of targets
* @param best_idx Pointer to store index of highest quality target
* @return esp_err_t ESP_OK on success
*/
esp_err_t ld2450_find_best_quality_target(const ld2450_target_t targets[], size_t count, 
 size_t *best_idx)
{
ESP_RETURN_ON_FALSE(targets != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid targets array");
ESP_RETURN_ON_FALSE(best_idx != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid best_idx pointer");

uint16_t best_resolution = UINT16_MAX;
bool found = false;

for (size_t i = 0; i < count; i++) {
if (targets[i].active) {
/* Lower resolution value indicates higher quality */
if (targets[i].resolution < best_resolution) {
best_resolution = targets[i].resolution;
*best_idx = i;
found = true;
}
}
}

return found ? ESP_OK : ESP_ERR_NOT_FOUND;
}

/**
* @brief Calculate target speed in meters per second
* 
* Convert raw speed value to m/s.
* 
* @param target Target structure
* @return float Speed in meters per second
*/
float ld2450_calculate_speed_mps(const ld2450_target_t *target)
{
if (target == NULL || !target->active) {
return 0.0f;
}

/* Speed is in cm/s, convert to m/s */
return (float)target->speed / 100.0f;
}

/**
* @brief Calculate distance from radar to target
* 
* Calculate the direct distance to target in mm.
* 
* @param target Target structure
* @return uint32_t Distance in mm
*/
uint32_t ld2450_calculate_distance_mm(const ld2450_target_t *target)
{
if (target == NULL || !target->active) {
return 0;
}

/* Calculate distance using Pythagorean theorem */
int32_t x = (int32_t)target->x;
int32_t y = (int32_t)target->y;

/* sqrt(x² + y²) */
return (uint32_t)sqrt(x * x + y * y);
}

/**
* @brief Calculate angle to target
* 
* Calculate the angle from radar to target in degrees.
* 
* @param target Target structure
* @return float Angle in degrees (0-359)
*/
float ld2450_calculate_angle_degrees(const ld2450_target_t *target)
{
if (target == NULL || !target->active) {
return 0.0f;
}

/* Calculate angle using atan2 */
float angle = atan2f(target->x, target->y) * 180.0f / M_PI;

/* Convert to 0-359 range */
if (angle < 0) {
angle += 360.0f;
}

return angle;
}

/**
* @brief Format target data as JSON string
* 
* Creates a JSON representation of target data for easy integration
* with other systems.
* 
* @param targets Array of target data
* @param count Number of targets
* @param buffer Output buffer
* @param buffer_size Buffer size
* @return esp_err_t ESP_OK on success
*/
esp_err_t ld2450_format_as_json(const ld2450_target_t targets[], size_t count,
char *buffer, size_t buffer_size)
{
ESP_RETURN_ON_FALSE(targets != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid targets array");
ESP_RETURN_ON_FALSE(buffer != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid buffer");
ESP_RETURN_ON_FALSE(buffer_size > 0, ESP_ERR_INVALID_ARG, TAG, "Invalid buffer size");

/* Start JSON array */
int written = snprintf(buffer, buffer_size, "{\"targets\":[");
if (written < 0 || written >= buffer_size) {
return ESP_ERR_NO_MEM;
}

char *cur_pos = buffer + written;
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

/* Format target data */
written = snprintf(cur_pos, remaining, 
"{\"id\":%zu,\"x\":%d,\"y\":%d,\"speed\":%d,\"resolution\":%u}",
i, targets[i].x, targets[i].y, 
targets[i].speed, targets[i].resolution);

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

return ESP_OK;
}

/**
* @brief Validate raw data buffer for protocol conformity
* 
* Advanced data validation beyond basic frame checks.
* 
* @param buffer Buffer to validate
* @param length Buffer length
* @return esp_err_t ESP_OK if valid
*/
esp_err_t ld2450_validate_data_buffer(const uint8_t *buffer, size_t length)
{
ESP_RETURN_ON_FALSE(buffer != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid buffer");

/* Check minimum length */
if (length < LD2450_DATA_FRAME_HEADER_SIZE + LD2450_DATA_FRAME_TAIL_SIZE) {
return ESP_ERR_INVALID_SIZE;
}

/* Validate header */
if (!ld2450_validate_data_frame_header(buffer, length)) {
return ESP_ERR_INVALID_STATE;
}

/* Check expected length based on protocol */
size_t expected_length = LD2450_DATA_FRAME_HEADER_SIZE + 
(LD2450_MAX_TARGETS * LD2450_TARGET_DATA_SIZE) + 
LD2450_DATA_FRAME_TAIL_SIZE;

if (length != expected_length) {
ESP_LOGW(TAG, "Invalid data frame length: %u, expected %u", length, expected_length);
return ESP_ERR_INVALID_SIZE;
}

/* Validate tail */
const uint8_t *tail_pos = buffer + length - LD2450_DATA_FRAME_TAIL_SIZE;
if (!ld2450_validate_data_frame_tail(tail_pos)) {
return ESP_ERR_INVALID_STATE;
}

return ESP_OK;
}