/**
 * @file ld2450_private.h
 * @brief Private definitions for the LD2450 driver
 * 
 * This file contains internal definitions and structures used by the driver
 * implementation. These should not be used directly by applications.
 * 
 * @copyright Copyright (c) 2025 NieRVoid
 * @license MIT License
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "ld2450.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Protocol constants
 */
#define LD2450_FRAME_HEADER_SIZE      4       /*!< Size of the frame header in bytes */
#define LD2450_FRAME_TAIL_SIZE        4       /*!< Size of the frame tail in bytes */
#define LD2450_DATA_LENGTH_SIZE       2       /*!< Size of the data length field in bytes */
#define LD2450_CMD_WORD_SIZE          2       /*!< Size of the command word in bytes */

#define LD2450_MAX_CMD_VALUE_SIZE     26      /*!< Maximum size of command value in bytes (for region filter) */
#define LD2450_MAX_RETURN_VALUE_SIZE  26      /*!< Maximum size of return value in bytes (for region filter) */

#define LD2450_MAX_CMD_FRAME_SIZE     (LD2450_FRAME_HEADER_SIZE + \
                                     LD2450_DATA_LENGTH_SIZE + \
                                     LD2450_CMD_WORD_SIZE + \
                                     LD2450_MAX_CMD_VALUE_SIZE + \
                                     LD2450_FRAME_TAIL_SIZE)

#define LD2450_MAX_ACK_FRAME_SIZE     (LD2450_FRAME_HEADER_SIZE + \
                                     LD2450_DATA_LENGTH_SIZE + \
                                     LD2450_CMD_WORD_SIZE + \
                                     LD2450_MAX_RETURN_VALUE_SIZE + \
                                     LD2450_FRAME_TAIL_SIZE)

#define LD2450_DATA_FRAME_HEADER      0xFFAA  /*!< Data frame header (little-endian) */
#define LD2450_DATA_FRAME_TAIL        0xCC55  /*!< Data frame tail (little-endian) */
#define LD2450_TARGET_DATA_SIZE       8       /*!< Size of one target's data in bytes */
#define LD2450_DATA_FRAME_HEADER_SIZE 4       /*!< Size of the data frame header in bytes (header + type) */
#define LD2450_DATA_FRAME_TAIL_SIZE   2       /*!< Size of the data frame tail in bytes */

#define LD2450_MAX_DATA_FRAME_SIZE    (LD2450_DATA_FRAME_HEADER_SIZE + \
                                     (LD2450_MAX_TARGETS * LD2450_TARGET_DATA_SIZE) + \
                                     LD2450_DATA_FRAME_TAIL_SIZE)

/**
 * @brief Command frame header (FD FC FB FA in little-endian)
 */
static const uint8_t LD2450_CMD_FRAME_HEADER[LD2450_FRAME_HEADER_SIZE] = {0xFD, 0xFC, 0xFB, 0xFA};

/**
 * @brief Command frame tail (04 03 02 01 in little-endian)
 */
static const uint8_t LD2450_CMD_FRAME_TAIL[LD2450_FRAME_TAIL_SIZE] = {0x04, 0x03, 0x02, 0x01};

/**
 * @brief Command words
 */
#define LD2450_CMD_ENABLE_CONFIG      0xFF00  /*!< Enable configuration mode command */
#define LD2450_CMD_END_CONFIG         0xFE00  /*!< End configuration mode command */
#define LD2450_CMD_SINGLE_TARGET      0x8000  /*!< Set single target tracking mode command */
#define LD2450_CMD_MULTI_TARGET       0x9000  /*!< Set multi-target tracking mode command */
#define LD2450_CMD_QUERY_TARGET_MODE  0x9100  /*!< Query target tracking mode command */
#define LD2450_CMD_READ_FW_VERSION    0xA000  /*!< Read firmware version command */
#define LD2450_CMD_SET_BAUD_RATE      0xA100  /*!< Set serial port baud rate command */
#define LD2450_CMD_RESTORE_FACTORY    0xA200  /*!< Restore factory settings command */
#define LD2450_CMD_RESTART            0xA300  /*!< Restart module command */
#define LD2450_CMD_BLUETOOTH          0xA400  /*!< Bluetooth settings command */
#define LD2450_CMD_GET_MAC            0xA500  /*!< Get MAC address command */
#define LD2450_CMD_QUERY_REGION       0xC100  /*!< Query region filtering configuration command */
#define LD2450_CMD_SET_REGION         0xC200  /*!< Set region filtering configuration command */

/**
 * @brief Status codes for ACK responses
 */
#define LD2450_ACK_SUCCESS            0x0000  /*!< Command executed successfully */
#define LD2450_ACK_FAILURE            0x0001  /*!< Command execution failed */

/**
 * @brief Timeouts
 */
#define LD2450_DEFAULT_TIMEOUT_MS     1000     /*!< Default timeout for commands in milliseconds */
#define LD2450_RESTART_TIMEOUT_MS     3000    /*!< Timeout for restart command in milliseconds */
#define LD2450_UART_TIMEOUT_TICKS     (100 / portTICK_PERIOD_MS) /*!< Timeout for UART operations */

/**
 * @brief Driver state
 */
#define LD2450_STATE_UNINITIALIZED    0       /*!< Driver not initialized */
#define LD2450_STATE_IDLE             1       /*!< Driver idle, waiting for data */
#define LD2450_STATE_CONFIG           2       /*!< Driver in configuration mode */
#define LD2450_STATE_COMMAND          3       /*!< Driver sending a command */
#define LD2450_STATE_WAITING_ACK      4       /*!< Driver waiting for ACK */
#define LD2450_STATE_DEINITIALIZING   5       /*!< Driver deinitializing */

/**
 * @brief Frame parsing states
 */
typedef enum {
    LD2450_PARSE_WAIT_HEADER,         /*!< Waiting for frame header */
    LD2450_PARSE_WAIT_LENGTH,         /*!< Waiting for data length */
    LD2450_PARSE_WAIT_DATA,           /*!< Waiting for frame data */
    LD2450_PARSE_WAIT_TAIL,           /*!< Waiting for frame tail */
} ld2450_parse_state_t;

/**
 * @brief Data frame parsing states
 */
typedef enum {
    LD2450_DATA_PARSE_WAIT_HEADER,    /*!< Waiting for data frame header (AA FF) */
    LD2450_DATA_PARSE_WAIT_TYPE,      /*!< Waiting for data frame type (03 00) */
    LD2450_DATA_PARSE_WAIT_TARGETS,   /*!< Waiting for target data */
    LD2450_DATA_PARSE_WAIT_TAIL,      /*!< Waiting for data frame tail (55 CC) */
} ld2450_data_parse_state_t;

/**
 * @brief Driver context structure
 */
typedef struct {
    /* Configuration */
    uart_port_t uart_port;            /*!< UART port number */
    uint32_t uart_baud_rate;          /*!< UART baud rate */
    
    /* State */
    uint8_t state;                    /*!< Current driver state */
    bool config_mode;                 /*!< Whether currently in configuration mode */
    
    /* Parsing state for command responses */
    ld2450_parse_state_t parse_state;             /*!< Current command frame parsing state */
    uint16_t data_length;                         /*!< Length of frame data being parsed */
    uint16_t bytes_received;                      /*!< Number of bytes received in current frame */
    uint16_t cmd_word;                            /*!< Command word of current frame */
    
    /* Parsing state for data frames */
    ld2450_data_parse_state_t data_parse_state;   /*!< Current data frame parsing state */
    uint32_t data_bytes_received;                 /*!< Number of bytes received in current data frame */
    uint8_t target_index;                         /*!< Current target index being parsed */
    
    /* Data buffers */
    uint8_t cmd_tx_buffer[LD2450_MAX_CMD_FRAME_SIZE];     /*!< Buffer for command transmission */
    uint8_t cmd_rx_buffer[LD2450_MAX_ACK_FRAME_SIZE];     /*!< Buffer for command reception */
    
    /* Double buffering for data frames */
    uint8_t data_buffer_a[LD2450_MAX_DATA_FRAME_SIZE];    /*!< First data buffer */
    uint8_t data_buffer_b[LD2450_MAX_DATA_FRAME_SIZE];    /*!< Second data buffer */
    uint8_t *active_buffer;                               /*!< Pointer to active reception buffer */
    uint8_t *processing_buffer;                           /*!< Pointer to buffer being processed */
    
    /* Target data */
    ld2450_target_t targets[LD2450_MAX_TARGETS];          /*!< Array of target data */
    size_t active_target_count;                           /*!< Number of active targets */
    
    /* Callback */
    ld2450_data_callback_t data_callback;                 /*!< User callback for data */
    void *user_ctx;                                       /*!< User context for callback */
    
    /* Synchronization */
    SemaphoreHandle_t uart_mutex;                         /*!< Mutex for UART access */
    SemaphoreHandle_t cmd_sem;                            /*!< Semaphore for command completion */
    TaskHandle_t uart_task;                               /*!< UART task handle */
    QueueHandle_t uart_queue;                             /*!< UART event queue */
    
    /* Result of last command */
    esp_err_t last_error;                                 /*!< Last error code */
    uint16_t last_ack_status;                             /*!< Last ACK status */
    uint8_t last_ack_data[LD2450_MAX_RETURN_VALUE_SIZE];  /*!< Last ACK data */
    uint16_t last_ack_length;                             /*!< Length of last ACK data */
    
} ld2450_context_t;

/**
 * @brief Global driver context
 */
extern ld2450_context_t g_ld2450_ctx;

/**
 * @brief Send a command to the radar and wait for ACK
 * 
 * @param[in] cmd_word Command word
 * @param[in] cmd_value Command value buffer
 * @param[in] cmd_value_len Length of command value
 * @param[out] ack_data Buffer to store ACK data
 * @param[out] ack_len Length of ACK data
 * @param[in] timeout_ms Timeout in milliseconds
 * @return esp_err_t ESP_OK on success
 */
esp_err_t ld2450_send_command(uint16_t cmd_word, const uint8_t *cmd_value, 
                            uint16_t cmd_value_len, uint8_t *ack_data,
                            uint16_t *ack_len, uint32_t timeout_ms);

/**
 * @brief Process a received data frame
 * 
 * @param[in] buffer Data frame buffer
 * @param[in] length Buffer length
 * @return esp_err_t ESP_OK on success
 */
esp_err_t ld2450_process_data_frame(const uint8_t *buffer, size_t length);

#ifdef __cplusplus
}
#endif