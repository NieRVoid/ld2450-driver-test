/**
 * @file protocol.h
 * @brief HLK-LD2450 radar sensor internal protocol definitions
 *
 * This file contains the internal protocol definitions for the HLK-LD2450
 * 24GHz radar sensor module, including frame formats, command codes, and 
 * data structures for communication.
 *
 * @copyright Copyright (c) 2025 NieRVoid
 * @license MIT License
 */

 #ifndef LD2450_PROTOCOL_H_
 #define LD2450_PROTOCOL_H_
 
 #include <stdint.h>
 #include <stdbool.h>
 
 #ifdef __cplusplus
 extern "C" {
 #endif
 
 /**
  * @brief Protocol frame header and trailer constants
  */
 #define LD2450_FRAME_HEADER           {0xFD, 0xFC, 0xFB, 0xFA}
 #define LD2450_FRAME_TRAILER          {0x04, 0x03, 0x02, 0x01}
 #define LD2450_HEADER_LEN             4
 #define LD2450_TRAILER_LEN            4
 
 /**
  * @brief Data frame header and trailer constants
  */
 #define LD2450_DATA_FRAME_HEADER      {0xAA, 0xFF, 0x03, 0x00}
 #define LD2450_DATA_FRAME_TRAILER     {0x55, 0xCC}
 #define LD2450_DATA_HEADER_LEN        4
 #define LD2450_DATA_TRAILER_LEN       2
 
 /**
  * @brief Command words (2 bytes each)
  */
 typedef enum {
     LD2450_CMD_ENABLE_CONFIG      = 0x00FF, /*!< Enable configuration mode */
     LD2450_CMD_END_CONFIG         = 0x00FE, /*!< End configuration mode */
     LD2450_CMD_SINGLE_TARGET      = 0x0080, /*!< Set single target tracking mode */
     LD2450_CMD_MULTI_TARGET       = 0x0090, /*!< Set multi-target tracking mode */
     LD2450_CMD_QUERY_TRACK_MODE   = 0x0091, /*!< Query target tracking mode */
     LD2450_CMD_READ_FW_VERSION    = 0x00A0, /*!< Read firmware version */
     LD2450_CMD_SET_BAUD_RATE      = 0x00A1, /*!< Set serial port baud rate */
     LD2450_CMD_RESTORE_FACTORY    = 0x00A2, /*!< Restore factory settings */
     LD2450_CMD_RESTART_MODULE     = 0x00A3, /*!< Restart module */
     LD2450_CMD_BLUETOOTH_SETTINGS = 0x00A4, /*!< Configure Bluetooth settings */
     LD2450_CMD_GET_MAC_ADDRESS    = 0x00A5, /*!< Get MAC address */
     LD2450_CMD_QUERY_REGION_FILTER = 0x00C1, /*!< Query region filtering configuration */
     LD2450_CMD_SET_REGION_FILTER  = 0x00C2  /*!< Set region filtering configuration */
 } ld2450_cmd_t;
 
 /**
  * @brief Baud rate selection indices
  */
 typedef enum {
     LD2450_BAUD_9600   = 0x0001,
     LD2450_BAUD_19200  = 0x0002,
     LD2450_BAUD_38400  = 0x0003,
     LD2450_BAUD_57600  = 0x0004,
     LD2450_BAUD_115200 = 0x0005,
     LD2450_BAUD_230400 = 0x0006,
     LD2450_BAUD_256000 = 0x0007, /*!< Factory default */
     LD2450_BAUD_460800 = 0x0008
 } ld2450_baud_t;
 
 /**
  * @brief Tracking mode values
  */
 typedef enum {
     LD2450_TRACK_SINGLE = 0x0001, /*!< Single target tracking */
     LD2450_TRACK_MULTI  = 0x0002  /*!< Multi-target tracking (default) */
 } ld2450_track_mode_t;
 
 /**
  * @brief Region filtering types
  */
 typedef enum {
     LD2450_FILTER_DISABLE = 0x0000, /*!< Disable region filtering */
     LD2450_FILTER_INSIDE  = 0x0001, /*!< Only detect targets within region */
     LD2450_FILTER_OUTSIDE = 0x0002  /*!< Do not detect targets within region */
 } ld2450_region_filter_type_t;
 
 /**
  * @brief Bluetooth settings
  */
 typedef enum {
     LD2450_BT_DISABLE = 0x0000,
     LD2450_BT_ENABLE  = 0x0100  /*!< Default */
 } ld2450_bluetooth_t;
 
 /**
  * @brief ACK status values
  */
 typedef enum {
     LD2450_ACK_SUCCESS = 0x0000,
     LD2450_ACK_FAILURE = 0x0001
 } ld2450_ack_status_t;
 
 /**
  * @brief Coordinate structure for region filtering
  */
 typedef struct {
     int16_t x;
     int16_t y;
 } ld2450_coord_t;
 
 /**
  * @brief Region filtering configuration (for up to 3 regions)
  */
 typedef struct {
     ld2450_region_filter_type_t type;
     ld2450_coord_t region1_p1;
     ld2450_coord_t region1_p2;
     ld2450_coord_t region2_p1;
     ld2450_coord_t region2_p2;
     ld2450_coord_t region3_p1;
     ld2450_coord_t region3_p2;
 } ld2450_region_config_t;
 
 /**
  * @brief Internal firmware version structure (for protocol use only)
  */
 typedef struct {
     uint16_t fw_type;
     uint16_t major_version;
     uint32_t sub_version;
 } ld2450_protocol_firmware_version_t;
 
 /**
  * @brief Target data structure
  */
 typedef struct {
     int16_t x;           /*!< X coordinate in mm */
     int16_t y;           /*!< Y coordinate in mm */
     int16_t speed;       /*!< Speed in cm/s */
     uint16_t resolution; /*!< Distance resolution in mm */
     bool present;        /*!< Target present flag */
 } ld2450_target_t;
 
 /**
  * @brief Maximum number of targets supported by the radar
  */
 #define LD2450_MAX_TARGETS 3
 
 /**
  * @brief MAC address length in bytes
  */
 #define LD2450_MAC_ADDR_LEN 6
 
 /**
  * @brief Maximum command and response buffer sizes
  */
 #define LD2450_MAX_CMD_LEN     32
 #define LD2450_MAX_RESP_LEN    64
 
 /**
  * @brief Protocol state machine states
  */
 typedef enum {
     LD2450_STATE_WAIT_HEADER,      /*!< Waiting for frame header */
     LD2450_STATE_READ_LENGTH,      /*!< Reading data length */
     LD2450_STATE_READ_DATA,        /*!< Reading frame data */
     LD2450_STATE_WAIT_TRAILER,     /*!< Waiting for frame trailer */
     LD2450_STATE_COMPLETE          /*!< Frame reception complete */
 } ld2450_rx_state_t;
 
 /**
  * @brief Data frame parsing states
  */
 typedef enum {
     LD2450_DATA_STATE_WAIT_HEADER,   /*!< Waiting for data frame header */
     LD2450_DATA_STATE_READ_TARGETS,  /*!< Reading target data */
     LD2450_DATA_STATE_WAIT_TRAILER,  /*!< Waiting for data frame trailer */
     LD2450_DATA_STATE_COMPLETE       /*!< Data frame complete */
 } ld2450_data_rx_state_t;
 
 #ifdef __cplusplus
 }
 #endif
 
 #endif /* LD2450_PROTOCOL_H_ */