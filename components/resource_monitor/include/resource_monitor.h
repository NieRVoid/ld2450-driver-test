/**
 * @file resource_monitor.h
 * @brief Simple resource monitoring component for ESP32
 */

#pragma once

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Memory statistics structure
 */
typedef struct {
    size_t total_heap;          /*!< Total heap size */
    size_t free_heap;           /*!< Current free heap */
    size_t min_free_heap;       /*!< Minimum free heap since boot */
    size_t max_alloc_heap;      /*!< Largest allocatable block */
#ifdef CONFIG_SPIRAM
    size_t total_psram;         /*!< Total PSRAM size */
    size_t free_psram;          /*!< Current free PSRAM */
    size_t min_free_psram;      /*!< Minimum free PSRAM since boot */
    size_t max_alloc_psram;     /*!< Largest allocatable PSRAM block */
#endif
} resource_monitor_mem_stats_t;

/**
 * @brief Task statistics structure
 */
typedef struct {
    size_t high_water_mark;     /*!< Minimum free stack space ever */
    uint32_t runtime;           /*!< Total runtime in ticks */
    float cpu_usage_pct;        /*!< CPU usage percentage */
} resource_monitor_task_stats_t;

/**
 * @brief I/O performance statistics
 */
typedef struct {
    uint32_t uart_tx_bytes;     /*!< UART transmitted bytes */
    uint32_t uart_rx_bytes;     /*!< UART received bytes */
    uint32_t uart_errors;       /*!< UART error count */
    float uart_tx_rate;         /*!< UART TX rate in bytes/sec */
    float uart_rx_rate;         /*!< UART RX rate in bytes/sec */
    uint32_t i2c_transfers;     /*!< I2C transfer count */
    uint32_t i2c_errors;        /*!< I2C error count */
    uint32_t spi_transfers;     /*!< SPI transfer count */
    uint32_t spi_errors;        /*!< SPI error count */
} resource_monitor_io_stats_t;

/**
 * @brief Timing metrics
 */
typedef struct {
    uint32_t processing_time_us;    /*!< Processing time in microseconds */
    uint32_t min_processing_time_us; /*!< Minimum processing time */
    uint32_t max_processing_time_us; /*!< Maximum processing time */
    uint32_t avg_processing_time_us; /*!< Average processing time */
    uint32_t frame_rate;            /*!< Frame rate for sensor data */
    uint32_t response_time_us;      /*!< Command response time */
} resource_monitor_timing_stats_t;

#ifdef CONFIG_PM_ENABLE
/**
 * @brief Power metrics
 */
typedef struct {
    float current_ma;           /*!< Current consumption in mA */
    float voltage_v;            /*!< Supply voltage */
    float power_mw;             /*!< Power consumption in mW */
    uint32_t sleep_time_ms;     /*!< Time spent in sleep modes */
    uint32_t active_time_ms;    /*!< Time spent in active mode */
} resource_monitor_power_stats_t;
#endif

/**
 * @brief Initialize the resource monitor
 *
 * This must be called before using other resource monitor functions.
 * 
 * @return esp_err_t ESP_OK on success
 */
esp_err_t resource_monitor_init(void);

/**
 * @brief Deinitialize the resource monitor
 * 
 * @return esp_err_t ESP_OK on success
 */
esp_err_t resource_monitor_deinit(void);

/**
 * @brief Get memory usage statistics
 * 
 * @param stats Pointer to store memory statistics
 * @return esp_err_t ESP_OK on success
 */
esp_err_t resource_monitor_get_mem_stats(resource_monitor_mem_stats_t *stats);

/**
 * @brief Get CPU usage by task
 * 
 * Prints CPU usage information for all tasks to the log.
 * 
 * @return esp_err_t ESP_OK on success
 */
esp_err_t resource_monitor_print_cpu_stats(void);

/**
 * @brief Print memory usage statistics
 * 
 * @return esp_err_t ESP_OK on success
 */
esp_err_t resource_monitor_print_mem_stats(void);

/**
 * @brief Print all resource usage (memory and CPU) 
 * 
 * @return esp_err_t ESP_OK on success
 */
esp_err_t resource_monitor_print_all(void);

/**
 * @brief Start periodic resource usage printing
 * 
 * @param interval_ms Interval between prints in milliseconds
 * @return esp_err_t ESP_OK on success
 */
esp_err_t resource_monitor_start_periodic(uint32_t interval_ms);

/**
 * @brief Stop periodic resource usage printing
 * 
 * @return esp_err_t ESP_OK on success
 */
esp_err_t resource_monitor_stop_periodic(void);

/**
 * @brief Get task statistics for a specific task
 * 
 * @param task_handle Task handle, NULL for calling task
 * @param stats Pointer to task statistics structure
 * @return esp_err_t ESP_OK on success
 */
esp_err_t resource_monitor_get_task_stats(void* task_handle, resource_monitor_task_stats_t *stats);

/**
 * @brief Print task stack high watermarks for all tasks
 * 
 * @return esp_err_t ESP_OK on success
 */
esp_err_t resource_monitor_print_stack_watermarks(void);

/**
 * @brief Begin collecting I/O statistics
 * 
 * @return esp_err_t ESP_OK on success
 */
esp_err_t resource_monitor_start_io_stats(void);

/**
 * @brief Get I/O performance statistics
 * 
 * @param stats Pointer to I/O statistics structure
 * @return esp_err_t ESP_OK on success 
 */
esp_err_t resource_monitor_get_io_stats(resource_monitor_io_stats_t *stats);

/**
 * @brief Print I/O performance statistics
 * 
 * @return esp_err_t ESP_OK on success
 */
esp_err_t resource_monitor_print_io_stats(void);

/**
 * @brief Begin timing measurement for processing time stats
 * 
 * @param label Identifier for the timing measurement
 * @return esp_err_t ESP_OK on success
 */
esp_err_t resource_monitor_timing_begin(const char *label);

/**
 * @brief End timing measurement and record statistics
 * 
 * @param label Identifier matching the begin call
 * @return esp_err_t ESP_OK on success 
 */
esp_err_t resource_monitor_timing_end(const char *label);

/**
 * @brief Print timing statistics
 * 
 * @return esp_err_t ESP_OK on success
 */
esp_err_t resource_monitor_print_timing_stats(void);

#ifdef CONFIG_PM_ENABLE
/**
 * @brief Get power consumption statistics
 * 
 * @param stats Pointer to power statistics structure
 * @return esp_err_t ESP_OK on success
 */
esp_err_t resource_monitor_get_power_stats(resource_monitor_power_stats_t *stats);

/**
 * @brief Print power consumption statistics
 * 
 * @return esp_err_t ESP_OK on success
 */
esp_err_t resource_monitor_print_power_stats(void);
#endif

#ifdef __cplusplus
}
#endif
