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

#ifdef __cplusplus
}
#endif
