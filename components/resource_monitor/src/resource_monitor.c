/**
 * @file resource_monitor.c
 * @brief Resource monitor implementation
 */

#include "resource_monitor.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include <string.h>
#include <inttypes.h>

static const char *TAG = "resource-monitor";

// Configuration for runtime stats
#if configGENERATE_RUN_TIME_STATS == 0
#error "Resource monitor requires configGENERATE_RUN_TIME_STATS to be enabled in FreeRTOSConfig.h"
#endif

#if configUSE_TRACE_FACILITY == 0 
#error "Resource monitor requires configUSE_TRACE_FACILITY to be enabled in FreeRTOSConfig.h"
#endif

// Internal context
typedef struct {
    bool initialized;
    bool periodic_running;
    TimerHandle_t periodic_timer;
} resource_monitor_ctx_t;

static resource_monitor_ctx_t s_rm_ctx = {
    .initialized = false,
    .periodic_running = false,
    .periodic_timer = NULL
};

// Forward declarations
static void periodic_timer_callback(TimerHandle_t timer);

esp_err_t resource_monitor_init(void)
{
    if (s_rm_ctx.initialized) {
        return ESP_OK; // Already initialized
    }
    
    s_rm_ctx.initialized = true;
    ESP_LOGI(TAG, "Resource monitor initialized");
    
    return ESP_OK;
}

esp_err_t resource_monitor_deinit(void)
{
    if (!s_rm_ctx.initialized) {
        return ESP_OK; // Not initialized
    }
    
    // Stop periodic timer if running
    if (s_rm_ctx.periodic_running) {
        resource_monitor_stop_periodic();
    }
    
    s_rm_ctx.initialized = false;
    ESP_LOGI(TAG, "Resource monitor deinitialized");
    
    return ESP_OK;
}

esp_err_t resource_monitor_get_mem_stats(resource_monitor_mem_stats_t *stats)
{
    if (!s_rm_ctx.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (stats == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Clear stats structure
    memset(stats, 0, sizeof(resource_monitor_mem_stats_t));
    
    // Get internal heap stats
    stats->free_heap = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
    stats->total_heap = heap_caps_get_total_size(MALLOC_CAP_INTERNAL);
    stats->min_free_heap = heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL);
    stats->max_alloc_heap = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL);
    
#ifdef CONFIG_SPIRAM
    // Get PSRAM stats if enabled
    stats->free_psram = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
    stats->total_psram = heap_caps_get_total_size(MALLOC_CAP_SPIRAM);
    stats->min_free_psram = heap_caps_get_minimum_free_size(MALLOC_CAP_SPIRAM);
    stats->max_alloc_psram = heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM);
#endif
    
    return ESP_OK;
}

esp_err_t resource_monitor_print_mem_stats(void)
{
    if (!s_rm_ctx.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    resource_monitor_mem_stats_t stats;
    esp_err_t err = resource_monitor_get_mem_stats(&stats);
    if (err != ESP_OK) {
        return err;
    }
    
    ESP_LOGI(TAG, "==== MEMORY STATISTICS ====");
    ESP_LOGI(TAG, "Internal Heap: Free: %" PRIu32 " bytes, Total: %" PRIu32 " bytes, Min Free: %" PRIu32 " bytes, Max Block: %" PRIu32 " bytes",
             (uint32_t)stats.free_heap, (uint32_t)stats.total_heap, (uint32_t)stats.min_free_heap, (uint32_t)stats.max_alloc_heap);
    ESP_LOGI(TAG, "Internal Heap Usage: %" PRIu32 "%% used, %" PRIu32 "%% free", 
             (uint32_t)((stats.total_heap - stats.free_heap) * 100 / stats.total_heap),
             (uint32_t)(stats.free_heap * 100 / stats.total_heap));
             
#ifdef CONFIG_SPIRAM
    ESP_LOGI(TAG, "PSRAM: Free: %" PRIu32 " bytes, Total: %" PRIu32 " bytes, Min Free: %" PRIu32 " bytes, Max Block: %" PRIu32 " bytes",
             (uint32_t)stats.free_psram, (uint32_t)stats.total_psram, (uint32_t)stats.min_free_psram, (uint32_t)stats.max_alloc_psram);
    if (stats.total_psram > 0) {
        ESP_LOGI(TAG, "PSRAM Usage: %" PRIu32 "%% used, %" PRIu32 "%% free", 
                 (uint32_t)((stats.total_psram - stats.free_psram) * 100 / stats.total_psram),
                 (uint32_t)(stats.free_psram * 100 / stats.total_psram));
    }
#endif
    
    return ESP_OK;
}

esp_err_t resource_monitor_print_cpu_stats(void)
{
    if (!s_rm_ctx.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Buffer to store task stats
    char *buf = malloc(1024);
    if (buf == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for CPU stats");
        return ESP_ERR_NO_MEM;
    }
    
    ESP_LOGI(TAG, "==== CPU USAGE BY TASK ====");
    
    // Get task runtime statistics
    vTaskGetRunTimeStats(buf);
    ESP_LOGI(TAG, "Task Name\tRuntime\t\tPercentage");
    ESP_LOGI(TAG, "%s", buf);
    
    free(buf);
    return ESP_OK;
}

esp_err_t resource_monitor_print_all(void)
{
    if (!s_rm_ctx.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    resource_monitor_print_mem_stats();
    resource_monitor_print_cpu_stats();
    
    return ESP_OK;
}

static void periodic_timer_callback(TimerHandle_t timer)
{
    resource_monitor_print_all();
}

esp_err_t resource_monitor_start_periodic(uint32_t interval_ms)
{
    if (!s_rm_ctx.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (s_rm_ctx.periodic_running) {
        ESP_LOGW(TAG, "Periodic monitoring already running");
        return ESP_OK;
    }
    
    // Create timer if it doesn't exist
    if (s_rm_ctx.periodic_timer == NULL) {
        s_rm_ctx.periodic_timer = xTimerCreate(
            "rm_timer",
            pdMS_TO_TICKS(interval_ms),
            pdTRUE,  // Auto reload
            NULL,
            periodic_timer_callback
        );
        
        if (s_rm_ctx.periodic_timer == NULL) {
            ESP_LOGE(TAG, "Failed to create periodic timer");
            return ESP_ERR_NO_MEM;
        }
    } else {
        // Update timer period
        xTimerChangePeriod(s_rm_ctx.periodic_timer, pdMS_TO_TICKS(interval_ms), portMAX_DELAY);
    }
    
    // Start timer
    if (xTimerStart(s_rm_ctx.periodic_timer, portMAX_DELAY) != pdPASS) {
        ESP_LOGE(TAG, "Failed to start periodic timer");
        return ESP_FAIL;
    }
    
    s_rm_ctx.periodic_running = true;
    ESP_LOGI(TAG, "Started periodic resource monitoring (interval: %" PRIu32 "ms)", interval_ms);
    
    return ESP_OK;
}

esp_err_t resource_monitor_stop_periodic(void)
{
    if (!s_rm_ctx.initialized || !s_rm_ctx.periodic_running) {
        return ESP_OK; // Not running
    }
    
    if (s_rm_ctx.periodic_timer != NULL) {
        if (xTimerStop(s_rm_ctx.periodic_timer, portMAX_DELAY) != pdPASS) {
            ESP_LOGE(TAG, "Failed to stop periodic timer");
            return ESP_FAIL;
        }
    }
    
    s_rm_ctx.periodic_running = false;
    ESP_LOGI(TAG, "Stopped periodic resource monitoring");
    
    return ESP_OK;
}
