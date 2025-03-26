/**
 * @file resource_monitor.c
 * @brief Resource monitor implementation
 */

#include "resource_monitor.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "driver/uart.h"
#include "driver/i2c.h"
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#ifdef CONFIG_PM_ENABLE
#include "esp_pm.h"
#endif
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

// Increase buffer size for systems with many tasks
#define RESOURCE_MONITOR_CPU_STATS_BUFFER_SIZE 2048
#define MAX_MONITORED_TASKS 16

// Additional structures for new monitoring features
#define MAX_TIMING_ENTRIES 10
typedef struct {
    char label[32];
    int64_t start_time;
    uint32_t last_duration_us;
    uint32_t min_duration_us;
    uint32_t max_duration_us;
    uint64_t total_duration_us;
    uint32_t count;
    bool active;
} timing_entry_t;

typedef struct {
    uint32_t uart_tx_bytes_prev;
    uint32_t uart_rx_bytes_prev;
    uint32_t uart_errors_prev;
    uint32_t i2c_transfers_prev;
    uint32_t i2c_errors_prev;
    uint32_t spi_transfers_prev; 
    uint32_t spi_errors_prev;
    int64_t last_io_check_time;
    bool io_stats_started;
} io_stats_ctx_t;

// Internal context
typedef struct {
    bool initialized;
    bool periodic_running;
    TimerHandle_t periodic_timer;
    TaskHandle_t monitor_task;
    SemaphoreHandle_t mutex;           // Mutex to prevent concurrent execution
    uint32_t interval_ms;              // Store the interval for the monitoring task
    bool monitor_request;              // Flag to request monitoring
} resource_monitor_ctx_t;

// Context extension for new monitoring features
typedef struct {
    timing_entry_t timing_entries[MAX_TIMING_ENTRIES];
    uint8_t timing_entry_count;
    io_stats_ctx_t io_stats;
#ifdef CONFIG_PM_ENABLE
    int64_t last_power_check_time;
    uint32_t sleep_time_ms_prev;
    uint32_t active_time_ms_prev;
#endif
    struct {
        uint32_t previous_runtime_ticks[MAX_MONITORED_TASKS];
        TaskHandle_t task_handles[MAX_MONITORED_TASKS];
        uint8_t task_count;
        int64_t last_sample_time;
        float task_cpu_usage[MAX_MONITORED_TASKS];  // Store CPU usage percentages
    } cpu_monitor;
} resource_monitor_extended_ctx_t;

static resource_monitor_ctx_t s_rm_ctx = {
    .initialized = false,
    .periodic_running = false,
    .periodic_timer = NULL,
    .monitor_task = NULL,
    .mutex = NULL,
    .interval_ms = 0,
    .monitor_request = false
};

static resource_monitor_extended_ctx_t s_rm_ext_ctx = {
    .timing_entry_count = 0,
    .io_stats = {
        .uart_tx_bytes_prev = 0,
        .uart_rx_bytes_prev = 0,
        .uart_errors_prev = 0,
        .i2c_transfers_prev = 0,
        .i2c_errors_prev = 0,
        .spi_transfers_prev = 0,
        .spi_errors_prev = 0,
        .last_io_check_time = 0,
        .io_stats_started = false
    },
#ifdef CONFIG_PM_ENABLE
    .last_power_check_time = 0,
    .sleep_time_ms_prev = 0,
    .active_time_ms_prev = 0
#endif
};

// Forward declarations
static void periodic_timer_callback(TimerHandle_t timer);
static void resource_monitor_task(void *arg);
static void sample_task_cpu_usage(void);

esp_err_t resource_monitor_init(void)
{
    if (s_rm_ctx.initialized) {
        return ESP_OK; // Already initialized
    }
    
    // Create mutex for thread safety
    s_rm_ctx.mutex = xSemaphoreCreateMutex();
    if (s_rm_ctx.mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_ERR_NO_MEM;
    }
    
    // Initialize extended context
    memset(&s_rm_ext_ctx, 0, sizeof(resource_monitor_extended_ctx_t));
    
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
    
    // Delete monitoring task if it exists
    if (s_rm_ctx.monitor_task != NULL) {
        vTaskDelete(s_rm_ctx.monitor_task);
        s_rm_ctx.monitor_task = NULL;
    }
    
    // Delete the mutex
    if (s_rm_ctx.mutex != NULL) {
        vSemaphoreDelete(s_rm_ctx.mutex);
        s_rm_ctx.mutex = NULL;
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
    char *buf = malloc(RESOURCE_MONITOR_CPU_STATS_BUFFER_SIZE);
    if (buf == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for CPU stats");
        return ESP_ERR_NO_MEM;
    }
    
    ESP_LOGI(TAG, "==== CPU USAGE BY TASK ====");
    
    // Get task runtime statistics with safety check on buffer size
    memset(buf, 0, RESOURCE_MONITOR_CPU_STATS_BUFFER_SIZE);
    vTaskGetRunTimeStats(buf);
    
    // Ensure string is properly terminated
    buf[RESOURCE_MONITOR_CPU_STATS_BUFFER_SIZE - 1] = '\0';
    
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
    resource_monitor_print_stack_watermarks();
    
    if (s_rm_ext_ctx.io_stats.io_stats_started) {
        resource_monitor_print_io_stats();
    }
    
    resource_monitor_print_timing_stats();
    
#ifdef CONFIG_PM_ENABLE
    resource_monitor_print_power_stats();
#endif
    
    return ESP_OK;
}

static void periodic_timer_callback(TimerHandle_t timer)
{
    // Just set the flag - actual work done in the monitoring task
    s_rm_ctx.monitor_request = true;
}

static void resource_monitor_task(void *arg)
{
    TickType_t last_wake_time = xTaskGetTickCount();
    
    while (s_rm_ctx.periodic_running) {
        // Check if monitoring was requested by the timer
        if (s_rm_ctx.monitor_request && xSemaphoreTake(s_rm_ctx.mutex, 0) == pdTRUE) {
            // Sample CPU usage before printing
            sample_task_cpu_usage();
            
            s_rm_ctx.monitor_request = false;
            resource_monitor_print_all();
            xSemaphoreGive(s_rm_ctx.mutex);
        }
        
        // Sleep until next check
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(100)); // Check every 100ms
    }
    
    // Task will be deleted by the caller
    vTaskDelete(NULL);
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
    
    s_rm_ctx.interval_ms = interval_ms;
    
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
    
    // Create dedicated task for monitoring
    s_rm_ctx.periodic_running = true;
    BaseType_t task_created = xTaskCreate(
        resource_monitor_task,
        "res_monitor",
        4096,  // Larger stack to handle all the operations
        NULL,
        tskIDLE_PRIORITY + 1,
        &s_rm_ctx.monitor_task
    );
    
    if (task_created != pdPASS) {
        ESP_LOGE(TAG, "Failed to create monitoring task");
        s_rm_ctx.periodic_running = false;
        return ESP_ERR_NO_MEM;
    }
    
    // Start timer
    if (xTimerStart(s_rm_ctx.periodic_timer, portMAX_DELAY) != pdPASS) {
        ESP_LOGE(TAG, "Failed to start periodic timer");
        vTaskDelete(s_rm_ctx.monitor_task);
        s_rm_ctx.monitor_task = NULL;
        s_rm_ctx.periodic_running = false;
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "Started periodic resource monitoring (interval: %" PRIu32 "ms)", interval_ms);
    
    return ESP_OK;
}

esp_err_t resource_monitor_stop_periodic(void)
{
    if (!s_rm_ctx.initialized || !s_rm_ctx.periodic_running) {
        return ESP_OK; // Not running
    }
    
    // Stop and delete timer
    if (s_rm_ctx.periodic_timer != NULL) {
        if (xTimerStop(s_rm_ctx.periodic_timer, portMAX_DELAY) != pdPASS) {
            ESP_LOGE(TAG, "Failed to stop periodic timer");
            return ESP_FAIL;
        }
        
        // Delete the timer to free resources
        if (xTimerDelete(s_rm_ctx.periodic_timer, portMAX_DELAY) != pdPASS) {
            ESP_LOGE(TAG, "Failed to delete periodic timer");
            // Continue anyway - we'll try to clean up as much as possible
        }
        s_rm_ctx.periodic_timer = NULL;
    }
    
    // Signal task to stop and wait for it to exit
    s_rm_ctx.periodic_running = false;
    
    // Wait a bit for task to terminate cleanly
    vTaskDelay(pdMS_TO_TICKS(200));
    
    // Delete task if still running
    if (s_rm_ctx.monitor_task != NULL) {
        vTaskDelete(s_rm_ctx.monitor_task);
        s_rm_ctx.monitor_task = NULL;
    }
    
    ESP_LOGI(TAG, "Stopped periodic resource monitoring");
    
    return ESP_OK;
}

// Implement task statistics monitoring
esp_err_t resource_monitor_get_task_stats(void* task_handle, resource_monitor_task_stats_t *stats)
{
    if (!s_rm_ctx.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (stats == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    TaskHandle_t handle = task_handle ? (TaskHandle_t)task_handle : xTaskGetCurrentTaskHandle();
    
    // Clear stats structure
    memset(stats, 0, sizeof(resource_monitor_task_stats_t));
    
    // Get stack high water mark
    stats->high_water_mark = uxTaskGetStackHighWaterMark(handle);
    
    // Look up CPU usage if available
    for (int i = 0; i < s_rm_ext_ctx.cpu_monitor.task_count; i++) {
        if (s_rm_ext_ctx.cpu_monitor.task_handles[i] == handle) {
            stats->cpu_usage_pct = s_rm_ext_ctx.cpu_monitor.task_cpu_usage[i];
            break;
        }
    }
    
    return ESP_OK;
}

esp_err_t resource_monitor_print_stack_watermarks(void)
{
    if (!s_rm_ctx.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Buffer to store task list
    char *buf = malloc(RESOURCE_MONITOR_CPU_STATS_BUFFER_SIZE);
    if (buf == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for task list");
        return ESP_ERR_NO_MEM;
    }
    
    ESP_LOGI(TAG, "==== TASK STACK USAGE ====");
    
    // Get task list with safety check on buffer size
    memset(buf, 0, RESOURCE_MONITOR_CPU_STATS_BUFFER_SIZE);
    vTaskList(buf);
    
    // Ensure string is properly terminated
    buf[RESOURCE_MONITOR_CPU_STATS_BUFFER_SIZE - 1] = '\0';
    
    ESP_LOGI(TAG, "Task Name\tState\tPrio\tStack\tNum");
    ESP_LOGI(TAG, "%s", buf);
    
    free(buf);
    return ESP_OK;
}

// I/O Statistics Implementation
esp_err_t resource_monitor_start_io_stats(void)
{
    if (!s_rm_ctx.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (xSemaphoreTake(s_rm_ctx.mutex, portMAX_DELAY) != pdTRUE) {
        return ESP_FAIL;
    }
    
    // Reset I/O statistics
    memset(&s_rm_ext_ctx.io_stats, 0, sizeof(io_stats_ctx_t));
    s_rm_ext_ctx.io_stats.last_io_check_time = esp_timer_get_time();
    s_rm_ext_ctx.io_stats.io_stats_started = true;
    
    xSemaphoreGive(s_rm_ctx.mutex);
    
    ESP_LOGI(TAG, "I/O statistics collection started");
    return ESP_OK;
}

esp_err_t resource_monitor_get_io_stats(resource_monitor_io_stats_t *stats)
{
    if (!s_rm_ctx.initialized || !s_rm_ext_ctx.io_stats.io_stats_started) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (stats == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (xSemaphoreTake(s_rm_ctx.mutex, portMAX_DELAY) != pdTRUE) {
        return ESP_FAIL;
    }
    
    // Clear stats structure
    memset(stats, 0, sizeof(resource_monitor_io_stats_t));
    
    // Calculate time since last check
    int64_t now = esp_timer_get_time();
    float seconds_elapsed = (now - s_rm_ext_ctx.io_stats.last_io_check_time) / 1000000.0f;
    
    // TODO: Read actual I/O statistics from drivers
    // This requires platform-specific code for UART, I2C, and SPI drivers
    // For example with UART:
    // uart_get_tx_buffer_free_size(UART_NUM_0)
    
    // For now, this is a placeholder implementation
    stats->uart_tx_bytes = 1000; // Example value
    stats->uart_rx_bytes = 500;  // Example value
    
    // Calculate rates
    if (seconds_elapsed > 0) {
        stats->uart_tx_rate = (stats->uart_tx_bytes - s_rm_ext_ctx.io_stats.uart_tx_bytes_prev) / seconds_elapsed;
        stats->uart_rx_rate = (stats->uart_rx_bytes - s_rm_ext_ctx.io_stats.uart_rx_bytes_prev) / seconds_elapsed;
    }
    
    // Update previous values for next calculation
    s_rm_ext_ctx.io_stats.uart_tx_bytes_prev = stats->uart_tx_bytes;
    s_rm_ext_ctx.io_stats.uart_rx_bytes_prev = stats->uart_rx_bytes;
    s_rm_ext_ctx.io_stats.last_io_check_time = now;
    
    xSemaphoreGive(s_rm_ctx.mutex);
    
    return ESP_OK;
}

esp_err_t resource_monitor_print_io_stats(void)
{
    if (!s_rm_ctx.initialized || !s_rm_ext_ctx.io_stats.io_stats_started) {
        return ESP_ERR_INVALID_STATE;
    }
    
    resource_monitor_io_stats_t stats;
    esp_err_t err = resource_monitor_get_io_stats(&stats);
    if (err != ESP_OK) {
        return err;
    }
    
    ESP_LOGI(TAG, "==== I/O STATISTICS ====");
    ESP_LOGI(TAG, "UART: TX %"PRIu32" bytes (%.2f B/s), RX %"PRIu32" bytes (%.2f B/s), Errors: %"PRIu32,
             stats.uart_tx_bytes, stats.uart_tx_rate,
             stats.uart_rx_bytes, stats.uart_rx_rate,
             stats.uart_errors);
    
    ESP_LOGI(TAG, "I2C: Transfers: %"PRIu32", Errors: %"PRIu32,
             stats.i2c_transfers, stats.i2c_errors);
    
    ESP_LOGI(TAG, "SPI: Transfers: %"PRIu32", Errors: %"PRIu32,
             stats.spi_transfers, stats.spi_errors);
    
    return ESP_OK;
}

// Timing Statistics Implementation
esp_err_t resource_monitor_timing_begin(const char *label)
{
    if (!s_rm_ctx.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (label == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (xSemaphoreTake(s_rm_ctx.mutex, portMAX_DELAY) != pdTRUE) {
        return ESP_FAIL;
    }
    
    // Find existing timing entry or create new one
    int idx = -1;
    for (int i = 0; i < s_rm_ext_ctx.timing_entry_count; i++) {
        if (strcmp(s_rm_ext_ctx.timing_entries[i].label, label) == 0) {
            idx = i;
            break;
        }
    }
    
    if (idx == -1) {
        // Create new entry if we have space
        if (s_rm_ext_ctx.timing_entry_count >= MAX_TIMING_ENTRIES) {
            ESP_LOGW(TAG, "Too many timing entries, cannot add %s", label);
            xSemaphoreGive(s_rm_ctx.mutex);
            return ESP_ERR_NO_MEM;
        }
        
        idx = s_rm_ext_ctx.timing_entry_count++;
        strncpy(s_rm_ext_ctx.timing_entries[idx].label, label, sizeof(s_rm_ext_ctx.timing_entries[idx].label) - 1);
        s_rm_ext_ctx.timing_entries[idx].min_duration_us = UINT32_MAX;
        s_rm_ext_ctx.timing_entries[idx].max_duration_us = 0;
        s_rm_ext_ctx.timing_entries[idx].total_duration_us = 0;
        s_rm_ext_ctx.timing_entries[idx].count = 0;
    }
    
    if (s_rm_ext_ctx.timing_entries[idx].active) {
        ESP_LOGW(TAG, "Timing for %s already started", label);
        xSemaphoreGive(s_rm_ctx.mutex);
        return ESP_ERR_INVALID_STATE;
    }
    
    // Start timing
    s_rm_ext_ctx.timing_entries[idx].start_time = esp_timer_get_time();
    s_rm_ext_ctx.timing_entries[idx].active = true;
    
    xSemaphoreGive(s_rm_ctx.mutex);
    return ESP_OK;
}

esp_err_t resource_monitor_timing_end(const char *label)
{
    if (!s_rm_ctx.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (label == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (xSemaphoreTake(s_rm_ctx.mutex, portMAX_DELAY) != pdTRUE) {
        return ESP_FAIL;
    }
    
    // Find timing entry
    int idx = -1;
    for (int i = 0; i < s_rm_ext_ctx.timing_entry_count; i++) {
        if (strcmp(s_rm_ext_ctx.timing_entries[i].label, label) == 0) {
            idx = i;
            break;
        }
    }
    
    if (idx == -1 || !s_rm_ext_ctx.timing_entries[idx].active) {
        ESP_LOGW(TAG, "Timing for %s not started", label);
        xSemaphoreGive(s_rm_ctx.mutex);
        return ESP_ERR_INVALID_STATE;
    }
    
    // Calculate duration
    int64_t now = esp_timer_get_time();
    uint32_t duration_us = (uint32_t)(now - s_rm_ext_ctx.timing_entries[idx].start_time);
    
    // Update stats
    s_rm_ext_ctx.timing_entries[idx].last_duration_us = duration_us;
    s_rm_ext_ctx.timing_entries[idx].min_duration_us = 
        (duration_us < s_rm_ext_ctx.timing_entries[idx].min_duration_us) ? 
        duration_us : s_rm_ext_ctx.timing_entries[idx].min_duration_us;
    s_rm_ext_ctx.timing_entries[idx].max_duration_us = 
        (duration_us > s_rm_ext_ctx.timing_entries[idx].max_duration_us) ? 
        duration_us : s_rm_ext_ctx.timing_entries[idx].max_duration_us;
    s_rm_ext_ctx.timing_entries[idx].total_duration_us += duration_us;
    s_rm_ext_ctx.timing_entries[idx].count++;
    s_rm_ext_ctx.timing_entries[idx].active = false;
    
    xSemaphoreGive(s_rm_ctx.mutex);
    return ESP_OK;
}

esp_err_t resource_monitor_print_timing_stats(void)
{
    if (!s_rm_ctx.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (xSemaphoreTake(s_rm_ctx.mutex, portMAX_DELAY) != pdTRUE) {
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "==== TIMING STATISTICS ====");
    ESP_LOGI(TAG, "Label\t\tLast\tMin\tMax\tAvg\tCount");
    
    for (int i = 0; i < s_rm_ext_ctx.timing_entry_count; i++) {
        timing_entry_t *entry = &s_rm_ext_ctx.timing_entries[i];
        
        uint32_t avg_us = 0;
        if (entry->count > 0) {
            avg_us = (uint32_t)(entry->total_duration_us / entry->count);
        }
        
        ESP_LOGI(TAG, "%s\t%"PRIu32"\t%"PRIu32"\t%"PRIu32"\t%"PRIu32"\t%"PRIu32,
                 entry->label,
                 entry->last_duration_us,
                 entry->min_duration_us == UINT32_MAX ? 0 : entry->min_duration_us,
                 entry->max_duration_us,
                 avg_us,
                 entry->count);
    }
    
    xSemaphoreGive(s_rm_ctx.mutex);
    return ESP_OK;
}

#ifdef CONFIG_PM_ENABLE
// Power monitoring implementation
esp_err_t resource_monitor_get_power_stats(resource_monitor_power_stats_t *stats)
{
    if (!s_rm_ctx.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (stats == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Clear stats structure
    memset(stats, 0, sizeof(resource_monitor_power_stats_t));
    
    // TODO: Read actual power measurements
    // This requires platform-specific code, possibly using ADC to measure voltage and current
    // or external power monitoring IC
    
    // For now, this is a placeholder implementation with example values
    stats->current_ma = 100.0f;  // Example value
    stats->voltage_v = 3.3f;     // Example value
    stats->power_mw = stats->current_ma * stats->voltage_v;
    
    // Get time in power modes (if available from ESP-IDF)
    // This is platform-specific
    
    return ESP_OK;
}

esp_err_t resource_monitor_print_power_stats(void)
{
    if (!s_rm_ctx.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    resource_monitor_power_stats_t stats;
    esp_err_t err = resource_monitor_get_power_stats(&stats);
    if (err != ESP_OK) {
        return err;
    }
    
    ESP_LOGI(TAG, "==== POWER STATISTICS ====");
    ESP_LOGI(TAG, "Current: %.2f mA, Voltage: %.2f V, Power: %.2f mW",
             stats.current_ma, stats.voltage_v, stats.power_mw);
    ESP_LOGI(TAG, "Sleep time: %"PRIu32" ms, Active time: %"PRIu32" ms",
             stats.sleep_time_ms, stats.active_time_ms);
    
    return ESP_OK;
}
#endif

static void sample_task_cpu_usage(void)
{
    int64_t now = esp_timer_get_time();
    
    // Get current runtime for all tasks
    TaskStatus_t *task_status_array = malloc(uxTaskGetNumberOfTasks() * sizeof(TaskStatus_t));
    if (task_status_array == NULL) {
        return;
    }
    
    uint64_t total_runtime;  // Changed from uint32_t to uint64_t
    uint32_t task_count = uxTaskGetSystemState(task_status_array, 
                                             uxTaskGetNumberOfTasks(), 
                                             &total_runtime);
                                             
    // Calculate CPU usage for each task
    for (int i = 0; i < task_count; i++) {
        // Find if we're already tracking this task
        int idx = -1;
        for (int j = 0; j < s_rm_ext_ctx.cpu_monitor.task_count; j++) {
            if (s_rm_ext_ctx.cpu_monitor.task_handles[j] == task_status_array[i].xHandle) {
                idx = j;
                break;
            }
        }
        
        // Add new task if needed
        if (idx == -1 && s_rm_ext_ctx.cpu_monitor.task_count < MAX_MONITORED_TASKS) {
            idx = s_rm_ext_ctx.cpu_monitor.task_count++;
            s_rm_ext_ctx.cpu_monitor.task_handles[idx] = task_status_array[i].xHandle;
            s_rm_ext_ctx.cpu_monitor.previous_runtime_ticks[idx] = 0;
            s_rm_ext_ctx.cpu_monitor.task_cpu_usage[idx] = 0.0f;
        }
        
        // Calculate percentage if we're tracking this task
        if (idx != -1) {
            uint32_t current_runtime = task_status_array[i].ulRunTimeCounter;
            uint32_t runtime_diff = current_runtime - s_rm_ext_ctx.cpu_monitor.previous_runtime_ticks[idx];
            s_rm_ext_ctx.cpu_monitor.previous_runtime_ticks[idx] = current_runtime;
            
            // Calculate and store CPU usage percentage if we have valid total runtime
            if (total_runtime > 0 && runtime_diff <= total_runtime) {
                s_rm_ext_ctx.cpu_monitor.task_cpu_usage[idx] = (100.0f * runtime_diff) / total_runtime;
            }
        }
    }
    
    s_rm_ext_ctx.cpu_monitor.last_sample_time = now;
    free(task_status_array);
}
