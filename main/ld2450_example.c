/**
 * @file ld2450_example.c
 * @brief Example application for HLK-LD2450 radar sensor on ESP32-S3
 * 
 * This example demonstrates how to use the LD2450 driver with a console
 * interface to control the radar and display output data.
 * 
 * @copyright Copyright (c) 2025
 * @license MIT License
 */

#include <stdio.h>
#include <string.h>
#include "esp_system.h"
#include "esp_log.h"
#include "esp_console.h"
#include "esp_vfs_dev.h"
#include "driver/uart.h"
#include "linenoise.h"
#include "argtable3.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "ld2450.h"

static const char *TAG = "ld2450_example";

// Target data display state
static bool display_data_active = false;
static esp_timer_handle_t display_timer = NULL;
static bool verbose_output = false;

// Forward declarations of command functions
static int cmd_ld2450_init(int argc, char **argv);
static int cmd_ld2450_status(int argc, char **argv);
static int cmd_ld2450_config(int argc, char **argv);
static int cmd_ld2450_data(int argc, char **argv);
static int cmd_ld2450_region(int argc, char **argv);
static int cmd_ld2450_version(int argc, char **argv);
static int cmd_ld2450_reset(int argc, char **argv);
static int cmd_ld2450_help(int argc, char **argv);

// Timer callback for periodic data display
static void display_data_timer_callback(void *arg)
{
    ld2450_frame_data_t frame_data;

    esp_err_t err = ld2450_get_data(&frame_data);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error getting data: %s", ld2450_err_to_str(err));
        return;
    }

    printf("\n----- LD2450 Radar Data (%u) -----\n", frame_data.timestamp);
    
    for (int i = 0; i < 3; i++) {
        if (frame_data.targets[i].present) {
            printf("Target %d: Position (X,Y) = (%d,%d) mm, Speed: %d cm/s, Resolution: %u mm\n", 
                  i + 1,
                  frame_data.targets[i].x, 
                  frame_data.targets[i].y,
                  frame_data.targets[i].speed,
                  frame_data.targets[i].resolution);
            
            if (verbose_output) {
                // Calculate distance from origin
                float distance = sqrtf(
                    (float)frame_data.targets[i].x * frame_data.targets[i].x + 
                    (float)frame_data.targets[i].y * frame_data.targets[i].y
                ) / 1000.0f; // Convert to meters
                
                printf("  Distance from origin: %.2f m\n", distance);
                printf("  Moving: %s\n", 
                      (frame_data.targets[i].speed > 5) ? "Approaching" : 
                      (frame_data.targets[i].speed < -5) ? "Receding" : "Stationary");
            }
        } else if (verbose_output) {
            printf("Target %d: Not present\n", i + 1);
        }
    }
    
    printf("-------------------------------\n");
}

// Event handler for LD2450 events
static void ld2450_event_handler(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{
    switch (id) {
        case LD2450_EVENT_TARGET_DETECTED:
            printf("\nEvent: Target detected\n");
            break;
        case LD2450_EVENT_TARGET_LOST:
            printf("\nEvent: Target lost\n");
            break;
        case LD2450_EVENT_COMM_ERROR:
            printf("\nEvent: Communication error\n");
            break;
        case LD2450_EVENT_CONFIG_CHANGED:
            printf("\nEvent: Configuration changed\n");
            break;
        default:
            printf("\nEvent: Unknown (%ld)\n", id);
            break;
    }
}

// Initialize console
static void initialize_console(void)
{
    /* Initialize the console */
    esp_console_repl_t *repl = NULL;
    esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    repl_config.prompt = "ld2450> ";
    repl_config.max_cmdline_length = 256;

    /* Register commands */
    esp_console_register_help_command();
    
    esp_console_dev_uart_config_t hw_config = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_uart(&hw_config, &repl_config, &repl));

    ESP_ERROR_CHECK(esp_console_start_repl(repl));
}

// Command initialization
static void register_commands(void)
{
    // Initialize command
    const esp_console_cmd_t cmd_init = {
        .command = "init",
        .help = "Initialize LD2450 radar sensor",
        .hint = NULL,
        .func = &cmd_ld2450_init,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd_init));

    // Status command
    const esp_console_cmd_t cmd_status = {
        .command = "status",
        .help = "Get current status and configuration",
        .hint = NULL,
        .func = &cmd_ld2450_status,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd_status));

    // Config command
    const esp_console_cmd_t cmd_config = {
        .command = "config",
        .help = "Configure radar parameters",
        .hint = "[--mode single|multi] [--baud <rate>] [--bt on|off]",
        .func = &cmd_ld2450_config,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd_config));

    // Data command
    const esp_console_cmd_t cmd_data = {
        .command = "data",
        .help = "Control data display",
        .hint = "[start|stop] [--interval <ms>] [--verbose]",
        .func = &cmd_ld2450_data,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd_data));

    // Region command
    const esp_console_cmd_t cmd_region = {
        .command = "region",
        .help = "Configure region filtering",
        .hint = "[--mode none|inside|outside] [--region <id> <x1> <y1> <x2> <y2>]",
        .func = &cmd_ld2450_region,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd_region));

    // Version command
    const esp_console_cmd_t cmd_version = {
        .command = "version",
        .help = "Get firmware version",
        .hint = NULL,
        .func = &cmd_ld2450_version,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd_version));

    // Reset command
    const esp_console_cmd_t cmd_reset = {
        .command = "reset",
        .help = "Reset radar (factory|restart)",
        .hint = "[factory|restart]",
        .func = &cmd_ld2450_reset,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd_reset));

    // Help command
    const esp_console_cmd_t cmd_help = {
        .command = "help",
        .help = "Print help information",
        .hint = NULL,
        .func = &cmd_ld2450_help,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd_help));
}

static int cmd_ld2450_init(int argc, char **argv)
{
    static bool initialized = false;

    if (initialized) {
        printf("LD2450 already initialized. Deinitializing first...\n");
        esp_err_t err = ld2450_deinit();
        if (err != ESP_OK) {
            printf("Error deinitializing: %s\n", ld2450_err_to_str(err));
            return 1;
        }
        initialized = false;
    }

    printf("Initializing LD2450 radar sensor...\n");
    
    // Default configuration
    ld2450_config_t config = LD2450_DEFAULT_CONFIG();
    
    // Customize UART pins for ESP32-S3 (adjust as needed)
    config.uart_port = UART_NUM_1;
    config.tx_pin = 17;   // Adjust based on your wiring
    config.rx_pin = 18;   // Adjust based on your wiring
    config.baud_rate = LD2450_BAUD_RATE_256000;  // Default baud rate
    
    esp_err_t err = ld2450_init(&config);
    if (err != ESP_OK) {
        printf("Error initializing LD2450: %s\n", ld2450_err_to_str(err));
        return 1;
    }
    
    // Register event handler
    err = ld2450_register_event_handler(ld2450_event_handler, NULL);
    if (err != ESP_OK) {
        printf("Error registering event handler: %s\n", ld2450_err_to_str(err));
        return 1;
    }
    
    printf("LD2450 initialized successfully\n");
    initialized = true;
    return 0;
}

static int cmd_ld2450_status(int argc, char **argv)
{
    ld2450_tracking_mode_t mode;
    ld2450_region_filter_t filter;
    uint8_t mac_addr[6];
    
    esp_err_t err = ld2450_get_tracking_mode(&mode);
    if (err != ESP_OK) {
        printf("Error getting tracking mode: %s\n", ld2450_err_to_str(err));
        return 1;
    }
    
    err = ld2450_get_region_filter(&filter);
    if (err != ESP_OK) {
        printf("Error getting region filter: %s\n", ld2450_err_to_str(err));
        return 1;
    }
    
    err = ld2450_get_mac_address(mac_addr);
    if (err != ESP_OK) {
        printf("Error getting MAC address: %s\n", ld2450_err_to_str(err));
        return 1;
    }
    
    printf("\n----- LD2450 Status -----\n");
    printf("Tracking mode: %s\n", mode == LD2450_MODE_SINGLE_TARGET ? "Single target" : "Multi target");
    printf("MAC Address: %02X:%02X:%02X:%02X:%02X:%02X\n", 
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
    
    printf("Region Filter: %s\n", 
           filter.mode == LD2450_FILTER_NONE ? "None" : 
           filter.mode == LD2450_FILTER_INSIDE_REGION ? "Inside region" : "Outside region");
    
    if (filter.mode != LD2450_FILTER_NONE) {
        printf("  Region 1: (%d,%d) to (%d,%d)\n", 
               filter.region1_p1.x, filter.region1_p1.y, filter.region1_p2.x, filter.region1_p2.y);
        printf("  Region 2: (%d,%d) to (%d,%d)\n", 
               filter.region2_p1.x, filter.region2_p1.y, filter.region2_p2.x, filter.region2_p2.y);
        printf("  Region 3: (%d,%d) to (%d,%d)\n", 
               filter.region3_p1.x, filter.region3_p1.y, filter.region3_p2.x, filter.region3_p2.y);
    }
    
    printf("Data display: %s\n", display_data_active ? "Active" : "Inactive");
    printf("Verbose output: %s\n", verbose_output ? "Enabled" : "Disabled");
    printf("-------------------------\n");
    
    return 0;
}

static int cmd_ld2450_config(int argc, char **argv)
{
    // Define arguments using argtable3
    struct {
        struct arg_str *mode;
        struct arg_int *baud;
        struct arg_str *bt;
        struct arg_end *end;
    } args;
    
    args.mode = arg_str0(NULL, "mode", "<mode>", "Tracking mode (single or multi)");
    args.baud = arg_int0(NULL, "baud", "<rate>", "Baud rate (0-7)");
    args.bt = arg_str0(NULL, "bt", "<on|off>", "Bluetooth mode");
    args.end = arg_end(5);
    
    int nerrors = arg_parse(argc, argv, (void **)&args);
    if (nerrors != 0) {
        arg_print_errors(stderr, args.end, argv[0]);
        return 1;
    }
    
    // Enter config mode
    esp_err_t err = ld2450_enable_config();
    if (err != ESP_OK) {
        printf("Error entering config mode: %s\n", ld2450_err_to_str(err));
        return 1;
    }
    
    // Set tracking mode if specified
    if (args.mode->count > 0) {
        ld2450_tracking_mode_t mode;
        if (strcmp(args.mode->sval[0], "single") == 0) {
            mode = LD2450_MODE_SINGLE_TARGET;
        } else if (strcmp(args.mode->sval[0], "multi") == 0) {
            mode = LD2450_MODE_MULTI_TARGET;
        } else {
            printf("Invalid tracking mode. Use 'single' or 'multi'.\n");
            ld2450_end_config();
            return 1;
        }
        
        err = ld2450_set_tracking_mode(mode);
        if (err != ESP_OK) {
            printf("Error setting tracking mode: %s\n", ld2450_err_to_str(err));
            ld2450_end_config();
            return 1;
        }
        printf("Tracking mode set to %s\n", mode == LD2450_MODE_SINGLE_TARGET ? "single target" : "multi-target");
    }
    
        // Set baud rate if specified
        if (args.baud->count > 0) {
            int baud_idx = args.baud->ival[0];
            if (baud_idx < 0 || baud_idx > 7) {
                printf("Invalid baud rate index. Use 0-7.\n");
                ld2450_end_config();
                return 1;
            }
            
            ld2450_baud_rate_t baud_rate = (ld2450_baud_rate_t)baud_idx;
            err = ld2450_set_baud_rate(baud_rate);
            if (err != ESP_OK) {
                printf("Error setting baud rate: %s\n", ld2450_err_to_str(err));
                ld2450_end_config();
                return 1;
            }
            printf("Baud rate set. Note: You'll need to restart the radar and update driver settings.\n");
        }
        
        // Set Bluetooth mode if specified
        if (args.bt->count > 0) {
            ld2450_bt_mode_t bt_mode;
            if (strcmp(args.bt->sval[0], "on") == 0) {
                bt_mode = LD2450_BT_ON;
            } else if (strcmp(args.bt->sval[0], "off") == 0) {
                bt_mode = LD2450_BT_OFF;
            } else {
                printf("Invalid Bluetooth mode. Use 'on' or 'off'.\n");
                ld2450_end_config();
                return 1;
            }
            
            err = ld2450_set_bluetooth(bt_mode);
            if (err != ESP_OK) {
                printf("Error setting Bluetooth mode: %s\n", ld2450_err_to_str(err));
                ld2450_end_config();
                return 1;
            }
            printf("Bluetooth mode set to %s\n", bt_mode == LD2450_BT_ON ? "on" : "off");
        }
        
        // End config mode
        err = ld2450_end_config();
        if (err != ESP_OK) {
            printf("Error exiting config mode: %s\n", ld2450_err_to_str(err));
            return 1;
        }
        
        printf("Configuration updated successfully\n");
        return 0;
    }
    
    static int cmd_ld2450_data(int argc, char **argv)
    {
        // Define arguments using argtable3
        struct {
            struct arg_str *action;
            struct arg_int *interval;
            struct arg_lit *verbose;
            struct arg_end *end;
        } args;
        
        args.action = arg_str0(NULL, NULL, "<action>", "Action (start or stop)");
        args.interval = arg_int0(NULL, "interval", "<ms>", "Update interval in milliseconds");
        args.verbose = arg_lit0("v", "verbose", "Enable verbose output");
        args.end = arg_end(5);
        
        int nerrors = arg_parse(argc, argv, (void **)&args);
        if (nerrors != 0) {
            arg_print_errors(stderr, args.end, argv[0]);
            return 1;
        }
        
        // Handle verbose flag
        if (args.verbose->count > 0) {
            verbose_output = true;
            printf("Verbose output enabled\n");
        }
        
        // Get update interval (default to 1000ms)
        uint32_t interval_ms = 1000;
        if (args.interval->count > 0) {
            interval_ms = args.interval->ival[0];
            if (interval_ms < 100) {
                printf("Warning: Interval below 100ms may cause high CPU usage\n");
            }
        }
        
        // Handle action
        if (args.action->count > 0) {
            if (strcmp(args.action->sval[0], "start") == 0) {
                // Stop existing timer if active
                if (display_data_active && display_timer) {
                    esp_timer_stop(display_timer);
                    esp_timer_delete(display_timer);
                    display_timer = NULL;
                    display_data_active = false;
                }
                
                // Create and start new timer
                esp_timer_create_args_t timer_args = {
                    .callback = &display_data_timer_callback,
                    .name = "display_timer"
                };
                
                esp_err_t err = esp_timer_create(&timer_args, &display_timer);
                if (err != ESP_OK) {
                    printf("Error creating timer: %d\n", err);
                    return 1;
                }
                
                err = esp_timer_start_periodic(display_timer, interval_ms * 1000);
                if (err != ESP_OK) {
                    printf("Error starting timer: %d\n", err);
                    esp_timer_delete(display_timer);
                    display_timer = NULL;
                    return 1;
                }
                
                display_data_active = true;
                printf("Data display started (interval: %ums)\n", interval_ms);
                
            } else if (strcmp(args.action->sval[0], "stop") == 0) {
                if (display_data_active && display_timer) {
                    esp_timer_stop(display_timer);
                    esp_timer_delete(display_timer);
                    display_timer = NULL;
                    display_data_active = false;
                    printf("Data display stopped\n");
                } else {
                    printf("Data display is not active\n");
                }
            } else {
                printf("Invalid action. Use 'start' or 'stop'.\n");
                return 1;
            }
        } else {
            // No action specified, just fetch and display current data once
            ld2450_frame_data_t frame_data;
            
            esp_err_t err = ld2450_get_data(&frame_data);
            if (err != ESP_OK) {
                printf("Error getting data: %s\n", ld2450_err_to_str(err));
                return 1;
            }
            
            display_data_timer_callback(NULL);
        }
        
        return 0;
    }
    
    static int cmd_ld2450_region(int argc, char **argv)
    {
        // Define arguments using argtable3
        struct {
            struct arg_str *mode;
            struct arg_str *region;
            struct arg_int *coords;
            struct arg_end *end;
        } args;
        
        args.mode = arg_str0(NULL, "mode", "<mode>", "Filter mode (none, inside, outside)");
        args.region = arg_str0(NULL, "region", "<id>", "Region number (1, 2, or 3)");
        args.coords = arg_intn(NULL, NULL, "<x1> <y1> <x2> <y2>", 0, 4, "Region coordinates");
        args.end = arg_end(5);
        
        int nerrors = arg_parse(argc, argv, (void **)&args);
        if (nerrors != 0) {
            arg_print_errors(stderr, args.end, argv[0]);
            return 1;
        }
        
        // Get current region filter
        ld2450_region_filter_t filter;
        esp_err_t err = ld2450_get_region_filter(&filter);
        if (err != ESP_OK) {
            printf("Error getting region filter: %s\n", ld2450_err_to_str(err));
            return 1;
        }
        
        // Set filter mode if specified
        if (args.mode->count > 0) {
            if (strcmp(args.mode->sval[0], "none") == 0) {
                filter.mode = LD2450_FILTER_NONE;
            } else if (strcmp(args.mode->sval[0], "inside") == 0) {
                filter.mode = LD2450_FILTER_INSIDE_REGION;
            } else if (strcmp(args.mode->sval[0], "outside") == 0) {
                filter.mode = LD2450_FILTER_OUTSIDE_REGION;
            } else {
                printf("Invalid filter mode. Use 'none', 'inside', or 'outside'.\n");
                return 1;
            }
        }
        
        // Update region coordinates if specified
        if (args.region->count > 0 && args.coords->count == 4) {
            int region_num = atoi(args.region->sval[0]);
            int x1 = args.coords->ival[0];
            int y1 = args.coords->ival[1];
            int x2 = args.coords->ival[2];
            int y2 = args.coords->ival[3];
            
            switch (region_num) {
                case 1:
                    filter.region1_p1.x = x1;
                    filter.region1_p1.y = y1;
                    filter.region1_p2.x = x2;
                    filter.region1_p2.y = y2;
                    break;
                case 2:
                    filter.region2_p1.x = x1;
                    filter.region2_p1.y = y1;
                    filter.region2_p2.x = x2;
                    filter.region2_p2.y = y2;
                    break;
                case 3:
                    filter.region3_p1.x = x1;
                    filter.region3_p1.y = y1;
                    filter.region3_p2.x = x2;
                    filter.region3_p2.y = y2;
                    break;
                default:
                    printf("Invalid region number. Use 1, 2, or 3.\n");
                    return 1;
            }
        } else if (args.region->count > 0 && args.coords->count > 0 && args.coords->count < 4) {
            printf("You must specify all four coordinates (x1 y1 x2 y2) for a region.\n");
            return 1;
        }
        
        // Enter config mode
        err = ld2450_enable_config();
        if (err != ESP_OK) {
            printf("Error entering config mode: %s\n", ld2450_err_to_str(err));
            return 1;
        }
        
        // Update filter settings
        err = ld2450_set_region_filter(&filter);
        if (err != ESP_OK) {
            printf("Error setting region filter: %s\n", ld2450_err_to_str(err));
            ld2450_end_config();
            return 1;
        }
        
        // End config mode
        err = ld2450_end_config();
        if (err != ESP_OK) {
            printf("Error exiting config mode: %s\n", ld2450_err_to_str(err));
            return 1;
        }
        
        printf("Region filter updated successfully\n");
        return 0;
    }
    
    static int cmd_ld2450_version(int argc, char **argv)
    {
        ld2450_firmware_version_t version;
        
        esp_err_t err = ld2450_get_firmware_version(&version);
        if (err != ESP_OK) {
            printf("Error getting firmware version: %s\n", ld2450_err_to_str(err));
            return 1;
        }
        
        printf("\n----- LD2450 Firmware Version -----\n");
        printf("Version string: %s\n", version.version_str);
        printf("Main version: %u\n", version.main_version);
        printf("Sub version: %u\n", version.sub_version);
        printf("---------------------------------\n");
        
        return 0;
    }
    
    static int cmd_ld2450_reset(int argc, char **argv)
    {
        if (argc < 2) {
            printf("Please specify reset type: 'factory' or 'restart'\n");
            return 1;
        }
        
        esp_err_t err;
        
        if (strcmp(argv[1], "factory") == 0) {
            printf("Performing factory reset...\n");
            
            err = ld2450_enable_config();
            if (err != ESP_OK) {
                printf("Error entering config mode: %s\n", ld2450_err_to_str(err));
                return 1;
            }
            
            err = ld2450_restore_factory_settings();
            if (err != ESP_OK) {
                printf("Error performing factory reset: %s\n", ld2450_err_to_str(err));
                ld2450_end_config();
                return 1;
            }
            
            ld2450_end_config();
            printf("Factory reset completed. Please restart the radar module.\n");
            
        } else if (strcmp(argv[1], "restart") == 0) {
            printf("Restarting radar module...\n");
            
            err = ld2450_restart();
            if (err != ESP_OK) {
                printf("Error restarting radar module: %s\n", ld2450_err_to_str(err));
                return 1;
            }
            
            printf("Radar module restart command sent\n");
            
        } else {
            printf("Invalid reset type. Use 'factory' or 'restart'\n");
            return 1;
        }
        
        return 0;
    }
    
    static int cmd_ld2450_help(int argc, char **argv)
    {
        printf("\nLD2450 Radar Sensor Commands:\n\n");
        printf("  init                  Initialize the radar driver\n");
        printf("  status                Show current status and configuration\n");
        printf("  config                Configure radar parameters\n");
        printf("    --mode single|multi   Set tracking mode\n");
        printf("    --baud <0-7>          Set baud rate\n");
        printf("    --bt on|off           Enable/disable Bluetooth\n");
        printf("  data [start|stop]     Control data display\n");
        printf("    --interval <ms>       Set update interval\n");
        printf("    --verbose             Enable verbose output\n");
        printf("  region                Configure region filtering\n");
        printf("    --mode none|inside|outside   Set filter mode\n");
        printf("    --region <1-3> <x1> <y1> <x2> <y2>   Set region coordinates\n");
        printf("  version               Get firmware version\n");
        printf("  reset [factory|restart]   Reset or restart the radar module\n");
        printf("  help                  Display this help message\n\n");
        printf("Example usage:\n");
        printf("  ld2450> init\n");
        printf("  ld2450> config --mode multi --bt off\n");
        printf("  ld2450> data start --interval 500 --verbose\n");
        printf("  ld2450> region --mode inside --region 1 -1000 -1000 1000 1000\n");
        printf("  ld2450> reset restart\n\n");
        return 0;
    }
    
    void app_main(void)
{
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // Print welcome message
    printf("\n\n========================================\n");
    printf("  LD2450 Radar Sensor Example\n");
    printf("  ESP32-S3 Development Board\n");
    printf("========================================\n");
    printf("\nType 'help' for available commands\n\n");
    
    // Register console commands
    esp_console_register_help_command();
    register_commands();
    
    // Initialize the console
    initialize_console();
}