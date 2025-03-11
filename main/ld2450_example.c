/**
 * @file ld2450_example.c
 * @brief Example application for testing LD2450 radar sensor driver
 *
 * This example provides a command-line interface to interact with 
 * the HLK-LD2450 radar sensor through the driver.
 *
 * @copyright Copyright (c) 2025 NieRVoid
 * @license MIT License
 */

 #include <stdio.h>
 #include <string.h>
 #include <stdlib.h>
 #include "esp_log.h"
 #include "esp_console.h"
 #include "esp_system.h"
 #include "esp_vfs_dev.h"
 #include "driver/uart.h"
 #include "linenoise/linenoise.h"
 #include "argtable3/argtable3.h"
 #include "freertos/FreeRTOS.h"
 #include "freertos/task.h"
 #include "freertos/semphr.h"
 #include "ld2450.h"
 
 static const char *TAG = "LD2450_EXAMPLE";
 static bool continuous_data = false;
 static TaskHandle_t data_task_handle = NULL;
 static SemaphoreHandle_t console_mutex = NULL;
 
 /* Event handler for LD2450 events */
 static void ld2450_event_handler(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
 {
     if (xSemaphoreTake(console_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
         return;
     }
 
     switch (id) {
         case LD2450_EVENT_TARGET_DETECTED:
             ESP_LOGI(TAG, "Event: Target detected");
             break;
         case LD2450_EVENT_TARGET_LOST:
             ESP_LOGI(TAG, "Event: Target lost");
             break;
         case LD2450_EVENT_COMM_ERROR:
             ESP_LOGI(TAG, "Event: Communication error");
             break;
         case LD2450_EVENT_CONFIG_CHANGED:
             ESP_LOGI(TAG, "Event: Configuration changed");
             break;
         default:
             ESP_LOGI(TAG, "Unknown event: %ld", id);
             break;
     }
 
     xSemaphoreGive(console_mutex);
 }
 
 /* Print target data in a formatted way */
 static void print_target_data(const ld2450_frame_data_t *data)
 {
     if (xSemaphoreTake(console_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
         return;
     }
 
     printf("\n===== Target Data (timestamp: %lu) =====\n", data->timestamp);
     
     bool targets_found = false;
     for (int i = 0; i < 3; i++) {
         if (data->targets[i].present) {
             targets_found = true;
             printf("Target %d: X=%5d mm, Y=%5d mm, Speed=%4d cm/s, Resolution=%3u mm\n",
                    i + 1, data->targets[i].x, data->targets[i].y, 
                    data->targets[i].speed, data->targets[i].resolution);
         }
     }
     
     if (!targets_found) {
         printf("No targets detected\n");
     }
     
     printf("=====================================\n");
     xSemaphoreGive(console_mutex);
 }
 
 /* Task for continuous data reading */
 static void data_read_task(void *pvParameters)
 {
     ld2450_frame_data_t data;
     
     while (continuous_data) {
         esp_err_t err = ld2450_wait_for_data(&data, 1000);
         if (err == ESP_OK) {
             print_target_data(&data);
         } else if (err != ESP_ERR_TIMEOUT) {
             ESP_LOGE(TAG, "Error reading data: %s", ld2450_err_to_str(err));
             vTaskDelay(pdMS_TO_TICKS(1000));
         }
     }
     
     data_task_handle = NULL;
     vTaskDelete(NULL);
 }
 
 /* Start continuous data reading */
 static void start_continuous_data(void)
 {
     if (data_task_handle != NULL) {
         ESP_LOGW(TAG, "Continuous data reading already active");
         return;
     }
     
     continuous_data = true;
     xTaskCreate(data_read_task, "data_read_task", 4096, NULL, 5, &data_task_handle);
 }
 
 /* Stop continuous data reading */
 static void stop_continuous_data(void)
 {
     continuous_data = false;
     // Task will delete itself
     if (data_task_handle != NULL) {
         // Wait for task to finish
         vTaskDelay(pdMS_TO_TICKS(100));
     }
 }
 
 /* Command: Get latest data */
 static int cmd_get_data(int argc, char **argv)
 {
     ld2450_frame_data_t data;
     esp_err_t err = ld2450_get_data(&data);
     
     if (err != ESP_OK) {
         ESP_LOGE(TAG, "Failed to get data: %s", ld2450_err_to_str(err));
         return 1;
     }
     
     print_target_data(&data);
     return 0;
 }
 
 /* Command: Start continuous data reading */
 static int cmd_start_data(int argc, char **argv)
 {
     start_continuous_data();
     return 0;
 }
 
 /* Command: Stop continuous data reading */
 static int cmd_stop_data(int argc, char **argv)
 {
     stop_continuous_data();
     return 0;
 }
 
 /* Command: Enter configuration mode */
 static int cmd_enter_config(int argc, char **argv)
 {
     esp_err_t err = ld2450_enable_config();
     if (err != ESP_OK) {
         ESP_LOGE(TAG, "Failed to enter config mode: %s", ld2450_err_to_str(err));
         return 1;
     }
     
     ESP_LOGI(TAG, "Entered configuration mode");
     return 0;
 }
 
 /* Command: Exit configuration mode */
 static int cmd_exit_config(int argc, char **argv)
 {
     esp_err_t err = ld2450_end_config();
     if (err != ESP_OK) {
         ESP_LOGE(TAG, "Failed to exit config mode: %s", ld2450_err_to_str(err));
         return 1;
     }
     
     ESP_LOGI(TAG, "Exited configuration mode");
     return 0;
 }
 
 /* Command: Get firmware version */
 static int cmd_get_version(int argc, char **argv)
 {
     ld2450_firmware_version_t version;
     esp_err_t err = ld2450_get_firmware_version(&version);
     
     if (err != ESP_OK) {
         ESP_LOGE(TAG, "Failed to get firmware version: %s", ld2450_err_to_str(err));
         return 1;
     }
     
     printf("Firmware Version: %s (Main: %u, Sub: %lu)\n", 
            version.version_str, version.main_version, version.sub_version);
     return 0;
 }
 
 /* Command: Set tracking mode */
 static struct {
     struct arg_int *mode;
     struct arg_end *end;
 } set_mode_args;
 
 static int cmd_set_mode(int argc, char **argv)
 {
     int nerrors = arg_parse(argc, argv, (void **)&set_mode_args);
     if (nerrors != 0) {
         arg_print_errors(stderr, set_mode_args.end, argv[0]);
         return 1;
     }
     
     int mode = set_mode_args.mode->ival[0];
     if (mode != LD2450_MODE_SINGLE_TARGET && mode != LD2450_MODE_MULTI_TARGET) {
         ESP_LOGE(TAG, "Invalid mode. Use 0 for single target, 1 for multi-target");
         return 1;
     }
     
     esp_err_t err = ld2450_set_tracking_mode((ld2450_tracking_mode_t)mode);
     if (err != ESP_OK) {
         ESP_LOGE(TAG, "Failed to set tracking mode: %s", ld2450_err_to_str(err));
         return 1;
     }
     
     ESP_LOGI(TAG, "Tracking mode set to %s", 
              mode == LD2450_MODE_SINGLE_TARGET ? "single target" : "multi-target");
     return 0;
 }
 
 /* Command: Get tracking mode */
 static int cmd_get_mode(int argc, char **argv)
 {
     ld2450_tracking_mode_t mode;
     esp_err_t err = ld2450_get_tracking_mode(&mode);
     
     if (err != ESP_OK) {
         ESP_LOGE(TAG, "Failed to get tracking mode: %s", ld2450_err_to_str(err));
         return 1;
     }
     
     printf("Current tracking mode: %s\n", 
            mode == LD2450_MODE_SINGLE_TARGET ? "single target" : "multi-target");
     return 0;
 }
 
 /* Command: Set baud rate */
 static struct {
     struct arg_int *baud;
     struct arg_end *end;
 } set_baud_args;
 
 static int cmd_set_baud(int argc, char **argv)
 {
     int nerrors = arg_parse(argc, argv, (void **)&set_baud_args);
     if (nerrors != 0) {
         arg_print_errors(stderr, set_baud_args.end, argv[0]);
         return 1;
     }
     
     int baud_idx = set_baud_args.baud->ival[0];
     if (baud_idx < LD2450_BAUD_RATE_9600 || baud_idx > LD2450_BAUD_RATE_460800) {
         ESP_LOGE(TAG, "Invalid baud rate index (0-7)");
         return 1;
     }
     
     esp_err_t err = ld2450_set_baud_rate((ld2450_baud_rate_t)baud_idx);
     if (err != ESP_OK) {
         ESP_LOGE(TAG, "Failed to set baud rate: %s", ld2450_err_to_str(err));
         return 1;
     }
     
     static const int baud_values[] = {9600, 19200, 38400, 57600, 115200, 230400, 256000, 460800};
     ESP_LOGI(TAG, "Baud rate set to %d. Restart the radar for changes to take effect.", 
              baud_values[baud_idx]);
     return 0;
 }
 
 /* Command: Restart radar */
 static int cmd_restart(int argc, char **argv)
 {
     esp_err_t err = ld2450_restart();
     if (err != ESP_OK) {
         ESP_LOGE(TAG, "Failed to restart radar: %s", ld2450_err_to_str(err));
         return 1;
     }
     
     ESP_LOGI(TAG, "Radar restarting...");
     return 0;
 }
 
 /* Command: Factory reset */
 static int cmd_factory_reset(int argc, char **argv)
 {
     esp_err_t err = ld2450_restore_factory_settings();
     if (err != ESP_OK) {
         ESP_LOGE(TAG, "Failed to factory reset: %s", ld2450_err_to_str(err));
         return 1;
     }
     
     ESP_LOGI(TAG, "Radar restored to factory settings");
     return 0;
 }
 
 /* Command: Get MAC address */
 static int cmd_get_mac(int argc, char **argv)
 {
     uint8_t mac_addr[6];
     esp_err_t err = ld2450_get_mac_address(mac_addr);
     
     if (err != ESP_OK) {
         ESP_LOGE(TAG, "Failed to get MAC address: %s", ld2450_err_to_str(err));
         return 1;
     }
     
     printf("MAC Address: %02X:%02X:%02X:%02X:%02X:%02X\n", 
            mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
     return 0;
 }
 
 /* Command: Set Bluetooth mode */
 static struct {
     struct arg_int *mode;
     struct arg_end *end;
 } set_bt_args;
 
 static int cmd_set_bt(int argc, char **argv)
 {
     int nerrors = arg_parse(argc, argv, (void **)&set_bt_args);
     if (nerrors != 0) {
         arg_print_errors(stderr, set_bt_args.end, argv[0]);
         return 1;
     }
     
     int mode = set_bt_args.mode->ival[0];
     if (mode != LD2450_BT_OFF && mode != LD2450_BT_ON) {
         ESP_LOGE(TAG, "Invalid Bluetooth mode. Use 0 for off, 1 for on");
         return 1;
     }
     
     esp_err_t err = ld2450_set_bluetooth((ld2450_bt_mode_t)mode);
     if (err != ESP_OK) {
         ESP_LOGE(TAG, "Failed to set Bluetooth mode: %s", ld2450_err_to_str(err));
         return 1;
     }
     
     ESP_LOGI(TAG, "Bluetooth %s", mode == LD2450_BT_ON ? "enabled" : "disabled");
     return 0;
 }
 
 /* Command: Help */
 static int cmd_help(int argc, char **argv)
 {
     printf("\nLD2450 Radar Sensor Test Console\n");
     printf("Available commands:\n");
     printf("  get_data           - Get latest radar data\n");
     printf("  start_data         - Start continuous data reading\n");
     printf("  stop_data          - Stop continuous data reading\n");
     printf("  enter_config       - Enter configuration mode\n");
     printf("  exit_config        - Exit configuration mode\n");
     printf("  get_version        - Get firmware version\n");
     printf("  set_mode <mode>    - Set tracking mode (0=single, 1=multi)\n");
     printf("  get_mode           - Get current tracking mode\n");
     printf("  set_baud <idx>     - Set baud rate (0=9600...7=460800)\n");
     printf("  restart            - Restart the radar\n");
     printf("  factory_reset      - Restore factory settings\n");
     printf("  get_mac            - Get MAC address\n");
     printf("  set_bt <mode>      - Set Bluetooth mode (0=off, 1=on)\n");
     printf("  help               - Print this help message\n");
     printf("  exit               - Exit the program\n");
     return 0;
 }
 
 /* Register all console commands */
 static void register_commands(void)
 {
     const esp_console_cmd_t get_data_cmd = {
         .command = "get_data",
         .help = "Get latest radar data",
         .hint = NULL,
         .func = &cmd_get_data,
     };
     ESP_ERROR_CHECK(esp_console_cmd_register(&get_data_cmd));
 
     const esp_console_cmd_t start_data_cmd = {
         .command = "start_data",
         .help = "Start continuous data reading",
         .hint = NULL,
         .func = &cmd_start_data,
     };
     ESP_ERROR_CHECK(esp_console_cmd_register(&start_data_cmd));
 
     const esp_console_cmd_t stop_data_cmd = {
         .command = "stop_data",
         .help = "Stop continuous data reading",
         .hint = NULL,
         .func = &cmd_stop_data,
     };
     ESP_ERROR_CHECK(esp_console_cmd_register(&stop_data_cmd));
 
     const esp_console_cmd_t enter_config_cmd = {
         .command = "enter_config",
         .help = "Enter configuration mode",
         .hint = NULL,
         .func = &cmd_enter_config,
     };
     ESP_ERROR_CHECK(esp_console_cmd_register(&enter_config_cmd));
 
     const esp_console_cmd_t exit_config_cmd = {
         .command = "exit_config",
         .help = "Exit configuration mode",
         .hint = NULL,
         .func = &cmd_exit_config,
     };
     ESP_ERROR_CHECK(esp_console_cmd_register(&exit_config_cmd));
 
     const esp_console_cmd_t get_version_cmd = {
         .command = "get_version",
         .help = "Get firmware version",
         .hint = NULL,
         .func = &cmd_get_version,
     };
     ESP_ERROR_CHECK(esp_console_cmd_register(&get_version_cmd));
 
     set_mode_args.mode = arg_int1(NULL, NULL, "<mode>", "Mode (0=single, 1=multi)");
     set_mode_args.end = arg_end(2);
     const esp_console_cmd_t set_mode_cmd = {
         .command = "set_mode",
         .help = "Set tracking mode (0=single target, 1=multi-target)",
         .hint = NULL,
         .func = &cmd_set_mode,
         .argtable = &set_mode_args
     };
     ESP_ERROR_CHECK(esp_console_cmd_register(&set_mode_cmd));
 
     const esp_console_cmd_t get_mode_cmd = {
         .command = "get_mode",
         .help = "Get current tracking mode",
         .hint = NULL,
         .func = &cmd_get_mode,
     };
     ESP_ERROR_CHECK(esp_console_cmd_register(&get_mode_cmd));
 
     set_baud_args.baud = arg_int1(NULL, NULL, "<idx>", "Baud rate index (0-7)");
     set_baud_args.end = arg_end(2);
     const esp_console_cmd_t set_baud_cmd = {
         .command = "set_baud",
         .help = "Set baud rate (0=9600...7=460800)",
         .hint = NULL,
         .func = &cmd_set_baud,
         .argtable = &set_baud_args
     };
     ESP_ERROR_CHECK(esp_console_cmd_register(&set_baud_cmd));
 
     const esp_console_cmd_t restart_cmd = {
         .command = "restart",
         .help = "Restart the radar",
         .hint = NULL,
         .func = &cmd_restart,
     };
     ESP_ERROR_CHECK(esp_console_cmd_register(&restart_cmd));
 
     const esp_console_cmd_t factory_reset_cmd = {
         .command = "factory_reset",
         .help = "Restore factory settings",
         .hint = NULL,
         .func = &cmd_factory_reset,
     };
     ESP_ERROR_CHECK(esp_console_cmd_register(&factory_reset_cmd));
 
     const esp_console_cmd_t get_mac_cmd = {
         .command = "get_mac",
         .help = "Get MAC address",
         .hint = NULL,
         .func = &cmd_get_mac,
     };
     ESP_ERROR_CHECK(esp_console_cmd_register(&get_mac_cmd));
 
     set_bt_args.mode = arg_int1(NULL, NULL, "<mode>", "BT mode (0=off, 1=on)");
     set_bt_args.end = arg_end(2);
     const esp_console_cmd_t set_bt_cmd = {
         .command = "set_bt",
         .help = "Set Bluetooth mode (0=off, 1=on)",
         .hint = NULL,
         .func = &cmd_set_bt,
         .argtable = &set_bt_args
     };
     ESP_ERROR_CHECK(esp_console_cmd_register(&set_bt_cmd));
 
     const esp_console_cmd_t help_cmd = {
         .command = "help",
         .help = "Print help information",
         .hint = NULL,
         .func = &cmd_help,
     };
     ESP_ERROR_CHECK(esp_console_cmd_register(&help_cmd));
 }
 
 /* Initialize console */
 static void initialize_console(void)
 {
     /* Drain stdout before reconfiguring it */
     fflush(stdout);
     fsync(fileno(stdout));
 
     /* Disable buffering on stdin */
     setvbuf(stdin, NULL, _IONBF, 0);
 
     /* Minicom, screen, idf_monitor send CR when ENTER key is pressed */
     esp_vfs_dev_uart_port_set_rx_line_endings(CONFIG_ESP_CONSOLE_UART_NUM, ESP_LINE_ENDINGS_CR);
     /* Move the caret to the beginning of the next line on '\n' */
     esp_vfs_dev_uart_port_set_tx_line_endings(CONFIG_ESP_CONSOLE_UART_NUM, ESP_LINE_ENDINGS_CRLF);
 
     /* Configure UART. */
     const uart_config_t uart_config = {
         .baud_rate = CONFIG_ESP_CONSOLE_UART_BAUDRATE,
         .data_bits = UART_DATA_8_BITS,
         .parity = UART_PARITY_DISABLE,
         .stop_bits = UART_STOP_BITS_1,
         .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
         .source_clk = UART_SCLK_DEFAULT,
     };
     ESP_ERROR_CHECK(uart_driver_install(CONFIG_ESP_CONSOLE_UART_NUM, 256, 0, 0, NULL, 0));
     ESP_ERROR_CHECK(uart_param_config(CONFIG_ESP_CONSOLE_UART_NUM, &uart_config));
 
     /* Tell VFS to use UART driver */
     esp_vfs_dev_uart_use_driver(CONFIG_ESP_CONSOLE_UART_NUM);
 
     /* Initialize the console */
     esp_console_config_t console_config = {
         .max_cmdline_length = 256,
         .max_cmdline_args = 8,
 #if CONFIG_LOG_COLORS
         .hint_color = atoi(LOG_COLOR_CYAN),
 #endif
     };
     ESP_ERROR_CHECK(esp_console_init(&console_config));
 
     /* Configure linenoise line completion library */
     /* Enable multiline editing. */
     linenoiseSetMultiLine(1);
 
     /* Clear the screen */
     linenoiseClearScreen();
 }
 
 void app_main(void)
 {
     /* Create mutex for console output protection */
     console_mutex = xSemaphoreCreateMutex();
     
     /* Initialize the ESP Event Loop */
     ESP_ERROR_CHECK(esp_event_loop_create_default());
     
     /* Initialize LD2450 driver with default configuration */
     ld2450_config_t config = LD2450_DEFAULT_CONFIG();
     config.uart_port = UART_NUM_1;  // Use UART1 for the radar
     config.tx_pin = 17;             // Set your pins according to your board
     config.rx_pin = 16;
     config.event_loop_enabled = true; // Make sure events are enabled
     
     ESP_LOGI(TAG, "Initializing LD2450 radar sensor");
     esp_err_t err = ld2450_init(&config);
     if (err != ESP_OK) {
         ESP_LOGE(TAG, "Failed to initialize LD2450: %s", ld2450_err_to_str(err));
         return;
     }
     
     /* Register event handler */
     ESP_ERROR_CHECK(ld2450_register_event_handler(ld2450_event_handler, NULL));
     
     /* Initialize console */
     initialize_console();
     
     /* Register commands */
     esp_console_register_help_command();
     register_commands();
     
     /* Main loop */
     const char* prompt = "ld2450> ";
     
     printf("\n"
            "LD2450 Radar Sensor Test Console\n"
            "Type 'help' to get the list of commands.\n"
            "Use UP/DOWN arrows to navigate through command history.\n"
            "Press TAB when typing command name to auto-complete.\n");
     
     /* Show initial prompt */
     linenoiseClearScreen();
     printf("\n%s", prompt);
     
     /* Main loop */
     while (true) {
         /* Get a line using linenoise.
          * The line is returned when ENTER is pressed.
          */
         char *line = linenoise(prompt);
         if (line == NULL) { /* Ignore empty lines */
             continue;
         }
         
         /* Add the command to the history */
         linenoiseHistoryAdd(line);
         
         /* Try to run the command */
         int ret;
         esp_err_t err = esp_console_run(line, &ret);
         if (err == ESP_ERR_NOT_FOUND) {
             printf("Unknown command: %s\n", line);
         } else if (err == ESP_ERR_INVALID_ARG) {
             // Command was empty
         } else if (err == ESP_OK && ret != ESP_OK) {
             printf("Command returned non-zero error code: 0x%x (%s)\n", ret, esp_err_to_name(ret));
         } else if (err != ESP_OK) {
             printf("Internal error: %s\n", esp_err_to_name(err));
         }
         
         /* linenoise allocates line buffer on the heap, so we need to free it */
         linenoiseFree(line);
         
         /* Check for exit */
         if (ret == EXIT_SUCCESS) {
             break;
         }
     }
     
     /* Cleanup */
     stop_continuous_data();
     ESP_ERROR_CHECK(ld2450_deinit());
     ESP_ERROR_CHECK(esp_console_deinit());
 }