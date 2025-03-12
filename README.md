# LD2450 Basic Example

This example demonstrates the basic functionality of the HLK-LD2450 radar sensor driver for ESP32-IDF.

## Features

- Initialize the LD2450 radar driver
- Configure radar tracking mode
- Receive and process target data
- Output radar data in both JSON and human-readable formats

## Hardware Required

- ESP32 development board
- HLK-LD2450 24GHz radar sensor
- UART connection between ESP32 and LD2450

## Wiring

Connect the LD2450 to your ESP32 board as follows:

| LD2450 Pin | ESP32 Pin | Description      |
|------------|-----------|------------------|
| TX         | GPIO 18   | UART RX (to ESP) |
| RX         | GPIO 17   | UART TX (from ESP) |
| GND        | GND       | Ground           |
| 5V         | 5V        | Power (5V)       |

## Build and Run

1. Configure the project:
   ```
   idf.py menuconfig
   ```

2. Build the project:
   ```
   idf.py build
   ```

3. Flash the firmware and monitor the output:
   ```
   idf.py -p [PORT] flash monitor
   ```

## Expected Output

When running properly, the example will output detected targets in the following format:

```
I (5345) ld2450_example: Radar targets: {"timestamp_ms":5345,"targets":[{"id":0,"x_mm":152,"y_mm":1230,"speed_cmps":5,"distance_m":1.24,"angle_deg":7.1,"speed_mps":0.05,"resolution":180}]}
I (5345) ld2450_example: Detected 1 active targets:
I (5345) ld2450_example:   Target 0: x=152mm, y=1230mm, distance=1.24m, speed=0.05m/s
```

## Customizing

You can modify the following parameters in `main.c`:

- `LD2450_UART_PORT`: UART port number
- `LD2450_UART_TX_PIN`: GPIO pin for UART TX
- `LD2450_UART_RX_PIN`: GPIO pin for UART RX
- `LD2450_UART_BAUD_RATE`: UART baud rate (default: 256000)
- `USE_MULTI_TARGET_MODE`: Set to true for multi-target tracking, false for single target
- `PRINT_INTERVAL_MS`: How often to print target data (in milliseconds)