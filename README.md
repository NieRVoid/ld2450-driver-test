# HLK-LD2450 Basic Operation Example

This example demonstrates the basic functionality of the HLK-LD2450 driver component for ESP-IDF.

## Overview

The example initializes the radar sensor driver, registers a callback function for target detection events, and displays information about detected targets including:

- Position (X,Y) in millimeters
- Movement speed in cm/s
- Calculated distance in millimeters
- Calculated angle in degrees

## Hardware Required

To run this example, you need:

* An ESP32 development board
* A HLK-LD2450 24GHz radar module
* Connection cables

## Connection

By default, the example uses the following pins for UART communication with the radar:

| ESP32 Pin | LD2450 Pin |
|-----------|------------|
| GPIO 16   | RXD        |
| GPIO 17   | TXD        |
| GND       | GND        |
| 3.3V      | VCC        |

You can modify the pin configuration in the code if needed.

## How to Use

### Build and Flash

Run the following commands to build and flash the example:

```bash
idf.py build
idf.py -p PORT flash monitor
```

Replace `PORT` with your ESP32's serial port (e.g., `/dev/ttyUSB0` on Linux or `COM3` on Windows).

### Example Output

When the example runs successfully, you should see output similar to the following:

```
I (341) ld2450_example: Starting HLK-LD2450 basic example
I (351) LD2450: LD2450 driver initialized on UART2 (RX: GPIO16, TX: GPIO17, baud: 256000)
I (361) LD2450: Auto-processing enabled with task priority 5
I (371) ld2450_example: LD2450 radar initialized successfully
I (381) LD2450: Entered configuration mode
I (431) LD2450: Firmware version: V1.02.25031210
I (441) LD2450: Exited configuration mode
I (451) ld2450_example: Radar firmware version: V1.02.25031210
I (461) LD2450: Entered configuration mode
I (471) LD2450: Exited configuration mode
I (481) ld2450_example: Waiting for target detection events...
I (2481) ld2450_example: Target frame received: 0 targets detected
I (3481) ld2450_example: Target frame received: 1 targets detected
I (3481) ld2450_example: Target 1: position (x=-358, y=1251) mm, speed=12 cm/s, distance=1302.3 mm, angle=15.9°

```

When you move in front of the radar, you will see the target information change accordingly.

## Troubleshooting

If you don't see any target detection events:
- Make sure the radar is connected correctly
- Check that the radar has a clear field of view
- Verify that the UART pins are configured correctly
- Try moving around in front of the radar to trigger detection

## Additional Features

This example demonstrates only the basic functionality. The driver also supports:

- Region filtering (include-only or exclude specific areas)
- Single target or multi-target tracking modes
- Bluetooth configuration
- Factory reset and module restart
- Baud rate configuration

Check the driver API documentation for more information.