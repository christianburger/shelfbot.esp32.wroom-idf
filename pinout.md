# Shelfbot ESP32 Pinout

This document reflects the **current firmware allocation** in this repository.

## Allocated pins

| GPIO | Direction | Function | Module |
|---|---|---|---|
| 2 | Output | Onboard status LED | `led_control` |
| 4 | Output | Motor 3 STEP | `motor_control` |
| 5 | Output | Ultrasonic #1 TRIG | `sensor_control` |
| 13 | Output | Motor 2 STEP | `motor_control` |
| 14 | Output | Motor 1 STEP | `motor_control` |
| 16 | Output | Motor 3 DIR | `motor_control` |
| 17 | Output | Motor 4 STEP | `motor_control` |
| 18 | Input | LiDAR UART RX (LiDAR TX -> ESP32) | `sensor_control` / `lydsto_driver` |
| 19 | Output/PWM | LiDAR PWM control | `sensor_control` / `lydsto_driver` |
| 21 | BiDir | I2C SDA (ToF bus; feature-flag controllable) | `sensor_control` |
| 22 | BiDir | I2C SCL (ToF bus; feature-flag controllable) | `sensor_control` |
| 23 | Output | Motor 4 DIR | `motor_control` |
| 25 | Output | Motor 2 DIR (also ToF XSHUT when enabled) | `motor_control` / `sensor_control` |
| 26 | Output | Motor 0 DIR | `motor_control` |
| 27 | Output | Motor 0 STEP | `motor_control` |
| 32 | Output | Ultrasonic #0 TRIG | `sensor_control` |
| 33 | Output | Motor 1 DIR | `motor_control` |
| 34 | Input only | Ultrasonic #0 ECHO | `sensor_control` |
| 35 | Input only | Ultrasonic #1 ECHO | `sensor_control` |

## Reserved / special-purpose pins

| GPIO | Reason |
|---|---|
| 6-11 | Connected to SPI flash on ESP32-WROOM modules (do not use) |
| 1, 3 | Primary UART0 console/boot logging |
| 34-39 | Input-only pins |

## Available pins (not currently allocated in firmware)

The following pins are currently not used by this firmware pin map:

- **GPIO0** (strap pin; use with care)
- **GPIO12** (strap pin; use with care)
- **GPIO15** (strap pin; use with care)

All other commonly available GPIOs on this board are currently allocated by active modules.
