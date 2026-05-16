# MOTOR_SETUP.md – Stepper Motor Wiring & Configuration (TMC2209)

This document describes the GPIO allocation for 5 stepper motors and provides clear steps to connect **TMC2209** drivers to the ESP32.

---

## 1. GPIO Pin Mapping (PULSE / DIR)

The motor control component (`motor_control.cpp`) uses the following pins:

| Motor | STEP (PULSE) | DIR  |
|-------|--------------|------|
| 0     | GPIO27       | GPIO26|
| 1     | GPIO14       | GPIO33|
| 2     | GPIO13       | GPIO19|
| 3     | GPIO4        | GPIO18|
| 4     | GPIO12       | GPIO23|

> **Note:** No dedicated `ENABLE` pins are configured – the drivers stay enabled when power is applied. If you need enable control, assign an extra GPIO per motor and modify the code.

---

## 2. TMC2209 Driver Pinout

Typical TMC2209 breakout board (e.g., from BigTreeTech or FYSETC):

| Pin Label | Function            | Connection to ESP32 / Power |
|-----------|---------------------|-----------------------------|
| **VM**    | Motor power (8‑28V) | Power supply positive (+)   |
| **GND**   | Motor power ground  | Power supply negative (-)   |
| **VIO**   | Logic voltage (3.3V)| ESP32 3.3V pin              |
| **EN**    | Enable (active low) | GND (permanently enabled) or ESP32 GPIO (optional) |
| **STEP**  | Step input          | ESP32 STEP pin (see table)  |
| **DIR**   | Direction input     | ESP32 DIR pin (see table)   |
| **MS1**   | Microstep selection | GND or VIO (standalone)     |
| **MS2**   | Microstep selection | GND or VIO (standalone)     |
| **DIAG**  | Diagnostic output   | Not connected (optional)    |
| **UART**  | Single‑wire UART    | ESP32 RX/TX (UART mode only)|

---

## 3. Wiring Steps (Standalone Mode – Recommended for simplicity)

In standalone mode, you set microsteps with jumpers on **MS1** and **MS2**. The driver does **not** need a UART connection.

### 3.1 Power Connections (per driver)

- Connect **VM** to the positive terminal of your motor power supply (e.g., 24V).
- Connect **GND** (power ground) to the negative terminal of the power supply.
- Connect **VIO** to **ESP32 3.3V** (all drivers can share the same 3.3V rail).

### 3.2 Step & Direction

- **STEP** → corresponding ESP32 GPIO (from pin mapping table)
- **DIR**  → corresponding ESP32 GPIO (from pin mapping table)

> ⚠️ **Common ground:** Connect the **GND** of the ESP32 to the **GND** of the motor power supply. This is mandatory for correct signal levels.

### 3.3 Enable (optional)

- If you want the driver always enabled, connect **EN** pin to **GND**.
- If you want software enable/disable, connect **EN** to an unused ESP32 GPIO and modify the firmware (call `stepper->setEnablePin(pin)` and `setAutoEnable(true)`).

### 3.4 Microstep Configuration (Standalone Mode)

Set **MS1** and **MS2** according to the table below. Connect each pin either to **VIO** (high / logic 1) or **GND** (low / logic 0).

| MS1 | MS2 | Microsteps |
|-----|-----|------------|
| 0   | 0   | 1/8 (default) |
| 1   | 0   | 1/16        |
| 0   | 1   | 1/32        |
| 1   | 1   | 1/64        |

*(0 = GND, 1 = VIO)*

### 3.5 Motor Coil Connections

Connect the two coils of your stepper motor to **A1/A2** and **B1/B2** (order affects direction – you can swap one coil if the motor turns the wrong way).

---

## 4. Wiring Diagram Summary (5 motors)
