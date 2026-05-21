
# Shelfbot – ESP32 Robotic Platform with micro‑ROS

Shelfbot is a firmware for ESP32‑based robotic platforms that integrates:
- **5‑axis stepper motor control** (using TMC2209 drivers and FastAccelStepper)
- **Multi‑sensor suite** (ultrasonic, LiDAR, optional ToF)
- **micro‑ROS** (ROS 2 Humble) for low‑latency robot control
- **Wi‑Fi manager** with multi‑AP roaming and RSSI monitoring
- **Embedded web interface** for configuration and monitoring

This document gives an overview of the project and points to detailed setup guides.

---

## Repository Structure

| File / Directory       | Description                                                      |
|------------------------|------------------------------------------------------------------|
| `README.md`            | This file – project overview and quick start                    |
| `ENVIRONMENT_SETUP.md` | Complete ESP‑IDF + micro‑ROS + FastAccelStepper environment guide |
| `MOTOR_SETUP.md`       | Stepper motor wiring, TMC2209 configuration, GPIO mapping        |
| `partitions.csv`       | Custom partition table (2 MB factory + 1.9 MB storage)           |
| `main/`                | Application entry point (`app_main`)                             |
| `components/`          | Custom components (shelfbot, wifi_manager, motor_control, etc.)  |

---

## Quick Start

1. **Follow the environment setup** – See [`ENVIRONMENT_SETUP.md`](ENVIRONMENT_SETUP.md) for installing ESP‑IDF v5.3, micro‑ROS (Humble), and FastAccelStepper.

2. **Configure Wi‑Fi credentials** – Use menuconfig (see section below).

3. **Wire the hardware** – Refer to [`MOTOR_SETUP.md`](MOTOR_SETUP.md) for TMC2209 connections and GPIO mapping.

4. **Build & flash**:
   ```bash
   idf.py build
   idf.py -p /dev/ttyUSB0 flash monitor
   ```

5. **Start the micro‑ROS agent** (on your PC):
   ```bash
   docker run -it --rm --net=host microros/micro-ros-agent:humble udp4 --port 8888 -v6
   ```

---

## Wi‑Fi Configuration (Menuconfig)

The firmware supports up to **4 Wi‑Fi networks** (SSID/password pairs). At boot, it scans for visible networks and connects to the one with the strongest signal. It also monitors RSSI and switches to a better network when signal degrades.

### How to set credentials

1. Run menuconfig:
   ```bash
   idf.py menuconfig
   ```

2. Navigate to:
   ```
   Wi-Fi Configuration
   ```

3. Fill in your SSID and password for each slot (up to 4). Leave a slot empty to disable it.

4. Optionally adjust RSSI thresholds and retry parameters.

5. Save & exit.

> **Note:** The credentials are stored in the firmware at compile time. To change them, re‑run menuconfig and rebuild.

### How the Wi‑Fi manager works

- **Scanning** – Periodically scans for known SSIDs.
- **Connection** – Tries each visible network (strongest first) with retries.
- **Monitoring** – While connected, checks RSSI every 5 seconds.
- **Roaming** – If RSSI falls below the warning threshold for N consecutive readings, it scans for a better network and switches without disconnecting the ROS interface.

The Wi‑Fi manager exports a thread‑safe status structure (`wifi_manager_info_t`) and two event bits:
- `WM_CONNECTED_BIT` – set when IP address is obtained
- `WM_DISCONNECTED_BIT` – set when not connected

---

## Web Interface

The embedded HTTP server serves a real‑time dashboard for monitoring and control.

### Accessing the web interface

1. After the firmware connects to Wi‑Fi, the console will show the assigned IP address (e.g., `got IP: 192.168.1.100`).
2. Open a web browser and enter:
   ```
   http://192.168.1.100
   ```
   (replace with your ESP32’s IP)

### Available pages / endpoints

| Endpoint          | Description                                                                 |
|-------------------|-----------------------------------------------------------------------------|
| `/`               | Main dashboard – shows sensor readings, motor positions, LED state         |
| `/health`         | JSON endpoint with system status (Wi‑Fi, heap, uptime, RSSI)               |
| `/wifi_status`    | JSON with current SSID, IP, RSSI, and connection uptime                    |
| `/motor_control`  | Manual motor control (position and velocity sliders)                       |
| `/led`            | Toggle the onboard LED (if configured)                                     |

All pages auto‑refresh every 2 seconds. The web server uses a responsive design that works on desktops and mobile phones.

> **Note:** The web interface communicates with the firmware via HTTP REST calls. It does **not** use ROS; it is a separate monitoring/configuration tool.

---

## Hardware Overview (GPIO Summary)

For full details, see [`MOTOR_SETUP.md`](MOTOR_SETUP.md). Here is a quick reference:

| Function          | GPIO Pins                            |
|-------------------|--------------------------------------|
| Motors (STEP/DIR) | 27/26, 14/33, 13/19, 4/18, 12/23    |
| Ultrasonic sensors| TRIG: 25,32,16,17 / ECHO: 34,35,36,39|
| LiDAR (UART2 RX)  | GPIO3 (may conflict with console)    |
| I2C (optional ToF)| SDA=21, SCL=22                       |
| LED (if used)     | Depends on `led_control` component   |

**Important:** GPIO3 is also the default UART0 RX (console). If you need both the console and LiDAR, either move LiDAR to UART1 (pins 9/10) or disable the console on GPIO3 (change UART console to another UART in menuconfig).

---

## ROS 2 Topics (verified)

### Published by firmware

| Topic | Type | Notes |
|---|---|---|
| `shelfbot_firmware/heartbeat` | `std_msgs/msg/Int32` | 1 Hz heartbeat counter |
| `shelfbot_firmware/motor_positions` | `std_msgs/msg/Float32MultiArray` | 5 motor positions in radians (indices 0..4) |
| `shelfbot_firmware/distance_sensors` | `std_msgs/msg/Float32MultiArray` | Sensor distances in cm: 4 ultrasonic + 1 ToF + 1 LiDAR |
| `shelfbot_firmware/led_state` | `std_msgs/msg/Bool` | Current LED state |
| `shelfbot_firmware/tof_distance` | `std_msgs/msg/Float32` | ToF[0] in cm (`-1` when invalid) |

### Subscribed by firmware

| Topic | Type | Notes |
|---|---|---|
| `shelfbot_firmware/motor_command` | `std_msgs/msg/Float32MultiArray` | Position targets in radians; first 5 entries are used |
| `shelfbot_firmware/set_speed` | `std_msgs/msg/Float32MultiArray` | Velocity targets in rad/s; first 5 entries are used |
| `shelfbot_firmware/led` | `std_msgs/msg/Bool` | LED ON/OFF command |

---

## Building with Custom Wi‑Fi Credentials

If you prefer to hard‑code credentials (not recommended), edit `components/wifi_manager/wifi_manager.cpp` and modify the `s_creds` table:

```cpp
static const cred_t s_creds[] = {
    { "MyHomeSSID",   "MyPassword" },
    { "OfficeWiFi",   "OfficePass" },
    // up to 4 networks
};
```

Then rebuild.

---

## Troubleshooting

| Symptom                           | Likely solution                                                      |
|-----------------------------------|----------------------------------------------------------------------|
| `app partition is too small`      | Flash size not set to 4 MB (see ENVIRONMENT_SETUP.md step 1.8)       |
| micro‑ROS agent connection fails  | Check that the agent uses `humble` and the correct serial port       |
| Wi‑Fi does not connect            | Verify SSID/password in menuconfig; check RSSI thresholds            |
| Web interface not loading         | Check IP address; ensure HTTP server started (look for log message)  |
| GPIO3 conflict (LiDAR + console)  | Move LiDAR to another UART or disable console on GPIO3               |

---

## Further Reading

- [`ENVIRONMENT_SETUP.md`](ENVIRONMENT_SETUP.md) – Detailed environment setup and common pitfalls
- [`MOTOR_SETUP.md`](MOTOR_SETUP.md) – TMC2209 wiring, microstep selection, and pinout diagrams
- [micro‑ROS ESP32 documentation](https://micro.ros.org/docs/tutorials/core/overview/)
- [FastAccelStepper library](https://github.com/gin66/FastAccelStepper)

---

## License
This project is open‑source under the MIT license. See the `LICENSE` file for details.
