# Shelfbot – ESP32 Robotic Platform with micro‑ROS

Shelfbot is a firmware for ESP32‑based robotic platforms that integrates:
- **5‑axis stepper motor control** (using TMC2209 drivers and FastAccelStepper)
- **Multi‑sensor suite** (ultrasonic, LiDAR, optional ToF)
- **micro‑ROS** (ROS 2 Humble) for low‑latency robot control
- **Wi‑Fi manager** with multi‑AP roaming and RSSI monitoring
- **Embedded web interface** for configuration and monitoring

---

## Repository Structure

| File / Directory       | Description                                                      |
|------------------------|------------------------------------------------------------------|
| `README.md`            | This file – project overview and quick start                    |
| `ENVIRONMENT_SETUP.md` | Complete ESP‑IDF + micro‑ROS + FastAccelStepper environment guide |
| `MOTOR_SETUP.md`       | Stepper motor wiring, TMC2209 configuration, GPIO mapping        |
| `partitions.csv`       | Custom partition table (2 MB factory + 1.9 MB storage)           |
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
   docker run -it --rm --net=host microros/micro-ros-agent:humble udp4 --port 8888 --time -v6
   ```

> **`--time` is required.** Without it the agent silently ignores `rmw_uros_sync_session()` requests and the firmware must rely on SNTP alone for clock synchronisation, which delays the first sensor messages by several seconds.

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
- **SNTP** – The instant a DHCP address is obtained, the Wi-Fi manager automatically starts the SNTP client (`pool.ntp.org`, `time.cloudflare.com`). No manual call is needed.
- **Monitoring** – While connected, checks RSSI every 5 seconds.
- **Roaming** – If RSSI falls below the warning threshold for N consecutive readings, it scans for a better network and switches without disconnecting the ROS interface.

The Wi‑Fi manager exports a thread‑safe status structure (`wifi_manager_info_t`) and two event bits:
- `WM_CONNECTED_BIT` – set when IP address is obtained
- `WM_DISCONNECTED_BIT` – set when not connected

---

## Clock Synchronisation and Timestamped Topics

The firmware must synchronise the ESP32 wall clock before publishing sensor messages with meaningful timestamps. Two paths run in parallel; the first to succeed wins.

**Path A — micro-ROS agent (`rmw_uros_sync_session`):** attempted immediately after the XRCE session is established. Requires the agent to be started with `--time`.

**Path B — SNTP:** started automatically by the Wi-Fi manager as soon as an IP address is obtained (`pool.ntp.org`, `time.cloudflare.com`). Typically settles within 1–3 seconds on a local network.

The firmware polls `ShelfbotTimestamp::isEpochValid()` and waits up to 30 seconds. If neither path succeeds, it continues with monotonic (boot-relative) timestamps for the `laser_scan` header and logs a warning.

### Effect on topic publishing

| Topic | Publishes before sync? | Notes |
|---|---|---|
| `heartbeat` | ✅ Yes | No timestamp in message |
| `led_state` | ✅ Yes | No timestamp in message |
| `motor_positions` | ❌ After sync only | — |
| `distance_sensors` | ❌ After sync only | — |
| `tof_distance` | ❌ After sync only | — |
| `laser_scan` | ❌ After sync only | `header.stamp` = wall clock |

---

## Web Interface

The embedded HTTP server serves a real‑time dashboard for monitoring and control.

### Accessing the web interface

1. After the firmware connects to Wi‑Fi, the console will show the assigned IP address (e.g., `got IP: 192.168.1.100`).
2. Open a web browser and enter `http://shelfbot.local` (mDNS) or the raw IP address.

### Available pages / endpoints

| Endpoint          | Description                                                                 |
|-------------------|-----------------------------------------------------------------------------|
| `/`               | Main dashboard – shows sensor readings, motor positions, LED state         |
| `/health`         | JSON endpoint with system status (Wi‑Fi, heap, uptime, RSSI)               |
| `/wifi_status`    | JSON with current SSID, IP, RSSI, and connection uptime                    |
| `/motor_control`  | Manual motor control (position and velocity sliders)                       |
| `/led`            | Toggle the onboard LED (if configured)                                     |

All pages auto‑refresh every 2 seconds.

---

## Hardware Overview (GPIO Summary)

For full details, see [`MOTOR_SETUP.md`](MOTOR_SETUP.md). Here is a quick reference. **Motor pin assignments are verified against `motor_control.cpp`.**

| Function          | GPIO Pins                              |
|-------------------|----------------------------------------|
| Motors (STEP/DIR) | 13/19, 14/33, 4/18, 27/26, 12/23      |
| Ultrasonic sensors| TRIG: 25, 32, 16, 17 / ECHO: 34, 35, 36, 39 |
| LiDAR (UART2 RX)  | GPIO3 (may conflict with console)      |
| I2C (optional ToF)| SDA=21, SCL=22                         |
| LED (if used)     | Depends on `led_control` component     |

**Important:** GPIO3 is also the default UART0 RX (console). If you need both the console and LiDAR, either move LiDAR to UART1 (pins 9/10) or disable the console on GPIO3 in menuconfig.

---

## ROS 2 Topics (verified against firmware source)

### Published by firmware

All publishers use **RELIABILITY_BEST_EFFORT, DURABILITY_VOLATILE** (ROS 2 `SENSOR_DATA` QoS). Subscribe on the host with the same QoS profile or you will receive no messages.

| Topic | Type | Rate | Notes |
|---|---|---|---|
| `shelfbot_firmware/heartbeat` | `std_msgs/msg/Int32` | 1 Hz | Incrementing counter; publishes immediately, before clock sync |
| `shelfbot_firmware/motor_positions` | `std_msgs/msg/Float32MultiArray` | 10 Hz | 5 motor positions in **radians** (indices 0–4); published only after clock sync |
| `shelfbot_firmware/distance_sensors` | `std_msgs/msg/Float32MultiArray` | 5 Hz | 6 values in **cm**: 4 ultrasonic + 1 ToF + 1 LiDAR min distance; `-1.0` = invalid; after sync |
| `shelfbot_firmware/led_state` | `std_msgs/msg/Bool` | On change | Current LED state; publishes immediately |
| `shelfbot_firmware/tof_distance` | `std_msgs/msg/Float32` | 5 Hz | ToF[0] distance in **metres**; `-1.0` = invalid/unavailable; after sync |
| `shelfbot_firmware/laser_scan` | `sensor_msgs/msg/LaserScan` | 5 Hz | 12 points per LYDSTO packet; `frame_id`=`lidar_frame`; `range_min`=0.02 m, `range_max`=12 m; `header.stamp` = wall-clock epoch; after sync |

### Subscribed by firmware

| Topic | Type | Notes |
|---|---|---|
| `shelfbot_firmware/motor_command` | `std_msgs/msg/Float32MultiArray` | Position targets in radians; first 5 elements used |
| `shelfbot_firmware/set_speed` | `std_msgs/msg/Float32MultiArray` | Velocity targets in rad/s; first 5 elements used |
| `shelfbot_firmware/led` | `std_msgs/msg/Bool` | `true` = LED ON, `false` = LED OFF |

### mDNS services advertised

| Service | Protocol | Port |
|---|---|---|
| `_http` | `_tcp` | 80 |
| `_microros` | `_udp` | 8888 |

Hostname: `shelfbot.local`

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

| Symptom                                        | Likely solution                                                                      |
|------------------------------------------------|--------------------------------------------------------------------------------------|
| `app partition is too small`                   | Flash size not set to 4 MB (see ENVIRONMENT_SETUP.md §1.8)                           |
| micro‑ROS agent connection fails               | Verify UDP agent on port `8888`, mDNS host `shelfbot.local`, Wi‑Fi reachability      |
| Sensor topics silent after CONNECTED           | Agent not started with `--time`; check TIME_SYNC log messages; wait up to 30 s for SNTP |
| `laser_scan` stamps look like seconds-since-boot | Clock sync failed; restart agent with `--time` or verify SNTP reachability          |
| Wi‑Fi does not connect                         | Verify SSID/password in menuconfig; check RSSI thresholds                            |
| Web interface not loading                      | Check IP / `shelfbot.local`; ensure HTTP server started (look for log message)       |
| GPIO3 conflict (LiDAR + console)               | Move LiDAR to another UART or disable console on GPIO3                               |

---

## micro-ROS Lifecycle (State Machine)

The micro-ROS task progresses through these states. The `TIME_SYNC` state is new and runs between entity creation and the first sensor publications.

```
OFF → DISCOVERING → TIME_SYNC → CONNECTED ↔ DISCONNECTED → DISCOVERING
                                    ↕
                                  ERROR
```

| State | Description |
|---|---|
| `OFF` | Initial state before `MicrorosSync::init()` is called |
| `DISCOVERING` | Wi-Fi connected; polling mDNS for the agent host |
| `TIME_SYNC` | Agent session established; running clock sync (agent + SNTP) |
| `CONNECTED` | Clock sync complete; all entities running; timers firing |
| `DISCONNECTED` | Session lost; will transition back to `DISCOVERING` |
| `ERROR` | Unrecoverable error; transitions to `DISCOVERING` after backoff |

---

## micro-ROS Lifecycle Troubleshooting (Logs + Grep)

Use these commands on your host log file (example: `cutecom.log`):

```bash
# 1) High-level lifecycle in order (includes TIME_SYNC step)
grep -nEi "Waiting for Wi-Fi|Wi-Fi ready|Querying mDNS|Agent IP|Creating micro-ROS entities|Entities created|Starting clock sync|Clock synced|Clock sync timed out|micro-ROS ready|time_synced" cutecom.log

# 2) Focus on rcl/rmw errors with surrounding context
grep -nEi -B4 -A6 "rcl|rmw|RCL_RET|failed:" cutecom.log

# 3) Confirm time sync path taken
grep -nEi "Agent ping|rmw_uros_sync_session|Clock synced via|Waiting for SNTP|Clock sync timed out|Proceeding without valid epoch" cutecom.log

# 4) Confirm sensor topic publishing started (only after sync)
grep -nEi "sensor_control_timer: waiting|time_synced=yes|time_synced=no|micro-ROS ready" cutecom.log

# 5) Confirm subscription activity (commands received)
grep -nEi "motor_command|set_speed|led command|led_cb" cutecom.log

# 6) Trace backoff and reconnect behaviour
grep -nEi "Backing off|consecutive spin failures|disconnecting|Destroying|DISCONNECTED|DISCOVERING" cutecom.log
```

### Expected startup sequence

```
1.  [wifi_manager]    Got IP: 192.168.x.x
2.  [wifi_manager]    SNTP client started (pool.ntp.org, time.cloudflare.com)
3.  [MicrorosSync]    Waiting for Wi-Fi...  →  Wi-Fi ready
4.  [MicrorosSync]    Querying mDNS for <host>.local  →  Agent IP: x.x.x.x
5.  [MicrorosSync]    Creating micro-ROS entities  →  Entities created OK
6.  [MicrorosSync]    Starting clock sync (agent first, then SNTP fallback)
7a. [MicrorosSync]    Agent ping OK — attempting rmw_uros_sync_session
    [MicrorosSync]    Clock synced via micro-ROS agent — wall time ...
    OR
7b. [MicrorosSync]    Waiting for SNTP... (200 ms polls)
    [MicrorosSync]    Clock synced via SNTP after N polls — wall time ...
8.  [MicrorosSync]    micro-ROS ready (time_synced=yes)
9.  Sensor topics begin publishing with wall-clock stamps
```

If you see `time_synced=no` at step 8, the firmware is running but all sensor topics remain silent. Check:
- Agent was started with `--time`
- NTP servers are reachable from the ESP32's network
- 30-second SNTP poll window did not expire before DHCP was stable

### Diagnostic tags

| Log tag | Meaning |
|---|---|
| `microros-info-lifecycle` | Successful state transitions (DISCOVERING → TIME_SYNC → CONNECTED) |
| `microros-warn-spin` | Non-OK `rclc_executor_spin_some` return codes |
| `microros-err-lifecycle` | Forced disconnect after 3 consecutive spin failures |
| `microros-err-support` | Support init failure after transport setup |

---

## Further Reading

- [`ENVIRONMENT_SETUP.md`](ENVIRONMENT_SETUP.md) – Detailed environment setup and common pitfalls
- [`MOTOR_SETUP.md`](MOTOR_SETUP.md) – TMC2209 wiring, microstep selection, and pinout diagrams
- [micro‑ROS ESP32 documentation](https://micro.ros.org/docs/tutorials/core/overview/)
- [FastAccelStepper library](https://github.com/gin66/FastAccelStepper)

---

## License

This project is open‑source under the MIT license. See the `LICENSE` file for details.
