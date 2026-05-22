# Shelfbot ESP32 Firmware – Complete Hardware & Setup Guide
## 1. Environment Setup (ESP‑IDF + micro‑ROS + FastAccelStepper)
### 1.1 Install ESP‑IDF v5.3

```bash
mkdir -p ~/esp
cd ~/esp
git clone -b v5.3 --recursive https://github.com/espressif/esp-idf.git
cd esp-idf
./install.sh esp32
```

### 1.2 Load Environment (every new terminal)

```bash
. ~/esp/esp-idf/export.sh
idf.py --version   # Must show v5.3
```

### 1.3 Install Python Packages

```bash
pip3 install catkin_pkg lark-parser colcon-common-extensions empy==3.3.4 pyserial setuptools
```

> **Why empy==3.3.4?** Newer versions break ROS 2 message generation.

### 1.4 Create Project

```bash
mkdir -p ~/shelfbot
cd ~/shelfbot
idf.py create-project shelfbot.esp32.wroom-idf
cd shelfbot.esp32.wroom-idf
```

### 1.5 Add micro‑ROS (Humble)

```bash
mkdir -p components
cd components
git clone https://github.com/micro-ROS/micro_ros_espidf_component.git
cd micro_ros_espidf_component
git checkout humble
cd ../..
```

### 1.6 Add FastAccelStepper

```bash
idf.py add-dependency "gin66/fastaccelstepper^1.2.5"
```

### 1.7 Set Target & Clean

```bash
idf.py set-target esp32
rm -rf components/micro_ros_espidf_component/micro_ros_src \
       components/micro_ros_espidf_component/micro_ros_dev \
       build
```

### 1.8 Configure Flash Size (4 MB)

```bash
idf.py menuconfig
```
Navigate to: `Serial flasher config → Flash size → 4 MB` – save & exit.

### 1.9 Custom Partition Table

Create `partitions.csv` in project root:

```csv
# Name,   Type, SubType, Offset,  Size, Flags
nvs,      data, nvs,     ,        0x4000,
otadata,  data, ota,     ,        0x2000,
phy_init, data, phy,     ,        0x1000,
factory,  app,  factory, ,        0x200000,
storage,  data, spiffs,  ,        0x1E0000,
```

In menuconfig:
- `Partition Table` → `Custom partition table CSV`
- `Custom partition table CSV file` → `partitions.csv`

### 1.10 Build, Flash, Monitor

```bash
idf.py build
idf.py -p /dev/ttyUSB0 flash
idf.py monitor
```

### 1.11 Run micro‑ROS Agent (on host PC, UDP over IP)

```bash
docker run -it --rm --net=host microros/micro-ros-agent:humble udp4 --port 8888 -v6
```

> The firmware uses `rmw_uros_options_set_udp_address(..., "8888", ...)`, so transport is **UDP/IP** (not serial).

### 1.12 Full Clean Rebuild

```bash
rm -rf components/micro_ros_espidf_component/micro_ros_src \
       components/micro_ros_espidf_component/micro_ros_dev \
       build managed_components dependencies.lock
idf.py fullclean
idf.py reconfigure
idf.py build
```

---

## 2. Stepper Motor Wiring (TMC2209)

### 2.1 GPIO Mapping (PULSE / DIR)

| Motor | STEP (PULSE) | DIR  |
|-------|--------------|------|
| 0     | GPIO27       | GPIO26|
| 1     | GPIO14       | GPIO33|
| 2     | GPIO13       | GPIO19|
| 3     | GPIO4        | GPIO18|
| 4     | GPIO12       | GPIO23|

### 2.2 TMC2209 Connections

| TMC2209 Pin | Connection                     |
|-------------|--------------------------------|
| VM          | Motor power supply + (24V)     |
| GND (power) | Motor power supply -           |
| VIO         | ESP32 3.3V                     |
| EN          | GND (always enabled)           |
| STEP        | ESP32 STEP pin (see table)     |
| DIR         | ESP32 DIR pin (see table)      |
| MS1, MS2    | GND or 3.3V for microsteps     |
| A1/A2, B1/B2| Stepper motor coils            |

**Microstep selection (standalone):**

| MS1 | MS2 | Microsteps |
|-----|-----|------------|
| GND | GND | 1/8        |
| 3.3V| GND | 1/16       |
| GND | 3.3V| 1/32       |
| 3.3V| 3.3V| 1/64       |

### 2.3 Wiring Diagram

```
ESP32                                 TMC2209 (x5)
------                                -----------
GPIO27 (STEP0)  --------------------> STEP0
GPIO26 (DIR0)   --------------------> DIR0
GPIO14 (STEP1)  --------------------> STEP1
GPIO33 (DIR1)   --------------------> DIR1
GPIO13 (STEP2)  --------------------> STEP2
GPIO19 (DIR2)   --------------------> DIR2
GPIO4  (STEP3)  --------------------> STEP3
GPIO18 (DIR3)   --------------------> DIR3
GPIO12 (STEP4)  --------------------> STEP4
GPIO23 (DIR4)   --------------------> DIR4
3.3V            --------------------> VIO (all)
GND             --------------------> GND (logic)
                    |
                    | (common ground)
                    v
               Power Supply
               24V+  --------> VM (all)
               GND   --------> GND (power)
```

---

## 3. GPIO Pinout Table (Full System)

| Peripheral       | Function        | GPIO   | Notes                          |
|------------------|-----------------|--------|--------------------------------|
| **Motor 0**      | STEP            | 27     | TMC2209 STEP0                  |
|                  | DIR             | 26     | TMC2209 DIR0                   |
| **Motor 1**      | STEP            | 14     |                                |
|                  | DIR             | 33     |                                |
| **Motor 2**      | STEP            | 13     |                                |
|                  | DIR             | 19     |                                |
| **Motor 3**      | STEP            | 4      |                                |
|                  | DIR             | 18     |                                |
| **Motor 4**      | STEP            | 12     |                                |
|                  | DIR             | 23     |                                |
| **Ultrasonic 0** | TRIG            | 25     | HC‑SR04 trigger                |
|                  | ECHO            | 34     | Input only                     |
| **Ultrasonic 1** | TRIG            | 32     |                                |
|                  | ECHO            | 35     | Input only                     |
| **Ultrasonic 2** | TRIG            | 16     |                                |
|                  | ECHO            | 36     | Input only                     |
| **Ultrasonic 3** | TRIG            | 17     |                                |
|                  | ECHO            | 39     | Input only                     |
| **LiDAR**        | UART2 RX        | 3      | LYDSTO LDS02RR (RX only)       |
|                  | UART2 TX        | -      | Not connected                  |
| **I2C (ToF)**    | SDA             | 21     | Disabled at compile time       |
|                  | SCL             | 22     |                                |
| **LED**          | (depends)       | ?      | See `led_control` component    |
| **Power**        | 3.3V            | -      | To VIO of TMC2209              |
|                  | GND             | -      | Common ground with motor PSU   |

> **Important:** GPIO3 is also the default UART0 RX (console). If you need both the console and LiDAR, either move LiDAR to another UART (e.g., UART1 on GPIO9/10) or disable console on GPIO3.

---

## 4. Verification Status

| Component          | Matches Code? | Notes                                    |
|--------------------|---------------|------------------------------------------|
| Motor pins         | ✅ Yes        | Verified against `motor_control.cpp`    |
| Ultrasonic pins    | ✅ Yes        | Verified against `shelfbot.cpp`          |
| LiDAR pin (GPIO3)  | ⚠️ Conflict   | May conflict with console                |
| LED pin            | ❌ Missing    | Check `led_control` for actual GPIO     |
| Environment setup  | ✅ Correct    | Uses UDP micro‑ROS agent, flash size, partition |

**The system is ready for deployment after resolving the LED pin and potential UART conflict.**

---

## 5. ROS 2 Interface Verification (from firmware code)

Node name: `shelfbot_firmware`

### Publishers (ESP32 → ROS 2)

| Topic | Type | Payload details |
|---|---|---|
| `shelfbot_firmware/heartbeat` | `std_msgs/msg/Int32` | Increments every 1 second |
| `shelfbot_firmware/motor_positions` | `std_msgs/msg/Float32MultiArray` | **5 values** (radians), one per motor index 0..4 |
| `shelfbot_firmware/distance_sensors` | `std_msgs/msg/Float32MultiArray` | Up to 6 values (cm): 4 ultrasonic + 1 ToF + 1 LiDAR (`-1.0` if invalid) |
| `shelfbot_firmware/led_state` | `std_msgs/msg/Bool` | Current LED state |
| `shelfbot_firmware/tof_distance` | `std_msgs/msg/Float32` | ToF[0] distance in cm (`-1.0` if invalid/unavailable) |

### Subscriptions (ROS 2 → ESP32)

| Topic | Type | Payload details |
|---|---|---|
| `shelfbot_firmware/motor_command` | `std_msgs/msg/Float32MultiArray` | Position commands in radians; consumes up to first 5 elements |
| `shelfbot_firmware/set_speed` | `std_msgs/msg/Float32MultiArray` | Velocity commands in rad/s; consumes up to first 5 elements |
| `shelfbot_firmware/led` | `std_msgs/msg/Bool` | `true` = LED ON, `false` = LED OFF |
