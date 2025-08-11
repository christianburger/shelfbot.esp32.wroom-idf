# Shelfbot Firmware

This firmware runs on an ESP32-WROOM-32 and serves as the real-time hardware interface for the Shelfbot, a ROS 2 (Humble) based robotic system. It is written in C++ and structured as a set of ESP-IDF components, with the core logic encapsulated in a `Shelfbot` class.

---

## Architecture

The Shelfbot control system is a distributed system composed of three key software components:

1.  **ROS 2 Host Application (`shelfbot`)**: The high-level "brain" of the robot. It runs on a host computer (e.g., a laptop running ROS 2 Humble in a Docker container) and is responsible for navigation, path planning, and high-level control logic.
2.  **micro-ROS Agent**: A standalone application that runs on the host computer and acts as a bridge. It relays messages between the main ROS 2 network and the ESP32 firmware over a UDP-based Wi-Fi connection.
3.  **ESP32 Firmware (This Repository)**: The low-level C++ controller that directly manages the stepper motors and other hardware. It connects to the micro-ROS Agent to join the ROS 2 network.

### Firmware Architecture

The firmware is organized into modular, reusable ESP-IDF components to promote separation of concerns:

*   **`main`**: A minimal entry point. Its sole responsibility is to instantiate the `Shelfbot` class and start the application. This adheres to the ESP-IDF requirement for a C-style `app_main` function while keeping the application logic purely in C++.
*   **`shelfbot`**: The core application component. The `Shelfbot` C++ class within this component orchestrates all major functions: network initialization, mDNS discovery, SNTP time synchronization, and managing the micro-ROS task.
*   **`motor_control`**: A hardware abstraction layer for the stepper motors. It uses the `FastAccelStepper` library to provide a simple API for motor control.
*   **`led_control`**: A simple hardware abstraction for the ESP32's built-in LED, providing a clean C-style interface (`led_control_init`, `led_control_set`).
*   **`wifi_station`**: Manages the Wi-Fi connection.
*   **`http_server`**: Provides a REST API for basic troubleshooting and manual control.

This component-based C++ design makes the system easier to maintain, test, and extend.

```
+---------------------------------------+
|         ROS 2 Host Computer           |
|                                       |
|  +---------------------------------+  |
|  |      `shelfbot` ROS 2 App       |  |
|  +---------------------------------+  |
|                 |                     |
| (ROS 2 Topics)  v                     |
|  +---------------------------------+  |
|  |   micro-ROS Agent (Bridge)      |  |
|  +---------------------------------+  |
|                                       |
+-----------------|---------------------+
                  |
                  | (Wi-Fi / UDP)
                  v
+---------------------------------------+
|          ESP32 Firmware               |
|                                       |
|  +---------------------------------+  |
|  |        `Shelfbot` Class         |  |
|  | (Orchestrates all components)   |  |
|  +---------------------------------+  |
|                 |                     |
|                 v                     |
|  +---------------------------------+  |
|  | Hardware Components (motor, led)|  |
|  +---------------------------------+  |
|                                       |
+---------------------------------------+
```

---

## ROS 2 Interface

The firmware creates a single ROS 2 node named `shelfbot_firmware`.

### Published Topics

| Topic | Message Type | Description |
| :--- | :--- | :--- |
| `/shelfbot_firmware/heartbeat` | `std_msgs/msg/Int32` | A simple counter published every 2 seconds to indicate that the firmware is alive and connected. |

### Subscribed Topics

| Topic | Message Type | Description |
| :--- | :--- | :--- |
| `/shelfbot_firmware/led` | `std_msgs/msg/Bool` | Controls the state of the ESP32's built-in blue LED. `true` turns the LED on, `false` turns it off. |
| `/shelfbot_firmware/motor_command` | `std_msgs/msg/Float32MultiArray` | Commands the motors to specific positions. The `data` field should be an array of target positions in **radians**. |

---

## Testing and Troubleshooting

### Testing Motor Control

This test verifies that you can command the motors to specific positions.

| Action | Command | Expected Result |
| :--- | :--- | :--- |
| **1. Move Motor 0 to 90°** | `ros2 topic pub --once /shelfbot_firmware/motor_command std_msgs/msg/Float32MultiArray "{data: [1.57]}"` | Motor 0 moves to the 90-degree position.<br/>Serial monitor shows: `I (xxxx) shelfbot: Motor 0 command: 1.57 rad` |
| **2. Move Motors 0 & 1** | `ros2 topic pub --once /shelfbot_firmware/motor_command std_msgs/msg/Float32MultiArray "{data: [1.57, -1.57]}"` | Motor 0 moves to +90° and Motor 1 moves to -90°.<br/>Serial monitor shows logs for both motors. |
| **3. Return all to Zero** | `ros2 topic pub --once /shelfbot_firmware/motor_command std_msgs/msg/Float32MultiArray "{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"` | All motors return to their zero position. |

**Troubleshooting:**
*   **Motors do not move:** Verify motor power supply and wiring. Check the serial monitor for any error messages from the `motor_control` or `FastAccelStepper` libraries.
*   **Moves to wrong position:** Ensure your high-level application is sending commands in **radians**, not degrees.

### Testing the LED Control

This test verifies basic two-way communication with the ESP32.

| Action | Command | Expected Result |
| :--- | :--- | :--- |
| **1. Turn LED ON** | `ros2 topic pub --once /shelfbot_firmware/led std_msgs/msg/Bool "data: true"` | The blue LED on the ESP32 module turns ON.<br/>Serial monitor shows: `I (xxxx) shelfbot: LED command received: ON` |
| **2. Turn LED OFF** | `ros2 topic pub --once /shelfbot_firmware/led std_msgs/msg/Bool "data: false"` | The blue LED on the ESP32 module turns OFF.<br/>Serial monitor shows: `I (xxxx) shelfbot: LED command received: OFF` |

**Troubleshooting:**
*   **Command fails or hangs:** Ensure the micro-ROS agent is running and the ESP32 is connected to Wi-Fi.
*   **No serial log message:** Verify the ESP32 is powered and the `idf.py monitor` is running and connected to the correct serial port.

---

## Building the Firmware

This project uses the ESP-IDF build system.

1.  **Set up ESP-IDF**: Ensure you have the ESP-IDF environment configured.
2.  **Configure**: Run `idf.py menuconfig` to set Wi-Fi credentials and other project settings.
3.  **Build**: Run `idf.py build`.
4.  **Flash**: Run `idf.py flash` to upload the firmware to the ESP32.
5.  **Monitor**: Run `idf.py monitor` to view the serial output.
