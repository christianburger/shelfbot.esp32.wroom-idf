# Shelfbot Firmware

This firmware runs on an ESP32-WROOM-32 and serves as the real-time hardware interface for the Shelfbot, a ROS 2 (Humble) based robotic system. It is written in C++ and structured as a set of ESP-IDF components, with the core logic encapsulated in a `Shelfbot` class.

The firmware uses the micro-ROS framework to communicate with a main ROS 2 application, receiving motion commands and sending back motor state.

---

## System Architecture

The Shelfbot control system is composed of three distinct software components that work together:

1.  **ROS 2 Application (`shelfbot`)**: This is the high-level "brain" of the robot. It runs on a host computer (e.g., a laptop running ROS 2 Humble) and includes navigation, path planning, and high-level control logic.
2.  **micro-ROS Agent**: A standalone application that runs on the host computer and acts as a bridge, relaying messages between the main ROS 2 network and the ESP32 over a UDP-based Wi-Fi connection.
3.  **ESP32 Firmware (This Repository)**: The low-level C++ controller that directly manages the stepper motors and other hardware. It connects to the micro-ROS Agent to join the ROS 2 network.

### Firmware Architecture

The firmware is organized into ESP-IDF components. The `main` component serves only as a minimal entry point, instantiating and running the `Shelfbot` class. The core application logic, including network setup, ROS communication, and hardware initialization, resides within the `shelfbot` component.

```
+---------------------------------------+
|         ROS 2 Host Computer           |
|                                       |
|  +---------------------------------+  |
|  |      `shelfbot` ROS 2 App       |  |
|  |  (Commands & States in RADIANS)   |  |
|  +---------------------------------+  |
|                 |                     |
| (ROS 2 Topics)  |                     |
|                 v                     |
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
|  | (Manages ROS Comms & Init)      |  |
|  +---------------------------------+  |
|                 |                     |
|                 v                     |
|  +---------------------------------+  |
|  | Hardware Drivers (motor_control,|  |
|  | wifi_station, http_server, etc.)|  |
|  +---------------------------------+  |
|                                       |
+---------------------------------------+
```

---

## Initialization and Execution Sequence

1.  **Agent Startup**: The `micro-ROS Agent` is started on the host computer.
2.  **Firmware Boot**: The ESP32 powers on. The ESP-IDF framework initializes the system and starts the main task.
3.  **`app_main` Entry Point**: The C-style `app_main` function (the required entry point for ESP-IDF) is called. Its sole purpose is to instantiate the `Shelfbot` C++ object and call its `begin()` method.
4.  **`Shelfbot::begin()`**: This method orchestrates all initialization:
    *   Initializes NVS flash, networking (TCP/IP and event loop).
    *   Calls `motor_control_begin()` to configure the stepper motor hardware.
    *   Calls `wifi_init_sta()` to connect to the configured Wi-Fi network.
    *   Initializes SNTP for time synchronization.
    *   Starts the mDNS service to discover the micro-ROS agent on the network.
    *   Starts a simple web server for REST API-based troubleshooting.
5.  **micro-ROS Task**: Once the agent is found, `Shelfbot::begin()` creates a new FreeRTOS task (`micro_ros_task`) dedicated to handling all micro-ROS communication.
6.  **Main Loop**: The `Shelfbot::begin()` method enters an infinite `while(true)` loop. This keeps the main application task alive, which is standard practice for embedded systems. This loop can be used for other periodic, non-ROS tasks in the future.

---

## Building the Firmware

This project uses the ESP-IDF build system.

1.  **Set up ESP-IDF**: Ensure you have the ESP-IDF environment configured.
2.  **Configure**: Run `idf.py menuconfig` to set Wi-Fi credentials and other project settings.
3.  **Build**: Run `idf.py build`.
4.  **Flash**: Run `idf.py flash` to upload the firmware to the ESP32.
5.  **Monitor**: Run `idf.py monitor` to view the serial output.
