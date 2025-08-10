# Shelfbot Firmware

This firmware is designed for an ESP32-WROOM-32 microcontroller and acts as a bridge between the robot's hardware (motors, sensors) and a ROS 2 (Humble) control system. It uses the micro-ROS framework to communicate with a main ROS 2 application.

## Key Features

- **micro-ROS Node**: Exposes the robot's functionality as a standard ROS 2 node.
- **Wi-Fi Connectivity**: Connects to a local Wi-Fi network to communicate with the ROS 2 system.
- **mDNS Discovery**: Automatically discovers the IP address of the micro-ROS agent on the network, eliminating the need for static IP configurations.
- **Motor Control**: Integrates with a `motor_control` component to manage the robot's motors.
- **REST Endpoint**: Provides a simple HTTP endpoint for basic connectivity troubleshooting.

---

## System Architecture

The system consists of two main parts:

1.  **Host Computer**: A computer running ROS 2 Humble (typically inside a Docker container) which contains:
    *   The main `shelfbot` ROS 2 application (in `~/shelfbot_workspace`).
    *   The `micro-ROS Agent`, which acts as a bridge between the full ROS 2 world and the microcontroller.

2.  **ESP32 Firmware**: This repository. It runs on the ESP32, connects to Wi-Fi, finds the agent via mDNS, and then communicates with it to send and receive ROS 2 messages.

---

## Setup and Installation

This guide assumes you have a working ESP-IDF v5.3 environment and a Docker container with ROS 2 Humble.

### 1. Host Setup and Operation

These steps are performed on your host machine (e.g., Gentoo Linux) to run the Docker container and the micro-ROS agent.

1.  **Start the Docker Container**:
    Open a terminal and start your ROS 2 container.

    ```bash
    # Start the container interactively
    docker start -i ros2_humble_shelfbot_container
    ```

2.  **Enter the Docker Container**:
    If the above command doesn't automatically attach, open a new terminal and run:
    ```bash
    docker exec -it ros2_humble_shelfbot_container /bin/bash
    ```

3.  **Initialize the Environment**:
    Inside the container, run your initialization script. This script sources the main ROS 2 environment and your `shelfbot` application workspace.
    ```bash
    # This script should source both /opt/ros/humble/setup.bash and ~/shelfbot_workspace/install/setup.bash
    init.sh
    ```

4.  **Build and Run the micro-ROS Agent**:
    The agent must be running for the firmware to connect.

    ```bash
    # Navigate to the micro-ROS agent workspace
    cd ~/microros_ws/

    # Build the agent (only needs to be done once)
    colcon build

    # IMPORTANT: Source the agent's workspace to find its executable
    source install/setup.bash

    # Run the agent using UDP on port 8888
    ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
    ```
    The agent will now be running and waiting for the ESP32 to connect.

### 2. Firmware Setup

These steps are for the ESP32 firmware in this repository.

1.  **Clone the Repository**:
    Clone this repository onto your machine where you have the ESP-IDF installed.

2.  **Fix Python Dependencies**:
    The micro-ROS build system requires a specific version of the `empy` Python package. The default version installed by ESP-IDF is incompatible. You must force the correct version.

    ```bash
    # Activate the ESP-IDF environment
    . $IDF_PATH/export.sh

    # Force the installation of the correct empy version
    pip3 install --ignore-installed empy==3.3.4
    ```

3.  **Configure Wi-Fi Credentials**:
    Use the ESP-IDF configuration menu to set your Wi-Fi SSID and password.

    ```bash
    idf.py menuconfig
    ```
    Navigate to `Component config` -> `WiFi Configuration` and set your credentials. Save and exit.

4.  **Build and Flash**:
    Compile the firmware and flash it to your connected ESP32.

    ```bash
    idf.py flash
    ```

5.  **Monitor**:
    View the serial output from the ESP32.

    ```bash
    idf.py monitor
    ```

---

## Runtime Behavior

1.  The ESP32 will connect to your Wi-Fi network.
2.  It will then use mDNS to find the IP address of the host named `gentoo-laptop`.
3.  Once found, it will connect to the micro-ROS agent. The agent's terminal will show a "Client connected" message.
4.  The ESP32's monitor will print `Node is alive...` every second.

## Troubleshooting

- **`Package 'micro_ros_agent' not found`**: You forgot to run `source install/setup.bash` inside the `~/microros_ws` directory before running the agent.
- **Firmware fails with `mDNS host not found`**: Ensure your host computer's hostname is `gentoo-laptop` and that the Avahi daemon (or equivalent mDNS service) is running.
- **REST Endpoint**: You can check if the ESP32 is alive and on the network by sending a GET request to its IP address.
  ```bash
  # Find the ESP32's IP from the idf.py monitor log
  curl http://<ESP32_IP_ADDRESS>/
  ```
  It should return `"Hello from Shelfbot Firmware!"`.
