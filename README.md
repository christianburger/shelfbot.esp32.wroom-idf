# ShelfBot Firmware

This firmware controls ShelfBot, a multi-axis robotic system running on an ESP32. It hosts a web-based user interface for real-time motor control and provides a JSON-based API for programmatic access.

The project is built using the ESP-IDF framework.

## Core Functionality

- **HTTP Server**: Provides a web interface and a RESTful API for controlling the robot.
- **WiFi Station**: Connects the ESP32 to a local WiFi network.
- **Motor Control**: Manages up to 6 stepper motors using the FastAccelStepper library.

---

## Communication Flow & GPIO Details

The system is controlled via HTTP requests, which are processed by the ESP32. The firmware then translates these commands into low-level hardware signals to drive the stepper motors.

**Frontend (Web Browser) -> Backend (ESP32 HTTP Server) -> Motor Control (GPIO)**

### GPIO Pinout

The firmware is configured to control 6 motors. Each motor requires a STEP pin and a DIRECTION pin.

| Motor Index | STEP Pin | DIRECTION Pin |
| :---------- | :------- | :------------ |
| 0           | GPIO 27  | GPIO 26       |
| 1           | GPIO 14  | GPIO 12       |
| 2           | GPIO 13  | GPIO 15       |
| 3           | GPIO 4   | GPIO 16       |
| 4           | GPIO 17  | GPIO 5        |
| 5           | GPIO 18  | GPIO 19       |

---

## API Commands

The server provides several endpoints to monitor and control the motors.

### GET `/`

- **Description**: Serves the main `index.html` file, which contains the user interface for controlling the robot.
- **`curl` Example**:
  ```bash
  curl http://<ESP32_IP_ADDRESS>/
  ```

### GET `/status`

- **Description**: Retrieves a detailed status of all motors, including their current position, velocity, and running state.
- **Response Body**: A JSON object containing an array of motor statuses.
  ```json
  {
    "motors": [
      {"index": 0, "position": 1500.0, "velocity": 0.0, "running": false},
      {"index": 1, "position": -200.0, "velocity": 0.0, "running": false}
    ]
  }
  ```
- **`curl` Example**:
  ```bash
  curl http://<ESP32_IP_ADDRESS>/status
  ```

### GET `/motor_positions`

- **Description**: Retrieves just the current positions of all motors.
- **Response Body**: A JSON object containing an array of motor positions.
  ```json
  {
    "positions": [1500.0, -200.0, 0.0, 0.0, 0.0, 0.0]
  }
  ```
- **`curl` Example**:
  ```bash
  curl http://<ESP32_IP_ADDRESS>/motor_positions
  ```

### GET `/motor_velocities`

- **Description**: Retrieves the current velocities of all motors.
- **Response Body**: A JSON object containing an array of motor velocities.
  ```json
  {
    "velocities": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  }
  ```
- **`curl` Example**:
  ```bash
  curl http://<ESP32_IP_ADDRESS>/motor_velocities
  ```

### POST `/motor`

- **Description**: Moves a single, specified motor to a target position (in degrees).
- **Request Body**: A JSON object specifying the motor index and target position.
  ```json
  {"motor": 0, "position": 90.5}
  ```
- **`curl` Example**:
  ```bash
  curl -X POST -H "Content-Type: application/json" -d '''{"motor": 0, "position": 90.5}''' http://<ESP32_IP_ADDRESS>/motor
  ```

### POST `/motors`

- **Description**: Moves all motors to their respective target positions simultaneously. This is the primary endpoint for coordinated movement.
- **Request Body**: A JSON object containing an array of target positions, a target speed, and a flag for non-blocking movement.
  - `positions`: An array of target positions (in degrees). The array size must match the number of motors.
  - `speed`: The desired speed for the movement.
  - `nonBlocking`: If `true`, the HTTP response is sent immediately while the motors move in the background. If `false`, the server waits for all motors to finish moving before responding.
- **`curl` Example**:
  ```bash
  curl -X POST -H "Content-Type: application/json" -d '''{"positions": [90.5, -45.0, 0, 0, 0, 0], "speed": 4000, "nonBlocking": true}''' http://<ESP32_IP_ADDRESS>/motors
  ```
