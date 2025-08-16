# Shelfbot Calibration Plan

## 1. Overview and Architectural Approach

This document outlines the plan for creating an intrinsic hardware calibration system for the Shelfbot. The primary goal is to characterize and correct for physical inaccuracies in the robot's hardware (e.g., wheel diameter variance, wheel slip, sensor non-linearities) to provide a more accurate and reliable foundation for high-level control and localization.

The calibration system will be implemented as a new, standalone ROS 2 node named `shelfbot_calibration`.

### Rationale for a Separate Node:
*   **Separation of Concerns:** Calibration is a distinct, non-real-time task. This keeps its complex, interactive logic separate from the real-time `ros2_control` hardware interface.
*   **Direct Hardware Control:** The calibration node will bypass the `ros2_control` stack to issue specific, low-level commands directly to the ESP32 firmware topics. This is essential for the precise, repeatable movements required for calibration.
*   **State Management:** The node will manage the state of the interactive calibration process, guiding the user through the required steps via terminal prompts.
*   **Persistent Output:** The node's final output will be a `shelfbot_calibration.yaml` file containing the calculated correction parameters. This file can then be loaded by the main robot launch file.

---

## 2. Odometry Calibration Routine

### Objective
To calculate correction factors for linear and angular movements, compensating for discrepancies between the URDF model and the physical robot, as well as systematic wheel slip.

### ROS 2 Interface
*   **Service:** `/shelfbot_calibration/start_odometry_test`
*   **Service Type:** `std_srvs/Trigger`

### Procedure
This will be a user-guided routine managed by the `shelfbot_calibration` node.

1.  **Initiation:** The user places the robot on a flat surface with at least 2 meters of clear space and places a reference marker on the floor at the robot's starting position. The user then calls the service:
    ```bash
    ros2 service call /shelfbot_calibration/start_odometry_test std_srvs/srv/Trigger
    ```
2.  **Linear Test:**
    *   The node will print: `[INFO] Starting linear calibration. Commanding robot to move forward 1.0 meter.`
    *   The node will publish a command to the `/shelfbot_firmware/motor_command` topic, calculated to move the robot's base exactly 1.0 meter forward based on its current URDF parameters.
    *   Once the movement is complete, the node will print: `Please measure the actual distance the robot traveled in meters and enter the value:`
    *   The user measures the distance with a tape measure and enters the value (e.g., `0.98`).
3.  **Angular Test:**
    *   The node will print: `[INFO] Starting angular calibration. Commanding robot to rotate 360 degrees.`
    *   The node will publish commands to rotate the robot in-place for what it calculates to be exactly 360 degrees.
    *   Once the rotation is complete, the node will print: `Please measure the robot's rotational error in degrees (e.g., +5 for over-rotation, -3 for under-rotation) and enter the value:`
    *   The user measures the error and enters the value (e.g., `-5.5`).
4.  **Calculation and Output:**
    *   The node calculates the correction factors.
    *   It saves the results to `shelfbot_calibration.yaml`.
    *   The node prints: `[INFO] Calibration complete. Results saved to shelfbot_calibration.yaml.`

### Output File (`shelfbot_calibration.yaml`)
```yaml
odometry_correction:
  # Multiply linear commands by this factor. (e.g., 1.0 / 0.98 = 1.0204)
  linear_factor: 1.0204
  # Multiply angular commands by this factor. (e.g., 360.0 / (360.0 - 5.5) = 1.0155)
  angular_factor: 1.0155
```

---

## 3. Distance Sensor Calibration Routine

### Objective
To characterize the response curve of each distance sensor, correcting for non-linearities and offsets in their readings.

### ROS 2 Interface
*   **Service:** `/shelfbot_calibration/start_sensor_test`
*   **Service Type:** `my_interfaces/srv/StartSensorTest` (custom interface needed)
    *   **Request:** `int8 sensor_id`
    *   **Response:** `bool success`

### Procedure
This routine will be run once for each sensor.

1.  **Initiation:** The user calls the service for a specific sensor:
    ```bash
    ros2 service call /shelfbot_calibration/start_sensor_test my_interfaces/srv/StartSensorTest "{sensor_id: 0}"
    ```
2.  **Data Collection Loop:**
    *   The node will subscribe to the `/shelfbot_firmware/distance_sensors` topic.
    *   The node will prompt the user for the first data point: `Place the robot so sensor 0 is exactly 0.1 meters (10cm) from a flat wall, then press Enter.`
    *   When the user presses Enter, the node will record the average of 10 readings from the specified sensor.
    *   The node will repeat this prompt for a predefined set of distances (e.g., 0.2m, 0.3m, 0.5m, 0.8m, 1.0m).
3.  **Calculation and Output:**
    *   After collecting all data points, the node will save the raw data to the calibration file.
    *   The node prints: `[INFO] Sensor 0 calibration complete. Results saved.`

### Output File (`shelfbot_calibration.yaml`)
```yaml
# ... (odometry_correction from above)
distance_sensor_correction:
  sensor_0:
    # Raw values reported by the firmware at each test point
    raw_readings: [105, 212, 318, 525, 840, 1060]
    # The ground-truth distances used for the test
    actual_distances_meters: [0.1, 0.2, 0.3, 0.5, 0.8, 1.0]
  sensor_1:
    # ... (data for the next sensor)
```

### Next Steps
A separate "filter" node will be created that reads this calibration file, subscribes to the raw `/shelfbot_firmware/distance_sensors` topic, applies a correction (e.g., using linear interpolation on the calibration data), and publishes a corrected `/sensors/distance_corrected` topic for use by the rest of the ROS 2 system.
