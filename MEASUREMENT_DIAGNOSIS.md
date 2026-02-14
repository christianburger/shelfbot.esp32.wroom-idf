# Measurement Pipeline Diagnosis

## What the boot log proves

The provided serial log shows that sensor initialization **succeeds**:

- `SensorControl` initializes 4 ultrasonic sensors.
- `TofSensor` initializes all 3 ToF sensors successfully.
- Continuous reading is started (`Continuous reading started`).

So measurement acquisition is started in firmware.

## Why measurements look like they are "not happening"

The firmware only publishes measurements to ROS after the micro-ROS agent is reachable.

Current flow:

1. `SensorManager::start()` starts the local background sensor task.
2. ROS publishing happens in `distance_sensors_timer_callback()` / `tof_timer_callback()`.
3. Those timer callbacks run only while `micro_ros_task_impl()` is in `AGENT_CONNECTED` and executor spinning.
4. Transition to `AGENT_CONNECTED` depends on successful mDNS lookup.

In the provided log, mDNS repeatedly fails:

- `mDNS host not found!`

Therefore the state machine remains in `WAITING_AGENT`, executor is not spun, and no ROS sensor messages are published.

## Code locations that match the log behavior

- Sensor init/start path: `Shelfbot::begin()` initializes `SensorManager` and starts it before micro-ROS task.
- mDNS gating path: `Shelfbot::micro_ros_task_impl()` only creates ROS entities after `query_mdns_host("gentoo-laptop")` succeeds.
- mDNS failure behavior: `query_mdns_host()` returns false and logs "mDNS host not found!".
- ROS sensor publishing path: timer callbacks publish only via executor in connected state.

## Practical fixes

- Ensure ROS agent host is discoverable via mDNS as `gentoo-laptop.local` on same network.
- Or add deterministic fallback (static agent IP / configurable hostname) when mDNS fails.
- Optionally add periodic sensor debug logs from `SensorControl` to verify local measurement values independent of ROS transport.
