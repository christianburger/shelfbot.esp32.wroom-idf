# colcon.meta placement

## The problem

`colcon.meta` is a colcon build tool configuration file. It is only read when
colcon is invoked with a workspace root containing that file **at the root of
the workspace** — i.e. the directory that *contains* `src/`, not the
component directory itself.

In the ESP-IDF + micro-ROS workflow the file that actually controls cmake-args
for the micro-ROS precompiled library is **not** `colcon.meta` in the project
root. The micro-ROS ESP-IDF component builds its own colcon workspace
internally. The correct way to pass cmake args is via the `colcon.meta` file
that lives inside the micro-ROS component's own build directory, or — more
practically — by placing it at:

```
<project_root>/managed_components/micro-ros/micro_ros_espidf_component/colcon.meta
```

or at the path:

```
components/micro_ros_espidf_component/colcon.meta
```

…whichever directory is the root of the micro-ROS ESP-IDF component in your
project.

## What actually happens at build time

When you run `idf.py build`, the micro-ROS component's `CMakeLists.txt`
invokes `colcon build` inside its own directory. It looks for `colcon.meta`
relative to **that** directory. A `colcon.meta` at the shelfbot project root
is ignored by that process.

## Recommended fix

Copy (or symlink) `colcon.meta` into the micro-ROS component directory:

```bash
# If using the managed component:
cp colcon.meta managed_components/micro-ros/micro_ros_espidf_component/colcon.meta

# Or if using a local components/ clone:
cp colcon.meta components/micro_ros_espidf_component/colcon.meta
```

Then clean and rebuild:

```bash
idf.py fullclean
idf.py build
```

## Verification

After rebuilding, the serial log emitted by `logMicrorosLimits()` in
`microros_sync.cpp` shows the limits that were compiled in. Cross-check them
against your `colcon.meta` values. If they still show the defaults (e.g.
`MAX_PUBLISHERS=1`) the file is not being picked up.

## Key values in colcon.meta and why they matter for LaserScan publishing

| Parameter | Value | Reason |
|---|---|---|
| `RMW_UXRCE_MAX_MESSAGE_SIZE` | 4096 | LaserScan with 360 floats ≈ 2940 bytes serialised; must be > 2940 |
| `RMW_UXRCE_MAX_PUBLISHERS` | 15 | Need at least 5 (heartbeat, led_state, motor_pos, laser_scan + 1 spare) |
| `RMW_UXRCE_MAX_HISTORY` | 16 | Depth of the XRCE send queue; low values cause pub-failed:1 under load |
