# micro-ROS ESP32 Setup Guide

## Prerequisites

- ESP-IDF v5.3 installed and sourced
- `micro_ros_espidf_component` added as a component in your project

---

## 1. Source ESP-IDF and install dependencies

```bash
. $IDF_PATH/export.sh
pip3 install catkin_pkg lark-parser colcon-common-extensions
```

---

## 2. Switch the component to the Humble branch

The `rolling` branch of `micro_ros_espidf_component` clones bleeding-edge ROS 2 sources that may break with the ESP32 toolchain (e.g. `rosidl_buffer` using C++ exceptions which are disabled by default). Use the Humble LTS branch instead.

```bash
cd components/micro_ros_espidf_component
git fetch
git checkout humble
```

---

## 3. Clean previous build artifacts

If you've attempted a build already, remove stale micro-ROS source and dev trees before rebuilding:

```bash
cd ~/shelfbot/shelfbot.esp32.wroom-idf
rm -rf components/micro_ros_espidf_component/micro_ros_src \
       components/micro_ros_espidf_component/micro_ros_dev \
       build
```

---

## 4. Build from the project root

> ⚠️ Always run `idf.py build` from the **project root**, not from inside a component directory. Running it inside a component causes CMake to fail with `Unknown CMake command "idf_component_register"`.

```bash
cd ~/shelfbot/shelfbot.esp32.wroom-idf
idf.py build
```

---

## Notes

- **micro-ROS agent must match the distro.** If the firmware is built against Humble, run a Humble micro-ROS agent on the host PC. Mixing distros will cause communication failures.
- **ESP-IDF 5.3 compatibility.** The Humble branch officially targets IDF v4.4/v5.0 but works on v5.3. If you hit issues, check the component README for the exact supported IDF version.
- **Do not update dependencies lightly.** The build log may warn about newer versions of managed components (e.g. `espressif/mdns`, `espressif/esp-dsp`). Run `idf.py update-dependencies` only intentionally, and re-test after.
