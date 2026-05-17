# ENVIRONMENT_SETUP.md – Complete ESP32 micro-ROS & FastAccelStepper Environment

This guide sets up a working ESP-IDF v5.3 project with:
- **micro-ROS** (Humble branch) – for ROS 2 communication
- **FastAccelStepper** – for stepper motor control
- All necessary workarounds (empy version, partition table, flash size)

---

## 1. Install ESP-IDF v5.3

```bash
mkdir -p ~/esp
cd ~/esp
git clone -b v5.3 --recursive https://github.com/espressif/esp-idf.git
cd esp-idf
./install.sh esp32
```

---

## 2. Load ESP-IDF Environment (do this every new terminal)

```bash
. ~/esp/esp-idf/export.sh
idf.py --version   # Must show v5.3
```

---

## 3. Install Required Python Packages (critical: pin empy)

```bash
pip3 install catkin_pkg lark-parser colcon-common-extensions empy==3.3.4 pyserial setuptools
```

> **Why empy==3.3.4?** Newer versions break the ROS 2 message generation (`rosidl_adapter` crashes with `AttributeError: 'NoneType' object has no attribute 'shutdown'`).

---

## 4. Create the Project

```bash
mkdir -p ~/shelfbot
cd ~/shelfbot
idf.py create-project shelfbot.esp32.wroom-idf
cd shelfbot.esp32.wroom-idf
```

---

## 5. Add micro-ROS Component (Humble branch)

```bash
mkdir -p components
cd components
git clone https://github.com/micro-ROS/micro_ros_espidf_component.git
cd micro_ros_espidf_component
git checkout humble
cd ../..
```

---

## 6. Add FastAccelStepper Dependency

```bash
idf.py add-dependency "gin66/fastaccelstepper^1.2.5"
```

Verify:
```bash
ls managed_components   # Should show gin66__fastaccelstepper
```

---

## 7. Set Target and Clean Old Builds

```bash
idf.py set-target esp32
rm -rf components/micro_ros_espidf_component/micro_ros_src \
       components/micro_ros_espidf_component/micro_ros_dev \
       build
```

---

## 8. Configure Flash Size (Critical for partition table)

**The default flash size is 2 MB but your ESP32-WROOM module likely has 4 MB.**  
Set it correctly:

```bash
idf.py menuconfig
```

Navigate to:
```
Serial flasher config → Flash size → 4 MB
```
Save & exit.

---

## 9. Create Custom Partition Table (2 MB factory partition for micro-ROS)

Create a file `partitions.csv` in the project root:

```csv
# Name,   Type, SubType, Offset,  Size, Flags
nvs,      data, nvs,     ,        0x4000,
otadata,  data, ota,     ,        0x2000,
phy_init, data, phy,     ,        0x1000,
factory,  app,  factory, ,        0x200000,   # 2 MB – fits micro‑ROS binary (~1.25 MB)
storage,  data, spiffs,  ,        0x1E0000,   # remaining ~1.9 MB
```

Now tell ESP-IDF to use this custom table:

```bash
idf.py menuconfig
```

Set:
- `Partition Table` → `Custom partition table CSV`
- `Custom partition table CSV file` → `partitions.csv`

Save & exit.

---

## 10. Build the Firmware

```bash
idf.py build
```

If you see `app partition is too small`, double‑check that:
- Flash size is **4 MB** (step 8)
- `partitions.csv` has `0x200000` for factory partition
- You ran `idf.py menuconfig` to select the custom table

---

## 11. Flash and Monitor

```bash
idf.py -p /dev/ttyUSB0 flash
idf.py monitor
```

Press `Ctrl+]` to exit monitor.

---

## 12. Run micro-ROS Agent (on your host PC)

Use Docker (recommended):

```bash
docker run -it --rm --net=host microros/micro-ros-agent:humble serial --dev /dev/ttyUSB0 -v6
```

> The agent **must** be `humble` to match the firmware branch.

---

## 13. Full Clean Rebuild (if something breaks)

```bash
cd ~/shelfbot/shelfbot.esp32.wroom-idf

# Remove all generated micro-ROS sources and build artifacts
rm -rf components/micro_ros_espidf_component/micro_ros_src \
       components/micro_ros_espidf_component/micro_ros_dev \
       build managed_components dependencies.lock

idf.py fullclean
idf.py reconfigure
idf.py build
```

---

## 14. Example Code Snippet

In `main/main.cpp`:

```cpp
#include "FastAccelStepper.h"
#include "micro_ros_esp_idf.h"

FastAccelStepperEngine engine;
FastAccelStepper *stepper = nullptr;

extern "C" void app_main() {
    // Init micro-ROS
    micro_ros_esp_idf_init(0, 0, "shelfbot", "esp32");

    // Init stepper
    engine.init();
    stepper = engine.stepperConnectToPin(18);
    if (stepper) {
        stepper->setDirectionPin(19);
        stepper->setEnablePin(21);
        stepper->setAutoEnable(true);
        stepper->setSpeedInHz(1000);
        stepper->setAcceleration(1000);
        stepper->move(2000);
    }

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
```

---

## Summary of Critical Fixes

| Problem | Solution |
|---------|----------|
| `rosidl_adapter` crash (`AttributeError: shutdown`) | Pin `empy==3.3.4` |
| `app partition is too small` | Custom `partitions.csv` with `0x200000` factory size |
| `Partition table does not fit in flash` | Set flash size to **4 MB** in menuconfig (Serial flasher config) |
| micro-ROS agent mismatch | Use `humble` branch everywhere |

---

**Your environment is now ready.** Follow the steps in order, and the build will succeed.
