#pragma once

// C headers (must be wrapped)
extern "C" {
// FreeRTOS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "freertos/event_groups.h"
// ESP-IDF core
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_err.h"
#include "esp_task_wdt.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_http_server.h"
#include "esp_sntp.h"
#include "esp_wifi.h"

// NVS
#include "nvs_flash.h"

// Drivers
#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "driver/i2c_master.h"
#include "driver/uart.h"
#include "soc/gpio_num.h"

// Protocols
#include "cJSON.h"
#include "mdns.h"


// micro-ROS
#include "rcl/rcl.h"
#include "rclc/rclc.h"
#include "rclc/executor.h"
#include "rmw_microros/rmw_microros.h"
#include "rmw_microros/ping.h"
#include "std_msgs/msg/int32.h"
#include "std_msgs/msg/bool.h"
#include "std_msgs/msg/float32.h"
#include "std_msgs/msg/float32_multi_array.h"
}

#include "FastAccelStepper.h"
// C++ standard library
#include <functional>
#include <cstdint>
#include <cstring>
#include <cinttypes>
#include <cmath>
#include <ctime>
#include <vector>
#include <array>
#include <memory>
#include <algorithm>
#include <string>
#include <fstream>
#include <sstream>
#include <iomanip>
