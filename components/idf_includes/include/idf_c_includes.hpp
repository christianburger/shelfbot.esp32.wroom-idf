#pragma once
// C headers (must be wrapped)
extern "C" {
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"

#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "esp_http_server.h"
#include "esp_timer.h"
#include "esp_err.h"
#include "esp_task_wdt.h"
#include "driver/i2c_master.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "soc/gpio_num.h"
}

// C++ headers (never wrapped)
#include <functional>
#include <cstdint>
#include <cstring>
#include <vector>
#include <cmath>
#include <memory>
#include <algorithm>
#include <string>
#include <sstream>
