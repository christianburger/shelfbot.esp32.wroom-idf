#include "shelfbot.hpp"
#include "sensor_control.hpp"
#include "motor_control.hpp"
#include "firmware_version.hpp"
#include "ultrasonic_sensor.hpp"
#include "idf_c_includes.hpp"

// The ESP-IDF framework requires a C-style `app_main` entry point.
// We use `extern "C"` to prevent C++ name mangling.
extern "C" void app_main(void)
{
  ESP_LOGI("app_main", "App starting...");

  // Use singleton instance instead of direct construction
  Shelfbot& shelfbot = Shelfbot::get_instance();

  ESP_LOGI("app_main", "Shelfbot instance obtained.");
  shelfbot.begin();
  ESP_LOGI("app_main", "Shelfbot begin() returned. Entering safety loop.");

  // Optional: Safety loop for emergency stop handling
  int stop_signal;
  while (1) {
    // Wait for emergency stop signal with timeout
    if (xQueueReceive(motor_stop_queue, &stop_signal, pdMS_TO_TICKS(1000)) == pdPASS) {

      ESP_LOGW("app_main", "EMERGENCY STOP SIGNAL RECEIVED! Stopping all motors.");
      motor_control_stop_all_motors();
    }
  }
}