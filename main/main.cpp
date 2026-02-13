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
  ESP_LOGI("app_main", "Shelfbot begin() returned. Entering main loop.");

  // Main loop - just delay to keep task alive
  while (1) {
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}