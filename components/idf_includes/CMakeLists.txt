idf_component_register(
  SRCS "."
  INCLUDE_DIRS "include"
  REQUIRES
  freertos
  esp_system
  esp_timer
  esp_hw_support
  esp_driver_gpio
  esp_driver_i2c
  esp_driver_gptimer
  esp_driver_uart
  esp_http_server
  esp_common
)
