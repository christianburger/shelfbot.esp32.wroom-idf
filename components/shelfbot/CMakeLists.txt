idf_component_register(SRCS "shelfbot.cpp"
  INCLUDE_DIRS "."
  REQUIRES
  micro_ros_espidf_component
  motor_control
  wifi_station
  http_server
  led_control
  sensor_control
  mdns
  nvs_flash
  esp_netif
  esp_event
  esp_system
  freertos
)
