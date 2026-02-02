// [file name]: http_server.hpp
#pragma once
#ifndef SHELFBOT_HTTP_SERVER_H
#define SHELFBOT_HTTP_SERVER_H

#include <esp_http_server.h>
#include <esp_log.h>
#include <esp_system.h>
#include <cJSON.h>
#include "sensor_manager.hpp"
#include "sensor_common.hpp"
#include "i2c_scanner.hpp"
#include <string>
#include <vector>

class HttpServer {
public:
  static HttpServer& get_instance() {
    static HttpServer instance;
    return instance;
  }

  esp_err_t start();
  esp_err_t stop();

  bool is_running() const { return server_ != nullptr; }

private:
  HttpServer() = default;
  ~HttpServer() = default;

  HttpServer(const HttpServer&) = delete;
  HttpServer& operator=(const HttpServer&) = delete;

  httpd_handle_t server_ = nullptr;
  static const char* TAG;

  // HTTP request handlers
  static esp_err_t root_handler(httpd_req_t* req);
  static esp_err_t tof_handler(httpd_req_t* req);
  static esp_err_t ultrasonic_handler(httpd_req_t* req);
  static esp_err_t sensors_handler(httpd_req_t* req);
  static esp_err_t health_handler(httpd_req_t* req);

  // Helper functions
  static std::string get_sensor_status_text(const SensorCommon::TofMeasurement& measurement);
  static cJSON* create_sensor_json(const SensorCommon::SensorDataPacket& sensor_data);
  static cJSON* create_ultrasonic_json(const SensorCommon::SensorDataPacket& sensor_data);
  static cJSON* create_tof_json(const SensorCommon::SensorDataPacket& sensor_data);

  // Register URI handlers
  esp_err_t register_uri_handlers();
};

#endif // SHELFBOT_HTTP_SERVER_H