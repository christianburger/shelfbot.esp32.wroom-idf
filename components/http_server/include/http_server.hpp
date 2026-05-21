#pragma once
#ifndef SHELFBOT_HTTP_SERVER_H
#define SHELFBOT_HTTP_SERVER_H

#include <idf_c_includes.hpp>
#include <sensor_manager.hpp>
#include <sensor_common.hpp>
#include <i2c_scanner.hpp>
#include <motor_control.hpp>
#include <firmware_version.hpp>
#include <lidar_packet_parser.hpp>

class HttpServer {
public:
    static HttpServer& get_instance() {
        static HttpServer instance;
        return instance;
    }

    // Non-copyable, non-movable
    HttpServer(const HttpServer&) = delete;
    HttpServer& operator=(const HttpServer&) = delete;

    esp_err_t start();
    esp_err_t stop();
    [[nodiscard]] bool is_running() const { return server_ != nullptr; }

private:
    HttpServer() = default;
    ~HttpServer() = default;

    httpd_handle_t server_ = nullptr;
    static const char* TAG;

    static esp_err_t register_uri_handlers(httpd_handle_t server);

    // URI handlers
    static esp_err_t root_handler(httpd_req_t* req);
    static esp_err_t tof_handler(httpd_req_t* req);
    static esp_err_t lidar_handler(httpd_req_t* req);
    static esp_err_t ultrasonic_handler(httpd_req_t* req);
    static esp_err_t sensors_handler(httpd_req_t* req);
    static esp_err_t health_handler(httpd_req_t* req);
    static esp_err_t motor_page_handler(httpd_req_t* req);
    static esp_err_t lidar_page_handler(httpd_req_t* req);
    static esp_err_t lidar_js_handler(httpd_req_t* req);
    static esp_err_t motor_status_handler(httpd_req_t* req);
    static esp_err_t motor_set_handler(httpd_req_t* req);

    // Shared helper — avoids duplicating the switch across TofMeasurement and LidarMeasurement
    static std::string sensor_status_to_string(bool valid, int status);

    static std::string get_sensor_status_text(const SensorCommon::TofMeasurement& measurement);
    static std::string get_sensor_status_text(const SensorCommon::LidarMeasurement& measurement);

    static cJSON* create_sensor_json(const SensorCommon::SensorDataPacket& sensor_data);
    static cJSON* create_ultrasonic_json(const SensorCommon::SensorDataPacket& sensor_data);
    static cJSON* create_tof_json(const SensorCommon::SensorDataPacket& sensor_data);
};

#endif // SHELFBOT_HTTP_SERVER_H
