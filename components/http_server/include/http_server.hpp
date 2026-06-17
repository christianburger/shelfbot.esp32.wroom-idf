#pragma once
#ifndef SHELFBOT_HTTP_SERVER_H
#define SHELFBOT_HTTP_SERVER_H

#include <idf_c_includes.hpp>
#include <firmware_version.hpp>

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

    // Page handlers
    static esp_err_t root_handler(httpd_req_t* req);
    static esp_err_t motor_page_handler(httpd_req_t* req);
    static esp_err_t lidar_page_handler(httpd_req_t* req);
    static esp_err_t lidar_js_handler(httpd_req_t* req);

    // API handlers
    static esp_err_t motor_status_handler(httpd_req_t* req);
    static esp_err_t motor_set_handler(httpd_req_t* req);
    static esp_err_t lidar_api_handler(httpd_req_t* req);
};

#endif // SHELFBOT_HTTP_SERVER_H
