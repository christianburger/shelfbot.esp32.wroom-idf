#pragma once
#ifndef SHELFBOT_HTTP_SERVER_HPP
#define SHELFBOT_HTTP_SERVER_HPP

// ---------------------------------------------------------------------------
// http_server.hpp
//
// HTTP server component for Shelfbot.
//
// Design constraints (ESP32-D0WD-V3, no PSRAM, ~4 MB flash):
//   • No heap allocation after start().  Every buffer, page string, and
//     LidarScan that must survive across calls is declared static inside
//     its handler function (lives in BSS/rodata — not on the httpd task stack).
//   • Page HTML/CSS/JS lives entirely in http_pages.cpp (flash rodata).
//   • The httpd task stack is set to 6 KB — enough for JSON serialisation
//     of motor status (cJSON nodes are small) but NOT enough for a LidarScan
//     struct (13.7 KB) which is therefore declared static in lidar_api_handler.
//   • microros_sync and all other components are independent; the HTTP server
//     failing has no effect on them.
// ---------------------------------------------------------------------------

#include <idf_c_includes.hpp>

class HttpServer {
public:
    // Singleton accessor — safe to call before start().
    static HttpServer& get_instance();

    // Non-copyable, non-movable.
    HttpServer(const HttpServer&)            = delete;
    HttpServer& operator=(const HttpServer&) = delete;

    // Start the HTTP server.  May be called multiple times; subsequent calls
    // are no-ops if the server is already running.
    esp_err_t start();

    // Stop the HTTP server and release the httpd handle.
    esp_err_t stop();

    // Returns true if the server is currently running.
    [[nodiscard]] bool is_running() const { return server_ != nullptr; }

private:
    HttpServer()  = default;
    ~HttpServer() = default;

    httpd_handle_t server_ = nullptr;

    static const char* TAG;

    // Register all URI handlers with the given httpd instance.
    static esp_err_t register_uri_handlers(httpd_handle_t server);

    // ---------------------------------------------------------------------------
    // Page handlers  (serve static content from flash)
    // ---------------------------------------------------------------------------
    static esp_err_t root_handler       (httpd_req_t* req);
    static esp_err_t motor_page_handler (httpd_req_t* req);
    static esp_err_t lidar_page_handler (httpd_req_t* req);
    static esp_err_t lidar_js_handler   (httpd_req_t* req);

    // ---------------------------------------------------------------------------
    // API handlers
    // ---------------------------------------------------------------------------
    static esp_err_t motor_status_handler(httpd_req_t* req);
    static esp_err_t motor_set_handler   (httpd_req_t* req);
    static esp_err_t lidar_api_handler   (httpd_req_t* req);

    // ---------------------------------------------------------------------------
    // Shared helpers
    // ---------------------------------------------------------------------------
    static void      add_cors_headers   (httpd_req_t* req);
    static esp_err_t options_handler    (httpd_req_t* req);
};

#endif // SHELFBOT_HTTP_SERVER_HPP
