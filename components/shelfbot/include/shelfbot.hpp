#pragma once
#include <idf_c_includes.hpp>

class Shelfbot {
public:
    static Shelfbot& get_instance();
    static esp_err_t begin();

private:
    Shelfbot() = default;
    static Shelfbot* instance_;

    // Optional: store task handles for later inspection/control
    TaskHandle_t wifi_monitor_handle_   = nullptr;
    TaskHandle_t net_services_handle_   = nullptr;
    TaskHandle_t time_sync_handle_      = nullptr;
    TaskHandle_t shelfbot_init_handle_  = nullptr;
};