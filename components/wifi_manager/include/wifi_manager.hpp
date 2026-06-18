#pragma once

#include <idf_c_includes.hpp>
#include "state_machine_lifecycle.hpp"   // for WifiManagerState

// Public event group bits (legacy macros)
#define WM_CONNECTED_BIT    BIT0
#define WM_DISCONNECTED_BIT BIT1

// Legacy state values for info struct – used by HTTP dashboard
#define WM_STATE_IDLE       0
#define WM_STATE_SCANNING   1
#define WM_STATE_CONNECTING 2
#define WM_STATE_CONNECTED  3
#define WM_STATE_DEGRADED   4

typedef struct {
    uint8_t  state;          // one of WM_STATE_* values
    char     ssid[33];
    int8_t   rssi_dbm;
    char     ip[16];
    uint32_t uptime_s;
    bool     degraded;
    uint32_t switches;
    uint32_t reconnects;
} wifi_manager_info_t;

class WifiManager {
public:
    static WifiManager& get_instance();

    WifiManager(const WifiManager&) = delete;
    WifiManager& operator=(const WifiManager&) = delete;

    esp_err_t init();
    esp_err_t start();

    EventGroupHandle_t get_event_group() const { return event_group_; }
    void get_info(wifi_manager_info_t* out);

    static constexpr uint32_t WIFI_CONNECTED_BIT    = WM_CONNECTED_BIT;
    static constexpr uint32_t WIFI_DISCONNECTED_BIT = WM_DISCONNECTED_BIT;

private:
    WifiManager() = default;
    ~WifiManager() = default;

    EventGroupHandle_t event_group_ = nullptr;
    esp_netif_t*       netif_       = nullptr;
    TaskHandle_t       task_handle_ = nullptr;
    bool               initialized_ = false;

    struct cred_t { const char* ssid; const char* pass; };
    static constexpr cred_t creds_[] = {
        { CONFIG_WIFI_SSID_1, CONFIG_WIFI_PASS_1 },
        { CONFIG_WIFI_SSID_2, CONFIG_WIFI_PASS_2 },
        { CONFIG_WIFI_SSID_3, CONFIG_WIFI_PASS_3 },
        { CONFIG_WIFI_SSID_4, CONFIG_WIFI_PASS_4 },
    };
    static constexpr int CRED_COUNT = static_cast<int>(std::size(creds_));

    wifi_manager_info_t info_ = {};
    portMUX_TYPE info_mux_ = portMUX_INITIALIZER_UNLOCKED;
    int64_t connected_us_ = 0;
    uint32_t switches_ = 0;
    uint32_t reconnects_ = 0;
    int degrade_streak_ = 0;

    static void manager_task(void* arg);
    static void event_handler(void* arg, esp_event_base_t base, int32_t id, void* data);

    void set_wifi_state(WifiManagerState new_state);
    void publish_info(const char* ssid, int8_t rssi, bool degraded);
    int scan_pick_best();
    void disconnect_blocking();
    esp_err_t start_connect(int ci);
    bool monitor_rssi();
    void start_or_restart_sntp();
};