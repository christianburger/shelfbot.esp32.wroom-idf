#pragma once
#include <idf_c_includes.hpp>
#include "state_machine_lifecycle.hpp"

// Public event group bits
#define WM_CONNECTED_BIT    BIT0
#define WM_DISCONNECTED_BIT BIT1

#define WM_STATE_IDLE       0
#define WM_STATE_SCANNING   1
#define WM_STATE_CONNECTING 2
#define WM_STATE_CONNECTED  3
#define WM_STATE_DEGRADED   4

typedef struct {
    uint8_t  state;
    char     ssid[33];
    int8_t   rssi_dbm;
    char     ip[16];
    uint32_t uptime_s;
    bool     degraded;
    uint32_t switches;
    uint32_t reconnects;
} wifi_manager_info_t;

// ============================================================================
// Singleton Wi‑Fi manager
// ============================================================================
class WifiManager {
public:
    static WifiManager& getInstance();

    // Non‑copyable, non‑movable
    WifiManager(const WifiManager&)            = delete;
    WifiManager& operator=(const WifiManager&) = delete;

    // Initialise hardware & event handlers – does NOT create the task.
    esp_err_t init();

    // Public queries – non‑const because of port critical sections.
    void getInfo(wifi_manager_info_t* out);
    EventGroupHandle_t getEventGroup() const { return evt_; }

    // Task handle setter (called from shelfbot.cpp after xTaskCreate)
    void setTaskHandle(TaskHandle_t h) { task_handle_ = h; }

    // Static task function – passes 'this' as arg.
    static void task_fn(void* arg);

private:
    WifiManager()  = default;
    ~WifiManager() = default;

    // Main loop (called by task_fn)
    void mainLoop();

    // Internal helpers
    void setWifiState(WifiManagerState new_state);
    void publishInfo(const char* ssid, int8_t rssi, bool degraded);
    void startOrRestartSntp();
    int  scanPickBest();
    void disconnectBlocking();
    esp_err_t startConnect(int ci);
    bool monitorRssi();

    // Event handlers (called from static C callbacks)
    void handleWifiEvent(int32_t event_id, const void* data);
    void handleIpEvent(int32_t event_id, const void* data);
    void handleSntpSync(struct timeval* tv);

    // Friends – these are external C‑compatible functions, not static.
    friend void wifi_event_handler(void* arg, esp_event_base_t base, int32_t id, void* data);
    friend void ip_event_handler(void* arg, esp_event_base_t base, int32_t id, void* data);
    friend void sntp_sync_callback(struct timeval* tv);

    // ── Members ────────────────────────────────────────────────────────────
    TaskHandle_t     task_handle_   = nullptr;
    EventGroupHandle_t evt_         = nullptr;
    esp_netif_t*     netif_         = nullptr;

    wifi_manager_info_t info_       = {};
    portMUX_TYPE      info_mux_     = portMUX_INITIALIZER_UNLOCKED;

    int64_t  connected_us_ = 0;
    uint32_t switches_     = 0;
    uint32_t reconnects_   = 0;
    int      degrade_streak_ = 0;
    bool     sntp_started_ = false;
    bool     initialized_  = false;
};

// ============================================================================
// Free functions – forward to singleton (backward compatibility)
// ============================================================================
inline esp_err_t wifi_manager_init() {
    return WifiManager::getInstance().init();
}

inline void wifi_manager_get_info(wifi_manager_info_t* out) {
    WifiManager::getInstance().getInfo(out);
}

inline EventGroupHandle_t wifi_manager_get_event_group() {
    return WifiManager::getInstance().getEventGroup();
}

// Entry point used by xTaskCreate – calls the static task_fn.
inline void wifi_manager_task_fn(void* arg) {
    WifiManager::task_fn(arg);
}