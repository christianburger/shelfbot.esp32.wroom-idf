#pragma once
#include <idf_c_includes.hpp>

// Public event group bits
#define WM_CONNECTED_BIT    BIT0
#define WM_DISCONNECTED_BIT BIT1

typedef enum {
    WM_STATE_IDLE       = 0,
    WM_STATE_SCANNING,
    WM_STATE_CONNECTING,
    WM_STATE_CONNECTED,
    WM_STATE_DEGRADED,
} wifi_manager_state_t;

typedef struct {
    wifi_manager_state_t state;
    char                 ssid[33];
    int8_t               rssi_dbm;
    char                 ip[16];
    uint32_t             uptime_s;
    bool                 degraded;
    uint32_t             switches;
    uint32_t             reconnects;
} wifi_manager_info_t;

esp_err_t wifi_manager_init();
void wifi_manager_get_info(wifi_manager_info_t *out);
EventGroupHandle_t wifi_manager_get_event_group();
const char *wifi_manager_state_str(wifi_manager_state_t s);