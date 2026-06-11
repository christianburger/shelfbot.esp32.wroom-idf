#pragma once
#include <idf_c_includes.hpp>
#include "state_machine_lifecycle.hpp"   // for WifiManagerState

// Public event group bits
#define WM_CONNECTED_BIT    BIT0
#define WM_DISCONNECTED_BIT BIT1

// Legacy state values for info struct – keep as uint8_t to match original API
// but we no longer define an enum for them.
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

esp_err_t wifi_manager_init();
void wifi_manager_get_info(wifi_manager_info_t *out);
EventGroupHandle_t wifi_manager_get_event_group();