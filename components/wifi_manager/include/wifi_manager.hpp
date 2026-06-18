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

// ---------------------------------------------------------------------------
// Task function — created by shelfbot.cpp.
// Stack budget: 3072 words (12 KB).
//   Scans up to SCAN_MAX_APS=20 AP records on stack + WiFi driver frames.
//   3072 words measured safe; 2048 caused overflow in testing.
// ---------------------------------------------------------------------------
void wifi_manager_task_fn(void* arg);

// Initialise hardware and event handlers.  Does NOT spawn the manager task —
// call wifi_manager_task_fn via shelfbot.cpp's create_task() instead.
esp_err_t wifi_manager_init();

void wifi_manager_get_info(wifi_manager_info_t* out);
EventGroupHandle_t wifi_manager_get_event_group();
