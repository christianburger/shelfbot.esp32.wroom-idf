#include "wifi_manager.hpp"

static auto TAG = "wifi_manager";

// -------------------------------------------------------------------
// Credential table – populated from Kconfig (menuconfig)
// -------------------------------------------------------------------
struct cred_t { const char *ssid; const char *pass; };

static constexpr cred_t s_creds[] = {
    { CONFIG_WIFI_SSID_1, CONFIG_WIFI_PASS_1 },
    { CONFIG_WIFI_SSID_2, CONFIG_WIFI_PASS_2 },
    { CONFIG_WIFI_SSID_3, CONFIG_WIFI_PASS_3 },
    { CONFIG_WIFI_SSID_4, CONFIG_WIFI_PASS_4 },
};

static constexpr int CRED_COUNT = static_cast<int>(std::size(s_creds));
static_assert(CRED_COUNT > 0, "no credential slots defined");  // guardrail

// -------------------------------------------------------------------
// Tuning from Kconfig
// -------------------------------------------------------------------
#define RSSI_WARN       CONFIG_WIFI_RSSI_WARN_THRESHOLD
#define RSSI_CRITICAL   CONFIG_WIFI_RSSI_CRITICAL_THRESHOLD
#define DEGRADE_N       CONFIG_WIFI_DEGRADED_CONFIRM_N
#define MONITOR_MS      CONFIG_WIFI_MONITOR_INTERVAL_MS
#define RETRIES_PER_NET CONFIG_WIFI_RETRIES_PER_NETWORK
#define CYCLE_DELAY_S   CONFIG_WIFI_INTER_CYCLE_DELAY_S

#define CONNECT_TIMEOUT_MS  12000
#define SCAN_MAX_APS        20

// -------------------------------------------------------------------
// Internal event bits – must not overlap public WM_*_BIT values
// -------------------------------------------------------------------
#define EVT_GOT_IP       BIT2
#define EVT_DISCONNECTED BIT3

// FIX: guardrails – catch bit collisions at compile time
static_assert((EVT_GOT_IP       & (WM_CONNECTED_BIT | WM_DISCONNECTED_BIT)) == 0,
              "EVT_GOT_IP overlaps a public WM bit");
static_assert((EVT_DISCONNECTED & (WM_CONNECTED_BIT | WM_DISCONNECTED_BIT)) == 0,
              "EVT_DISCONNECTED overlaps a public WM bit");
static_assert(EVT_GOT_IP != EVT_DISCONNECTED,
              "internal event bits must be distinct");

// -------------------------------------------------------------------
// Static state
// -------------------------------------------------------------------
static EventGroupHandle_t   s_evt      = nullptr;
static esp_netif_t         *s_netif    = nullptr;
static wifi_manager_state_t s_state    = WM_STATE_IDLE;
static wifi_manager_info_t  s_info     = {};
static portMUX_TYPE         s_info_mux = portMUX_INITIALIZER_UNLOCKED;

static int64_t  s_connected_at_us = 0;
static uint32_t s_switches        = 0;
static uint32_t s_reconnects      = 0;
static int      s_degrade_streak  = 0;

// -------------------------------------------------------------------
// Helper: state string
// -------------------------------------------------------------------
const char *wifi_manager_state_str(wifi_manager_state_t s) {
    switch (s) {
        case WM_STATE_IDLE:       return "IDLE";
        case WM_STATE_SCANNING:   return "SCANNING";
        case WM_STATE_CONNECTING: return "CONNECTING";
        case WM_STATE_CONNECTED:  return "CONNECTED";
        case WM_STATE_DEGRADED:   return "DEGRADED";
        default:                  return "UNKNOWN";
    }
}

static void set_state(const wifi_manager_state_t next) {
    if (s_state != next) {
        ESP_LOGI(TAG, "state %s → %s", wifi_manager_state_str(s_state), wifi_manager_state_str(next));
        s_state = next;
        portENTER_CRITICAL(&s_info_mux);
        s_info.state = next;
        portEXIT_CRITICAL(&s_info_mux);
    }
}

// -------------------------------------------------------------------
// Publish thread-safe info snapshot
// -------------------------------------------------------------------
static void publish_info(const char *ssid, const int8_t rssi, const bool degraded) {
    esp_netif_ip_info_t ip = {};
    if (s_netif) esp_netif_get_ip_info(s_netif, &ip);

    char ip_str[16] = "";
    if (ip.ip.addr)
        snprintf(ip_str, sizeof(ip_str), IPSTR, IP2STR(&ip.ip));

    const uint32_t uptime = (s_connected_at_us > 0)
        ? static_cast<uint32_t>(
              (esp_timer_get_time() - s_connected_at_us) / 1000000ULL)
        : 0;

    portENTER_CRITICAL(&s_info_mux);
    s_info.state      = s_state;   // belt-and-suspenders; set_state keeps this in sync too
    s_info.rssi_dbm   = rssi;
    s_info.degraded   = degraded;
    s_info.uptime_s   = uptime;
    s_info.switches   = s_switches;
    s_info.reconnects = s_reconnects;
    strlcpy(s_info.ip, ip_str, sizeof(s_info.ip));
    if (ssid) strlcpy(s_info.ssid, ssid, sizeof(s_info.ssid));
    portEXIT_CRITICAL(&s_info_mux);
}

// -------------------------------------------------------------------
// Event handlers
// -------------------------------------------------------------------
static void on_wifi_event(void *arg, esp_event_base_t base, const int32_t id, const void *data) {
    if (id == WIFI_EVENT_STA_DISCONNECTED) {
        const auto *d =
            static_cast<const wifi_event_sta_disconnected_t *>(data);
        ESP_LOGW(TAG, "STA disconnected, reason=%d", d->reason);
        xEventGroupClearBits(s_evt, WM_CONNECTED_BIT);
        xEventGroupSetBits(s_evt, WM_DISCONNECTED_BIT | EVT_DISCONNECTED);
    }
}

static void on_ip_event(void *arg, esp_event_base_t base, const int32_t id, const void *data) {
    if (id == IP_EVENT_STA_GOT_IP) {
        const auto *e =
            static_cast<const ip_event_got_ip_t *>(data);
        ESP_LOGI(TAG, "got IP: " IPSTR, IP2STR(&e->ip_info.ip));
        xEventGroupClearBits(s_evt, WM_DISCONNECTED_BIT | EVT_DISCONNECTED);
        xEventGroupSetBits(s_evt, WM_CONNECTED_BIT | EVT_GOT_IP);
    }
}

// -------------------------------------------------------------------
// Scan: find strongest visible known AP
// -------------------------------------------------------------------
static int scan_pick_best() {
    constexpr wifi_scan_config_t cfg = {
        .ssid        = nullptr,
        .bssid       = nullptr,
        .channel     = 0,
        .show_hidden = false,
        .scan_type   = WIFI_SCAN_TYPE_ACTIVE,
        .scan_time   = {
            .active  = { .min = WIFI_ACTIVE_SCAN_MIN_DEFAULT_TIME,
                         .max = WIFI_ACTIVE_SCAN_MAX_DEFAULT_TIME },
            .passive = WIFI_PASSIVE_SCAN_DEFAULT_TIME
        },
        .home_chan_dwell_time = WIFI_SCAN_HOME_CHANNEL_DWELL_DEFAULT_TIME,
        .channel_bitmap      = { .ghz_2_channels = 0, .ghz_5_channels = 0 }
    };

    const esp_err_t r = esp_wifi_scan_start(&cfg, true);
    if (r != ESP_OK) {
        ESP_LOGE(TAG, "scan_start failed: %s", esp_err_to_name(r));
        return -1;
    }

    uint16_t count = SCAN_MAX_APS;
    wifi_ap_record_t aps[SCAN_MAX_APS] = {};
    esp_wifi_scan_get_ap_records(&count, aps);
    ESP_LOGI(TAG, "scan: %u APs found", static_cast<unsigned>(count));

    int    best_cred = -1;
    int8_t best_rssi = INT8_MIN;

    for (int ci = 0; ci < CRED_COUNT; ci++) {
        // FIX: guard null pointer before dereference (Kconfig edge case)
        if (!s_creds[ci].ssid || s_creds[ci].ssid[0] == '\0') continue;
        for (uint16_t ai = 0; ai < count; ai++) {
            if (strcmp(reinterpret_cast<const char *>(aps[ai].ssid),
                       s_creds[ci].ssid) == 0) {
                // FIX: was "%p" – printed the array's stack address, not the string
                ESP_LOGI(TAG, "  found known AP \"%s\" ch%d rssi=%d",
                         reinterpret_cast<const char *>(aps[ai].ssid),
                         aps[ai].primary, aps[ai].rssi);
                if (aps[ai].rssi > best_rssi) {
                    best_rssi = aps[ai].rssi;
                    best_cred = ci;
                }
            }
        }
    }

    if (best_cred >= 0)
        ESP_LOGI(TAG, "scan winner: \"%s\" rssi=%d dBm",
                 s_creds[best_cred].ssid, best_rssi);
    else
        ESP_LOGW(TAG, "scan: no known AP visible");

    return best_cred;
}

// -------------------------------------------------------------------
// Connect helpers
// -------------------------------------------------------------------
static void disconnect_blocking() {
    xEventGroupClearBits(s_evt, EVT_DISCONNECTED | EVT_GOT_IP);
    const esp_err_t r = esp_wifi_disconnect();
    if (r != ESP_OK) {
        // Not always fatal (e.g. already disconnected); log and fall through
        ESP_LOGW(TAG, "disconnect: %s", esp_err_to_name(r));
    }
    xEventGroupWaitBits(s_evt, EVT_DISCONNECTED,
                        pdTRUE, pdFALSE, pdMS_TO_TICKS(2000));
    // Clear again: remove the disconnect event we just consumed so it
    // doesn't confuse the subsequent connect wait.
    xEventGroupClearBits(s_evt, EVT_DISCONNECTED | EVT_GOT_IP);
}

static esp_err_t start_connect(int ci) {
    // FIX: runtime guardrails
    configASSERT(ci >= 0 && ci < CRED_COUNT);
    configASSERT(s_creds[ci].ssid && s_creds[ci].ssid[0] != '\0');
    wifi_config_t cfg = {};
    strlcpy(reinterpret_cast<char *>(cfg.sta.ssid), s_creds[ci].ssid, sizeof(cfg.sta.ssid));
    const char *pass = s_creds[ci].pass ? s_creds[ci].pass : "";
    strlcpy(reinterpret_cast<char *>(cfg.sta.password), pass, sizeof(cfg.sta.password));
    cfg.sta.threshold.authmode = (pass[0] == '\0') ? WIFI_AUTH_OPEN : WIFI_AUTH_WPA2_PSK;
    cfg.sta.pmf_cfg.capable = true;
    ESP_LOGI(TAG, "connecting → \"%s\"", reinterpret_cast<const char *>(cfg.sta.ssid));
    esp_err_t r = esp_wifi_set_config(WIFI_IF_STA, &cfg);
    if (r == ESP_OK) r = esp_wifi_connect();
    return r;
}

// -------------------------------------------------------------------
// RSSI monitor (called from manager task while connected)
// -------------------------------------------------------------------
static bool monitor_rssi() {
    wifi_ap_record_t ap;
    if (esp_wifi_sta_get_ap_info(&ap) != ESP_OK) {
        return false;
    }
    if (const bool degraded = (ap.rssi < RSSI_WARN); !degraded) {
        if (s_degrade_streak > 0)
            ESP_LOGI(TAG, "RSSI recovered: %d dBm", ap.rssi);
        s_degrade_streak = 0;
        if (s_state == WM_STATE_DEGRADED) set_state(WM_STATE_CONNECTED);
        publish_info(nullptr, ap.rssi, false);
        return false;
    }

    s_degrade_streak++;
    ESP_LOGW(TAG, "RSSI %d dBm (warn=%d, crit=%d) streak=%d/%d", ap.rssi, RSSI_WARN, RSSI_CRITICAL, s_degrade_streak, DEGRADE_N);

    if (s_state == WM_STATE_CONNECTED) set_state(WM_STATE_DEGRADED);
    publish_info(nullptr, ap.rssi, true);

    if (ap.rssi < RSSI_CRITICAL) {
        ESP_LOGE(TAG, "RSSI critical – immediate scan");
        s_degrade_streak = 0;
        return true;
    }

    if (s_degrade_streak >= DEGRADE_N) {
        s_degrade_streak = 0;
        return true;
    }
    return false;
}

// -------------------------------------------------------------------
// Manager task
// -------------------------------------------------------------------
[[noreturn]] static void manager_task(void *arg) {
    ESP_LOGI(TAG, "manager task started, %d credential slot(s)", CRED_COUNT);
    for (int i = 0; i < CRED_COUNT; i++) {
        if (s_creds[i].ssid && s_creds[i].ssid[0] != '\0')
            ESP_LOGI(TAG, "  [%d] \"%s\"", i, s_creds[i].ssid);
    }
    int  ci        = -1;
    bool skip_scan = false;

    while (true) {
        // ---------- SCAN ----------
        if (!skip_scan) {
            set_state(WM_STATE_SCANNING);
            publish_info(nullptr, 0, false);

            ci = scan_pick_best();
            if (ci < 0) {
                ESP_LOGW(TAG, "no known AP visible, waiting %d s", CYCLE_DELAY_S);
                vTaskDelay(pdMS_TO_TICKS(CYCLE_DELAY_S * 1000));
                continue;
            }
        }
        skip_scan = false;
        configASSERT(ci >= 0 && ci < CRED_COUNT);

        // ---------- CONNECT ----------
        set_state(WM_STATE_CONNECTING);
        bool connected = false;
        s_reconnects++;

        for (int attempt = 1; attempt <= RETRIES_PER_NET && !connected; attempt++) {
            disconnect_blocking();

            if (start_connect(ci) != ESP_OK) {
                vTaskDelay(pdMS_TO_TICKS(2000));
                continue;
            }

            const EventBits_t bits = xEventGroupWaitBits(
                s_evt,
                EVT_GOT_IP | EVT_DISCONNECTED,
                pdTRUE, pdFALSE,
                pdMS_TO_TICKS(CONNECT_TIMEOUT_MS));

            if (bits & EVT_GOT_IP) {
                connected = true;
            } else {
                ESP_LOGW(TAG, "\"%s\" attempt %d/%d failed",
                         s_creds[ci].ssid, attempt, RETRIES_PER_NET);
                vTaskDelay(pdMS_TO_TICKS(1500));
            }
        }

        if (!connected) {
            ESP_LOGW(TAG, "\"%s\" unreachable – rescanning", s_creds[ci].ssid);
            vTaskDelay(pdMS_TO_TICKS(3000));
            continue;
        }

        // ---------- CONNECTED ----------
        s_connected_at_us = esp_timer_get_time();
        s_degrade_streak  = 0;
        set_state(WM_STATE_CONNECTED);
        publish_info(s_creds[ci].ssid, 0, false);
        ESP_LOGI(TAG, "connected to \"%s\" (reconnect #%lu)",
                 s_creds[ci].ssid, static_cast<unsigned long>(s_reconnects));

        while (true) {
            const EventBits_t bits = xEventGroupWaitBits(
                s_evt,
                EVT_DISCONNECTED,
                pdTRUE, pdFALSE,
                pdMS_TO_TICKS(MONITOR_MS));

            if (bits & EVT_DISCONNECTED) {
                ESP_LOGW(TAG, "connection lost – rescanning");
                s_connected_at_us = 0;
                break; // outer loop will scan from scratch
            }

            if (monitor_rssi()) {
                set_state(WM_STATE_SCANNING);
                const int better = scan_pick_best();

                if (better < 0) {
                    ESP_LOGW(TAG, "degraded but no alternatives – staying");
                    set_state(WM_STATE_DEGRADED);
                    continue;
                }

                if (better == ci) {
                    ESP_LOGI(TAG, "\"%s\" is still best – staying",
                             s_creds[ci].ssid);
                    set_state(WM_STATE_DEGRADED);
                    continue;
                }

                s_switches++;
                ESP_LOGI(TAG, "switching \"%s\" → \"%s\" (switch #%lu)",
                         s_creds[ci].ssid, s_creds[better].ssid,
                         static_cast<unsigned long>(s_switches));

                ci        = better;  // FIX: now actually used – outer loop
                skip_scan = true;    //      skips the scan and reconnects directly
                s_connected_at_us = 0;
                xEventGroupClearBits(s_evt, WM_CONNECTED_BIT);
                xEventGroupSetBits(s_evt, WM_DISCONNECTED_BIT);
                break;
            }
        }
    }
}

// -------------------------------------------------------------------
// Public API
// -------------------------------------------------------------------
esp_err_t wifi_manager_init() {
    s_evt = xEventGroupCreate();
    if (!s_evt) return ESP_ERR_NO_MEM;

    xEventGroupSetBits(s_evt, WM_DISCONNECTED_BIT);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    s_netif = esp_netif_create_default_wifi_sta();

    const wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(84));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, reinterpret_cast<esp_event_handler_t>(on_wifi_event), nullptr));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, reinterpret_cast<esp_event_handler_t>(on_ip_event), nullptr));

    const BaseType_t r = xTaskCreatePinnedToCore( manager_task, "wifi_mgr", 4096, nullptr,
        configMAX_PRIORITIES - 2, nullptr, 1);
    if (r != pdPASS) {
        ESP_LOGE(TAG, "failed to create manager task");
        return ESP_FAIL;
    }
    return ESP_OK;
}

void wifi_manager_get_info(wifi_manager_info_t *out) {
    portENTER_CRITICAL(&s_info_mux);
    *out = s_info;
    portEXIT_CRITICAL(&s_info_mux);
}

EventGroupHandle_t wifi_manager_get_event_group() {
    return s_evt;
}
