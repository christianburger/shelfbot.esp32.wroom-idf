#include <wifi_manager.hpp>
#include <state_machine.hpp>
#include <state_machine_lifecycle.hpp>

static const char* TAG = "wifi_manager";

// ---------------------------------------------------------------------------
// Credential table from Kconfig
// ---------------------------------------------------------------------------
struct cred_t { const char* ssid; const char* pass; };
static constexpr cred_t s_creds[] = {
    { CONFIG_WIFI_SSID_1, CONFIG_WIFI_PASS_1 },
    { CONFIG_WIFI_SSID_2, CONFIG_WIFI_PASS_2 },
    { CONFIG_WIFI_SSID_3, CONFIG_WIFI_PASS_3 },
    { CONFIG_WIFI_SSID_4, CONFIG_WIFI_PASS_4 },
};
static constexpr int CRED_COUNT = static_cast<int>(std::size(s_creds));
static_assert(CRED_COUNT > 0, "no credential slots defined");

static int valid_cred_count() {
    int v = 0;
    for (int i = 0; i < CRED_COUNT; ++i)
        if (s_creds[i].ssid && s_creds[i].ssid[0] != '\0') ++v;
    return v;
}

// ---------------------------------------------------------------------------
// Tuning from Kconfig
// ---------------------------------------------------------------------------
#define RSSI_WARN       CONFIG_WIFI_RSSI_WARN_THRESHOLD
#define RSSI_CRITICAL   CONFIG_WIFI_RSSI_CRITICAL_THRESHOLD
#define DEGRADE_N       CONFIG_WIFI_DEGRADED_CONFIRM_N
#define MONITOR_MS      CONFIG_WIFI_MONITOR_INTERVAL_MS
#define RETRIES_PER_NET CONFIG_WIFI_RETRIES_PER_NETWORK
#define CYCLE_DELAY_S   CONFIG_WIFI_INTER_CYCLE_DELAY_S

#define CONNECT_TIMEOUT_MS  12000
#define SCAN_MAX_APS        20

// Internal event bits – must not overlap the public WM_* bits
#define EVT_GOT_IP       BIT2
#define EVT_DISCONNECTED BIT3

static_assert((EVT_GOT_IP       & (WM_CONNECTED_BIT | WM_DISCONNECTED_BIT)) == 0);
static_assert((EVT_DISCONNECTED & (WM_CONNECTED_BIT | WM_DISCONNECTED_BIT)) == 0);

// ---------------------------------------------------------------------------
// Singleton implementation
// ---------------------------------------------------------------------------
WifiManager& WifiManager::getInstance() {
    static WifiManager instance;
    return instance;
}

// ---------------------------------------------------------------------------
// SNTP callback (C function) → forwards to singleton
// ---------------------------------------------------------------------------
void sntp_sync_callback(struct timeval* tv) {
    WifiManager::getInstance().handleSntpSync(tv);
}

void WifiManager::handleSntpSync(struct timeval* tv) {
    ESP_LOGI(TAG, "SNTP time sync received: sec=%lld, usec=%ld",
             (long long)tv->tv_sec, tv->tv_usec);
}

// ---------------------------------------------------------------------------
// Event handlers (C functions) → forward to singleton methods
// ---------------------------------------------------------------------------
void wifi_event_handler(void* arg, esp_event_base_t base, int32_t id, void* data) {
    WifiManager::getInstance().handleWifiEvent(id, data);
}

void ip_event_handler(void* arg, esp_event_base_t base, int32_t id, void* data) {
    WifiManager::getInstance().handleIpEvent(id, data);
}

void WifiManager::handleWifiEvent(int32_t id, const void* data) {
    if (id == WIFI_EVENT_STA_DISCONNECTED) {
        const auto* d = static_cast<const wifi_event_sta_disconnected_t*>(data);
        ESP_LOGW(TAG, "STA disconnected reason=%d", d->reason);
        xEventGroupClearBits(evt_, WM_CONNECTED_BIT);
        xEventGroupSetBits(evt_, WM_DISCONNECTED_BIT | EVT_DISCONNECTED);
        setWifiState(WifiManagerState::DISCONNECTED);
    }
}

void WifiManager::handleIpEvent(int32_t id, const void* data) {
    if (id == IP_EVENT_STA_GOT_IP) {
        const auto* e = static_cast<const ip_event_got_ip_t*>(data);
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&e->ip_info.ip));
        xEventGroupClearBits(evt_, WM_DISCONNECTED_BIT | EVT_DISCONNECTED);
        xEventGroupSetBits(evt_, WM_CONNECTED_BIT | EVT_GOT_IP);
        setWifiState(WifiManagerState::CONNECTED);
        startOrRestartSntp();
    }
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------
esp_err_t WifiManager::init() {
    if (initialized_) {
        ESP_LOGW(TAG, "init() called again – already initialised, skipping");
        return ESP_OK;
    }

    evt_ = xEventGroupCreate();
    if (!evt_) {
        ESP_LOGE(TAG, "xEventGroupCreate failed – out of memory");
        return ESP_ERR_NO_MEM;
    }
    xEventGroupSetBits(evt_, WM_DISCONNECTED_BIT);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    netif_ = esp_netif_create_default_wifi_sta();

    const wifi_init_config_t wifi_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wifi_cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(84));

    ESP_ERROR_CHECK(esp_event_handler_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID,
        reinterpret_cast<esp_event_handler_t>(wifi_event_handler), nullptr));
    ESP_ERROR_CHECK(esp_event_handler_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP,
        reinterpret_cast<esp_event_handler_t>(ip_event_handler), nullptr));

    // StateMachine::setInitial() for "wifi_manager" is called from shelfbot.cpp
    // before wifi_manager_init() – we do not set it here.

    initialized_ = true;
    ESP_LOGI(TAG, "wifi_manager_init complete – task will be created by shelfbot");
    return ESP_OK;
}

// getInfo is now non‑const because of port critical sections
void WifiManager::getInfo(wifi_manager_info_t* out) {
    portENTER_CRITICAL(&info_mux_);
    *out = info_;
    portEXIT_CRITICAL(&info_mux_);
}

// ---------------------------------------------------------------------------
// State machine helper
// ---------------------------------------------------------------------------
void WifiManager::setWifiState(WifiManagerState new_state) {
    StateMachine::changeState("wifi_manager",
                              stateToString(new_state),
                              /*force_skip_prereqs=*/true);
    portENTER_CRITICAL(&info_mux_);
    switch (new_state) {
        case WifiManagerState::OFF:
        case WifiManagerState::ERROR:       info_.state = WM_STATE_IDLE;       break;
        case WifiManagerState::DISCONNECTED:info_.state = WM_STATE_SCANNING;   break;
        case WifiManagerState::CONNECTING:  info_.state = WM_STATE_CONNECTING; break;
        case WifiManagerState::CONNECTED:   info_.state = WM_STATE_CONNECTED;  break;
        default:                            info_.state = WM_STATE_IDLE;       break;
    }
    portEXIT_CRITICAL(&info_mux_);
}

void WifiManager::publishInfo(const char* ssid, int8_t rssi, bool degraded) {
    esp_netif_ip_info_t ip = {};
    if (netif_) esp_netif_get_ip_info(netif_, &ip);
    char ip_str[16] = {};
    if (ip.ip.addr) snprintf(ip_str, sizeof(ip_str), IPSTR, IP2STR(&ip.ip));
    const uint32_t uptime = (connected_us_ > 0)
        ? static_cast<uint32_t>((esp_timer_get_time() - connected_us_) / 1000000ULL)
        : 0;

    portENTER_CRITICAL(&info_mux_);
    info_.rssi_dbm   = rssi;
    info_.degraded   = degraded;
    info_.uptime_s   = uptime;
    info_.switches   = switches_;
    info_.reconnects = reconnects_;
    strlcpy(info_.ip, ip_str, sizeof(info_.ip));
    if (ssid) strlcpy(info_.ssid, ssid, sizeof(info_.ssid));
    portEXIT_CRITICAL(&info_mux_);
}

// ---------------------------------------------------------------------------
// SNTP
// ---------------------------------------------------------------------------
void WifiManager::startOrRestartSntp() {
    if (sntp_started_) esp_sntp_stop();
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "pool.ntp.org");
    esp_sntp_setservername(1, "time.cloudflare.com");
    ESP_LOGI(TAG, "SNTP client STARTING");
    esp_sntp_init();
    ESP_LOGI(TAG, "SNTP client initialized, sync status: %d", esp_sntp_get_sync_status());
    esp_log_level_set("sntp", ESP_LOG_VERBOSE);
    esp_sntp_set_time_sync_notification_cb(sntp_sync_callback);
    sntp_started_ = true;
    ESP_LOGI(TAG, "SNTP client started (pool.ntp.org, time.cloudflare.com)");
}

// ---------------------------------------------------------------------------
// Scan: pick strongest visible known AP
// ---------------------------------------------------------------------------
int WifiManager::scanPickBest() {
    constexpr wifi_scan_config_t cfg = {
        .ssid = nullptr, .bssid = nullptr, .channel = 0,
        .show_hidden = false, .scan_type = WIFI_SCAN_TYPE_ACTIVE,
        .scan_time = { .active  = { WIFI_ACTIVE_SCAN_MIN_DEFAULT_TIME,
                                    WIFI_ACTIVE_SCAN_MAX_DEFAULT_TIME },
                       .passive = WIFI_PASSIVE_SCAN_DEFAULT_TIME },
        .home_chan_dwell_time = WIFI_SCAN_HOME_CHANNEL_DWELL_DEFAULT_TIME,
        .channel_bitmap = { .ghz_2_channels = 0, .ghz_5_channels = 0 }
    };
    if (esp_wifi_scan_start(&cfg, true) != ESP_OK) {
        ESP_LOGE(TAG, "scan_start failed");
        return -1;
    }
    uint16_t count = SCAN_MAX_APS;
    wifi_ap_record_t aps[SCAN_MAX_APS] = {};
    esp_wifi_scan_get_ap_records(&count, aps);
    ESP_LOGI(TAG, "scan: %u APs found", (unsigned)count);

    int    best_cred = -1;
    int8_t best_rssi = INT8_MIN;
    for (int ci = 0; ci < CRED_COUNT; ++ci) {
        if (!s_creds[ci].ssid || s_creds[ci].ssid[0] == '\0') continue;
        for (uint16_t ai = 0; ai < count; ++ai) {
            if (strcmp(reinterpret_cast<const char*>(aps[ai].ssid),
                       s_creds[ci].ssid) == 0) {
                ESP_LOGI(TAG, "  found known AP \"%s\" ch%d rssi=%d",
                         s_creds[ci].ssid, aps[ai].primary, aps[ai].rssi);
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

// ---------------------------------------------------------------------------
// Connect helpers
// ---------------------------------------------------------------------------
void WifiManager::disconnectBlocking() {
    xEventGroupClearBits(evt_, EVT_DISCONNECTED | EVT_GOT_IP);
    esp_wifi_disconnect();
    xEventGroupWaitBits(evt_, EVT_DISCONNECTED, pdTRUE, pdFALSE,
                        pdMS_TO_TICKS(2000));
    xEventGroupClearBits(evt_, EVT_DISCONNECTED | EVT_GOT_IP);
}

esp_err_t WifiManager::startConnect(int ci) {
    configASSERT(ci >= 0 && ci < CRED_COUNT);
    wifi_config_t cfg = {};
    strlcpy(reinterpret_cast<char*>(cfg.sta.ssid),
            s_creds[ci].ssid, sizeof(cfg.sta.ssid));
    const char* pass = s_creds[ci].pass ? s_creds[ci].pass : "";
    strlcpy(reinterpret_cast<char*>(cfg.sta.password), pass, sizeof(cfg.sta.password));
    cfg.sta.threshold.authmode =
        (pass[0] == '\0') ? WIFI_AUTH_OPEN : WIFI_AUTH_WPA2_PSK;
    cfg.sta.pmf_cfg.capable = true;
    ESP_LOGI(TAG, "connecting → \"%s\"", s_creds[ci].ssid);
    const esp_err_t r = esp_wifi_set_config(WIFI_IF_STA, &cfg);
    return (r == ESP_OK) ? esp_wifi_connect() : r;
}

bool WifiManager::monitorRssi() {
    wifi_ap_record_t ap;
    if (esp_wifi_sta_get_ap_info(&ap) != ESP_OK) return false;
    if (ap.rssi >= RSSI_WARN) {
        if (degrade_streak_ > 0)
            ESP_LOGI(TAG, "RSSI recovered: %d dBm", ap.rssi);
        degrade_streak_ = 0;
        publishInfo(nullptr, ap.rssi, false);
        return false;
    }
    ++degrade_streak_;
    ESP_LOGW(TAG, "RSSI %d dBm (streak %d/%d)", ap.rssi, degrade_streak_, DEGRADE_N);
    publishInfo(nullptr, ap.rssi, true);
    if (ap.rssi < RSSI_CRITICAL) { degrade_streak_ = 0; return true; }
    if (degrade_streak_ >= DEGRADE_N) { degrade_streak_ = 0; return true; }
    return false;
}

// ---------------------------------------------------------------------------
// Main loop (runs in the manager task)
// ---------------------------------------------------------------------------
void WifiManager::mainLoop() {
    ESP_LOGI(TAG, "manager task started, %d slot(s), %d configured",
             CRED_COUNT, valid_cred_count());

    bool no_creds_logged = false;
    int  ci              = -1;
    bool skip_scan       = false;

    while (true) {
        if (!skip_scan) {
            if (valid_cred_count() == 0) {
                if (!no_creds_logged) {
                    ESP_LOGE(TAG, "No valid SSIDs configured "
                             "(set CONFIG_WIFI_SSID_1..4 in menuconfig)");
                    no_creds_logged = true;
                }
                setWifiState(WifiManagerState::OFF);
                vTaskDelay(pdMS_TO_TICKS(CYCLE_DELAY_S * 1000));
                continue;
            }
            no_creds_logged = false;
            setWifiState(WifiManagerState::DISCONNECTED);
            publishInfo(nullptr, 0, false);
            ci = scanPickBest();
            if (ci < 0) {
                ESP_LOGW(TAG, "No known AP visible, waiting %ds", CYCLE_DELAY_S);
                vTaskDelay(pdMS_TO_TICKS(CYCLE_DELAY_S * 1000));
                continue;
            }
        }
        skip_scan = false;

        setWifiState(WifiManagerState::CONNECTING);
        bool connected = false;
        ++reconnects_;

        for (int attempt = 1; attempt <= RETRIES_PER_NET && !connected; ++attempt) {
            disconnectBlocking();
            if (startConnect(ci) != ESP_OK) {
                vTaskDelay(pdMS_TO_TICKS(2000));
                continue;
            }
            const EventBits_t bits = xEventGroupWaitBits(
                evt_, EVT_GOT_IP | EVT_DISCONNECTED,
                pdTRUE, pdFALSE, pdMS_TO_TICKS(CONNECT_TIMEOUT_MS));
            if (bits & EVT_GOT_IP) {
                connected = true;
            } else {
                ESP_LOGW(TAG, "\"%s\" attempt %d/%d failed",
                         s_creds[ci].ssid, attempt, RETRIES_PER_NET);
                vTaskDelay(pdMS_TO_TICKS(1500));
            }
        }

        if (!connected) {
            ESP_LOGW(TAG, "\"%s\" unreachable, rescanning", s_creds[ci].ssid);
            vTaskDelay(pdMS_TO_TICKS(3000));
            continue;
        }

        connected_us_ = esp_timer_get_time();
        degrade_streak_ = 0;
        publishInfo(s_creds[ci].ssid, 0, false);
        ESP_LOGI(TAG, "connected to \"%s\" (reconnect #%lu)",
                 s_creds[ci].ssid, (unsigned long)reconnects_);

        while (true) {
            const EventBits_t bits = xEventGroupWaitBits(
                evt_, EVT_DISCONNECTED,
                pdTRUE, pdFALSE, pdMS_TO_TICKS(MONITOR_MS));

            if (bits & EVT_DISCONNECTED) {
                ESP_LOGW(TAG, "connection lost, rescanning");
                connected_us_ = 0;
                break;
            }

            if (monitorRssi()) {
                const int better = scanPickBest();
                if (better < 0) {
                    ESP_LOGW(TAG, "degraded but no alternative AP visible, staying");
                    continue;
                }
                if (better == ci) {
                    ESP_LOGI(TAG, "\"%s\" still best, staying", s_creds[ci].ssid);
                    degrade_streak_ = 0;
                    continue;
                }
                ++switches_;
                ESP_LOGI(TAG, "roaming \"%s\" -> \"%s\" (switch #%lu)",
                         s_creds[ci].ssid, s_creds[better].ssid,
                         (unsigned long)switches_);
                ci             = better;
                skip_scan      = true;
                connected_us_  = 0;
                xEventGroupClearBits(evt_, WM_CONNECTED_BIT);
                xEventGroupSetBits(evt_, WM_DISCONNECTED_BIT);
                break;
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Static task function – called by xTaskCreate
// ---------------------------------------------------------------------------
void WifiManager::task_fn(void* arg) {
    WifiManager* self = static_cast<WifiManager*>(arg);
    self->mainLoop();
    // mainLoop never returns; if it does, delete the task.
    vTaskDelete(nullptr);
}