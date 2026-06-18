#include "wifi_manager.hpp"
#include <state_machine.hpp>
#include <state_machine_lifecycle.hpp>
#include <esp_wifi.h>
#include <esp_netif.h>
#include <esp_event.h>
#include <nvs_flash.h>
#include <esp_log.h>
#include <esp_sntp.h>
#include <cstring>
#include <algorithm>

static const char* TAG = "WifiManager";

// ─── Singleton implementation ──────────────────────────────────────────────

WifiManager& WifiManager::get_instance() {
    static WifiManager instance;
    return instance;
}

// ─── SNTP ──────────────────────────────────────────────────────────────────

void WifiManager::start_or_restart_sntp() {
    static bool started = false;
    if (started) esp_sntp_stop();
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "pool.ntp.org");
    esp_sntp_setservername(1, "time.cloudflare.com");
    esp_sntp_init();
    started = true;
    ESP_LOGI(TAG, "SNTP client started");
}

// ─── State machine helper ──────────────────────────────────────────────────

void WifiManager::set_wifi_state(WifiManagerState new_state) {
    StateMachine::changeState("wifi_manager",
                              stateToString(new_state),
                              true);

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

void WifiManager::publish_info(const char* ssid, int8_t rssi, bool degraded) {
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

// ─── Event handler ─────────────────────────────────────────────────────────

void WifiManager::event_handler(void* arg, esp_event_base_t base, int32_t id, void* data) {
    WifiManager& self = get_instance();

    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        const auto* d = static_cast<const wifi_event_sta_disconnected_t*>(data);
        ESP_LOGW(TAG, "STA disconnected reason=%d", d->reason);
        xEventGroupClearBits(self.event_group_, WM_CONNECTED_BIT);
        xEventGroupSetBits(self.event_group_, WM_DISCONNECTED_BIT);
        self.set_wifi_state(WifiManagerState::DISCONNECTED);
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        const auto* e = static_cast<const ip_event_got_ip_t*>(data);
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&e->ip_info.ip));
        xEventGroupClearBits(self.event_group_, WM_DISCONNECTED_BIT);
        xEventGroupSetBits(self.event_group_, WM_CONNECTED_BIT);
        self.set_wifi_state(WifiManagerState::CONNECTED);
        self.start_or_restart_sntp();
    }
}

// ─── Scan ──────────────────────────────────────────────────────────────────

int WifiManager::scan_pick_best() {
    constexpr wifi_scan_config_t cfg = {
        .ssid = nullptr, .bssid = nullptr, .channel = 0,
        .show_hidden = false, .scan_type = WIFI_SCAN_TYPE_ACTIVE,
        .scan_time = { .active  = { .min = WIFI_ACTIVE_SCAN_MIN_DEFAULT_TIME,
                                    .max = WIFI_ACTIVE_SCAN_MAX_DEFAULT_TIME },
                       .passive = WIFI_PASSIVE_SCAN_DEFAULT_TIME },
        .home_chan_dwell_time = WIFI_SCAN_HOME_CHANNEL_DWELL_DEFAULT_TIME,
        .channel_bitmap = { .ghz_2_channels = 0, .ghz_5_channels = 0 }
    };
    if (esp_wifi_scan_start(&cfg, true) != ESP_OK) {
        ESP_LOGE(TAG, "scan_start failed"); return -1;
    }
    uint16_t count = 20;
    wifi_ap_record_t aps[20] = {};
    esp_wifi_scan_get_ap_records(&count, aps);
    ESP_LOGI(TAG, "scan: %u APs found", (unsigned)count);

    int    best_cred = -1;
    int8_t best_rssi = INT8_MIN;
    for (int ci = 0; ci < CRED_COUNT; ++ci) {
        if (!creds_[ci].ssid || creds_[ci].ssid[0] == '\0') continue;
        for (uint16_t ai = 0; ai < count; ++ai) {
            if (strcmp(reinterpret_cast<const char*>(aps[ai].ssid),
                       creds_[ci].ssid) == 0) {
                ESP_LOGI(TAG, "  found known AP \"%s\" ch%d rssi=%d",
                         creds_[ci].ssid, aps[ai].primary, aps[ai].rssi);
                if (aps[ai].rssi > best_rssi) {
                    best_rssi = aps[ai].rssi;
                    best_cred = ci;
                }
            }
        }
    }
    if (best_cred >= 0)
        ESP_LOGI(TAG, "scan winner: \"%s\" rssi=%d dBm",
                 creds_[best_cred].ssid, best_rssi);
    else
        ESP_LOGW(TAG, "scan: no known AP visible");
    return best_cred;
}

// ─── Connect helpers ──────────────────────────────────────────────────────

void WifiManager::disconnect_blocking() {
    xEventGroupClearBits(event_group_, WM_CONNECTED_BIT);
    esp_wifi_disconnect();
    vTaskDelay(pdMS_TO_TICKS(100));
    xEventGroupClearBits(event_group_, WM_DISCONNECTED_BIT);
}

esp_err_t WifiManager::start_connect(int ci) {
    configASSERT(ci >= 0 && ci < CRED_COUNT);
    wifi_config_t cfg = {};
    strlcpy(reinterpret_cast<char*>(cfg.sta.ssid),
            creds_[ci].ssid, sizeof(cfg.sta.ssid));
    const char* pass = creds_[ci].pass ? creds_[ci].pass : "";
    strlcpy(reinterpret_cast<char*>(cfg.sta.password), pass, sizeof(cfg.sta.password));
    cfg.sta.threshold.authmode =
        (pass[0] == '\0') ? WIFI_AUTH_OPEN : WIFI_AUTH_WPA2_PSK;
    cfg.sta.pmf_cfg.capable = true;
    ESP_LOGI(TAG, "connecting → \"%s\"", creds_[ci].ssid);
    const esp_err_t r = esp_wifi_set_config(WIFI_IF_STA, &cfg);
    return (r == ESP_OK) ? esp_wifi_connect() : r;
}

bool WifiManager::monitor_rssi() {
    wifi_ap_record_t ap;
    if (esp_wifi_sta_get_ap_info(&ap) != ESP_OK) return false;
    const int RSSI_WARN     = CONFIG_WIFI_RSSI_WARN_THRESHOLD;
    const int RSSI_CRITICAL = CONFIG_WIFI_RSSI_CRITICAL_THRESHOLD;
    const int DEGRADE_N     = CONFIG_WIFI_DEGRADED_CONFIRM_N;
    if (ap.rssi >= RSSI_WARN) {
        if (degrade_streak_ > 0)
            ESP_LOGI(TAG, "RSSI recovered: %d dBm", ap.rssi);
        degrade_streak_ = 0;
        publish_info(nullptr, ap.rssi, false);
        return false;
    }
    ++degrade_streak_;
    ESP_LOGW(TAG, "RSSI %d dBm (streak %d/%d)", ap.rssi, degrade_streak_, DEGRADE_N);
    publish_info(nullptr, ap.rssi, true);
    if (ap.rssi < RSSI_CRITICAL) { degrade_streak_ = 0; return true; }
    if (degrade_streak_ >= DEGRADE_N) { degrade_streak_ = 0; return true; }
    return false;
}

// ─── Manager task ──────────────────────────────────────────────────────────

void WifiManager::manager_task(void* arg) {
    auto* self = static_cast<WifiManager*>(arg);

    const int RETRIES_PER_NET = CONFIG_WIFI_RETRIES_PER_NETWORK;
    const int CYCLE_DELAY_S   = CONFIG_WIFI_INTER_CYCLE_DELAY_S;
    const int MONITOR_MS      = CONFIG_WIFI_MONITOR_INTERVAL_MS;
    const int CONNECT_TIMEOUT_MS = 12000;

    ESP_LOGI(TAG, "manager task started, %d slots", self->CRED_COUNT);

    ESP_ERROR_CHECK(esp_wifi_start());
    esp_wifi_set_max_tx_power(84);   // ~20 dBm

    self->set_wifi_state(WifiManagerState::DISCONNECTED);

    bool no_creds_logged = false;
    int  ci              = -1;
    bool skip_scan       = false;

    while (true) {
        if (!skip_scan) {
            int valid = 0;
            for (int i = 0; i < self->CRED_COUNT; ++i)
                if (self->creds_[i].ssid && self->creds_[i].ssid[0] != '\0') ++valid;
            if (valid == 0) {
                if (!no_creds_logged) {
                    ESP_LOGE(TAG, "No valid SSIDs configured "
                             "(set CONFIG_WIFI_SSID_1..4 in menuconfig)");
                    no_creds_logged = true;
                }
                self->set_wifi_state(WifiManagerState::OFF);
                vTaskDelay(pdMS_TO_TICKS(CYCLE_DELAY_S * 1000));
                continue;
            }
            no_creds_logged = false;
            self->set_wifi_state(WifiManagerState::DISCONNECTED);
            self->publish_info(nullptr, 0, false);
            ci = self->scan_pick_best();
            if (ci < 0) {
                ESP_LOGW(TAG, "No known AP visible, waiting %ds", CYCLE_DELAY_S);
                vTaskDelay(pdMS_TO_TICKS(CYCLE_DELAY_S * 1000));
                continue;
            }
        }
        skip_scan = false;

        self->set_wifi_state(WifiManagerState::CONNECTING);
        bool connected = false;
        ++self->reconnects_;

        for (int attempt = 1; attempt <= RETRIES_PER_NET && !connected; ++attempt) {
            self->disconnect_blocking();
            if (self->start_connect(ci) != ESP_OK) {
                vTaskDelay(pdMS_TO_TICKS(2000));
                continue;
            }
            const EventBits_t bits = xEventGroupWaitBits(
                self->event_group_, WM_CONNECTED_BIT,
                pdTRUE, pdFALSE, pdMS_TO_TICKS(CONNECT_TIMEOUT_MS));
            if (bits & WM_CONNECTED_BIT) {
                connected = true;
            } else {
                ESP_LOGW(TAG, "\"%s\" attempt %d/%d failed",
                         self->creds_[ci].ssid, attempt, RETRIES_PER_NET);
                vTaskDelay(pdMS_TO_TICKS(1500));
            }
        }

        if (!connected) {
            ESP_LOGW(TAG, "\"%s\" unreachable, rescanning", self->creds_[ci].ssid);
            vTaskDelay(pdMS_TO_TICKS(3000));
            continue;
        }

        self->connected_us_ = esp_timer_get_time();
        self->degrade_streak_ = 0;
        self->publish_info(self->creds_[ci].ssid, 0, false);
        ESP_LOGI(TAG, "connected to \"%s\" (reconnect #%lu)",
                 self->creds_[ci].ssid, (unsigned long)self->reconnects_);

        while (true) {
            const EventBits_t bits = xEventGroupWaitBits(
                self->event_group_, WM_DISCONNECTED_BIT,
                pdTRUE, pdFALSE, pdMS_TO_TICKS(MONITOR_MS));

            if (bits & WM_DISCONNECTED_BIT) {
                ESP_LOGW(TAG, "connection lost, rescanning");
                self->connected_us_ = 0;
                break;
            }

            if (self->monitor_rssi()) {
                const int better = self->scan_pick_best();
                if (better < 0) {
                    ESP_LOGW(TAG, "degraded but no alternative AP visible, staying");
                    continue;
                }
                if (better == ci) {
                    ESP_LOGI(TAG, "\"%s\" still best, staying", self->creds_[ci].ssid);
                    self->degrade_streak_ = 0;
                    continue;
                }
                ++self->switches_;
                ESP_LOGI(TAG, "roaming \"%s\" -> \"%s\" (switch #%lu)",
                         self->creds_[ci].ssid, self->creds_[better].ssid,
                         (unsigned long)self->switches_);
                ci = better;
                skip_scan = true;
                self->connected_us_ = 0;
                xEventGroupClearBits(self->event_group_, WM_CONNECTED_BIT);
                xEventGroupSetBits(self->event_group_, WM_DISCONNECTED_BIT);
                break;
            }
        }
    }
}

// ─── Public API ────────────────────────────────────────────────────────────

esp_err_t WifiManager::init() {
    if (initialized_) {
        ESP_LOGW(TAG, "init() already called – ignoring");
        return ESP_OK;
    }

    StateMachine::setInitial("wifi_manager",
                             stateToString(WifiManagerState::OFF),
                             orderedStates(WifiManagerState()));

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS erase required");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "nvs_flash_init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    netif_ = esp_netif_create_default_wifi_sta();

    event_group_ = xEventGroupCreate();
    if (!event_group_) {
        ESP_LOGE(TAG, "Failed to create event group");
        return ESP_ERR_NO_MEM;
    }
    xEventGroupSetBits(event_group_, WM_DISCONNECTED_BIT);

    ESP_ERROR_CHECK(esp_event_handler_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID,
        reinterpret_cast<esp_event_handler_t>(event_handler), nullptr));
    ESP_ERROR_CHECK(esp_event_handler_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP,
        reinterpret_cast<esp_event_handler_t>(event_handler), nullptr));

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

    initialized_ = true;
    ESP_LOGI(TAG, "WifiManager initialised (state registered once)");
    return ESP_OK;
}

esp_err_t WifiManager::start() {
    if (!initialized_) {
        ESP_LOGE(TAG, "start() called before init()");
        return ESP_ERR_INVALID_STATE;
    }
    if (task_handle_ != nullptr) {
        ESP_LOGW(TAG, "Manager task already running");
        return ESP_OK;
    }

    BaseType_t r = xTaskCreate(manager_task, "wifi_mgr", 4096,
                               this, 5, &task_handle_);
    if (r != pdPASS) {
        ESP_LOGE(TAG, "Failed to create manager task");
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "Manager task started");
    return ESP_OK;
}

void WifiManager::get_info(wifi_manager_info_t* out) {
    portENTER_CRITICAL(&info_mux_);
    *out = info_;
    portEXIT_CRITICAL(&info_mux_);
}