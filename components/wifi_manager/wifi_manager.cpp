#include <wifi_manager.hpp>
#include <state_machine.hpp>
#include <state_machine_lifecycle.hpp>

static const char* TAG = "wifi_manager";

// ---------------------------------------------------------------------------
// Credential table – populated from Kconfig (menuconfig)
// ---------------------------------------------------------------------------
struct cred_t { const char *ssid; const char *pass; };

static constexpr cred_t s_creds[] = {
    { CONFIG_WIFI_SSID_1, CONFIG_WIFI_PASS_1 },
    { CONFIG_WIFI_SSID_2, CONFIG_WIFI_PASS_2 },
    { CONFIG_WIFI_SSID_3, CONFIG_WIFI_PASS_3 },
    { CONFIG_WIFI_SSID_4, CONFIG_WIFI_PASS_4 },
};

static constexpr int CRED_COUNT = static_cast<int>(std::size(s_creds));
static_assert(CRED_COUNT > 0, "no credential slots defined");

static int valid_cred_count() {
    int valid = 0;
    for (int i = 0; i < CRED_COUNT; i++)
        if (s_creds[i].ssid && s_creds[i].ssid[0] != '\0') valid++;
    return valid;
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

// Internal event bits — must not overlap public WM_*_BIT values (BIT0, BIT1)
#define EVT_GOT_IP       BIT2
#define EVT_DISCONNECTED BIT3

static_assert((EVT_GOT_IP       & (WM_CONNECTED_BIT | WM_DISCONNECTED_BIT)) == 0);
static_assert((EVT_DISCONNECTED & (WM_CONNECTED_BIT | WM_DISCONNECTED_BIT)) == 0);
static_assert(EVT_GOT_IP != EVT_DISCONNECTED);

// ---------------------------------------------------------------------------
// SNTP — started once when we first obtain an IP address.
// esp_sntp_* is idempotent: repeated calls after the first are safe.
// We call it here (in the IP event) rather than in a later task so that SNTP
// has maximum time to settle before microros_sync reaches TIME_SYNCING.
// ---------------------------------------------------------------------------
static bool s_sntp_started = false;

static void start_sntp_once() {
    if (s_sntp_started) return;
    s_sntp_started = true;
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "pool.ntp.org");
    esp_sntp_setservername(1, "time.cloudflare.com");
    esp_sntp_init();
    ESP_LOGI(TAG, "SNTP client started (pool.ntp.org, time.cloudflare.com)");
}

// ---------------------------------------------------------------------------
// Static state
// ---------------------------------------------------------------------------
static EventGroupHandle_t   s_evt      = nullptr;
static esp_netif_t         *s_netif    = nullptr;
static wifi_manager_info_t  s_info     = {};
static portMUX_TYPE         s_info_mux = portMUX_INITIALIZER_UNLOCKED;

static int64_t  s_connected_at_us = 0;
static uint32_t s_switches        = 0;
static uint32_t s_reconnects      = 0;
static int      s_degrade_streak  = 0;

// ---------------------------------------------------------------------------
// State machine integration
// ---------------------------------------------------------------------------
static void set_wifi_state(WifiManagerState new_state) {
    const char* state_str = stateToString(new_state);
    if (!StateMachine::changeState("wifi_manager", state_str)) {
        ESP_LOGE(TAG, "Failed to transition to %s", state_str);
    } else {
        ESP_LOGI(TAG, "State → %s", state_str);
    }
    portENTER_CRITICAL(&s_info_mux);
    switch (new_state) {
        case WifiManagerState::OFF:          s_info.state = WM_STATE_IDLE;       break;
        case WifiManagerState::DISCONNECTED: s_info.state = WM_STATE_SCANNING;   break;
        case WifiManagerState::CONNECTING:   s_info.state = WM_STATE_CONNECTING; break;
        case WifiManagerState::CONNECTED:    s_info.state = WM_STATE_CONNECTED;  break;
        default:                             s_info.state = WM_STATE_IDLE;       break;
    }
    portEXIT_CRITICAL(&s_info_mux);
}

static void publish_info(const char *ssid, int8_t rssi, bool degraded) {
    esp_netif_ip_info_t ip = {};
    if (s_netif) esp_netif_get_ip_info(s_netif, &ip);
    char ip_str[16] = "";
    if (ip.ip.addr) snprintf(ip_str, sizeof(ip_str), IPSTR, IP2STR(&ip.ip));
    const uint32_t uptime = (s_connected_at_us > 0)
        ? static_cast<uint32_t>((esp_timer_get_time() - s_connected_at_us) / 1000000ULL) : 0;
    portENTER_CRITICAL(&s_info_mux);
    s_info.rssi_dbm   = rssi;
    s_info.degraded   = degraded;
    s_info.uptime_s   = uptime;
    s_info.switches   = s_switches;
    s_info.reconnects = s_reconnects;
    strlcpy(s_info.ip, ip_str, sizeof(s_info.ip));
    if (ssid) strlcpy(s_info.ssid, ssid, sizeof(s_info.ssid));
    portEXIT_CRITICAL(&s_info_mux);
}

// ---------------------------------------------------------------------------
// Event handlers
// ---------------------------------------------------------------------------
static void on_wifi_event(void *arg, esp_event_base_t base, int32_t id, const void *data) {
    if (id == WIFI_EVENT_STA_DISCONNECTED) {
        const auto *d = static_cast<const wifi_event_sta_disconnected_t *>(data);
        ESP_LOGW(TAG, "STA disconnected, reason=%d", d->reason);
        xEventGroupClearBits(s_evt, WM_CONNECTED_BIT);
        xEventGroupSetBits(s_evt, WM_DISCONNECTED_BIT | EVT_DISCONNECTED);
        set_wifi_state(WifiManagerState::DISCONNECTED);
    }
}

static void on_ip_event(void *arg, esp_event_base_t base, int32_t id, const void *data) {
    if (id == IP_EVENT_STA_GOT_IP) {
        const auto *e = static_cast<const ip_event_got_ip_t *>(data);
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&e->ip_info.ip));
        xEventGroupClearBits(s_evt, WM_DISCONNECTED_BIT | EVT_DISCONNECTED);
        xEventGroupSetBits(s_evt, WM_CONNECTED_BIT | EVT_GOT_IP);
        set_wifi_state(WifiManagerState::CONNECTED);

        // Start SNTP immediately when we have a route to the internet.
        // This gives it maximum time to settle before microros_sync's
        // TIME_SYNCING state begins polling ShelfbotTimestamp::isEpochValid().
        // safe to call on every reconnect — idempotent.
        start_sntp_once();
    }
}

// ---------------------------------------------------------------------------
// Scan: pick the strongest visible known AP
// ---------------------------------------------------------------------------
static int scan_pick_best() {
    constexpr wifi_scan_config_t cfg = {
        .ssid        = nullptr, .bssid       = nullptr, .channel     = 0,
        .show_hidden = false,   .scan_type   = WIFI_SCAN_TYPE_ACTIVE,
        .scan_time   = {
            .active  = { .min = WIFI_ACTIVE_SCAN_MIN_DEFAULT_TIME,
                         .max = WIFI_ACTIVE_SCAN_MAX_DEFAULT_TIME },
            .passive = WIFI_PASSIVE_SCAN_DEFAULT_TIME },
        .home_chan_dwell_time = WIFI_SCAN_HOME_CHANNEL_DWELL_DEFAULT_TIME,
        .channel_bitmap      = { .ghz_2_channels = 0, .ghz_5_channels = 0 }
    };
    if (esp_wifi_scan_start(&cfg, true) != ESP_OK) {
        ESP_LOGE(TAG, "scan_start failed"); return -1;
    }
    uint16_t count = SCAN_MAX_APS;
    wifi_ap_record_t aps[SCAN_MAX_APS] = {};
    esp_wifi_scan_get_ap_records(&count, aps);
    ESP_LOGI(TAG, "scan: %u APs found", (unsigned)count);

    int best_cred = -1; int8_t best_rssi = INT8_MIN;
    for (int ci = 0; ci < CRED_COUNT; ci++) {
        if (!s_creds[ci].ssid || s_creds[ci].ssid[0] == '\0') continue;
        for (uint16_t ai = 0; ai < count; ai++) {
            if (strcmp(reinterpret_cast<const char *>(aps[ai].ssid), s_creds[ci].ssid) == 0) {
                ESP_LOGI(TAG, "  found known AP \"%s\" ch%d rssi=%d",
                         reinterpret_cast<const char *>(aps[ai].ssid),
                         aps[ai].primary, aps[ai].rssi);
                if (aps[ai].rssi > best_rssi) { best_rssi = aps[ai].rssi; best_cred = ci; }
            }
        }
    }
    if (best_cred >= 0) ESP_LOGI(TAG, "scan winner: \"%s\" rssi=%d dBm", s_creds[best_cred].ssid, best_rssi);
    else                ESP_LOGW(TAG, "scan: no known AP visible");
    return best_cred;
}

// ---------------------------------------------------------------------------
// Connect helpers
// ---------------------------------------------------------------------------
static void disconnect_blocking() {
    xEventGroupClearBits(s_evt, EVT_DISCONNECTED | EVT_GOT_IP);
    esp_wifi_disconnect();
    xEventGroupWaitBits(s_evt, EVT_DISCONNECTED, pdTRUE, pdFALSE, pdMS_TO_TICKS(2000));
    xEventGroupClearBits(s_evt, EVT_DISCONNECTED | EVT_GOT_IP);
}

static esp_err_t start_connect(int ci) {
    configASSERT(ci >= 0 && ci < CRED_COUNT);
    wifi_config_t cfg = {};
    strlcpy(reinterpret_cast<char *>(cfg.sta.ssid), s_creds[ci].ssid, sizeof(cfg.sta.ssid));
    const char *pass = s_creds[ci].pass ? s_creds[ci].pass : "";
    strlcpy(reinterpret_cast<char *>(cfg.sta.password), pass, sizeof(cfg.sta.password));
    cfg.sta.threshold.authmode = (pass[0] == '\0') ? WIFI_AUTH_OPEN : WIFI_AUTH_WPA2_PSK;
    cfg.sta.pmf_cfg.capable = true;
    ESP_LOGI(TAG, "connecting → \"%s\"", s_creds[ci].ssid);
    esp_err_t r = esp_wifi_set_config(WIFI_IF_STA, &cfg);
    if (r == ESP_OK) r = esp_wifi_connect();
    return r;
}

static bool monitor_rssi() {
    wifi_ap_record_t ap;
    if (esp_wifi_sta_get_ap_info(&ap) != ESP_OK) return false;
    const bool degraded = (ap.rssi < RSSI_WARN);
    if (!degraded) {
        if (s_degrade_streak > 0) ESP_LOGI(TAG, "RSSI recovered: %d dBm", ap.rssi);
        s_degrade_streak = 0; publish_info(nullptr, ap.rssi, false); return false;
    }
    s_degrade_streak++;
    ESP_LOGW(TAG, "RSSI %d dBm streak=%d/%d", ap.rssi, s_degrade_streak, DEGRADE_N);
    publish_info(nullptr, ap.rssi, true);
    if (ap.rssi < RSSI_CRITICAL) { s_degrade_streak = 0; return true; }
    if (s_degrade_streak >= DEGRADE_N) { s_degrade_streak = 0; return true; }
    return false;
}

// ---------------------------------------------------------------------------
// Manager task
// ---------------------------------------------------------------------------
[[noreturn]] static void manager_task(void *) {
    ESP_LOGI(TAG, "manager task started, %d slot(s), %d configured",
             CRED_COUNT, valid_cred_count());
    bool no_creds_logged = false;
    int ci = -1; bool skip_scan = false;

    while (true) {
        if (!skip_scan) {
            if (valid_cred_count() == 0) {
                if (!no_creds_logged) {
                    ESP_LOGE(TAG, "No valid Wi-Fi SSIDs configured — set CONFIG_WIFI_SSID_1..4 in menuconfig.");
                    no_creds_logged = true;
                }
                set_wifi_state(WifiManagerState::OFF);
                vTaskDelay(pdMS_TO_TICKS(CYCLE_DELAY_S * 1000)); continue;
            }
            no_creds_logged = false;
            set_wifi_state(WifiManagerState::DISCONNECTED);
            publish_info(nullptr, 0, false);
            ci = scan_pick_best();
            if (ci < 0) {
                ESP_LOGW(TAG, "No known AP, waiting %d s", CYCLE_DELAY_S);
                vTaskDelay(pdMS_TO_TICKS(CYCLE_DELAY_S * 1000)); continue;
            }
        }
        skip_scan = false;

        set_wifi_state(WifiManagerState::CONNECTING);
        bool connected = false; s_reconnects++;

        for (int attempt = 1; attempt <= RETRIES_PER_NET && !connected; attempt++) {
            disconnect_blocking();
            if (start_connect(ci) != ESP_OK) { vTaskDelay(pdMS_TO_TICKS(2000)); continue; }
            const EventBits_t bits = xEventGroupWaitBits(
                s_evt, EVT_GOT_IP | EVT_DISCONNECTED, pdTRUE, pdFALSE, pdMS_TO_TICKS(CONNECT_TIMEOUT_MS));
            if (bits & EVT_GOT_IP) connected = true;
            else {
                ESP_LOGW(TAG, "\"%s\" attempt %d/%d failed", s_creds[ci].ssid, attempt, RETRIES_PER_NET);
                vTaskDelay(pdMS_TO_TICKS(1500));
            }
        }
        if (!connected) {
            ESP_LOGW(TAG, "\"%s\" unreachable – rescanning", s_creds[ci].ssid);
            vTaskDelay(pdMS_TO_TICKS(3000)); continue;
        }

        s_connected_at_us = esp_timer_get_time(); s_degrade_streak = 0;
        publish_info(s_creds[ci].ssid, 0, false);
        ESP_LOGI(TAG, "connected to \"%s\" (reconnect #%lu)",
                 s_creds[ci].ssid, (unsigned long)s_reconnects);

        while (true) {
            const EventBits_t bits = xEventGroupWaitBits(
                s_evt, EVT_DISCONNECTED, pdTRUE, pdFALSE, pdMS_TO_TICKS(MONITOR_MS));
            if (bits & EVT_DISCONNECTED) {
                ESP_LOGW(TAG, "connection lost – rescanning");
                s_connected_at_us = 0; set_wifi_state(WifiManagerState::DISCONNECTED); break;
            }
            if (monitor_rssi()) {
                const int better = scan_pick_best();
                if (better < 0) { ESP_LOGW(TAG, "degraded but no alternatives – staying"); continue; }
                if (better == ci) { ESP_LOGI(TAG, "\"%s\" is still best – staying", s_creds[ci].ssid); continue; }
                s_switches++;
                ESP_LOGI(TAG, "switching \"%s\" → \"%s\" (switch #%lu)",
                         s_creds[ci].ssid, s_creds[better].ssid, (unsigned long)s_switches);
                ci = better; skip_scan = true; s_connected_at_us = 0;
                xEventGroupClearBits(s_evt, WM_CONNECTED_BIT);
                xEventGroupSetBits(s_evt, WM_DISCONNECTED_BIT); break;
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------
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
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
        reinterpret_cast<esp_event_handler_t>(on_wifi_event), nullptr));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
        reinterpret_cast<esp_event_handler_t>(on_ip_event), nullptr));

    StateMachine::setInitial("wifi_manager", stateToString(WifiManagerState::OFF));
    set_wifi_state(WifiManagerState::OFF);

    const BaseType_t r = xTaskCreatePinnedToCore(
        manager_task, "wifi_mgr", 4096, nullptr,
        configMAX_PRIORITIES - 2, nullptr, 1);
    if (r != pdPASS) {
        ESP_LOGE(TAG, "failed to create manager task");
        return ESP_FAIL;
    }
    return ESP_OK;
}

void wifi_manager_get_info(wifi_manager_info_t *out) {
    portENTER_CRITICAL(&s_info_mux); *out = s_info; portEXIT_CRITICAL(&s_info_mux);
}

EventGroupHandle_t wifi_manager_get_event_group() { return s_evt; }
