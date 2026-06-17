// http_server.cpp
//
// Memory model (ESP32-D0WD-V3, no PSRAM):
//   Flash (4 MB)     — code + rodata (page HTML lives here via http_pages.cpp)
//   Internal SRAM    — 520 KB total; ~100-150 KB available at runtime after
//                      WiFi/BT stacks occupy ~200 KB and tasks use stacks.
//   Heap budget      — cJSON motor objects: ~600 bytes peak.
//                      LiDAR scan struct: 13.7 KB → declared static in handler
//                      (BSS, not stack or heap).
//   httpd stack      — 6 144 bytes (set in start()).  Enough for JSON + request
//                      parsing.  The LiDAR scan struct must NOT live on this stack.
//
// No std::string, no new/delete, no malloc after init.
// All page content is sent from flash via chunked transfer.

#include <http_server.hpp>
#include <http_pages.hpp>
#include <firmware_version.hpp>
#include <motor_control.hpp>
#include <lidar_sensor.hpp>
#include <lidar_scan.hpp>

// Binary blobs embedded by CMakeLists.txt target_add_binary_data()
extern const uint8_t binary_lidar_html_start[] asm("_binary_lidar_html_start");
extern const uint8_t binary_lidar_html_end[]   asm("_binary_lidar_html_end");
extern const uint8_t binary_lidar_viz_js_start[] asm("_binary_lidar_viz_js_start");
extern const uint8_t binary_lidar_viz_js_end[]   asm("_binary_lidar_viz_js_end");

const char* HttpServer::TAG = "HttpServer";

// ---------------------------------------------------------------------------
// CORS helpers
// ---------------------------------------------------------------------------

void HttpServer::add_cors_headers(httpd_req_t* req) {
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin",  "*");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "GET, POST, OPTIONS");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "Content-Type");
}

esp_err_t HttpServer::options_handler(httpd_req_t* req) {
    add_cors_headers(req);
    httpd_resp_send(req, nullptr, 0);
    return ESP_OK;
}

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------

HttpServer& HttpServer::get_instance() {
    static HttpServer instance;
    return instance;
}

esp_err_t HttpServer::start() {
    if (server_ != nullptr) {
        ESP_LOGW(TAG, "start() called but server is already running — no-op");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Starting HTTP server...");

    httpd_config_t config   = HTTPD_DEFAULT_CONFIG();
    config.uri_match_fn     = httpd_uri_match_wildcard;
    config.max_uri_handlers = 20;
    // 6 144 bytes: enough for JSON + request parsing.
    // LidarScan (13.7 KB) is declared static in lidar_api_handler — not on stack.
    config.stack_size       = 6144;
    // Allow a reasonable queue of pending connections.
    config.backlog_conn     = 4;

    esp_err_t err = httpd_start(&server_, &config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "httpd_start failed: %s", esp_err_to_name(err));
        server_ = nullptr;
        return err;
    }

    err = register_uri_handlers(server_);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "register_uri_handlers failed: %s — stopping httpd", esp_err_to_name(err));
        esp_err_t stop_err = httpd_stop(server_);
        if (stop_err != ESP_OK) {
            ESP_LOGE(TAG, "httpd_stop also failed: %s", esp_err_to_name(stop_err));
        }
        server_ = nullptr;
        return err;
    }

    ESP_LOGI(TAG, "HTTP server started on port %d (stack=%u bytes, max_handlers=%u)",
             (int)config.server_port,
             (unsigned)config.stack_size,
             (unsigned)config.max_uri_handlers);
    return ESP_OK;
}

esp_err_t HttpServer::stop() {
    if (server_ == nullptr) {
        ESP_LOGD(TAG, "stop() called but server is not running — no-op");
        return ESP_OK;
    }
    ESP_LOGI(TAG, "Stopping HTTP server...");
    esp_err_t err = httpd_stop(server_);
    if (err == ESP_OK) {
        server_ = nullptr;
        ESP_LOGI(TAG, "HTTP server stopped");
    } else {
        ESP_LOGE(TAG, "httpd_stop failed: %s", esp_err_to_name(err));
    }
    return err;
}

// ---------------------------------------------------------------------------
// URI handler registration
// ---------------------------------------------------------------------------

esp_err_t HttpServer::register_uri_handlers(const httpd_handle_t server) {
    // Table of GET + POST handlers.
    // Designated initialisers ensure every field is explicit.
    const httpd_uri_t routes[] = {
        {
            .uri      = "/",
            .method   = HTTP_GET,
            .handler  = root_handler,
            .user_ctx = nullptr
        },
        {
            .uri      = "/motor.html",
            .method   = HTTP_GET,
            .handler  = motor_page_handler,
            .user_ctx = nullptr
        },
        {
            .uri      = "/lidar.html",
            .method   = HTTP_GET,
            .handler  = lidar_page_handler,
            .user_ctx = nullptr
        },
        {
            .uri      = "/lidar_viz.js",
            .method   = HTTP_GET,
            .handler  = lidar_js_handler,
            .user_ctx = nullptr
        },
        {
            .uri      = "/api/lidar",
            .method   = HTTP_GET,
            .handler  = lidar_api_handler,
            .user_ctx = nullptr
        },
        {
            .uri      = "/api/motor/status",
            .method   = HTTP_GET,
            .handler  = motor_status_handler,
            .user_ctx = nullptr
        },
        {
            .uri      = "/api/motor/set",
            .method   = HTTP_POST,
            .handler  = motor_set_handler,
            .user_ctx = nullptr
        },
    };

    const size_t nRoutes = sizeof(routes) / sizeof(routes[0]);
    for (size_t i = 0; i < nRoutes; ++i) {
        esp_err_t err = httpd_register_uri_handler(server, &routes[i]);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to register handler for %s %s: %s",
                     (routes[i].method == HTTP_GET  ? "GET"  :
                      routes[i].method == HTTP_POST ? "POST" : "?"),
                     routes[i].uri,
                     esp_err_to_name(err));
            return err;
        }
        ESP_LOGD(TAG, "Registered %s %s",
                 (routes[i].method == HTTP_GET  ? "GET"  :
                  routes[i].method == HTTP_POST ? "POST" : "?"),
                 routes[i].uri);
    }

    // OPTIONS pre-flight handlers for CORS endpoints.
    const char* cors_endpoints[] = {
        "/api/lidar",
        "/api/motor/status",
        "/api/motor/set",
    };
    const size_t nCors = sizeof(cors_endpoints) / sizeof(cors_endpoints[0]);
    for (size_t i = 0; i < nCors; ++i) {
        const httpd_uri_t opt = {
            .uri      = cors_endpoints[i],
            .method   = HTTP_OPTIONS,
            .handler  = options_handler,
            .user_ctx = nullptr
        };
        esp_err_t err = httpd_register_uri_handler(server, &opt);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to register OPTIONS %s: %s",
                     cors_endpoints[i], esp_err_to_name(err));
            return err;
        }
    }

    ESP_LOGI(TAG, "All URI handlers registered (%zu routes + %zu CORS OPTIONS)",
             nRoutes, nCors);
    return ESP_OK;
}

// ---------------------------------------------------------------------------
// Page handlers
// ---------------------------------------------------------------------------

esp_err_t HttpServer::root_handler(httpd_req_t* req) {
    // kPageRoot is a flash rodata string — no heap needed.
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    esp_err_t err = httpd_resp_send(req, kPageRoot, (ssize_t)kPageRootLen);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "root_handler: send failed: %s", esp_err_to_name(err));
    }
    return err;
}

esp_err_t HttpServer::motor_page_handler(httpd_req_t* req) {
    // The motor page injects the firmware version string between prefix and
    // suffix chunks.  Chunked transfer — no heap allocation.
    httpd_resp_set_type(req, "text/html; charset=utf-8");

    esp_err_t err;
    err = httpd_resp_send_chunk(req, kPageMotorPrefix, (ssize_t)kPageMotorPrefixLen);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "motor_page_handler: send prefix failed: %s", esp_err_to_name(err));
        return err;
    }

    const char* ver = FirmwareVersion::get_version_string();
    err = httpd_resp_send_chunk(req, ver, (ssize_t)strlen(ver));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "motor_page_handler: send version failed: %s", esp_err_to_name(err));
        return err;
    }

    err = httpd_resp_send_chunk(req, kPageMotorSuffix, (ssize_t)kPageMotorSuffixLen);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "motor_page_handler: send suffix failed: %s", esp_err_to_name(err));
        return err;
    }

    // Terminate chunked response.
    err = httpd_resp_send_chunk(req, nullptr, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "motor_page_handler: send terminator failed: %s", esp_err_to_name(err));
    }
    return err;
}

esp_err_t HttpServer::lidar_page_handler(httpd_req_t* req) {
    const size_t len = (size_t)(binary_lidar_html_end - binary_lidar_html_start);
    if (len == 0) {
        ESP_LOGE(TAG, "lidar_page_handler: embedded lidar.html is empty");
        return httpd_resp_send_500(req);
    }
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    esp_err_t err = httpd_resp_send(req,
        reinterpret_cast<const char*>(binary_lidar_html_start), (ssize_t)len);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "lidar_page_handler: send failed: %s", esp_err_to_name(err));
    }
    return err;
}

esp_err_t HttpServer::lidar_js_handler(httpd_req_t* req) {
    const size_t len = (size_t)(binary_lidar_viz_js_end - binary_lidar_viz_js_start);
    if (len == 0) {
        ESP_LOGE(TAG, "lidar_js_handler: embedded lidar_viz.js is empty");
        return httpd_resp_send_500(req);
    }
    httpd_resp_set_type(req, "application/javascript; charset=utf-8");
    esp_err_t err = httpd_resp_send(req,
        reinterpret_cast<const char*>(binary_lidar_viz_js_start), (ssize_t)len);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "lidar_js_handler: send failed: %s", esp_err_to_name(err));
    }
    return err;
}

// ---------------------------------------------------------------------------
// Motor API handlers
// ---------------------------------------------------------------------------

esp_err_t HttpServer::motor_status_handler(httpd_req_t* req) {
    add_cors_headers(req);

    cJSON* root = cJSON_CreateObject();
    if (!root) {
        ESP_LOGE(TAG, "motor_status_handler: cJSON_CreateObject failed (heap exhausted?)");
        return httpd_resp_send_500(req);
    }

    cJSON* motors = cJSON_CreateArray();
    if (!motors) {
        ESP_LOGE(TAG, "motor_status_handler: cJSON_CreateArray failed");
        cJSON_Delete(root);
        return httpd_resp_send_500(req);
    }
    cJSON_AddItemToObject(root, "motors", motors);

    for (int i = 0; i < NUM_MOTORS; i++) {
        cJSON* m = cJSON_CreateObject();
        if (!m) {
            ESP_LOGE(TAG, "motor_status_handler: cJSON_CreateObject failed for motor %d", i);
            cJSON_Delete(root);
            return httpd_resp_send_500(req);
        }
        cJSON_AddNumberToObject(m, "motor",          i);
        cJSON_AddNumberToObject(m, "position_rad",   motor_control_get_position(i));
        cJSON_AddNumberToObject(m, "velocity_rad_s", motor_control_get_velocity(i));
        cJSON_AddBoolToObject  (m, "running",        motor_control_is_motor_running(i));
        cJSON_AddItemToArray(motors, m);
    }

    char* json = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);

    if (!json) {
        ESP_LOGE(TAG, "motor_status_handler: cJSON_PrintUnformatted returned NULL");
        return httpd_resp_send_500(req);
    }

    httpd_resp_set_type(req, "application/json");
    esp_err_t err = httpd_resp_send(req, json, HTTPD_RESP_USE_STRLEN);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "motor_status_handler: send failed: %s", esp_err_to_name(err));
    }
    cJSON_free(json);
    return err;
}

esp_err_t HttpServer::motor_set_handler(httpd_req_t* req) {
    add_cors_headers(req);

    // Maximum body size: motor index (1 byte int) + two floats + JSON overhead.
    // 255 bytes is more than sufficient and keeps the buffer on the stack.
    static constexpr size_t MAX_BODY = 255u;

    if ((size_t)req->content_len > MAX_BODY) {
        ESP_LOGW(TAG, "motor_set_handler: body too large (%d > %zu bytes)",
                 (int)req->content_len, MAX_BODY);
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                   "request body too large (max 255 bytes)");
    }

    // Stack-allocated receive buffer — no heap.
    char buf[MAX_BODY + 1u];
    int  received = httpd_req_recv(req, buf, MAX_BODY);
    if (received <= 0) {
        ESP_LOGW(TAG, "motor_set_handler: recv returned %d (%s)",
                 received, received == 0 ? "connection closed" : "error");
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "invalid or empty body");
    }
    buf[received] = '\0';

    cJSON* body = cJSON_Parse(buf);
    if (!body) {
        ESP_LOGW(TAG, "motor_set_handler: JSON parse failed on: %.*s", received, buf);
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "malformed JSON");
    }

    const cJSON* motor_item = cJSON_GetObjectItemCaseSensitive(body, "motor");
    const cJSON* pos        = cJSON_GetObjectItemCaseSensitive(body, "position_rad");
    const cJSON* vel        = cJSON_GetObjectItemCaseSensitive(body, "velocity_rad_s");

    if (!cJSON_IsNumber(motor_item)) {
        ESP_LOGW(TAG, "motor_set_handler: missing or non-numeric 'motor' field");
        cJSON_Delete(body);
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "missing 'motor' field");
    }

    const int    idx      = motor_item->valueint;
    const float  pos_rad  = cJSON_IsNumber(pos) ? (float)pos->valuedouble  : 0.0f;
    const float  vel_rads = cJSON_IsNumber(vel) ? (float)vel->valuedouble  : 0.0f;

    if (idx < 0 || idx >= NUM_MOTORS) {
        ESP_LOGW(TAG, "motor_set_handler: motor index %d out of range [0,%d)", idx, NUM_MOTORS);
        cJSON_Delete(body);
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "motor index out of range");
    }

    ESP_LOGD(TAG, "motor_set: motor=%d pos=%.4f vel=%.4f", idx, (double)pos_rad, (double)vel_rads);
    motor_control_apply((uint8_t)idx, pos_rad, vel_rads);
    cJSON_Delete(body);

    httpd_resp_set_type(req, "application/json");
    esp_err_t err = httpd_resp_sendstr(req, "{\"ok\":true}");
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "motor_set_handler: send failed: %s", esp_err_to_name(err));
    }
    return err;
}

// ---------------------------------------------------------------------------
// LiDAR API handler
//
// Memory note:
//   LidarScan is ~13.7 KB.  The httpd task stack is 6 KB — declaring a
//   LidarScan on the stack would immediately overflow it.  Declaring it
//   static here means it lives in BSS (zero-initialized data segment).
//   The httpd server serialises requests on a single task so a single
//   static instance is safe: only one request handler runs at a time.
//
//   cJSON points array: capped at MAX_JSON_POINTS.
//   A full 360° LYDSTO scan is ~150 points.  400 points × ~42 bytes ≈ 17 KB
//   of heap used by cJSON during serialisation, freed immediately after send.
// ---------------------------------------------------------------------------

esp_err_t HttpServer::lidar_api_handler(httpd_req_t* req) {
    add_cors_headers(req);

    // Static: keeps 13.7 KB off the httpd task stack.
    static LidarScan scan;

    if (!lidar_get_latest_scan(scan)) {
        // No scan available — send a well-formed JSON error; do NOT crash.
        static const char kNoScan[] =
            "{\"error\":\"No LiDAR scan available yet\",\"available\":false}";
        httpd_resp_set_type(req, "application/json");
        esp_err_t err = httpd_resp_send(req, kNoScan, (ssize_t)(sizeof(kNoScan) - 1u));
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "lidar_api_handler: send (no-scan) failed: %s", esp_err_to_name(err));
        }
        return err;
    }

    // Build JSON response.
    cJSON* root = cJSON_CreateObject();
    if (!root) {
        ESP_LOGE(TAG, "lidar_api_handler: cJSON_CreateObject failed (heap exhausted?)");
        return httpd_resp_send_500(req);
    }

    cJSON_AddNumberToObject(root, "timestamp_us", (double)scan.end_time_us);
    cJSON_AddNumberToObject(root, "timestamp_ms", (double)(scan.end_time_us / 1000));
    cJSON_AddNumberToObject(root, "point_count",  (double)scan.point_count);
    cJSON_AddBoolToObject  (root, "complete",     scan.complete);

    // Compute minimum distance and its angle.
    uint16_t min_mm    = 0xFFFFu;
    float    min_angle = 0.0f;
    for (uint16_t i = 0; i < scan.point_count; ++i) {
        if (scan.distances_mm[i] > 0u && scan.distances_mm[i] < min_mm) {
            min_mm    = scan.distances_mm[i];
            min_angle = scan.angles_deg[i];
        }
    }
    if (min_mm == 0xFFFFu) min_mm = 0u;

    cJSON_AddNumberToObject(root, "distance_mm",            (double)min_mm);
    cJSON_AddNumberToObject(root, "distance_cm",            (double)min_mm / 10.0);
    cJSON_AddNumberToObject(root, "min_distance_angle_deg", (double)min_angle);

    const float start_deg = (scan.point_count > 0u) ? scan.angles_deg[0]                     : 0.0f;
    const float end_deg   = (scan.point_count > 0u) ? scan.angles_deg[scan.point_count - 1u] : 0.0f;
    cJSON_AddNumberToObject(root, "start_angle_deg", (double)start_deg);
    cJSON_AddNumberToObject(root, "end_angle_deg",   (double)end_deg);

    // Cap at MAX_JSON_POINTS to avoid exhausting heap.
    static constexpr uint16_t MAX_JSON_POINTS = 400u;
    const uint16_t n = (scan.point_count < MAX_JSON_POINTS) ? scan.point_count : MAX_JSON_POINTS;

    cJSON* points = cJSON_CreateArray();
    if (!points) {
        ESP_LOGE(TAG, "lidar_api_handler: cJSON_CreateArray failed");
        cJSON_Delete(root);
        return httpd_resp_send_500(req);
    }

    bool alloc_ok = true;
    for (uint16_t i = 0; i < n && alloc_ok; ++i) {
        cJSON* pt = cJSON_CreateObject();
        if (!pt) {
            ESP_LOGE(TAG, "lidar_api_handler: cJSON_CreateObject failed at point %u (heap low?)", (unsigned)i);
            alloc_ok = false;
            break;
        }
        cJSON_AddNumberToObject(pt, "d",         (double)scan.distances_mm[i]);
        cJSON_AddNumberToObject(pt, "c",         (double)scan.confidences[i]);
        cJSON_AddNumberToObject(pt, "angle_deg", (double)scan.angles_deg[i]);
        cJSON_AddItemToArray(points, pt);
    }

    if (!alloc_ok) {
        // Partial build — still try to send what we have rather than a 500.
        ESP_LOGW(TAG, "lidar_api_handler: partial point array due to heap pressure");
    }

    cJSON_AddItemToObject(root, "points", points);

    char* json_str = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);  // frees all children including points array

    if (!json_str) {
        ESP_LOGE(TAG, "lidar_api_handler: cJSON_PrintUnformatted returned NULL (heap exhausted?)");
        return httpd_resp_send_500(req);
    }

    httpd_resp_set_type(req, "application/json");
    esp_err_t ret = httpd_resp_send(req, json_str, HTTPD_RESP_USE_STRLEN);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "lidar_api_handler: send failed: %s", esp_err_to_name(ret));
    }
    cJSON_free(json_str);
    return ret;
}
