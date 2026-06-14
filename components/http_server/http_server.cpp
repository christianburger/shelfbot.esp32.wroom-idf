#include <http_server.hpp>
#include <lidar_sensor.hpp>
#include <lidar_scan.hpp>

const char* HttpServer::TAG = "HttpServer";

extern const uint8_t binary_lidar_html_start[] asm("_binary_lidar_html_start");
extern const uint8_t binary_lidar_html_end[]   asm("_binary_lidar_html_end");
extern const uint8_t binary_lidar_viz_js_start[] asm("_binary_lidar_viz_js_start");
extern const uint8_t binary_lidar_viz_js_end[]   asm("_binary_lidar_viz_js_end");

// ---------------------------------------------------------------------------
// CORS helpers
// ---------------------------------------------------------------------------

static void add_cors_headers(httpd_req_t* req) {
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "GET, OPTIONS");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "Content-Type");
}

static esp_err_t options_handler(httpd_req_t* req) {
    add_cors_headers(req);
    httpd_resp_send(req, nullptr, 0);
    return ESP_OK;
}

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------

esp_err_t HttpServer::start() {
    if (server_ != nullptr) {
        ESP_LOGW(TAG, "HTTP server already running");
        return ESP_OK;
    }
    ESP_LOGI(TAG, "Starting HTTP server...");
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.uri_match_fn   = httpd_uri_match_wildcard;
    config.max_uri_handlers = 10;
    config.stack_size     = 8192;

    esp_err_t err = httpd_start(&server_, &config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start HTTP server: %s", esp_err_to_name(err));
        return err;
    }

    err = register_uri_handlers(server_);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register URI handlers");
        httpd_stop(server_);
        server_ = nullptr;
        return err;
    }

    ESP_LOGI(TAG, "HTTP server started on port %d", config.server_port);
    return ESP_OK;
}

esp_err_t HttpServer::stop() {
    if (server_ == nullptr) {
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Stopping HTTP server...");
    esp_err_t err = httpd_stop(server_);
    if (err == ESP_OK) {
        server_ = nullptr;
    }
    return err;
}

esp_err_t HttpServer::register_uri_handlers(const httpd_handle_t server) {
    // Main routes
    const httpd_uri_t routes[] = {
        { .uri = "/",           .method = HTTP_GET, .handler = root_handler,       .user_ctx = nullptr },
        { .uri = "/lidar.html", .method = HTTP_GET, .handler = lidar_page_handler, .user_ctx = nullptr },
        { .uri = "/lidar_viz.js", .method = HTTP_GET, .handler = lidar_js_handler, .user_ctx = nullptr },
        { .uri = "/api/lidar",  .method = HTTP_GET, .handler = lidar_api_handler,  .user_ctx = nullptr },
    };
    for (const auto& r : routes) {
        httpd_register_uri_handler(server, &r);
    }

    // CORS OPTIONS for API endpoint
    httpd_uri_t opt = {
        .uri      = "/api/lidar",
        .method   = HTTP_OPTIONS,
        .handler  = options_handler,
        .user_ctx = nullptr
    };
    httpd_register_uri_handler(server, &opt);

    return ESP_OK;
}

// ---------------------------------------------------------------------------
// Page handlers
// ---------------------------------------------------------------------------

esp_err_t HttpServer::root_handler(httpd_req_t* req) {
    // FIXED: Convert first raw string to std::string for concatenation
    std::string html = std::string(R"html(
<!DOCTYPE html>
<html>
<head>
    <title>Shelfbot - LiDAR Viewer</title>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body {
            font-family: Inter, Arial, sans-serif;
            background: #0f172a;
            color: #e2e8f0;
            margin: 0;
            padding: 28px;
        }
        .container {
            max-width: 800px;
            margin: 0 auto;
            text-align: center;
        }
        h1 { margin-bottom: 0; }
        .subtitle { color: #94a3b8; margin-bottom: 32px; }
        .card {
            background: #1e293b;
            border: 1px solid #334155;
            border-radius: 16px;
            padding: 24px;
            margin: 20px 0;
        }
        a {
            display: inline-block;
            background: #22c55e;
            color: #04110a;
            font-weight: bold;
            padding: 12px 24px;
            border-radius: 32px;
            text-decoration: none;
            font-size: 1.2rem;
        }
        a:hover { background: #16a34a; }
        .footer {
            margin-top: 32px;
            font-size: 0.8rem;
            color: #64748b;
        }
        code { background: #0f172a; padding: 2px 6px; border-radius: 6px; }
    </style>
</head>
<body>
<div class="container">
    <h1>🤖 Shelfbot</h1>
    <p class="subtitle">Firmware: )html") + FirmwareVersion::get_version_string() + R"html(</p>
    <div class="card">
        <h2>🛰️ LiDAR Live Viewer</h2>
        <p>Visualise 360° scans, log data, and analyse distance measurements.</p>
        <a href="/lidar.html">Launch LiDAR Viewer →</a>
    </div>
    <div class="footer">
        <code>GET /api/lidar</code> returns raw scan data (JSON)
    </div>
</div>
</body>
</html>
)html";

    httpd_resp_set_type(req, "text/html; charset=utf-8");
    return httpd_resp_send(req, html.c_str(), HTTPD_RESP_USE_STRLEN);
}

esp_err_t HttpServer::lidar_page_handler(httpd_req_t* req) {
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    const size_t len = binary_lidar_html_end - binary_lidar_html_start;
    return httpd_resp_send(req, reinterpret_cast<const char*>(binary_lidar_html_start), len);
}

esp_err_t HttpServer::lidar_js_handler(httpd_req_t* req) {
    httpd_resp_set_type(req, "application/javascript; charset=utf-8");
    const size_t len = binary_lidar_viz_js_end - binary_lidar_viz_js_start;
    return httpd_resp_send(req, reinterpret_cast<const char*>(binary_lidar_viz_js_start), len);
}

// ---------------------------------------------------------------------------
// LiDAR API handler (uses lidar_sensor component)
// ---------------------------------------------------------------------------

esp_err_t HttpServer::lidar_api_handler(httpd_req_t* req) {
    add_cors_headers(req);

    LidarScan scan;
    if (!lidar_get_latest_scan(scan)) {
        cJSON* err = cJSON_CreateObject();
        cJSON_AddStringToObject(err, "error", "No LiDAR scan available yet");
        cJSON_AddBoolToObject(err, "available", false);
        char* json = cJSON_PrintUnformatted(err);
        httpd_resp_set_type(req, "application/json");
        httpd_resp_send(req, json, HTTPD_RESP_USE_STRLEN);
        cJSON_free(json);
        cJSON_Delete(err);
        return ESP_OK;
    }

    cJSON* root = cJSON_CreateObject();

    // Basic metadata
    cJSON_AddNumberToObject(root, "timestamp_us", scan.end_time_us);
    cJSON_AddNumberToObject(root, "timestamp_ms", scan.end_time_us / 1000);
    cJSON_AddNumberToObject(root, "point_count", scan.point_count);
    cJSON_AddBoolToObject(root, "complete", scan.complete);

    // Compute min distance and its angle from the scan
    uint16_t min_mm = 0xFFFF;
    float min_angle = 0.0f;
    for (uint16_t i = 0; i < scan.point_count; ++i) {
        if (scan.distances_mm[i] > 0 && scan.distances_mm[i] < min_mm) {
            min_mm = scan.distances_mm[i];
            min_angle = scan.angles_deg[i];
        }
    }
    if (min_mm == 0xFFFF) min_mm = 0;

    cJSON_AddNumberToObject(root, "distance_mm", min_mm);
    cJSON_AddNumberToObject(root, "distance_cm", min_mm / 10.0f);
    cJSON_AddNumberToObject(root, "min_distance_angle_deg", min_angle);

    // Approximate start/end angles from first and last point
    float start_deg = scan.point_count > 0 ? scan.angles_deg[0] : 0.0f;
    float end_deg   = scan.point_count > 0 ? scan.angles_deg[scan.point_count - 1] : 0.0f;
    cJSON_AddNumberToObject(root, "start_angle_deg", start_deg);
    cJSON_AddNumberToObject(root, "end_angle_deg", end_deg);

    // Points array – format expected by lidar_viz.js
    cJSON* points = cJSON_CreateArray();
    for (uint16_t i = 0; i < scan.point_count; ++i) {
        cJSON* pt = cJSON_CreateObject();
        cJSON_AddNumberToObject(pt, "d", scan.distances_mm[i]);
        cJSON_AddNumberToObject(pt, "c", scan.confidences[i]);
        cJSON_AddNumberToObject(pt, "angle_deg", scan.angles_deg[i]);
        cJSON_AddItemToArray(points, pt);
    }
    cJSON_AddItemToObject(root, "points", points);

    char* json_str = cJSON_PrintUnformatted(root);
    httpd_resp_set_type(req, "application/json");
    esp_err_t ret = httpd_resp_send(req, json_str, HTTPD_RESP_USE_STRLEN);
    cJSON_free(json_str);
    cJSON_Delete(root);
    return ret;
}