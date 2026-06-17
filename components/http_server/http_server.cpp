#include <http_server.hpp>
#include <motor_control.hpp>
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
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin",  "*");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "GET, POST, OPTIONS");
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
    httpd_config_t config  = HTTPD_DEFAULT_CONFIG();
    config.uri_match_fn    = httpd_uri_match_wildcard;
    config.max_uri_handlers = 20;
    config.stack_size      = 8192;

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
    if (server_ == nullptr) return ESP_OK;
    ESP_LOGI(TAG, "Stopping HTTP server...");
    esp_err_t err = httpd_stop(server_);
    if (err == ESP_OK) server_ = nullptr;
    return err;
}

esp_err_t HttpServer::register_uri_handlers(const httpd_handle_t server) {
    const httpd_uri_t routes[] = {
        { .uri = "/",                 .method = HTTP_GET,  .handler = root_handler,         .user_ctx = nullptr },
        { .uri = "/motor.html",       .method = HTTP_GET,  .handler = motor_page_handler,   .user_ctx = nullptr },
        { .uri = "/lidar.html",       .method = HTTP_GET,  .handler = lidar_page_handler,   .user_ctx = nullptr },
        { .uri = "/lidar_viz.js",     .method = HTTP_GET,  .handler = lidar_js_handler,     .user_ctx = nullptr },
        { .uri = "/api/lidar",        .method = HTTP_GET,  .handler = lidar_api_handler,    .user_ctx = nullptr },
        { .uri = "/api/motor/status", .method = HTTP_GET,  .handler = motor_status_handler, .user_ctx = nullptr },
        { .uri = "/api/motor/set",    .method = HTTP_POST, .handler = motor_set_handler,    .user_ctx = nullptr },
    };
    for (const auto& r : routes) {
        httpd_register_uri_handler(server, &r);
    }

    const char* cors_endpoints[] = {
        "/api/lidar",
        "/api/motor/status",
        "/api/motor/set",
    };
    for (const auto& endpoint : cors_endpoints) {
        httpd_uri_t opt = {
            .uri      = endpoint,
            .method   = HTTP_OPTIONS,
            .handler  = options_handler,
            .user_ctx = nullptr
        };
        httpd_register_uri_handler(server, &opt);
    }

    return ESP_OK;
}

// ---------------------------------------------------------------------------
// Page handlers
// ---------------------------------------------------------------------------

esp_err_t HttpServer::root_handler(httpd_req_t* req) {
    const std::string html = std::string(R"(
<!DOCTYPE html>
<html>
<head>
    <title>Shelfbot ESP32</title>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body { font-family: Inter, Arial, sans-serif; margin: 0; background:#0f172a; color:#e2e8f0; }
        .container { max-width: 1100px; margin: 0 auto; padding: 28px; }
        h1 { margin: 0 0 6px; }
        .subtitle { color:#94a3b8; margin-bottom:24px; }
        .grid { display:grid; grid-template-columns: repeat(auto-fit,minmax(240px,1fr)); gap:16px; }
        .card { background:#1e293b; border:1px solid #334155; padding:16px; border-radius:12px; }
        .card h3 { margin-top:0; }
        code { background:#0b1220; padding:2px 6px; border-radius:4px; color:#93c5fd; }
        a { color: #93c5fd; text-decoration: none; font-weight:600; }
        a:hover { text-decoration: underline; }
    </style>
</head>
<body>
    <div class="container">
        <h1>🤖 Shelfbot Dashboard</h1>
        <p class="subtitle">Firmware: )") + FirmwareVersion::get_version_string() + R"(</p>
        <div class="grid">
            <div class="card"><h3>🗺️ LiDAR Viewer</h3><p><a href="/lidar.html">Open live LiDAR visualisation</a></p><code>/lidar.html</code></div>
            <div class="card"><h3>🛰️ LiDAR API</h3><p><a href="/api/lidar">Open LiDAR JSON</a></p><code>GET /api/lidar</code></div>
            <div class="card"><h3>⚙️ Motor Control</h3><p><a href="/motor.html">Open motor control page</a></p><code>/motor.html</code></div>
            <div class="card"><h3>⚙️ Motor API</h3><p><a href="/api/motor/status">Open motor status JSON</a></p><code>GET /api/motor/status</code></div>
        </div>
    </div>
</body>
</html>
)";
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    return httpd_resp_send(req, html.c_str(), HTTPD_RESP_USE_STRLEN);
}

esp_err_t HttpServer::motor_page_handler(httpd_req_t* req) {
    std::string page = std::string(R"HTML(
<!doctype html><html><head><meta name='viewport' content='width=device-width,initial-scale=1'>
<style>
body{font-family:Inter,Arial;background:#0b1020;color:#e2e8f0;margin:0;padding:20px}
.top{display:flex;justify-content:space-between;align-items:center}.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(280px,1fr));gap:14px;margin-top:16px}
.card{background:#141b34;border:1px solid #334155;border-radius:14px;padding:14px}.row{display:flex;gap:8px;align-items:center;margin:8px 0}
input{width:100%;padding:8px;border-radius:8px;border:1px solid #475569;background:#0f172a;color:#e2e8f0}
button{padding:8px 12px;border:0;border-radius:10px;background:#22c55e;color:#04110a;font-weight:700;cursor:pointer}
.json{background:#020617;border-radius:10px;padding:10px;font-family:monospace;min-height:90px;white-space:pre-wrap}
.pill{padding:2px 8px;border-radius:999px;background:#1e293b;font-size:12px}
</style></head><body>
<div class='top'><h2>Motor Control Dashboard</h2><a href='/' style='color:#93c5fd'>← Main Dashboard</a></div>
<div class='pill'>Firmware: )HTML") + FirmwareVersion::get_version_string() + R"HTML(</div>
<p>Per-motor independent set/get using micro-ROS units: <b>position_rad</b> and <b>velocity_rad_s</b>.</p>
<div class='grid' id='motors'></div>
<script>
const N=5;
function card(i){return `<div class='card'>
  <div class='row'><h3 style='margin:0'>Motor ${i}</h3><span class='pill' id='run${i}'>--</span></div>
  <div class='row'><label>Position (rad) — signed, 0 = continuous</label></div>
  <div class='row'><input id='pos${i}' type='number' step='0.01' value='0'></div>
  <div class='row'><label>Velocity (rad/s) — signed, 0 = stop</label></div>
  <div class='row'><input id='vel${i}' type='number' step='0.01' value='0'></div>
  <div class='row'>
    <button onclick='send(${i})'>Apply</button>
    <button style='background:#ef4444;color:#fff' onclick='stop(${i})'>Stop</button>
  </div>
  <div class='row' style='font-size:12px;color:#94a3b8'>
    pos: <span id='apos${i}'>--</span> rad &nbsp;|&nbsp;
    vel: <span id='avel${i}'>--</span> rad/s
  </div>
  <div class='json' id='json${i}'>Loading...</div></div>`;}
document.getElementById('motors').innerHTML=[...Array(N).keys()].map(card).join('');
async function load(){
  try {
    const data=await (await fetch('/api/motor/status')).json();
    data.motors.forEach(m=>{
      document.getElementById('run'+m.motor).textContent=m.running?'▶ RUNNING':'■ IDLE';
      document.getElementById('run'+m.motor).style.background=m.running?'#14532d':'#1e293b';
      document.getElementById('apos'+m.motor).textContent=m.position_rad.toFixed(4);
      const vel=m.velocity_rad_s;
      const velEl=document.getElementById('avel'+m.motor);
      velEl.textContent=(vel>=0?'+':'')+vel.toFixed(4);
      velEl.style.color=vel>0?'#4ade80':vel<0?'#f87171':'#94a3b8';
      document.getElementById('json'+m.motor).textContent=JSON.stringify(m,null,2);
    });
  } catch(e) { console.error('status fetch failed',e); }
}
async function send(i){
  const b={motor:i,position_rad:+document.getElementById('pos'+i).value,velocity_rad_s:+document.getElementById('vel'+i).value};
  await fetch('/api/motor/set',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(b)});
  load();
}
async function stop(i){
  await fetch('/api/motor/set',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({motor:i,position_rad:0,velocity_rad_s:0})});
  load();
}
load(); setInterval(load,500);
</script></body></html>)HTML";
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    return httpd_resp_send(req, page.c_str(), HTTPD_RESP_USE_STRLEN);
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
// Motor API handlers
// ---------------------------------------------------------------------------

esp_err_t HttpServer::motor_status_handler(httpd_req_t* req) {
    add_cors_headers(req);
    cJSON* root   = cJSON_CreateObject();
    cJSON* motors = cJSON_CreateArray();
    for (int i = 0; i < NUM_MOTORS; i++) {
        cJSON* m = cJSON_CreateObject();
        cJSON_AddNumberToObject(m, "motor",          i);
        cJSON_AddNumberToObject(m, "position_rad",   motor_control_get_position(i));
        cJSON_AddNumberToObject(m, "velocity_rad_s", motor_control_get_velocity(i));
        cJSON_AddBoolToObject  (m, "running",        motor_control_is_motor_running(i));
        cJSON_AddItemToArray(motors, m);
    }
    cJSON_AddItemToObject(root, "motors", motors);
    char* json = cJSON_PrintUnformatted(root);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json, HTTPD_RESP_USE_STRLEN);
    cJSON_free(json);
    cJSON_Delete(root);
    return ESP_OK;
}

esp_err_t HttpServer::motor_set_handler(httpd_req_t* req) {
    add_cors_headers(req);

    static constexpr size_t MAX_BODY = 255;
    if (req->content_len > MAX_BODY) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "request body too large (max 255 bytes)");
    }

    char buf[MAX_BODY + 1];
    int len = httpd_req_recv(req, buf, MAX_BODY);
    if (len <= 0) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "invalid body");
    }
    buf[len] = '\0';

    cJSON* body = cJSON_Parse(buf);
    if (!body) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad json");
    }

    cJSON* motor_item = cJSON_GetObjectItem(body, "motor");
    cJSON* pos        = cJSON_GetObjectItem(body, "position_rad");
    cJSON* vel        = cJSON_GetObjectItem(body, "velocity_rad_s");

    if (cJSON_IsNumber(motor_item)) {
        const uint8_t idx    = static_cast<uint8_t>(motor_item->valueint);
        const float pos_rad  = cJSON_IsNumber(pos) ? static_cast<float>(pos->valuedouble) : 0.0f;
        const float vel_rads = cJSON_IsNumber(vel) ? static_cast<float>(vel->valuedouble) : 0.0f;
        motor_control_apply(idx, pos_rad, vel_rads);
    }
    cJSON_Delete(body);

    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, "{\"ok\":true}");
}

// ---------------------------------------------------------------------------
// LiDAR API handler
//
// LidarScan is ~14 KB — declaring it on the httpd task stack (8 KB) causes
// an immediate stack overflow and LoadProhibited panic.  Declared static so
// it lives in BSS.  The httpd server serialises requests on a single task so
// this is safe: only one request handler runs at a time.
//
// Points are capped at MAX_JSON_POINTS before building the cJSON array.
// A full 360° LYDSTO scan at ~150 pts/rev easily fits within 400 points.
// Sending all 2000 possible points would allocate ~80 KB of heap per request
// which exhausts the ESP32 heap under any load.
// ---------------------------------------------------------------------------

esp_err_t HttpServer::lidar_api_handler(httpd_req_t* req) {
    add_cors_headers(req);

    // Static: keeps 13.7 KB off the httpd task stack.
    static LidarScan scan;

    if (!lidar_get_latest_scan(scan)) {
        cJSON* err = cJSON_CreateObject();
        cJSON_AddStringToObject(err, "error",     "No LiDAR scan available yet");
        cJSON_AddBoolToObject  (err, "available", false);
        char* json = cJSON_PrintUnformatted(err);
        if (json) {
            httpd_resp_set_type(req, "application/json");
            httpd_resp_send(req, json, HTTPD_RESP_USE_STRLEN);
            cJSON_free(json);
        }
        cJSON_Delete(err);
        return ESP_OK;
    }

    cJSON* root = cJSON_CreateObject();

    cJSON_AddNumberToObject(root, "timestamp_us", scan.end_time_us);
    cJSON_AddNumberToObject(root, "timestamp_ms", scan.end_time_us / 1000);
    cJSON_AddNumberToObject(root, "point_count",  scan.point_count);
    cJSON_AddBoolToObject  (root, "complete",     scan.complete);

    // Compute min distance and its angle
    uint16_t min_mm    = 0xFFFF;
    float    min_angle = 0.0f;
    for (uint16_t i = 0; i < scan.point_count; ++i) {
        if (scan.distances_mm[i] > 0 && scan.distances_mm[i] < min_mm) {
            min_mm    = scan.distances_mm[i];
            min_angle = scan.angles_deg[i];
        }
    }
    if (min_mm == 0xFFFF) min_mm = 0;

    cJSON_AddNumberToObject(root, "distance_mm",            min_mm);
    cJSON_AddNumberToObject(root, "distance_cm",            min_mm / 10.0f);
    cJSON_AddNumberToObject(root, "min_distance_angle_deg", min_angle);

    float start_deg = scan.point_count > 0 ? scan.angles_deg[0]                    : 0.0f;
    float end_deg   = scan.point_count > 0 ? scan.angles_deg[scan.point_count - 1] : 0.0f;
    cJSON_AddNumberToObject(root, "start_angle_deg", start_deg);
    cJSON_AddNumberToObject(root, "end_angle_deg",   end_deg);

    // Cap points sent in the JSON response to avoid heap exhaustion.
    // 400 points × ~42 bytes/point ≈ 17 KB JSON — well within heap budget.
    // A full LYDSTO 360° scan is ~150 points so nothing is lost in practice.
    static constexpr uint16_t MAX_JSON_POINTS = 400;
    const uint16_t n = (scan.point_count < MAX_JSON_POINTS)
                       ? scan.point_count : MAX_JSON_POINTS;

    cJSON* points = cJSON_CreateArray();
    for (uint16_t i = 0; i < n; ++i) {
        cJSON* pt = cJSON_CreateObject();
        cJSON_AddNumberToObject(pt, "d",         scan.distances_mm[i]);
        cJSON_AddNumberToObject(pt, "c",         scan.confidences[i]);
        cJSON_AddNumberToObject(pt, "angle_deg", scan.angles_deg[i]);
        cJSON_AddItemToArray(points, pt);
    }
    cJSON_AddItemToObject(root, "points", points);

    char* json_str = cJSON_PrintUnformatted(root);
    esp_err_t ret = ESP_ERR_NO_MEM;
    if (json_str) {
        httpd_resp_set_type(req, "application/json");
        ret = httpd_resp_send(req, json_str, HTTPD_RESP_USE_STRLEN);
        cJSON_free(json_str);
    } else {
        ESP_LOGE(TAG, "lidar_api_handler: cJSON_PrintUnformatted returned NULL (heap exhausted?)");
        httpd_resp_send_500(req);
    }
    cJSON_Delete(root);
    return ret;
}
