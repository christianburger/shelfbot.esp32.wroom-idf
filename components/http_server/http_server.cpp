#include "http_server.hpp"
#include <esp_timer.h>
#include <esp_task_wdt.h>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include "firmware_version.hpp"

const char* HttpServer::TAG = "HttpServer";

// Helper function to add CORS headers
static void add_cors_headers(httpd_req_t* req) {
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "GET, POST, OPTIONS");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "Content-Type");
}

// Handle OPTIONS requests for CORS
static esp_err_t options_handler(httpd_req_t* req) {
    add_cors_headers(req);
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}

esp_err_t HttpServer::start() {
    if (server_ != nullptr) {
        ESP_LOGW(TAG, "HTTP server already running");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Starting HTTP server...");

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.uri_match_fn = httpd_uri_match_wildcard;
    config.max_uri_handlers = 20;
    config.stack_size = 8192;

    esp_err_t err = httpd_start(&server_, &config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start HTTP server: %s", esp_err_to_name(err));
        return err;
    }

    err = register_uri_handlers();
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

esp_err_t HttpServer::register_uri_handlers() {
    // Root endpoint
    httpd_uri_t root_uri = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = root_handler,
        .user_ctx = nullptr
    };
    httpd_register_uri_handler(server_, &root_uri);

    // ToF sensor endpoint
    httpd_uri_t tof_uri = {
        .uri = "/api/tof",
        .method = HTTP_GET,
        .handler = tof_handler,
        .user_ctx = nullptr
    };
    httpd_register_uri_handler(server_, &tof_uri);

    // LiDAR endpoint
    httpd_uri_t lidar_uri = {
        .uri = "/api/lidar",
        .method = HTTP_GET,
        .handler = lidar_handler,
        .user_ctx = nullptr
    };
    httpd_register_uri_handler(server_, &lidar_uri);

    // Ultrasonic sensors endpoint
    httpd_uri_t ultrasonic_uri = {
        .uri = "/api/ultrasonic",
        .method = HTTP_GET,
        .handler = ultrasonic_handler,
        .user_ctx = nullptr
    };
    httpd_register_uri_handler(server_, &ultrasonic_uri);

    // All sensors endpoint
    httpd_uri_t sensors_uri = {
        .uri = "/api/sensors",
        .method = HTTP_GET,
        .handler = sensors_handler,
        .user_ctx = nullptr
    };
    httpd_register_uri_handler(server_, &sensors_uri);

    // Health endpoint
    httpd_uri_t health_uri = {
        .uri = "/api/health",
        .method = HTTP_GET,
        .handler = health_handler,
        .user_ctx = nullptr
    };
    httpd_register_uri_handler(server_, &health_uri);

    httpd_uri_t motor_page_uri = {
        .uri = "/motor.html",
        .method = HTTP_GET,
        .handler = motor_page_handler,
        .user_ctx = nullptr
    };
    httpd_register_uri_handler(server_, &motor_page_uri);

    httpd_uri_t motor_status_uri = {
        .uri = "/api/motor/status",
        .method = HTTP_GET,
        .handler = motor_status_handler,
        .user_ctx = nullptr
    };
    httpd_register_uri_handler(server_, &motor_status_uri);

    httpd_uri_t motor_set_uri = {
        .uri = "/api/motor/set",
        .method = HTTP_POST,
        .handler = motor_set_handler,
        .user_ctx = nullptr
    };
    httpd_register_uri_handler(server_, &motor_set_uri);

    // CORS OPTIONS handlers
    const char* cors_endpoints[] = {
        "/api/tof",
        "/api/lidar",
        "/api/ultrasonic",
        "/api/sensors",
        "/api/health",
        "/api/motor/status",
        "/api/motor/set",
    };

    for (const auto& endpoint : cors_endpoints) {
        httpd_uri_t options_uri = {
            .uri = endpoint,
            .method = HTTP_OPTIONS,
            .handler = options_handler,
            .user_ctx = nullptr
        };
        httpd_register_uri_handler(server_, &options_uri);
    }

    return ESP_OK;
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
  <div class='row'><label>Position (rad)</label></div><div class='row'><input id='pos${i}' type='number' step='0.01' value='0'></div>
  <div class='row'><label>Velocity (rad/s)</label></div><div class='row'><input id='vel${i}' type='number' step='0.01' value='0'></div>
  <div class='row'><button onclick='send(${i})'>Apply Motor ${i}</button></div>
  <div class='json' id='json${i}'>Loading...</div></div>`;}
document.getElementById('motors').innerHTML=[...Array(N).keys()].map(card).join('');
async function load(){const data=await (await fetch('/api/motor/status')).json(); data.motors.forEach(m=>{document.getElementById('run'+m.motor).textContent=m.running?'RUNNING':'IDLE';document.getElementById('json'+m.motor).textContent=JSON.stringify(m,null,2);});}
async function send(i){const b={motor:i,position_rad:+document.getElementById('pos'+i).value,velocity_rad_s:+document.getElementById('vel'+i).value};await fetch('/api/motor/set',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(b)});load();}
load(); setInterval(load,500);
</script></body></html>)HTML";
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    return httpd_resp_send(req, page.c_str(), HTTPD_RESP_USE_STRLEN);
}

esp_err_t HttpServer::motor_status_handler(httpd_req_t* req) {
    add_cors_headers(req);
    cJSON* root = cJSON_CreateObject();
    cJSON* motors = cJSON_CreateArray();
    for (int i = 0; i < NUM_MOTORS; i++) {
        cJSON* m = cJSON_CreateObject();
        cJSON_AddNumberToObject(m, "motor", i);
        cJSON_AddNumberToObject(m, "position_rad", motor_control_get_position(i));
        cJSON_AddNumberToObject(m, "velocity_rad_s", motor_control_get_velocity(i));
        cJSON_AddBoolToObject(m, "running", motor_control_is_motor_running(i));
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
    char buf[256];
    int len = httpd_req_recv(req, buf, std::min((int)sizeof(buf) - 1, (int)req->content_len));
    if (len <= 0) return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "invalid body");
    buf[len] = '\0';
    cJSON* body = cJSON_Parse(buf);
    if (!body) return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad json");
    cJSON* motor = cJSON_GetObjectItem(body, "motor");
    cJSON* pos = cJSON_GetObjectItem(body, "position_rad");
    cJSON* vel = cJSON_GetObjectItem(body, "velocity_rad_s");
    if (cJSON_IsNumber(motor)) {
        uint8_t idx = (uint8_t)motor->valueint;
        if (cJSON_IsNumber(pos)) motor_control_set_position(idx, pos->valuedouble);
        if (cJSON_IsNumber(vel)) motor_control_set_velocity(idx, vel->valuedouble);
    }
    cJSON_Delete(body);
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, "{\"ok\":true}");
}

// Handler implementations

esp_err_t HttpServer::root_handler(httpd_req_t* req) {
    std::string html_response = std::string(R"(
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
                pre { white-space: pre-wrap; background:#020617; color:#a7f3d0; padding:10px; border-radius:8px; min-height:160px; }
            </style>
        </head>
        <body>
            <div class="container">
                <h1>🤖 Shelfbot Dashboard</h1>
                <p class="subtitle">Jump to sensors, motor controls, and diagnostics.</p>
                <p class="subtitle">Firmware: )") + FirmwareVersion::get_version_string() + R"(</p>

                <div class="grid">
                    <div class="card"><h3>📦 Sensor Dashboard</h3><p><a href="/api/sensors">Open all sensors JSON</a></p><code>GET /api/sensors</code></div>
                    <div class="card"><h3>📡 ToF</h3><p><a href="/api/tof">Open ToF JSON</a></p><code>GET /api/tof</code></div>
                    <div class="card"><h3>🛰️ LiDAR</h3><p><a href="/api/lidar">Open LiDAR JSON</a></p><code>GET /api/lidar</code></div>
                    <div class="card"><h3>📏 Ultrasonic</h3><p><a href="/api/ultrasonic">Open ultrasonic JSON</a></p><code>GET /api/ultrasonic</code></div>
                    <div class="card"><h3>⚙️ Motor Control UI</h3><p><a href="/motor.html">Open motor control page</a></p><code>/motor.html</code></div>
                    <div class="card"><h3>❤️ Health</h3><p><a href="/api/health">Open system health JSON</a></p><code>GET /api/health</code></div>
                </div>

                <h3 style="margin-top:24px;">Live Sensor Stream</h3>
                <code>Auto-refresh: 1s from /api/sensors</code>
                <pre id="sensor-data">Loading...</pre>
            </div>

            <script>
                async function refreshSensors() {
                    const target = document.getElementById('sensor-data');
                    try {
                        const response = await fetch('/api/sensors');
                        const data = await response.json();
                        target.textContent = JSON.stringify(data, null, 2);
                    } catch (err) {
                        target.textContent = 'Failed to fetch /api/sensors: ' + err;
                    }
                }

                refreshSensors();
                setInterval(refreshSensors, 1000);
            </script>
        </body>
        </html>
    )";

    httpd_resp_set_type(req, "text/html; charset=utf-8");
    return httpd_resp_send(req, html_response.c_str(), HTTPD_RESP_USE_STRLEN);
}

std::string HttpServer::get_sensor_status_text(const SensorCommon::TofMeasurement& measurement) {
    if (!measurement.valid) {
        return "invalid";
    }

    switch (measurement.status) {
        case 0: return "ok";
        case 1: return "sigma_fail";
        case 2: return "signal_fail";
        case 3: return "min_range_fail";
        case 4: return "phase_fail";
        case 5: return "hw_fail";
        case 6: return "range_valid_min_range_clipped";
        case 7: return "sync_int_fail";
        case 8: return "no_update";
        case 9: return "wrapped_target_fail";
        case 10: return "processing_fail";
        case 11: return "x_talk_fail";
        case 12: return "range_ignore_threshold";
        default: return "unknown";
    }
}
std::string HttpServer::get_sensor_status_text(const SensorCommon::LidarMeasurement& measurement) {
    if (!measurement.valid) {
        return "invalid";
    }

    switch (measurement.status) {
        case 0: return "ok";
        case 1: return "sigma_fail";
        case 2: return "signal_fail";
        case 3: return "min_range_fail";
        case 4: return "phase_fail";
        case 5: return "hw_fail";
        case 6: return "range_valid_min_range_clipped";
        case 7: return "sync_int_fail";
        case 8: return "no_update";
        case 9: return "wrapped_target_fail";
        case 10: return "processing_fail";
        case 11: return "x_talk_fail";
        case 12: return "range_ignore_threshold";
        default: return "unknown";
    }
}


cJSON* HttpServer::create_tof_json(const SensorCommon::SensorDataPacket& sensor_data) {
    cJSON* tof_array = cJSON_CreateArray();

    for (int i = 0; i < SensorCommon::NUM_TOF_SENSORS; i++) {
        const auto& measurement = sensor_data.tof_measurements[i];
        cJSON* tof_obj = cJSON_CreateObject();

        cJSON_AddNumberToObject(tof_obj, "sensor_id", i);
        cJSON_AddNumberToObject(tof_obj, "distance_mm", measurement.distance_mm);
        cJSON_AddNumberToObject(tof_obj, "distance_cm", measurement.distance_cm());
        cJSON_AddBoolToObject(tof_obj, "valid", measurement.valid);
        cJSON_AddNumberToObject(tof_obj, "status", measurement.status);
        cJSON_AddStringToObject(tof_obj, "status_text", get_sensor_status_text(measurement).c_str());
        cJSON_AddNumberToObject(tof_obj, "timestamp_us", measurement.timestamp_us);
        cJSON_AddBoolToObject(tof_obj, "timeout_occurred", measurement.timeout_occurred);

        cJSON_AddItemToArray(tof_array, tof_obj);
    }

    return tof_array;
}

cJSON* HttpServer::create_ultrasonic_json(const SensorCommon::SensorDataPacket& sensor_data) {
    cJSON* ultrasonic_array = cJSON_CreateArray();

    for (int i = 0; i < SensorCommon::NUM_ULTRASONIC_SENSORS; i++) {
        const auto& reading = sensor_data.ultrasonic_readings[i];
        cJSON* sensor_obj = cJSON_CreateObject();

        cJSON_AddNumberToObject(sensor_obj, "sensor_id", i);
        cJSON_AddNumberToObject(sensor_obj, "distance_cm", reading.distance_cm);
        cJSON_AddBoolToObject(sensor_obj, "valid", reading.valid);
        cJSON_AddNumberToObject(sensor_obj, "status", reading.status);
        cJSON_AddNumberToObject(sensor_obj, "timestamp_us", reading.timestamp_us);

        cJSON_AddItemToArray(ultrasonic_array, sensor_obj);
    }

    return ultrasonic_array;
}

cJSON* HttpServer::create_sensor_json(const SensorCommon::SensorDataPacket& sensor_data) {
    cJSON* root = cJSON_CreateObject();

    // Add timestamp
    cJSON_AddNumberToObject(root, "timestamp_us", sensor_data.timestamp_us);

    // Add ultrasonic sensors
    cJSON* ultrasonic_json = create_ultrasonic_json(sensor_data);
    cJSON_AddItemToObject(root, "ultrasonic", ultrasonic_json);

    // Add ToF sensors
    cJSON* tof_json = create_tof_json(sensor_data);
    cJSON_AddItemToObject(root, "tof", tof_json);

    const auto& lidar = sensor_data.lidar_measurement;
    cJSON* lidar_json = cJSON_CreateObject();
    cJSON_AddStringToObject(lidar_json, "source", "lidar_measurement");
    cJSON_AddBoolToObject(lidar_json, "valid", lidar.valid);
    cJSON_AddNumberToObject(lidar_json, "distance_mm", lidar.distance_mm);
    cJSON_AddNumberToObject(lidar_json, "distance_cm", lidar.distance_cm());
    cJSON_AddNumberToObject(lidar_json, "status", lidar.status);
    cJSON_AddStringToObject(lidar_json, "status_text", get_sensor_status_text(lidar).c_str());
    cJSON_AddNumberToObject(lidar_json, "timestamp_us", lidar.timestamp_us);
    cJSON_AddBoolToObject(lidar_json, "timeout_occurred", lidar.timeout_occurred);
    cJSON_AddItemToObject(root, "lidar", lidar_json);

    return root;
}

esp_err_t HttpServer::tof_handler(httpd_req_t* req) {
    ESP_LOGI(TAG, "ToF handler called");

    add_cors_headers(req);

    SensorCommon::SensorDataPacket sensor_data;
    // FIXED: Changed from &sensor_data to sensor_data (pass by reference, not pointer)
    if (!SensorManager::get_instance().get_latest_data(sensor_data)) {
        ESP_LOGW(TAG, "No sensor data available");
        cJSON* root = cJSON_CreateObject();
        cJSON_AddStringToObject(root, "error", "No sensor data available");
        cJSON_AddBoolToObject(root, "available", false);

        char* json_str = cJSON_PrintUnformatted(root);
        if (json_str) {
            httpd_resp_set_type(req, "application/json");
            httpd_resp_sendstr(req, json_str);
            free(json_str);
        } else {
            httpd_resp_send_500(req);
        }
        cJSON_Delete(root);
        return ESP_OK;
    }

    cJSON* root = create_tof_json(sensor_data);

    char* json_str = cJSON_PrintUnformatted(root);
    if (json_str) {
        httpd_resp_set_type(req, "application/json");
        esp_err_t ret = httpd_resp_sendstr(req, json_str);
        free(json_str);
        cJSON_Delete(root);
        return ret;
    } else {
        cJSON_Delete(root);
        return httpd_resp_send_500(req);
    }
}

esp_err_t HttpServer::lidar_handler(httpd_req_t* req) {
    SensorCommon::SensorDataPacket sensor_data;
    if (!SensorManager::get_instance().get_latest_data(sensor_data)) {
        cJSON* root = cJSON_CreateObject();
        cJSON_AddStringToObject(root, "error", "No sensor data available");
        cJSON_AddBoolToObject(root, "available", false);
        char* json_str = cJSON_PrintUnformatted(root);
        if (json_str) {
            httpd_resp_set_type(req, "application/json");
            httpd_resp_sendstr(req, json_str);
            free(json_str);
        }
        cJSON_Delete(root);
        return ESP_OK;
    }

    const auto& m = sensor_data.lidar_measurement;
    cJSON* root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "source", "lidar_measurement");
    cJSON_AddBoolToObject(root, "valid", m.valid);
    cJSON_AddNumberToObject(root, "distance_mm", m.distance_mm);
    cJSON_AddNumberToObject(root, "distance_cm", m.distance_cm());
    cJSON_AddNumberToObject(root, "status", m.status);
    cJSON_AddStringToObject(root, "status_text", get_sensor_status_text(m).c_str());
    cJSON_AddNumberToObject(root, "timestamp_us", m.timestamp_us);
    cJSON_AddBoolToObject(root, "timeout_occurred", m.timeout_occurred);

    char* json_str = cJSON_PrintUnformatted(root);
    if (json_str) {
        httpd_resp_set_type(req, "application/json");
        esp_err_t ret = httpd_resp_sendstr(req, json_str);
        free(json_str);
        cJSON_Delete(root);
        return ret;
    }
    cJSON_Delete(root);
    return ESP_FAIL;
}

esp_err_t HttpServer::ultrasonic_handler(httpd_req_t* req) {
    ESP_LOGI(TAG, "Ultrasonic handler called");

    add_cors_headers(req);

    SensorCommon::SensorDataPacket sensor_data;
    // FIXED: Changed from &sensor_data to sensor_data (pass by reference, not pointer)
    if (!SensorManager::get_instance().get_latest_data(sensor_data)) {
        ESP_LOGW(TAG, "No sensor data available");
        cJSON* root = cJSON_CreateObject();
        cJSON_AddStringToObject(root, "error", "No sensor data available");
        cJSON_AddBoolToObject(root, "available", false);

        char* json_str = cJSON_PrintUnformatted(root);
        if (json_str) {
            httpd_resp_set_type(req, "application/json");
            httpd_resp_sendstr(req, json_str);
            free(json_str);
        } else {
            httpd_resp_send_500(req);
        }
        cJSON_Delete(root);
        return ESP_OK;
    }

    cJSON* root = create_ultrasonic_json(sensor_data);

    char* json_str = cJSON_PrintUnformatted(root);
    if (json_str) {
        httpd_resp_set_type(req, "application/json");
        esp_err_t ret = httpd_resp_sendstr(req, json_str);
        free(json_str);
        cJSON_Delete(root);
        return ret;
    } else {
        cJSON_Delete(root);
        return httpd_resp_send_500(req);
    }
}

esp_err_t HttpServer::sensors_handler(httpd_req_t* req) {
    ESP_LOGI(TAG, "Sensors handler called");

    add_cors_headers(req);

    SensorCommon::SensorDataPacket sensor_data;
    // FIXED: Changed from &sensor_data to sensor_data (pass by reference, not pointer)
    if (!SensorManager::get_instance().get_latest_data(sensor_data)) {
        ESP_LOGW(TAG, "No sensor data available");
        cJSON* root = cJSON_CreateObject();
        cJSON_AddStringToObject(root, "error", "No sensor data available");
        cJSON_AddBoolToObject(root, "available", false);

        char* json_str = cJSON_PrintUnformatted(root);
        if (json_str) {
            httpd_resp_set_type(req, "application/json");
            httpd_resp_sendstr(req, json_str);
            free(json_str);
        } else {
            httpd_resp_send_500(req);
        }
        cJSON_Delete(root);
        return ESP_OK;
    }

    cJSON* root = create_sensor_json(sensor_data);

    char* json_str = cJSON_PrintUnformatted(root);
    if (json_str) {
        httpd_resp_set_type(req, "application/json");
        esp_err_t ret = httpd_resp_sendstr(req, json_str);
        free(json_str);
        cJSON_Delete(root);
        return ret;
    } else {
        cJSON_Delete(root);
        return httpd_resp_send_500(req);
    }
}

esp_err_t HttpServer::health_handler(httpd_req_t* req) {
    ESP_LOGI(TAG, "Health handler called");

    add_cors_headers(req);

    cJSON* root = cJSON_CreateObject();

    // System info
    cJSON_AddStringToObject(root, "device", "Shelfbot ESP32");
    cJSON_AddNumberToObject(root, "free_heap", esp_get_free_heap_size());
    cJSON_AddNumberToObject(root, "min_free_heap", esp_get_minimum_free_heap_size());
    cJSON_AddNumberToObject(root, "uptime_ms", esp_timer_get_time() / 1000);

    // Sensor status
    SensorCommon::SensorDataPacket sensor_data;
    // FIXED: Changed from &sensor_data to sensor_data (pass by reference, not pointer)
    bool has_sensor_data = SensorManager::get_instance().get_latest_data(sensor_data);
    cJSON_AddBoolToObject(root, "sensors_available", has_sensor_data);

    if (has_sensor_data) {
        cJSON* sensors = cJSON_CreateObject();

        // Ultrasonic health
        cJSON* ultrasonic = cJSON_CreateArray();
        for (int i = 0; i < SensorCommon::NUM_ULTRASONIC_SENSORS; i++) {
            cJSON* sensor = cJSON_CreateObject();
            cJSON_AddNumberToObject(sensor, "id", i);
            bool valid = sensor_data.ultrasonic_readings[i].valid;
            cJSON_AddBoolToObject(sensor, "healthy", valid);
            cJSON_AddNumberToObject(sensor, "distance_cm", sensor_data.ultrasonic_readings[i].distance_cm);
            cJSON_AddItemToArray(ultrasonic, sensor);
        }
        cJSON_AddItemToObject(sensors, "ultrasonic", ultrasonic);

        // ToF health
        cJSON* tof = cJSON_CreateArray();
        for (int i = 0; i < SensorCommon::NUM_TOF_SENSORS; i++) {
            cJSON* sensor = cJSON_CreateObject();
            cJSON_AddNumberToObject(sensor, "id", i);
            bool valid = sensor_data.tof_measurements[i].valid;
            cJSON_AddBoolToObject(sensor, "healthy", valid);
            cJSON_AddStringToObject(sensor, "status", get_sensor_status_text(sensor_data.tof_measurements[i]).c_str());
            cJSON_AddNumberToObject(sensor, "distance_mm", sensor_data.tof_measurements[i].distance_mm);
            cJSON_AddItemToArray(tof, sensor);
        }
        cJSON_AddItemToObject(sensors, "tof", tof);

        cJSON_AddItemToObject(root, "sensors", sensors);
    }

    char* json_str = cJSON_PrintUnformatted(root);
    if (json_str) {
        httpd_resp_set_type(req, "application/json");
        esp_err_t ret = httpd_resp_sendstr(req, json_str);
        free(json_str);
        cJSON_Delete(root);
        return ret;
    } else {
        cJSON_Delete(root);
        return httpd_resp_send_500(req);
    }
}
