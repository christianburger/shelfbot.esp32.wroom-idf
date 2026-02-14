#include "http_server.hpp"
#include <esp_timer.h>
#include <esp_task_wdt.h>
#include <sstream>
#include <iomanip>

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

    // CORS OPTIONS handlers
    const char* cors_endpoints[] = {
        "/api/tof",
        "/api/ultrasonic",
        "/api/sensors",
        "/api/health",
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

// Handler implementations

esp_err_t HttpServer::root_handler(httpd_req_t* req) {
    const char* html_response = R"(
        <!DOCTYPE html>
        <html>
        <head>
            <title>Shelfbot ESP32</title>
            <meta charset="UTF-8">
            <meta name="viewport" content="width=device-width, initial-scale=1">
            <style>
                body { font-family: Arial, sans-serif; margin: 40px; }
                h1 { color: #333; }
                .endpoint { background: #f5f5f5; padding: 15px; margin: 10px 0; border-radius: 5px; }
                code { background: #e0e0e0; padding: 2px 5px; border-radius: 3px; }
                a { color: #0066cc; text-decoration: none; }
                a:hover { text-decoration: underline; }
            </style>
        </head>
        <body>
            <h1>📡 Shelfbot ESP32 HTTP Server</h1>
            <p>Welcome to the Shelfbot sensor monitoring interface.</p>

            <h2>📊 Sensor Endpoints</h2>
            <div class="endpoint">
                <h3><a href="/api/sensors">All Sensors</a></h3>
                <code>GET /api/sensors</code>
                <p>Get data from all sensors (ultrasonic + ToF)</p>
            </div>

            <div class="endpoint">
                <h3><a href="/api/tof">ToF Sensor</a></h3>
                <code>GET /api/tof</code>
                <p>Get data from Time-of-Flight sensor</p>
            </div>

            <div class="endpoint">
                <h3><a href="/api/ultrasonic">Ultrasonic Sensors</a></h3>
                <code>GET /api/ultrasonic</code>
                <p>Get data from ultrasonic distance sensors</p>
            </div>

            <h2>⚙️ System Endpoints</h2>
            <div class="endpoint">
                <h3><a href="/api/health">System Health</a></h3>
                <code>GET /api/health</code>
                <p>Check system health and sensor status</p>
            </div>

            <h2>🔴 Live Sensor Data</h2>
            <div class="endpoint">
                <code>Auto-refresh: 1s from /api/sensors</code>
                <pre id="sensor-data" style="white-space: pre-wrap; background:#111; color:#0f0; padding:10px; border-radius:4px; min-height:140px;">Loading...</pre>
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

            <hr>
            <p style="color: #666; font-size: 0.9em;">
                Shelfbot ESP32 Firmware | Built with ESP-IDF
            </p>
        </body>
        </html>
    )";

    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, html_response, strlen(html_response));
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