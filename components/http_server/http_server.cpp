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

    // I2C scan endpoint
    httpd_uri_t i2c_scan_uri = {
        .uri = "/api/i2c-scan",
        .method = HTTP_GET,
        .handler = i2c_scan_handler,
        .user_ctx = nullptr
    };
    httpd_register_uri_handler(server_, &i2c_scan_uri);

    // ToF switch mode endpoint
    httpd_uri_t tof_switch_uri = {
        .uri = "/api/tof/switch",
        .method = HTTP_POST,
        .handler = tof_switch_handler,
        .user_ctx = nullptr
    };
    httpd_register_uri_handler(server_, &tof_switch_uri);

    // ToF mode endpoint
    httpd_uri_t tof_mode_uri = {
        .uri = "/api/tof/mode",
        .method = HTTP_POST,
        .handler = tof_mode_handler,
        .user_ctx = nullptr
    };
    httpd_register_uri_handler(server_, &tof_mode_uri);

    // CORS OPTIONS handlers
    const char* cors_endpoints[] = {
        "/api/tof",
        "/api/ultrasonic",
        "/api/sensors",
        "/api/health",
        "/api/i2c-scan",
        "/api/tof/switch",
        "/api/tof/mode"
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

            <div class="endpoint">
                <h3><a href="/api/i2c-scan">I2C Scan</a></h3>
                <code>GET /api/i2c-scan</code>
                <p>Scan I2C bus for connected devices</p>
            </div>

            <h2>🔧 Control Endpoints</h2>
            <div class="endpoint">
                <h3>Switch ToF to I2C Mode</h3>
                <code>POST /api/tof/switch</code>
                <p>Switch TOF400F module from UART to I2C mode</p>
            </div>

            <div class="endpoint">
                <h3>Change ToF Mode</h3>
                <code>POST /api/tof/mode</code>
                <p>Change ToF sensor mode (long_distance/high_precision)</p>
                <p>Body: <code>{"mode": "long_distance"}</code></p>
            </div>

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

esp_err_t HttpServer::i2c_scan_handler(httpd_req_t* req) {
    ESP_LOGI(TAG, "I2C scan handler called");

    add_cors_headers(req);

    cJSON* root = cJSON_CreateObject();

    // Perform I2C scan
    std::vector<uint8_t> addresses;
    bool scan_result = I2CScanner::scan(I2C_NUM_0, addresses, GPIO_NUM_21, GPIO_NUM_22, 100000);

    cJSON_AddBoolToObject(root, "success", scan_result);
    cJSON_AddNumberToObject(root, "port", 0);
    cJSON_AddStringToObject(root, "sda_pin", "GPIO21");
    cJSON_AddStringToObject(root, "scl_pin", "GPIO22");
    cJSON_AddNumberToObject(root, "frequency_hz", 100000);

    if (scan_result && !addresses.empty()) {
        cJSON* devices = cJSON_CreateArray();

        for (uint8_t addr : addresses) {
            cJSON* device = cJSON_CreateObject();
            std::stringstream addr_ss;
            addr_ss << "0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(addr);

            cJSON_AddStringToObject(device, "address", addr_ss.str().c_str());
            cJSON_AddStringToObject(device, "name", I2CScanner::getDeviceName(addr));

            cJSON_AddItemToArray(devices, device);
        }

        cJSON_AddItemToObject(root, "devices", devices);
        cJSON_AddNumberToObject(root, "device_count", addresses.size());
    } else {
        cJSON_AddStringToObject(root, "message", scan_result ? "No I2C devices found" : "I2C scan failed");
        cJSON_AddNumberToObject(root, "device_count", 0);
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

esp_err_t HttpServer::tof_switch_handler(httpd_req_t* req) {
    ESP_LOGI(TAG, "ToF switch handler called");

    add_cors_headers(req);

    // Read JSON body if present
    char content[256];
    int received = httpd_req_recv(req, content, sizeof(content) - 1);

    int uart_port = 1;
    int tx_pin = 17;
    int rx_pin = 16;
    uint32_t baud_rate = 115200;
    uint8_t slave_addr = 0x01;

    if (received > 0) {
        content[received] = '\0';
        cJSON* json = cJSON_Parse(content);
        if (json) {
            cJSON* port = cJSON_GetObjectItem(json, "uart_port");
            cJSON* tx = cJSON_GetObjectItem(json, "tx_pin");
            cJSON* rx = cJSON_GetObjectItem(json, "rx_pin");
            cJSON* baud = cJSON_GetObjectItem(json, "baud_rate");
            cJSON* addr = cJSON_GetObjectItem(json, "slave_address");

            if (port) uart_port = port->valueint;
            if (tx) tx_pin = tx->valueint;
            if (rx) rx_pin = rx->valueint;
            if (baud) baud_rate = baud->valueint;
            if (addr) slave_addr = addr->valueint;

            cJSON_Delete(json);
        }
    }

    cJSON* root = cJSON_CreateObject();

    ESP_LOGI(TAG, "Attempting to switch TOF400F to I2C mode");
    bool success = I2CScanner::switchTOF400FToI2CMode(
        static_cast<uart_port_t>(uart_port),
        static_cast<gpio_num_t>(tx_pin),
        static_cast<gpio_num_t>(rx_pin),
        baud_rate,
        slave_addr
    );

    cJSON_AddBoolToObject(root, "success", success);
    cJSON_AddNumberToObject(root, "uart_port", uart_port);
    cJSON_AddNumberToObject(root, "tx_pin", tx_pin);
    cJSON_AddNumberToObject(root, "rx_pin", rx_pin);
    cJSON_AddNumberToObject(root, "baud_rate", baud_rate);
    cJSON_AddNumberToObject(root, "slave_address", slave_addr);

    if (success) {
        cJSON_AddStringToObject(root, "message", "TOF400F switched to I2C mode successfully");
    } else {
        cJSON_AddStringToObject(root, "message", "Failed to switch TOF400F to I2C mode");
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

esp_err_t HttpServer::tof_mode_handler(httpd_req_t* req) {
    ESP_LOGI(TAG, "ToF mode handler called");

    add_cors_headers(req);

    // Read JSON body
    char content[256];
    int received = httpd_req_recv(req, content, sizeof(content) - 1);

    if (received <= 0) {
        cJSON* root = cJSON_CreateObject();
        cJSON_AddBoolToObject(root, "success", false);
        cJSON_AddStringToObject(root, "error", "No JSON body received");

        char* json_str = cJSON_PrintUnformatted(root);
        if (json_str) {
            httpd_resp_set_type(req, "application/json");
            httpd_resp_sendstr(req, json_str);
            free(json_str);
        }
        cJSON_Delete(root);
        return ESP_OK;
    }

    content[received] = '\0';
    cJSON* json = cJSON_Parse(content);
    if (!json) {
        cJSON* root = cJSON_CreateObject();
        cJSON_AddBoolToObject(root, "success", false);
        cJSON_AddStringToObject(root, "error", "Invalid JSON");

        char* json_str = cJSON_PrintUnformatted(root);
        if (json_str) {
            httpd_resp_set_type(req, "application/json");
            httpd_resp_sendstr(req, json_str);
            free(json_str);
        }
        cJSON_Delete(root);
        return ESP_OK;
    }

    cJSON* mode_json = cJSON_GetObjectItem(json, "mode");
    cJSON* sensor_id_json = cJSON_GetObjectItem(json, "sensor_id");

    if (!mode_json) {
        cJSON_Delete(json);

        cJSON* root = cJSON_CreateObject();
        cJSON_AddBoolToObject(root, "success", false);
        cJSON_AddStringToObject(root, "error", "Missing 'mode' field");

        char* json_str = cJSON_PrintUnformatted(root);
        if (json_str) {
            httpd_resp_set_type(req, "application/json");
            httpd_resp_sendstr(req, json_str);
            free(json_str);
        }
        cJSON_Delete(root);
        return ESP_OK;
    }

    std::string mode_str = mode_json->valuestring;
    int sensor_id = sensor_id_json ? sensor_id_json->valueint : 0;

    // Get sensor control from manager
    SensorControl* sensor_control = SensorManager::get_instance().get_sensor_control();
    if (!sensor_control) {
        cJSON_Delete(json);

        cJSON* root = cJSON_CreateObject();
        cJSON_AddBoolToObject(root, "success", false);
        cJSON_AddStringToObject(root, "error", "Sensor control not available");

        char* json_str = cJSON_PrintUnformatted(root);
        if (json_str) {
            httpd_resp_set_type(req, "application/json");
            httpd_resp_sendstr(req, json_str);
            free(json_str);
        }
        cJSON_Delete(root);
        return ESP_OK;
    }

    bool long_distance;
    if (mode_str == "long_distance" || mode_str == "long") {
        long_distance = true;
    } else if (mode_str == "high_precision" || mode_str == "high" || mode_str == "precision") {
        long_distance = false;
    } else {
        cJSON_Delete(json);

        cJSON* root = cJSON_CreateObject();
        cJSON_AddBoolToObject(root, "success", false);
        cJSON_AddStringToObject(root, "error", "Invalid mode. Use 'long_distance' or 'high_precision'");

        char* json_str = cJSON_PrintUnformatted(root);
        if (json_str) {
            httpd_resp_set_type(req, "application/json");
            httpd_resp_sendstr(req, json_str);
            free(json_str);
        }
        cJSON_Delete(root);
        return ESP_OK;
    }

    // Change the mode
    esp_err_t err = sensor_control->set_tof_mode(sensor_id, long_distance);
    cJSON_Delete(json);

    cJSON* root = cJSON_CreateObject();
    cJSON_AddBoolToObject(root, "success", err == ESP_OK);
    cJSON_AddNumberToObject(root, "sensor_id", sensor_id);
    cJSON_AddStringToObject(root, "mode", mode_str.c_str());
    cJSON_AddStringToObject(root, "description",
                           long_distance ? "Long distance mode (~4m range)" :
                                           "High precision mode (~1.3m range)");

    if (err != ESP_OK) {
        cJSON_AddStringToObject(root, "error", esp_err_to_name(err));
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
