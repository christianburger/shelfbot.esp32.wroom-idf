#include "http_server.h"
#include "motor_control.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "cJSON.h"

static const char *TAG = "http_server";

static httpd_handle_t server = NULL;

// Embedded Javascript for the root page
const char* javascriptContent = R"(
function sendCommand(motor, position) {
    const body = { motor: motor, position: parseFloat(position) };
    fetch('/motor', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(body)
    }).then(r => r.json()).then(d => console.log(d));
}

function moveAllMotors() {
    const pos_inputs = Array.from(document.querySelectorAll('.pos-input'));
    const positions = pos_inputs.map(i => parseFloat(i.value));
    const speed = parseInt(document.getElementById('allSpeed').value);
    const nonBlocking = document.getElementById('nonBlocking').checked;

    const body = { positions: positions, speed: speed, nonBlocking: nonBlocking };

    fetch('/motors', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(body)
    }).then(r => r.json()).then(d => console.log(d));
}
)";

// Handler for the root URL (/)
static esp_err_t root_handler(httpd_req_t *req) {
    char html[2048];
    char motor_controls[1024] = "";

    for (int i = 0; i < NUM_MOTORS; i++) {
        char motor_control_html[200];
        snprintf(motor_control_html, sizeof(motor_control_html),
                 "<div style='margin: 10px'>Motor %d: <input type='number' class='pos-input' id='position%d' value='0' placeholder='Position'></div>",
                 i, i);
        strcat(motor_controls, motor_control_html);
    }

    snprintf(html, sizeof(html), R"(
        <html><head>
        <title>Shelfbot Control</title>
        <script>%s</script>
        <style>
            body { font-family: sans-serif; }
            .endpoint { margin: 20px; padding: 15px; border: 1px solid #ccc; }
        </style>
        </head><body>
        <h1>Shelfbot Control</h1>
        <div class='endpoint'>
            <h2>Motor Control</h2>
            %s
            <div>
                Speed: <input type='number' id='allSpeed' value='4000'>
                Non-blocking: <input type='checkbox' id='nonBlocking'>
            </div>
            <button onclick='moveAllMotors()'>Move All Motors</button>
        </div>
        </body></html>
    )", javascriptContent, motor_controls);

    httpd_resp_send(req, html, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

// Handler for /status
static esp_err_t status_handler(httpd_req_t *req) {
    cJSON *root = cJSON_CreateObject();
    cJSON *motors = cJSON_CreateArray();
    cJSON_AddItemToObject(root, "motors", motors);

    for (int i = 0; i < NUM_MOTORS; i++) {
        cJSON *motor = cJSON_CreateObject();
        cJSON_AddNumberToObject(motor, "index", i);
        cJSON_AddNumberToObject(motor, "position", motor_control_get_motor_position_double(i));
        cJSON_AddNumberToObject(motor, "velocity", motor_control_get_motor_velocity_double(i));
        cJSON_AddBoolToObject(motor, "running", motor_control_is_motor_running(i));
        cJSON_AddItemToArray(motors, motor);
    }

    const char *json_string = cJSON_Print(root);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json_string, HTTPD_RESP_USE_STRLEN);
    cJSON_Delete(root);
    free((void*)json_string);
    return ESP_OK;
}

// Handler for /motor_positions
static esp_err_t motor_positions_handler(httpd_req_t *req) {
    cJSON *root = cJSON_CreateObject();
    cJSON *positions = cJSON_CreateArray();
    cJSON_AddItemToObject(root, "positions", positions);

    for (int i = 0; i < NUM_MOTORS; i++) {
        cJSON_AddItemToArray(positions, cJSON_CreateNumber(motor_control_get_motor_position_double(i)));
    }

    const char *json_string = cJSON_Print(root);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json_string, HTTPD_RESP_USE_STRLEN);
    cJSON_Delete(root);
    free((void*)json_string);
    return ESP_OK;
}

// Handler for /motor_velocities
static esp_err_t motor_velocities_handler(httpd_req_t *req) {
    cJSON *root = cJSON_CreateObject();
    cJSON *velocities = cJSON_CreateArray();
    cJSON_AddItemToObject(root, "velocities", velocities);

    for (int i = 0; i < NUM_MOTORS; i++) {
        cJSON_AddItemToArray(velocities, cJSON_CreateNumber(motor_control_get_motor_velocity_double(i)));
    }

    const char *json_string = cJSON_Print(root);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json_string, HTTPD_RESP_USE_STRLEN);
    cJSON_Delete(root);
    free((void*)json_string);
    return ESP_OK;
}

// Handler for POST /motor
static esp_err_t motor_move_handler(httpd_req_t *req) {
    char buf[128];
    int ret, remaining = req->content_len;
    
    if (remaining > sizeof(buf) - 1) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Request too long");
        return ESP_FAIL;
    }
    ret = httpd_req_recv(req, buf, remaining);
    if (ret <= 0) { return ESP_FAIL; }
    buf[ret] = '\0';

    cJSON *root = cJSON_Parse(buf);
    if (!root) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }

    uint8_t motor_index = cJSON_GetObjectItem(root, "motor")->valueint;
    double position = cJSON_GetObjectItem(root, "position")->valuedouble;

    motor_control_set_motor_position_double(motor_index, position);

    cJSON_Delete(root);
    httpd_resp_send(req, "OK", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

// Handler for POST /motors
static esp_err_t all_motors_move_handler(httpd_req_t *req) {
    char buf[512];
    int ret, remaining = req->content_len;

    if (remaining > sizeof(buf) - 1) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Request too long");
        return ESP_FAIL;
    }
    ret = httpd_req_recv(req, buf, remaining);
    if (ret <= 0) { return ESP_FAIL; }
    buf[ret] = '\0';

    cJSON *root = cJSON_Parse(buf);
    if (!root) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }

    cJSON *positions_json = cJSON_GetObjectItem(root, "positions");
    long speed = cJSON_GetObjectItem(root, "speed")->valueint;
    bool nonBlocking = cJSON_GetObjectItem(root, "nonBlocking")->valueint;

    double positions[NUM_MOTORS];
    int num_positions = cJSON_GetArraySize(positions_json);
    if (num_positions > NUM_MOTORS) {
        num_positions = NUM_MOTORS; // Prevent buffer overflow
    }

    for (int i = 0; i < num_positions; i++) {
        positions[i] = cJSON_GetArrayItem(positions_json, i)->valuedouble;
    }

    motor_control_move_all_motors_vector(positions, num_positions, speed, nonBlocking);

    cJSON_Delete(root);
    httpd_resp_send(req, "OK", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}


esp_err_t start_webserver(void) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.lru_purge_enable = true;

    ESP_LOGI(TAG, "Starting server on port: '%d'", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK) {
        // Registering URI handlers
        httpd_uri_t root_uri = { .uri = "/", .method = HTTP_GET, .handler = root_handler, .user_ctx = NULL };
        httpd_register_uri_handler(server, &root_uri);

        httpd_uri_t status_uri = { .uri = "/status", .method = HTTP_GET, .handler = status_handler, .user_ctx = NULL };
        httpd_register_uri_handler(server, &status_uri);

        httpd_uri_t motor_pos_uri = { .uri = "/motor_positions", .method = HTTP_GET, .handler = motor_positions_handler, .user_ctx = NULL };
        httpd_register_uri_handler(server, &motor_pos_uri);

        httpd_uri_t motor_vel_uri = { .uri = "/motor_velocities", .method = HTTP_GET, .handler = motor_velocities_handler, .user_ctx = NULL };
        httpd_register_uri_handler(server, &motor_vel_uri);

        httpd_uri_t motor_move_uri = { .uri = "/motor", .method = HTTP_POST, .handler = motor_move_handler, .user_ctx = NULL };
        httpd_register_uri_handler(server, &motor_move_uri);

        httpd_uri_t all_motors_move_uri = { .uri = "/motors", .method = HTTP_POST, .handler = all_motors_move_handler, .user_ctx = NULL };
        httpd_register_uri_handler(server, &all_motors_move_uri);

        return ESP_OK;
    }
    ESP_LOGI(TAG, "Error starting server!");
    return ESP_FAIL;
}

esp_err_t stop_webserver(void) {
    if (server) {
        httpd_stop(server);
        server = NULL;
        return ESP_OK;
    }
    return ESP_FAIL;
}
