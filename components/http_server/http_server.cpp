#include "esp_http_server.h"
#include "esp_log.h"
#include "cJSON.h"
#include "motor_control.h"
#include "http_server.h"

static const char *TAG = "http_server";

// Symbols for the embedded HTML file
extern const uint8_t index_html_start[] asm("_binary_index_html_start");
extern const uint8_t index_html_end[]   asm("_binary_index_html_end");

// Handler to serve the main index.html page
static esp_err_t root_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, (const char *)index_html_start, index_html_end - index_html_start);
    return ESP_OK;
}

// Handler for the /move API endpoint
static esp_err_t move_post_handler(httpd_req_t *req)
{
    char buf[128];
    int ret, remaining = req->content_len;

    if (remaining > sizeof(buf) - 1) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Request body too large");
        return ESP_FAIL;
    }

    ret = httpd_req_recv(req, buf, remaining);
    if (ret <= 0) {
        if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
            httpd_resp_send_408(req);
        }
        return ESP_FAIL;
    }
    buf[ret] = '\0';

    cJSON *root = cJSON_Parse(buf);
    if (!root) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }

    const cJSON *motor = cJSON_GetObjectItem(root, "motor");
    const cJSON *speed = cJSON_GetObjectItem(root, "speed");

    if (cJSON_IsNumber(motor) && cJSON_IsNumber(speed)) {
        motor_control_set_motor_speed(motor->valueint, speed->valueint);
        ESP_LOGI(TAG, "Set motor %d to speed %d", motor->valueint, speed->valueint);
    } else {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing or invalid motor/speed");
        cJSON_Delete(root);
        return ESP_FAIL;
    }

    cJSON_Delete(root);
    httpd_resp_send(req, "OK", 2);
    return ESP_OK;
}

static const httpd_uri_t uri_root = { .uri = "/", .method = HTTP_GET, .handler = root_get_handler, .user_ctx = NULL };
static const httpd_uri_t uri_move = { .uri = "/move", .method = HTTP_POST, .handler = move_post_handler, .user_ctx = NULL };

void start_webserver(void)
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.lru_purge_enable = true;

    ESP_LOGI(TAG, "Starting server on port: '%d'", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_register_uri_handler(server, &uri_root);
        httpd_register_uri_handler(server, &uri_move);
        return;
    }

    ESP_LOGE(TAG, "Error starting server!");
}
