#include "http_server.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "cJSON.h"
#include "motor_control.h"

static const char *TAG = "http_server";

// Handler for the root URI
static esp_err_t root_handler(httpd_req_t *req)
{
    const char* resp_str = "<html><head><title>Shelfbot Control</title></head><body><h1>Shelfbot Control</h1><p><a href=\"/motors\">View All Motor Positions (JSON)</a></p></body></html>";
    httpd_resp_send(req, resp_str, strlen(resp_str));
    return ESP_OK;
}

// Handler for getting all motor positions
static esp_err_t get_all_motors_handler(httpd_req_t *req)
{
    cJSON *root = cJSON_CreateArray();
    int positions[6];
    get_all_motor_positions(positions, 6);

    for (int i = 0; i < 6; i++) {
        cJSON *motor_json = cJSON_CreateObject();
        cJSON_AddNumberToObject(motor_json, "motor", i + 1);
        cJSON_AddNumberToObject(motor_json, "position", positions[i]);
        cJSON_AddItemToArray(root, motor_json);
    }

    const char *json_string = cJSON_Print(root);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json_string, strlen(json_string));
    cJSON_Delete(root);
    free((void *)json_string);
    return ESP_OK;
}

// Handler for getting a single motor position
static esp_err_t get_motor_handler(httpd_req_t *req)
{
    char*  buf;
    size_t buf_len;

    buf_len = httpd_req_get_url_query_len(req) + 1;
    if (buf_len > 1) {
        buf = malloc(buf_len);
        if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {
            char param[32];
            if (httpd_query_key_value(buf, "id", param, sizeof(param)) == ESP_OK) {
                int motor_id = atoi(param);
                int position = get_motor_position(motor_id);
                cJSON *root = cJSON_CreateObject();
                cJSON_AddNumberToObject(root, "motor", motor_id);
                cJSON_AddNumberToObject(root, "position", position);
                const char *json_string = cJSON_Print(root);
                httpd_resp_set_type(req, "application/json");
                httpd_resp_send(req, json_string, strlen(json_string));
                cJSON_Delete(root);
                free((void *)json_string);
                free(buf);
                return ESP_OK;
            }
        }
        free(buf);
    }
    httpd_resp_send_404(req);
    return ESP_FAIL;
}

httpd_handle_t start_webserver(void)
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    ESP_LOGI(TAG, "Starting server on port: '%d'", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_uri_t root_uri = {
            .uri      = "/",
            .method   = HTTP_GET,
            .handler  = root_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &root_uri);

        httpd_uri_t all_motors_uri = {
            .uri      = "/motors",
            .method   = HTTP_GET,
            .handler  = get_all_motors_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &all_motors_uri);

        httpd_uri_t motor_uri = {
            .uri      = "/motor",
            .method   = HTTP_GET,
            .handler  = get_motor_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &motor_uri);
    }
    return server;
}
