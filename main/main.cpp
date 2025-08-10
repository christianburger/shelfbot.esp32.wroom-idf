#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_http_server.h"
#include "mdns.h"

#include "wifi_station.h"
#include "motor_control.h"

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

static const char* TAG = "main";

// Agent IP address, discovered via mDNS
static char agent_ip_str[16];

// --- Error Handling ---
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){ESP_LOGE(TAG, "Failed status on line %d: %d. Aborting micro_ros_task.", __LINE__, (int)temp_rc); vTaskDelete(NULL);}}

// --- REST Server ---
static esp_err_t root_get_handler(httpd_req_t *req)
{
    const char* resp_str = "Hello from Shelfbot!";
    httpd_resp_send(req, resp_str, strlen(resp_str));
    return ESP_OK;
}

static const httpd_uri_t root_uri = {
    .uri       = "/",
    .method    = HTTP_GET,
    .handler   = root_get_handler,
    .user_ctx  = NULL
};

static httpd_handle_t start_webserver(void)
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.lru_purge_enable = true;

    ESP_LOGI(TAG, "Starting server on port: '%d'", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_register_uri_handler(server, &root_uri);
        return server;
    }

    ESP_LOGI(TAG, "Error starting server!");
    return NULL;
}

// --- mDNS ---
static void initialise_mdns(void)
{
    mdns_init();
    mdns_hostname_set("shelfbot");
    mdns_instance_name_set("Shelfbot ESP32 Client");
}

static bool query_mdns_host(const char * host_name)
{
    ESP_LOGI(TAG, "Querying for mDNS host: %s.local", host_name);

    esp_ip4_addr_t addr;
    addr.addr = 0;

    esp_err_t err = mdns_query_a(host_name, 5000,  &addr);

    if (err) {
        if (err == ESP_ERR_NOT_FOUND) {
            ESP_LOGE(TAG, "mDNS host '%s.local' not found!", host_name);
            return false;
        }
        ESP_LOGE(TAG, "mDNS query failed: %s", esp_err_to_name(err));
        return false;
    }

    inet_ntoa_r(addr.addr, agent_ip_str, sizeof(agent_ip_str) - 1);
    ESP_LOGI(TAG, "mDNS host '%s.local' found at IP: %s", host_name, agent_ip_str);
    return true;
}

// --- micro-ROS ---
void micro_ros_task(void * arg)
{
    char* agent_ip = (char*) arg;
    ESP_LOGI(TAG, "micro_ros_task started, trying to connect to agent at %s:8888", agent_ip);

    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;

    // Create init_options using the correct Humble API
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));
    // The arguments must be in the correct order: ip, port, rmw_options
    RCCHECK(rmw_uros_options_set_udp_address(agent_ip, "8888", rcl_init_options_get_rmw_init_options(&init_options)));

    // Now, init support with the configured options
    RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

    // Create node
    rcl_node_t node;
    RCCHECK(rclc_node_init_default(&node, "shelfbot_firmware", "", &support));

    // Create timer
    rcl_timer_t timer;
    RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(1000), [](rcl_timer_t * timer, int64_t last_call_time) {
        (void) last_call_time;
        if (timer != NULL) {
            ESP_LOGI(TAG, "Node is alive...");
        }
    }));

    // Create executor
    rclc_executor_t executor;
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));

    while(1){
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        usleep(100000);
    }

    // Free resources
    RCCHECK(rcl_timer_fini(&timer));
    RCCHECK(rcl_node_fini(&node));
    RCCHECK(rclc_support_fini(&support));
    RCCHECK(rcl_init_options_fini(&init_options));

    vTaskDelete(NULL);
}

// --- Main --- 
extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "Starting app_main()");

    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize networking stack ONCE
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // Start components
    motor_control_begin();
    
    // This function will now block until Wi-Fi is connected.
    wifi_init_sta();

    // Start services that require a network connection
    start_webserver();
    initialise_mdns();

    // Find agent and start micro-ROS
    if (query_mdns_host("gentoo-laptop")) {
        xTaskCreate(micro_ros_task, "micro_ros_task", 16000, agent_ip_str, 5, NULL);
    } else {
        ESP_LOGE(TAG, "Could not find micro-ROS agent. Aborting firmware.");
    }

    ESP_LOGI(TAG, "app_main() finished.");
}
