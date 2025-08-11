#include "shelfbot.h"

#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <inttypes.h> // For PRId32 macro

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "mdns.h"
#include "esp_sntp.h"

#include "wifi_station.h"
#include "motor_control.h"
#include "http_server.h"

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <std_msgs/msg/int32.h>

static const char* TAG = "shelfbot";

// --- Error Handling ---
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){ESP_LOGE(TAG, "Failed status on line %d: %d. Aborting micro_ros_task.", __LINE__, (int)temp_rc); vTaskDelete(NULL);}}

// --- Static Member Definitions ---
bool Shelfbot::time_synchronized = false;
rcl_publisher_t Shelfbot::heartbeat_publisher;
std_msgs__msg__Int32 Shelfbot::heartbeat_msg;

// --- mDNS ---
void Shelfbot::initialise_mdns(void)
{
    mdns_init();
    mdns_hostname_set("shelfbot");
    mdns_instance_name_set("Shelfbot ESP32 Client");
}

bool Shelfbot::query_mdns_host(const char * host_name)
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

// --- SNTP ---
void Shelfbot::time_sync_notification_cb(struct timeval *tv)
{
    ESP_LOGI(TAG, "Time synchronized");
    time_synchronized = true;
}

void Shelfbot::initialize_sntp(void)
{
    ESP_LOGI(TAG, "Initializing SNTP");
    esp_sntp_setoperatingmode(ESP_SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "pool.ntp.org");
    sntp_set_time_sync_notification_cb(time_sync_notification_cb);
    esp_sntp_init();
}

// --- micro-ROS Timer Callback ---
void Shelfbot::heartbeat_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
    (void) last_call_time;
    if (timer != NULL) {
        heartbeat_msg.data++;
        RCCHECK(rcl_publish(&heartbeat_publisher, &heartbeat_msg, NULL));
        ESP_LOGI(TAG, "Heartbeat sent: %" PRId32, heartbeat_msg.data);
    }
}

// --- micro-ROS Task ---
void Shelfbot::micro_ros_task_wrapper(void * arg)
{
    Shelfbot* shelfbot = static_cast<Shelfbot*>(arg);
    shelfbot->micro_ros_task_impl();
}

void Shelfbot::micro_ros_task_impl()
{
    ESP_LOGI(TAG, "micro_ros_task started, trying to connect to agent at %s:8888", agent_ip_str);

    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;

    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));
    RCCHECK(rmw_uros_options_set_udp_address(agent_ip_str, "8888", rcl_init_options_get_rmw_init_options(&init_options)));

    RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

    rcl_node_t node;
    RCCHECK(rclc_node_init_default(&node, "shelfbot_firmware", "", &support));
    ESP_LOGI(TAG, "micro-ROS node created successfully.");

    RCCHECK(rclc_publisher_init_default(
        &heartbeat_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "/shelfbot_firmware/heartbeat"));

    rcl_timer_t heartbeat_timer;
    RCCHECK(rclc_timer_init_default(&heartbeat_timer, &support, RCL_MS_TO_NS(2000), heartbeat_timer_callback));

    rclc_executor_t executor;
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &heartbeat_timer));

    heartbeat_msg.data = 0;
    while(1){
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        usleep(100000);
    }

    RCCHECK(rcl_publisher_fini(&heartbeat_publisher, &node));
    RCCHECK(rcl_timer_fini(&heartbeat_timer));
    RCCHECK(rcl_node_fini(&node));
    RCCHECK(rclc_support_fini(&support));
    RCCHECK(rcl_init_options_fini(&init_options));

    vTaskDelete(NULL);
}


void Shelfbot::begin()
{
    ESP_LOGI(TAG, "Starting Shelfbot");

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    motor_control_begin();
    
    wifi_init_sta();

    initialize_sntp();
    int retry = 0;
    const int retry_count = 15;
    while (!time_synchronized && ++retry < retry_count) {
        ESP_LOGI(TAG, "Waiting for system time to be set... (%d/%d)", retry, retry_count);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
    if (!time_synchronized) {
        ESP_LOGE(TAG, "Failed to synchronize time. Continuing without real time.");
    }

    start_webserver();
    initialise_mdns();

    if (query_mdns_host("gentoo-laptop")) {
        xTaskCreate(micro_ros_task_wrapper, "micro_ros_task", 16000, this, 5, NULL);
    } else {
        ESP_LOGE(TAG, "Could not find micro-ROS agent. Aborting firmware.");
    }

    ESP_LOGI(TAG, "Shelfbot initialization complete. Entering main loop.");

    while (true) {
        // The main task can sleep here, or perform other periodic tasks.
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
