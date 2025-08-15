#include "shelfbot.h"
#include <rcl/allocator.h>

static const char* TAG = "shelfbot";

// --- Helper Functions ---
void init_multi_array(std_msgs__msg__Float32MultiArray& msg, float* data_buffer, int capacity) {
    msg.data.data = data_buffer;
    msg.data.capacity = capacity;
    msg.data.size = 0;
    // Initialize layout
    msg.layout.dim.data = NULL;
    msg.layout.dim.size = 0;
    msg.layout.dim.capacity = 0;
    msg.layout.data_offset = 0;
}

// --- Error Handling ---
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){ESP_LOGE(TAG, "Failed status on line %d: %d. Aborting micro_ros_task.", __LINE__, (int)temp_rc); vTaskDelete(NULL);}}

// --- Static Member Definitions ---
bool Shelfbot::time_synchronized = false;
rcl_publisher_t Shelfbot::heartbeat_publisher;
std_msgs__msg__Int32 Shelfbot::heartbeat_msg;
rcl_publisher_t Shelfbot::motor_position_publisher;
std_msgs__msg__Float32MultiArray Shelfbot::motor_position_msg;
float Shelfbot::motor_position_data[NUM_MOTORS];
rcl_publisher_t Shelfbot::distance_sensors_publisher;
std_msgs__msg__Float32MultiArray Shelfbot::distance_sensors_msg;
float Shelfbot::distance_sensors_data[8];
rcl_subscription_t Shelfbot::motor_command_subscriber;
std_msgs__msg__Float32MultiArray Shelfbot::motor_command_msg;
float Shelfbot::motor_command_data[NUM_MOTORS];
rcl_subscription_t Shelfbot::set_speed_subscriber;
std_msgs__msg__Float32 Shelfbot::set_speed_msg;
rcl_subscription_t Shelfbot::led_subscriber;
std_msgs__msg__Bool Shelfbot::led_msg;

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

    esp_ip4addr_ntoa(&addr, agent_ip_str, sizeof(agent_ip_str));
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

// --- micro-ROS Callbacks ---
void Shelfbot::heartbeat_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
    (void) last_call_time;
    if (timer != NULL) {
        heartbeat_msg.data++;
        RCCHECK(rcl_publish(&heartbeat_publisher, &heartbeat_msg, NULL));
    }
}

void Shelfbot::motor_position_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
    (void) last_call_time;
    if (timer != NULL) {
        // Populate the message with current motor positions
        for (int i = 0; i < NUM_MOTORS; ++i) {
            motor_position_msg.data.data[i] = motor_control_get_position(i);
        }
        motor_position_msg.data.size = NUM_MOTORS;

        // Publish the message
        RCCHECK(rcl_publish(&motor_position_publisher, &motor_position_msg, NULL));
    }
}

void Shelfbot::distance_sensors_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
    (void) last_call_time;
    if (timer != NULL) {
        // In a real implementation, you would read the sensor values here.
        // For now, we'll just publish a placeholder.
        for (int i = 0; i < 8; ++i) {
            distance_sensors_msg.data.data[i] = 0.0f; // Placeholder value
        }
        distance_sensors_msg.data.size = 8;

        // Publish the message
        RCCHECK(rcl_publish(&distance_sensors_publisher, &distance_sensors_msg, NULL));
    }
}

void Shelfbot::motor_command_subscription_callback(const void * msin) {
    const std_msgs__msg__Float32MultiArray * msg = (const std_msgs__msg__Float32MultiArray *)msin;
    if (msg->data.size > NUM_MOTORS) {
        ESP_LOGW(TAG, "Received motor command with %d positions, but only %d are supported. Ignoring extra values.", msg->data.size, NUM_MOTORS);
    }

    for (size_t i = 0; i < msg->data.size && i < NUM_MOTORS; i++) {
        ESP_LOGI(TAG, "Motor %d command: %.2f rad", i, msg->data.data[i]);
        motor_control_set_position(i, msg->data.data[i]);
    }
}

void Shelfbot::set_speed_subscription_callback(const void * msin) {
    const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msin;
    long speed_in_hz = (long)msg->data;
    ESP_LOGI(TAG, "Set speed command received: %ld Hz", speed_in_hz);
    motor_control_set_all_speeds_hz(speed_in_hz);
}

void Shelfbot::led_subscription_callback(const void * msin) {
    const std_msgs__msg__Bool * msg = (const std_msgs__msg__Bool *)msin;
    ESP_LOGI(TAG, "LED command received: %s", msg->data ? "ON" : "OFF");
    led_control_set(msg->data);
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

    // Create init_options
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));
    RCCHECK(rmw_uros_options_set_udp_address(agent_ip_str, "8888", rcl_init_options_get_rmw_init_options(&init_options)));

    // Setup support structure
    RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

    // Create node
    rcl_node_t node;
    RCCHECK(rclc_node_init_default(&node, "shelfbot_firmware", "", &support));
    ESP_LOGI(TAG, "micro-ROS node created successfully.");

    // Create Publishers
    ESP_LOGI(TAG, "Creating heartbeat publisher...");
    RCCHECK(rclc_publisher_init_default(
        &heartbeat_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "/shelfbot_firmware/heartbeat"));

    ESP_LOGI(TAG, "Creating motor position publisher...");
    RCCHECK(rclc_publisher_init_default(
        &motor_position_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "/shelfbot_firmware/motor_positions"));

    ESP_LOGI(TAG, "Creating distance sensors publisher...");
    RCCHECK(rclc_publisher_init_default(
        &distance_sensors_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "/shelfbot_firmware/distance_sensors"));

    // Create Subscribers
    ESP_LOGI(TAG, "Creating motor command subscriber...");
    RCCHECK(rclc_subscription_init_default(
        &motor_command_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "/shelfbot_firmware/motor_command"));

    ESP_LOGI(TAG, "Creating set speed subscriber...");
    RCCHECK(rclc_subscription_init_default(
        &set_speed_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "/shelfbot_firmware/set_speed"));

    ESP_LOGI(TAG, "Creating LED subscriber...");
    RCCHECK(rclc_subscription_init_default(
        &led_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
        "/shelfbot_firmware/led"));

    // Create Timers
    ESP_LOGI(TAG, "Creating timers...");
    rcl_timer_t heartbeat_timer;
    RCCHECK(rclc_timer_init_default2(&heartbeat_timer, &support, RCL_MS_TO_NS(2000), heartbeat_timer_callback, true));
    rcl_timer_t motor_position_timer;
    RCCHECK(rclc_timer_init_default2(&motor_position_timer, &support, RCL_MS_TO_NS(500), motor_position_timer_callback, true));
    rcl_timer_t distance_sensors_timer;
    RCCHECK(rclc_timer_init_default2(&distance_sensors_timer, &support, RCL_MS_TO_NS(1000), distance_sensors_timer_callback, true));

    // Statically allocate memory for messages
    init_multi_array(motor_command_msg, Shelfbot::motor_command_data, NUM_MOTORS);
    init_multi_array(motor_position_msg, Shelfbot::motor_position_data, NUM_MOTORS);
    init_multi_array(distance_sensors_msg, Shelfbot::distance_sensors_data, 8);

    // Create Executor
    ESP_LOGI(TAG, "Creating executor...");
    rclc_executor_t executor;
    unsigned int num_handles = 3 + 3; // 3 timers, 3 subscribers
    RCCHECK(rclc_executor_init(&executor, &support.context, num_handles, &allocator));

    ESP_LOGI(TAG, "Adding entities to executor...");
    RCCHECK(rclc_executor_add_timer(&executor, &heartbeat_timer));
    RCCHECK(rclc_executor_add_timer(&executor, &motor_position_timer));
    RCCHECK(rclc_executor_add_timer(&executor, &distance_sensors_timer));
    RCCHECK(rclc_executor_add_subscription(&executor, &motor_command_subscriber, &motor_command_msg, &motor_command_subscription_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &set_speed_subscriber, &set_speed_msg, &set_speed_subscription_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &led_subscriber, &led_msg, &led_subscription_callback, ON_NEW_DATA));

    ESP_LOGI(TAG, "Executor setup complete.");
    heartbeat_msg.data = 0;
    while(1){
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        usleep(100000);
    }

    // Cleanup
    RCCHECK(rcl_publisher_fini(&heartbeat_publisher, &node));
    RCCHECK(rcl_publisher_fini(&motor_position_publisher, &node));
    RCCHECK(rcl_publisher_fini(&distance_sensors_publisher, &node));
    RCCHECK(rcl_subscription_fini(&motor_command_subscriber, &node));
    RCCHECK(rcl_subscription_fini(&set_speed_subscriber, &node));
    RCCHECK(rcl_subscription_fini(&led_subscriber, &node));
    RCCHECK(rcl_timer_fini(&heartbeat_timer));
    RCCHECK(rcl_timer_fini(&motor_position_timer));
    RCCHECK(rcl_timer_fini(&distance_sensors_timer));
    RCCHECK(rcl_node_fini(&node));
    RCCHECK(rclc_support_fini(&support));
    RCCHECK(rcl_init_options_fini(&init_options));

    vTaskDelete(NULL);
}


void Shelfbot::begin()
{
    ESP_LOGI(TAG, "Starting Shelfbot");

    // Initialize peripherals
    led_control_init();
    motor_control_begin();

    // Initialize networking
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    
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