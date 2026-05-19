// shelfbot.cpp
#include "shelfbot.hpp"

static const char* TAG = "shelfbot";

// --- Static Member Definitions (unchanged) ---
bool Shelfbot::time_synchronized = false;
bool Shelfbot::led_state = false;
Shelfbot* Shelfbot::instance = nullptr;

// --- Helper Functions (unchanged) ---
void init_multi_array(std_msgs__msg__Float32MultiArray& msg, float* data_buffer, int capacity) {
    msg.data.data = data_buffer;
    msg.data.capacity = capacity;
    msg.data.size = 0;
    msg.layout.dim.data = nullptr;
    msg.layout.dim.size = 0;
    msg.layout.dim.capacity = 0;
    msg.layout.data_offset = 0;
}

// --- mDNS (unchanged) ---
void Shelfbot::initialise_mdns(void) {
    mdns_init();
    mdns_hostname_set("shelfbot");
    mdns_instance_name_set("Shelfbot ESP32 Client");
}

bool Shelfbot::query_mdns_host(const char * host_name) {
    ESP_LOGI(TAG, "Querying for mDNS host: %s.local", host_name);
    esp_ip4_addr_t addr;
    addr.addr = 0;
    esp_err_t err = mdns_query_a(host_name, 2000, &addr);
    if(err){
        if(err == ESP_ERR_NOT_FOUND){
            ESP_LOGW(TAG, "mDNS host not found!");
            return false;
        }
        ESP_LOGE(TAG, "mDNS Query Failed: %d", err);
        return false;
    }
    ESP_LOGI(TAG, "mDNS Query Result: " IPSTR, IP2STR(&addr));
    snprintf(agent_ip_str, sizeof(agent_ip_str), IPSTR, IP2STR(&addr));
    return true;
}

// --- SNTP (unchanged) ---
void Shelfbot::time_sync_notification_cb(struct timeval *tv) {
    ESP_LOGI(TAG, "Time synchronized");
    time_synchronized = true;
}

void Shelfbot::initialize_sntp(void) {
    ESP_LOGI(TAG, "Initializing SNTP");
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "pool.ntp.org");
    sntp_set_time_sync_notification_cb(time_sync_notification_cb);
    esp_sntp_init();
}

// --- ROS Timer Callbacks (unchanged) ---
void Shelfbot::heartbeat_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
    (void) last_call_time;
    if (timer != NULL) {
        heartbeat_msg.data++;
        RCSOFTCHECK(rcl_publish(&heartbeat_publisher, &heartbeat_msg, NULL));
    }
}

void Shelfbot::motor_position_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
    (void) last_call_time;
    if (timer != NULL) {
        motor_pos_data[0] = motor_control_get_position(0);
        motor_pos_data[1] = motor_control_get_position(1);
        motor_position_msg.data.size = 2;
        RCSOFTCHECK(rcl_publish(&motor_position_publisher, &motor_position_msg, NULL));
    }
}

void Shelfbot::distance_sensors_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
    (void) last_call_time;
    if (timer != NULL) {
        SensorCommon::SensorDataPacket sensor_data;
        if (SensorManager::get_instance().get_latest_data(sensor_data)) {
            size_t idx = 0;
            for (int i = 0; i < SensorCommon::NUM_ULTRASONIC_SENSORS && idx < SensorCommon::NUM_ULTRASONIC_SENSORS; i++, idx++) {
                distance_sensors_data[idx] = sensor_data.ultrasonic_readings[i].distance_cm;
            }
            for (int i = 0; i < SensorCommon::NUM_TOF_SENSORS && idx < SensorCommon::NUM_SENSORS; i++, idx++) {
                distance_sensors_data[idx] = sensor_data.tof_measurements[i].distance_mm / 10.0f;
            }
            if (idx < SensorCommon::NUM_SENSORS) {
                distance_sensors_data[idx] = sensor_data.lidar_measurement.valid
                    ? (sensor_data.lidar_measurement.distance_mm / 10.0f)
                    : -1.0f;
                idx++;
            }
            distance_sensors_msg.data.size = idx;
            RCSOFTCHECK(rcl_publish(&distance_sensors_publisher, &distance_sensors_msg, NULL));
        }
    }
}

void Shelfbot::led_state_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
    (void) last_call_time;
    if (timer != NULL) {
        led_state_msg.data = led_state;
        RCSOFTCHECK(rcl_publish(&led_state_publisher, &led_state_msg, NULL));
    }
}

void Shelfbot::tof_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
    (void) last_call_time;
    if (timer != NULL) {
        SensorCommon::SensorDataPacket sensor_data;
        if (SensorManager::get_instance().get_latest_data(sensor_data)) {
            if (sensor_data.tof_measurements[0].valid) {
                tof_distance_msg.data = sensor_data.tof_measurements[0].distance_mm / 10.0f;
            } else {
                tof_distance_msg.data = -1.0f;
            }
        } else {
            tof_distance_msg.data = -1.0f;
        }
        RCSOFTCHECK(rcl_publish(&tof_distance_publisher, &tof_distance_msg, NULL));
    }
}

// --- ROS Subscription Callbacks (unchanged) ---
void Shelfbot::motor_command_subscription_callback(const void * msin) {
    const std_msgs__msg__Float32MultiArray * msg = (const std_msgs__msg__Float32MultiArray *)msin;
    if (msg->data.size >= 2) {
        motor_control_set_position(0, msg->data.data[0]);
        motor_control_set_position(1, msg->data.data[1]);
    }
}

void Shelfbot::set_speed_subscription_callback(const void * msin) {
    const std_msgs__msg__Float32MultiArray * msg = (const std_msgs__msg__Float32MultiArray *)msin;
    if (msg->data.size >= 2) {
         motor_control_set_velocity(0, msg->data.data[0]);
         motor_control_set_velocity(1, msg->data.data[1]);
    }
}

void Shelfbot::led_subscription_callback(const void * msin) {
    const std_msgs__msg__Bool * msg = (const std_msgs__msg__Bool *)msin;
    led_state = msg->data;
    led_control_set(led_state);
}

// --- Static Callback Wrappers (unchanged) ---
void Shelfbot::heartbeat_timer_callback_wrapper(rcl_timer_t * t, int64_t l) { if(instance) instance->heartbeat_timer_callback(t, l); }
void Shelfbot::motor_position_timer_callback_wrapper(rcl_timer_t * t, int64_t l) { if(instance) instance->motor_position_timer_callback(t, l); }
void Shelfbot::distance_sensors_timer_callback_wrapper(rcl_timer_t * t, int64_t l) { if(instance) instance->distance_sensors_timer_callback(t, l); }
void Shelfbot::led_state_timer_callback_wrapper(rcl_timer_t * t, int64_t l) { if(instance) instance->led_state_timer_callback(t, l); }
void Shelfbot::tof_timer_callback_wrapper(rcl_timer_t * t, int64_t l) { if(instance) instance->tof_timer_callback(t, l); }
void Shelfbot::motor_command_subscription_callback_wrapper(const void * m) { if(instance) instance->motor_command_subscription_callback(m); }
void Shelfbot::set_speed_subscription_callback_wrapper(const void * m) { if(instance) instance->set_speed_subscription_callback(m); }
void Shelfbot::led_subscription_callback_wrapper(const void * m) { if(instance) instance->led_subscription_callback(m); }
void Shelfbot::micro_ros_task_wrapper(void * arg) { if(instance) instance->micro_ros_task_impl(); }

// --- Entity Lifecycle (unchanged) ---
void Shelfbot::create_entities() {
    allocator = rcl_get_default_allocator();
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCCHECK(rclc_node_init_default(&node, "shelfbot_firmware", "", &support));

    RCCHECK(rclc_publisher_init_default(&heartbeat_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "shelfbot_firmware/heartbeat"));
    RCCHECK(rclc_publisher_init_default(&motor_position_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "shelfbot_firmware/motor_positions"));
    RCCHECK(rclc_publisher_init_default(&distance_sensors_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "shelfbot_firmware/distance_sensors"));
    RCCHECK(rclc_publisher_init_default(&led_state_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "shelfbot_firmware/led_state"));
    RCCHECK(rclc_publisher_init_default(&tof_distance_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "shelfbot_firmware/tof_distance"));

    RCCHECK(rclc_subscription_init_default(&motor_command_subscription, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "shelfbot_firmware/motor_command"));
    RCCHECK(rclc_subscription_init_default(&set_speed_subscription, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "shelfbot_firmware/set_speed"));
    RCCHECK(rclc_subscription_init_default(&led_subscription, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "shelfbot_firmware/led"));

    RCCHECK(rclc_timer_init_default(&heartbeat_timer, &support, RCL_MS_TO_NS(1000), heartbeat_timer_callback_wrapper));
    RCCHECK(rclc_timer_init_default(&motor_position_timer, &support, RCL_MS_TO_NS(100), motor_position_timer_callback_wrapper));
    RCCHECK(rclc_timer_init_default(&distance_sensors_timer, &support, RCL_MS_TO_NS(200), distance_sensors_timer_callback_wrapper));
    RCCHECK(rclc_timer_init_default(&led_state_timer, &support, RCL_MS_TO_NS(2000), led_state_timer_callback_wrapper));
    RCCHECK(rclc_timer_init_default(&tof_timer, &support, RCL_MS_TO_NS(100), tof_timer_callback_wrapper));

    unsigned int num_handles = 5 + 3;
    RCCHECK(rclc_executor_init(&executor, &support.context, num_handles, &allocator));

    RCCHECK(rclc_executor_add_timer(&executor, &heartbeat_timer));
    RCCHECK(rclc_executor_add_timer(&executor, &motor_position_timer));
    RCCHECK(rclc_executor_add_timer(&executor, &distance_sensors_timer));
    RCCHECK(rclc_executor_add_timer(&executor, &led_state_timer));
    RCCHECK(rclc_executor_add_timer(&executor, &tof_timer));

    RCCHECK(rclc_executor_add_subscription(&executor, &motor_command_subscription, &motor_command_msg, &motor_command_subscription_callback_wrapper, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &set_speed_subscription, &set_speed_msg, &set_speed_subscription_callback_wrapper, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &led_subscription, &led_msg, &led_subscription_callback_wrapper, ON_NEW_DATA));

    init_multi_array(motor_position_msg, motor_pos_data, 2);
    init_multi_array(distance_sensors_msg, distance_sensors_data, SensorCommon::NUM_SENSORS);
}

void Shelfbot::destroy_entities() {
    rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
    (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
    rcl_ret_t ret;
    ret = rcl_publisher_fini(&heartbeat_publisher, &node);       (void)ret;
    ret = rcl_publisher_fini(&motor_position_publisher, &node);  (void)ret;
    ret = rcl_publisher_fini(&distance_sensors_publisher, &node);(void)ret;
    ret = rcl_publisher_fini(&led_state_publisher, &node);       (void)ret;
    ret = rcl_publisher_fini(&tof_distance_publisher, &node);    (void)ret;
    ret = rcl_subscription_fini(&motor_command_subscription, &node); (void)ret;
    ret = rcl_subscription_fini(&set_speed_subscription, &node);     (void)ret;
    ret = rcl_subscription_fini(&led_subscription, &node);           (void)ret;
    ret = rcl_timer_fini(&heartbeat_timer);        (void)ret;
    ret = rcl_timer_fini(&motor_position_timer);   (void)ret;
    ret = rcl_timer_fini(&distance_sensors_timer); (void)ret;
    ret = rcl_timer_fini(&led_state_timer);        (void)ret;
    ret = rcl_timer_fini(&tof_timer);              (void)ret;
    rclc_executor_fini(&executor);
    ret = rcl_node_fini(&node); (void)ret;
    rclc_support_fini(&support);
}

// --- New network services task (waits for Wi-Fi, then starts services) ---
static void network_services_task(void *arg) {
    Shelfbot* bot = (Shelfbot*)arg;
    EventGroupHandle_t wifi_evt = wifi_manager_get_event_group();

    ESP_LOGI(TAG, "Network services task: waiting for Wi-Fi connection...");
    xEventGroupWaitBits(wifi_evt, WM_CONNECTED_BIT, pdFALSE, pdFALSE, portMAX_DELAY);

    ESP_LOGI(TAG, "Wi-Fi connected – starting mDNS, HTTP server, and SNTP");
    bot->initialise_mdns();

    esp_err_t err = HttpServer::get_instance().start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start HTTP server: %s", esp_err_to_name(err));
    }

    bot->initialize_sntp();

    // Keep task alive (can also suspend)
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(60000));
    }
}

// --- Micro-ROS task implementation (full, with Wi-Fi wait) ---
void Shelfbot::micro_ros_task_impl() {
    // Wait for Wi-Fi connection before any mDNS query
    EventGroupHandle_t wifi_evt = wifi_manager_get_event_group();
    ESP_LOGI(TAG, "Micro-ROS task waiting for Wi-Fi...");
    xEventGroupWaitBits(wifi_evt, WM_CONNECTED_BIT, pdFALSE, pdFALSE, portMAX_DELAY);
    ESP_LOGI(TAG, "Wi-Fi ready, starting Micro-ROS agent discovery");

    bool entities_created = false;
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();

    while(1) {
        switch(state) {
            case WAITING_AGENT:
                if (query_mdns_host("gentoo-laptop")) {
                    rcl_ret_t ret = rcl_init_options_init(&init_options, allocator);
                    if (ret != RCL_RET_OK) {
                        ESP_LOGE(TAG, "Failed to initialize rcl init options: %ld", ret);
                        break;
                    }
                    rmw_uros_options_set_udp_address(agent_ip_str, "8888", rcl_init_options_get_rmw_init_options(&init_options));
                    if (rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator) == RCL_RET_OK) {
                        state = AGENT_CONNECTED;
                    }
                    ret = rcl_init_options_fini(&init_options);
                    if (ret != RCL_RET_OK) {
                        ESP_LOGE(TAG, "Failed to finalize rcl init options: %ld", ret);
                    }
                }
                vTaskDelay(pdMS_TO_TICKS(2000));
                break;

            case AGENT_CONNECTED:
                if (!entities_created) {
                    create_entities();
                    entities_created = true;
                }
                if (rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)) != RCL_RET_OK) {
                    state = AGENT_DISCONNECTED;
                }
                break;

            case AGENT_DISCONNECTED:
                if (entities_created) {
                    destroy_entities();
                    entities_created = false;
                }
                rclc_support_fini(&support);
                state = WAITING_AGENT;
                break;

            default:
                break;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// --- Main Begin (refactored) ---
void Shelfbot::begin() {
    instance = this;

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "Starting Shelfbot Firmware");
    ESP_LOGI(TAG, "Firmware Version: %s", FirmwareVersion::get_version_string());
    ESP_LOGI(TAG, "========================================");

    // 1. Hardware subsystems
    ESP_LOGI(TAG, "1. Initializing hardware subsystems...");
    led_control_init();
    motor_control_begin();

    // 2. NVS (needed for Wi‑Fi)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_flash_init();
    }

    // 3. Initialize Wi‑Fi manager (starts netif, event loop, and manager task)
    ESP_LOGI(TAG, "2. Initializing Wi‑Fi manager...");
    ESP_ERROR_CHECK(wifi_manager_init());

    // 4. Sensor Manager (does not depend on network)
    ESP_LOGI(TAG, "3. Initializing sensors...");
    SensorControl::Config sensor_config;
    sensor_config.ultrasonic_configs = {
        {.trig_pin = 25, .echo_pin = 34, .timeout_us = 30000, .max_distance_mm = 4000},
        {.trig_pin = 32, .echo_pin = 35, .timeout_us = 30000, .max_distance_mm = 4000},
        {.trig_pin = 16, .echo_pin = 36, .timeout_us = 30000, .max_distance_mm = 4000},
        {.trig_pin = 17, .echo_pin = 39, .timeout_us = 30000, .max_distance_mm = 4000},
    };
    for (int i = 0; i < SensorCommon::NUM_TOF_SENSORS; i++) {
        sensor_config.tof_configs[i].timeout_ms = 500;
        sensor_config.tof_configs[i].enabled = false;
    }
    sensor_config.ultrasonic_read_interval_ms = 100;
    sensor_config.tof_read_interval_ms = 200;
    sensor_config.lidar_config.enabled      = true;
    sensor_config.lidar_config.uart_port    = UART_NUM_2;
    sensor_config.lidar_config.uart_tx_pin  = UART_PIN_NO_CHANGE;
    sensor_config.lidar_config.uart_rx_pin  = GPIO_NUM_3;
    sensor_config.lidar_config.baud_rate    = 115200;

    SensorManager::get_instance().initialize(sensor_config);
    SensorManager::get_instance().start();

    // 5. Start network services task (will wait for Wi-Fi)
    ESP_LOGI(TAG, "4. Starting network services task...");
    xTaskCreate(network_services_task, "net_services", 8192, this, 5, nullptr);

    // 6. Start Micro‑ROS task (will wait for Wi-Fi)
    ESP_LOGI(TAG, "5. Starting Micro‑ROS task...");
    xTaskCreate(micro_ros_task_wrapper, "uros_task", 16000, this, 5, nullptr);

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "Shelfbot Firmware Initialization Complete");
    ESP_LOGI(TAG, "========================================");
}