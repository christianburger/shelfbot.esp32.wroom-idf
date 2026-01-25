#include "shelfbot.hpp"
#include "sensor_control.hpp"
#include <rcl/allocator.h>
#include <rmw_microros/init_options.h>

static const char* TAG = "shelfbot";

// --- Helper Functions ---
void init_multi_array(std_msgs__msg__Float32MultiArray& msg, float* data_buffer, int capacity) {
    msg.data.data = data_buffer;
    msg.data.capacity = capacity;
    msg.data.size = 0;
    msg.layout.dim.data = NULL;
    msg.layout.dim.size = 0;
    msg.layout.dim.capacity = 0;
    msg.layout.data_offset = 0;
}

// --- Static Member Definitions ---
bool Shelfbot::time_synchronized = false;
bool Shelfbot::led_state = false;
Shelfbot* Shelfbot::instance = nullptr;

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

    esp_err_t err = mdns_query_a(host_name, 2000,  &addr);

    if (err) {
        if (err == ESP_ERR_NOT_FOUND) {
            ESP_LOGW(TAG, "mDNS host '%s.local' not found!", host_name);
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
        if (rcl_publish(&heartbeat_publisher, &heartbeat_msg, NULL) != RCL_RET_OK) {
            ESP_LOGE(TAG, "Error publishing heartbeat message");
            agent_connected = false;
        }
    }
}

void Shelfbot::motor_position_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
    (void) last_call_time;
    if (timer != NULL) {
        for (int i = 0; i < NUM_MOTORS; ++i) {
            motor_position_msg.data.data[i] = motor_control_get_position(i);
        }
        motor_position_msg.data.size = NUM_MOTORS;
        if (rcl_publish(&motor_position_publisher, &motor_position_msg, NULL) != RCL_RET_OK) {
            ESP_LOGE(TAG, "Error publishing motor position message");
            agent_connected = false;
        }
    }
}

void Shelfbot::distance_sensors_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
    (void) last_call_time;
    if (timer != NULL) {
        float received_distances[NUM_SENSORS];
        if (xQueueReceive(distance_data_queue, &received_distances, 0) == pdPASS) {
            for (int i = 0; i < NUM_SENSORS; ++i) {
                distance_sensors_msg.data.data[i] = received_distances[i];
            }
        }
        distance_sensors_msg.data.size = NUM_SENSORS;
        if (rcl_publish(&distance_sensors_publisher, &distance_sensors_msg, NULL) != RCL_RET_OK) {
            ESP_LOGE(TAG, "Error publishing distance sensors message");
            agent_connected = false;
        }
    }
}

void Shelfbot::led_state_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
    (void) last_call_time;
    if (timer != NULL) {
        led_state_msg.data = led_state;
        if (rcl_publish(&led_state_publisher, &led_state_msg, NULL) != RCL_RET_OK) {
            ESP_LOGE(TAG, "Error publishing led state message");
            agent_connected = false;
        }
    }
}

void Shelfbot::tof_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
    (void) last_call_time;
    if (timer != NULL) {
        std::vector<SensorCommon::Reading> readings;
        if (ToFSensorManager::instance().get_latest_readings(readings)) {
            if (!readings.empty() && readings[0].valid) {
                tof_distance_msg.data = readings[0].distance_cm;
            } else {
                tof_distance_msg.data = -1.0f;
            }
        } else {
            tof_distance_msg.data = -1.0f;
        }

        if (rcl_publish(&tof_distance_publisher, &tof_distance_msg, NULL) != RCL_RET_OK) {
            ESP_LOGE(TAG, "Error publishing ToF distance");
            agent_connected = false;
        }
    }
}

void Shelfbot::motor_command_subscription_callback(const void * msin) {
    const std_msgs__msg__Float32MultiArray * msg = (const std_msgs__msg__Float32MultiArray *)msin;
    if (msg->data.size > NUM_MOTORS) {
        ESP_LOGW(TAG, "Received motor command with %d positions, but only %d are supported. Ignoring extra values.", msg->data.size, NUM_MOTORS);
    }

    for (size_t i = 0; i < msg->data.size && i < NUM_MOTORS; i++) {
        ESP_LOGD(TAG, "Motor %d command: %.2f rad", i, msg->data.data[i]);
        motor_control_set_position(i, msg->data.data[i]);
    }
}

void Shelfbot::set_speed_subscription_callback(const void * msin) {
    const std_msgs__msg__Float32MultiArray * msg = (const std_msgs__msg__Float32MultiArray *)msin;

    if (msg->data.size > NUM_MOTORS) {
        ESP_LOGW(TAG, "Received set_speed command with %d values, but only %d motors are supported. Ignoring extra values.", msg->data.size, NUM_MOTORS);
    }

    for (size_t i = 0; i < msg->data.size && i < NUM_MOTORS; i++) {
        float velocity_in_rad_s = msg->data.data[i];
        ESP_LOGD(TAG, "Motor %d velocity command: %.2f rad/s", i, velocity_in_rad_s);
        motor_control_set_velocity(i, velocity_in_rad_s);
    }
}

void Shelfbot::led_subscription_callback(const void * msin) {
    const std_msgs__msg__Bool * msg = (const std_msgs__msg__Bool *)msin;
    ESP_LOGI(TAG, "LED command received: %s", msg->data ? "ON" : "OFF");
    led_state = msg->data;
    led_control_set(led_state);
}

// --- Static Callback Wrappers ---
void Shelfbot::heartbeat_timer_callback_wrapper(rcl_timer_t* timer, int64_t last_call_time) {
    if (instance) instance->heartbeat_timer_callback(timer, last_call_time);
}
void Shelfbot::motor_position_timer_callback_wrapper(rcl_timer_t* timer, int64_t last_call_time) {
    if (instance) instance->motor_position_timer_callback(timer, last_call_time);
}
void Shelfbot::distance_sensors_timer_callback_wrapper(rcl_timer_t* timer, int64_t last_call_time) {
    if (instance) instance->distance_sensors_timer_callback(timer, last_call_time);
}
void Shelfbot::led_state_timer_callback_wrapper(rcl_timer_t* timer, int64_t last_call_time) {
    if (instance) instance->led_state_timer_callback(timer, last_call_time);
}
void Shelfbot::tof_timer_callback_wrapper(rcl_timer_t* timer, int64_t last_call_time) {
    if (instance) instance->tof_timer_callback(timer, last_call_time);
}
void Shelfbot::motor_command_subscription_callback_wrapper(const void* msin) {
    if (instance) instance->motor_command_subscription_callback(msin);
}
void Shelfbot::set_speed_subscription_callback_wrapper(const void* msin) {
    if (instance) instance->set_speed_subscription_callback(msin);
}
void Shelfbot::led_subscription_callback_wrapper(const void* msin) {
    if (instance) instance->led_subscription_callback(msin);
}

// --- micro-ROS Entity Management ---
bool Shelfbot::create_entities() {
    allocator = rcl_get_default_allocator();

    // Create node
    RCCHECK(rclc_node_init_default(&node, "shelfbot_firmware", "", &support));

    // Create Publishers
    RCCHECK(rclc_publisher_init_default(&heartbeat_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "/shelfbot_firmware/heartbeat"));
    RCCHECK(rclc_publisher_init_default(&motor_position_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "/shelfbot_firmware/motor_positions"));
    RCCHECK(rclc_publisher_init_default(&distance_sensors_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "/shelfbot_firmware/distance_sensors"));
    RCCHECK(rclc_publisher_init_default(&led_state_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "/shelfbot_firmware/led_state"));
    RCCHECK(rclc_publisher_init_default(&tof_distance_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "/shelfbot_firmware/tof_distance"));

    // Create Subscribers
    RCCHECK(rclc_subscription_init_default(&motor_command_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "/shelfbot_firmware/motor_command"));
    RCCHECK(rclc_subscription_init_default(&set_speed_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "/shelfbot_firmware/set_speed"));
    RCCHECK(rclc_subscription_init_default(&led_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "/shelfbot_firmware/led"));

    // Create Timers
    RCCHECK(rclc_timer_init_default(&heartbeat_timer, &support, RCL_MS_TO_NS(2000), heartbeat_timer_callback_wrapper));
    RCCHECK(rclc_timer_init_default(&motor_position_timer, &support, RCL_MS_TO_NS(500), motor_position_timer_callback_wrapper));
    RCCHECK(rclc_timer_init_default(&distance_sensors_timer, &support, RCL_MS_TO_NS(1000), distance_sensors_timer_callback_wrapper));
    RCCHECK(rclc_timer_init_default(&led_state_timer, &support, RCL_MS_TO_NS(2000), led_state_timer_callback_wrapper));
    RCCHECK(rclc_timer_init_default(&tof_timer, &support, RCL_MS_TO_NS(100), tof_timer_callback_wrapper));

    // Initialize message memory
    init_multi_array(motor_command_msg, motor_command_data, NUM_MOTORS);
    init_multi_array(motor_position_msg, motor_position_data, NUM_MOTORS);
    init_multi_array(set_speed_msg, set_speed_data, NUM_MOTORS);
    init_multi_array(distance_sensors_msg, distance_sensors_data, NUM_SENSORS);
    tof_distance_msg.data = 0.0f;

    // Create Executor
    unsigned int num_handles = 5 + 3; // 5 timers, 3 subscribers
    RCCHECK(rclc_executor_init(&executor, &support.context, num_handles, &allocator));

    RCCHECK(rclc_executor_add_timer(&executor, &heartbeat_timer));
    RCCHECK(rclc_executor_add_timer(&executor, &motor_position_timer));
    RCCHECK(rclc_executor_add_timer(&executor, &distance_sensors_timer));
    RCCHECK(rclc_executor_add_timer(&executor, &led_state_timer));
    RCCHECK(rclc_executor_add_timer(&executor, &tof_timer));
    RCCHECK(rclc_executor_add_subscription(&executor, &motor_command_subscriber, &motor_command_msg, &motor_command_subscription_callback_wrapper, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &set_speed_subscriber, &set_speed_msg, &set_speed_subscription_callback_wrapper, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &led_subscriber, &led_msg, &led_subscription_callback_wrapper, ON_NEW_DATA));

    heartbeat_msg.data = 0;
    return true;
}

void Shelfbot::destroy_entities() {
    RCSOFTCHECK(rclc_executor_fini(&executor));
    RCSOFTCHECK(rcl_subscription_fini(&motor_command_subscriber, &node));
    RCSOFTCHECK(rcl_subscription_fini(&set_speed_subscriber, &node));
    RCSOFTCHECK(rcl_subscription_fini(&led_subscriber, &node));
    RCSOFTCHECK(rcl_publisher_fini(&heartbeat_publisher, &node));
    RCSOFTCHECK(rcl_publisher_fini(&motor_position_publisher, &node));
    RCSOFTCHECK(rcl_publisher_fini(&distance_sensors_publisher, &node));
    RCSOFTCHECK(rcl_publisher_fini(&led_state_publisher, &node));
    RCSOFTCHECK(rcl_publisher_fini(&tof_distance_publisher, &node));
    RCSOFTCHECK(rcl_timer_fini(&heartbeat_timer));
    RCSOFTCHECK(rcl_timer_fini(&motor_position_timer));
    RCSOFTCHECK(rcl_timer_fini(&distance_sensors_timer));
    RCSOFTCHECK(rcl_timer_fini(&led_state_timer));
    RCSOFTCHECK(rcl_timer_fini(&tof_timer));
    RCSOFTCHECK(rcl_node_fini(&node));
}

// --- micro-ROS Task ---
void Shelfbot::micro_ros_task_wrapper(void * arg)
{
    Shelfbot* shelfbot = static_cast<Shelfbot*>(arg);
    shelfbot->micro_ros_task_impl();
}

void Shelfbot::micro_ros_task_impl()
{
    bool entities_created = false;

    while(1) {
        allocator = rcl_get_default_allocator();

        switch(state) {
            case WAITING_AGENT:
                ESP_LOGI(TAG, "State: WAITING_AGENT");
                if (query_mdns_host("gentoo-laptop")) {
                    ESP_LOGI(TAG, "Agent found, attempting to connect...");

                    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
                    RCCHECK_VOID(rcl_init_options_init(&init_options, allocator));
                    RCCHECK_VOID(rmw_uros_options_set_udp_address(agent_ip_str, "8888", rcl_init_options_get_rmw_init_options(&init_options)));

                    if (rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator) == RCL_RET_OK) {
                        ESP_LOGI(TAG, "Successfully connected to agent.");
                        agent_connected = true;
                        state = AGENT_CONNECTED;
                    } else {
                        ESP_LOGW(TAG, "Connection to agent failed.");
                        RCSOFTCHECK(rclc_support_fini(&support));
                    }
                    RCSOFTCHECK(rcl_init_options_fini(&init_options));
                }
                vTaskDelay(pdMS_TO_TICKS(2000));
                break;

            case AGENT_CONNECTED:
                if (!entities_created) {
                    ESP_LOGI(TAG, "State: AGENT_CONNECTED (creating entities)");
                    if (create_entities()) {
                        entities_created = true;
                        ESP_LOGI(TAG, "Entities created successfully.");
                    } else {
                        ESP_LOGE(TAG, "Failed to create entities.");
                        state = AGENT_DISCONNECTED;
                        break;
                    }
                }

                if (rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)) != RCL_RET_OK) {
                    ESP_LOGE(TAG, "Executor spin failed, disconnecting.");
                    agent_connected = false;
                }

                if (!agent_connected) {
                    state = AGENT_DISCONNECTED;
                }

                vTaskDelay(pdMS_TO_TICKS(100));
                break;

            case AGENT_DISCONNECTED:
                ESP_LOGW(TAG, "State: AGENT_DISCONNECTED.");

                if (entities_created) {
                    ESP_LOGW(TAG, "Destroying entities.");
                    destroy_entities();
                    entities_created = false;
                }

                ESP_LOGW(TAG, "Tearing down transport.");
                RCSOFTCHECK(rclc_support_fini(&support));

                state = WAITING_AGENT;
                ESP_LOGI(TAG, "Transitioning to WAITING_AGENT");
                vTaskDelay(pdMS_TO_TICKS(1000));
                break;
        }
    }
    vTaskDelete(NULL);
}

void Shelfbot::begin() {
  // Initialize peripherals
  led_control_init();
  motor_control_begin();
  sensor_control_init();

  // Initialize ToF sensor
  ESP_LOGI(TAG, "Initializing ToF sensor...");

  ToFArrayConfig tof_array_config = {
    .i2c_port = I2C_NUM_0,
    .i2c_freq_hz = 400000,
    .sda_gpio = GPIO_NUM_21,  // Fixed: Cast to gpio_num_t
    .scl_gpio = GPIO_NUM_22,  // Fixed: Cast to gpio_num_t
    .enable_pullups = true,
    .num_sensors = 1
};

  ToFSensorConfig tof_sensor_config = {
    .i2c_port = I2C_NUM_0,
    .i2c_address = 0x29,
    .xshut_gpio = GPIO_NUM_NC,  // Fixed: Use GPIO_NUM_NC instead of 255
    .int_gpio = GPIO_NUM_NC,    // Fixed: Use GPIO_NUM_NC (was GPIO_NUM_MAX)
    .range_mm = 2000,
    .timing_budget_ms = 33
};

if (!ToFSensorManager::instance().configure(tof_array_config, &tof_sensor_config, 1)) {
  ESP_LOGE(TAG, "Failed to configure ToF sensor!");
} else {
  ESP_LOGI(TAG, "ToF sensor configured successfully");
  if (!ToFSensorManager::instance().start_reading_task(100, 5)) {
      ESP_LOGE(TAG, "Failed to start ToF reading task!");
  } else {
      ESP_LOGI(TAG, "ToF reading task started");
  }
}

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
      ESP_LOGW(TAG, "Failed to synchronize time. Continuing without real time.");
   }

    start_webserver();
    initialise_mdns();

    xTaskCreate(micro_ros_task_wrapper, "micro_ros_task", 16000, this, 5, NULL);
    sensor_control_start_task();

    ESP_LOGI(TAG, "Shelfbot initialization complete. Returning to app_main.");
}
