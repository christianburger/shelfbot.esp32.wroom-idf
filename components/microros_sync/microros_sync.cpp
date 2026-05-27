#include <microros_sync.hpp>
#include <sensor_manager.hpp>
#include <motor_control.hpp>
#include <led_control.hpp>
#include <wifi_manager.hpp>
#include <firmware_version.hpp>
#include <state_machine.hpp>
#include <state_machine_lifecycle.hpp>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/float32.h>

static const char* TAG = "MicrorosSync";

#define ROS_CHECK(call, msg) do { \
    rcl_ret_t _ret = (call); \
    if (_ret != RCL_RET_OK) { \
        ESP_LOGE(TAG, "%s failed: %ld (%s)", msg, (long)_ret, rcl_get_error_string().str); \
        rcl_reset_error(); \
    } \
} while(0)

struct MicrorosSyncImpl {
    rcl_node_t node;
    rcl_allocator_t allocator;
    rclc_support_t support;
    rclc_executor_t executor;

    rcl_publisher_t heartbeat_pub;
    rcl_publisher_t motor_positions_pub;
    rcl_publisher_t distance_sensors_pub;
    rcl_publisher_t led_state_pub;
    rcl_publisher_t tof_distance_pub;
    rcl_publisher_t lidar_scan_pub;

    rcl_subscription_t motor_command_sub;
    rcl_subscription_t set_speed_sub;
    rcl_subscription_t led_sub;

    rcl_timer_t heartbeat_timer;
    rcl_timer_t motor_position_timer;
    rcl_timer_t sensor_control_timer;

    std_msgs__msg__Int32 heartbeat_msg;
    std_msgs__msg__Float32MultiArray motor_positions_msg;
    std_msgs__msg__Float32MultiArray distance_sensors_msg;
    std_msgs__msg__Bool led_state_msg;
    std_msgs__msg__Float32 tof_distance_msg;
    std_msgs__msg__Float32MultiArray lidar_scan_msg;

    std_msgs__msg__Float32MultiArray motor_command_msg;
    std_msgs__msg__Float32MultiArray set_speed_msg;
    std_msgs__msg__Bool led_msg;

    float motor_positions_data[NUM_MOTORS];
    float distance_sensors_data[SensorCommon::NUM_SENSORS];
    float lidar_scan_data[30];

    bool entities_created;
    TaskHandle_t task_handle;
    MicrorosState current_state;

    MicrorosSyncImpl()
        : node(rcl_get_zero_initialized_node()),
          allocator(rcl_get_default_allocator()),
          support(),
          executor(rclc_executor_get_zero_initialized_executor()),
          heartbeat_pub(rcl_get_zero_initialized_publisher()),
          motor_positions_pub(rcl_get_zero_initialized_publisher()),
          distance_sensors_pub(rcl_get_zero_initialized_publisher()),
          led_state_pub(rcl_get_zero_initialized_publisher()),
          tof_distance_pub(rcl_get_zero_initialized_publisher()),
          lidar_scan_pub(rcl_get_zero_initialized_publisher()),
          motor_command_sub(rcl_get_zero_initialized_subscription()),
          set_speed_sub(rcl_get_zero_initialized_subscription()),
          led_sub(rcl_get_zero_initialized_subscription()),
          heartbeat_timer(rcl_get_zero_initialized_timer()),
          motor_position_timer(rcl_get_zero_initialized_timer()),
          sensor_control_timer(rcl_get_zero_initialized_timer()),
          entities_created(false), task_handle(nullptr), current_state(MicrorosState::OFF)
    {
        std_msgs__msg__Int32__init(&heartbeat_msg);
        std_msgs__msg__Float32MultiArray__init(&motor_positions_msg);
        std_msgs__msg__Float32MultiArray__init(&distance_sensors_msg);
        std_msgs__msg__Bool__init(&led_state_msg);
        std_msgs__msg__Float32__init(&tof_distance_msg);
        std_msgs__msg__Float32MultiArray__init(&lidar_scan_msg);
        std_msgs__msg__Float32MultiArray__init(&motor_command_msg);
        std_msgs__msg__Float32MultiArray__init(&set_speed_msg);
        std_msgs__msg__Bool__init(&led_msg);

        memset(motor_positions_data, 0, sizeof(motor_positions_data));
        memset(distance_sensors_data, 0, sizeof(distance_sensors_data));
        memset(lidar_scan_data, 0, sizeof(lidar_scan_data));

        motor_positions_msg.data.data = motor_positions_data;
        motor_positions_msg.data.capacity = sizeof(motor_positions_data)/sizeof(float);
        motor_positions_msg.data.size = 0;

        distance_sensors_msg.data.data = distance_sensors_data;
        distance_sensors_msg.data.capacity = SensorCommon::NUM_SENSORS;
        distance_sensors_msg.data.size = 0;

        lidar_scan_msg.data.data = lidar_scan_data;
        lidar_scan_msg.data.capacity = 30;
        lidar_scan_msg.data.size = 0;
    }

    ~MicrorosSyncImpl() {
        if (task_handle) vTaskDelete(task_handle);
    }

    void setState(MicrorosState new_state) {
        if (current_state == new_state) return;
        const char* state_str = stateToString(new_state);
        if (StateMachine::changeState("microros_sync", state_str)) {
            current_state = new_state;
        } else {
            ESP_LOGE(TAG, "Failed to transition to state %s", state_str);
        }
    }
};

static MicrorosSyncImpl* g_impl = nullptr;

// Publishing helpers
static void publish_heartbeat() {
    if (!g_impl || !g_impl->entities_created) return;
    ROS_CHECK(rcl_publish(&g_impl->heartbeat_pub, &g_impl->heartbeat_msg, NULL), "heartbeat publish");
}
static void publish_motor_positions() {
    if (!g_impl || !g_impl->entities_created) return;
    ROS_CHECK(rcl_publish(&g_impl->motor_positions_pub, &g_impl->motor_positions_msg, NULL), "motor_positions publish");
}
static void publish_distance_sensors() {
    if (!g_impl || !g_impl->entities_created) return;
    ROS_CHECK(rcl_publish(&g_impl->distance_sensors_pub, &g_impl->distance_sensors_msg, NULL), "distance_sensors publish");
}
static void publish_led_state() {
    if (!g_impl || !g_impl->entities_created) return;
    ROS_CHECK(rcl_publish(&g_impl->led_state_pub, &g_impl->led_state_msg, NULL), "led_state publish");
}
static void publish_tof_distance() {
    if (!g_impl || !g_impl->entities_created) return;
    ROS_CHECK(rcl_publish(&g_impl->tof_distance_pub, &g_impl->tof_distance_msg, NULL), "tof_distance publish");
}
static void publish_lidar_scan() {
    if (!g_impl || !g_impl->entities_created) return;
    ROS_CHECK(rcl_publish(&g_impl->lidar_scan_pub, &g_impl->lidar_scan_msg, NULL), "lidar_scan publish");
}

static void heartbeat_timer_cb(rcl_timer_t* timer, int64_t last_call_time) {
    (void)last_call_time;
    static int32_t counter = 0;
    if (g_impl && g_impl->entities_created) {
        g_impl->heartbeat_msg.data = ++counter;
        publish_heartbeat();
    }
}

static void motor_position_timer_cb(rcl_timer_t* timer, int64_t last_call_time) {
    (void)last_call_time;
    if (g_impl && g_impl->entities_created) {
        for (uint8_t i = 0; i < NUM_MOTORS; ++i)
            g_impl->motor_positions_data[i] = static_cast<float>(motor_control_get_position(i));
        g_impl->motor_positions_msg.data.size = NUM_MOTORS;
        publish_motor_positions();
    }
}

static void sensor_control_timer_cb(rcl_timer_t* timer, int64_t last_call_time) {
    (void)last_call_time;
    if (!g_impl || !g_impl->entities_created) return;

    SensorCommon::SensorDataPacket data;
    if (!SensorManager::get_instance().get_latest_data(data)) {
        ESP_LOGW(TAG, "Failed to get latest sensor data");
        return;
    }

    float distances[SensorCommon::NUM_SENSORS];
    size_t idx = 0;
    for (int i = 0; i < SensorCommon::NUM_ULTRASONIC_SENSORS && idx < SensorCommon::NUM_SENSORS; ++i, ++idx) {
        distances[idx] = data.ultrasonic_readings[i].distance_cm;
    }
    for (int i = 0; i < SensorCommon::NUM_TOF_SENSORS && idx < SensorCommon::NUM_SENSORS; ++i, ++idx) {
        distances[idx] = data.tof_measurements[i].distance_mm / 10.0f;
    }
    if (idx < SensorCommon::NUM_SENSORS) {
        distances[idx] = data.lidar_measurement.valid ? (data.lidar_measurement.distance_mm / 10.0f) : -1.0f;
        ++idx;
    }
    MicrorosSync::publishDistanceSensors(distances, idx);

    float tof_m = data.tof_measurements[0].valid ? (data.tof_measurements[0].distance_mm / 1000.0f) : -1.0f;
    MicrorosSync::publishTofDistance(tof_m);
    MicrorosSync::publishLidarScan(data.lidar_measurement);
}

static void motor_command_cb(const void* msg) {
    auto* cmd = static_cast<const std_msgs__msg__Float32MultiArray*>(msg);
    size_t count = std::min((size_t)NUM_MOTORS, cmd->data.size);
    for (size_t i = 0; i < count; ++i) motor_control_set_position(i, cmd->data.data[i]);
}

static void set_speed_cb(const void* msg) {
    auto* speed = static_cast<const std_msgs__msg__Float32MultiArray*>(msg);
    size_t count = std::min((size_t)NUM_MOTORS, speed->data.size);
    for (size_t i = 0; i < count; ++i) motor_control_set_velocity(i, speed->data.data[i]);
}

static void led_cb(const void* msg) {
    auto* led = static_cast<const std_msgs__msg__Bool*>(msg);
    led_control_set(led->data);
    if (g_impl && g_impl->entities_created) {
        g_impl->led_state_msg.data = led->data;
        publish_led_state();
    }
}

static void create_entities(MicrorosSyncImpl& impl) {
    ESP_LOGI(TAG, "Creating micro-ROS entities");
    ROS_CHECK(rclc_node_init_default(&impl.node, "shelfbot_firmware", "", &impl.support), "node init");

    ROS_CHECK(rclc_publisher_init_default(&impl.heartbeat_pub, &impl.node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "shelfbot_firmware/heartbeat"), "heartbeat pub");
    ROS_CHECK(rclc_publisher_init_default(&impl.motor_positions_pub, &impl.node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "shelfbot_firmware/motor_positions"), "motor_positions pub");
    ROS_CHECK(rclc_publisher_init_default(&impl.distance_sensors_pub, &impl.node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "shelfbot_firmware/distance_sensors"), "distance_sensors pub");
    ROS_CHECK(rclc_publisher_init_default(&impl.led_state_pub, &impl.node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "shelfbot_firmware/led_state"), "led_state pub");
    ROS_CHECK(rclc_publisher_init_default(&impl.tof_distance_pub, &impl.node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "shelfbot_firmware/tof_distance"), "tof_distance pub");
    ROS_CHECK(rclc_publisher_init_default(&impl.lidar_scan_pub, &impl.node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "shelfbot_firmware/lidar_scan"), "lidar_scan pub");

    ROS_CHECK(rclc_subscription_init_default(&impl.motor_command_sub, &impl.node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "shelfbot_firmware/motor_command"), "motor_command sub");
    ROS_CHECK(rclc_subscription_init_default(&impl.set_speed_sub, &impl.node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "shelfbot_firmware/set_speed"), "set_speed sub");
    ROS_CHECK(rclc_subscription_init_default(&impl.led_sub, &impl.node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "shelfbot_firmware/led"), "led sub");

    ROS_CHECK(rclc_timer_init_default(&impl.heartbeat_timer, &impl.support, RCL_MS_TO_NS(1000), heartbeat_timer_cb), "heartbeat timer");
    ROS_CHECK(rclc_timer_init_default(&impl.motor_position_timer, &impl.support, RCL_MS_TO_NS(100), motor_position_timer_cb), "motor_position timer");
    ROS_CHECK(rclc_timer_init_default(&impl.sensor_control_timer, &impl.support, RCL_MS_TO_NS(200), sensor_control_timer_cb), "sensor_control timer");

    unsigned int num_handles = 3 + 3;
    ROS_CHECK(rclc_executor_init(&impl.executor, &impl.support.context, num_handles, &impl.allocator), "executor init");
    ROS_CHECK(rclc_executor_add_timer(&impl.executor, &impl.heartbeat_timer), "add heartbeat timer");
    ROS_CHECK(rclc_executor_add_timer(&impl.executor, &impl.motor_position_timer), "add motor_position timer");
    ROS_CHECK(rclc_executor_add_timer(&impl.executor, &impl.sensor_control_timer), "add sensor_control timer");
    ROS_CHECK(rclc_executor_add_subscription(&impl.executor, &impl.motor_command_sub, &impl.motor_command_msg, motor_command_cb, ON_NEW_DATA), "add motor_command sub");
    ROS_CHECK(rclc_executor_add_subscription(&impl.executor, &impl.set_speed_sub, &impl.set_speed_msg, set_speed_cb, ON_NEW_DATA), "add set_speed sub");
    ROS_CHECK(rclc_executor_add_subscription(&impl.executor, &impl.led_sub, &impl.led_msg, led_cb, ON_NEW_DATA), "add led sub");

    ESP_LOGI(TAG, "Entities created");
}

static void destroy_entities(MicrorosSyncImpl& impl) {
    ESP_LOGI(TAG, "Destroying entities");
    ROS_CHECK(rcl_publisher_fini(&impl.heartbeat_pub, &impl.node), "heartbeat pub fini");
    ROS_CHECK(rcl_publisher_fini(&impl.motor_positions_pub, &impl.node), "motor_positions pub fini");
    ROS_CHECK(rcl_publisher_fini(&impl.distance_sensors_pub, &impl.node), "distance_sensors pub fini");
    ROS_CHECK(rcl_publisher_fini(&impl.led_state_pub, &impl.node), "led_state pub fini");
    ROS_CHECK(rcl_publisher_fini(&impl.tof_distance_pub, &impl.node), "tof_distance pub fini");
    ROS_CHECK(rcl_publisher_fini(&impl.lidar_scan_pub, &impl.node), "lidar_scan pub fini");
    ROS_CHECK(rcl_subscription_fini(&impl.motor_command_sub, &impl.node), "motor_command sub fini");
    ROS_CHECK(rcl_subscription_fini(&impl.set_speed_sub, &impl.node), "set_speed sub fini");
    ROS_CHECK(rcl_subscription_fini(&impl.led_sub, &impl.node), "led sub fini");
    ROS_CHECK(rcl_timer_fini(&impl.heartbeat_timer), "heartbeat timer fini");
    ROS_CHECK(rcl_timer_fini(&impl.motor_position_timer), "motor_position timer fini");
    ROS_CHECK(rcl_timer_fini(&impl.sensor_control_timer), "sensor_control timer fini");
    ROS_CHECK(rclc_executor_fini(&impl.executor), "executor fini");
    ROS_CHECK(rcl_node_fini(&impl.node), "node fini");
    ROS_CHECK(rclc_support_fini(&impl.support), "support fini");
}

static bool query_mdns_host(const char* host_name, char* out_ip, size_t len) {
    ESP_LOGI(TAG, "Querying mDNS for %s.local", host_name);
    esp_ip4_addr_t addr;
    addr.addr = 0;
    esp_err_t err = mdns_query_a(host_name, 2000, &addr);
    if (err != ESP_OK) {
        if (err == ESP_ERR_NOT_FOUND) ESP_LOGW(TAG, "Host not found");
        else ESP_LOGE(TAG, "mDNS query failed: %d", err);
        return false;
    }
    snprintf(out_ip, len, IPSTR, IP2STR(&addr));
    ESP_LOGI(TAG, "Agent IP: %s", out_ip);
    return true;
}

static void microros_task(void* arg) {
    auto* impl = static_cast<MicrorosSyncImpl*>(arg);
    if (!impl) return;

    enum class TaskState {
        WAITING_WIFI,
        DISCOVER_AGENT,
        INITIALIZING,
        CONNECTED,
        BACKING_OFF
    } state = TaskState::WAITING_WIFI;

    uint32_t backoff_ms = 250;
    const uint32_t MAX_BACKOFF = 5000;
    uint8_t consecutive_spin_failures = 0;
    char agent_ip[16];
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();

    while (1) {
        switch (state) {
            case TaskState::WAITING_WIFI: {
                EventGroupHandle_t wifi_evt = wifi_manager_get_event_group();
                ESP_LOGI(TAG, "Waiting for Wi-Fi...");
                xEventGroupWaitBits(wifi_evt, WM_CONNECTED_BIT, pdFALSE, pdFALSE, portMAX_DELAY);
                ESP_LOGI(TAG, "Wi-Fi ready");
                mdns_init();
                mdns_hostname_set("shelfbot");
                mdns_instance_name_set("Shelfbot ESP32 Client");
                impl->setState(MicrorosState::DISCOVERING);
                state = TaskState::DISCOVER_AGENT;
                break;
            }

            case TaskState::DISCOVER_AGENT: {
                if (query_mdns_host(CONFIG_MICROROS_AGENT_MDNS_HOST, agent_ip, sizeof(agent_ip))) {
                    state = TaskState::INITIALIZING;
                } else {
                    vTaskDelay(pdMS_TO_TICKS(100));
                }
                break;
            }

            case TaskState::INITIALIZING: {
                init_options = rcl_get_zero_initialized_init_options();
                rcl_ret_t ret = rcl_init_options_init(&init_options, impl->allocator);
                if (ret != RCL_RET_OK) {
                    ESP_LOGE(TAG, "rcl_init_options_init failed: %ld (%s)", (long)ret, rcl_get_error_string().str);
                    rcl_reset_error();
                    state = TaskState::BACKING_OFF;
                    break;
                }
                rmw_uros_options_set_udp_address(agent_ip, "8888",
                    rcl_init_options_get_rmw_init_options(&init_options));

                if (rclc_support_init_with_options(&impl->support, 0, NULL, &init_options, &impl->allocator) == RCL_RET_OK) {
                    create_entities(*impl);
                    impl->entities_created = true;
                    consecutive_spin_failures = 0;
                    backoff_ms = 250;
                    impl->setState(MicrorosState::CONNECTED);
                    state = TaskState::CONNECTED;
                } else {
                    ESP_LOGE(TAG, "rclc_support_init_with_options failed");
                    rcl_reset_error();
                    state = TaskState::BACKING_OFF;
                }
                ROS_CHECK(rcl_init_options_fini(&init_options), "init_options fini");
                break;
            }

            case TaskState::CONNECTED: {
                rcl_ret_t spin_ret = rclc_executor_spin_some(&impl->executor, RCL_MS_TO_NS(100));
                if (spin_ret != RCL_RET_OK) {
                    if (++consecutive_spin_failures >= 3) {
                        ESP_LOGW(TAG, "3 consecutive spin failures, disconnecting...");
                        destroy_entities(*impl);
                        impl->entities_created = false;
                        impl->setState(MicrorosState::DISCONNECTED);
                        state = TaskState::BACKING_OFF;
                    }
                } else {
                    consecutive_spin_failures = 0;
                }
                vTaskDelay(pdMS_TO_TICKS(10));
                break;
            }

            case TaskState::BACKING_OFF: {
                ESP_LOGW(TAG, "Backing off for %lu ms", (unsigned long)backoff_ms);
                vTaskDelay(pdMS_TO_TICKS(backoff_ms));
                backoff_ms = std::min(MAX_BACKOFF, backoff_ms * 2);
                impl->setState(MicrorosState::DISCOVERING);
                state = TaskState::DISCOVER_AGENT;
                break;
            }
        }
    }
}

MicrorosSync::MicrorosSync() { g_impl = new MicrorosSyncImpl(); }
MicrorosSync::~MicrorosSync() { delete g_impl; g_impl = nullptr; }

MicrorosSync& MicrorosSync::getInstance() {
    static MicrorosSync instance;
    return instance;
}

bool MicrorosSync::init() {
    ESP_LOGI(TAG, "MicrorosSync initialised");
    StateMachine::setInitial("microros_sync", stateToString(MicrorosState::OFF));
    if (g_impl) g_impl->setState(MicrorosState::OFF);
    return true;
}

void MicrorosSync::start() {
    if (!g_impl || g_impl->task_handle) return;
    xTaskCreate(microros_task, "microros_task", 16000, g_impl, 5, &g_impl->task_handle);
    ESP_LOGI(TAG, "MicrorosSync task started");
}

void MicrorosSync::publishHeartbeat(int32_t value) {
    if (g_impl && g_impl->entities_created) {
        g_impl->heartbeat_msg.data = value;
        publish_heartbeat();
    }
}

void MicrorosSync::publishMotorPositions(const float* positions, size_t count) {
    if (!g_impl || !g_impl->entities_created) return;
    size_t copy = std::min(count, sizeof(g_impl->motor_positions_data)/sizeof(float));
    memcpy(g_impl->motor_positions_data, positions, copy * sizeof(float));
    g_impl->motor_positions_msg.data.size = copy;
    publish_motor_positions();
}

void MicrorosSync::publishDistanceSensors(const float* distances, size_t count) {
    if (!g_impl || !g_impl->entities_created) return;
    size_t copy = std::min(count, (size_t)SensorCommon::NUM_SENSORS);
    memcpy(g_impl->distance_sensors_data, distances, copy * sizeof(float));
    g_impl->distance_sensors_msg.data.size = copy;
    publish_distance_sensors();
}

void MicrorosSync::publishLedState(bool state) {
    if (g_impl && g_impl->entities_created) {
        g_impl->led_state_msg.data = state;
        publish_led_state();
    }
}

void MicrorosSync::publishTofDistance(float distance_m) {
    if (g_impl && g_impl->entities_created) {
        g_impl->tof_distance_msg.data = distance_m;
        publish_tof_distance();
    }
}

void MicrorosSync::publishLidarScan(const SensorCommon::LidarMeasurement& m) {
    if (!g_impl || !g_impl->entities_created) return;
    g_impl->lidar_scan_data[0] = m.start_angle_deg;
    g_impl->lidar_scan_data[1] = m.end_angle_deg;
    g_impl->lidar_scan_data[2] = m.min_distance_angle_deg;
    g_impl->lidar_scan_data[3] = static_cast<float>(m.distance_mm);
    g_impl->lidar_scan_data[4] = static_cast<float>(m.rotational_speed_rpm);   // renamed
    for (int i = 0; i < 12; ++i) {
        g_impl->lidar_scan_data[5 + i] = static_cast<float>(m.packet_distances_mm[i]);
        g_impl->lidar_scan_data[17 + i] = static_cast<float>(m.packet_confidences[i]);
    }
    g_impl->lidar_scan_data[29] = (m.valid && m.has_packet_points) ? 1.0f : 0.0f;
    g_impl->lidar_scan_msg.data.size = 30;
    publish_lidar_scan();
}