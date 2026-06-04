#include <microros_sync.hpp>
#include <sensor_manager.hpp>
#include <motor_control.hpp>
#include <led_control.hpp>
#include <wifi_manager.hpp>
#include <firmware_version.hpp>
#include <state_machine.hpp>
#include <state_machine_lifecycle.hpp>
#include <shelfbot_timestamp.hpp>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/float32.h>
#include <sensor_msgs/msg/laser_scan.h>
#include <builtin_interfaces/msg/time.h>

static const char* TAG = "MicrorosSync";

#define ROS_CHECK(call, msg) do { \
    rcl_ret_t _ret = (call); \
    if (_ret != RCL_RET_OK) { \
        ESP_LOGE(TAG, "%s failed: %ld (%s)", msg, (long)_ret, \
                 rcl_get_error_string().str); \
        rcl_reset_error(); \
    } \
} while(0)

// ---------------------------------------------------------------------------
// Time-sync configuration
// ---------------------------------------------------------------------------
static constexpr int32_t  TIME_SYNC_TIMEOUT_MS  = 5000;
static constexpr uint8_t  TIME_SYNC_MAX_ATTEMPTS = 3;
static constexpr uint32_t EPOCH_WAIT_TIMEOUT_MS  = 30000;
static constexpr uint32_t EPOCH_POLL_MS          = 200;

// ---------------------------------------------------------------------------
// MicrorosSyncImpl
// ---------------------------------------------------------------------------
struct MicrorosSyncImpl {
    rcl_node_t          node;
    rcl_allocator_t     allocator;
    rclc_support_t      support;
    rclc_executor_t     executor;

    rcl_publisher_t heartbeat_pub;
    rcl_publisher_t motor_positions_pub;
    rcl_publisher_t distance_sensors_pub;
    rcl_publisher_t led_state_pub;
    rcl_publisher_t tof_distance_pub;
    rcl_publisher_t laser_scan_pub;

    rcl_subscription_t motor_command_sub;
    rcl_subscription_t set_speed_sub;
    rcl_subscription_t led_sub;

    rcl_timer_t heartbeat_timer;
    rcl_timer_t motor_position_timer;
    rcl_timer_t sensor_control_timer;

    std_msgs__msg__Int32               heartbeat_msg;
    std_msgs__msg__Float32MultiArray   motor_positions_msg;
    std_msgs__msg__Float32MultiArray   distance_sensors_msg;
    std_msgs__msg__Bool                led_state_msg;
    std_msgs__msg__Float32             tof_distance_msg;
    sensor_msgs__msg__LaserScan        laser_scan_msg;

    std_msgs__msg__Float32MultiArray   motor_command_msg;
    std_msgs__msg__Float32MultiArray   set_speed_msg;
    std_msgs__msg__Bool                led_msg;

    float motor_cmd_data[NUM_MOTORS]                       = {};
    float set_speed_data[NUM_MOTORS]                       = {};
    float motor_positions_data[NUM_MOTORS]                 = {};
    float distance_sensors_data[SensorCommon::NUM_SENSORS] = {};
    float ranges_data[12]                                  = {};
    float intensities_data[12]                             = {};

    static std_msgs__msg__MultiArrayDimension motor_dim[1];
    static std_msgs__msg__MultiArrayDimension distance_dim[1];

    bool              entities_created = false;
    bool              time_synced      = false;
    TaskHandle_t      task_handle      = nullptr;
    MicrorosState     current_state    = MicrorosState::OFF;
    SemaphoreHandle_t mutex            = nullptr;

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
          laser_scan_pub(rcl_get_zero_initialized_publisher()),
          motor_command_sub(rcl_get_zero_initialized_subscription()),
          set_speed_sub(rcl_get_zero_initialized_subscription()),
          led_sub(rcl_get_zero_initialized_subscription()),
          heartbeat_timer(rcl_get_zero_initialized_timer()),
          motor_position_timer(rcl_get_zero_initialized_timer()),
          sensor_control_timer(rcl_get_zero_initialized_timer())
    {
        std_msgs__msg__Int32__init(&heartbeat_msg);
        std_msgs__msg__Float32MultiArray__init(&motor_positions_msg);
        std_msgs__msg__Float32MultiArray__init(&distance_sensors_msg);
        std_msgs__msg__Bool__init(&led_state_msg);
        std_msgs__msg__Float32__init(&tof_distance_msg);
        sensor_msgs__msg__LaserScan__init(&laser_scan_msg);
        std_msgs__msg__Float32MultiArray__init(&motor_command_msg);
        std_msgs__msg__Float32MultiArray__init(&set_speed_msg);
        std_msgs__msg__Bool__init(&led_msg);

        motor_dim[0].label.data     = const_cast<char*>("");
        motor_dim[0].label.size     = 0;
        motor_dim[0].label.capacity = 1;
        motor_dim[0].size           = NUM_MOTORS;
        motor_dim[0].stride         = NUM_MOTORS;
        motor_positions_msg.layout.dim.data   = motor_dim;
        motor_positions_msg.layout.dim.size   = 1;
        motor_positions_msg.layout.data_offset = 0;

        distance_dim[0].label.data     = const_cast<char*>("");
        distance_dim[0].label.size     = 0;
        distance_dim[0].label.capacity = 1;
        distance_dim[0].size           = SensorCommon::NUM_SENSORS;
        distance_dim[0].stride         = SensorCommon::NUM_SENSORS;
        distance_sensors_msg.layout.dim.data   = distance_dim;
        distance_sensors_msg.layout.dim.size   = 1;
        distance_sensors_msg.layout.data_offset = 0;

        motor_positions_msg.data.data     = motor_positions_data;
        motor_positions_msg.data.capacity = NUM_MOTORS;
        motor_positions_msg.data.size     = 0;

        distance_sensors_msg.data.data     = distance_sensors_data;
        distance_sensors_msg.data.capacity = SensorCommon::NUM_SENSORS;
        distance_sensors_msg.data.size     = 0;

        motor_command_msg.data.data     = motor_cmd_data;
        motor_command_msg.data.capacity = NUM_MOTORS;
        set_speed_msg.data.data         = set_speed_data;
        set_speed_msg.data.capacity     = NUM_MOTORS;

        laser_scan_msg.ranges.data          = ranges_data;
        laser_scan_msg.ranges.capacity      = 12;
        laser_scan_msg.ranges.size          = 0;
        laser_scan_msg.intensities.data     = intensities_data;
        laser_scan_msg.intensities.capacity = 12;
        laser_scan_msg.intensities.size     = 0;

        laser_scan_msg.range_min      = 0.02f;
        laser_scan_msg.range_max      = 12.0f;
        laser_scan_msg.time_increment = 0.0f;
        laser_scan_msg.scan_time      = 0.1f;

        static char frame_id_str[] = "lidar_frame";
        laser_scan_msg.header.frame_id.data     = frame_id_str;
        laser_scan_msg.header.frame_id.size     = sizeof(frame_id_str) - 1;
        laser_scan_msg.header.frame_id.capacity = sizeof(frame_id_str);

        mutex = xSemaphoreCreateMutex();
    }

    ~MicrorosSyncImpl() {
        if (mutex)       vSemaphoreDelete(mutex);
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

    void fillStamp(int32_t& sec_out, uint32_t& nanosec_out) const {
        if (time_synced) {
            shelfbot::ShelfbotTimestamp::toRosTime(
                shelfbot::ShelfbotTimestamp::epochMicros(),
                sec_out, nanosec_out);
        } else {
            const int64_t mono_us = shelfbot::ShelfbotTimestamp::monotonicMicros();
            sec_out     = static_cast<int32_t>(mono_us / 1000000LL);
            nanosec_out = static_cast<uint32_t>((mono_us % 1000000LL) * 1000LL);
        }
    }
};

std_msgs__msg__MultiArrayDimension MicrorosSyncImpl::motor_dim[1];
std_msgs__msg__MultiArrayDimension MicrorosSyncImpl::distance_dim[1];

static MicrorosSyncImpl* g_impl = nullptr;

static bool lock_impl() {
    return g_impl && xSemaphoreTake(g_impl->mutex, pdMS_TO_TICKS(100)) == pdTRUE;
}
static void unlock_impl() {
    if (g_impl) xSemaphoreGive(g_impl->mutex);
}

// ---------------------------------------------------------------------------
// Unlocked publish helpers
// ---------------------------------------------------------------------------
static void _pub_heartbeat()        { ROS_CHECK(rcl_publish(&g_impl->heartbeat_pub,        &g_impl->heartbeat_msg,        NULL), "heartbeat publish"); }
static void _pub_motor_positions()  { ROS_CHECK(rcl_publish(&g_impl->motor_positions_pub,  &g_impl->motor_positions_msg,  NULL), "motor_positions publish"); }
static void _pub_distance_sensors() { ROS_CHECK(rcl_publish(&g_impl->distance_sensors_pub, &g_impl->distance_sensors_msg, NULL), "distance_sensors publish"); }
static void _pub_led_state()        { ROS_CHECK(rcl_publish(&g_impl->led_state_pub,        &g_impl->led_state_msg,        NULL), "led_state publish"); }
static void _pub_tof_distance()     { ROS_CHECK(rcl_publish(&g_impl->tof_distance_pub,     &g_impl->tof_distance_msg,     NULL), "tof_distance publish"); }
static void _pub_laser_scan()       { ROS_CHECK(rcl_publish(&g_impl->laser_scan_pub,       &g_impl->laser_scan_msg,       NULL), "laser_scan publish"); }

// ---------------------------------------------------------------------------
// Timer callbacks
// ---------------------------------------------------------------------------
static void heartbeat_timer_cb(rcl_timer_t* /*timer*/, int64_t /*last_call_time*/) {
    static int32_t counter = 0;
    if (!lock_impl()) return;
    if (g_impl->entities_created) {
        g_impl->heartbeat_msg.data = ++counter;
        _pub_heartbeat();
    }
    unlock_impl();
}

static void motor_position_timer_cb(rcl_timer_t* /*timer*/, int64_t /*last_call_time*/) {
    if (!lock_impl()) return;
    if (g_impl->entities_created && g_impl->time_synced) {
        for (uint8_t i = 0; i < NUM_MOTORS; ++i)
            g_impl->motor_positions_data[i] =
                static_cast<float>(motor_control_get_position(i));
        g_impl->motor_positions_msg.data.size = NUM_MOTORS;
        _pub_motor_positions();
    }
    unlock_impl();
}

static void sensor_control_timer_cb(rcl_timer_t* /*timer*/, int64_t /*last_call_time*/) {
    if (!lock_impl()) return;
    if (!g_impl->entities_created || !g_impl->time_synced) {
        static uint32_t skip_count = 0;
        if ((++skip_count & 0x1F) == 1) {
            ESP_LOGW(TAG, "sensor_control_timer: waiting for time sync (%lu skips)",
                     (unsigned long)skip_count);
        }
        unlock_impl();
        return;
    }

    SensorCommon::SensorDataPacket data;
    if (!SensorManager::get_instance().get_latest_data(data)) {
        ESP_LOGW(TAG, "Failed to get latest sensor data");
        unlock_impl();
        return;
    }

    // Distance array
    float distances[SensorCommon::NUM_SENSORS];
    size_t idx = 0;
    for (int i = 0; i < SensorCommon::NUM_ULTRASONIC_SENSORS && idx < (size_t)SensorCommon::NUM_SENSORS; ++i, ++idx)
        distances[idx] = data.ultrasonic_readings[i].distance_cm;
    for (int i = 0; i < SensorCommon::NUM_TOF_SENSORS && idx < (size_t)SensorCommon::NUM_SENSORS; ++i, ++idx)
        distances[idx] = data.tof_measurements[i].distance_mm / 10.0f;
    if (idx < (size_t)SensorCommon::NUM_SENSORS) {
        distances[idx] = data.lidar_measurement.valid
            ? (data.lidar_measurement.distance_mm / 10.0f) : -1.0f;
        ++idx;
    }
    const size_t copy = std::min(idx, (size_t)SensorCommon::NUM_SENSORS);
    for (size_t i = 0; i < copy; ++i)
        g_impl->distance_sensors_data[i] = distances[i];
    g_impl->distance_sensors_msg.data.size = copy;
    _pub_distance_sensors();

    // ToF scalar
    g_impl->tof_distance_msg.data = data.tof_measurements[0].valid
        ? (data.tof_measurements[0].distance_mm / 1000.0f) : -1.0f;
    _pub_tof_distance();

    // LaserScan
    const auto& m = data.lidar_measurement;
    if (m.valid && m.has_packet_points) {
        const float start_rad = m.start_angle_deg * static_cast<float>(M_PI) / 180.0f;
        const float end_rad   = m.end_angle_deg   * static_cast<float>(M_PI) / 180.0f;
        g_impl->laser_scan_msg.angle_min       = start_rad;
        g_impl->laser_scan_msg.angle_max       = end_rad;
        g_impl->laser_scan_msg.angle_increment = (end_rad - start_rad) / 11.0f;
        g_impl->fillStamp(
            g_impl->laser_scan_msg.header.stamp.sec,
            g_impl->laser_scan_msg.header.stamp.nanosec);
        for (int i = 0; i < 12; ++i) {
            const uint16_t dist_mm = m.packet_distances_mm[i];
            g_impl->ranges_data[i] = (dist_mm == 0 || dist_mm > 12000)
                ? (g_impl->laser_scan_msg.range_max + 1.0f)
                : (dist_mm / 1000.0f);
            g_impl->intensities_data[i] = static_cast<float>(m.packet_confidences[i]);
        }
        g_impl->laser_scan_msg.ranges.size      = 12;
        g_impl->laser_scan_msg.intensities.size = 12;
        _pub_laser_scan();
    } else {
        static uint32_t skip_counter = 0;
        if (skip_counter++ % 100 == 0)
            ESP_LOGW(TAG, "No valid LiDAR packet with points, skipping LaserScan publish");
    }

    unlock_impl();
}

// ---------------------------------------------------------------------------
// Subscription callbacks
// ---------------------------------------------------------------------------
static void motor_command_cb(const void* msg) {
    const auto* cmd    = static_cast<const std_msgs__msg__Float32MultiArray*>(msg);
    const size_t count = std::min((size_t)NUM_MOTORS, cmd->data.size);
    for (size_t i = 0; i < count; ++i)
        motor_control_set_position(i, cmd->data.data[i]);
}

static void set_speed_cb(const void* msg) {
    const auto* speed  = static_cast<const std_msgs__msg__Float32MultiArray*>(msg);
    const size_t count = std::min((size_t)NUM_MOTORS, speed->data.size);
    for (size_t i = 0; i < count; ++i)
        motor_control_set_velocity(i, speed->data.data[i]);
}

static void led_cb(const void* msg) {
    const auto* led = static_cast<const std_msgs__msg__Bool*>(msg);
    led_control_set(led->data);
    if (lock_impl()) {
        if (g_impl->entities_created) {
            g_impl->led_state_msg.data = led->data;
            _pub_led_state();
        }
        unlock_impl();
    }
}

// ---------------------------------------------------------------------------
// Entity creation
//
// QoS assignment rationale (verified against live ros2 topic info --verbose):
//
//   BEST_EFFORT (rclc_publisher_init_best_effort):
//     heartbeat        — no subscriber; diagnostic counter, loss acceptable
//     motor_positions  — shelfbot_hardware_interface_microros_node subscribes
//                        BEST_EFFORT; MUST match or data is silently dropped
//     distance_sensors — no subscriber; sensor data, loss acceptable
//     led_state        — no subscriber; feedback, loss acceptable
//     tof_distance     — no subscriber; sensor data, loss acceptable
//
//   RELIABLE (rclc_publisher_init_default):
//     laser_scan       — lidar_relay_node subscribes RELIABLE/VOLATILE;
//                        a BEST_EFFORT publisher with a RELIABLE subscriber is
//                        incompatible under DDS — data would be silently dropped.
//                        Must stay RELIABLE to match the existing subscriber.
//
//   Subscriptions (rclc_subscription_init_default = RELIABLE):
//     motor_command    — published by hardware_interface at RELIABLE ✓
//     set_speed        — published by hardware_interface at RELIABLE ✓
//     led              — published at RELIABLE ✓
// ---------------------------------------------------------------------------
static void create_entities(MicrorosSyncImpl& impl) {
    ESP_LOGI(TAG, "Creating micro-ROS entities");
    ROS_CHECK(rclc_node_init_default(&impl.node, "shelfbot_firmware", "", &impl.support), "node init");

    // ---- BEST_EFFORT publishers ----
    ROS_CHECK(rclc_publisher_init_best_effort(&impl.heartbeat_pub, &impl.node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "shelfbot_firmware/heartbeat"), "heartbeat pub");

    ROS_CHECK(rclc_publisher_init_best_effort(&impl.motor_positions_pub, &impl.node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "shelfbot_firmware/motor_positions"), "motor_positions pub");

    ROS_CHECK(rclc_publisher_init_best_effort(&impl.distance_sensors_pub, &impl.node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "shelfbot_firmware/distance_sensors"), "distance_sensors pub");

    ROS_CHECK(rclc_publisher_init_best_effort(&impl.led_state_pub, &impl.node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
        "shelfbot_firmware/led_state"), "led_state pub");

    ROS_CHECK(rclc_publisher_init_best_effort(&impl.tof_distance_pub, &impl.node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "shelfbot_firmware/tof_distance"), "tof_distance pub");

    // ---- RELIABLE publisher (must match lidar_relay_node subscriber) ----
    ROS_CHECK(rclc_publisher_init_default(&impl.laser_scan_pub, &impl.node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan),
        "shelfbot_firmware/laser_scan"), "laser_scan pub");

    // ---- RELIABLE subscriptions (hardware_interface publishes RELIABLE) ----
    ROS_CHECK(rclc_subscription_init_default(&impl.motor_command_sub, &impl.node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "shelfbot_firmware/motor_command"), "motor_command sub");

    ROS_CHECK(rclc_subscription_init_default(&impl.set_speed_sub, &impl.node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "shelfbot_firmware/set_speed"), "set_speed sub");

    ROS_CHECK(rclc_subscription_init_default(&impl.led_sub, &impl.node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
        "shelfbot_firmware/led"), "led sub");

    // ---- timers ----
    ROS_CHECK(rclc_timer_init_default(&impl.heartbeat_timer,      &impl.support, RCL_MS_TO_NS(1000), heartbeat_timer_cb),      "heartbeat timer");
    ROS_CHECK(rclc_timer_init_default(&impl.motor_position_timer, &impl.support, RCL_MS_TO_NS(100),  motor_position_timer_cb), "motor_position timer");
    ROS_CHECK(rclc_timer_init_default(&impl.sensor_control_timer, &impl.support, RCL_MS_TO_NS(200),  sensor_control_timer_cb), "sensor_control timer");

    // ---- executor: 3 timers + 3 subscriptions = 6 handles ----
    ROS_CHECK(rclc_executor_init(&impl.executor, &impl.support.context, 6, &impl.allocator), "executor init");
    ROS_CHECK(rclc_executor_add_timer(&impl.executor, &impl.heartbeat_timer),      "add heartbeat timer");
    ROS_CHECK(rclc_executor_add_timer(&impl.executor, &impl.motor_position_timer), "add motor_position timer");
    ROS_CHECK(rclc_executor_add_timer(&impl.executor, &impl.sensor_control_timer), "add sensor_control timer");
    ROS_CHECK(rclc_executor_add_subscription(&impl.executor, &impl.motor_command_sub, &impl.motor_command_msg, motor_command_cb, ON_NEW_DATA), "add motor_command sub");
    ROS_CHECK(rclc_executor_add_subscription(&impl.executor, &impl.set_speed_sub,     &impl.set_speed_msg,     set_speed_cb,     ON_NEW_DATA), "add set_speed sub");
    ROS_CHECK(rclc_executor_add_subscription(&impl.executor, &impl.led_sub,           &impl.led_msg,           led_cb,           ON_NEW_DATA), "add led sub");

    ESP_LOGI(TAG, "Entities created OK");
}

static void destroy_entities(MicrorosSyncImpl& impl) {
    ESP_LOGI(TAG, "Destroying micro-ROS entities");
    ROS_CHECK(rcl_publisher_fini(&impl.heartbeat_pub,        &impl.node), "heartbeat pub fini");
    ROS_CHECK(rcl_publisher_fini(&impl.motor_positions_pub,  &impl.node), "motor_positions pub fini");
    ROS_CHECK(rcl_publisher_fini(&impl.distance_sensors_pub, &impl.node), "distance_sensors pub fini");
    ROS_CHECK(rcl_publisher_fini(&impl.led_state_pub,        &impl.node), "led_state pub fini");
    ROS_CHECK(rcl_publisher_fini(&impl.tof_distance_pub,     &impl.node), "tof_distance pub fini");
    ROS_CHECK(rcl_publisher_fini(&impl.laser_scan_pub,       &impl.node), "laser_scan pub fini");
    ROS_CHECK(rcl_subscription_fini(&impl.motor_command_sub, &impl.node), "motor_command sub fini");
    ROS_CHECK(rcl_subscription_fini(&impl.set_speed_sub,     &impl.node), "set_speed sub fini");
    ROS_CHECK(rcl_subscription_fini(&impl.led_sub,           &impl.node), "led sub fini");
    ROS_CHECK(rcl_timer_fini(&impl.heartbeat_timer),      "heartbeat timer fini");
    ROS_CHECK(rcl_timer_fini(&impl.motor_position_timer), "motor_position timer fini");
    ROS_CHECK(rcl_timer_fini(&impl.sensor_control_timer), "sensor_control timer fini");
    ROS_CHECK(rclc_executor_fini(&impl.executor), "executor fini");
    ROS_CHECK(rcl_node_fini(&impl.node),          "node fini");
    ROS_CHECK(rclc_support_fini(&impl.support),   "support fini");
}

// ---------------------------------------------------------------------------
// mDNS helper
// ---------------------------------------------------------------------------
static bool query_mdns_host(const char* host_name, char* out_ip, size_t len) {
    ESP_LOGI(TAG, "Querying mDNS for %s.local", host_name);
    esp_ip4_addr_t addr;
    addr.addr = 0;
    const esp_err_t err = mdns_query_a(host_name, 2000, &addr);
    if (err != ESP_OK) {
        if (err == ESP_ERR_NOT_FOUND) ESP_LOGW(TAG, "Host not found");
        else                          ESP_LOGE(TAG, "mDNS query failed: %d", err);
        return false;
    }
    snprintf(out_ip, len, IPSTR, IP2STR(&addr));
    ESP_LOGI(TAG, "Agent IP: %s", out_ip);
    return true;
}

// ---------------------------------------------------------------------------
// Clock synchronisation (Path A: agent, Path B: SNTP)
// ---------------------------------------------------------------------------
static bool try_agent_sync() {
    const rmw_ret_t ping = rmw_uros_ping_agent(200, 20);
    if (ping != RMW_RET_OK) {
        ESP_LOGW(TAG, "Agent ping failed — skipping rmw time sync");
        return false;
    }
    ESP_LOGI(TAG, "Agent ping OK — attempting rmw_uros_sync_session");
    for (uint8_t i = 1; i <= TIME_SYNC_MAX_ATTEMPTS; ++i) {
        const rmw_ret_t ret = rmw_uros_sync_session(TIME_SYNC_TIMEOUT_MS);
        if (ret != RMW_RET_OK) {
            ESP_LOGW(TAG, "rmw_uros_sync_session attempt %u/%u failed: %d",
                     i, TIME_SYNC_MAX_ATTEMPTS, (int)ret);
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }
        if (shelfbot::ShelfbotTimestamp::isEpochValid()) {
            int32_t sec; uint32_t ns;
            shelfbot::ShelfbotTimestamp::toRosTime(
                shelfbot::ShelfbotTimestamp::epochMicros(), sec, ns);
            ESP_LOGI(TAG, "Clock synced via micro-ROS agent — wall time %ld.%09lu",
                     (long)sec, (unsigned long)ns);
            return true;
        }
        ESP_LOGW(TAG, "rmw_uros_sync_session OK but epoch still invalid (raw=%" PRId64 " us) "
                      "— agent may need --time flag",
                 shelfbot::ShelfbotTimestamp::epochMicros());
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    return false;
}

static bool sync_time() {
    ESP_LOGI(TAG, "Starting clock sync (agent first, then SNTP fallback)");
    if (try_agent_sync()) return true;
    ESP_LOGI(TAG, "Agent sync did not produce valid epoch — polling for SNTP (timeout %lu ms)",
             (unsigned long)EPOCH_WAIT_TIMEOUT_MS);
    const TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(EPOCH_WAIT_TIMEOUT_MS);
    uint32_t polls = 0;
    while (xTaskGetTickCount() < deadline) {
        if (shelfbot::ShelfbotTimestamp::isEpochValid()) {
            int32_t sec; uint32_t ns;
            shelfbot::ShelfbotTimestamp::toRosTime(
                shelfbot::ShelfbotTimestamp::epochMicros(), sec, ns);
            ESP_LOGI(TAG, "Clock synced via SNTP after %lu polls — wall time %ld.%09lu",
                     (unsigned long)polls, (long)sec, (unsigned long)ns);
            return true;
        }
        if (++polls % 10 == 0)
            ESP_LOGI(TAG, "Waiting for SNTP... (%lu ms elapsed)",
                     (unsigned long)(polls * EPOCH_POLL_MS));
        vTaskDelay(pdMS_TO_TICKS(EPOCH_POLL_MS));
    }
    ESP_LOGE(TAG, "Clock sync timed out — will use monotonic timestamps");
    return false;
}

// ---------------------------------------------------------------------------
// Main micro-ROS task
// ---------------------------------------------------------------------------
static void microros_task(void* arg) {
    auto* impl = static_cast<MicrorosSyncImpl*>(arg);
    if (!impl) { vTaskDelete(nullptr); return; }

    enum class TaskState {
        WAITING_WIFI, DISCOVER_AGENT, INITIALIZING, TIME_SYNCING, CONNECTED, BACKING_OFF
    } state = TaskState::WAITING_WIFI;

    uint32_t backoff_ms = 250;
    constexpr uint32_t MAX_BACKOFF = 5000;
    uint8_t consecutive_spin_failures = 0;
    char agent_ip[16] = {};
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();

    while (true) {
        switch (state) {

            case TaskState::WAITING_WIFI: {
                ESP_LOGI(TAG, "Waiting for Wi-Fi...");
                xEventGroupWaitBits(wifi_manager_get_event_group(),
                                    WM_CONNECTED_BIT, pdFALSE, pdFALSE, portMAX_DELAY);
                ESP_LOGI(TAG, "Wi-Fi ready");
                if (lock_impl()) { impl->setState(MicrorosState::DISCOVERING); unlock_impl(); }
                state = TaskState::DISCOVER_AGENT;
                break;
            }

            case TaskState::DISCOVER_AGENT: {
                if (query_mdns_host(CONFIG_MICROROS_AGENT_MDNS_HOST, agent_ip, sizeof(agent_ip)))
                    state = TaskState::INITIALIZING;
                else
                    vTaskDelay(pdMS_TO_TICKS(100));
                break;
            }

            case TaskState::INITIALIZING: {
                init_options = rcl_get_zero_initialized_init_options();
                rcl_ret_t ret = rcl_init_options_init(&init_options, impl->allocator);
                if (ret != RCL_RET_OK) {
                    ESP_LOGE(TAG, "rcl_init_options_init failed: %ld (%s)",
                             (long)ret, rcl_get_error_string().str);
                    rcl_reset_error();
                    state = TaskState::BACKING_OFF; break;
                }
                rmw_uros_options_set_udp_address(agent_ip, "8888",
                    rcl_init_options_get_rmw_init_options(&init_options));
                ret = rclc_support_init_with_options(
                    &impl->support, 0, NULL, &init_options, &impl->allocator);
                ROS_CHECK(rcl_init_options_fini(&init_options), "init_options fini");
                if (ret != RCL_RET_OK) {
                    ESP_LOGE(TAG, "rclc_support_init_with_options failed: %ld (%s)",
                             (long)ret, rcl_get_error_string().str);
                    rcl_reset_error();
                    state = TaskState::BACKING_OFF; break;
                }
                create_entities(*impl);
                if (lock_impl()) {
                    impl->entities_created = true;
                    impl->time_synced      = false;
                    impl->setState(MicrorosState::TIME_SYNC);
                    unlock_impl();
                }
                consecutive_spin_failures = 0;
                backoff_ms = 250;
                state = TaskState::TIME_SYNCING;
                break;
            }

            case TaskState::TIME_SYNCING: {
                const bool synced = sync_time();
                if (lock_impl()) { impl->time_synced = synced; unlock_impl(); }
                if (!synced)
                    ESP_LOGW(TAG, "Proceeding without valid epoch — sensor topics gated until next sync");
                if (lock_impl()) { impl->setState(MicrorosState::CONNECTED); unlock_impl(); }
                ESP_LOGI(TAG, "micro-ROS ready (time_synced=%s)", synced ? "yes" : "no");
                state = TaskState::CONNECTED;
                break;
            }

            case TaskState::CONNECTED: {
                const rcl_ret_t spin_ret =
                    rclc_executor_spin_some(&impl->executor, RCL_MS_TO_NS(100));
                if (spin_ret != RCL_RET_OK) {
                    if (++consecutive_spin_failures >= 3) {
                        ESP_LOGW(TAG, "3 consecutive spin failures — disconnecting");
                        destroy_entities(*impl);
                        if (lock_impl()) {
                            impl->entities_created = false;
                            impl->time_synced      = false;
                            impl->setState(MicrorosState::DISCONNECTED);
                            unlock_impl();
                        }
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
                if (lock_impl()) { impl->setState(MicrorosState::DISCOVERING); unlock_impl(); }
                state = TaskState::DISCOVER_AGENT;
                break;
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------
MicrorosSync::MicrorosSync()  { g_impl = new MicrorosSyncImpl(); }
MicrorosSync::~MicrorosSync() { delete g_impl; g_impl = nullptr; }

MicrorosSync& MicrorosSync::getInstance() {
    static MicrorosSync instance;
    return instance;
}

bool MicrorosSync::init() {
    FirmwareVersion::print_version(TAG);
    StateMachine::setInitial("microros_sync", stateToString(MicrorosState::OFF));
    if (lock_impl()) { g_impl->setState(MicrorosState::OFF); unlock_impl(); }
    ESP_LOGI(TAG, "MicrorosSync initialised");
    return true;
}

void MicrorosSync::start() {
    if (!g_impl || g_impl->task_handle) return;
    xTaskCreate(microros_task, "microros_task", 16000, g_impl, 5, &g_impl->task_handle);
    ESP_LOGI(TAG, "MicrorosSync task started");
}

void MicrorosSync::publishHeartbeat(int32_t value) {
    if (lock_impl()) {
        if (g_impl->entities_created) {
            g_impl->heartbeat_msg.data = value;
            _pub_heartbeat();
        }
        unlock_impl();
    }
}

void MicrorosSync::publishMotorPositions(const float* positions, size_t count) {
    if (lock_impl()) {
        if (g_impl->entities_created && g_impl->time_synced) {
            const size_t copy = std::min(count,
                sizeof(g_impl->motor_positions_data) / sizeof(float));
            for (size_t i = 0; i < copy; ++i)
                g_impl->motor_positions_data[i] = positions[i];
            g_impl->motor_positions_msg.data.size = copy;
            _pub_motor_positions();
        }
        unlock_impl();
    }
}

void MicrorosSync::publishDistanceSensors(const float* distances, size_t count) {
    if (lock_impl()) {
        if (g_impl->entities_created && g_impl->time_synced) {
            const size_t copy = std::min(count, (size_t)SensorCommon::NUM_SENSORS);
            for (size_t i = 0; i < copy; ++i)
                g_impl->distance_sensors_data[i] = distances[i];
            g_impl->distance_sensors_msg.data.size = copy;
            _pub_distance_sensors();
        }
        unlock_impl();
    }
}

void MicrorosSync::publishLedState(bool led_state) {
    if (lock_impl()) {
        if (g_impl->entities_created) {
            g_impl->led_state_msg.data = led_state;
            _pub_led_state();
        }
        unlock_impl();
    }
}

void MicrorosSync::publishTofDistance(float distance_m) {
    if (lock_impl()) {
        if (g_impl->entities_created && g_impl->time_synced) {
            g_impl->tof_distance_msg.data = distance_m;
            _pub_tof_distance();
        }
        unlock_impl();
    }
}

void MicrorosSync::publishLidarScan(const SensorCommon::LidarMeasurement& m) {
    (void)m; // publishing handled by sensor_control_timer_cb
}
