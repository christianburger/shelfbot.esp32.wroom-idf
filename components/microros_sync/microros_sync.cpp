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

#include <sys/time.h>

static const char* TAG = "MicrorosSync";

// ---------------------------------------------------------------------------
// Logging helpers
// ---------------------------------------------------------------------------
#define RCL_LOG(call, msg) do { \
    const rcl_ret_t _r = (call); \
    if (_r != RCL_RET_OK) { \
        ESP_LOGE(TAG, "%s failed: %ld (%s)", (msg), (long)_r, \
                 rcl_get_error_string().str); \
        rcl_reset_error(); \
    } else { \
        ESP_LOGD(TAG, "%s OK", (msg)); \
    } \
} while(0)

#define RCL_LOG_RMW(call, msg) do { \
    const rmw_ret_t _r = (call); \
    if (_r != RMW_RET_OK) { \
        ESP_LOGE(TAG, "%s failed: %d", (msg), (int)_r); \
    } else { \
        ESP_LOGD(TAG, "%s OK", (msg)); \
    } \
} while(0)

static constexpr int32_t  SYNC_TIMEOUT_MS            = 5000;
static constexpr uint32_t EPOCH_WAIT_TIMEOUT_MS      = 30000;
static constexpr uint32_t EPOCH_POLL_MS              = 200;
static constexpr uint8_t  SPIN_FAIL_THRESHOLD        = 15;
static constexpr uint8_t  SPIN_RECOVERY_CREDIT       = 5;
static constexpr uint32_t PUB_FAIL_RECONNECT_THRESHOLD = 30;

// ---------------------------------------------------------------------------
// MicrorosSyncImpl – no internal state booleans
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

    uint32_t          pub_fail_count   = 0;
    TaskHandle_t      task_handle      = nullptr;
    SemaphoreHandle_t mutex            = nullptr;

    MicrorosSyncImpl();
    ~MicrorosSyncImpl();
    void fillStamp(int32_t& sec_out, uint32_t& nanosec_out) const;
};

std_msgs__msg__MultiArrayDimension MicrorosSyncImpl::motor_dim[1];
std_msgs__msg__MultiArrayDimension MicrorosSyncImpl::distance_dim[1];

static MicrorosSyncImpl* g_impl = nullptr;

static bool lock_impl() {
    return g_impl && g_impl->mutex &&
           xSemaphoreTake(g_impl->mutex, pdMS_TO_TICKS(100)) == pdTRUE;
}
static void unlock_impl() {
    if (g_impl && g_impl->mutex) xSemaphoreGive(g_impl->mutex);
}

// ---------------------------------------------------------------------------
// Constructor / Destructor (unchanged)
// ---------------------------------------------------------------------------
MicrorosSyncImpl::MicrorosSyncImpl()
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
    motor_positions_msg.layout.dim.data    = motor_dim;
    motor_positions_msg.layout.dim.size    = 1;
    motor_positions_msg.layout.data_offset = 0;

    distance_dim[0].label.data     = const_cast<char*>("");
    distance_dim[0].label.size     = 0;
    distance_dim[0].label.capacity = 1;
    distance_dim[0].size           = SensorCommon::NUM_SENSORS;
    distance_dim[0].stride         = SensorCommon::NUM_SENSORS;
    distance_sensors_msg.layout.dim.data    = distance_dim;
    distance_sensors_msg.layout.dim.size    = 1;
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
    configASSERT(mutex != nullptr);
}

MicrorosSyncImpl::~MicrorosSyncImpl() {
    if (mutex)       vSemaphoreDelete(mutex);
    if (task_handle) vTaskDelete(task_handle);
}

void MicrorosSyncImpl::fillStamp(int32_t& sec_out, uint32_t& nanosec_out) const {
    if (StateMachine::isAtLeast("time_sync", "synced")) {
        shelfbot::ShelfbotTimestamp::toRosTime(
            shelfbot::ShelfbotTimestamp::epochMicros(),
            sec_out, nanosec_out);
    } else {
        const int64_t mono_us = shelfbot::ShelfbotTimestamp::monotonicMicros();
        sec_out     = static_cast<int32_t>(mono_us / 1000000LL);
        nanosec_out = static_cast<uint32_t>((mono_us % 1000000LL) * 1000LL);
    }
}

// ---------------------------------------------------------------------------
// Publish helpers – all check state machine
// ---------------------------------------------------------------------------
static void _pub_heartbeat() {
    if (!StateMachine::isAtLeast("microros_sync", "connected")) return;
    const rcl_ret_t r = rcl_publish(&g_impl->heartbeat_pub, &g_impl->heartbeat_msg, NULL);
    if (r != RCL_RET_OK) {
        ESP_LOGE(TAG, "heartbeat publish failed: %ld (%s)", (long)r, rcl_get_error_string().str);
        rcl_reset_error();
        if (lock_impl()) { ++g_impl->pub_fail_count; unlock_impl(); }
    }
}

static void _pub_motor_positions() {
    if (!StateMachine::isAtLeast("microros_sync", "connected") ||
        !StateMachine::isAtLeast("time_sync", "synced")) return;
    const rcl_ret_t r = rcl_publish(&g_impl->motor_positions_pub, &g_impl->motor_positions_msg, NULL);
    if (r != RCL_RET_OK) {
        ESP_LOGE(TAG, "motor_positions publish failed: %ld (%s)", (long)r, rcl_get_error_string().str);
        rcl_reset_error();
        if (lock_impl()) { ++g_impl->pub_fail_count; unlock_impl(); }
    }
}

static void _pub_distance_sensors() {
    if (!StateMachine::isAtLeast("microros_sync", "connected") ||
        !StateMachine::isAtLeast("time_sync", "synced")) return;
    const rcl_ret_t r = rcl_publish(&g_impl->distance_sensors_pub, &g_impl->distance_sensors_msg, NULL);
    if (r != RCL_RET_OK) {
        ESP_LOGE(TAG, "distance_sensors publish failed: %ld (%s)", (long)r, rcl_get_error_string().str);
        rcl_reset_error();
        if (lock_impl()) { ++g_impl->pub_fail_count; unlock_impl(); }
    }
}

static void _pub_led_state() {
    if (!StateMachine::isAtLeast("microros_sync", "connected")) return;
    const rcl_ret_t r = rcl_publish(&g_impl->led_state_pub, &g_impl->led_state_msg, NULL);
    if (r != RCL_RET_OK) {
        ESP_LOGE(TAG, "led_state publish failed: %ld (%s)", (long)r, rcl_get_error_string().str);
        rcl_reset_error();
        if (lock_impl()) { ++g_impl->pub_fail_count; unlock_impl(); }
    }
}

static void _pub_tof_distance() {
    if (!StateMachine::isAtLeast("microros_sync", "connected") ||
        !StateMachine::isAtLeast("time_sync", "synced")) return;
    const rcl_ret_t r = rcl_publish(&g_impl->tof_distance_pub, &g_impl->tof_distance_msg, NULL);
    if (r != RCL_RET_OK) {
        ESP_LOGE(TAG, "tof_distance publish failed: %ld (%s)", (long)r, rcl_get_error_string().str);
        rcl_reset_error();
        if (lock_impl()) { ++g_impl->pub_fail_count; unlock_impl(); }
    }
}

static void _pub_laser_scan() {
    if (!StateMachine::isAtLeast("microros_sync", "connected") ||
        !StateMachine::isAtLeast("time_sync", "synced")) return;
    const rcl_ret_t r = rcl_publish(&g_impl->laser_scan_pub, &g_impl->laser_scan_msg, NULL);
    if (r != RCL_RET_OK) {
        ESP_LOGE(TAG, "laser_scan publish failed: %ld (%s)", (long)r, rcl_get_error_string().str);
        rcl_reset_error();
        if (lock_impl()) { ++g_impl->pub_fail_count; unlock_impl(); }
    }
}

// ---------------------------------------------------------------------------
// Timer callbacks
// ---------------------------------------------------------------------------
static void heartbeat_timer_cb(rcl_timer_t* /*timer*/, int64_t /*last_call_time*/) {
    static int32_t counter = 0;
    if (!lock_impl()) return;
    if (StateMachine::isAtLeast("microros_sync", "connected")) {
        g_impl->heartbeat_msg.data = ++counter;
        _pub_heartbeat();
    }
    unlock_impl();
}

static void motor_position_timer_cb(rcl_timer_t* /*timer*/, int64_t /*last_call_time*/) {
    if (!lock_impl()) return;
    if (StateMachine::isAtLeast("microros_sync", "connected") &&
        StateMachine::isAtLeast("time_sync", "synced")) {
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
    if (!StateMachine::isAtLeast("microros_sync", "connected") ||
        !StateMachine::isAtLeast("time_sync", "synced")) {
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

    float distances[SensorCommon::NUM_SENSORS];
    size_t idx = 0;
    for (int i = 0; i < SensorCommon::NUM_ULTRASONIC_SENSORS &&
         idx < (size_t)SensorCommon::NUM_SENSORS; ++i, ++idx)
        distances[idx] = data.ultrasonic_readings[i].distance_cm;
    for (int i = 0; i < SensorCommon::NUM_TOF_SENSORS &&
         idx < (size_t)SensorCommon::NUM_SENSORS; ++i, ++idx)
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

    g_impl->tof_distance_msg.data = data.tof_measurements[0].valid
        ? (data.tof_measurements[0].distance_mm / 1000.0f) : -1.0f;
    _pub_tof_distance();

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
        if (StateMachine::isAtLeast("microros_sync", "connected")) {
            g_impl->led_state_msg.data = led->data;
            _pub_led_state();
        }
        unlock_impl();
    }
}

// ---------------------------------------------------------------------------
// Entity management functions
// ---------------------------------------------------------------------------
static void reset_entity_handles(MicrorosSyncImpl& impl) {
    impl.node                 = rcl_get_zero_initialized_node();
    impl.executor             = rclc_executor_get_zero_initialized_executor();
    impl.heartbeat_pub        = rcl_get_zero_initialized_publisher();
    impl.motor_positions_pub  = rcl_get_zero_initialized_publisher();
    impl.distance_sensors_pub = rcl_get_zero_initialized_publisher();
    impl.led_state_pub        = rcl_get_zero_initialized_publisher();
    impl.tof_distance_pub     = rcl_get_zero_initialized_publisher();
    impl.laser_scan_pub       = rcl_get_zero_initialized_publisher();
    impl.motor_command_sub    = rcl_get_zero_initialized_subscription();
    impl.set_speed_sub        = rcl_get_zero_initialized_subscription();
    impl.led_sub              = rcl_get_zero_initialized_subscription();
    impl.heartbeat_timer      = rcl_get_zero_initialized_timer();
    impl.motor_position_timer = rcl_get_zero_initialized_timer();
    impl.sensor_control_timer = rcl_get_zero_initialized_timer();
}

static bool safe_destroy_support(MicrorosSyncImpl& impl) {
    const rcl_ret_t r = rclc_support_fini(&impl.support);
    if (r != RCL_RET_OK) {
        ESP_LOGE(TAG, "support fini failed: %ld (%s)", (long)r,
                 rcl_get_error_string().str);
        rcl_reset_error();
    } else {
        ESP_LOGD(TAG, "support fini OK");
    }
    memset(&impl.support, 0, sizeof(impl.support));
    return (r == RCL_RET_OK);
}

static bool destroy_entities(MicrorosSyncImpl& impl) {
    ESP_LOGI(TAG, "Destroying micro-ROS entities");
    bool all_ok = true;
    rcl_ret_t r;

    r = rclc_executor_fini(&impl.executor);
    if (r != RCL_RET_OK) {
        ESP_LOGE(TAG, "executor fini failed: %ld (%s)", (long)r, rcl_get_error_string().str);
        rcl_reset_error(); all_ok = false;
    }

    r = rcl_publisher_fini(&impl.heartbeat_pub, &impl.node);
    if (r != RCL_RET_OK) { ESP_LOGE(TAG, "heartbeat pub fini failed"); rcl_reset_error(); all_ok = false; }
    r = rcl_publisher_fini(&impl.motor_positions_pub, &impl.node);
    if (r != RCL_RET_OK) { ESP_LOGE(TAG, "motor_positions pub fini failed"); rcl_reset_error(); all_ok = false; }
    r = rcl_publisher_fini(&impl.distance_sensors_pub, &impl.node);
    if (r != RCL_RET_OK) { ESP_LOGE(TAG, "distance_sensors pub fini failed"); rcl_reset_error(); all_ok = false; }
    r = rcl_publisher_fini(&impl.led_state_pub, &impl.node);
    if (r != RCL_RET_OK) { ESP_LOGE(TAG, "led_state pub fini failed"); rcl_reset_error(); all_ok = false; }
    r = rcl_publisher_fini(&impl.tof_distance_pub, &impl.node);
    if (r != RCL_RET_OK) { ESP_LOGE(TAG, "tof_distance pub fini failed"); rcl_reset_error(); all_ok = false; }
    r = rcl_publisher_fini(&impl.laser_scan_pub, &impl.node);
    if (r != RCL_RET_OK) { ESP_LOGE(TAG, "laser_scan pub fini failed"); rcl_reset_error(); all_ok = false; }

    r = rcl_subscription_fini(&impl.motor_command_sub, &impl.node);
    if (r != RCL_RET_OK) { ESP_LOGE(TAG, "motor_command sub fini failed"); rcl_reset_error(); all_ok = false; }
    r = rcl_subscription_fini(&impl.set_speed_sub, &impl.node);
    if (r != RCL_RET_OK) { ESP_LOGE(TAG, "set_speed sub fini failed"); rcl_reset_error(); all_ok = false; }
    r = rcl_subscription_fini(&impl.led_sub, &impl.node);
    if (r != RCL_RET_OK) { ESP_LOGE(TAG, "led sub fini failed"); rcl_reset_error(); all_ok = false; }

    r = rcl_timer_fini(&impl.heartbeat_timer);
    if (r != RCL_RET_OK) { ESP_LOGE(TAG, "heartbeat timer fini failed"); rcl_reset_error(); all_ok = false; }
    r = rcl_timer_fini(&impl.motor_position_timer);
    if (r != RCL_RET_OK) { ESP_LOGE(TAG, "motor_position timer fini failed"); rcl_reset_error(); all_ok = false; }
    r = rcl_timer_fini(&impl.sensor_control_timer);
    if (r != RCL_RET_OK) { ESP_LOGE(TAG, "sensor_control timer fini failed"); rcl_reset_error(); all_ok = false; }

    r = rcl_node_fini(&impl.node);
    if (r != RCL_RET_OK) {
        ESP_LOGE(TAG, "node fini failed: %ld (%s)", (long)r, rcl_get_error_string().str);
        rcl_reset_error();
        all_ok = false;
    }

    if (all_ok) ESP_LOGI(TAG, "destroy_entities: all fini calls succeeded");
    else ESP_LOGW(TAG, "destroy_entities: one or more fini calls failed");
    return all_ok;
}

static bool create_entities(MicrorosSyncImpl& impl) {
    ESP_LOGI(TAG, "Creating micro-ROS entities");

    rcl_ret_t r = rclc_node_init_default(
        &impl.node, "shelfbot_firmware", "", &impl.support);
    if (r != RCL_RET_OK) {
        ESP_LOGE(TAG, "node init failed: %ld (%s)", (long)r, rcl_get_error_string().str);
        rcl_reset_error();
        impl.node = rcl_get_zero_initialized_node();
        return false;
    }

    bool ok = true;

    // Publishers
    r = rclc_publisher_init_best_effort(&impl.heartbeat_pub, &impl.node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "shelfbot_firmware/heartbeat");
    if (r != RCL_RET_OK) { ESP_LOGE(TAG, "heartbeat pub failed"); rcl_reset_error(); ok = false; }

    if (ok) {
        r = rclc_publisher_init_best_effort(&impl.motor_positions_pub, &impl.node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "shelfbot_firmware/motor_positions");
        if (r != RCL_RET_OK) { ESP_LOGE(TAG, "motor_positions pub failed"); rcl_reset_error(); ok = false; }
    }

    if (ok) {
        r = rclc_publisher_init_best_effort(&impl.distance_sensors_pub, &impl.node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "shelfbot_firmware/distance_sensors");
        if (r != RCL_RET_OK) { ESP_LOGE(TAG, "distance_sensors pub failed"); rcl_reset_error(); ok = false; }
    }

    if (ok) {
        r = rclc_publisher_init_best_effort(&impl.led_state_pub, &impl.node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "shelfbot_firmware/led_state");
        if (r != RCL_RET_OK) { ESP_LOGE(TAG, "led_state pub failed"); rcl_reset_error(); ok = false; }
    }

    if (ok) {
        r = rclc_publisher_init_best_effort(&impl.tof_distance_pub, &impl.node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "shelfbot_firmware/tof_distance");
        if (r != RCL_RET_OK) { ESP_LOGE(TAG, "tof_distance pub failed"); rcl_reset_error(); ok = false; }
    }

    if (ok) {
        r = rclc_publisher_init_default(&impl.laser_scan_pub, &impl.node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan), "shelfbot_firmware/laser_scan");
        if (r != RCL_RET_OK) { ESP_LOGE(TAG, "laser_scan pub failed"); rcl_reset_error(); ok = false; }
    }

    // Subscriptions
    if (ok) {
        r = rclc_subscription_init_default(&impl.motor_command_sub, &impl.node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "shelfbot_firmware/motor_command");
        if (r != RCL_RET_OK) { ESP_LOGE(TAG, "motor_command sub failed"); rcl_reset_error(); ok = false; }
    }

    if (ok) {
        r = rclc_subscription_init_default(&impl.set_speed_sub, &impl.node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "shelfbot_firmware/set_speed");
        if (r != RCL_RET_OK) { ESP_LOGE(TAG, "set_speed sub failed"); rcl_reset_error(); ok = false; }
    }

    if (ok) {
        r = rclc_subscription_init_default(&impl.led_sub, &impl.node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "shelfbot_firmware/led");
        if (r != RCL_RET_OK) { ESP_LOGE(TAG, "led sub failed"); rcl_reset_error(); ok = false; }
    }

    // Timers
    if (ok) {
        r = rclc_timer_init_default(&impl.heartbeat_timer, &impl.support, RCL_MS_TO_NS(1000), heartbeat_timer_cb);
        if (r != RCL_RET_OK) { ESP_LOGE(TAG, "heartbeat timer failed"); rcl_reset_error(); ok = false; }
    }

    if (ok) {
        r = rclc_timer_init_default(&impl.motor_position_timer, &impl.support, RCL_MS_TO_NS(100), motor_position_timer_cb);
        if (r != RCL_RET_OK) { ESP_LOGE(TAG, "motor_position timer failed"); rcl_reset_error(); ok = false; }
    }

    if (ok) {
        r = rclc_timer_init_default(&impl.sensor_control_timer, &impl.support, RCL_MS_TO_NS(200), sensor_control_timer_cb);
        if (r != RCL_RET_OK) { ESP_LOGE(TAG, "sensor_control timer failed"); rcl_reset_error(); ok = false; }
    }

    // Executor (6 handles: 3 timers + 3 subs)
    if (ok) {
        r = rclc_executor_init(&impl.executor, &impl.support.context, 6, &impl.allocator);
        if (r != RCL_RET_OK) { ESP_LOGE(TAG, "executor init failed"); rcl_reset_error(); ok = false; }
    }

    if (ok) {
        r = rclc_executor_add_timer(&impl.executor, &impl.heartbeat_timer);
        if (r != RCL_RET_OK) { ESP_LOGE(TAG, "add heartbeat timer failed"); rcl_reset_error(); ok = false; }
    }
    if (ok) {
        r = rclc_executor_add_timer(&impl.executor, &impl.motor_position_timer);
        if (r != RCL_RET_OK) { ESP_LOGE(TAG, "add motor_position timer failed"); rcl_reset_error(); ok = false; }
    }
    if (ok) {
        r = rclc_executor_add_timer(&impl.executor, &impl.sensor_control_timer);
        if (r != RCL_RET_OK) { ESP_LOGE(TAG, "add sensor_control timer failed"); rcl_reset_error(); ok = false; }
    }
    if (ok) {
        r = rclc_executor_add_subscription(&impl.executor, &impl.motor_command_sub, &impl.motor_command_msg, motor_command_cb, ON_NEW_DATA);
        if (r != RCL_RET_OK) { ESP_LOGE(TAG, "add motor_command sub failed"); rcl_reset_error(); ok = false; }
    }
    if (ok) {
        r = rclc_executor_add_subscription(&impl.executor, &impl.set_speed_sub, &impl.set_speed_msg, set_speed_cb, ON_NEW_DATA);
        if (r != RCL_RET_OK) { ESP_LOGE(TAG, "add set_speed sub failed"); rcl_reset_error(); ok = false; }
    }
    if (ok) {
        r = rclc_executor_add_subscription(&impl.executor, &impl.led_sub, &impl.led_msg, led_cb, ON_NEW_DATA);
        if (r != RCL_RET_OK) { ESP_LOGE(TAG, "add led sub failed"); rcl_reset_error(); ok = false; }
    }

    if (!ok) {
        rcl_ret_t fini_r = rcl_node_fini(&impl.node);
        if (fini_r != RCL_RET_OK) {
            ESP_LOGE(TAG, "node fini after failed creation: %ld", (long)fini_r);
            rcl_reset_error();
        }
        reset_entity_handles(impl);
        return false;
    }

    ESP_LOGI(TAG, "create_entities: all entities created successfully");
    return true;
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
        if (err == ESP_ERR_NOT_FOUND) ESP_LOGW(TAG, "mDNS host not found");
        else ESP_LOGE(TAG, "mDNS query failed: %s", esp_err_to_name(err));
        return false;
    }
    snprintf(out_ip, len, IPSTR, IP2STR(&addr));
    ESP_LOGI(TAG, "Agent IP: %s", out_ip);
    return true;
}

// ---------------------------------------------------------------------------
// SNTP clock sync – also sets time_sync state
// ---------------------------------------------------------------------------
static bool sync_time() {
    ESP_LOGI(TAG, "Waiting for SNTP to provide valid wall clock (timeout %lu ms)",
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
            StateMachine::changeState("time_sync", "synced");
            return true;
        }
        if (++polls % 10 == 0)
            ESP_LOGI(TAG, "Waiting for SNTP... (%lu ms elapsed)",
                     (unsigned long)(polls * EPOCH_POLL_MS));
        vTaskDelay(pdMS_TO_TICKS(EPOCH_POLL_MS));
    }
    ESP_LOGW(TAG, "SNTP sync timed out — will use monotonic timestamps");
    return false;
}

// ---------------------------------------------------------------------------
// microros_task – with full state machine integration and agent tracking
// ---------------------------------------------------------------------------
static void microros_task(void* arg) {
    auto* impl = static_cast<MicrorosSyncImpl*>(arg);
    if (!impl) { vTaskDelete(nullptr); return; }

    // Register agent module with ordered states from AgentState enum
    StateMachine::setInitial("agent", stateToString(AgentState::OFFLINE),
                             {stateToString(AgentState::OFFLINE),
                              stateToString(AgentState::DISCOVERED),
                              stateToString(AgentState::PING_OK),
                              stateToString(AgentState::SESSION_SYNCED),
                              stateToString(AgentState::ENTITIES_CREATED),
                              stateToString(AgentState::CONNECTED),
                              stateToString(AgentState::ERROR)});

    // Prerequisites for microros_sync using agent states
    StateMachine::Prerequisite wifi_prereq{"wifi_manager", stateToString(WifiManagerState::CONNECTED)};
    StateMachine::Prerequisite mdns_prereq{"network_service", stateToString(NetworkServiceState::MDNS_READY)};
    StateMachine::registerPrerequisite("microros_sync", "discovering", wifi_prereq);
    StateMachine::registerPrerequisite("microros_sync", "discovering", mdns_prereq);

    StateMachine::Prerequisite agent_discovered{"agent", stateToString(AgentState::DISCOVERED)};
    StateMachine::Prerequisite agent_ping_ok{"agent", stateToString(AgentState::PING_OK)};
    StateMachine::Prerequisite agent_session_synced{"agent", stateToString(AgentState::SESSION_SYNCED)};
    StateMachine::registerPrerequisite("microros_sync", "time_sync", agent_discovered);
    StateMachine::registerPrerequisite("microros_sync", "creating_entities", agent_ping_ok);
    StateMachine::registerPrerequisite("microros_sync", "creating_entities", StateMachine::Prerequisite{"time_sync", "synced"});
    StateMachine::registerPrerequisite("microros_sync", "connected", agent_session_synced);

    char agent_ip[16] = {};
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    bool init_options_inited = false;

    while (true) {
        // Wait for Wi-Fi
        xEventGroupWaitBits(wifi_manager_get_event_group(), WM_CONNECTED_BIT, pdFALSE, pdFALSE, portMAX_DELAY);

        if (!StateMachine::waitForPrerequisites("microros_sync", "discovering", 30000, 500)) {
            ESP_LOGW(TAG, "Prerequisites for discovering not met, retrying...");
            vTaskDelay(pdMS_TO_TICKS(2000));
            continue;
        }
        StateMachine::changeState("microros_sync", "discovering");

        // mDNS lookup
        if (!query_mdns_host(CONFIG_MICROROS_AGENT_MDNS_HOST, agent_ip, sizeof(agent_ip))) {
            ESP_LOGW(TAG, "Agent mDNS not found");
            StateMachine::changeState("microros_sync", "disconnected");
            StateMachine::changeState("agent", stateToString(AgentState::OFFLINE));
            vTaskDelay(pdMS_TO_TICKS(2000));
            continue;
        }
        StateMachine::changeState("agent", stateToString(AgentState::DISCOVERED));
        StateMachine::changeState("microros_sync", "time_sync");

        // Initialise support
        rcl_ret_t r = rcl_init_options_init(&init_options, impl->allocator);
        if (r != RCL_RET_OK) {
            ESP_LOGE(TAG, "rcl_init_options_init failed");
            rcl_reset_error();
            StateMachine::changeState("microros_sync", "error");
            StateMachine::changeState("agent", stateToString(AgentState::ERROR));
            vTaskDelay(pdMS_TO_TICKS(2000));
            continue;
        }
        init_options_inited = true;

        rmw_ret_t rmw_r = rmw_uros_options_set_udp_address(agent_ip, "8888",
            rcl_init_options_get_rmw_init_options(&init_options));
        if (rmw_r != RMW_RET_OK) {
            ESP_LOGE(TAG, "set_udp_address failed");
            if (init_options_inited) {
                rcl_ret_t fini_r = rcl_init_options_fini(&init_options);
                if (fini_r != RCL_RET_OK) ESP_LOGE(TAG, "init_options fini failed");
                init_options_inited = false;
            }
            StateMachine::changeState("microros_sync", "error");
            StateMachine::changeState("agent", stateToString(AgentState::ERROR));
            vTaskDelay(pdMS_TO_TICKS(2000));
            continue;
        }

        r = rclc_support_init_with_options(&impl->support, 0, nullptr, &init_options, &impl->allocator);
        if (init_options_inited) {
            rcl_ret_t fini_r = rcl_init_options_fini(&init_options);
            if (fini_r != RCL_RET_OK) ESP_LOGE(TAG, "init_options fini failed");
            init_options_inited = false;
        }
        if (r != RCL_RET_OK) {
            ESP_LOGE(TAG, "rclc_support_init failed");
            rcl_reset_error();
            StateMachine::changeState("microros_sync", "error");
            StateMachine::changeState("agent", stateToString(AgentState::ERROR));
            vTaskDelay(pdMS_TO_TICKS(2000));
            continue;
        }

        vTaskDelay(pdMS_TO_TICKS(200));
        if (rmw_uros_ping_agent(500, 3) != RMW_RET_OK) {
            ESP_LOGE(TAG, "Agent ping failed");
            safe_destroy_support(*impl);
            StateMachine::changeState("microros_sync", "disconnected");
            StateMachine::changeState("agent", stateToString(AgentState::OFFLINE));
            vTaskDelay(pdMS_TO_TICKS(2000));
            continue;
        }
        StateMachine::changeState("agent", stateToString(AgentState::PING_OK));

        if (rmw_uros_sync_session(SYNC_TIMEOUT_MS) != RMW_RET_OK) {
            ESP_LOGE(TAG, "Session sync failed");
            safe_destroy_support(*impl);
            StateMachine::changeState("microros_sync", "disconnected");
            StateMachine::changeState("agent", stateToString(AgentState::OFFLINE));
            vTaskDelay(pdMS_TO_TICKS(2000));
            continue;
        }
        StateMachine::changeState("agent", stateToString(AgentState::SESSION_SYNCED));

        // Wait for time sync
        if (!StateMachine::waitForPrerequisites("microros_sync", "creating_entities", EPOCH_WAIT_TIMEOUT_MS, EPOCH_POLL_MS)) {
            ESP_LOGW(TAG, "Time sync prerequisite not met – continuing");
        }
        (void)sync_time();  // sets time_sync state

        StateMachine::changeState("microros_sync", "creating_entities");
        if (!create_entities(*impl)) {
            ESP_LOGE(TAG, "create_entities failed");
            safe_destroy_support(*impl);
            StateMachine::changeState("microros_sync", "error");
            StateMachine::changeState("agent", stateToString(AgentState::ERROR));
            vTaskDelay(pdMS_TO_TICKS(5000));
            continue;
        }
        StateMachine::changeState("agent", stateToString(AgentState::ENTITIES_CREATED));
        StateMachine::changeState("microros_sync", "connected");
        StateMachine::changeState("agent", stateToString(AgentState::CONNECTED));

        // Main spin loop
        uint8_t consecutive_spin_failures = 0;
        while (true) {
            if (!(xEventGroupGetBits(wifi_manager_get_event_group()) & WM_CONNECTED_BIT)) {
                ESP_LOGW(TAG, "Wi-Fi lost");
                break;
            }
            rcl_ret_t spin_ret = rclc_executor_spin_some(&impl->executor, RCL_MS_TO_NS(100));
            bool need_reconnect = false;
            if (spin_ret != RCL_RET_OK) {
                if (++consecutive_spin_failures >= SPIN_FAIL_THRESHOLD) need_reconnect = true;
            } else {
                consecutive_spin_failures = std::max(0, consecutive_spin_failures - SPIN_RECOVERY_CREDIT);
                uint32_t pub_fails = 0;
                if (lock_impl()) { pub_fails = impl->pub_fail_count; unlock_impl(); }
                if (pub_fails >= PUB_FAIL_RECONNECT_THRESHOLD) need_reconnect = true;
            }
            if (need_reconnect) break;
            vTaskDelay(pdMS_TO_TICKS(10));
        }

        // Teardown
        destroy_entities(*impl);
        reset_entity_handles(*impl);
        safe_destroy_support(*impl);
        StateMachine::changeState("microros_sync", "disconnected");
        StateMachine::changeState("agent", stateToString(AgentState::OFFLINE));
        vTaskDelay(pdMS_TO_TICKS(2000));
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
    std::vector<std::string> ordered = {"off", "discovering", "time_sync", "creating_entities", "connected", "error", "disconnected"};
    StateMachine::setInitial("microros_sync", stateToString(MicrorosState::OFF), ordered);
    StateMachine::setInitial("time_sync", "unsynced", {"unsynced", "synced"});
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
        g_impl->heartbeat_msg.data = value;
        _pub_heartbeat();
        unlock_impl();
    }
}

void MicrorosSync::publishMotorPositions(const float* positions, size_t count) {
    if (lock_impl()) {
        const size_t copy = std::min(count, sizeof(g_impl->motor_positions_data) / sizeof(float));
        for (size_t i = 0; i < copy; ++i)
            g_impl->motor_positions_data[i] = positions[i];
        g_impl->motor_positions_msg.data.size = copy;
        _pub_motor_positions();
        unlock_impl();
    }
}

void MicrorosSync::publishDistanceSensors(const float* distances, size_t count) {
    if (lock_impl()) {
        const size_t copy = std::min(count, (size_t)SensorCommon::NUM_SENSORS);
        for (size_t i = 0; i < copy; ++i)
            g_impl->distance_sensors_data[i] = distances[i];
        g_impl->distance_sensors_msg.data.size = copy;
        _pub_distance_sensors();
        unlock_impl();
    }
}

void MicrorosSync::publishLedState(bool led_state) {
    if (lock_impl()) {
        g_impl->led_state_msg.data = led_state;
        _pub_led_state();
        unlock_impl();
    }
}

void MicrorosSync::publishTofDistance(float distance_m) {
    if (lock_impl()) {
        g_impl->tof_distance_msg.data = distance_m;
        _pub_tof_distance();
        unlock_impl();
    }
}

void MicrorosSync::publishLidarScan(const SensorCommon::LidarMeasurement& m) {
    (void)m;
}