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
// Timing constants
// ---------------------------------------------------------------------------
static constexpr int32_t  SYNC_TIMEOUT_MS             = 5000;
static constexpr uint32_t EPOCH_WAIT_TIMEOUT_MS        = 30000;
static constexpr uint32_t EPOCH_POLL_MS                = 250;
static constexpr uint32_t BACKOFF_INITIAL_MS           = 1000;
static constexpr uint32_t BACKOFF_MAX_MS               = 30000;
static constexpr int      SPIN_FAIL_THRESHOLD          = 15;
static constexpr int      SPIN_RECOVERY_CREDIT         = 5;
static constexpr uint32_t PUB_FAIL_RECONNECT_THRESHOLD = 30;

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

    std_msgs__msg__Int32              heartbeat_msg;
    std_msgs__msg__Float32MultiArray  motor_positions_msg;
    std_msgs__msg__Float32MultiArray  distance_sensors_msg;
    std_msgs__msg__Bool               led_state_msg;
    std_msgs__msg__Float32            tof_distance_msg;
    sensor_msgs__msg__LaserScan       laser_scan_msg;
    std_msgs__msg__Float32MultiArray  motor_command_msg;
    std_msgs__msg__Float32MultiArray  set_speed_msg;
    std_msgs__msg__Bool               led_msg;

    float motor_cmd_data[NUM_MOTORS]                       = {};
    float set_speed_data[NUM_MOTORS]                       = {};
    float motor_positions_data[NUM_MOTORS]                 = {};
    float distance_sensors_data[SensorCommon::NUM_SENSORS] = {};
    float ranges_data[12]                                  = {};
    float intensities_data[12]                             = {};

    // pub_fail_count is written from timer callbacks (executor task) and
    // read + cleared from the spin loop (same task) – no lock needed.
    uint32_t pub_fail_count = 0;

    TaskHandle_t      task_handle    = nullptr;
    SemaphoreHandle_t mutex          = nullptr;

    static std_msgs__msg__MultiArrayDimension motor_dim[1];
    static std_msgs__msg__MultiArrayDimension distance_dim[1];

    MicrorosSyncImpl();
    ~MicrorosSyncImpl();

    void fillStamp(int32_t& sec_out, uint32_t& nanosec_out) const;
};

std_msgs__msg__MultiArrayDimension MicrorosSyncImpl::motor_dim[1];
std_msgs__msg__MultiArrayDimension MicrorosSyncImpl::distance_dim[1];

static MicrorosSyncImpl* g_impl = nullptr;

static bool lock_impl()  {
    return g_impl && g_impl->mutex &&
           xSemaphoreTake(g_impl->mutex, pdMS_TO_TICKS(100)) == pdTRUE;
}
static void unlock_impl() {
    if (g_impl && g_impl->mutex) xSemaphoreGive(g_impl->mutex);
}

// ---------------------------------------------------------------------------
// Constructor / Destructor
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

    motor_dim[0] = { {const_cast<char*>(""), 0, 1}, NUM_MOTORS, NUM_MOTORS };
    motor_positions_msg.layout.dim.data    = motor_dim;
    motor_positions_msg.layout.dim.size    = 1;
    motor_positions_msg.layout.data_offset = 0;
    motor_positions_msg.data.data          = motor_positions_data;
    motor_positions_msg.data.capacity      = NUM_MOTORS;
    motor_positions_msg.data.size          = 0;

    distance_dim[0] = { {const_cast<char*>(""), 0, 1},
                        SensorCommon::NUM_SENSORS, SensorCommon::NUM_SENSORS };
    distance_sensors_msg.layout.dim.data    = distance_dim;
    distance_sensors_msg.layout.dim.size    = 1;
    distance_sensors_msg.layout.data_offset = 0;
    distance_sensors_msg.data.data          = distance_sensors_data;
    distance_sensors_msg.data.capacity      = SensorCommon::NUM_SENSORS;
    distance_sensors_msg.data.size          = 0;

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
    laser_scan_msg.range_min            = 0.02f;
    laser_scan_msg.range_max            = 12.0f;
    laser_scan_msg.time_increment       = 0.0f;
    laser_scan_msg.scan_time            = 0.1f;

    static char frame_id[] = "lidar_frame";
    laser_scan_msg.header.frame_id = { frame_id, sizeof(frame_id) - 1, sizeof(frame_id) };

    mutex = xSemaphoreCreateMutex();
    configASSERT(mutex != nullptr);
}

MicrorosSyncImpl::~MicrorosSyncImpl() {
    if (mutex)       vSemaphoreDelete(mutex);
    if (task_handle) vTaskDelete(task_handle);
}

void MicrorosSyncImpl::fillStamp(int32_t& sec, uint32_t& ns) const {
    if (StateMachine::isAtLeast("time_sync",
                                stateToString(TimeSyncState::SYNCED))) {
        shelfbot::ShelfbotTimestamp::toRosTime(
            shelfbot::ShelfbotTimestamp::epochMicros(), sec, ns);
    } else {
        const int64_t mono = shelfbot::ShelfbotTimestamp::monotonicMicros();
        sec = static_cast<int32_t>(mono / 1000000LL);
        ns  = static_cast<uint32_t>((mono % 1000000LL) * 1000LL);
    }
}

// ---------------------------------------------------------------------------
// Publish helpers
// These run only inside the executor task (serialised) — no lock needed.
// pub_fail_count is read and cleared from the same task in the spin loop.
// ---------------------------------------------------------------------------
static bool is_connected() {
    return StateMachine::isAtLeast("microros_sync",
                                   stateToString(MicrorosState::CONNECTED));
}
static bool is_time_synced() {
    return StateMachine::isAtLeast("time_sync",
                                   stateToString(TimeSyncState::SYNCED));
}

#define PUB_OR_FAIL(pub_handle, msg_ptr, label)                        \
    do {                                                               \
        const rcl_ret_t _r = rcl_publish(&(pub_handle), (msg_ptr), NULL); \
        if (_r != RCL_RET_OK) {                                        \
            ESP_LOGE(TAG, label " pub failed: %ld (%s)",               \
                     (long)_r, rcl_get_error_string().str);            \
            rcl_reset_error();                                         \
            g_impl->pub_fail_count = g_impl->pub_fail_count + 1;       \
        }                                                              \
    } while (0)

static void _pub_heartbeat() {
    if (!is_connected()) return;
    PUB_OR_FAIL(g_impl->heartbeat_pub, &g_impl->heartbeat_msg, "heartbeat");
}
static void _pub_motor_positions() {
    if (!is_connected() || !is_time_synced()) return;
    PUB_OR_FAIL(g_impl->motor_positions_pub, &g_impl->motor_positions_msg, "motor_positions");
}
static void _pub_distance_sensors() {
    if (!is_connected() || !is_time_synced()) return;
    PUB_OR_FAIL(g_impl->distance_sensors_pub, &g_impl->distance_sensors_msg, "distance_sensors");
}
static void _pub_led_state() {
    if (!is_connected()) return;
    PUB_OR_FAIL(g_impl->led_state_pub, &g_impl->led_state_msg, "led_state");
}
static void _pub_tof_distance() {
    if (!is_connected() || !is_time_synced()) return;
    PUB_OR_FAIL(g_impl->tof_distance_pub, &g_impl->tof_distance_msg, "tof_distance");
}
static void _pub_laser_scan() {
    if (!is_connected() || !is_time_synced()) return;
    PUB_OR_FAIL(g_impl->laser_scan_pub, &g_impl->laser_scan_msg, "laser_scan");
}

// ---------------------------------------------------------------------------
// Timer callbacks (executor task only)
// ---------------------------------------------------------------------------
static void heartbeat_timer_cb(rcl_timer_t*, int64_t) {
    static int32_t counter = 0;
    if (!is_connected()) return;
    g_impl->heartbeat_msg.data = ++counter;
    _pub_heartbeat();
}

static void motor_position_timer_cb(rcl_timer_t*, int64_t) {
    if (!is_connected() || !is_time_synced()) return;
    for (uint8_t i = 0; i < NUM_MOTORS; ++i)
        g_impl->motor_positions_data[i] = static_cast<float>(motor_control_get_position(i));
    g_impl->motor_positions_msg.data.size = NUM_MOTORS;
    _pub_motor_positions();
}

static void sensor_control_timer_cb(rcl_timer_t*, int64_t) {
    if (!is_connected() || !is_time_synced()) {
        static uint32_t skips = 0;
        if ((++skips & 0x1F) == 1)
            ESP_LOGD(TAG, "sensor_control_timer: waiting for connected+synced (%lu skips)",
                     (unsigned long)skips);
        return;
    }

    SensorCommon::SensorDataPacket data;
    if (!SensorManager::get_instance().get_latest_data(data)) {
        ESP_LOGW(TAG, "sensor_control_timer: no sensor data available");
        return;
    }

    // Pack distance array: ultrasonic, ToF, LiDAR
    size_t idx = 0;
    for (int i = 0; i < SensorCommon::NUM_ULTRASONIC_SENSORS &&
         idx < (size_t)SensorCommon::NUM_SENSORS; ++i, ++idx)
        g_impl->distance_sensors_data[idx] = data.ultrasonic_readings[i].distance_cm;
    for (int i = 0; i < SensorCommon::NUM_TOF_SENSORS &&
         idx < (size_t)SensorCommon::NUM_SENSORS; ++i, ++idx)
        g_impl->distance_sensors_data[idx] = data.tof_measurements[i].distance_mm / 10.0f;
    if (idx < (size_t)SensorCommon::NUM_SENSORS) {
        g_impl->distance_sensors_data[idx++] = data.lidar_measurement.valid
            ? (data.lidar_measurement.distance_mm / 10.0f) : -1.0f;
    }
    g_impl->distance_sensors_msg.data.size = idx;
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
        g_impl->fillStamp(g_impl->laser_scan_msg.header.stamp.sec,
                          g_impl->laser_scan_msg.header.stamp.nanosec);
        for (int i = 0; i < 12; ++i) {
            const uint16_t mm = m.packet_distances_mm[i];
            g_impl->ranges_data[i] = (mm == 0 || mm > 12000)
                ? (g_impl->laser_scan_msg.range_max + 1.0f)
                : (mm / 1000.0f);
            g_impl->intensities_data[i] = static_cast<float>(m.packet_confidences[i]);
        }
        g_impl->laser_scan_msg.ranges.size      = 12;
        g_impl->laser_scan_msg.intensities.size = 12;
        _pub_laser_scan();
    } else {
        static uint32_t skip_lidar = 0;
        if ((++skip_lidar % 100) == 1)
            ESP_LOGD(TAG, "sensor_control_timer: no valid LiDAR packet (skip #%lu)",
                     (unsigned long)skip_lidar);
    }
}

// ---------------------------------------------------------------------------
// Subscription callbacks
// ---------------------------------------------------------------------------
static void motor_command_cb(const void* msg) {
    const auto* cmd = static_cast<const std_msgs__msg__Float32MultiArray*>(msg);
    const size_t n  = std::min((size_t)NUM_MOTORS, cmd->data.size);
    for (size_t i = 0; i < n; ++i)
        motor_control_set_position(i, cmd->data.data[i]);
}

static void set_speed_cb(const void* msg) {
    const auto* spd = static_cast<const std_msgs__msg__Float32MultiArray*>(msg);
    const size_t n  = std::min((size_t)NUM_MOTORS, spd->data.size);
    for (size_t i = 0; i < n; ++i)
        motor_control_set_velocity(i, spd->data.data[i]);
}

static void led_cb(const void* msg) {
    const auto* led = static_cast<const std_msgs__msg__Bool*>(msg);
    led_control_set(led->data);
    if (is_connected()) {
        g_impl->led_state_msg.data = led->data;
        _pub_led_state();
    }
}

// ---------------------------------------------------------------------------
// Entity management
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
        ESP_LOGE(TAG, "support_fini failed: %ld (%s)", (long)r,
                 rcl_get_error_string().str);
        rcl_reset_error();
    }
    memset(&impl.support, 0, sizeof(impl.support));
    return (r == RCL_RET_OK);
}

static bool destroy_entities(MicrorosSyncImpl& impl) {
    ESP_LOGI(TAG, "Destroying micro-ROS entities");
    bool ok = true;
    rcl_ret_t r;

#define FINI_CHECK(call, label) \
    r = (call); \
    if (r != RCL_RET_OK) { \
        ESP_LOGE(TAG, label " fini failed: %ld (%s)", (long)r, rcl_get_error_string().str); \
        rcl_reset_error(); ok = false; \
    }

    FINI_CHECK(rclc_executor_fini(&impl.executor),              "executor")
    FINI_CHECK(rcl_publisher_fini(&impl.heartbeat_pub,        &impl.node), "heartbeat_pub")
    FINI_CHECK(rcl_publisher_fini(&impl.motor_positions_pub,  &impl.node), "motor_positions_pub")
    FINI_CHECK(rcl_publisher_fini(&impl.distance_sensors_pub, &impl.node), "distance_sensors_pub")
    FINI_CHECK(rcl_publisher_fini(&impl.led_state_pub,        &impl.node), "led_state_pub")
    FINI_CHECK(rcl_publisher_fini(&impl.tof_distance_pub,     &impl.node), "tof_distance_pub")
    FINI_CHECK(rcl_publisher_fini(&impl.laser_scan_pub,       &impl.node), "laser_scan_pub")
    FINI_CHECK(rcl_subscription_fini(&impl.motor_command_sub, &impl.node), "motor_command_sub")
    FINI_CHECK(rcl_subscription_fini(&impl.set_speed_sub,     &impl.node), "set_speed_sub")
    FINI_CHECK(rcl_subscription_fini(&impl.led_sub,           &impl.node), "led_sub")
    FINI_CHECK(rcl_timer_fini(&impl.heartbeat_timer),           "heartbeat_timer")
    FINI_CHECK(rcl_timer_fini(&impl.motor_position_timer),      "motor_position_timer")
    FINI_CHECK(rcl_timer_fini(&impl.sensor_control_timer),      "sensor_control_timer")
    FINI_CHECK(rcl_node_fini(&impl.node),                       "node")
#undef FINI_CHECK

    ESP_LOGI(TAG, "destroy_entities: %s", ok ? "all OK" : "errors (see above)");
    return ok;
}

// Returns true only if ALL entities were created.
// On failure: node is finalised, all handles reset to zero.
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
#define PUB_BE(handle, type_pkg, type_name, topic)                          \
    if (ok) {                                                               \
        r = rclc_publisher_init_best_effort(                                \
            &impl.handle, &impl.node,                                       \
            ROSIDL_GET_MSG_TYPE_SUPPORT(type_pkg, msg, type_name), topic);  \
        if (r != RCL_RET_OK) {                                              \
            ESP_LOGE(TAG, #handle " pub init failed: %ld (%s)",             \
                     (long)r, rcl_get_error_string().str);                  \
            rcl_reset_error(); ok = false;                                  \
        }                                                                   \
    }
#define PUB_REL(handle, type_pkg, type_name, topic)                         \
    if (ok) {                                                               \
        r = rclc_publisher_init_default(                                    \
            &impl.handle, &impl.node,                                       \
            ROSIDL_GET_MSG_TYPE_SUPPORT(type_pkg, msg, type_name), topic);  \
        if (r != RCL_RET_OK) {                                              \
            ESP_LOGE(TAG, #handle " pub init failed: %ld (%s)",             \
                     (long)r, rcl_get_error_string().str);                  \
            rcl_reset_error(); ok = false;                                  \
        }                                                                   \
    }
#define SUB_REL(handle, type_pkg, type_name, topic)                         \
    if (ok) {                                                               \
        r = rclc_subscription_init_default(                                 \
            &impl.handle, &impl.node,                                       \
            ROSIDL_GET_MSG_TYPE_SUPPORT(type_pkg, msg, type_name), topic);  \
        if (r != RCL_RET_OK) {                                              \
            ESP_LOGE(TAG, #handle " sub init failed: %ld (%s)",             \
                     (long)r, rcl_get_error_string().str);                  \
            rcl_reset_error(); ok = false;                                  \
        }                                                                   \
    }
#define TIMER(handle, period_ms, cb)                                        \
    if (ok) {                                                               \
        r = rclc_timer_init_default(&impl.handle, &impl.support,            \
            RCL_MS_TO_NS(period_ms), cb);                                   \
        if (r != RCL_RET_OK) {                                              \
            ESP_LOGE(TAG, #handle " timer init failed: %ld (%s)",           \
                     (long)r, rcl_get_error_string().str);                  \
            rcl_reset_error(); ok = false;                                  \
        }                                                                   \
    }

    PUB_BE (heartbeat_pub,        std_msgs, Int32,            "shelfbot_firmware/heartbeat")
    PUB_BE (motor_positions_pub,  std_msgs, Float32MultiArray,"shelfbot_firmware/motor_positions")
    PUB_BE (distance_sensors_pub, std_msgs, Float32MultiArray,"shelfbot_firmware/distance_sensors")
    PUB_BE (led_state_pub,        std_msgs, Bool,             "shelfbot_firmware/led_state")
    PUB_BE (tof_distance_pub,     std_msgs, Float32,          "shelfbot_firmware/tof_distance")
    PUB_REL(laser_scan_pub,  sensor_msgs, LaserScan,          "shelfbot_firmware/laser_scan")
    SUB_REL(motor_command_sub,    std_msgs, Float32MultiArray,"shelfbot_firmware/motor_command")
    SUB_REL(set_speed_sub,        std_msgs, Float32MultiArray,"shelfbot_firmware/set_speed")
    SUB_REL(led_sub,              std_msgs, Bool,             "shelfbot_firmware/led")
    TIMER  (heartbeat_timer,       1000, heartbeat_timer_cb)
    TIMER  (motor_position_timer,   100, motor_position_timer_cb)
    TIMER  (sensor_control_timer,   200, sensor_control_timer_cb)

#undef PUB_BE
#undef PUB_REL
#undef SUB_REL
#undef TIMER

    if (ok) {
        r = rclc_executor_init(&impl.executor, &impl.support.context, 6, &impl.allocator);
        if (r != RCL_RET_OK) {
            ESP_LOGE(TAG, "executor init failed: %ld (%s)", (long)r, rcl_get_error_string().str);
            rcl_reset_error(); ok = false;
        }
    }

    if (ok) {
        bool ex_ok = true;
        ex_ok &= (rclc_executor_add_timer(&impl.executor, &impl.heartbeat_timer)      == RCL_RET_OK);
        ex_ok &= (rclc_executor_add_timer(&impl.executor, &impl.motor_position_timer) == RCL_RET_OK);
        ex_ok &= (rclc_executor_add_timer(&impl.executor, &impl.sensor_control_timer) == RCL_RET_OK);
        ex_ok &= (rclc_executor_add_subscription(&impl.executor, &impl.motor_command_sub,
                      &impl.motor_command_msg, motor_command_cb, ON_NEW_DATA) == RCL_RET_OK);
        ex_ok &= (rclc_executor_add_subscription(&impl.executor, &impl.set_speed_sub,
                      &impl.set_speed_msg, set_speed_cb, ON_NEW_DATA) == RCL_RET_OK);
        ex_ok &= (rclc_executor_add_subscription(&impl.executor, &impl.led_sub,
                      &impl.led_msg, led_cb, ON_NEW_DATA) == RCL_RET_OK);
        if (!ex_ok) {
            ESP_LOGE(TAG, "executor add handle(s) failed");
            rcl_reset_error();
            ok = false;
        }
    }

    if (!ok) {
        // Node was created; fini it before resetting all handles.
        const rcl_ret_t fr = rcl_node_fini(&impl.node);
        if (fr != RCL_RET_OK) {
            ESP_LOGE(TAG, "node fini (cleanup) failed: %ld (%s)", (long)fr,
                     rcl_get_error_string().str);
            rcl_reset_error();
        }
        reset_entity_handles(impl);
        return false;
    }

    ESP_LOGI(TAG, "create_entities: all OK");
    return true;
}

// ---------------------------------------------------------------------------
// mDNS helper
// ---------------------------------------------------------------------------
static bool query_mdns_host(const char* host, char* out_ip, size_t len) {
    ESP_LOGI(TAG, "Querying mDNS for %s.local", host);
    esp_ip4_addr_t addr = { .addr = 0 };
    const esp_err_t err = mdns_query_a(host, 2000, &addr);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "mDNS query for %s failed: %s", host, esp_err_to_name(err));
        return false;
    }
    snprintf(out_ip, len, IPSTR, IP2STR(&addr));
    ESP_LOGI(TAG, "Agent IP: %s", out_ip);
    return true;
}

// ---------------------------------------------------------------------------
// Clock synchronisation
//
// Try rmw_uros_sync_session() first (requires agent started with --time).
// Fall back to SNTP.  Returns true if wall clock is now valid.
//
// IMPORTANT: this function is called AFTER rclc_support_init so the agent
// session transport is already alive when we call rmw_uros_sync_session().
// ---------------------------------------------------------------------------
static bool sync_clock_with_agent() {
    ESP_LOGI(TAG, "Attempting clock sync via micro-ROS agent (rmw_uros_sync_session)");
    const rmw_ret_t r = rmw_uros_sync_session(SYNC_TIMEOUT_MS);
    if (r == RMW_RET_OK && shelfbot::ShelfbotTimestamp::isEpochValid()) {
        int32_t sec; uint32_t ns;
        shelfbot::ShelfbotTimestamp::toRosTime(
            shelfbot::ShelfbotTimestamp::epochMicros(), sec, ns);
        ESP_LOGI(TAG, "Clock synced via micro-ROS agent — wall time %ld.%09lu",
                 (long)sec, (unsigned long)ns);
        return true;
    }
    ESP_LOGW(TAG, "Agent clock sync failed (rmw=%d) — falling back to SNTP", (int)r);
    return false;
}

static bool wait_for_sntp(uint32_t timeout_ms) {
    ESP_LOGI(TAG, "Waiting for SNTP (timeout %lu ms)", (unsigned long)timeout_ms);
    const TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms);
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
        if ((++polls % 20) == 0)
            ESP_LOGI(TAG, "Waiting for SNTP... (%lu ms elapsed)",
                     (unsigned long)(polls * EPOCH_POLL_MS));
        vTaskDelay(pdMS_TO_TICKS(EPOCH_POLL_MS));
    }
    return false;
}

// ---------------------------------------------------------------------------
// microros_task
// ---------------------------------------------------------------------------
static void microros_task(void* arg) {
    auto* impl = static_cast<MicrorosSyncImpl*>(arg);
    if (!impl) { vTaskDelete(nullptr); return; }

    char  agent_ip[16] = {};
    bool  support_inited = false;
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    bool  init_options_inited = false;
    uint32_t backoff_ms = BACKOFF_INITIAL_MS;

    // Helper: full teardown to a clean disconnected state
    auto full_teardown = [&](const char* reason) {
        ESP_LOGW(TAG, "full_teardown: %s", reason);
        if (StateMachine::isAtLeast("microros_sync",
                                    stateToString(MicrorosState::CREATING_ENTITIES))) {
            destroy_entities(*impl);
            reset_entity_handles(*impl);
        }
        if (support_inited) {
            safe_destroy_support(*impl);
            support_inited = false;
        }
        if (init_options_inited) {
            rcl_ret_t fini_ret = rcl_init_options_fini(&init_options);
            if (fini_ret != RCL_RET_OK) {
                ESP_LOGE(TAG, "rcl_init_options_fini in teardown failed: %ld (%s)",
                         (long)fini_ret, rcl_get_error_string().str);
                rcl_reset_error();
            }
            init_options = rcl_get_zero_initialized_init_options();
            init_options_inited = false;
        }
        impl->pub_fail_count = 0;
        // force_skip_prereqs=true: teardown must always succeed regardless of
        // what other modules are doing.
        StateMachine::changeState("microros_sync",
                                  stateToString(MicrorosState::DISCONNECTED),
                                  /*force_skip_prereqs=*/true);
        StateMachine::changeState("agent",
                                  stateToString(AgentState::OFFLINE),
                                  /*force_skip_prereqs=*/true);
    };

    ESP_LOGI(TAG, "MicrorosSync task started, waiting for prerequisites");

    while (true) {
        // ------------------------------------------------------------------
        // Wait for wifi + mDNS before attempting discovery
        // ------------------------------------------------------------------
        if (!StateMachine::waitForPrerequisites("microros_sync",
                stateToString(MicrorosState::DISCOVERING), 60000, 500)) {
            ESP_LOGW(TAG, "Prerequisites for discovering not met within 60s, retrying");
            backoff_ms = std::min(BACKOFF_MAX_MS, backoff_ms * 2);
            vTaskDelay(pdMS_TO_TICKS(backoff_ms));
            continue;
        }
        StateMachine::changeState("microros_sync",
                                  stateToString(MicrorosState::DISCOVERING));

        // ------------------------------------------------------------------
        // mDNS discovery
        // ------------------------------------------------------------------
        if (!query_mdns_host(CONFIG_MICROROS_AGENT_MDNS_HOST,
                             agent_ip, sizeof(agent_ip))) {
            ESP_LOGW(TAG, "Agent mDNS not found, backing off %lu ms",
                     (unsigned long)backoff_ms);
            full_teardown("mDNS not found");
            vTaskDelay(pdMS_TO_TICKS(backoff_ms));
            backoff_ms = std::min(BACKOFF_MAX_MS, backoff_ms * 2);
            continue;
        }
        StateMachine::changeState("agent", stateToString(AgentState::DISCOVERED),
                                  /*force*/true);

        // ------------------------------------------------------------------
        // rcl init options + support init
        // ------------------------------------------------------------------
        init_options = rcl_get_zero_initialized_init_options();
        rcl_ret_t r = rcl_init_options_init(&init_options, impl->allocator);
        if (r != RCL_RET_OK) {
            ESP_LOGE(TAG, "rcl_init_options_init failed: %ld", (long)r);
            rcl_reset_error();
            full_teardown("init_options_init failed");
            vTaskDelay(pdMS_TO_TICKS(backoff_ms));
            backoff_ms = std::min(BACKOFF_MAX_MS, backoff_ms * 2);
            continue;
        }
        init_options_inited = true;

        const rmw_ret_t rmw_r = rmw_uros_options_set_udp_address(
            agent_ip, "8888",
            rcl_init_options_get_rmw_init_options(&init_options));
        if (rmw_r != RMW_RET_OK) {
            ESP_LOGE(TAG, "rmw set_udp_address failed: %d", (int)rmw_r);
            rcl_ret_t fini_ret = rcl_init_options_fini(&init_options);
            if (fini_ret != RCL_RET_OK) {
                ESP_LOGE(TAG, "rcl_init_options_fini after set_udp_address failed: %ld (%s)",
                         (long)fini_ret, rcl_get_error_string().str);
                rcl_reset_error();
            }
            init_options = rcl_get_zero_initialized_init_options();
            init_options_inited = false;
            full_teardown("set_udp_address failed");
            vTaskDelay(pdMS_TO_TICKS(backoff_ms));
            backoff_ms = std::min(BACKOFF_MAX_MS, backoff_ms * 2);
            continue;
        }

        const rcl_ret_t support_r = rclc_support_init_with_options(
            &impl->support, 0, nullptr, &init_options, &impl->allocator);
        // init_options consumed; fini regardless of result
        rcl_ret_t fini_ret = rcl_init_options_fini(&init_options);
        if (fini_ret != RCL_RET_OK) {
            ESP_LOGE(TAG, "rcl_init_options_fini after support_init failed: %ld (%s)",
                     (long)fini_ret, rcl_get_error_string().str);
            rcl_reset_error();
        }
        init_options = rcl_get_zero_initialized_init_options();
        init_options_inited = false;

        if (support_r != RCL_RET_OK) {
            ESP_LOGE(TAG, "rclc_support_init failed: %ld (%s)",
                     (long)support_r, rcl_get_error_string().str);
            rcl_reset_error();
            safe_destroy_support(*impl);  // partial init — always fini
            full_teardown("support_init failed");
            vTaskDelay(pdMS_TO_TICKS(backoff_ms));
            backoff_ms = std::min(BACKOFF_MAX_MS, backoff_ms * 2);
            continue;
        }
        support_inited = true;
        ESP_LOGI(TAG, "rclc_support_init OK");

        // brief settle — let the DDS participant announce itself
        vTaskDelay(pdMS_TO_TICKS(200));

        // ------------------------------------------------------------------
        // Ping
        // ------------------------------------------------------------------
        if (rmw_uros_ping_agent(500, 3) != RMW_RET_OK) {
            ESP_LOGE(TAG, "Agent ping failed after support init — agent down or wrong port?");
            full_teardown("ping failed");
            vTaskDelay(pdMS_TO_TICKS(backoff_ms));
            backoff_ms = std::min(BACKOFF_MAX_MS, backoff_ms * 2);
            continue;
        }
        StateMachine::changeState("agent", stateToString(AgentState::PING_OK), true);
        ESP_LOGI(TAG, "Agent ping OK");

        // ------------------------------------------------------------------
        // Clock sync — try agent first, then SNTP fallback.
        // MUST happen after support_init so rmw_uros_sync_session can work.
        // ------------------------------------------------------------------
        bool time_synced = shelfbot::ShelfbotTimestamp::isEpochValid();
        if (!time_synced) {
            time_synced = sync_clock_with_agent();
        }
        if (!time_synced) {
            time_synced = wait_for_sntp(EPOCH_WAIT_TIMEOUT_MS);
        }
        if (!time_synced) {
            ESP_LOGE(TAG, "Clock sync failed (neither agent nor SNTP succeeded). "
                     "Sensor topics with timestamps will NOT publish.");
            // Do NOT tear down — heartbeat and led_state can still work.
            // Log clearly so the user knows what is broken and why.
        } else {
            StateMachine::changeState("time_sync",
                                      stateToString(TimeSyncState::SYNCED),
                                      /*force*/true);
        }

        // ------------------------------------------------------------------
        // Session sync (agent state machine)
        // ------------------------------------------------------------------
        {
            const rmw_ret_t sess_r = rmw_uros_sync_session(SYNC_TIMEOUT_MS);
            if (sess_r != RMW_RET_OK) {
                ESP_LOGE(TAG, "rmw_uros_sync_session failed: %d", (int)sess_r);
                full_teardown("session sync failed");
                vTaskDelay(pdMS_TO_TICKS(backoff_ms));
                backoff_ms = std::min(BACKOFF_MAX_MS, backoff_ms * 2);
                continue;
            }
        }
        StateMachine::changeState("agent", stateToString(AgentState::SESSION_SYNCED), true);
        ESP_LOGI(TAG, "Session established");

        // ------------------------------------------------------------------
        // Second ping — agent could have dropped during the sync window
        // ------------------------------------------------------------------
        if (rmw_uros_ping_agent(300, 2) != RMW_RET_OK) {
            ESP_LOGE(TAG, "Agent gone after session sync");
            full_teardown("ping failed after session sync");
            vTaskDelay(pdMS_TO_TICKS(backoff_ms));
            backoff_ms = std::min(BACKOFF_MAX_MS, backoff_ms * 2);
            continue;
        }

        // ------------------------------------------------------------------
        // Check prerequisites for creating_entities (time_sync must be synced
        // for timestamped topics — optional, we already logged above)
        // ------------------------------------------------------------------
        StateMachine::changeState("microros_sync",
                                  stateToString(MicrorosState::CREATING_ENTITIES));

        if (!create_entities(*impl)) {
            ESP_LOGE(TAG, "create_entities failed");
            full_teardown("create_entities failed");
            vTaskDelay(pdMS_TO_TICKS(backoff_ms));
            backoff_ms = std::min(BACKOFF_MAX_MS, backoff_ms * 2);
            continue;
        }

        StateMachine::changeState("agent", stateToString(AgentState::ENTITIES_CREATED), true);
        StateMachine::changeState("microros_sync", stateToString(MicrorosState::CONNECTED));
        StateMachine::changeState("agent", stateToString(AgentState::CONNECTED), true);

        impl->pub_fail_count = 0;
        backoff_ms           = BACKOFF_INITIAL_MS;  // reset on successful connect
        ESP_LOGI(TAG, "micro-ROS fully connected and ready (time_synced=%s)",
                 time_synced ? "yes" : "no");

        // ------------------------------------------------------------------
        // Spin loop
        // ------------------------------------------------------------------
        int consecutive_spin_failures = 0;
        while (true) {
            // Wi-Fi watchdog
            if (!(xEventGroupGetBits(wifi_manager_get_event_group()) & WM_CONNECTED_BIT)) {
                ESP_LOGW(TAG, "Wi-Fi lost while connected — tearing down");
                break;
            }

            const rcl_ret_t spin_r =
                rclc_executor_spin_some(&impl->executor, RCL_MS_TO_NS(100));

            if (spin_r != RCL_RET_OK) {
                ++consecutive_spin_failures;
                if ((consecutive_spin_failures % 5) == 0)
                    ESP_LOGW(TAG, "Spin failure #%d (threshold %d)",
                             consecutive_spin_failures, SPIN_FAIL_THRESHOLD);
                if (consecutive_spin_failures >= SPIN_FAIL_THRESHOLD) {
                    ESP_LOGE(TAG, "Spin failure threshold reached — agent likely crashed");
                    break;
                }
            } else {
                // Healthy spin: decay the failure counter
                consecutive_spin_failures =
                    std::max(0, consecutive_spin_failures - SPIN_RECOVERY_CREDIT);

                // Check publish failure accumulator (written by executor callbacks,
                // read here in the same task — no lock needed)
                const uint32_t pf = impl->pub_fail_count;
                impl->pub_fail_count = 0;   // single-writer reset, executor task only
                if (pf >= PUB_FAIL_RECONNECT_THRESHOLD) {
                    ESP_LOGE(TAG, "Pub failure count %lu >= %lu — DDS writer broken",
                             (unsigned long)pf,
                             (unsigned long)PUB_FAIL_RECONNECT_THRESHOLD);
                    break;
                }
            }

            vTaskDelay(pdMS_TO_TICKS(10));
        }

        // ------------------------------------------------------------------
        // Teardown after spin exit
        // ------------------------------------------------------------------
        full_teardown("spin loop exited");
        // If we were time-synced, the sync is only for this session; reset
        // so the next reconnect re-validates.
        StateMachine::changeState("time_sync",
                                  stateToString(TimeSyncState::UNSYNCED),
                                  /*force*/true);
        vTaskDelay(pdMS_TO_TICKS(backoff_ms));
        backoff_ms = std::min(BACKOFF_MAX_MS, backoff_ms * 2);
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

    // Register all modules owned by microros_sync
    StateMachine::setInitial("microros_sync",
                             stateToString(MicrorosState::DISCONNECTED),
                             orderedStates(MicrorosState::DISCONNECTED));
    StateMachine::setInitial("time_sync",
                             stateToString(TimeSyncState::UNSYNCED),
                             orderedStates(TimeSyncState::UNSYNCED));
    StateMachine::setInitial("agent",
                             stateToString(AgentState::OFFLINE),
                             orderedStates(AgentState::OFFLINE));

    // ------------------------------------------------------------------
    // Register ALL prerequisites here, before start() is called.
    // This guarantees prereqs are visible the moment the task starts.
    // ------------------------------------------------------------------

    // microros_sync can only enter DISCOVERING when:
    //   1. wifi_manager is connected (has IP)
    //   2. network_service has mDNS running (so mdns_query_a will work)
    //   3. microros_sync itself is in a safe stopped state (>= DISCONNECTED)
    //      This prevents jumping to DISCOVERING from OFF on first boot.
    StateMachine::registerPrerequisite("microros_sync",
        stateToString(MicrorosState::DISCOVERING),
        { "wifi_manager", stateToString(WifiManagerState::CONNECTED) });
    StateMachine::registerPrerequisite("microros_sync",
        stateToString(MicrorosState::DISCOVERING),
        { "network_service", stateToString(NetworkServiceState::MDNS_READY) });
    StateMachine::registerPrerequisite("microros_sync",
        stateToString(MicrorosState::DISCOVERING),
        { "microros_sync", stateToString(MicrorosState::DISCONNECTED) });

    ESP_LOGI(TAG, "MicrorosSync initialised");
    return true;
}

void MicrorosSync::start() {
    if (!g_impl || g_impl->task_handle) return;
    const BaseType_t r = xTaskCreate(
        microros_task, "microros_task", 24576, g_impl, 5, &g_impl->task_handle);
    if (r != pdPASS) {
        ESP_LOGE(TAG, "xTaskCreate failed");
        g_impl->task_handle = nullptr;
        return;
    }
    ESP_LOGI(TAG, "MicrorosSync task started");
}

// Public publish helpers — safe to call from any task; guard via SM state
void MicrorosSync::publishHeartbeat(int32_t value) {
    if (!lock_impl()) return;
    g_impl->heartbeat_msg.data = value;
    unlock_impl();
    _pub_heartbeat();
}

void MicrorosSync::publishMotorPositions(const float* positions, size_t count) {
    if (!lock_impl()) return;
    const size_t n = std::min(count, (size_t)NUM_MOTORS);
    for (size_t i = 0; i < n; ++i) g_impl->motor_positions_data[i] = positions[i];
    g_impl->motor_positions_msg.data.size = n;
    unlock_impl();
    _pub_motor_positions();
}

void MicrorosSync::publishDistanceSensors(const float* distances, size_t count) {
    if (!lock_impl()) return;
    const size_t n = std::min(count, (size_t)SensorCommon::NUM_SENSORS);
    for (size_t i = 0; i < n; ++i) g_impl->distance_sensors_data[i] = distances[i];
    g_impl->distance_sensors_msg.data.size = n;
    unlock_impl();
    _pub_distance_sensors();
}

void MicrorosSync::publishLedState(bool state) {
    if (!lock_impl()) return;
    g_impl->led_state_msg.data = state;
    unlock_impl();
    _pub_led_state();
}

void MicrorosSync::publishTofDistance(float distance_m) {
    if (!lock_impl()) return;
    g_impl->tof_distance_msg.data = distance_m;
    unlock_impl();
    _pub_tof_distance();
}

void MicrorosSync::publishLidarScan(const SensorCommon::LidarMeasurement& /*m*/) {
    // LaserScan is published by sensor_control_timer_cb with proper ROS timestamps.
    // This callback stub exists so shelfbot.cpp can register a lidar_callback
    // without changes; the actual publish happens via the executor timer.
}