#pragma once
#include <idf_c_includes.hpp>
#include <sensor_common.hpp>
#include <sensor_control.hpp>
#include <motor_control.hpp>
#include <led_control.hpp>
#include <../../wifi_manager/include/wifi_manager.hpp>
#include <http_server.hpp>
#include <firmware_version.hpp>

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){ESP_LOGE(TAG, "RCL error in %s: %d", #fn, (int)temp_rc);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){ESP_LOGW(TAG, "RCL soft error in %s: %d", #fn, (int)temp_rc);}}

class Shelfbot {
public:
    esp_err_t begin();                                  // now returns error code
    static Shelfbot& get_instance() {
        if (!instance) instance = new Shelfbot();
        return *instance;
    }

    void initialise_mdns();
    void initialize_sntp();

    static bool time_synchronized;
    static bool led_state;

private:
    Shelfbot() = default;

    enum MicroRosState {
        WAITING_AGENT,
        AGENT_AVAILABLE,
        AGENT_CONNECTED,
        AGENT_DISCONNECTED
    };

    MicroRosState state = WAITING_AGENT;
    char agent_ip_str[16]{};

    rcl_node_t node{};
    rcl_allocator_t allocator{};
    rclc_support_t support{};
    rclc_executor_t executor{};

    rcl_publisher_t heartbeat_publisher{};
    rcl_publisher_t motor_position_publisher{};
    rcl_publisher_t distance_sensors_publisher{};
    rcl_publisher_t led_state_publisher{};
    rcl_publisher_t tof_distance_publisher{};

    rcl_subscription_t motor_command_subscription{};
    rcl_subscription_t set_speed_subscription{};
    rcl_subscription_t led_subscription{};

    rcl_timer_t heartbeat_timer{};
    rcl_timer_t motor_position_timer{};
    rcl_timer_t distance_sensors_timer{};
    rcl_timer_t led_state_timer{};
    rcl_timer_t tof_timer{};

    std_msgs__msg__Int32 heartbeat_msg{};
    std_msgs__msg__Float32MultiArray motor_position_msg{};
    std_msgs__msg__Float32MultiArray distance_sensors_msg{};
    std_msgs__msg__Bool led_state_msg{};
    std_msgs__msg__Float32 tof_distance_msg{};

    std_msgs__msg__Float32MultiArray motor_command_msg{};
    std_msgs__msg__Float32MultiArray set_speed_msg{};
    std_msgs__msg__Bool led_msg{};

    float motor_pos_data[NUM_MOTORS]{};
    float distance_sensors_data[SensorCommon::NUM_SENSORS]{};

    std::unique_ptr<SensorControl> sensor_control_ = {

    };
    bool query_mdns_host(const char * host_name);

    void create_entities();
    void destroy_entities();
    void micro_ros_task_impl();

    static Shelfbot* instance;

    static void time_sync_notification_cb(struct timeval *tv);
    static void heartbeat_timer_callback_wrapper(rcl_timer_t * timer, int64_t last_call_time);
    static void motor_position_timer_callback_wrapper(rcl_timer_t * timer, int64_t last_call_time);
    static void distance_sensors_timer_callback_wrapper(rcl_timer_t * timer, int64_t last_call_time);
    static void led_state_timer_callback_wrapper(rcl_timer_t * timer, int64_t last_call_time);
    static void tof_timer_callback_wrapper(rcl_timer_t * timer, int64_t last_call_time);
    static void motor_command_subscription_callback_wrapper(const void * msin);
    static void set_speed_subscription_callback_wrapper(const void * msin);
    static void led_subscription_callback_wrapper(const void * msin);
    static void micro_ros_task_wrapper(void * arg);

    void heartbeat_timer_callback(rcl_timer_t * timer, int64_t last_call_time);
    void motor_position_timer_callback(rcl_timer_t * timer, int64_t last_call_time);
    void distance_sensors_timer_callback(rcl_timer_t * timer, int64_t last_call_time);
    void led_state_timer_callback(rcl_timer_t * timer, int64_t last_call_time);
    void tof_timer_callback(rcl_timer_t * timer, int64_t last_call_time);
    void motor_command_subscription_callback(const void * msin);
    void set_speed_subscription_callback(const void * msin);
    void led_subscription_callback(const void * msin);
};
