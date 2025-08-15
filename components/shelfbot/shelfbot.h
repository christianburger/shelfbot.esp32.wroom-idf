#pragma once

#include <string.h>
#include <unistd.h>
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
#include "led_control.h"

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/float32_multi_array.h>

#include <time.h> // for struct timeval

class Shelfbot {
public:
    void begin();

private:
    // Member variables
    char agent_ip_str[16];

    // Static members for C callbacks
    static bool time_synchronized;
    
    // ROS Communication Objects
    static rcl_publisher_t heartbeat_publisher;
    static std_msgs__msg__Int32 heartbeat_msg;
    static rcl_subscription_t motor_command_subscriber;
    static std_msgs__msg__Float32MultiArray motor_command_msg;
    static float motor_command_data[NUM_MOTORS];
    static rcl_subscription_t set_speed_subscriber;
    static std_msgs__msg__Float32 set_speed_msg;

    // Helper methods
    void initialise_mdns();
    bool query_mdns_host(const char * host_name);
    void initialize_sntp();
    void micro_ros_task_impl();

    // Static callbacks for C APIs
    static void time_sync_notification_cb(struct timeval *tv);
    static void heartbeat_timer_callback(rcl_timer_t * timer, int64_t last_call_time);
    static void motor_command_subscription_callback(const void * msin);
    static void set_speed_subscription_callback(const void * msin);
    static void micro_ros_task_wrapper(void * arg);
};
