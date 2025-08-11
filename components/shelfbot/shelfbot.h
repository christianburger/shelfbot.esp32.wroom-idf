#pragma once

#include <rcl/publisher.h>
#include <std_msgs/msg/int32.h>
#include <rcl/timer.h>
#include <time.h> // for struct timeval

class Shelfbot {
public:
    void begin();

private:
    // Member variables
    char agent_ip_str[16];

    // Static members for C callbacks
    static bool time_synchronized;
    static rcl_publisher_t heartbeat_publisher;
    static std_msgs__msg__Int32 heartbeat_msg;

    // Helper methods
    void initialise_mdns();
    bool query_mdns_host(const char * host_name);
    void initialize_sntp();
    void micro_ros_task_impl();

    // Static callbacks for C APIs
    static void time_sync_notification_cb(struct timeval *tv);
    static void heartbeat_timer_callback(rcl_timer_t * timer, int64_t last_call_time);
    static void micro_ros_task_wrapper(void * arg);
};
