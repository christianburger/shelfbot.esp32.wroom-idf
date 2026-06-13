#include "microros_ultrasonic.hpp"
#include <sensor_manager.hpp>

std_msgs__msg__MultiArrayDimension UltrasonicComponent::dim_[1] = {
    { {const_cast<char*>(""), 0, 1}, SensorCommon::NUM_SENSORS, SensorCommon::NUM_SENSORS }
};
UltrasonicComponent* UltrasonicComponent::s_instance = nullptr;

void UltrasonicComponent::timerCallback(rcl_timer_t*, int64_t) {
    if (!isMicrorosConnected() || !isTimeSynced()) return;
    if (lockCore() && s_instance) {
        SensorCommon::SensorDataPacket data;
        if (SensorManager::get_instance().get_latest_data(data)) {
            size_t idx = 0;
            for (int i = 0; i < SensorCommon::NUM_ULTRASONIC_SENSORS && idx < SensorCommon::NUM_SENSORS; ++i, ++idx)
                s_instance->data_[idx] = data.ultrasonic_readings[i].distance_cm;
            for (int i = 0; i < SensorCommon::NUM_TOF_SENSORS && idx < SensorCommon::NUM_SENSORS; ++i, ++idx)
                s_instance->data_[idx] = data.tof_measurements[i].distance_mm / 10.0f;
            if (idx < SensorCommon::NUM_SENSORS)
                s_instance->data_[idx++] = data.lidar_measurement.valid ? (data.lidar_measurement.distance_mm / 10.0f) : -1.0f;
            s_instance->msg_.data.size = idx;
            publish_or_fail(&s_instance->pub_, &s_instance->msg_, "distance_sensors");
        }
        unlockCore();
    }
}

UltrasonicComponent::UltrasonicComponent() {
    std_msgs__msg__Float32MultiArray__init(&msg_);
    msg_.layout.dim.data = dim_;
    msg_.layout.dim.size = 1;
    msg_.data.data = data_;
    msg_.data.capacity = SensorCommon::NUM_SENSORS;
    pub_ = rcl_get_zero_initialized_publisher();
    timer_ = rcl_get_zero_initialized_timer();
}

bool UltrasonicComponent::init(rcl_node_t* node, rclc_support_t* support, rclc_executor_t* executor) {
    s_instance = this;
    rcl_ret_t r;

    r = rclc_publisher_init_best_effort(&pub_, node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
            "shelfbot_firmware/distance_sensors");
    if (r != RCL_RET_OK) {
        ESP_LOGE("UltrasonicComponent", "pub init failed: %ld", (long)r);
        return false;
    }

    r = rclc_timer_init_default(&timer_, support, RCL_MS_TO_NS(200), timerCallback);
    if (r != RCL_RET_OK) {
        ESP_LOGE("UltrasonicComponent", "timer init failed: %ld", (long)r);
        return false;
    }

    r = rclc_executor_add_timer(executor, &timer_);
    if (r != RCL_RET_OK) {
        ESP_LOGE("UltrasonicComponent", "add_timer failed: %ld", (long)r);
        return false;
    }

    return true;
}

bool UltrasonicComponent::fini(rcl_node_t* node) {
    bool ok = true;
    rcl_ret_t r;

    r = rcl_publisher_fini(&pub_, node);
    if (r != RCL_RET_OK) {
        ESP_LOGE("UltrasonicComponent", "pub fini failed: %ld", (long)r);
        ok = false;
    }
    r = rcl_timer_fini(&timer_);
    if (r != RCL_RET_OK) {
        ESP_LOGE("UltrasonicComponent", "timer fini failed: %ld", (long)r);
        ok = false;
    }

    s_instance = nullptr;
    return ok;
}

void UltrasonicComponent::publishDistances(const float* distances, size_t count) {
    size_t n = std::min(count, (size_t)SensorCommon::NUM_SENSORS);
    for (size_t i = 0; i < n; ++i) data_[i] = distances[i];
    msg_.data.size = n;
    publish_or_fail(&pub_, &msg_, "distance_sensors");
}