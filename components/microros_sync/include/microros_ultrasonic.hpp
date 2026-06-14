#pragma once
#include "microros_common.hpp"
#include <std_msgs/msg/float32_multi_array.h>

// Maximum number of distance channels published in one message.
// Matches the previous SensorCommon::NUM_SENSORS (4 ultrasonic + 1 ToF + 1 LiDAR).
static constexpr size_t ULTRASONIC_MAX_CHANNELS = 6;

class UltrasonicComponent {
public:
    UltrasonicComponent();
    bool init(rcl_node_t* node, rclc_executor_t* executor);
    bool fini(rcl_node_t* node);

    /**
     * @brief Publish an array of distance readings (centimetres).
     *
     * Pure push: the caller owns the data and decides when to publish.
     * No internal timer or SensorManager reference.
     *
     * @param distances  Array of distances in centimetres.
     * @param count      Number of elements; capped at ULTRASONIC_MAX_CHANNELS.
     */
    void publishDistances(const float* distances, size_t count);

private:
    rcl_publisher_t pub_;
    std_msgs__msg__Float32MultiArray msg_;
    float data_[ULTRASONIC_MAX_CHANNELS];
    static std_msgs__msg__MultiArrayDimension dim_[1];
};
