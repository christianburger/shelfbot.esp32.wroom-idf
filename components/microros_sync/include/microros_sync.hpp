#pragma once
#include <idf_c_includes.hpp>
#include <sensor_common.hpp>

class MicrorosSync {
public:
    static MicrorosSync& getInstance();
    bool init();
    void start();

    static void publishHeartbeat(int32_t value);
    static void publishMotorPositions(const float* positions, size_t count);
    static void publishDistanceSensors(const float* distances, size_t count);
    static void publishLedState(bool state);
    static void publishTofDistance(float distance_m);
    static void publishLidarScan(const SensorCommon::LidarMeasurement& measurement);

private:
    MicrorosSync();
    ~MicrorosSync();
};