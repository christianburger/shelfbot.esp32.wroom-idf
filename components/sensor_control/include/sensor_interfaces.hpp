#pragma once

#include <idf_c_includes.hpp>
#include <sensor_common.hpp>

class IUltrasonicArray {
public:
    virtual ~IUltrasonicArray() = default;
    virtual bool init() = 0;
    virtual bool readAll(std::vector<SensorCommon::Reading>& readings, uint32_t timeout_ms) = 0;
    virtual size_t getSensorCount() const = 0;
    virtual bool add_sensor(uint8_t index, const struct UltrasonicSensorConfig& config) = 0;
};

class IToFSensor {
public:
    virtual ~IToFSensor() = default;
    virtual esp_err_t initialize() = 0;
    virtual bool isReady() const = 0;
    virtual esp_err_t readAll(SensorCommon::TofMeasurement results[SensorCommon::NUM_TOF_SENSORS]) = 0;
    virtual esp_err_t readSingle(uint8_t index, SensorCommon::TofMeasurement& result) = 0;
    virtual esp_err_t startContinuous() = 0;
    virtual esp_err_t stopContinuous() = 0;
    virtual bool probe(uint8_t index) const = 0;
    virtual bool isSensorReady(uint8_t index) const = 0;
};

class ILidarSensor {
public:
    virtual ~ILidarSensor() = default;
    virtual esp_err_t initialize() = 0;
    virtual bool isReady() const = 0;
    virtual esp_err_t read(SensorCommon::LidarMeasurement& measurement) = 0;
    virtual bool getLastRawPacket(uint8_t* out, size_t len) const = 0;
    virtual uint32_t getPacketCount() const = 0;
};