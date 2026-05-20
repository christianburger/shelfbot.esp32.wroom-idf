#pragma once

#include <tof_driver_interface.hpp>
#include <vl53l0x.hpp>

class VL53L0XAdapter : public ITofDriver {
public:
    const char* configure() override { return driver_.configure(); }
    const char* init() override { return driver_.init(); }
    const char* setup() override { return driver_.setup(); }
    const char* calibrate() override { return driver_.calibrate(); }
    const char* check() override { return driver_.check(); }
    bool isReady() const override { return driver_.isReady(); }
    bool readSensor(TofDriverMeasurement& result) override {
        TofDriver::MeasurementResult raw;
        if (!driver_.read_sensor(raw)) return false;
        result.distance_mm = raw.distance_mm;
        result.valid = raw.valid;
        result.range_status = raw.range_status;
        result.timestamp_us = raw.timestamp_us;
        result.timeout_occurred = raw.timeout_occurred;
        return true;
    }
private:
    TofDriver driver_;
};