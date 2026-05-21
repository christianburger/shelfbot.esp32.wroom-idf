#pragma once
#include <idf_c_includes.hpp>

struct TofDriverMeasurement {
    uint16_t distance_mm;
    bool valid;
    uint8_t range_status;
    int64_t timestamp_us;
    bool timeout_occurred;
};

class ITofDriver {
public:
    virtual ~ITofDriver() = default;
    virtual const char* configure() = 0;
    virtual const char* init() = 0;
    virtual const char* setup() = 0;
    virtual const char* calibrate() = 0;
    virtual const char* check() = 0;
    virtual bool isReady() const = 0;
    virtual bool readSensor(TofDriverMeasurement& result) = 0;
};