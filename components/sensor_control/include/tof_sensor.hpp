#pragma once

#include <idf_c_includes.hpp>
#include <sensor_common.hpp>
#include <sensor_interfaces.hpp>
#include <tof_driver_interface.hpp>

class TofSensor : public IToFSensor {
public:
    struct Config {
        struct SingleConfig {
            uint8_t device_id = 0;
            uint32_t timeout_ms = 500;
            bool enabled = true;
        };
        SingleConfig tof_configs[SensorCommon::NUM_TOF_SENSORS];
        uint32_t tof_read_interval_ms = 200;
        std::function<void(const SensorCommon::TofMeasurement*)> tof_callback = nullptr;
    };

    explicit TofSensor(const Config& config);
    ~TofSensor();

    void setDriverFactory(std::function<std::unique_ptr<ITofDriver>(uint8_t)> factory);

    esp_err_t initialize() override;
    bool isReady() const override;
    esp_err_t readAll(SensorCommon::TofMeasurement results[SensorCommon::NUM_TOF_SENSORS]) override;
    esp_err_t readSingle(uint8_t index, SensorCommon::TofMeasurement& result) override;
    esp_err_t startContinuous() override;
    esp_err_t stopContinuous() override;
    bool probe(uint8_t index) const override;
    bool isSensorReady(uint8_t index) const override;

private:
    Config config_;
    bool initialized_ = false;
    bool continuous_mode_ = false;
    std::unique_ptr<ITofDriver> drivers_[SensorCommon::NUM_TOF_SENSORS];
    bool sensor_enabled_[SensorCommon::NUM_TOF_SENSORS];
    SensorCommon::TofMeasurement last_measurements_[SensorCommon::NUM_TOF_SENSORS];
    int64_t last_read_time_us_ = 0;
    std::function<std::unique_ptr<ITofDriver>(uint8_t)> driver_factory_;

    static const char* TAG;
};