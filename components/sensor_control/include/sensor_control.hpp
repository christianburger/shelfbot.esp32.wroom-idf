// [file name]: sensor_control.hpp
#pragma once
#include <idf_c_includes.hpp>
#include "sensor_common.hpp"

// Forward declarations
class UltrasonicSensorArray;
class TofSensor;

class SensorControl {
public:
    struct UltrasonicConfig {
        int trig_pin;
        int echo_pin;
        uint32_t timeout_us;
        uint32_t max_distance_mm;
    };

    struct Config {
        // Ultrasonic sensors configuration
        std::vector<UltrasonicConfig> ultrasonic_configs;

        // ToF sensor configuration
        struct TofConfig {
            int uart_port = 1;
            int tx_pin = 17;
            int rx_pin = 16;
            uint8_t device_address = 0x01;
            uint32_t timeout_ms = 500;
            bool long_distance_mode = true;
        };
        TofConfig tof_configs[SensorCommon::NUM_TOF_SENSORS];

        // Reading intervals
        uint32_t ultrasonic_read_interval_ms = 100;
        uint32_t tof_read_interval_ms = 200;

        // Callbacks (can be nullptr if not needed)
        std::function<void(const std::vector<uint16_t>&)> ultrasonic_callback = nullptr;
        std::function<void(const SensorCommon::TofMeasurement*)> tof_callback = nullptr;
    };

    SensorControl(const Config& config);
    ~SensorControl();

    // Initialization
    esp_err_t initialize();
    bool is_ready() const;

    // Reading methods
    esp_err_t read_ultrasonic(std::vector<uint16_t>& distances);
    esp_err_t read_tof(SensorCommon::TofMeasurement results[SensorCommon::NUM_TOF_SENSORS]);
    esp_err_t read_tof_single(uint8_t sensor_index, SensorCommon::TofMeasurement& result);

    esp_err_t read_all(std::vector<uint16_t>& ultrasonic_distances,
                      SensorCommon::TofMeasurement tof_results[SensorCommon::NUM_TOF_SENSORS]);

    // Continuous mode
    esp_err_t start_continuous();
    esp_err_t stop_continuous();
    bool is_continuous() const;

    // Sensor control
    esp_err_t set_tof_mode(uint8_t sensor_index, bool long_distance);

    // Status
    size_t get_ultrasonic_count() const;
    bool is_tof_ready(uint8_t sensor_index = 0) const;
    bool is_ultrasonic_ready() const;

    // Diagnostics
    esp_err_t self_test();
    bool tof_probe(uint8_t sensor_index = 0);

    // Get latest data for ROS publishing
    bool get_latest_data(SensorCommon::SensorDataPacket* data);

private:
    Config config_;

    // State
    std::unique_ptr<UltrasonicSensorArray> ultrasonic_array_;
    std::unique_ptr<TofSensor> tof_sensor_;
    bool initialized_;
    bool continuous_mode_;
    TaskHandle_t continuous_task_handle_;

    // Latest data storage
    SensorCommon::SensorDataPacket latest_data_;
    SemaphoreHandle_t data_mutex_;

    // Continuous reading task
    static void continuous_read_task(void* arg);
    void continuous_read_loop();

    // Internal helpers
    esp_err_t initialize_ultrasonic();
    esp_err_t initialize_tof();

    static const char* TAG;

    // Disable copying
    SensorControl(const SensorControl&) = delete;
    SensorControl& operator=(const SensorControl&) = delete;
};
