#pragma once
#include <idf_c_includes.hpp>

// Forward declarations
class UltrasonicSensorArray;

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

        // Reading intervals
        uint32_t ultrasonic_read_interval_ms = 100;
        uint32_t tof_read_interval_ms = 200;

        // Callbacks
        std::function<void(const std::vector<uint16_t>&)> ultrasonic_callback = nullptr;
        std::function<void(uint16_t, bool, uint8_t)> tof_callback = nullptr;
    };

    SensorControl(const Config& config);
    ~SensorControl();

    // Initialization
    esp_err_t initialize();
    bool is_ready() const;

    // Reading methods
    esp_err_t read_ultrasonic(std::vector<uint16_t>& distances);
    esp_err_t read_tof(uint16_t& distance_mm, bool& valid, uint8_t& status);

    esp_err_t read_all(std::vector<uint16_t>& ultrasonic_distances,
                      uint16_t& tof_distance_mm, bool& tof_valid, uint8_t& tof_status);

    // Continuous mode
    esp_err_t start_continuous();
    esp_err_t stop_continuous();
    bool is_continuous() const;

    // Sensor control
    esp_err_t set_tof_mode(bool long_distance);

    // Status
    size_t get_ultrasonic_count() const;
    bool is_tof_ready() const;
    bool is_ultrasonic_ready() const;

    // Diagnostics
    esp_err_t self_test();
    bool tof_probe();

private:
    Config config_;

    // State - REORDERED to match constructor initialization order
    std::unique_ptr<UltrasonicSensorArray> ultrasonic_array_;
    class TofSensorImpl* tof_sensor_;  // Using pointer to avoid including header
    bool initialized_;
    bool continuous_mode_;
    TaskHandle_t continuous_task_handle_;

    // Continuous reading task
    static void continuous_read_task(void* arg);
    void continuous_read_loop();

    // Internal helpers
    esp_err_t initialize_ultrasonic();
    esp_err_t initialize_tof();

    static const char* TAG;

    SensorControl(const SensorControl&) = delete;
    SensorControl& operator=(const SensorControl&) = delete;
};