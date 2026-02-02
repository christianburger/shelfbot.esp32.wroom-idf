// [file name]: tof_sensor.hpp
#pragma once
#include <idf_c_includes.hpp>
#include "sensor_common.hpp"

/**
 * @brief Time-of-Flight (ToF) distance sensor manager for multiple sensors
 */
class TofSensor {
public:
    /**
     * @brief Sensor operation modes
     */
    enum class Mode {
        HIGH_PRECISION,   ///< Short range mode: ~1.3m range, 30ms measurement period
        LONG_DISTANCE     ///< Long range mode: ~4.0m range, 200ms measurement period
    };

    struct Config {
        // Per-sensor configuration
        struct SensorConfig {
            Mode mode = Mode::LONG_DISTANCE;
            uint16_t timeout_ms = 500;
            uint8_t device_address = 0x01;  // Modbus slave address

            // Communication pins (UART for Modbus)
            int uart_port = 1;
            int uart_tx_pin = 17;
            int uart_rx_pin = 16;

            bool enabled = true;
        };

        SensorConfig sensors[SensorCommon::NUM_TOF_SENSORS];
        uint16_t poll_interval_ms = 100;  // How often to read sensors
    };

    TofSensor(const Config& config);
    ~TofSensor();

    // ===== Core Interface =====
    esp_err_t initialize();
    bool is_ready() const;
    bool is_sensor_ready(uint8_t sensor_index) const;

    // Read all ToF sensors
    esp_err_t read_all(SensorCommon::TofMeasurement results[SensorCommon::NUM_TOF_SENSORS]);

    // Read single sensor
    esp_err_t read_sensor(uint8_t sensor_index, SensorCommon::TofMeasurement& result);

    // ===== Configuration Interface =====
    esp_err_t set_mode(uint8_t sensor_index, Mode mode);
    Mode get_mode(uint8_t sensor_index) const;

    esp_err_t set_timeout(uint8_t sensor_index, uint16_t timeout_ms);
    uint16_t get_timeout(uint8_t sensor_index) const;

    // ===== Continuous Mode Interface =====
    esp_err_t start_continuous();
    esp_err_t stop_continuous();
    bool is_continuous() const;

    // ===== Diagnostic Interface =====
    esp_err_t self_test(uint8_t sensor_index);
    bool probe(uint8_t sensor_index);
    bool timeout_occurred(uint8_t sensor_index);

    // ===== Sensor Control =====
    bool enable_sensor(uint8_t sensor_index, bool enable);
    bool is_sensor_enabled(uint8_t sensor_index) const;

private:
    Config config_;
    bool initialized_;
    bool continuous_mode_;

    // Driver handles - opaque pointers to avoid including driver headers
    void* drivers_[SensorCommon::NUM_TOF_SENSORS];
    bool sensor_enabled_[SensorCommon::NUM_TOF_SENSORS];
    Mode current_modes_[SensorCommon::NUM_TOF_SENSORS];

    // Internal state
    SensorCommon::TofMeasurement last_measurements_[SensorCommon::NUM_TOF_SENSORS];
    int64_t last_read_time_us_;

    // Internal helpers
    esp_err_t initialize_driver(uint8_t sensor_index);
    esp_err_t destroy_driver(uint8_t sensor_index);

    static const char* TAG;

    // Disable copying
    TofSensor(const TofSensor&) = delete;
    TofSensor& operator=(const TofSensor&) = delete;
};