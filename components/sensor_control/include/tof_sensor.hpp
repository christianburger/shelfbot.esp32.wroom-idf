#pragma once

#include <cstdint>
#include <memory>
#include "esp_err.h"

/**
 * @brief Abstract interface for Time-of-Flight (ToF) distance sensors
 *
 * This interface provides a unified API for various ToF sensors.
 * Currently implemented for VL53L1-based sensors using Modbus/I2C communication.
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

    /**
     * @brief Configuration structure for ToF sensor
     */
    struct Config {
        Mode mode = Mode::LONG_DISTANCE;          ///< Sensor operation mode
        uint16_t timeout_ms = 500;                ///< Communication timeout in milliseconds
        uint8_t device_address = 0x01;            ///< Modbus slave address
        bool enable_continuous = true;            ///< Enable continuous measurement mode
        uint16_t measurement_interval_ms = 200;   ///< Measurement interval in continuous mode

        // Optional: Add I2C/UART configuration if needed
        struct {
            int i2c_port = 0;                     ///< I2C port number
            int sda_pin = 21;                     ///< I2C SDA pin
            int scl_pin = 22;                     ///< I2C SCL pin
            int uart_port = 1;                    ///< UART port number
            int uart_tx_pin = 17;                 ///< UART TX pin
            int uart_rx_pin = 16;                 ///< UART RX pin
        } pins;
    };

    /**
     * @brief Measurement result structure
     */
    struct Measurement {
        uint16_t distance_mm;         ///< Distance measurement in millimeters
        bool valid;                   ///< Whether the measurement is valid
        uint8_t status;               ///< Measurement status code (0 = OK)
        int64_t timestamp_us;         ///< Timestamp in microseconds
        bool timeout_occurred;        ///< Whether a timeout occurred during measurement

        /**
         * @brief Construct a default measurement
         */
        Measurement() : distance_mm(0), valid(false), status(0),
                       timestamp_us(0), timeout_occurred(false) {}
    };

    /**
     * @brief Virtual destructor
     */
    virtual ~TofSensor() = default;

    // ===== Core Interface =====

    /**
     * @brief Initialize the sensor
     * @return ESP_OK on success, error code on failure
     */
    virtual esp_err_t initialize() = 0;

    /**
     * @brief Check if sensor is ready
     * @return true if sensor is initialized and ready, false otherwise
     */
    virtual bool is_ready() const = 0;

    /**
     * @brief Read a single measurement
     * @param result Reference to store the measurement result
     * @return ESP_OK on success, error code on failure
     */
    virtual esp_err_t read_measurement(Measurement& result) = 0;

    // ===== Configuration Interface =====

    /**
     * @brief Set the sensor operation mode
     * @param mode Desired operation mode
     * @return ESP_OK on success, error code on failure
     */
    virtual esp_err_t set_mode(Mode mode) = 0;

    /**
     * @brief Get the current sensor operation mode
     * @return Current operation mode
     */
    virtual Mode get_mode() const = 0;

    /**
     * @brief Set the communication timeout
     * @param timeout_ms Timeout in milliseconds
     * @return ESP_OK on success, error code on failure
     */
    virtual esp_err_t set_timeout(uint16_t timeout_ms) = 0;

    /**
     * @brief Get the current timeout setting
     * @return Timeout in milliseconds
     */
    virtual uint16_t get_timeout() const = 0;

    // ===== Continuous Mode Interface =====

    /**
     * @brief Start continuous measurement mode
     * @return ESP_OK on success, error code on failure
     */
    virtual esp_err_t start_continuous() = 0;

    /**
     * @brief Stop continuous measurement mode
     * @return ESP_OK on success, error code on failure
     */
    virtual esp_err_t stop_continuous() = 0;

    /**
     * @brief Check if continuous mode is active
     * @return true if continuous mode is active, false otherwise
     */
    virtual bool is_continuous() const = 0;

    // ===== Diagnostic Interface =====

    /**
     * @brief Run a self-test on the sensor
     * @return ESP_OK on success, error code on failure
     */
    virtual esp_err_t self_test() = 0;

    /**
     * @brief Probe for sensor presence
     * @return true if sensor responds, false otherwise
     */
    virtual bool probe() = 0;

    /**
     * @brief Check if a timeout occurred in the last operation
     * @return true if timeout occurred, false otherwise
     */
    virtual bool timeout_occurred() = 0;

    // ===== Factory Method =====

    /**
     * @brief Create a ToF sensor instance
     * @param config Configuration for the sensor
     * @return Unique pointer to the sensor instance
     */
    static std::unique_ptr<TofSensor> create(const Config& config);
};