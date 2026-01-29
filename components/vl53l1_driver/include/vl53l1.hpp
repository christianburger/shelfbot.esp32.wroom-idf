#pragma once

#include <cstdint>
#include <memory>
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "hal/uart_types.h"

/**
 * @brief VL53L1 Time-of-Flight sensor driver for ESP32
 *
 * Uses I2C master driver for reliable communication.
 * Supports both single-shot and continuous ranging modes.
 */
class VL53L1 {
public:
    enum class RangingMode {
        HIGH_PRECISION = 0,   // ~30ms measurement, 1.3m range
        LONG_DISTANCE = 1     // ~200ms measurement, 4.0m range
    };

    enum class MeasurementMode {
        SINGLE,
        CONTINUOUS
    };

    struct MeasurementResult {
        uint16_t distance_mm;
        uint8_t range_status;
        bool valid;
        bool timeout_occurred;
        int64_t timestamp_us;
    };

    struct Config {
        // I2C configuration
        i2c_port_t i2c_port;
        gpio_num_t sda_pin;
        gpio_num_t scl_pin;
        gpio_num_t xshut_pin;
        uint8_t i2c_address;
        uint32_t i2c_freq_hz;

        // UART configuration (for TOF400F modules)
        uart_port_t uart_port;
        gpio_num_t uart_tx_pin;
        gpio_num_t uart_rx_pin;
        uint32_t uart_baud_rate;

        // Sensor configuration
        RangingMode ranging_mode;
        uint16_t timeout_ms;
        bool enable_i2c_mode;  // Set to true to enable I2C mode via UART
    };

    VL53L1(const Config& config);
    ~VL53L1();

    // Disable copy constructor and assignment
    VL53L1(const VL53L1&) = delete;
    VL53L1& operator=(const VL53L1&) = delete;

    // Allow move operations
    VL53L1(VL53L1&& other) noexcept;
    VL53L1& operator=(VL53L1&& other) noexcept;

    /**
     * @brief Initialize the VL53L1 sensor
     * @return nullptr on success, error string on failure
     */
    const char* init();

    /**
     * @brief Check if sensor is ready
     */
    bool isReady() const;

    /**
     * @brief Read a single distance measurement
     * @param result Measurement result structure
     * @return true on success
     */
    bool readSingle(MeasurementResult& result);

    /**
     * @brief Start continuous measurement mode
     * @return true on success
     */
    bool startContinuous();

    /**
     * @brief Read from continuous measurement mode
     * @param result Measurement result structure
     * @return true on success
     */
    bool readContinuous(MeasurementResult& result);

    /**
     * @brief Stop continuous measurement mode
     * @return true on success
     */
    bool stopContinuous();

    /**
     * @brief Set ranging mode
     * @param mode Ranging mode
     * @return nullptr on success, error string on failure
     */
    const char* setRangingMode(RangingMode mode);

    /**
     * @brief Get current ranging mode
     */
    RangingMode getRangingMode() const;

    /**
     * @brief Set I2C slave address
     * @param new_addr New I2C address (7-bit)
     * @return true on success
     */
    bool setAddress(uint8_t new_addr);

    /**
     * @brief Get current I2C address
     */
    uint8_t getAddress() const;

    /**
     * @brief Set timeout for measurements
     * @param timeout_ms Timeout in milliseconds
     */
    void setTimeout(uint16_t timeout_ms);

    /**
     * @brief Get current timeout
     */
    uint16_t getTimeout() const;

    /**
     * @brief Check if timeout occurred
     */
    bool timeoutOccurred();

    /**
     * @brief Probe for sensor existence
     * @return true if sensor responds
     */
    bool probe();

    /**
     * @brief Perform self-test
     * @return nullptr on success, error string on failure
     */
    const char* selfTest();

private:
    struct Impl;
    std::unique_ptr<Impl> pimpl_;
};

// Helper function to create default configuration
VL53L1::Config vl53l1_default_config(
    i2c_port_t i2c_port = I2C_NUM_0,
    gpio_num_t sda_pin = GPIO_NUM_21,
    gpio_num_t scl_pin = GPIO_NUM_22,
    gpio_num_t xshut_pin = GPIO_NUM_NC,
    uart_port_t uart_port = UART_NUM_1,
    gpio_num_t uart_tx_pin = GPIO_NUM_17,
    gpio_num_t uart_rx_pin = GPIO_NUM_16
);