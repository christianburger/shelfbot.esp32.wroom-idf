#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"

/**
 * @brief High-level VL53L1 Time-of-Flight distance sensor driver
 * 
 * This driver supports the TOF400F module which wraps the VL53L1 sensor.
 * The module provides both UART/Modbus and I2C interfaces.
 * 
 * Key features:
 * - Automatic mode switching from UART to I2C
 * - Two ranging modes: High precision (1.3m) and Long distance (4.0m)
 * - Clean measurement APIs
 * - Built-in error handling and timeout management
 */
class VL53L1 {
public:
    /**
     * @brief Ranging mode for the sensor
     */
    enum class RangingMode {
        HIGH_PRECISION,    ///< 30ms timing, 1.3m range, higher accuracy
        LONG_DISTANCE      ///< 200ms timing, 4.0m range
    };

    /**
     * @brief Measurement mode
     */
    enum class MeasurementMode {
        SINGLE,           ///< Single-shot measurement
        CONTINUOUS,       ///< Continuous measurement
    };

    /**
     * @brief Measurement result structure
     */
    struct MeasurementResult {
        uint16_t distance_mm;      ///< Distance in millimeters
        bool valid;                ///< True if measurement is valid
        bool timeout_occurred;     ///< True if timeout occurred
        uint32_t timestamp_us;     ///< Timestamp in microseconds
        uint8_t range_status;      ///< Range status code
    };

    /**
     * @brief Configuration structure for VL53L1
     */
    struct Config {
        // I2C configuration
        i2c_port_t i2c_port;           ///< I2C port number
        gpio_num_t sda_pin;            ///< SDA pin
        gpio_num_t scl_pin;            ///< SCL pin
        gpio_num_t xshut_pin;          ///< XSHUT pin (GPIO_NUM_NC if not used)
        uint8_t i2c_address;           ///< I2C device address (default 0x29)
        uint32_t i2c_freq_hz;          ///< I2C clock frequency
        
        // UART configuration (for initial setup)
        uart_port_t uart_port;         ///< UART port for initial configuration
        gpio_num_t uart_tx_pin;        ///< UART TX pin (to module RX)
        gpio_num_t uart_rx_pin;        ///< UART RX pin (from module TX)
        uint32_t uart_baud_rate;       ///< UART baud rate (default 115200)
        
        // Sensor configuration
        RangingMode ranging_mode;      ///< Ranging mode
        uint16_t timeout_ms;           ///< Measurement timeout in milliseconds
        bool enable_i2c_mode;          ///< True to switch to I2C mode on init
    };

    // ===== CONSTRUCTION AND INITIALIZATION =====

    /**
     * @brief Construct a new VL53L1 sensor object
     * @param config Sensor configuration
     */
    explicit VL53L1(const Config& config);

    /**
     * @brief Destructor - cleans up resources
     */
    ~VL53L1();

    // Prevent copying
    VL53L1(const VL53L1&) = delete;
    VL53L1& operator=(const VL53L1&) = delete;

    // Allow moving
    VL53L1(VL53L1&& other) noexcept;
    VL53L1& operator=(VL53L1&& other) noexcept;

    /**
     * @brief Initialize the sensor
     * 
     * This will:
     * 1. Configure UART (if enable_i2c_mode is true)
     * 2. Send command to switch module to I2C mode
     * 3. Initialize I2C communication
     * 4. Configure ranging mode
     * 
     * @return nullptr on success, error message on failure
     */
    const char* init();

    /**
     * @brief Check if sensor is initialized and ready
     * @return true if initialized, false otherwise
     */
    bool isReady() const;

    // ===== MEASUREMENT APIS =====

    /**
     * @brief Perform a single distance measurement
     * @param result Output parameter for measurement result
     * @return true on success, false on failure
     */
    bool readSingle(MeasurementResult& result);

    /**
     * @brief Start continuous measurement mode
     * @return true on success, false on failure
     */
    bool startContinuous();

    /**
     * @brief Read a measurement in continuous mode
     * @param result Output parameter for measurement result
     * @return true on success, false on failure
     */
    bool readContinuous(MeasurementResult& result);

    /**
     * @brief Stop continuous measurement mode
     * @return true on success, false on failure
     */
    bool stopContinuous();

    // ===== CONFIGURATION APIS =====

    /**
     * @brief Set ranging mode
     * @param mode Ranging mode (high precision or long distance)
     * @return nullptr on success, error message on failure
     */
    const char* setRangingMode(RangingMode mode);

    /**
     * @brief Get current ranging mode
     * @return Current ranging mode
     */
    RangingMode getRangingMode() const;

    /**
     * @brief Set measurement timeout
     * @param timeout_ms Timeout in milliseconds
     */
    void setTimeout(uint16_t timeout_ms);

    /**
     * @brief Get current timeout setting
     * @return Timeout in milliseconds
     */
    uint16_t getTimeout() const;

    /**
     * @brief Change I2C address of the device
     * @param new_addr New I2C address (7-bit)
     * @return true on success, false on failure
     */
    bool setAddress(uint8_t new_addr);

    /**
     * @brief Get current I2C address
     * @return I2C address (7-bit)
     */
    uint8_t getAddress() const;

    // ===== DIAGNOSTIC APIS =====

    /**
     * @brief Check if the last operation had a timeout
     * @return true if timeout occurred, false otherwise
     */
    bool timeoutOccurred();

    /**
     * @brief Verify sensor connectivity
     * @return true if sensor responds, false otherwise
     */
    bool probe();

    /**
     * @brief Perform sensor self-test
     * @return nullptr on success, error message on failure
     */
    const char* selfTest();

private:
    struct Impl;
    std::unique_ptr<Impl> pimpl_;
};

/**
 * @brief Helper function to create default VL53L1 configuration
 * @param i2c_port I2C port number
 * @param sda_pin SDA GPIO pin
 * @param scl_pin SCL GPIO pin
 * @param xshut_pin XSHUT GPIO pin (optional)
 * @param uart_tx_pin UART TX pin (for initial setup)
 * @param uart_rx_pin UART RX pin (for initial setup)
 * @return Default configuration structure
 */
VL53L1::Config vl53l1_default_config(
    i2c_port_t i2c_port = I2C_NUM_0,
    gpio_num_t sda_pin = GPIO_NUM_21,
    gpio_num_t scl_pin = GPIO_NUM_22,
    gpio_num_t xshut_pin = GPIO_NUM_NC,
    uart_port_t uart_port = UART_NUM_1,
    gpio_num_t uart_tx_pin = GPIO_NUM_17,
    gpio_num_t uart_rx_pin = GPIO_NUM_16
);
