#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "esp_log.h"

/**
 * @brief High-level VL53L0X Time-of-Flight distance sensor driver
 *
 * This driver provides a clean interface to the VL53L0X sensor with:
 * - Automatic I2C bus management using the new I2C master driver
 * - CSV-based register configuration for maintainability
 * - Simple measurement APIs
 * - Built-in error handling and timeout management
 */
class VL53L0X {
public:
    /**
     * @brief Measurement mode for the sensor
     */
    enum class MeasurementMode {
        SINGLE,           ///< Single-shot measurement
        CONTINUOUS,       ///< Continuous measurement with default timing
        CONTINUOUS_TIMED  ///< Continuous with custom inter-measurement period
    };

    /**
     * @brief VCSEL pulse period type
     */
    enum class VcselPeriodType {
        PRE_RANGE,
        FINAL_RANGE
    };

    /**
     * @brief Measurement result structure
     */
    struct MeasurementResult {
        uint16_t distance_mm;      ///< Distance in millimeters
        bool valid;                ///< True if measurement is valid
        bool timeout_occurred;     ///< True if timeout occurred
        uint32_t timestamp_us;     ///< Timestamp in microseconds
    };

    /**
     * @brief Configuration structure for VL53L0X
     */
    struct Config {
        i2c_port_t i2c_port;           ///< I2C port number
        gpio_num_t sda_pin;            ///< SDA pin
        gpio_num_t scl_pin;            ///< SCL pin
        gpio_num_t xshut_pin;          ///< XSHUT pin (GPIO_NUM_NC if not used)
        uint8_t i2c_address;           ///< I2C device address
        uint32_t i2c_freq_hz;          ///< I2C clock frequency
        bool io_2v8;                   ///< True for 2.8V I/O, false for 1.8V
        uint16_t timeout_ms;           ///< Measurement timeout in milliseconds
        uint32_t timing_budget_us;     ///< Measurement timing budget in microseconds
        float signal_rate_limit_mcps;  ///< Signal rate limit in MCPS
    };

    // ===== CONSTRUCTION AND INITIALIZATION =====

    /**
     * @brief Construct a new VL53L0X sensor object
     * @param config Sensor configuration
     */
    explicit VL53L0X(const Config& config);

    /**
     * @brief Destructor - cleans up I2C resources
     */
    ~VL53L0X();

    // Prevent copying
    VL53L0X(const VL53L0X&) = delete;
    VL53L0X& operator=(const VL53L0X&) = delete;

    // Allow moving
    VL53L0X(VL53L0X&& other) noexcept;
    VL53L0X& operator=(VL53L0X&& other) noexcept;

    /**
     * @brief Initialize the sensor
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
     * @param period_ms Inter-measurement period in milliseconds (0 for back-to-back)
     * @return true on success, false on failure
     */
    bool startContinuous(uint32_t period_ms = 0);

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
     * @brief Set signal rate limit
     * @param limit_mcps Signal rate limit in mega counts per second
     * @return nullptr on success, error message on failure
     */
    const char* setSignalRateLimit(float limit_mcps);

    /**
     * @brief Get signal rate limit
     * @return Signal rate limit in MCPS
     */
    float getSignalRateLimit();

    /**
     * @brief Set measurement timing budget
     * @param budget_us Timing budget in microseconds
     * @return nullptr on success, error message on failure
     */
    const char* setMeasurementTimingBudget(uint32_t budget_us);

    /**
     * @brief Get measurement timing budget
     * @return Timing budget in microseconds
     */
    uint32_t getMeasurementTimingBudget();

    /**
     * @brief Set VCSEL pulse period
     * @param type Period type (pre-range or final range)
     * @param period_pclks Period in PCLKs
     * @return nullptr on success, error message on failure
     */
    const char* setVcselPulsePeriod(VcselPeriodType type, uint8_t period_pclks);

    /**
     * @brief Get VCSEL pulse period
     * @param type Period type (pre-range or final range)
     * @return Period in PCLKs
     */
    uint8_t getVcselPulsePeriod(VcselPeriodType type);

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
     * @brief Get sensor model ID
     * @return Model ID (should be 0xEE for VL53L0X)
     */
    uint8_t getModelId();

    /**
     * @brief Get sensor revision ID
     * @return Revision ID
     */
    uint8_t getRevisionId();

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
 * @brief Helper function to create default VL53L0X configuration
 * @param i2c_port I2C port number
 * @param sda_pin SDA GPIO pin
 * @param scl_pin SCL GPIO pin
 * @param xshut_pin XSHUT GPIO pin (optional)
 * @return Default configuration structure
 */
VL53L0X::Config vl53l0x_default_config(
    i2c_port_t i2c_port = I2C_NUM_0,
    gpio_num_t sda_pin = GPIO_NUM_21,
    gpio_num_t scl_pin = GPIO_NUM_22,
    gpio_num_t xshut_pin = GPIO_NUM_NC
);
