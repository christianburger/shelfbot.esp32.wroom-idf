#pragma once
#include "idf_c_includes.hpp"
#include "duart_modbus.hpp"

/**
 * @brief VL53L1 ToF sensor driver using Modbus/UART protocol (TOF400F module)
 * 
 * This driver communicates with the TOF400F module EXCLUSIVELY via Modbus over UART.
 * It does NOT switch to I2C mode - it stays in Modbus mode for all operations.
 * 
 * The TOF400F module contains a VL53L1 sensor and provides a Modbus interface for:
 * - Configuration (ranging mode, continuous output, etc.)
 * - Calibration (offset, crosstalk)
 * - Distance measurement readout
 */
class VL53L1_Modbus {
public:
    enum class RangingMode {
        HIGH_PRECISION,   // 30ms period, 1.3m range
        LONG_DISTANCE     // 200ms period, 4.0m range
    };

    struct Config {
        // UART/Modbus configuration
        uart_port_t uart_port;
        gpio_num_t uart_tx_pin;
        gpio_num_t uart_rx_pin;
        uint8_t modbus_slave_address;
        
        // Sensor configuration
        RangingMode ranging_mode;
        uint16_t timeout_ms;
        bool enable_continuous;
    };

    struct Measurement {
        uint16_t distance_mm;
        uint8_t range_status;
        bool valid;
        int64_t timestamp_us;
    };

    explicit VL53L1_Modbus(const Config& config);
    ~VL53L1_Modbus();

    /**
     * @brief Initialize the sensor via Modbus
     * @return nullptr on success, error message on failure
     */
    const char* init();

    /**
     * @brief Check if sensor is ready
     */
    bool isReady() const { return initialized_; }

    /**
     * @brief Start continuous measurements
     */
    bool startContinuous();

    /**
     * @brief Stop continuous measurements
     */
    bool stopContinuous();

    /**
     * @brief Read latest measurement
     */
    bool readContinuous(Measurement& result);

    /**
     * @brief Probe if sensor is responding
     */
    bool probe();

    /**
     * @brief Set ranging mode
     */
    const char* setRangingMode(RangingMode mode);

    /**
     * @brief Set timeout for Modbus communication
     */
    void setTimeout(uint16_t timeout_ms);

    /**
     * @brief Run self-test
     */
    const char* selfTest();

    /**
     * @brief Check if last operation timed out
     */
    bool timeoutOccurred() const { return timeout_occurred_; }

private:
    Config config_;
    DuartModbus* modbus_;
    bool initialized_;
    bool timeout_occurred_;

    static const char* TAG;

    // Initialization steps
    const char* initModbus();
    const char* testCommunication();
    const char* readCurrentConfiguration();
    const char* configureRangingMode();
    const char* configureContinuousMode();
    const char* verifyConfiguration();

    // Helper functions
    void logModbusResponse(const char* operation, const DuartModbus::ModbusResponse& response);

    // Prevent copying
    VL53L1_Modbus(const VL53L1_Modbus&) = delete;
    VL53L1_Modbus& operator=(const VL53L1_Modbus&) = delete;
};

/**
 * @brief Create default configuration
 * 
 * Note: i2c_port, i2c_sda, i2c_scl parameters are present for API compatibility
 * but are NOT USED by this driver. VL53L1_Modbus uses ONLY UART/Modbus.
 */
VL53L1_Modbus::Config vl53l1_modbus_default_config(
    i2c_port_t i2c_port = I2C_NUM_0,
    gpio_num_t i2c_sda = GPIO_NUM_21,
    gpio_num_t i2c_scl = GPIO_NUM_22,
    uart_port_t uart_port = UART_NUM_1,
    gpio_num_t uart_tx = GPIO_NUM_17,
    gpio_num_t uart_rx = GPIO_NUM_16
);
