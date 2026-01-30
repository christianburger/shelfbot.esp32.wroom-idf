#pragma once

#include "duart_modbus.hpp"
#include <memory>
#include <string>

/**
 * @brief TOF400F sensor using MODBUS over UART for configuration
 * This class provides high-level interface for TOF400F sensor
 * configuration and mode switching.
 */
class Tof400fModbus {
public:
    enum class RangingMode {
        HIGH_PRECISION = 0,   // ~30ms measurement, 1.3m range
        LONG_DISTANCE = 1     // ~200ms measurement, 4.0m range
    };

    struct Config {
        // MODBUS configuration
        uint8_t modbus_address;      // Default: 0x01
        DuartModbus::Config uart_config;

        // TOF400F specific registers
        struct {
            uint16_t device_address;     // 0x0002
            uint16_t baud_rate;          // 0x0003
            uint16_t range_mode;         // 0x0004
            uint16_t continuous_output;  // 0x0005
            uint16_t disable_iic;        // 0x0009 (write 1 to enable I2C mode)
            uint16_t measurement_result; // 0x0010
        } registers;
    };

    Tof400fModbus(const Config& config);
    ~Tof400fModbus();

    /**
     * @brief Initialize TOF400F MODBUS interface
     * @return nullptr on success, error string on failure
     */
    const char* init();

    /**
     * @brief Check if MODBUS interface is ready
     */
    bool isReady() const { return modbus_ && modbus_->isReady(); }

    /**
     * @brief Switch TOF400F to I2C mode
     * @param verify If true, verify the switch was successful
     * @return nullptr on success, error string on failure
     */
    const char* switchToI2CMode(bool verify = true);

    /**
     * @brief Set ranging mode
     * @param mode Ranging mode to set
     * @param verify If true, verify the mode was set
     * @return nullptr on success, error string on failure
     */
    const char* setRangingMode(RangingMode mode, bool verify = true);

    /**
     * @brief Get current ranging mode
     * @param mode Output parameter for current mode
     * @return nullptr on success, error string on failure
     */
    const char* getRangingMode(RangingMode& mode);

    /**
     * @brief Set MODBUS device address
     * @param new_address New MODBUS address (1-247)
     * @param verify If true, verify the address was set
     * @return nullptr on success, error string on failure
     */
    const char* setDeviceAddress(uint8_t new_address, bool verify = true);

    /**
     * @brief Get current device address
     * @return Current MODBUS address, 0 on error
     */
    uint8_t getDeviceAddress() const { return config_.modbus_address; }

  /**
   * @brief Read measurement result register
   * @param distance_mm Output parameter for distance in millimeters
   * @return nullptr on success, error string on failure
   */
  const char* readMeasurement(uint16_t& distance_mm);  // ADD THIS

    /**
     * @brief Perform self-test of MODBUS communication
     * @return Detailed test results
     */
    std::string selfTest();

    /**
     * @brief Get detailed status of last operation
     */
    std::string getStatus() const;

private:
    Config config_;
    std::unique_ptr<DuartModbus> modbus_;
    bool initialized_;
    std::string last_error_;
    static const char* TAG;

    // Helper methods
    bool writeAndVerifyRegister(uint16_t reg, uint16_t value, const std::string& operation);
    bool readAndLogRegister(uint16_t reg, uint16_t& value, const std::string& operation);

    // Error handling
    void setError(const std::string& error);
    void clearError();

    // Default register values
    static constexpr uint16_t DEFAULT_BAUD_RATE = 7;  // 115200 baud
    static constexpr uint16_t DISABLE_I2C_VALUE = 0;  // 0 = UART mode
    static constexpr uint16_t ENABLE_I2C_VALUE = 1;   // 1 = I2C mode
};
