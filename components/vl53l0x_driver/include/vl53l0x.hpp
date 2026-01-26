// VL53L0X control (C++ version)
// Copyright © 2019 Adrian Kennard, Andrews & Arnold Ltd. See LICENCE file for details. GPL 3.0
// Refactored: I2C bus must be pre-initialized by caller (no SCL/SDA parameters)

#ifndef VL53L0X_HPP
#define VL53L0X_HPP

#include <cstdint>
#include <cstddef>
#include <memory>
#include <driver/i2c.h>
#include <driver/gpio.h>

class VL53L0X {
public:
    enum class VcselPeriodType { PreRange, FinalRange };
    static constexpr uint8_t I2C_SLAVE_DEVICE_ADDRESS = 0x8A;

    // Refactored Constructor: bus must be pre-initialized → no SCL/SDA
    VL53L0X(i2c_port_t port,
            gpio_num_t xshut = GPIO_NUM_NC,
            uint8_t address = 0x29,
            bool io_2v8 = true);

    // Destructor
    ~VL53L0X();

    // Delete copy constructor and assignment operator
    VL53L0X(const VL53L0X&) = delete;
    VL53L0X& operator=(const VL53L0X&) = delete;

    // Move constructor and assignment
    VL53L0X(VL53L0X&& other) noexcept;
    VL53L0X& operator=(VL53L0X&& other) noexcept;

    // Initialize the sensor - returns nullptr on success, error string on failure
    const char* init();

    // Address management
    void setAddress(uint8_t new_addr);
    uint8_t getAddress() const;

    // Register operations
    void writeReg8Bit(uint8_t reg, uint8_t value);
    void writeReg16Bit(uint8_t reg, uint16_t value);
    void writeReg32Bit(uint8_t reg, uint32_t value);
    uint8_t readReg8Bit(uint8_t reg);
    uint16_t readReg16Bit(uint8_t reg);
    uint32_t readReg32Bit(uint8_t reg);

    void writeMulti(uint8_t reg, const uint8_t* src, uint8_t count);
    void readMulti(uint8_t reg, uint8_t* dst, uint8_t count);

    // Configuration
    const char* setSignalRateLimit(float limit_Mcps);
    float getSignalRateLimit();

    const char* setMeasurementTimingBudget(uint32_t budget_us);
    uint32_t getMeasurementTimingBudget();

    const char* setVcselPulsePeriod(VcselPeriodType type, uint8_t period_pclks);
    uint8_t getVcselPulsePeriod(VcselPeriodType type);

    // Measurement operations
    void startContinuous(uint32_t period_ms = 0);
    void stopContinuous();
    uint16_t readRangeContinuousMillimeters();
    uint16_t readRangeSingleMillimeters();

    // Timeout management
    void setTimeout(uint16_t timeout);
    uint16_t getTimeout() const;
    bool timeoutOccurred();
    bool i2cFail();

    // Factory function for shared_ptr
    static std::shared_ptr<VL53L0X> create(i2c_port_t port,
                                          gpio_num_t xshut = GPIO_NUM_NC,
                                          uint8_t address = 0x29,
                                          bool io_2v8 = true);

private:
    struct SequenceStepEnables {
        bool tcc, msrc, dss, pre_range, final_range;
    };

    struct SequenceStepTimeouts {
        uint16_t pre_range_vcsel_period_pclks, final_range_vcsel_period_pclks;
        uint16_t msrc_dss_tcc_mclks, pre_range_mclks, final_range_mclks;
        uint32_t msrc_dss_tcc_us, pre_range_us, final_range_us;
    };

    bool performSingleRefCalibration(uint8_t vhv_init_byte);
    void getSequenceStepEnables(SequenceStepEnables* enables);
    void getSequenceStepTimeouts(SequenceStepEnables* enables, SequenceStepTimeouts* timeouts);

    // Private structure to hold implementation details
    struct Impl;
    std::unique_ptr<Impl> pimpl;
};

#endif // VL53L0X_HPP
       //
