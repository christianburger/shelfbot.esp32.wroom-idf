#pragma once

#include <cstdint>
#include <memory>
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_log.h"

class VL53L0X {
public:
    enum VcselPeriodType {
        VcselPeriodPreRange,
        VcselPeriodFinalRange
    };

    struct SequenceStepEnables {
        bool tcc, msrc, dss, pre_range, final_range;
    };

    struct SequenceStepTimeouts {
        uint16_t pre_range_vcsel_period_pclks, final_range_vcsel_period_pclks;
        uint16_t msrc_dss_tcc_mclks, pre_range_mclks, final_range_mclks;
        uint32_t msrc_dss_tcc_us, pre_range_us, final_range_us;
    };

    // ===== CONSTRUCTOR =====
    VL53L0X(i2c_port_t i2c_port = I2C_NUM_0,
            gpio_num_t xshut_pin = GPIO_NUM_NC,
            uint8_t i2c_address = 0x29,
            bool io_2v8 = true);

    VL53L0X(VL53L0X&& other) noexcept;
    VL53L0X& operator=(VL53L0X&& other) noexcept;
    ~VL53L0X();

    // ===== FACTORY METHOD =====
    static std::shared_ptr<VL53L0X> create(i2c_port_t port,
                                           gpio_num_t xshut,
                                           uint8_t address = 0x29,
                                           bool io_2v8 = true);

    // ===== SENSOR API =====
    const char* init();
    uint16_t readRangeSingleMillimeters();
    uint16_t readRangeContinuousMillimeters();
    void startContinuous(uint32_t period_ms = 0);
    void stopContinuous();
    void setTimeout(uint16_t timeout_ms);
    uint16_t getTimeout() const;
    bool timeoutOccurred();
    bool i2cFail();

    // ===== CONFIGURATION =====
    void setAddress(uint8_t new_addr);
    uint8_t getAddress() const;

    const char* setSignalRateLimit(float limit_Mcps);
    float getSignalRateLimit();

    const char* setMeasurementTimingBudget(uint32_t budget_us);
    uint32_t getMeasurementTimingBudget();

    const char* setVcselPulsePeriod(VcselPeriodType type, uint8_t period_pclks);
    uint8_t getVcselPulsePeriod(VcselPeriodType type);

    // ===== DEBUG/UTILITY =====
    void dumpRegisters(uint8_t start = 0x00, uint8_t end = 0xFF);
    uint8_t getModuleType();

private:
    struct Impl;
    std::unique_ptr<Impl> pimpl;

    // Private I2C methods
    void writeReg8Bit(uint8_t reg, uint8_t value);
    void writeReg16Bit(uint8_t reg, uint16_t value);
    void writeReg32Bit(uint8_t reg, uint32_t value);
    uint8_t readReg8Bit(uint8_t reg);
    uint16_t readReg16Bit(uint8_t reg);
    uint32_t readReg32Bit(uint8_t reg);
    void writeMulti(uint8_t reg, const uint8_t* src, uint8_t count);
    void readMulti(uint8_t reg, uint8_t* dst, uint8_t count);

    // Private helper methods (from original code)
    bool performSingleRefCalibration(uint8_t vhv_init_byte);
    void getSequenceStepEnables(SequenceStepEnables* enables);
    void getSequenceStepTimeouts(SequenceStepEnables* enables, SequenceStepTimeouts* timeouts);
};
