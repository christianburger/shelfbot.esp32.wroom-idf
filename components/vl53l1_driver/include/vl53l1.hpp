#pragma once
#include <idf_c_includes.hpp>

// ══════════════════════════════════════════════════════════════
// VL53L1 DRIVER CONFIGURATION - EDIT ALL SETTINGS HERE
// ═══════════════════════════════════════════════════════════════
#define VL53L1_I2C_PORT         I2C_NUM_0
#define VL53L1_SDA_PIN          GPIO_NUM_21
#define VL53L1_SCL_PIN          GPIO_NUM_22
#define VL53L1_I2C_FREQ_HZ      400000
#define VL53L1_I2C_ADDRESS      0x29
#define VL53L1_TIMEOUT_MS       500
#define VL53L1_TIMING_BUDGET_US 200000
#define VL53L1_SIGNAL_RATE_MCPS 0.25f
// ═══════════════════════════════════════════════════════════════

class VL53L1_Driver {
public:
    struct MeasurementResult {
        uint16_t distance_mm;
        uint8_t  range_status;
        bool     valid;
        bool     timeout_occurred;
        int64_t  timestamp_us;
    };

    VL53L1_Driver();
    ~VL53L1_Driver();

    const char* configure();
    const char* init();
    const char* setup();
    const char* calibrate();
    const char* check();
    bool read_sensor(MeasurementResult& result);
    bool isReady() const;

    void setTimeout(uint16_t timeout_ms);
    bool timeoutOccurred();

    i2c_master_bus_handle_t getBusHandle() { return bus_handle_; }
    bool lockI2C();
    void unlockI2C();

    VL53L1_Driver(const VL53L1_Driver&) = delete;
    VL53L1_Driver& operator=(const VL53L1_Driver&) = delete;

private:
    i2c_port_t  i2c_port_;
    gpio_num_t  sda_pin_;
    gpio_num_t  scl_pin_;
    uint8_t     i2c_address_;
    uint32_t    i2c_freq_hz_;
    uint16_t    timeout_ms_;
    uint32_t    timing_budget_us_;
    float       signal_rate_limit_mcps_;

    i2c_master_bus_handle_t bus_handle_;
    i2c_master_dev_handle_t dev_handle_;
    SemaphoreHandle_t       i2c_mutex_;

    bool initialized_;
    bool did_timeout_;

    esp_err_t writeReg8(uint16_t reg, uint8_t value);
    esp_err_t writeReg16(uint16_t reg, uint16_t value);
    esp_err_t writeReg32(uint16_t reg, uint32_t value);
    esp_err_t readReg8(uint16_t reg, uint8_t* value);
    esp_err_t readReg16(uint16_t reg, uint16_t* value);
    esp_err_t readReg32(uint16_t reg, uint32_t* value);

    esp_err_t writeMulti(uint16_t reg, const uint8_t* src, size_t count);
    esp_err_t readMulti(uint16_t reg, uint8_t* dst, size_t count);

    esp_err_t waitForBoot();
    esp_err_t startContinuous();
};

using TofDriver = VL53L1_Driver;
