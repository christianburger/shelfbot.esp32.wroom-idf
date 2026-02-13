#pragma once
#include <idf_c_includes.hpp>

// ═══════════════════════════════════════════════════════════════
// VL53L1 DRIVER CONFIGURATION - EDIT ALL SETTINGS HERE
// ═══════════════════════════════════════════════════════════════
#define VL53L1_I2C_PORT         I2C_NUM_0
#define VL53L1_SDA_PIN          GPIO_NUM_21
#define VL53L1_SCL_PIN          GPIO_NUM_22
#define VL53L1_I2C_FREQ_HZ      40000
#define VL53L1_I2C_ADDRESS      0x29
#define VL53L1_TIMEOUT_MS       500
#define VL53L1_TIMING_BUDGET_US 200000
#define VL53L1_SIGNAL_RATE_MCPS 0.25f
// ═══════════════════════════════════════════════════════════════

/**
 * @brief VL53L1 ToF Driver - Simple and Explicit Implementation
 *
 * All configuration is defined above in #defines.
 * No external configuration accepted.
 * All members are clearly visible and accessible.
 */
class VL53L1_Driver {
public:
    struct MeasurementResult {
        uint16_t distance_mm;
        uint8_t  range_status;
        bool     valid;
        bool     timeout_occurred;
        int64_t  timestamp_us;
    };

    // Constructor - uses #define configuration
    VL53L1_Driver();
    ~VL53L1_Driver();

    // ── Standardized Interface Methods ──
    const char* configure();
    const char* init();
    const char* setup();
    const char* calibrate();
    const char* check();
    bool read_sensor(MeasurementResult& result);
    bool isReady() const;

    // ── Support operations ──
    void setTimeout(uint16_t timeout_ms);
    bool timeoutOccurred();

    // ── I2C Handle Access (for external scanning) ──
    i2c_master_bus_handle_t getBusHandle() { return bus_handle_; }
    bool lockI2C();
    void unlockI2C();

    // Non-copyable
    VL53L1_Driver(const VL53L1_Driver&) = delete;
    VL53L1_Driver& operator=(const VL53L1_Driver&) = delete;

private:
    // ── Configuration (from #defines) ──
    i2c_port_t  i2c_port_;
    gpio_num_t  sda_pin_;
    gpio_num_t  scl_pin_;
    uint8_t     i2c_address_;
    uint32_t    i2c_freq_hz_;
    uint16_t    timeout_ms_;
    uint32_t    timing_budget_us_;
    float       signal_rate_limit_mcps_;

    // ── Hardware handles ──
    i2c_master_bus_handle_t bus_handle_;
    i2c_master_dev_handle_t dev_handle_;
    SemaphoreHandle_t       i2c_mutex_;  // For thread-safe I2C access

    // ── State ──
    bool     initialized_;
    uint8_t  stop_variable_;
    uint32_t measurement_timing_budget_us_;
    bool     did_timeout_;

    // ── I2C Operations ──
    esp_err_t writeReg8(uint8_t reg, uint8_t value);
    esp_err_t writeReg16(uint8_t reg, uint16_t value);
    esp_err_t writeReg32(uint8_t reg, uint32_t value);
    esp_err_t readReg8(uint8_t reg, uint8_t* value);
    esp_err_t readReg16(uint8_t reg, uint16_t* value);
    esp_err_t readReg32(uint8_t reg, uint32_t* value);
    esp_err_t writeMulti(uint8_t reg, const uint8_t* src, uint8_t count);
    esp_err_t readMulti(uint8_t reg, uint8_t* dst, uint8_t count);

    // ── Initialization helpers ──
    struct RegisterWrite {
        uint8_t     reg;
        uint8_t     value;
        uint16_t    delay_ms;
        std::string comment;
    };

    bool      loadRegisterSequence(const char* csv_data, std::vector<RegisterWrite>& seq);
    esp_err_t executeRegisterSequence(const std::vector<RegisterWrite>& seq);
    esp_err_t loadInitSequence();

    // ── Calibration ──
    esp_err_t configureSPAD(uint8_t* count, bool* type_is_aperture);
    esp_err_t performSingleRefCalibration(uint8_t vhv_init_byte);

    // ── Timing calculation ──
    struct SequenceStepEnables {
        bool tcc, msrc, dss, pre_range, final_range;
    };

    struct SequenceStepTimeouts {
        uint16_t pre_range_vcsel_period_pclks, final_range_vcsel_period_pclks;
        uint16_t msrc_dss_tcc_mclks, pre_range_mclks, final_range_mclks;
        uint32_t msrc_dss_tcc_us, pre_range_us, final_range_us;
    };

    uint32_t timeoutMclksToMicroseconds(uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks);
    uint32_t timeoutMicrosecondsToMclks(uint32_t timeout_period_us, uint8_t vcsel_period_pclks);
    uint16_t decodeTimeout(uint16_t reg_val);
    uint16_t encodeTimeout(uint16_t timeout_mclks);
    void getSequenceStepEnables(SequenceStepEnables* enables);
    void getSequenceStepTimeouts(SequenceStepEnables* enables, SequenceStepTimeouts* timeouts);

    // ── Configuration (internal use) ──
    const char* setSignalRateLimit(float limit_mcps);
    const char* setMeasurementTimingBudget(uint32_t budget_us);
    uint32_t    getMeasurementTimingBudget();
    const char* setVcselPulsePeriod(uint8_t type, uint8_t period_pclks);
};

// ═══════════════════════════════════════════════════════════════
// TYPEDEF FOR TOF_SENSOR INTERFACE
// ═══════════════════════════════════════════════════════════════
using TofDriver = VL53L1_Driver;
// ═══════════════════════════════════════════════════════════════
