#include "vl53l1.hpp"

static const char* TAG = "TofDriver_VL53L0X";

// ==========================================================================
// Register addresses
// ==========================================================================
namespace Reg {
    constexpr uint8_t SYSRANGE_START                            = 0x00;
    constexpr uint8_t SYSTEM_THRESH_HIGH                        = 0x0C;
    constexpr uint8_t SYSTEM_THRESH_LOW                         = 0x0E;
    constexpr uint8_t SYSTEM_SEQUENCE_CONFIG                    = 0x01;
    constexpr uint8_t SYSTEM_RANGE_CONFIG                       = 0x09;
    constexpr uint8_t SYSTEM_INTERMEASUREMENT_PERIOD            = 0x04;
    constexpr uint8_t SYSTEM_INTERRUPT_CONFIG_GPIO              = 0x0A;
    constexpr uint8_t GPIO_HV_MUX_ACTIVE_HIGH                   = 0x84;
    constexpr uint8_t SYSTEM_INTERRUPT_CLEAR                    = 0x0B;
    constexpr uint8_t RESULT_INTERRUPT_STATUS                   = 0x13;
    constexpr uint8_t RESULT_RANGE_STATUS                       = 0x14;
    constexpr uint8_t RESULT_RANGE_VALUE                        = 0x1E;
    constexpr uint8_t I2C_SLAVE_DEVICE_ADDRESS                  = 0x8A;
    constexpr uint8_t MSRC_CONFIG_CONTROL                       = 0x60;
    constexpr uint8_t PRE_RANGE_CONFIG_VCSEL_PERIOD             = 0x50;
    constexpr uint8_t PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI        = 0x51;
    constexpr uint8_t PRE_RANGE_CONFIG_TIMEOUT_MACROP_LO        = 0x52;
    constexpr uint8_t FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT = 0x44;
    constexpr uint8_t FINAL_RANGE_CONFIG_VCSEL_PERIOD           = 0x70;
    constexpr uint8_t FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI      = 0x71;
    constexpr uint8_t FINAL_RANGE_CONFIG_TIMEOUT_MACROP_LO      = 0x72;
    constexpr uint8_t MSRC_CONFIG_TIMEOUT_MACROP                = 0x46;
    constexpr uint8_t SOFT_RESET_GO2_SOFT_RESET_N               = 0xBF;
    constexpr uint8_t IDENTIFICATION_MODEL_ID                   = 0xC0;
    constexpr uint8_t IDENTIFICATION_REVISION_ID                = 0xC2;
    constexpr uint8_t OSC_CALIBRATE_VAL                         = 0xF8;
    constexpr uint8_t GLOBAL_CONFIG_VCSEL_WIDTH                 = 0x32;
    constexpr uint8_t GLOBAL_CONFIG_SPAD_ENABLES_REF_0          = 0xB0;
    constexpr uint8_t GLOBAL_CONFIG_REF_EN_START_SELECT         = 0xB6;
    constexpr uint8_t DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD       = 0x4E;
    constexpr uint8_t DYNAMIC_SPAD_REF_EN_START_OFFSET          = 0x4F;
    constexpr uint8_t POWER_MANAGEMENT_GO1_POWER_FORCE          = 0x80;
    constexpr uint8_t VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV         = 0x89;
    constexpr uint8_t ALGO_PHASECAL_LIM                         = 0x30;
    constexpr uint8_t ALGO_PHASECAL_CONFIG_TIMEOUT              = 0x30;
}

#define decodeVcselPeriod(reg_val)   (((reg_val) + 1) << 1)
#define encodeVcselPeriod(period_pclks) (((period_pclks) >> 1) - 1)
#define calcMacroPeriod(vcsel_period_pclks) \
    ((((uint32_t)2304 * (vcsel_period_pclks) * 1655) + 500) / 1000)

// ==========================================================================
// Internal structures
// ==========================================================================
struct RegisterWrite {
    uint8_t     reg;
    uint8_t     value;
    uint16_t    delay_ms;
    std::string comment;
};

struct SequenceStepEnables {
    bool tcc, msrc, dss, pre_range, final_range;
};

struct SequenceStepTimeouts {
    uint16_t pre_range_vcsel_period_pclks, final_range_vcsel_period_pclks;
    uint16_t msrc_dss_tcc_mclks, pre_range_mclks, final_range_mclks;
    uint32_t msrc_dss_tcc_us, pre_range_us, final_range_us;
};

// ==========================================================================
// Impl
// ==========================================================================
struct TofDriver::Impl {
    i2c_port_t  i2c_port;
    gpio_num_t  sda_pin, scl_pin;
    uint8_t     i2c_address;
    uint32_t    i2c_freq_hz;
    uint16_t    timeout_ms;
    uint32_t    timing_budget_us;
    float       signal_rate_limit_mcps;

    i2c_master_bus_handle_t bus_handle  = nullptr;
    i2c_master_dev_handle_t dev_handle  = nullptr;

    bool     initialized  = false;
    uint8_t  stop_variable = 0;
    uint32_t measurement_timing_budget_us = 0;
    bool     did_timeout  = false;

    esp_err_t writeReg8(uint8_t reg, uint8_t value);
    esp_err_t writeReg16(uint8_t reg, uint16_t value);
    esp_err_t writeReg32(uint8_t reg, uint32_t value);
    esp_err_t readReg8(uint8_t reg, uint8_t* value);
    esp_err_t readReg16(uint8_t reg, uint16_t* value);
    esp_err_t readReg32(uint8_t reg, uint32_t* value);
    esp_err_t writeMulti(uint8_t reg, const uint8_t* src, uint8_t count);
    esp_err_t readMulti(uint8_t reg, uint8_t* dst, uint8_t count);

    bool      loadRegisterSequence(const char* csv_data, std::vector<RegisterWrite>& seq);
    esp_err_t executeRegisterSequence(const std::vector<RegisterWrite>& seq);
    esp_err_t loadInitSequence();
    esp_err_t configureSPAD(uint8_t* count, bool* type_is_aperture);
    esp_err_t performSingleRefCalibration(uint8_t vhv_init_byte);

    uint32_t timeoutMclksToMicroseconds(uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks);
    uint32_t timeoutMicrosecondsToMclks(uint32_t timeout_period_us,    uint8_t vcsel_period_pclks);
    uint16_t decodeTimeout(uint16_t reg_val);
    uint16_t encodeTimeout(uint16_t timeout_mclks);
    void getSequenceStepEnables(SequenceStepEnables* enables);
    void getSequenceStepTimeouts(SequenceStepEnables* enables, SequenceStepTimeouts* timeouts);

    const char* setSignalRateLimit(float limit_mcps);
    const char* setMeasurementTimingBudget(uint32_t budget_us);
    uint32_t    getMeasurementTimingBudget();
    const char* setVcselPulsePeriod(uint8_t type, uint8_t period_pclks);
};

esp_err_t TofDriver::Impl::writeReg8(uint8_t reg, uint8_t value) {
    uint8_t buf[2] = {reg, value};
    return i2c_master_transmit(dev_handle, buf, 2, timeout_ms);
}

esp_err_t TofDriver::Impl::writeReg16(uint8_t reg, uint16_t value) {
    uint8_t buf[3] = {reg, (uint8_t)(value >> 8), (uint8_t)(value & 0xFF)};
    return i2c_master_transmit(dev_handle, buf, 3, timeout_ms);
}

esp_err_t TofDriver::Impl::writeReg32(uint8_t reg, uint32_t value) {
    uint8_t buf[5] = {reg, (uint8_t)(value >> 24), (uint8_t)(value >> 16),
                      (uint8_t)(value >> 8), (uint8_t)(value)};
    return i2c_master_transmit(dev_handle, buf, 5, timeout_ms);
}

esp_err_t TofDriver::Impl::readReg8(uint8_t reg, uint8_t* value) {
    return i2c_master_transmit_receive(dev_handle, &reg, 1, value, 1, timeout_ms);
}

esp_err_t TofDriver::Impl::readReg16(uint8_t reg, uint16_t* value) {
    uint8_t buf[2];
    esp_err_t err = i2c_master_transmit_receive(dev_handle, &reg, 1, buf, 2, timeout_ms);
    if (err == ESP_OK) *value = (buf[0] << 8) | buf[1];
    return err;
}

esp_err_t TofDriver::Impl::readReg32(uint8_t reg, uint32_t* value) {
    uint8_t buf[4];
    esp_err_t err = i2c_master_transmit_receive(dev_handle, &reg, 1, buf, 4, timeout_ms);
    if (err == ESP_OK)
        *value = (buf[0] << 24) | (buf[1] << 16) | (buf[2] << 8) | buf[3];
    return err;
}

esp_err_t TofDriver::Impl::writeMulti(uint8_t reg, const uint8_t* src, uint8_t count) {
    std::vector<uint8_t> buf(count + 1);
    buf[0] = reg;
    std::memcpy(&buf[1], src, count);
    return i2c_master_transmit(dev_handle, buf.data(), buf.size(), timeout_ms);
}

esp_err_t TofDriver::Impl::readMulti(uint8_t reg, uint8_t* dst, uint8_t count) {
    return i2c_master_transmit_receive(dev_handle, &reg, 1, dst, count, timeout_ms);
}

bool TofDriver::Impl::loadRegisterSequence(const char* csv_data, std::vector<RegisterWrite>& sequence) {
    sequence.clear();
    std::istringstream stream(csv_data);
    std::string line;

    while (std::getline(stream, line)) {
        if (line.empty() || line[0] == '#') continue;

        std::istringstream ls(line);
        std::string reg_str, val_str, delay_str, comment;

        if (!std::getline(ls, reg_str, ',')) continue;
        if (!std::getline(ls, val_str, ',')) continue;
        if (!std::getline(ls, delay_str, ',')) continue;
        std::getline(ls, comment);

        RegisterWrite w;
        w.reg      = (uint8_t)std::stoul(reg_str, nullptr, 16);
        w.value    = (uint8_t)std::stoul(val_str, nullptr, 16);
        w.delay_ms = (uint16_t)std::stoul(delay_str);
        w.comment  = comment;
        sequence.push_back(w);
    }
    return !sequence.empty();
}

esp_err_t TofDriver::Impl::executeRegisterSequence(const std::vector<RegisterWrite>& sequence) {
    for (const auto& w : sequence) {
        esp_err_t err = writeReg8(w.reg, w.value);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to write 0x%02X=0x%02X: %s", w.reg, w.value, esp_err_to_name(err));
            return err;
        }
        if (w.delay_ms > 0) vTaskDelay(pdMS_TO_TICKS(w.delay_ms));
    }
    return ESP_OK;
}

static const char* INIT_SEQUENCE_CSV = R"(
0x88,0x00,0,Set I2C standard mode
0x80,0x01,0,Enable power
0xFF,0x01,0,Access hidden registers
0x00,0x00,0,Clear register 0x00
0x00,0x01,0,Set register 0x00
0xFF,0x00,0,Return to normal registers
0x80,0x00,0,Disable power sequence
0x01,0xFF,0,Set SYSTEM_SEQUENCE_CONFIG
0xFF,0x01,0,Access page 1
0x00,0x00,0,Clear base register
0xFF,0x00,0,Return to page 0
0x09,0x00,0,Set register 0x09
0x10,0x00,0,Set register 0x10
0x11,0x00,0,Set register 0x11
0x24,0x01,0,Set register 0x24
0x25,0xFF,0,Set register 0x25
0x75,0x00,0,Set register 0x75
0xFF,0x01,0,Access page 1
0x4E,0x2C,0,Set register 0x4E
0x48,0x00,0,Set register 0x48
0x30,0x20,0,Set register 0x30
0xFF,0x00,0,Return to page 0
0x30,0x09,0,Set register 0x30
0x54,0x00,0,Set register 0x54
0x31,0x04,0,Set register 0x31
0x32,0x03,0,Set register 0x32
0x40,0x83,0,Set register 0x40
0x46,0x25,0,Set register 0x46
0x60,0x00,0,Set register 0x60
0x27,0x00,0,Set register 0x27
0x50,0x06,0,Set PRE_RANGE_CONFIG_VCSEL_PERIOD
0x51,0x00,0,Set PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI
0x52,0x96,0,Set PRE_RANGE_CONFIG_TIMEOUT_MACROP_LO
0x56,0x08,0,Set PRE_RANGE_CONFIG_VALID_PHASE_LOW
0x57,0x30,0,Set PRE_RANGE_CONFIG_VALID_PHASE_HIGH
0x61,0x00,0,Set PRE_RANGE_CONFIG_SIGMA_THRESH_HI
0x62,0x00,0,Set PRE_RANGE_CONFIG_SIGMA_THRESH_LO
0x64,0x00,0,Set PRE_RANGE_MIN_COUNT_RATE_RTN_LIMIT byte 1
0x65,0x00,0,Set PRE_RANGE_MIN_COUNT_RATE_RTN_LIMIT byte 2
0x66,0xA0,0,Set register 0x66
0xFF,0x01,0,Access page 1
0x22,0x32,0,Set register 0x22
0x47,0x14,0,Set FINAL_RANGE_CONFIG_VALID_PHASE_LOW
0x49,0xFF,0,Set register 0x49
0x4A,0x00,0,Set register 0x4A
0xFF,0x00,0,Return to page 0
0x7A,0x0A,0,Set register 0x7A
0x7B,0x00,0,Set register 0x7B
0x78,0x21,0,Set register 0x78
0xFF,0x01,0,Access page 1
0x23,0x34,0,Set register 0x23
0x42,0x00,0,Set register 0x42
0x44,0xFF,0,Set FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT byte 1
0x45,0x26,0,Set FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT byte 2
0x46,0x05,0,Set register 0x46
0x40,0x40,0,Set register 0x40
0x0E,0x06,0,Set register 0x0E
0x20,0x1A,0,Set register 0x20
0x43,0x40,0,Set register 0x43
0xFF,0x00,0,Return to page 0
0x34,0x03,0,Set register 0x34
0x35,0x44,0,Set register 0x35
0xFF,0x01,0,Access page 1
0x31,0x04,0,Set register 0x31
0x4B,0x09,0,Set register 0x4B
0x4C,0x05,0,Set register 0x4C
0x4D,0x04,0,Set register 0x4D
0xFF,0x00,0,Return to page 0
0x44,0x00,0,Set FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT byte 1
0x45,0x20,0,Set FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT byte 2
0x47,0x08,0,Set FINAL_RANGE_CONFIG_VALID_PHASE_LOW
0x48,0x28,0,Set FINAL_RANGE_CONFIG_VALID_PHASE_HIGH
0x67,0x00,0,Set FINAL_RANGE_CONFIG_MIN_SNR
0x70,0x04,0,Set FINAL_RANGE_CONFIG_VCSEL_PERIOD
0x71,0x01,0,Set FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI
0x72,0xFE,0,Set FINAL_RANGE_CONFIG_TIMEOUT_MACROP_LO
0x76,0x00,0,Set register 0x76
0x77,0x00,0,Set register 0x77
0xFF,0x01,0,Access page 1
0x0D,0x01,0,Set register 0x0D
0xFF,0x00,0,Return to page 0
0x80,0x01,0,Set register 0x80
0x01,0xF8,0,Set SYSTEM_SEQUENCE_CONFIG
0xFF,0x01,0,Access page 1
0x8E,0x01,0,Set register 0x8E
0x00,0x01,0,Set register 0x00
0xFF,0x00,0,Return to page 0
0x80,0x00,0,Clear register 0x80
0x0A,0x04,0,Set SYSTEM_INTERRUPT_CONFIG_GPIO
0x0B,0x01,0,Set SYSTEM_INTERRUPT_CLEAR
)";

esp_err_t TofDriver::Impl::loadInitSequence() {
    std::vector<RegisterWrite> sequence;
    if (!loadRegisterSequence(INIT_SEQUENCE_CSV, sequence)) {
        ESP_LOGE(TAG, "Failed to parse initialization sequence");
        return ESP_FAIL;
    }
    return executeRegisterSequence(sequence);
}

esp_err_t TofDriver::Impl::configureSPAD(uint8_t* count, bool* type_is_aperture) {
    *count = 32;
    *type_is_aperture = true;

    uint8_t ref_spad_map[6];
    esp_err_t err = readMulti(Reg::GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);
    if (err != ESP_OK) return err;

    err = writeReg8(0xFF, 0x01);                                          if (err != ESP_OK) return err;
    err = writeReg8(Reg::DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00);        if (err != ESP_OK) return err;
    err = writeReg8(Reg::DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C);    if (err != ESP_OK) return err;
    err = writeReg8(0xFF, 0x00);                                          if (err != ESP_OK) return err;
    err = writeReg8(Reg::GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4);       if (err != ESP_OK) return err;

    uint8_t first_spad_to_enable = *type_is_aperture ? 12 : 0;
    uint8_t spads_enabled = 0;

    for (uint8_t i = 0; i < 48; i++) {
        if (i < first_spad_to_enable || spads_enabled == *count) {
            ref_spad_map[i / 8] &= ~(1 << (i % 8));
        } else if ((ref_spad_map[i / 8] >> (i % 8)) & 0x1) {
            spads_enabled++;
        }
    }

    return writeMulti(Reg::GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);
}

esp_err_t TofDriver::Impl::performSingleRefCalibration(uint8_t vhv_init_byte) {
    esp_err_t err = writeReg8(Reg::SYSRANGE_START, 0x01 | vhv_init_byte);
    if (err != ESP_OK) return err;

    int64_t start = esp_timer_get_time();
    uint8_t status;

    do {
        err = readReg8(Reg::RESULT_INTERRUPT_STATUS, &status);
        if (err != ESP_OK) return err;

        if ((esp_timer_get_time() - start) / 1000 > timeout_ms) {
            did_timeout = true;
            return ESP_ERR_TIMEOUT;
        }
        vTaskDelay(1);
    } while ((status & 0x07) == 0);

    err = writeReg8(Reg::SYSTEM_INTERRUPT_CLEAR, 0x01);
    if (err != ESP_OK) return err;

    return writeReg8(Reg::SYSRANGE_START, 0x00);
}

uint32_t TofDriver::Impl::timeoutMclksToMicroseconds(uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks) {
    uint64_t macro_period_ns = calcMacroPeriod(vcsel_period_pclks);
    return (uint32_t)(((timeout_period_mclks * macro_period_ns) + (macro_period_ns / 2)) / 1000);
}

uint32_t TofDriver::Impl::timeoutMicrosecondsToMclks(uint32_t timeout_period_us, uint8_t vcsel_period_pclks) {
    uint64_t macro_period_ns = calcMacroPeriod(vcsel_period_pclks);
    return (uint32_t)((((uint64_t)timeout_period_us * 1000) + (macro_period_ns / 2)) / macro_period_ns);
}

uint16_t TofDriver::Impl::decodeTimeout(uint16_t reg_val) {
    return (uint16_t)((reg_val & 0x00FF) << (uint16_t)((reg_val & 0xFF00) >> 8)) + 1;
}

uint16_t TofDriver::Impl::encodeTimeout(uint16_t timeout_mclks) {
    uint32_t ls_byte = 0;
    uint16_t ms_byte = 0;

    if (timeout_mclks > 0) {
        ls_byte = timeout_mclks - 1;
        while ((ls_byte & 0xFFFFFF00) > 0) {
            ls_byte >>= 1;
            ms_byte++;
        }
        return (ms_byte << 8) | (uint16_t)(ls_byte & 0xFF);
    }
    return 0;
}

void TofDriver::Impl::getSequenceStepEnables(SequenceStepEnables* enables) {
    uint8_t seq_cfg;
    readReg8(Reg::SYSTEM_SEQUENCE_CONFIG, &seq_cfg);
    enables->tcc        = (seq_cfg >> 4) & 0x1;
    enables->dss        = (seq_cfg >> 3) & 0x1;
    enables->msrc       = (seq_cfg >> 2) & 0x1;
    enables->pre_range  = (seq_cfg >> 6) & 0x1;
    enables->final_range= (seq_cfg >> 7) & 0x1;
}

void TofDriver::Impl::getSequenceStepTimeouts(SequenceStepEnables* enables, SequenceStepTimeouts* timeouts) {
    uint8_t vcsel_reg;
    readReg8(Reg::PRE_RANGE_CONFIG_VCSEL_PERIOD, &vcsel_reg);
    timeouts->pre_range_vcsel_period_pclks = decodeVcselPeriod(vcsel_reg);

    uint8_t msrc_timeout_reg;
    readReg8(Reg::MSRC_CONFIG_TIMEOUT_MACROP, &msrc_timeout_reg);
    timeouts->msrc_dss_tcc_mclks = msrc_timeout_reg + 1;
    timeouts->msrc_dss_tcc_us    = timeoutMclksToMicroseconds(
        timeouts->msrc_dss_tcc_mclks, timeouts->pre_range_vcsel_period_pclks);

    uint16_t pre_range_timeout_reg;
    readReg16(Reg::PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI, &pre_range_timeout_reg);
    timeouts->pre_range_mclks = decodeTimeout(pre_range_timeout_reg);
    timeouts->pre_range_us    = timeoutMclksToMicroseconds(
        timeouts->pre_range_mclks, timeouts->pre_range_vcsel_period_pclks);

    readReg8(Reg::FINAL_RANGE_CONFIG_VCSEL_PERIOD, &vcsel_reg);
    timeouts->final_range_vcsel_period_pclks = decodeVcselPeriod(vcsel_reg);

    uint16_t final_range_timeout_reg;
    readReg16(Reg::FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, &final_range_timeout_reg);
    timeouts->final_range_mclks = decodeTimeout(final_range_timeout_reg);

    if (enables->pre_range)
        timeouts->final_range_mclks -= timeouts->pre_range_mclks;

    timeouts->final_range_us = timeoutMclksToMicroseconds(
        timeouts->final_range_mclks, timeouts->final_range_vcsel_period_pclks);
}

const char* TofDriver::Impl::setSignalRateLimit(float limit_mcps) {
    if (limit_mcps < 0 || limit_mcps > 511.99f) return "Rate limit out of range";
    uint16_t reg_val = (uint16_t)(limit_mcps * (1 << 7));
    esp_err_t err = writeReg16(Reg::FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, reg_val);
    return (err == ESP_OK) ? nullptr : "I2C write failed";
}

const char* TofDriver::Impl::setMeasurementTimingBudget(uint32_t budget_us) {
    SequenceStepEnables enables;
    SequenceStepTimeouts timeouts;
    getSequenceStepEnables(&enables);
    getSequenceStepTimeouts(&enables, &timeouts);

    const uint32_t StartOverhead = 1910, EndOverhead = 960, MsrcOverhead = 660;
    const uint32_t TccOverhead = 590, DssOverhead = 690, PreRangeOverhead = 660, FinalRangeOverhead = 550;

    uint32_t used_budget_us = StartOverhead + EndOverhead;
    if (enables.tcc) used_budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);
    if (enables.dss) used_budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
    else if (enables.msrc) used_budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);
    if (enables.pre_range) used_budget_us += (timeouts.pre_range_us + PreRangeOverhead);

    if (enables.final_range) {
        used_budget_us += FinalRangeOverhead;
        if (used_budget_us > budget_us) return "Timing budget too small";

        uint32_t final_range_timeout_us = budget_us - used_budget_us;
        uint16_t final_range_timeout_mclks = timeoutMicrosecondsToMclks(
            final_range_timeout_us, timeouts.final_range_vcsel_period_pclks);

        if (enables.pre_range) final_range_timeout_mclks -= timeouts.pre_range_mclks;

        writeReg16(Reg::FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, encodeTimeout(final_range_timeout_mclks));
        measurement_timing_budget_us = budget_us;
    }
    return nullptr;
}

uint32_t TofDriver::Impl::getMeasurementTimingBudget() {
    SequenceStepEnables enables;
    SequenceStepTimeouts timeouts;
    getSequenceStepEnables(&enables);
    getSequenceStepTimeouts(&enables, &timeouts);

    const uint32_t StartOverhead = 1910, EndOverhead = 960, MsrcOverhead = 660;
    const uint32_t TccOverhead = 590, DssOverhead = 690, PreRangeOverhead = 660, FinalRangeOverhead = 550;

    uint32_t budget_us = StartOverhead + EndOverhead;
    if (enables.tcc) budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);
    if (enables.dss) budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
    else if (enables.msrc) budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);
    if (enables.pre_range) budget_us += (timeouts.pre_range_us + PreRangeOverhead);
    if (enables.final_range) budget_us += (timeouts.final_range_us + FinalRangeOverhead);

    measurement_timing_budget_us = budget_us;
    return budget_us;
}

const char* TofDriver::Impl::setVcselPulsePeriod(uint8_t type, uint8_t period_pclks) {
    uint8_t vcsel_period_reg = encodeVcselPeriod(period_pclks);

    SequenceStepEnables enables;
    SequenceStepTimeouts timeouts;
    getSequenceStepEnables(&enables);
    getSequenceStepTimeouts(&enables, &timeouts);

    if (type == 0) {
        writeReg8(Reg::PRE_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg);
        uint16_t new_pre_range_timeout_mclks = timeoutMicrosecondsToMclks(timeouts.pre_range_us, period_pclks);
        writeReg16(Reg::PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI, encodeTimeout(new_pre_range_timeout_mclks));
        uint16_t new_msrc_timeout_mclks = timeoutMicrosecondsToMclks(timeouts.msrc_dss_tcc_us, period_pclks);
        writeReg8(Reg::MSRC_CONFIG_TIMEOUT_MACROP,
                 (new_msrc_timeout_mclks > 256) ? 255 : (uint8_t)(new_msrc_timeout_mclks - 1));
    } else {
        writeReg8(Reg::FINAL_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg);
        uint16_t new_final_range_timeout_mclks = timeoutMicrosecondsToMclks(timeouts.final_range_us, period_pclks);
        if (enables.pre_range) new_final_range_timeout_mclks += timeouts.pre_range_mclks;
        writeReg16(Reg::FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, encodeTimeout(new_final_range_timeout_mclks));
    }

    setMeasurementTimingBudget(measurement_timing_budget_us);
    return nullptr;
}

// ==========================================================================
// Public API
// ==========================================================================
TofDriver::TofDriver() : pimpl_(std::make_unique<Impl>()) {
    pimpl_->i2c_port               = TOF_I2C_PORT;
    pimpl_->sda_pin                = TOF_SDA_PIN;
    pimpl_->scl_pin                = TOF_SCL_PIN;
    pimpl_->i2c_address            = TOF_I2C_ADDRESS;
    pimpl_->i2c_freq_hz            = TOF_I2C_FREQ_HZ;
    pimpl_->timeout_ms             = TOF_TIMEOUT_MS;
    pimpl_->timing_budget_us       = TOF_TIMING_BUDGET_US;
    pimpl_->signal_rate_limit_mcps = TOF_SIGNAL_RATE_MCPS;
}

TofDriver::~TofDriver() {
    if (pimpl_ && pimpl_->dev_handle)
        i2c_master_bus_rm_device(pimpl_->dev_handle);
    if (pimpl_ && pimpl_->bus_handle)
        i2c_del_master_bus(pimpl_->bus_handle);
}

const char* TofDriver::configure() {
    if (!pimpl_) return "Implementation not initialized";

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "VL53L0X ToF Driver Configuration");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "I2C Port: %d", pimpl_->i2c_port);
    ESP_LOGI(TAG, "SDA: GPIO%d, SCL: GPIO%d", pimpl_->sda_pin, pimpl_->scl_pin);
    ESP_LOGI(TAG, "Address: 0x%02X, Frequency: %lu Hz", pimpl_->i2c_address, pimpl_->i2c_freq_hz);
    ESP_LOGI(TAG, "Timeout: %d ms", pimpl_->timeout_ms);
    ESP_LOGI(TAG, "========================================");

    return nullptr;
}

const char* TofDriver::init() {
    if (!pimpl_) return "Implementation not initialized";
    if (pimpl_->initialized) return nullptr;

    ESP_LOGI(TAG, "Initializing I2C bus and device...");

    i2c_master_bus_config_t bus_config = {
        .i2c_port          = pimpl_->i2c_port,
        .sda_io_num        = pimpl_->sda_pin,
        .scl_io_num        = pimpl_->scl_pin,
        .clk_source        = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority     = 0,
        .trans_queue_depth = 0,
        .flags             = { .enable_internal_pullup = true },
    };

    esp_err_t err = i2c_new_master_bus(&bus_config, &pimpl_->bus_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "FAILED to create I2C bus: %s", esp_err_to_name(err));
        ESP_LOGE(TAG, "Check: I2C port not already initialized, GPIO pins available");
        return "I2C bus creation failed";
    }
    ESP_LOGI(TAG, "I2C bus created");

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = pimpl_->i2c_address,
        .scl_speed_hz    = pimpl_->i2c_freq_hz,
        .scl_wait_us     = 0,
        .flags           = {0},
    };

    err = i2c_master_bus_add_device(pimpl_->bus_handle, &dev_config, &pimpl_->dev_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "FAILED to add device: %s", esp_err_to_name(err));
        i2c_del_master_bus(pimpl_->bus_handle);
        pimpl_->bus_handle = nullptr;
        return "Device add failed";
    }
    ESP_LOGI(TAG, "Device added to bus at 0x%02X", pimpl_->i2c_address);

    pimpl_->initialized = true;
    return nullptr;
}

const char* TofDriver::setup() {
    if (!pimpl_ || !pimpl_->initialized) return "Not initialized";

    ESP_LOGI(TAG, "Loading initialization sequences...");

    uint8_t model_id;
    esp_err_t err = pimpl_->readReg8(Reg::IDENTIFICATION_MODEL_ID, &model_id);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "FAILED to read model ID: %s", esp_err_to_name(err));
        return "Model ID read failed";
    }
    if (model_id != 0xEE) {
        ESP_LOGE(TAG, "Model ID mismatch: expected 0xEE, got 0x%02X", model_id);
        return "Wrong sensor model";
    }
    ESP_LOGI(TAG, "Model ID verified: 0x%02X", model_id);

    err = pimpl_->readReg8(0x91, &pimpl_->stop_variable);
    if (err != ESP_OK) return "Failed to read stop variable";

    err = pimpl_->loadInitSequence();
    if (err != ESP_OK) return "Init sequence failed";
    ESP_LOGI(TAG, "Initialization sequence loaded");

    uint8_t msrc_config;
    err = pimpl_->readReg8(Reg::MSRC_CONFIG_CONTROL, &msrc_config);
    if (err == ESP_OK)
        pimpl_->writeReg8(Reg::MSRC_CONFIG_CONTROL, msrc_config | 0x12);

    pimpl_->setSignalRateLimit(pimpl_->signal_rate_limit_mcps);

    uint8_t spad_count;
    bool spad_type_is_aperture;
    err = pimpl_->configureSPAD(&spad_count, &spad_type_is_aperture);
    if (err != ESP_OK)
        ESP_LOGW(TAG, "SPAD configuration warning (continuing)");

    pimpl_->writeReg8(Reg::SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04);
    uint8_t gpio_mux;
    pimpl_->readReg8(Reg::GPIO_HV_MUX_ACTIVE_HIGH, &gpio_mux);
    pimpl_->writeReg8(Reg::GPIO_HV_MUX_ACTIVE_HIGH, gpio_mux & ~0x10);
    pimpl_->writeReg8(Reg::SYSTEM_INTERRUPT_CLEAR, 0x01);

    pimpl_->measurement_timing_budget_us = pimpl_->getMeasurementTimingBudget();
    pimpl_->writeReg8(Reg::SYSTEM_SEQUENCE_CONFIG, 0xE8);
    pimpl_->setMeasurementTimingBudget(pimpl_->timing_budget_us);
    ESP_LOGI(TAG, "Timing budget set: %lu us", pimpl_->timing_budget_us);

    ESP_LOGI(TAG, "Setup complete");
    return nullptr;
}

const char* TofDriver::calibrate() {
    if (!pimpl_ || !pimpl_->initialized) return "Not initialized";

    ESP_LOGI(TAG, "Running calibrations...");

    pimpl_->writeReg8(Reg::SYSTEM_SEQUENCE_CONFIG, 0x01);
    esp_err_t err = pimpl_->performSingleRefCalibration(0x40);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "VHV calibration FAILED");
        return "VHV calibration failed";
    }
    ESP_LOGI(TAG, "VHV calibration OK");

    pimpl_->writeReg8(Reg::SYSTEM_SEQUENCE_CONFIG, 0x02);
    err = pimpl_->performSingleRefCalibration(0x00);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Phase calibration FAILED");
        return "Phase calibration failed";
    }
    ESP_LOGI(TAG, "Phase calibration OK");

    pimpl_->writeReg8(Reg::SYSTEM_SEQUENCE_CONFIG, 0xE8);

    ESP_LOGI(TAG, "Calibration complete");
    return nullptr;
}

const char* TofDriver::check() {
    if (!pimpl_ || !pimpl_->initialized) return "Not initialized";

    ESP_LOGI(TAG, "Performing health check...");

    uint8_t model_id;
    esp_err_t err = pimpl_->readReg8(Reg::IDENTIFICATION_MODEL_ID, &model_id);
    if (err != ESP_OK || model_id != 0xEE) {
        ESP_LOGE(TAG, "Health check FAILED: sensor not responding or wrong model");
        return "Health check failed";
    }

    ESP_LOGI(TAG, "Health check PASSED");
    return nullptr;
}

bool TofDriver::read_sensor(MeasurementResult& result) {
    if (!pimpl_ || !pimpl_->initialized) return false;

    pimpl_->writeReg8(0x80, 0x01);
    pimpl_->writeReg8(0xFF, 0x01);
    pimpl_->writeReg8(0x00, 0x00);
    pimpl_->writeReg8(0x91, pimpl_->stop_variable);
    pimpl_->writeReg8(0x00, 0x01);
    pimpl_->writeReg8(0xFF, 0x00);
    pimpl_->writeReg8(0x80, 0x00);

    pimpl_->writeReg8(Reg::SYSRANGE_START, 0x01);

    int64_t start = esp_timer_get_time();
    uint8_t sysrange_start_val;
    do {
        esp_err_t err = pimpl_->readReg8(Reg::SYSRANGE_START, &sysrange_start_val);
        if (err != ESP_OK) return false;

        if (pimpl_->timeout_ms > 0 &&
            (esp_timer_get_time() - start) / 1000 > pimpl_->timeout_ms) {
            result.valid = false;
            result.timeout_occurred = true;
            pimpl_->did_timeout = true;
            return false;
        }
        vTaskDelay(1);
    } while (sysrange_start_val & 0x01);

    result.timestamp_us = esp_timer_get_time();
    result.timeout_occurred = false;
    result.range_status = 0;

    start = esp_timer_get_time();
    uint8_t interrupt_status;
    do {
        esp_err_t err = pimpl_->readReg8(Reg::RESULT_INTERRUPT_STATUS, &interrupt_status);
        if (err != ESP_OK) { result.valid = false; return false; }

        if (pimpl_->timeout_ms > 0 &&
            (esp_timer_get_time() - start) / 1000 > pimpl_->timeout_ms) {
            result.valid = false;
            result.timeout_occurred = true;
            result.distance_mm = 65535;
            pimpl_->did_timeout = true;
            return false;
        }
        vTaskDelay(1);
    } while ((interrupt_status & 0x07) == 0);

    uint16_t range_mm;
    esp_err_t err = pimpl_->readReg16(Reg::RESULT_RANGE_STATUS + 10, &range_mm);
    if (err != ESP_OK) { result.valid = false; return false; }

    pimpl_->writeReg8(Reg::SYSTEM_INTERRUPT_CLEAR, 0x01);

    result.distance_mm = range_mm;
    result.valid = (range_mm < 8190);
    result.timeout_occurred = false;
    return true;
}

bool TofDriver::isReady() const {
    return pimpl_ && pimpl_->initialized;
}

void TofDriver::setTimeout(uint16_t timeout_ms) {
    if (pimpl_) pimpl_->timeout_ms = timeout_ms;
}

bool TofDriver::timeoutOccurred() {
    if (!pimpl_) return false;
    bool occurred = pimpl_->did_timeout;
    pimpl_->did_timeout = false;
    return occurred;
}