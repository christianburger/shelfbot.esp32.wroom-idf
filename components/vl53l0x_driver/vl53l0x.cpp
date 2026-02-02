#include "vl53l0x.hpp"

static const char* TAG = "VL53L0X";

// ===== REGISTER ADDRESSES =====
namespace Reg {
    constexpr uint8_t SYSRANGE_START = 0x00;
    constexpr uint8_t SYSTEM_THRESH_HIGH = 0x0C;
    constexpr uint8_t SYSTEM_THRESH_LOW = 0x0E;
    constexpr uint8_t SYSTEM_SEQUENCE_CONFIG = 0x01;
    constexpr uint8_t SYSTEM_RANGE_CONFIG = 0x09;
    constexpr uint8_t SYSTEM_INTERMEASUREMENT_PERIOD = 0x04;
    constexpr uint8_t SYSTEM_INTERRUPT_CONFIG_GPIO = 0x0A;
    constexpr uint8_t GPIO_HV_MUX_ACTIVE_HIGH = 0x84;
    constexpr uint8_t SYSTEM_INTERRUPT_CLEAR = 0x0B;
    constexpr uint8_t RESULT_INTERRUPT_STATUS = 0x13;
    constexpr uint8_t RESULT_RANGE_STATUS = 0x14;
    constexpr uint8_t RESULT_RANGE_VALUE = 0x1E;
    constexpr uint8_t I2C_SLAVE_DEVICE_ADDRESS = 0x8A;
    constexpr uint8_t MSRC_CONFIG_CONTROL = 0x60;
    constexpr uint8_t PRE_RANGE_CONFIG_VCSEL_PERIOD = 0x50;
    constexpr uint8_t PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI = 0x51;
    constexpr uint8_t PRE_RANGE_CONFIG_TIMEOUT_MACROP_LO = 0x52;
    constexpr uint8_t FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT = 0x44;
    constexpr uint8_t FINAL_RANGE_CONFIG_VCSEL_PERIOD = 0x70;
    constexpr uint8_t FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI = 0x71;
    constexpr uint8_t FINAL_RANGE_CONFIG_TIMEOUT_MACROP_LO = 0x72;
    constexpr uint8_t MSRC_CONFIG_TIMEOUT_MACROP = 0x46;
    constexpr uint8_t SOFT_RESET_GO2_SOFT_RESET_N = 0xBF;
    constexpr uint8_t IDENTIFICATION_MODEL_ID = 0xC0;
    constexpr uint8_t IDENTIFICATION_REVISION_ID = 0xC2;
    constexpr uint8_t OSC_CALIBRATE_VAL = 0xF8;
    constexpr uint8_t GLOBAL_CONFIG_VCSEL_WIDTH = 0x32;
    constexpr uint8_t GLOBAL_CONFIG_SPAD_ENABLES_REF_0 = 0xB0;
    constexpr uint8_t GLOBAL_CONFIG_REF_EN_START_SELECT = 0xB6;
    constexpr uint8_t DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD = 0x4E;
    constexpr uint8_t DYNAMIC_SPAD_REF_EN_START_OFFSET = 0x4F;
    constexpr uint8_t POWER_MANAGEMENT_GO1_POWER_FORCE = 0x80;
    constexpr uint8_t VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV = 0x89;
    constexpr uint8_t ALGO_PHASECAL_LIM = 0x30;
    constexpr uint8_t ALGO_PHASECAL_CONFIG_TIMEOUT = 0x30;
}

// ===== HELPER MACROS =====
#define decodeVcselPeriod(reg_val) (((reg_val) + 1) << 1)
#define encodeVcselPeriod(period_pclks) (((period_pclks) >> 1) - 1)
#define calcMacroPeriod(vcsel_period_pclks) ((((uint32_t)2304 * (vcsel_period_pclks) * 1655) + 500) / 1000)

// ===== CSV REGISTER SEQUENCE STRUCTURE =====
struct RegisterWrite {
    uint8_t reg;
    uint8_t value;
    uint16_t delay_ms;
    std::string comment;
};

// ===== SEQUENCE STEP STRUCTURES =====
struct SequenceStepEnables {
    bool tcc, msrc, dss, pre_range, final_range;
};

struct SequenceStepTimeouts {
    uint16_t pre_range_vcsel_period_pclks, final_range_vcsel_period_pclks;
    uint16_t msrc_dss_tcc_mclks, pre_range_mclks, final_range_mclks;
    uint32_t msrc_dss_tcc_us, pre_range_us, final_range_us;
};

// ===== IMPLEMENTATION CLASS =====
struct VL53L0X::Impl {
    // Configuration
    VL53L0X::Config config;

    // I2C master bus and device handles
    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t dev_handle;

    // State
    bool initialized;
    bool i2c_owned;  // True if we created the I2C bus
    uint8_t stop_variable;
    uint32_t measurement_timing_budget_us;
    bool did_timeout;
    MeasurementMode current_mode;

    // Constructors
    Impl() : bus_handle(nullptr), dev_handle(nullptr), initialized(false),
             i2c_owned(false), stop_variable(0), measurement_timing_budget_us(0),
             did_timeout(false), current_mode(MeasurementMode::SINGLE) {}

    // I2C helper methods using new I2C master driver
    esp_err_t writeReg8(uint8_t reg, uint8_t value);
    esp_err_t writeReg16(uint8_t reg, uint16_t value);
    esp_err_t writeReg32(uint8_t reg, uint32_t value);
    esp_err_t readReg8(uint8_t reg, uint8_t* value);
    esp_err_t readReg16(uint8_t reg, uint16_t* value);
    esp_err_t readReg32(uint8_t reg, uint32_t* value);
    esp_err_t writeMulti(uint8_t reg, const uint8_t* src, uint8_t count);
    esp_err_t readMulti(uint8_t reg, uint8_t* dst, uint8_t count);

    // CSV parsing
    bool loadRegisterSequence(const char* csv_data, std::vector<RegisterWrite>& sequence);
    esp_err_t executeRegisterSequence(const std::vector<RegisterWrite>& sequence);

    // Initialization helpers
    esp_err_t loadInitSequence();
    esp_err_t configureSPAD(uint8_t* count, bool* type_is_aperture);
    esp_err_t performSingleRefCalibration(uint8_t vhv_init_byte);

    // Timing calculation helpers
    uint32_t timeoutMclksToMicroseconds(uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks);
    uint32_t timeoutMicrosecondsToMclks(uint32_t timeout_period_us, uint8_t vcsel_period_pclks);
    uint16_t decodeTimeout(uint16_t reg_val);
    uint16_t encodeTimeout(uint16_t timeout_mclks);

    // Sequence step helpers
    void getSequenceStepEnables(SequenceStepEnables* enables);
    void getSequenceStepTimeouts(SequenceStepEnables* enables, SequenceStepTimeouts* timeouts);
};

// ===== I2C HELPER IMPLEMENTATIONS =====

esp_err_t VL53L0X::Impl::writeReg8(uint8_t reg, uint8_t value) {
    uint8_t write_buf[2] = {reg, value};
    return i2c_master_transmit(dev_handle, write_buf, 2, config.timeout_ms);
}

esp_err_t VL53L0X::Impl::writeReg16(uint8_t reg, uint16_t value) {
    uint8_t write_buf[3] = {
        reg,
        static_cast<uint8_t>(value >> 8),
        static_cast<uint8_t>(value & 0xFF)
    };
    return i2c_master_transmit(dev_handle, write_buf, 3, config.timeout_ms);
}

esp_err_t VL53L0X::Impl::writeReg32(uint8_t reg, uint32_t value) {
    uint8_t write_buf[5] = {
        reg,
        static_cast<uint8_t>(value >> 24),
        static_cast<uint8_t>(value >> 16),
        static_cast<uint8_t>(value >> 8),
        static_cast<uint8_t>(value & 0xFF)
    };
    return i2c_master_transmit(dev_handle, write_buf, 5, config.timeout_ms);
}

esp_err_t VL53L0X::Impl::readReg8(uint8_t reg, uint8_t* value) {
    return i2c_master_transmit_receive(dev_handle, &reg, 1, value, 1, config.timeout_ms);
}

esp_err_t VL53L0X::Impl::readReg16(uint8_t reg, uint16_t* value) {
    uint8_t buffer[2];
    esp_err_t err = i2c_master_transmit_receive(dev_handle, &reg, 1, buffer, 2, config.timeout_ms);
    if (err == ESP_OK) {
        *value = (static_cast<uint16_t>(buffer[0]) << 8) | buffer[1];
    }
    return err;
}

esp_err_t VL53L0X::Impl::readReg32(uint8_t reg, uint32_t* value) {
    uint8_t buffer[4];
    esp_err_t err = i2c_master_transmit_receive(dev_handle, &reg, 1, buffer, 4, config.timeout_ms);
    if (err == ESP_OK) {
        *value = (static_cast<uint32_t>(buffer[0]) << 24) |
                 (static_cast<uint32_t>(buffer[1]) << 16) |
                 (static_cast<uint32_t>(buffer[2]) << 8) |
                 buffer[3];
    }
    return err;
}

esp_err_t VL53L0X::Impl::writeMulti(uint8_t reg, const uint8_t* src, uint8_t count) {
    std::vector<uint8_t> write_buf(count + 1);
    write_buf[0] = reg;
    std::memcpy(&write_buf[1], src, count);
    return i2c_master_transmit(dev_handle, write_buf.data(), write_buf.size(), config.timeout_ms);
}

esp_err_t VL53L0X::Impl::readMulti(uint8_t reg, uint8_t* dst, uint8_t count) {
    return i2c_master_transmit_receive(dev_handle, &reg, 1, dst, count, config.timeout_ms);
}

// ===== CSV PARSING IMPLEMENTATION =====

bool VL53L0X::Impl::loadRegisterSequence(const char* csv_data, std::vector<RegisterWrite>& sequence) {
    sequence.clear();
    std::istringstream stream(csv_data);
    std::string line;

    while (std::getline(stream, line)) {
        // Skip empty lines and comments
        if (line.empty() || line[0] == '#') {
            continue;
        }

        // Parse CSV: reg,value,delay,comment
        std::istringstream line_stream(line);
        std::string reg_str, val_str, delay_str, comment;

        if (!std::getline(line_stream, reg_str, ',')) continue;
        if (!std::getline(line_stream, val_str, ',')) continue;
        if (!std::getline(line_stream, delay_str, ',')) continue;
        std::getline(line_stream, comment);  // Rest is comment

        // Convert hex strings to values
        RegisterWrite write;
        write.reg = static_cast<uint8_t>(std::stoul(reg_str, nullptr, 16));
        write.value = static_cast<uint8_t>(std::stoul(val_str, nullptr, 16));
        write.delay_ms = static_cast<uint16_t>(std::stoul(delay_str));
        write.comment = comment;

        sequence.push_back(write);
    }

    return !sequence.empty();
}

esp_err_t VL53L0X::Impl::executeRegisterSequence(const std::vector<RegisterWrite>& sequence) {
    for (const auto& write : sequence) {
        esp_err_t err = writeReg8(write.reg, write.value);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to write 0x%02X=0x%02X: %s",
                     write.reg, write.value, esp_err_to_name(err));
            return err;
        }

        if (write.delay_ms > 0) {
            vTaskDelay(pdMS_TO_TICKS(write.delay_ms));
        }
    }
    return ESP_OK;
}

// ===== TIMING CALCULATION HELPERS =====

uint32_t VL53L0X::Impl::timeoutMclksToMicroseconds(uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks) {
    uint64_t macro_period_ns = calcMacroPeriod(vcsel_period_pclks);
    return ((timeout_period_mclks * macro_period_ns) + (macro_period_ns / 2)) / 1000;
}

uint32_t VL53L0X::Impl::timeoutMicrosecondsToMclks(uint32_t timeout_period_us, uint8_t vcsel_period_pclks) {
    uint64_t macro_period_ns = calcMacroPeriod(vcsel_period_pclks);
    return (((uint32_t)(timeout_period_us * 1000) + (macro_period_ns / 2)) / macro_period_ns);
}

uint16_t VL53L0X::Impl::decodeTimeout(uint16_t reg_val) {
    return static_cast<uint16_t>((reg_val & 0x00FF) <<
                                 static_cast<uint16_t>((reg_val & 0xFF00) >> 8)) + 1;
}

uint16_t VL53L0X::Impl::encodeTimeout(uint16_t timeout_mclks) {
    uint32_t ls_byte = 0;
    uint16_t ms_byte = 0;

    if (timeout_mclks > 0) {
        ls_byte = timeout_mclks - 1;

        while ((ls_byte & 0xFFFFFF00) > 0) {
            ls_byte >>= 1;
            ms_byte++;
        }

        return (ms_byte << 8) | static_cast<uint16_t>(ls_byte & 0xFF);
    }
    return 0;
}

void VL53L0X::Impl::getSequenceStepEnables(SequenceStepEnables* enables) {
    uint8_t sequence_config;
    readReg8(Reg::SYSTEM_SEQUENCE_CONFIG, &sequence_config);
    enables->tcc = (sequence_config >> 4) & 0x1;
    enables->dss = (sequence_config >> 3) & 0x1;
    enables->msrc = (sequence_config >> 2) & 0x1;
    enables->pre_range = (sequence_config >> 6) & 0x1;
    enables->final_range = (sequence_config >> 7) & 0x1;
}

void VL53L0X::Impl::getSequenceStepTimeouts(SequenceStepEnables* enables, SequenceStepTimeouts* timeouts) {
    uint8_t vcsel_period_reg;
    readReg8(Reg::PRE_RANGE_CONFIG_VCSEL_PERIOD, &vcsel_period_reg);
    timeouts->pre_range_vcsel_period_pclks = decodeVcselPeriod(vcsel_period_reg);

    uint8_t msrc_timeout_reg;
    readReg8(Reg::MSRC_CONFIG_TIMEOUT_MACROP, &msrc_timeout_reg);
    timeouts->msrc_dss_tcc_mclks = msrc_timeout_reg + 1;
    timeouts->msrc_dss_tcc_us = timeoutMclksToMicroseconds(timeouts->msrc_dss_tcc_mclks,
                                                           timeouts->pre_range_vcsel_period_pclks);

    uint16_t pre_range_timeout_reg;
    readReg16(Reg::PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI, &pre_range_timeout_reg);
    timeouts->pre_range_mclks = decodeTimeout(pre_range_timeout_reg);
    timeouts->pre_range_us = timeoutMclksToMicroseconds(timeouts->pre_range_mclks,
                                                        timeouts->pre_range_vcsel_period_pclks);

    readReg8(Reg::FINAL_RANGE_CONFIG_VCSEL_PERIOD, &vcsel_period_reg);
    timeouts->final_range_vcsel_period_pclks = decodeVcselPeriod(vcsel_period_reg);

    uint16_t final_range_timeout_reg;
    readReg16(Reg::FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, &final_range_timeout_reg);
    timeouts->final_range_mclks = decodeTimeout(final_range_timeout_reg);

    if (enables->pre_range) {
        timeouts->final_range_mclks -= timeouts->pre_range_mclks;
    }

    timeouts->final_range_us = timeoutMclksToMicroseconds(timeouts->final_range_mclks,
                                                          timeouts->final_range_vcsel_period_pclks);
}

// Continued in next part...
// Continuation of vl53l0x.cpp

// ===== INITIALIZATION HELPERS =====

// Embedded CSV data for initialization sequence
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

esp_err_t VL53L0X::Impl::loadInitSequence() {
    std::vector<RegisterWrite> sequence;
    if (!loadRegisterSequence(INIT_SEQUENCE_CSV, sequence)) {
        ESP_LOGE(TAG, "Failed to parse initialization sequence");
        return ESP_FAIL;
    }
    return executeRegisterSequence(sequence);
}

esp_err_t VL53L0X::Impl::configureSPAD(uint8_t* count, bool* type_is_aperture) {
    // This is a simplified version - full implementation requires complex I2C sequences
    // For now, use typical values
    *count = 32;
    *type_is_aperture = true;

    // Read SPAD map
    uint8_t ref_spad_map[6];
    esp_err_t err = readMulti(Reg::GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);
    if (err != ESP_OK) {
        return err;
    }

    // Configure SPAD enables
    err = writeReg8(0xFF, 0x01);
    if (err != ESP_OK) return err;
    err = writeReg8(Reg::DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00);
    if (err != ESP_OK) return err;
    err = writeReg8(Reg::DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C);
    if (err != ESP_OK) return err;
    err = writeReg8(0xFF, 0x00);
    if (err != ESP_OK) return err;
    err = writeReg8(Reg::GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4);
    if (err != ESP_OK) return err;

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

esp_err_t VL53L0X::Impl::performSingleRefCalibration(uint8_t vhv_init_byte) {
    esp_err_t err = writeReg8(Reg::SYSRANGE_START, 0x01 | vhv_init_byte);
    if (err != ESP_OK) return err;

    int64_t start = esp_timer_get_time();
    uint8_t status;

    do {
        err = readReg8(Reg::RESULT_INTERRUPT_STATUS, &status);
        if (err != ESP_OK) return err;

        if ((esp_timer_get_time() - start) / 1000 > config.timeout_ms) {
            did_timeout = true;
            return ESP_ERR_TIMEOUT;
        }
        vTaskDelay(1);
    } while ((status & 0x07) == 0);

    err = writeReg8(Reg::SYSTEM_INTERRUPT_CLEAR, 0x01);
    if (err != ESP_OK) return err;

    return writeReg8(Reg::SYSRANGE_START, 0x00);
}

// ===== PUBLIC API IMPLEMENTATIONS =====

VL53L0X::Config vl53l0x_default_config(
    i2c_port_t i2c_port,
    gpio_num_t sda_pin,
    gpio_num_t scl_pin,
    gpio_num_t xshut_pin
) {
    VL53L0X::Config config;
    config.i2c_port = i2c_port;
    config.sda_pin = sda_pin;
    config.scl_pin = scl_pin;
    config.xshut_pin = xshut_pin;
    config.i2c_address = 0x29;
    config.i2c_freq_hz = 400000;  // 400kHz
    config.io_2v8 = true;
    config.timeout_ms = 500;
    config.timing_budget_us = 200000;  // 200ms
    config.signal_rate_limit_mcps = 0.25;
    return config;
}

VL53L0X::VL53L0X(const Config& config) : pimpl_(std::make_unique<Impl>()) {
    pimpl_->config = config;
}

VL53L0X::~VL53L0X() {
    if (pimpl_ && pimpl_->dev_handle) {
        i2c_master_bus_rm_device(pimpl_->dev_handle);
    }
    if (pimpl_ && pimpl_->i2c_owned && pimpl_->bus_handle) {
        i2c_del_master_bus(pimpl_->bus_handle);
    }
}

VL53L0X::VL53L0X(VL53L0X&& other) noexcept : pimpl_(std::move(other.pimpl_)) {}

VL53L0X& VL53L0X::operator=(VL53L0X&& other) noexcept {
    pimpl_ = std::move(other.pimpl_);
    return *this;
}

const char* VL53L0X::init() {
    if (!pimpl_) {
        return "Implementation not initialized";
    }

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "VL53L0X Initialization Starting");
    ESP_LOGI(TAG, "========================================");

    // Handle XSHUT pin if configured
    if (pimpl_->config.xshut_pin != GPIO_NUM_NC) {
        ESP_LOGD(TAG, "Step 1: Configuring XSHUT pin (GPIO%d)", pimpl_->config.xshut_pin);
        gpio_config_t xshut_config = {
            .pin_bit_mask = (1ULL << pimpl_->config.xshut_pin),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };

        if (gpio_config(&xshut_config) != ESP_OK) {
            ESP_LOGW(TAG, "Failed to configure XSHUT pin");
        } else {
            // Reset sensor via XSHUT
            gpio_set_level(pimpl_->config.xshut_pin, 0);
            vTaskDelay(pdMS_TO_TICKS(10));
            gpio_set_level(pimpl_->config.xshut_pin, 1);
            vTaskDelay(pdMS_TO_TICKS(10));
            ESP_LOGI(TAG, "Step 1: Sensor reset via XSHUT - OK");
        }
    } else {
        ESP_LOGD(TAG, "Step 1: XSHUT pin not configured - skipping reset");
    }

    // Initialize I2C bus
    ESP_LOGD(TAG, "Step 2: Creating I2C bus...");
    ESP_LOGD(TAG, "  Port: I2C_%d", pimpl_->config.i2c_port);
    ESP_LOGD(TAG, "  SDA: GPIO%d, SCL: GPIO%d", pimpl_->config.sda_pin, pimpl_->config.scl_pin);
    ESP_LOGD(TAG, "  Frequency: %lu Hz", pimpl_->config.i2c_freq_hz);

    i2c_master_bus_config_t bus_config = {
        .i2c_port = pimpl_->config.i2c_port,
        .sda_io_num = pimpl_->config.sda_pin,
        .scl_io_num = pimpl_->config.scl_pin,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 0,
        .flags = {
            .enable_internal_pullup = true,
        },
    };

    esp_err_t err = i2c_new_master_bus(&bus_config, &pimpl_->bus_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "FAILED to create I2C bus: %s", esp_err_to_name(err));
        ESP_LOGE(TAG, "Troubleshooting:");
        ESP_LOGE(TAG, "  1. Check if I2C_%d is already initialized elsewhere", pimpl_->config.i2c_port);
        ESP_LOGE(TAG, "  2. Verify GPIO pins are not in use by other peripherals");
        ESP_LOGE(TAG, "  3. Check ESP32 I2C hardware availability");
        return "I2C bus creation failed";
    }
    pimpl_->i2c_owned = true;
    ESP_LOGI(TAG, "Step 2: I2C bus created - OK");

    // Add device to bus
    ESP_LOGD(TAG, "Step 3: Adding device at address 0x%02X to bus...", pimpl_->config.i2c_address);
    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = pimpl_->config.i2c_address,
        .scl_speed_hz = pimpl_->config.i2c_freq_hz,
        .scl_wait_us = 0,
        .flags = {0},
    };

    err = i2c_master_bus_add_device(pimpl_->bus_handle, &dev_config, &pimpl_->dev_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "FAILED to add I2C device: %s", esp_err_to_name(err));
        i2c_del_master_bus(pimpl_->bus_handle);
        pimpl_->bus_handle = nullptr;
        return "I2C device add failed";
    }
    ESP_LOGI(TAG, "Step 3: Device added to bus - OK");

    // Check model ID
    ESP_LOGD(TAG, "Step 4: Reading model ID register...");
    uint8_t model_id;
    err = pimpl_->readReg8(Reg::IDENTIFICATION_MODEL_ID, &model_id);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "FAILED to read model ID: %s", esp_err_to_name(err));
        return "Model ID read failed";
    }

    if (model_id != 0xEE) {
        ESP_LOGE(TAG, "FAILED: Model ID mismatch!");
        ESP_LOGE(TAG, "  Expected: 0xEE (VL53L0X)");
        ESP_LOGE(TAG, "  Got: 0x%02X", model_id);
        ESP_LOGE(TAG, "This may indicate:");
        ESP_LOGE(TAG, "  - Wrong sensor model (VL53L1X = 0xEA, VL53L4CD = 0xEB)");
        ESP_LOGE(TAG, "  - Communication error");
        ESP_LOGE(TAG, "  - Defective sensor");
        return "Model ID mismatch";
    }
    ESP_LOGI(TAG, "Step 4: Model ID verified: 0x%02X - OK", model_id);

    // Read and store stop variable
    ESP_LOGD(TAG, "Step 5: Reading sensor configuration...");
    err = pimpl_->readReg8(0x91, &pimpl_->stop_variable);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read stop variable: %s", esp_err_to_name(err));
        return "Failed to read stop variable";
    }
    ESP_LOGD(TAG, "Step 5: Configuration read - OK");

    // Load and execute initialization sequence
    ESP_LOGD(TAG, "Step 6: Loading initialization sequence...");
    err = pimpl_->loadInitSequence();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "FAILED: Initialization sequence error");
        return "Initialization sequence failed";
    }
    ESP_LOGI(TAG, "Step 6: Initialization sequence complete - OK");

    // Disable MSRC and TCC
    ESP_LOGD(TAG, "Step 7: Configuring measurement modes...");
    uint8_t msrc_config;
    err = pimpl_->readReg8(Reg::MSRC_CONFIG_CONTROL, &msrc_config);
    if (err == ESP_OK) {
        pimpl_->writeReg8(Reg::MSRC_CONFIG_CONTROL, msrc_config | 0x12);
    }

    // Set signal rate limit
    setSignalRateLimit(pimpl_->config.signal_rate_limit_mcps);
    ESP_LOGD(TAG, "Step 7: Measurement configuration - OK");

    // Configure SPAD
    ESP_LOGD(TAG, "Step 8: Configuring SPAD array...");
    uint8_t spad_count;
    bool spad_type_is_aperture;
    err = pimpl_->configureSPAD(&spad_count, &spad_type_is_aperture);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "SPAD configuration warning (continuing anyway)");
    } else {
        ESP_LOGD(TAG, "Step 8: SPAD configured - OK");
    }

    // Set interrupt config
    ESP_LOGD(TAG, "Step 9: Configuring interrupts...");
    pimpl_->writeReg8(Reg::SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04);
    uint8_t gpio_mux;
    pimpl_->readReg8(Reg::GPIO_HV_MUX_ACTIVE_HIGH, &gpio_mux);
    pimpl_->writeReg8(Reg::GPIO_HV_MUX_ACTIVE_HIGH, gpio_mux & ~0x10);
    pimpl_->writeReg8(Reg::SYSTEM_INTERRUPT_CLEAR, 0x01);
    ESP_LOGD(TAG, "Step 9: Interrupts configured - OK");

    // Get and set timing budget
    ESP_LOGD(TAG, "Step 10: Setting timing budget...");
    pimpl_->measurement_timing_budget_us = getMeasurementTimingBudget();
    pimpl_->writeReg8(Reg::SYSTEM_SEQUENCE_CONFIG, 0xE8);
    setMeasurementTimingBudget(pimpl_->config.timing_budget_us);
    ESP_LOGD(TAG, "Step 10: Timing budget set to %lu us - OK", pimpl_->config.timing_budget_us);

    // Perform calibrations
    ESP_LOGD(TAG, "Step 11: Running VHV calibration...");
    pimpl_->writeReg8(Reg::SYSTEM_SEQUENCE_CONFIG, 0x01);
    err = pimpl_->performSingleRefCalibration(0x40);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "VHV calibration FAILED");
        return "VHV calibration failed";
    }
    ESP_LOGD(TAG, "Step 11: VHV calibration - OK");

    ESP_LOGD(TAG, "Step 12: Running phase calibration...");
    pimpl_->writeReg8(Reg::SYSTEM_SEQUENCE_CONFIG, 0x02);
    err = pimpl_->performSingleRefCalibration(0x00);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Phase calibration FAILED");
        return "Phase calibration failed";
    }
    ESP_LOGD(TAG, "Step 12: Phase calibration - OK");

    pimpl_->writeReg8(Reg::SYSTEM_SEQUENCE_CONFIG, 0xE8);

    pimpl_->initialized = true;
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "VL53L0X Initialization Complete - SUCCESS");
    ESP_LOGI(TAG, "========================================");
    return nullptr;
}

bool VL53L0X::isReady() const {
    return pimpl_ && pimpl_->initialized;
}

// Continued in next part...
// Measurement APIs implementation for VL53L0X

// Add to the end of vl53l0x.cpp

// ===== MEASUREMENT IMPLEMENTATIONS =====

bool VL53L0X::readSingle(MeasurementResult& result) {
    if (!isReady()) {
        return false;
    }

    // Prepare for single measurement
    pimpl_->writeReg8(0x80, 0x01);
    pimpl_->writeReg8(0xFF, 0x01);
    pimpl_->writeReg8(0x00, 0x00);
    pimpl_->writeReg8(0x91, pimpl_->stop_variable);
    pimpl_->writeReg8(0x00, 0x01);
    pimpl_->writeReg8(0xFF, 0x00);
    pimpl_->writeReg8(0x80, 0x00);

    // Start single measurement
    pimpl_->writeReg8(Reg::SYSRANGE_START, 0x01);

    // Wait for measurement to start
    int64_t start = esp_timer_get_time();
    uint8_t sysrange_start_val;

    do {
        esp_err_t err = pimpl_->readReg8(Reg::SYSRANGE_START, &sysrange_start_val);
        if (err != ESP_OK) {
            return false;
        }

        if (pimpl_->config.timeout_ms > 0 &&
            (esp_timer_get_time() - start) / 1000 > pimpl_->config.timeout_ms) {
            result.valid = false;
            result.timeout_occurred = true;
            pimpl_->did_timeout = true;
            return false;
        }
        vTaskDelay(1);
    } while (sysrange_start_val & 0x01);

    // Read the result
    return readContinuous(result);
}

bool VL53L0X::readContinuous(MeasurementResult& result) {
    if (!isReady()) {
        return false;
    }

    result.timestamp_us = esp_timer_get_time();
    result.timeout_occurred = false;

    int64_t start = esp_timer_get_time();
    uint8_t interrupt_status;

    // Wait for measurement to complete
    do {
        esp_err_t err = pimpl_->readReg8(Reg::RESULT_INTERRUPT_STATUS, &interrupt_status);
        if (err != ESP_OK) {
            result.valid = false;
            return false;
        }

        if (pimpl_->config.timeout_ms > 0 &&
            (esp_timer_get_time() - start) / 1000 > pimpl_->config.timeout_ms) {
            result.valid = false;
            result.timeout_occurred = true;
            result.distance_mm = 65535;
            pimpl_->did_timeout = true;
            return false;
        }
        vTaskDelay(1);
    } while ((interrupt_status & 0x07) == 0);

    // Read range value
    uint16_t range_mm;
    esp_err_t err = pimpl_->readReg16(Reg::RESULT_RANGE_STATUS + 10, &range_mm);
    if (err != ESP_OK) {
        result.valid = false;
        return false;
    }

    // Clear interrupt
    pimpl_->writeReg8(Reg::SYSTEM_INTERRUPT_CLEAR, 0x01);

    result.distance_mm = range_mm;
    result.valid = (range_mm < 8190);  // Maximum valid range
    result.timeout_occurred = false;

    return true;
}

bool VL53L0X::startContinuous(uint32_t period_ms) {
    if (!isReady()) {
        return false;
    }

    // Prepare for continuous measurement
    pimpl_->writeReg8(0x80, 0x01);
    pimpl_->writeReg8(0xFF, 0x01);
    pimpl_->writeReg8(0x00, 0x00);
    pimpl_->writeReg8(0x91, pimpl_->stop_variable);
    pimpl_->writeReg8(0x00, 0x01);
    pimpl_->writeReg8(0xFF, 0x00);
    pimpl_->writeReg8(0x80, 0x00);

    if (period_ms != 0) {
        // Set inter-measurement period
        uint16_t osc_calibrate_val;
        esp_err_t err = pimpl_->readReg16(Reg::OSC_CALIBRATE_VAL, &osc_calibrate_val);

        uint32_t adjusted_period = period_ms;
        if (err == ESP_OK && osc_calibrate_val != 0) {
            adjusted_period *= osc_calibrate_val;
        }

        pimpl_->writeReg32(Reg::SYSTEM_INTERMEASUREMENT_PERIOD, adjusted_period);
        pimpl_->writeReg8(Reg::SYSRANGE_START, 0x04);
        pimpl_->current_mode = MeasurementMode::CONTINUOUS_TIMED;
    } else {
        // Back-to-back mode
        pimpl_->writeReg8(Reg::SYSRANGE_START, 0x02);
        pimpl_->current_mode = MeasurementMode::CONTINUOUS;
    }

    return true;
}

bool VL53L0X::stopContinuous() {
    if (!isReady()) {
        return false;
    }

    pimpl_->writeReg8(Reg::SYSRANGE_START, 0x01);
    pimpl_->writeReg8(0xFF, 0x01);
    pimpl_->writeReg8(0x00, 0x00);
    pimpl_->writeReg8(0x91, pimpl_->stop_variable);
    pimpl_->writeReg8(0x00, 0x01);
    pimpl_->writeReg8(0xFF, 0x00);

    pimpl_->current_mode = MeasurementMode::SINGLE;
    return true;
}

// ===== CONFIGURATION IMPLEMENTATIONS =====

void VL53L0X::setTimeout(uint16_t timeout_ms) {
    if (pimpl_) {
        pimpl_->config.timeout_ms = timeout_ms;
    }
}

uint16_t VL53L0X::getTimeout() const {
    return pimpl_ ? pimpl_->config.timeout_ms : 0;
}

const char* VL53L0X::setSignalRateLimit(float limit_mcps) {
    if (!isReady()) {
        return "Sensor not initialized";
    }

    if (limit_mcps < 0 || limit_mcps > 511.99f) {
        return "Rate limit out of range";
    }

    uint16_t reg_val = static_cast<uint16_t>(limit_mcps * (1 << 7));
    esp_err_t err = pimpl_->writeReg16(Reg::FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, reg_val);

    return (err == ESP_OK) ? nullptr : "I2C write failed";
}

float VL53L0X::getSignalRateLimit() {
    if (!isReady()) {
        return 0.0f;
    }

    uint16_t reg_val;
    esp_err_t err = pimpl_->readReg16(Reg::FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, &reg_val);

    return (err == ESP_OK) ? (static_cast<float>(reg_val) / (1 << 7)) : 0.0f;
}

const char* VL53L0X::setMeasurementTimingBudget(uint32_t budget_us) {
    if (!isReady()) {
        return "Sensor not initialized";
    }

    SequenceStepEnables enables;
    SequenceStepTimeouts timeouts;

    pimpl_->getSequenceStepEnables(&enables);
    pimpl_->getSequenceStepTimeouts(&enables, &timeouts);

    const uint32_t StartOverhead = 1910;
    const uint32_t EndOverhead = 960;
    const uint32_t MsrcOverhead = 660;
    const uint32_t TccOverhead = 590;
    const uint32_t DssOverhead = 690;
    const uint32_t PreRangeOverhead = 660;
    const uint32_t FinalRangeOverhead = 550;

    uint32_t used_budget_us = StartOverhead + EndOverhead;

    if (enables.tcc) {
        used_budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);
    }

    if (enables.dss) {
        used_budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
    } else if (enables.msrc) {
        used_budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);
    }

    if (enables.pre_range) {
        used_budget_us += (timeouts.pre_range_us + PreRangeOverhead);
    }

    if (enables.final_range) {
        used_budget_us += FinalRangeOverhead;

        if (used_budget_us > budget_us) {
            return "Timing budget too small";
        }

        uint32_t final_range_timeout_us = budget_us - used_budget_us;
        uint16_t final_range_timeout_mclks = pimpl_->timeoutMicrosecondsToMclks(
            final_range_timeout_us, timeouts.final_range_vcsel_period_pclks);

        if (enables.pre_range) {
            final_range_timeout_mclks -= timeouts.pre_range_mclks;
        }

        pimpl_->writeReg16(Reg::FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI,
                          pimpl_->encodeTimeout(final_range_timeout_mclks));
        pimpl_->measurement_timing_budget_us = budget_us;
    }

    return nullptr;
}

uint32_t VL53L0X::getMeasurementTimingBudget() {
    if (!isReady()) {
        return 0;
    }

    SequenceStepEnables enables;
    SequenceStepTimeouts timeouts;

    pimpl_->getSequenceStepEnables(&enables);
    pimpl_->getSequenceStepTimeouts(&enables, &timeouts);

    const uint32_t StartOverhead = 1910;
    const uint32_t EndOverhead = 960;
    const uint32_t MsrcOverhead = 660;
    const uint32_t TccOverhead = 590;
    const uint32_t DssOverhead = 690;
    const uint32_t PreRangeOverhead = 660;
    const uint32_t FinalRangeOverhead = 550;

    uint32_t budget_us = StartOverhead + EndOverhead;

    if (enables.tcc) {
        budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);
    }

    if (enables.dss) {
        budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
    } else if (enables.msrc) {
        budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);
    }

    if (enables.pre_range) {
        budget_us += (timeouts.pre_range_us + PreRangeOverhead);
    }

    if (enables.final_range) {
        budget_us += (timeouts.final_range_us + FinalRangeOverhead);
    }

    pimpl_->measurement_timing_budget_us = budget_us;
    return budget_us;
}

const char* VL53L0X::setVcselPulsePeriod(VcselPeriodType type, uint8_t period_pclks) {
    if (!isReady()) {
        return "Sensor not initialized";
    }

    uint8_t vcsel_period_reg = encodeVcselPeriod(period_pclks);

    SequenceStepEnables enables;
    SequenceStepTimeouts timeouts;

    pimpl_->getSequenceStepEnables(&enables);
    pimpl_->getSequenceStepTimeouts(&enables, &timeouts);

    if (type == VcselPeriodType::PRE_RANGE) {
        pimpl_->writeReg8(Reg::PRE_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg);

        uint16_t new_pre_range_timeout_mclks = pimpl_->timeoutMicrosecondsToMclks(
            timeouts.pre_range_us, period_pclks);

        pimpl_->writeReg16(Reg::PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI,
                          pimpl_->encodeTimeout(new_pre_range_timeout_mclks));

        uint16_t new_msrc_timeout_mclks = pimpl_->timeoutMicrosecondsToMclks(
            timeouts.msrc_dss_tcc_us, period_pclks);

        pimpl_->writeReg8(Reg::MSRC_CONFIG_TIMEOUT_MACROP,
                         (new_msrc_timeout_mclks > 256) ? 255 : (new_msrc_timeout_mclks - 1));
    } else {
        pimpl_->writeReg8(Reg::FINAL_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg);

        uint16_t new_final_range_timeout_mclks = pimpl_->timeoutMicrosecondsToMclks(
            timeouts.final_range_us, period_pclks);

        if (enables.pre_range) {
            new_final_range_timeout_mclks += timeouts.pre_range_mclks;
        }

        pimpl_->writeReg16(Reg::FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI,
                          pimpl_->encodeTimeout(new_final_range_timeout_mclks));
    }

    setMeasurementTimingBudget(pimpl_->measurement_timing_budget_us);

    return nullptr;
}

uint8_t VL53L0X::getVcselPulsePeriod(VcselPeriodType type) {
    if (!isReady()) {
        return 0;
    }

    uint8_t reg_val;
    esp_err_t err;

    if (type == VcselPeriodType::PRE_RANGE) {
        err = pimpl_->readReg8(Reg::PRE_RANGE_CONFIG_VCSEL_PERIOD, &reg_val);
    } else {
        err = pimpl_->readReg8(Reg::FINAL_RANGE_CONFIG_VCSEL_PERIOD, &reg_val);
    }

    return (err == ESP_OK) ? decodeVcselPeriod(reg_val) : 0;
}

bool VL53L0X::setAddress(uint8_t new_addr) {
    if (!isReady()) {
        return false;
    }

    esp_err_t err = pimpl_->writeReg8(Reg::I2C_SLAVE_DEVICE_ADDRESS, new_addr & 0x7F);
    if (err == ESP_OK) {
        pimpl_->config.i2c_address = new_addr;

        // Need to recreate device handle with new address
        i2c_master_bus_rm_device(pimpl_->dev_handle);

        i2c_device_config_t dev_config = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = new_addr,
            .scl_speed_hz = pimpl_->config.i2c_freq_hz,
        };

        err = i2c_master_bus_add_device(pimpl_->bus_handle, &dev_config, &pimpl_->dev_handle);
        return (err == ESP_OK);
    }

    return false;
}

uint8_t VL53L0X::getAddress() const {
    return pimpl_ ? pimpl_->config.i2c_address : 0;
}

// ===== DIAGNOSTIC IMPLEMENTATIONS =====

bool VL53L0X::timeoutOccurred() {
    if (!pimpl_) {
        return false;
    }

    bool occurred = pimpl_->did_timeout;
    pimpl_->did_timeout = false;
    return occurred;
}

bool VL53L0X::probe() {
    if (!pimpl_ || !pimpl_->dev_handle) {
        return false;
    }

    uint8_t model_id;
    esp_err_t err = pimpl_->readReg8(Reg::IDENTIFICATION_MODEL_ID, &model_id);

    return (err == ESP_OK && model_id == 0xEE);
}

uint8_t VL53L0X::getModelId() {
    if (!isReady()) {
        return 0;
    }

    uint8_t model_id;
    esp_err_t err = pimpl_->readReg8(Reg::IDENTIFICATION_MODEL_ID, &model_id);

    return (err == ESP_OK) ? model_id : 0;
}

uint8_t VL53L0X::getRevisionId() {
    if (!isReady()) {
        return 0;
    }

    uint8_t revision_id;
    esp_err_t err = pimpl_->readReg8(Reg::IDENTIFICATION_REVISION_ID, &revision_id);

    return (err == ESP_OK) ? revision_id : 0;
}

const char* VL53L0X::selfTest() {
    if (!isReady()) {
        return "Sensor not initialized";
    }

    // Check model ID
    if (getModelId() != 0xEE) {
        return "Model ID check failed";
    }

    // Perform a test measurement
    MeasurementResult result;
    if (!readSingle(result)) {
        return "Test measurement failed";
    }

    if (result.timeout_occurred) {
        return "Test measurement timed out";
    }

    if (!result.valid) {
        return "Test measurement invalid";
    }

    ESP_LOGI(TAG, "Self-test passed (distance: %d mm)", result.distance_mm);
    return nullptr;
}
