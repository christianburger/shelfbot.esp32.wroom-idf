#include "vl53l1.hpp"
static const char* TAG = "VL53L1";

// VL53L1 Register Map
namespace Reg {
    constexpr uint16_t SOFT_RESET = 0x0000;
    constexpr uint16_t I2C_SLAVE_DEVICE_ADDRESS = 0x0002;  // Fixed: Changed from 0x0001 to 0x0002 per datasheet
    constexpr uint16_t VHV_CONFIG_TIMEOUT_MACROP_LOOP_BOUND = 0x0008;
    constexpr uint16_t ALGO_CROSSTALK_COMPENSATION_VALID_HEIGHT_MM = 0x001E;
    constexpr uint16_t ALGO_RANGE_IGNORE_VALID_HEIGHT_MM = 0x0021;
    constexpr uint16_t ALGO_RANGE_MIN_CLIP = 0x002F;
    constexpr uint16_t ALGO_CONSISTENCY_CHECK_TOLERANCE = 0x002E;
    constexpr uint16_t SYSTEM__INTERMEASUREMENT_PERIOD = 0x006C;
    constexpr uint16_t SYSTEM__THRESH_HIGH = 0x0072;
    constexpr uint16_t SYSTEM__THRESH_LOW = 0x0074;
    constexpr uint16_t SYSTEM__INTERRUPT_CLEAR = 0x0086;
    constexpr uint16_t SYSTEM__MODE_START = 0x0087;
    constexpr uint16_t RESULT__INTERRUPT_STATUS = 0x008D;
    constexpr uint16_t RESULT__RANGE_STATUS = 0x008E;
    constexpr uint16_t RESULT__DISTANCE_MM = 0x0096;
    constexpr uint16_t RESULT__SIGNAL_RATE_CROSSTALK_CORRECTED_MCPS_SD0 = 0x0098;
    constexpr uint16_t RESULT__AMBIENT_RATE_MCPS_SD0 = 0x00A4;
    constexpr uint16_t PHASECAL_CONFIG_TIMEOUT_MACROP = 0x00B2;
    constexpr uint16_t RANGE_CONFIG_TIMEOUT_MACROP_A_HI = 0x00AE;
    constexpr uint16_t RANGE_CONFIG_TIMEOUT_MACROP_B_HI = 0x00BE;
    constexpr uint16_t RANGE_CONFIG_VCSEL_PERIOD_A = 0x0060;
    constexpr uint16_t RANGE_CONFIG_VCSEL_PERIOD_B = 0x0063;
    constexpr uint16_t RANGE_CONFIG_VALID_PHASE_HIGH = 0x0069;
    constexpr uint16_t SD_CONFIG_WOI_SD0 = 0x0078;
    constexpr uint16_t SD_CONFIG_INITIAL_PHASE_SD0 = 0x007A;
    constexpr uint16_t ROI_CONFIG_USER_ROI_CENTRE_SPAD = 0x007F;
    constexpr uint16_t ROI_CONFIG_USER_ROI_REQUESTED_GLOBAL_XY_SIZE = 0x0080;
    constexpr uint16_t SYSTEM__SEQUENCE_CONFIG = 0x0081;
    constexpr uint16_t POWER_MANAGEMENT_GO1_POWER_FORCE = 0x0083;
    constexpr uint16_t PAD_I2C_HV_CONFIG = 0x0088;
    constexpr uint16_t IDENTIFICATION__MODEL_ID = 0x010F;
    constexpr uint16_t IDENTIFICATION__REVISION_ID = 0x0110;
    constexpr uint16_t OSC_CALIBRATE_VAL = 0x01DE;
    constexpr uint16_t GLOBAL_CONFIG_SPAD_ENABLES_REF_0 = 0x0B0;
    constexpr uint16_t GLOBAL_CONFIG_SPAD_ENABLES_REF_1 = 0x0B1;
    constexpr uint16_t GLOBAL_CONFIG_SPAD_ENABLES_REF_2 = 0x0B2;
    constexpr uint16_t GLOBAL_CONFIG_SPAD_ENABLES_REF_3 = 0x0B3;
    constexpr uint16_t GLOBAL_CONFIG_SPAD_ENABLES_REF_4 = 0x0B4;
    constexpr uint16_t GLOBAL_CONFIG_SPAD_ENABLES_REF_5 = 0x0B5;
    constexpr uint16_t GLOBAL_CONFIG_REF_EN_START_SELECT = 0x0B6;
    constexpr uint16_t DYNAMIC_SPAD_REF_EN_START_OFFSET = 0x0B7;
    constexpr uint16_t DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD = 0x0B8;
    constexpr uint16_t MM_CONFIG_OUTER_OFFSET_MM = 0x0BD;
    // Added missing register for boot check
    constexpr uint16_t FIRMWARE__SYSTEM_STATUS = 0x0089;  // Added this missing register
}

struct VL53L1::Impl {
    VL53L1::Config config;

    // I2C handles
    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t dev_handle;

    // State
    bool initialized;
    bool i2c_owned;
    bool did_timeout;
    MeasurementMode current_mode;

    Impl() : bus_handle(nullptr), dev_handle(nullptr),
             initialized(false), i2c_owned(false),
             did_timeout(false), current_mode(MeasurementMode::SINGLE) {}

    // I2C helper methods
    esp_err_t writeReg8(uint16_t reg, uint8_t value);
    esp_err_t writeReg16(uint16_t reg, uint16_t value);
    esp_err_t writeReg32(uint16_t reg, uint32_t value);
    esp_err_t readReg8(uint16_t reg, uint8_t* value);
    esp_err_t readReg16(uint16_t reg, uint16_t* value);
    esp_err_t readReg32(uint16_t reg, uint32_t* value);
    esp_err_t writeMulti(uint16_t reg, const uint8_t* src, uint8_t count);
    esp_err_t readMulti(uint16_t reg, uint8_t* dst, uint8_t count);

    // Sensor initialization
    esp_err_t loadInitSequence();
    esp_err_t configureSPAD();
    esp_err_t performRefCalibration();
    bool waitForBoot();
    bool waitForMeasurement();
};

// I2C helper implementations
esp_err_t VL53L1::Impl::writeReg8(uint16_t reg, uint8_t value) {
    uint8_t write_buf[3] = {
        static_cast<uint8_t>(reg >> 8),
        static_cast<uint8_t>(reg & 0xFF),
        value
    };
    return i2c_master_transmit(dev_handle, write_buf, 3, config.timeout_ms);
}

esp_err_t VL53L1::Impl::writeReg16(uint16_t reg, uint16_t value) {
    uint8_t write_buf[4] = {
        static_cast<uint8_t>(reg >> 8),
        static_cast<uint8_t>(reg & 0xFF),
        static_cast<uint8_t>(value >> 8),
        static_cast<uint8_t>(value & 0xFF)
    };
    return i2c_master_transmit(dev_handle, write_buf, 4, config.timeout_ms);
}

esp_err_t VL53L1::Impl::writeReg32(uint16_t reg, uint32_t value) {
    uint8_t write_buf[6] = {
        static_cast<uint8_t>(reg >> 8),
        static_cast<uint8_t>(reg & 0xFF),
        static_cast<uint8_t>(value >> 24),
        static_cast<uint8_t>(value >> 16),
        static_cast<uint8_t>(value >> 8),
        static_cast<uint8_t>(value & 0xFF)
    };
    return i2c_master_transmit(dev_handle, write_buf, 6, config.timeout_ms);
}

esp_err_t VL53L1::Impl::readReg8(uint16_t reg, uint8_t* value) {
    uint8_t reg_buf[2] = {
        static_cast<uint8_t>(reg >> 8),
        static_cast<uint8_t>(reg & 0xFF)
    };
    return i2c_master_transmit_receive(dev_handle, reg_buf, 2, value, 1, config.timeout_ms);
}

esp_err_t VL53L1::Impl::readReg16(uint16_t reg, uint16_t* value) {
    uint8_t reg_buf[2] = {
        static_cast<uint8_t>(reg >> 8),
        static_cast<uint8_t>(reg & 0xFF)
    };
    uint8_t buffer[2];
    esp_err_t err = i2c_master_transmit_receive(dev_handle, reg_buf, 2, buffer, 2, config.timeout_ms);
    if (err == ESP_OK) {
        *value = (static_cast<uint16_t>(buffer[0]) << 8) | buffer[1];
    }
    return err;
}

esp_err_t VL53L1::Impl::readReg32(uint16_t reg, uint32_t* value) {
    uint8_t reg_buf[2] = {
        static_cast<uint8_t>(reg >> 8),
        static_cast<uint8_t>(reg & 0xFF)
    };
    uint8_t buffer[4];
    esp_err_t err = i2c_master_transmit_receive(dev_handle, reg_buf, 2, buffer, 4, config.timeout_ms);
    if (err == ESP_OK) {
        *value = (static_cast<uint32_t>(buffer[0]) << 24) |
                 (static_cast<uint32_t>(buffer[1]) << 16) |
                 (static_cast<uint32_t>(buffer[2]) << 8) |
                 buffer[3];
    }
    return err;
}

esp_err_t VL53L1::Impl::writeMulti(uint16_t reg, const uint8_t* src, uint8_t count) {
    std::vector<uint8_t> write_buf(count + 2);
    write_buf[0] = static_cast<uint8_t>(reg >> 8);
    write_buf[1] = static_cast<uint8_t>(reg & 0xFF);
    std::memcpy(&write_buf[2], src, count);
    return i2c_master_transmit(dev_handle, write_buf.data(), write_buf.size(), config.timeout_ms);
}

esp_err_t VL53L1::Impl::readMulti(uint16_t reg, uint8_t* dst, uint8_t count) {
    uint8_t reg_buf[2] = {
        static_cast<uint8_t>(reg >> 8),
        static_cast<uint8_t>(reg & 0xFF)
    };
    return i2c_master_transmit_receive(dev_handle, reg_buf, 2, dst, count, config.timeout_ms);
}

// Wait for boot - FIXED: Using correct register
bool VL53L1::Impl::waitForBoot() {
    uint8_t status;
    int64_t start = esp_timer_get_time();

    do {
        // Fixed: Using correct register address 0x0089
        esp_err_t err = readReg8(Reg::FIRMWARE__SYSTEM_STATUS, &status);
        if (err != ESP_OK) {
            return false;
        }

        if ((esp_timer_get_time() - start) / 1000 > config.timeout_ms) {
            ESP_LOGE(TAG, "VL53L1::Impl::waitForBoot:Boot timeout");
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    } while (status != 0x01);

    return true;
}

// Wait for measurement
bool VL53L1::Impl::waitForMeasurement() {
    uint8_t status;
    int64_t start = esp_timer_get_time();

    do {
        esp_err_t err = readReg8(Reg::RESULT__INTERRUPT_STATUS, &status);
        if (err != ESP_OK) {
            return false;
        }

        if ((esp_timer_get_time() - start) / 1000 > config.timeout_ms) {
            did_timeout = true;
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    } while ((status & 0x07) == 0);

    return true;
}

// Initialize sensor
esp_err_t VL53L1::Impl::loadInitSequence() {
    // Simplified initialization sequence for VL53L1
    // This is a basic sequence - full initialization would require more registers

    // Set ranging mode based on config
    if (config.ranging_mode == RangingMode::LONG_DISTANCE) {
        // Long range mode (200ms, 4m)
        writeReg8(0x002E, 0x01);  // Ranging config
        writeReg32(Reg::SYSTEM__INTERMEASUREMENT_PERIOD, 200000);  // 200ms
    } else {
        // High precision mode (30ms, 1.3m)
        writeReg8(0x002E, 0x00);  // Ranging config
        writeReg32(Reg::SYSTEM__INTERMEASUREMENT_PERIOD, 30000);   // 30ms
    }

    // Clear interrupts
    writeReg8(Reg::SYSTEM__INTERRUPT_CLEAR, 0x01);

    // Set timing budget
    writeReg16(Reg::RANGE_CONFIG_TIMEOUT_MACROP_A_HI, 0x001D);
    writeReg16(Reg::RANGE_CONFIG_TIMEOUT_MACROP_B_HI, 0x0027);

    return ESP_OK;
}

// Configure SPAD
esp_err_t VL53L1::Impl::configureSPAD() {
    // Enable SPADs - typical configuration
    uint8_t spad_enables[6] = {0x01, 0x00, 0x02, 0x00, 0x00, 0x00};
    writeMulti(Reg::GLOBAL_CONFIG_SPAD_ENABLES_REF_0, spad_enables, 6);

    // Request dynamic SPADs
    writeReg8(Reg::DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C);

    return ESP_OK;
}

// Perform calibration
esp_err_t VL53L1::Impl::performRefCalibration() {
    // Start calibration
    writeReg8(Reg::SYSTEM__MODE_START, 0x01);

    // Wait for calibration to complete
    uint8_t status;
    int64_t start = esp_timer_get_time();

    do {
        readReg8(Reg::RESULT__INTERRUPT_STATUS, &status);

        if ((esp_timer_get_time() - start) / 1000 > config.timeout_ms) {
            return ESP_ERR_TIMEOUT;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    } while ((status & 0x07) == 0);

    // Clear interrupt
    writeReg8(Reg::SYSTEM__INTERRUPT_CLEAR, 0x01);

    return ESP_OK;
}

// Public API implementations
VL53L1::VL53L1(const Config& config) : pimpl_(std::make_unique<Impl>()) {
    pimpl_->config = config;
}

VL53L1::~VL53L1() {
    if (pimpl_ && pimpl_->dev_handle) {
        i2c_master_bus_rm_device(pimpl_->dev_handle);
    }
    if (pimpl_ && pimpl_->i2c_owned && pimpl_->bus_handle) {
        i2c_del_master_bus(pimpl_->bus_handle);
    }
}

VL53L1::VL53L1(VL53L1&& other) noexcept : pimpl_(std::move(other.pimpl_)) {}

VL53L1& VL53L1::operator=(VL53L1&& other) noexcept {
    pimpl_ = std::move(other.pimpl_);
    return *this;
}

const char* VL53L1::init() {
    if (pimpl_->initialized) {
        return nullptr;
    }

    ESP_LOGI(TAG, "Initializing VL53L1 via I2C...");
    ESP_LOGI(TAG, "  Port: I2C_%d, Addr: 0x%02X", pimpl_->config.i2c_port, pimpl_->config.i2c_address);
    ESP_LOGI(TAG, "  Pins: SDA=GPIO%d, SCL=GPIO%d", pimpl_->config.sda_pin, pimpl_->config.scl_pin);

    // Initialize I2C bus
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
        ESP_LOGE(TAG, "Failed to create I2C bus: %s", esp_err_to_name(err));
        return "I2C bus creation failed";
    }
    pimpl_->i2c_owned = true;

    // Add device to bus
    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = pimpl_->config.i2c_address,
        .scl_speed_hz = pimpl_->config.i2c_freq_hz,
        .scl_wait_us = 0,
        .flags = {0},
    };

    err = i2c_master_bus_add_device(pimpl_->bus_handle, &dev_config, &pimpl_->dev_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add I2C device: %s", esp_err_to_name(err));
        return "I2C device add failed";
    }

    // Check model ID
    uint8_t model_id;
    err = pimpl_->readReg8(Reg::IDENTIFICATION__MODEL_ID, &model_id);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read model ID: %s", esp_err_to_name(err));
        return "Model ID read failed";
    }

    if (model_id != 0xEA) {
        ESP_LOGE(TAG, "Model ID mismatch: expected 0xEA (VL53L1), got 0x%02X", model_id);
        return "Model ID mismatch";
    }

    ESP_LOGI(TAG, "Model ID verified: 0x%02X", model_id);

    // Wait for boot
    if (!pimpl_->waitForBoot()) {
        ESP_LOGE(TAG, "Sensor boot failed");
        return "Sensor boot failed";
    }

    // Configure SPAD
    err = pimpl_->configureSPAD();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "SPAD configuration warning");
    }

    // Load initialization sequence
    err = pimpl_->loadInitSequence();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Initialization sequence failed");
        return "Initialization sequence failed";
    }

    // Perform calibration
    err = pimpl_->performRefCalibration();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Calibration warning (continuing)");
    }

    pimpl_->initialized = true;
    ESP_LOGI(TAG, "VL53L1 initialization complete");

    return nullptr;
}

bool VL53L1::isReady() const {
    return pimpl_ && pimpl_->initialized;
}

bool VL53L1::readSingle(MeasurementResult& result) {
    if (!isReady()) {
        return false;
    }

    // Start single measurement
    pimpl_->writeReg8(Reg::SYSTEM__MODE_START, 0x01);

    // Wait for measurement
    if (!pimpl_->waitForMeasurement()) {
        result.valid = false;
        result.timeout_occurred = pimpl_->did_timeout;
        return false;
    }

    // Read results
    uint16_t distance;
    uint8_t status;

    pimpl_->readReg16(Reg::RESULT__DISTANCE_MM, &distance);
    pimpl_->readReg8(Reg::RESULT__RANGE_STATUS, &status);

    // Clear interrupt
    pimpl_->writeReg8(Reg::SYSTEM__INTERRUPT_CLEAR, 0x01);

    // Fill result
    result.timestamp_us = esp_timer_get_time();
    result.distance_mm = distance;
    result.range_status = status;
    result.valid = (status == 0) && (distance < 4000);  // Valid if status is 0 and distance < 4m
    result.timeout_occurred = false;

    return true;
}

bool VL53L1::startContinuous() {
    if (!isReady()) {
        return false;
    }

    // Start continuous measurements
    pimpl_->writeReg8(Reg::SYSTEM__MODE_START, 0x03);
    pimpl_->current_mode = MeasurementMode::CONTINUOUS;

    return true;
}

bool VL53L1::readContinuous(MeasurementResult& result) {
    if (!isReady()) {
        return false;
    }

    if (pimpl_->current_mode != MeasurementMode::CONTINUOUS) {
        ESP_LOGE(TAG, "Not in continuous mode");
        return false;
    }

    // Wait for measurement
    if (!pimpl_->waitForMeasurement()) {
        result.valid = false;
        result.timeout_occurred = pimpl_->did_timeout;
        return false;
    }

    // Read results
    uint16_t distance;
    uint8_t status;

    pimpl_->readReg16(Reg::RESULT__DISTANCE_MM, &distance);
    pimpl_->readReg8(Reg::RESULT__RANGE_STATUS, &status);

    // Clear interrupt
    pimpl_->writeReg8(Reg::SYSTEM__INTERRUPT_CLEAR, 0x01);

    // Fill result
    result.timestamp_us = esp_timer_get_time();
    result.distance_mm = distance;
    result.range_status = status;
    result.valid = (status == 0) && (distance < 4000);
    result.timeout_occurred = false;

    return true;
}

bool VL53L1::stopContinuous() {
    if (!isReady()) {
        return false;
    }

    // Stop measurements
    pimpl_->writeReg8(Reg::SYSTEM__MODE_START, 0x00);
    pimpl_->current_mode = MeasurementMode::SINGLE;

    return true;
}

const char* VL53L1::setRangingMode(RangingMode mode) {
    if (!isReady()) {
        return "Sensor not ready";
    }

    // Save mode and reinitialize with new settings
    pimpl_->config.ranging_mode = mode;
    return init();
}

VL53L1::RangingMode VL53L1::getRangingMode() const {
    return pimpl_->config.ranging_mode;
}

void VL53L1::setTimeout(uint16_t timeout_ms) {
    if (pimpl_) {
        pimpl_->config.timeout_ms = timeout_ms;
    }
}

uint16_t VL53L1::getTimeout() const {
    return pimpl_ ? pimpl_->config.timeout_ms : 0;
}

bool VL53L1::setAddress(uint8_t new_addr) {
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
            .scl_wait_us = 0,
            .flags = {0},
        };

        err = i2c_master_bus_add_device(pimpl_->bus_handle, &dev_config, &pimpl_->dev_handle);
        return (err == ESP_OK);
    }

    return false;
}

uint8_t VL53L1::getAddress() const {
    return pimpl_ ? pimpl_->config.i2c_address : 0;
}

bool VL53L1::timeoutOccurred() {
    if (!pimpl_) {
        return false;
    }

    bool occurred = pimpl_->did_timeout;
    pimpl_->did_timeout = false;
    return occurred;
}

bool VL53L1::probe() {
    if (!pimpl_ || !pimpl_->dev_handle) {
        return false;
    }

    uint8_t model_id;
    esp_err_t err = pimpl_->readReg8(Reg::IDENTIFICATION__MODEL_ID, &model_id);

    return (err == ESP_OK && model_id == 0xEA);
}

const char* VL53L1::selfTest() {
    if (!isReady()) {
        return "Sensor not initialized";
    }

    // Check model ID
    if (!probe()) {
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

VL53L1::Config vl53l1_default_config(
    i2c_port_t port,
    gpio_num_t sda,
    gpio_num_t scl) {

    VL53L1::Config config;
    config.i2c_port = port;
    config.sda_pin = sda;
    config.scl_pin = scl;
    config.i2c_address = 0x29;
    config.i2c_freq_hz = 400000;
    config.ranging_mode = VL53L1::RangingMode::LONG_DISTANCE;  // Fixed: Added namespace
    config.timeout_ms = 500;
    return config;
}