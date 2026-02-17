#include "vl53l1.hpp"

static const char* TAG = "TofDriver_VL53L1";

namespace Reg {
constexpr uint16_t SOFT_RESET = 0x0000;
constexpr uint16_t I2C_SLAVE_DEVICE_ADDRESS = 0x0001;
constexpr uint16_t GPIO__TIO_HV_STATUS = 0x0031;
constexpr uint16_t SYSTEM__INTERRUPT_CLEAR = 0x0086;
constexpr uint16_t SYSTEM__MODE_START = 0x0087;
constexpr uint16_t RESULT__RANGE_STATUS = 0x0089;
constexpr uint16_t RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0 = 0x0096;
constexpr uint16_t FIRMWARE__SYSTEM_STATUS = 0x00E5;
constexpr uint16_t IDENTIFICATION__MODEL_ID = 0x010F;
} // namespace Reg

namespace {
constexpr uint8_t VL53L1_MODEL_ID = 0xEA;
constexpr int kIdReadRetries = 10;
} // namespace

VL53L1_Driver::VL53L1_Driver()
    : i2c_port_(VL53L1_I2C_PORT),
      sda_pin_(VL53L1_SDA_PIN),
      scl_pin_(VL53L1_SCL_PIN),
      i2c_address_(VL53L1_I2C_ADDRESS),
      i2c_freq_hz_(VL53L1_I2C_FREQ_HZ),
      timeout_ms_(VL53L1_TIMEOUT_MS),
      timing_budget_us_(VL53L1_TIMING_BUDGET_US),
      signal_rate_limit_mcps_(VL53L1_SIGNAL_RATE_MCPS),
      bus_handle_(nullptr),
      dev_handle_(nullptr),
      i2c_mutex_(xSemaphoreCreateMutex()),
      initialized_(false),
      did_timeout_(false),
      op_counter_(0) {}

VL53L1_Driver::~VL53L1_Driver() {
    if (dev_handle_) i2c_master_bus_rm_device(dev_handle_);
    if (bus_handle_) i2c_del_master_bus(bus_handle_);
    if (i2c_mutex_) vSemaphoreDelete(i2c_mutex_);
}

bool VL53L1_Driver::lockI2C() {
    return i2c_mutex_ && (xSemaphoreTake(i2c_mutex_, pdMS_TO_TICKS(1000)) == pdTRUE);
}

void VL53L1_Driver::unlockI2C() {
    if (i2c_mutex_) xSemaphoreGive(i2c_mutex_);
}

void VL53L1_Driver::logBuffer(const char* prefix, const uint8_t* data, size_t count) const {
#if VL53L1_DIAG_VERBOSE
    std::string line;
    line.reserve(count * 3 + 1);
    char tmp[4] = {0};
    for (size_t i = 0; i < count; i++) {
        std::snprintf(tmp, sizeof(tmp), "%02X", data[i]);
        line += tmp;
        if (i + 1 < count) line += ' ';
    }
    ESP_LOGI(TAG, "%s[%u bytes] %s", prefix, (unsigned)count, line.c_str());
#else
    (void)prefix;
    (void)data;
    (void)count;
#endif
}

uint16_t VL53L1_Driver::crc16Modbus(const uint8_t* data, size_t count) const {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < count; i++) {
        crc ^= static_cast<uint16_t>(data[i]);
        for (int bit = 0; bit < 8; bit++) {
            if (crc & 0x0001) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

esp_err_t VL53L1_Driver::writeMulti(uint16_t reg, const uint8_t* src, size_t count) {
    std::vector<uint8_t> tx(2 + count);
    tx[0] = static_cast<uint8_t>(reg >> 8);
    tx[1] = static_cast<uint8_t>(reg & 0xFF);
    std::memcpy(&tx[2], src, count);

    op_counter_++;
    ESP_LOGI(TAG, "[OP:%lu WRITE] reg=0x%04X payload=%u crc16=0x%04X", (unsigned long)op_counter_,
             reg, (unsigned)count, crc16Modbus(tx.data(), tx.size()));
    logBuffer("  TX=", tx.data(), tx.size());

    esp_err_t err = i2c_master_transmit(dev_handle_, tx.data(), tx.size(), timeout_ms_);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "[OP:%lu WRITE] failed: %s", (unsigned long)op_counter_, esp_err_to_name(err));
    }
    return err;
}

esp_err_t VL53L1_Driver::readMultiCombined(uint16_t reg, uint8_t* dst, size_t count) {
    uint8_t addr[2] = {static_cast<uint8_t>(reg >> 8), static_cast<uint8_t>(reg & 0xFF)};
    return i2c_master_transmit_receive(dev_handle_, addr, sizeof(addr), dst, count, timeout_ms_);
}

esp_err_t VL53L1_Driver::readMultiSplit(uint16_t reg, uint8_t* dst, size_t count) {
    uint8_t addr[2] = {static_cast<uint8_t>(reg >> 8), static_cast<uint8_t>(reg & 0xFF)};
    esp_err_t err = i2c_master_transmit(dev_handle_, addr, sizeof(addr), timeout_ms_);
    if (err != ESP_OK) return err;
    return i2c_master_receive(dev_handle_, dst, count, timeout_ms_);
}

esp_err_t VL53L1_Driver::readMulti(uint16_t reg, uint8_t* dst, size_t count) {
    uint8_t addr[2] = {static_cast<uint8_t>(reg >> 8), static_cast<uint8_t>(reg & 0xFF)};

    op_counter_++;
    ESP_LOGI(TAG, "[OP:%lu READ] reg=0x%04X req=%u tx_crc16=0x%04X", (unsigned long)op_counter_, reg,
             (unsigned)count, crc16Modbus(addr, sizeof(addr)));
    logBuffer("  ADDR=", addr, sizeof(addr));

    esp_err_t err = readMultiCombined(reg, dst, count);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "[OP:%lu READ] method=combined OK rx_crc16=0x%04X", (unsigned long)op_counter_,
                 crc16Modbus(dst, count));
        logBuffer("  RX=", dst, count);
        return ESP_OK;
    }

    ESP_LOGW(TAG, "[OP:%lu READ] method=combined failed: %s; retry split", (unsigned long)op_counter_,
             esp_err_to_name(err));

    err = readMultiSplit(reg, dst, count);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "[OP:%lu READ] method=split OK rx_crc16=0x%04X", (unsigned long)op_counter_,
                 crc16Modbus(dst, count));
        logBuffer("  RX=", dst, count);
    } else {
        ESP_LOGE(TAG, "[OP:%lu READ] method=split failed: %s", (unsigned long)op_counter_,
                 esp_err_to_name(err));
    }

    return err;
}

esp_err_t VL53L1_Driver::writeReg8(uint16_t reg, uint8_t value) {
    return writeMulti(reg, &value, 1);
}

esp_err_t VL53L1_Driver::writeReg16(uint16_t reg, uint16_t value) {
    uint8_t data[2] = {static_cast<uint8_t>(value >> 8), static_cast<uint8_t>(value & 0xFF)};
    return writeMulti(reg, data, sizeof(data));
}

esp_err_t VL53L1_Driver::writeReg32(uint16_t reg, uint32_t value) {
    uint8_t data[4] = {
        static_cast<uint8_t>(value >> 24),
        static_cast<uint8_t>(value >> 16),
        static_cast<uint8_t>(value >> 8),
        static_cast<uint8_t>(value & 0xFF),
    };
    return writeMulti(reg, data, sizeof(data));
}

esp_err_t VL53L1_Driver::readReg8(uint16_t reg, uint8_t* value) {
    return readMulti(reg, value, 1);
}

esp_err_t VL53L1_Driver::readReg16(uint16_t reg, uint16_t* value) {
    uint8_t data[2] = {0};
    esp_err_t err = readMulti(reg, data, sizeof(data));
    if (err == ESP_OK) *value = (static_cast<uint16_t>(data[0]) << 8) | data[1];
    return err;
}

esp_err_t VL53L1_Driver::readReg32(uint16_t reg, uint32_t* value) {
    uint8_t data[4] = {0};
    esp_err_t err = readMulti(reg, data, sizeof(data));
    if (err == ESP_OK) {
        *value = (static_cast<uint32_t>(data[0]) << 24) |
                 (static_cast<uint32_t>(data[1]) << 16) |
                 (static_cast<uint32_t>(data[2]) << 8) |
                 static_cast<uint32_t>(data[3]);
    }
    return err;
}

esp_err_t VL53L1_Driver::readReg8WithTrace(const char* label, uint16_t reg, uint8_t* value) {
    esp_err_t err = readReg8(reg, value);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "[TRACE] %s reg=0x%04X => 0x%02X", label, reg, *value);
    } else {
        ESP_LOGW(TAG, "[TRACE] %s reg=0x%04X failed: %s", label, reg, esp_err_to_name(err));
    }
    return err;
}

esp_err_t VL53L1_Driver::writeReg8AndVerify(uint16_t reg, uint8_t value) {
    esp_err_t err = writeReg8(reg, value);
    if (err != ESP_OK) return err;

    uint8_t readback = 0;
    err = readReg8(reg, &readback);
    if (err != ESP_OK) return err;

    if (readback != value) {
        ESP_LOGW(TAG, "[VERIFY] reg=0x%04X wrote=0x%02X readback=0x%02X", reg, value, readback);
    } else {
        ESP_LOGI(TAG, "[VERIFY] reg=0x%04X readback OK (0x%02X)", reg, readback);
    }
    return ESP_OK;
}

const char* VL53L1_Driver::configure() {
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "VL53L1 ToF Driver Configuration");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "I2C Port: %d", i2c_port_);
    ESP_LOGI(TAG, "SDA: GPIO%d, SCL: GPIO%d", sda_pin_, scl_pin_);
    ESP_LOGI(TAG, "Address: 0x%02X, Frequency: %lu Hz", i2c_address_, i2c_freq_hz_);
    ESP_LOGI(TAG, "Timeout: %d ms", timeout_ms_);
    ESP_LOGI(TAG, "Timing Budget: %lu us", timing_budget_us_);
    ESP_LOGI(TAG, "Signal Rate Limit: %.2f MCPS", signal_rate_limit_mcps_);
    ESP_LOGI(TAG, "========================================");
    return nullptr;
}

const char* VL53L1_Driver::init() {
    if (initialized_) return nullptr;
    if (!lockI2C()) return "Failed to lock I2C";

    i2c_master_bus_config_t bus_config = {
        .i2c_port = i2c_port_,
        .sda_io_num = sda_pin_,
        .scl_io_num = scl_pin_,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 0,
        .flags = {.enable_internal_pullup = true},
    };

    esp_err_t err = i2c_new_master_bus(&bus_config, &bus_handle_);
    if (err != ESP_OK) {
        unlockI2C();
        ESP_LOGE(TAG, "FAILED to create I2C bus: %s", esp_err_to_name(err));
        return "I2C bus creation failed";
    }

    err = i2c_master_probe(bus_handle_, i2c_address_, timeout_ms_);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Probe failed at 0x%02X: %s", i2c_address_, esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "Probe OK at 0x%02X", i2c_address_);
    }

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = i2c_address_,
        .scl_speed_hz = i2c_freq_hz_,
        .scl_wait_us = 0,
        .flags = {0},
    };

    err = i2c_master_bus_add_device(bus_handle_, &dev_config, &dev_handle_);
    if (err != ESP_OK) {
        i2c_del_master_bus(bus_handle_);
        bus_handle_ = nullptr;
        unlockI2C();
        ESP_LOGE(TAG, "FAILED to add device: %s", esp_err_to_name(err));
        return "Device add failed";
    }

    vTaskDelay(pdMS_TO_TICKS(10));

    initialized_ = true;
    unlockI2C();
    return nullptr;
}

esp_err_t VL53L1_Driver::waitForBoot() {
    const int64_t deadline = esp_timer_get_time() + (static_cast<int64_t>(timeout_ms_) * 1000);
    uint8_t fw_status = 0;
    while (esp_timer_get_time() < deadline) {
        esp_err_t err = readReg8(Reg::FIRMWARE__SYSTEM_STATUS, &fw_status);
        if (err == ESP_OK && (fw_status & 0x01)) return ESP_OK;
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    return ESP_ERR_TIMEOUT;
}

esp_err_t VL53L1_Driver::startContinuous() {
    return writeReg8AndVerify(Reg::SYSTEM__MODE_START, 0x40);
}

const char* VL53L1_Driver::setup() {
    if (!initialized_) return "Not initialized";

    uint8_t model_id = 0;
    uint8_t fw_status = 0;
    uint8_t data_ready = 0;
    esp_err_t err = ESP_FAIL;

    readReg8WithTrace("firmware_status_pre", Reg::FIRMWARE__SYSTEM_STATUS, &fw_status);
    readReg8WithTrace("gpio_data_ready_pre", Reg::GPIO__TIO_HV_STATUS, &data_ready);

    for (int i = 0; i < kIdReadRetries; i++) {
        err = readReg8(Reg::IDENTIFICATION__MODEL_ID, &model_id);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "Model ID attempt %d/%d -> 0x%02X", i + 1, kIdReadRetries, model_id);
            break;
        }
        ESP_LOGW(TAG, "Model ID attempt %d/%d failed: %s", i + 1, kIdReadRetries, esp_err_to_name(err));
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "FAILED to read model ID after %d attempts: %s", kIdReadRetries,
                 esp_err_to_name(err));
        return "Model ID read failed";
    }

    if (model_id != VL53L1_MODEL_ID) {
        ESP_LOGE(TAG, "Model ID mismatch: expected 0x%02X, got 0x%02X", VL53L1_MODEL_ID, model_id);
        return "Wrong sensor model";
    }
    ESP_LOGI(TAG, "Model ID verified: 0x%02X", model_id);

    err = writeReg8AndVerify(Reg::SOFT_RESET, 0x00);
    if (err == ESP_OK) {
        vTaskDelay(pdMS_TO_TICKS(2));
        err = writeReg8AndVerify(Reg::SOFT_RESET, 0x01);
    }
    if (err != ESP_OK) return "Soft reset failed";

    err = waitForBoot();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Sensor boot timeout: %s", esp_err_to_name(err));
        return "Boot wait failed";
    }

    err = writeReg8AndVerify(Reg::I2C_SLAVE_DEVICE_ADDRESS, static_cast<uint8_t>(i2c_address_ << 1));
    if (err != ESP_OK) return "Address register write failed";

    err = startContinuous();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start continuous mode: %s", esp_err_to_name(err));
        return "Continuous start failed";
    }

    readReg8WithTrace("firmware_status_post", Reg::FIRMWARE__SYSTEM_STATUS, &fw_status);
    readReg8WithTrace("gpio_data_ready_post", Reg::GPIO__TIO_HV_STATUS, &data_ready);

    ESP_LOGI(TAG, "Setup complete");
    return nullptr;
}

const char* VL53L1_Driver::calibrate() {
    if (!initialized_) return "Not initialized";
    // No external calibration flow implemented yet; keep explicit trace.
    ESP_LOGI(TAG, "Calibration stage currently uses sensor defaults (no extra writes)");
    return nullptr;
}

const char* VL53L1_Driver::check() {
    if (!initialized_) return "Not initialized";

    uint8_t model_id = 0;
    esp_err_t err = readReg8(Reg::IDENTIFICATION__MODEL_ID, &model_id);
    if (err != ESP_OK || model_id != VL53L1_MODEL_ID) {
        ESP_LOGE(TAG, "Health check failed (err=%s model=0x%02X)", esp_err_to_name(err), model_id);
        return "Health check failed";
    }

    ESP_LOGI(TAG, "Health check PASSED");
    return nullptr;
}

bool VL53L1_Driver::read_sensor(MeasurementResult& result) {
    result.timestamp_us = esp_timer_get_time();
    result.timeout_occurred = false;

    if (!initialized_ || !dev_handle_) {
        result.distance_mm = 0;
        result.range_status = 0xFF;
        result.valid = false;
        return false;
    }

    uint8_t range_status = 0;
    uint16_t distance_mm = 0;

    esp_err_t err = readReg8(Reg::RESULT__RANGE_STATUS, &range_status);
    if (err != ESP_OK) {
        did_timeout_ = (err == ESP_ERR_TIMEOUT || err == ESP_ERR_INVALID_STATE);
        result.timeout_occurred = did_timeout_;
        result.distance_mm = 0;
        result.range_status = 0xFE;
        result.valid = false;
        ESP_LOGE(TAG, "[READ] status read failed: %s", esp_err_to_name(err));
        return false;
    }

    err = readReg16(Reg::RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0, &distance_mm);
    if (err != ESP_OK) {
        did_timeout_ = (err == ESP_ERR_TIMEOUT || err == ESP_ERR_INVALID_STATE);
        result.timeout_occurred = did_timeout_;
        result.distance_mm = 0;
        result.range_status = 0xFE;
        result.valid = false;
        ESP_LOGE(TAG, "[READ] distance read failed: %s", esp_err_to_name(err));
        return false;
    }

    writeReg8(Reg::SYSTEM__INTERRUPT_CLEAR, 0x01);

    result.distance_mm = distance_mm;
    result.range_status = range_status;
    result.valid = (range_status == 0);

    ESP_LOGI(TAG, "[READ] Measurement complete: %u mm valid=%d status=%u", result.distance_mm,
             result.valid ? 1 : 0, result.range_status);

    return true;
}

bool VL53L1_Driver::isReady() const {
    return initialized_;
}

void VL53L1_Driver::setTimeout(uint16_t timeout_ms) {
    timeout_ms_ = timeout_ms;
}

bool VL53L1_Driver::timeoutOccurred() {
    bool tmp = did_timeout_;
    did_timeout_ = false;
    return tmp;
}
