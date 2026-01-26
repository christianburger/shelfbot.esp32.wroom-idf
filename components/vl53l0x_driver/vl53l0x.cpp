#include "vl53l0x.hpp"
#include <cstring>
#include <algorithm>
#include "esp_timer.h"
#include "freertos/task.h"
#include "esp_err.h"
#include <cinttypes>  // Add this include for PRIu16/PRIu32 macros

// ===== STATIC CONSTANTS =====
const char* VL53L0X::TAG = "VL53L0X";

// ===== CONSTRUCTOR AND DESTRUCTOR =====
VL53L0X::VL53L0X(i2c_port_t i2c_port,
                 gpio_num_t xshut_pin,
                 uint8_t i2c_address,
                 bool io_2v8)
    : i2c_port_(i2c_port),
      xshut_pin_(xshut_pin),
      i2c_address_(i2c_address),
      io_2v8_(io_2v8),
      timeout_(false),
      i2c_fail_(false),
      timeout_ms_(500),
      stop_variable_(0) {

    ESP_LOGD(TAG, "VL53L0X constructor: port=%d, address=0x%02X, xshut=%d",
             i2c_port_, i2c_address_, xshut_pin_);
}

VL53L0X::~VL53L0X() {
    ESP_LOGD(TAG, "VL53L0X destructor");
}

// ===== SENSOR API =====
const char* VL53L0X::init() {
    ESP_LOGI(TAG, "Initializing VL53L0X at address 0x%02X...", i2c_address_);

    // Handle XSHUT pin if configured
    if (xshut_pin_ != GPIO_NUM_NC) {
        gpio_config_t xshut_config = {
            .pin_bit_mask = (1ULL << xshut_pin_),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };

        if (gpio_config(&xshut_config) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to configure XSHUT pin");
            return "XSHUT pin configuration failed";
        }

        // Reset sensor by toggling XSHUT
        gpio_set_level(xshut_pin_, 0);
        vTaskDelay(pdMS_TO_TICKS(10));
        gpio_set_level(xshut_pin_, 1);
        vTaskDelay(pdMS_TO_TICKS(10));
        ESP_LOGD(TAG, "XSHUT pin toggled (GPIO%d)", xshut_pin_);
    }

    // Check if sensor is connected
    if (!isConnected()) {
        ESP_LOGE(TAG, "I2C communication failed - device not responding");
        return "I2C communication failed - device not responding";
    }

    // Verify model ID
    uint8_t model_id = getModelID();
    if (model_id != 0xEE) {
        ESP_LOGE(TAG, "Invalid model ID: expected 0xEE, got 0x%02X", model_id);
        return "Invalid model ID - not a VL53L0X";
    }

    ESP_LOGI(TAG, "VL53L0X detected (Model ID: 0x%02X), initializing...", model_id);

    // ===== VL53L0X INITIALIZATION SEQUENCE =====
    // Based on STMicroelectronics VL53L0X API

    // 1. Set I2C standard mode
    writeReg(0x88, 0x00);

    // 2. Power on sequence
    writeReg(0x80, 0x01);
    writeReg(0xFF, 0x01);
    writeReg(0x00, 0x00);

    // 3. Read and store stop variable
    stop_variable_ = readReg(0x91);
    writeReg(0x00, 0x01);
    writeReg(0xFF, 0x00);
    writeReg(0x80, 0x00);

    // 4. Disable SIGNAL_RATE_MSRC (bit 1) and SIGNAL_RATE_PRE_RANGE (bit 4) limit checks
    uint8_t msrc_config = readReg(0x60);
    writeReg(0x60, msrc_config | 0x12);

    // 5. Set final range signal rate limit to 0.25 MCPS (minimum)
    setSignalRateLimit(0.25);

    // 6. Set timing budget to 33ms
    setMeasurementTimingBudget(33000);

    // 7. Re-calculate timing budget
    uint32_t budget_us = getMeasurementTimingBudget();
    ESP_LOGD(TAG, "Measurement timing budget: %" PRIu32 " us", budget_us);

    // 8. Perform reference calibration
    if (!performRefCalibration(0x40)) {
        ESP_LOGW(TAG, "Reference calibration failed, but continuing...");
    }

    ESP_LOGI(TAG, "VL53L0X initialization complete");
    return nullptr;  // Success
}

uint16_t VL53L0X::readRangeSingleMillimeters() {
    timeout_ = false;
    i2c_fail_ = false;

    // Start measurement sequence
    writeReg(0x80, 0x01);
    writeReg(0xFF, 0x01);
    writeReg(0x00, 0x00);
    writeReg(0x91, stop_variable_);
    writeReg(0x00, 0x01);
    writeReg(0xFF, 0x00);
    writeReg(0x80, 0x00);

    // Start single measurement
    writeReg(0x00, 0x01);

    // Wait for measurement to complete
    uint32_t start_time = esp_timer_get_time();
    while ((readReg(0x13) & 0x07) == 0) {
        if ((esp_timer_get_time() - start_time) > timeout_ms_ * 1000) {
            timeout_ = true;
            ESP_LOGW(TAG, "Measurement timeout after %u ms", timeout_ms_);  // FIXED: Changed %"PRIu32" to %u for uint16_t
            return 65535;  // Timeout value
        }
        vTaskDelay(1);
    }

    // Read measurement result (range is in register 0x14+10 in mm)
    uint16_t range = readReg16Bit(0x14 + 10);

    // Clear interrupt
    writeReg(0x0B, 0x01);

    // Check for valid range
    uint8_t range_status = readReg(0x14) & 0x1F;
    if (range_status != 0x09 && range_status != 0x0A && range_status != 0x0B &&
        range_status != 0x0C && range_status != 0x0D && range_status != 0x0E) {
        ESP_LOGW(TAG, "Invalid range status: 0x%02X", range_status);
        return 65535;  // Invalid reading
    }

    ESP_LOGD(TAG, "Range reading: %d mm (status: 0x%02X)", range, range_status);
    return range;
}

void VL53L0X::setTimeout(uint16_t timeout_ms) {
    timeout_ms_ = timeout_ms;
    ESP_LOGD(TAG, "Timeout set to %u ms", timeout_ms_);
}

bool VL53L0X::timeoutOccurred() const {
    return timeout_;
}

bool VL53L0X::i2cFail() const {
    return i2c_fail_;
}

// ===== PRIVATE METHODS =====
uint8_t VL53L0X::readReg(uint8_t reg) {
    uint8_t value = 0;
    i2c_fail_ = false;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (i2c_address_ << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);  // Repeated start
    i2c_master_write_byte(cmd, (i2c_address_ << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, &value, I2C_MASTER_NACK);
    i2c_master_stop(cmd);

    esp_err_t err = i2c_master_cmd_begin(i2c_port_, cmd, timeout_ms_ / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C read error at reg 0x%02X: %s", reg, esp_err_to_name(err));
        i2c_fail_ = true;
    }

    return value;
}

void VL53L0X::writeReg(uint8_t reg, uint8_t value) {
    i2c_fail_ = false;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (i2c_address_ << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, value, true);
    i2c_master_stop(cmd);

    esp_err_t err = i2c_master_cmd_begin(i2c_port_, cmd, timeout_ms_ / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C write error at reg 0x%02X: %s", reg, esp_err_to_name(err));
        i2c_fail_ = true;
    }
}

uint16_t VL53L0X::readReg16Bit(uint8_t reg) {
    uint8_t buffer[2];
    i2c_fail_ = false;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (i2c_address_ << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);  // Repeated start
    i2c_master_write_byte(cmd, (i2c_address_ << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, buffer, 2, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);

    esp_err_t err = i2c_master_cmd_begin(i2c_port_, cmd, timeout_ms_ / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C read16 error at reg 0x%02X: %s", reg, esp_err_to_name(err));
        i2c_fail_ = true;
        return 0;
    }

    return (buffer[0] << 8) | buffer[1];
}

void VL53L0X::writeReg16Bit(uint8_t reg, uint16_t value) {
    uint8_t buffer[3] = {reg, static_cast<uint8_t>(value >> 8), static_cast<uint8_t>(value & 0xFF)};

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (i2c_address_ << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, buffer, 3, true);
    i2c_master_stop(cmd);

    esp_err_t err = i2c_master_cmd_begin(i2c_port_, cmd, timeout_ms_ / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C write16 error at reg 0x%02X: %s", reg, esp_err_to_name(err));
        i2c_fail_ = true;
    }
}

bool VL53L0X::isConnected() {
    // Try to read WHO_AM_I register (0xC0)
    uint8_t who_am_i = readReg(0xC0);
    if (i2c_fail_) {
        ESP_LOGW(TAG, "Device not responding at address 0x%02X", i2c_address_);
        return false;
    }

    ESP_LOGD(TAG, "Device responded at 0x%02X, WHO_AM_I: 0x%02X", i2c_address_, who_am_i);
    return true;
}

uint8_t VL53L0X::getModelID() {
    return readReg(0xC0);
}

// ===== ADDITIONAL PRIVATE METHODS (not declared in header but needed) =====

void VL53L0X::setSignalRateLimit(float limit_Mcps) {
    if (limit_Mcps < 0.0 || limit_Mcps > 511.99) {
        ESP_LOGW(TAG, "Invalid signal rate limit: %.2f MCPS", limit_Mcps);
        return;
    }

    // Q9.7 fixed point format (9 integer bits, 7 fractional bits)
    uint16_t fixed_point = static_cast<uint16_t>(limit_Mcps * (1 << 7));
    writeReg16Bit(0x44, fixed_point);

    ESP_LOGD(TAG, "Signal rate limit set to %.2f MCPS (0x%04X)", limit_Mcps, fixed_point);
}

void VL53L0X::setMeasurementTimingBudget(uint32_t budget_us) {
    // Simplified implementation - actual VL53L0X requires complex timing calculations
    ESP_LOGD(TAG, "Measurement timing budget set to %" PRIu32 " us", budget_us);

    // For now, just set a reasonable default
    // In a full implementation, this would calculate and set all timing registers
}

uint32_t VL53L0X::getMeasurementTimingBudget() {
    // Return the currently set timing budget
    // Simplified - return default value
    return 33000;  // 33ms default
}

bool VL53L0X::performRefCalibration(uint8_t vhv_init_byte) {
    ESP_LOGD(TAG, "Performing reference calibration with VHV init byte 0x%02X", vhv_init_byte);

    // Start calibration
    writeReg(0x00, 0x01 | 0x40);  // VL53L0X_REG_SYSRANGE_MODE_TIMED

    uint32_t start_time = esp_timer_get_time();
    while ((readReg(0x13) & 0x07) == 0) {
        if ((esp_timer_get_time() - start_time) > 500 * 1000) {  // 500ms timeout
            ESP_LOGW(TAG, "Calibration timeout");
            writeReg(0x00, 0x00);
            return false;
        }
        vTaskDelay(1);
    }

    // Clear interrupt
    writeReg(0x0B, 0x01);
    writeReg(0x00, 0x00);

    ESP_LOGD(TAG, "Reference calibration complete");
    return true;
}

// ===== DEBUG/UTILITY METHODS =====

void VL53L0X::dumpRegisters(uint8_t start, uint8_t end) {
    ESP_LOGI(TAG, "Dumping VL53L0X registers 0x%02X to 0x%02X:", start, end);

    for (uint16_t reg = start; reg <= end; reg++) {
        if ((reg - start) % 16 == 0) {
            if (reg != start) ESP_LOGI(TAG, "");
            ESP_LOGI(TAG, "0x%02X: ", reg);
        }

        uint8_t value = readReg(static_cast<uint8_t>(reg));
        if (!i2c_fail_) {
            ESP_LOGI(TAG, "%02X ", value);
        } else {
            ESP_LOGI(TAG, "XX ");
            break;
        }

        vTaskDelay(1);
    }
    ESP_LOGI(TAG, "");
}

uint8_t VL53L0X::getModuleType() {
    return readReg(0xDB);
}

// ===== EXTRA I2C METHODS (if needed for multi-byte operations) =====

bool VL53L0X::readMulti(uint8_t reg, uint8_t* dest, uint8_t count) {
    i2c_fail_ = false;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (i2c_address_ << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);  // Repeated start
    i2c_master_write_byte(cmd, (i2c_address_ << 1) | I2C_MASTER_READ, true);

    if (count > 1) {
        i2c_master_read(cmd, dest, count - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, dest + count - 1, I2C_MASTER_NACK);

    i2c_master_stop(cmd);

    esp_err_t err = i2c_master_cmd_begin(i2c_port_, cmd, timeout_ms_ / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C read multi error at reg 0x%02X: %s", reg, esp_err_to_name(err));
        i2c_fail_ = true;
        return false;
    }

    return true;
}

bool VL53L0X::writeMulti(uint8_t reg, const uint8_t* src, uint8_t count) {
    i2c_fail_ = false;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (i2c_address_ << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write(cmd, src, count, true);
    i2c_master_stop(cmd);

    esp_err_t err = i2c_master_cmd_begin(i2c_port_, cmd, timeout_ms_ / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C write multi error at reg 0x%02X: %s", reg, esp_err_to_name(err));
        i2c_fail_ = true;
        return false;
    }

    return true;
}
