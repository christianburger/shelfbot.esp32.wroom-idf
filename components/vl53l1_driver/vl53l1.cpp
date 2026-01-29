#include "vl53l1.hpp"
#include <cstring>
#include <algorithm>
#include <vector>
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"  // Added missing include

static const char* TAG = "VL53L1";

// ===== VL53L1 REGISTER ADDRESSES =====
namespace Reg {
    constexpr uint16_t IDENTIFICATION__MODEL_ID = 0x010F;
    constexpr uint16_t VHV_CONFIG__TIMEOUT_MACROP = 0x00B2;
    constexpr uint16_t RANGE_CONFIG__VCSEL_PERIOD_A = 0x0060;
    constexpr uint16_t RANGE_CONFIG__VCSEL_PERIOD_B = 0x0063;
    constexpr uint16_t RANGE_CONFIG__TIMEOUT_MACROP_A = 0x005E;
    constexpr uint16_t RANGE_CONFIG__TIMEOUT_MACROP_B = 0x0061;
    constexpr uint16_t SYSTEM__INTERRUPT_CLEAR = 0x0086;
    constexpr uint16_t SYSTEM__MODE_START = 0x0087;
    constexpr uint16_t RESULT__RANGE_STATUS = 0x0089;
    constexpr uint16_t RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0 = 0x0096;
    constexpr uint16_t GPIO__TIO_HV_STATUS = 0x0031;
    constexpr uint16_t I2C_SLAVE__DEVICE_ADDRESS = 0x0001;
    constexpr uint16_t SYSTEM__INTERMEASUREMENT_PERIOD = 0x006C;
}

// ===== IMPLEMENTATION CLASS =====
struct VL53L1::Impl {
    // Configuration - ONLY I2C
    VL53L1::Config config;

    // I2C master bus and device handles
    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t dev_handle;

    // State
    bool initialized;
    bool i2c_owned;
    bool did_timeout;
    MeasurementMode current_mode;

    // Constructors
    Impl() : bus_handle(nullptr), dev_handle(nullptr),
             initialized(false), i2c_owned(false),
             did_timeout(false), current_mode(MeasurementMode::SINGLE) {}

    // I2C helper methods ONLY
    esp_err_t writeReg8(uint16_t reg, uint8_t value);
    esp_err_t writeReg16(uint16_t reg, uint16_t value);
    esp_err_t writeReg32(uint16_t reg, uint32_t value);
    esp_err_t readReg8(uint16_t reg, uint8_t* value);
    esp_err_t readReg16(uint16_t reg, uint16_t* value);
    esp_err_t readReg32(uint16_t reg, uint32_t* value);
    esp_err_t writeMulti(uint16_t reg, const uint8_t* src, uint8_t count);
    esp_err_t readMulti(uint16_t reg, uint8_t* dst, uint8_t count);
};

// ===== I2C HELPER IMPLEMENTATIONS =====

esp_err_t VL53L1::Impl::writeReg8(uint16_t reg, uint8_t value) {
    uint8_t write_buf[3] = {
        static_cast<uint8_t>((reg >> 8) & 0xFF),
        static_cast<uint8_t>(reg & 0xFF),
        value
    };
    return i2c_master_transmit(dev_handle, write_buf, 3, config.timeout_ms);
}

esp_err_t VL53L1::Impl::writeReg16(uint16_t reg, uint16_t value) {
    uint8_t write_buf[4] = {
        static_cast<uint8_t>((reg >> 8) & 0xFF),
        static_cast<uint8_t>(reg & 0xFF),
        static_cast<uint8_t>((value >> 8) & 0xFF),
        static_cast<uint8_t>(value & 0xFF)
    };
    return i2c_master_transmit(dev_handle, write_buf, 4, config.timeout_ms);
}

esp_err_t VL53L1::Impl::writeReg32(uint16_t reg, uint32_t value) {
    uint8_t write_buf[6] = {
        static_cast<uint8_t>((reg >> 8) & 0xFF),
        static_cast<uint8_t>(reg & 0xFF),
        static_cast<uint8_t>((value >> 24) & 0xFF),
        static_cast<uint8_t>((value >> 16) & 0xFF),
        static_cast<uint8_t>((value >> 8) & 0xFF),
        static_cast<uint8_t>(value & 0xFF)
    };
    return i2c_master_transmit(dev_handle, write_buf, 6, config.timeout_ms);
}

esp_err_t VL53L1::Impl::readReg8(uint16_t reg, uint8_t* value) {
    uint8_t reg_buf[2] = {
        static_cast<uint8_t>((reg >> 8) & 0xFF),
        static_cast<uint8_t>(reg & 0xFF)
    };
    return i2c_master_transmit_receive(dev_handle, reg_buf, 2, value, 1,
                                      config.timeout_ms);
}

esp_err_t VL53L1::Impl::readReg16(uint16_t reg, uint16_t* value) {
    uint8_t reg_buf[2] = {
        static_cast<uint8_t>((reg >> 8) & 0xFF),
        static_cast<uint8_t>(reg & 0xFF)
    };
    uint8_t buffer[2];
    esp_err_t err = i2c_master_transmit_receive(dev_handle, reg_buf, 2,
                                               buffer, 2, config.timeout_ms);
    if (err == ESP_OK) {
        *value = (static_cast<uint16_t>(buffer[0]) << 8) | buffer[1];
    }
    return err;
}

esp_err_t VL53L1::Impl::readReg32(uint16_t reg, uint32_t* value) {
    uint8_t reg_buf[2] = {
        static_cast<uint8_t>((reg >> 8) & 0xFF),
        static_cast<uint8_t>(reg & 0xFF)
    };
    uint8_t buffer[4];
    esp_err_t err = i2c_master_transmit_receive(dev_handle, reg_buf, 2,
                                               buffer, 4, config.timeout_ms);
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
    write_buf[0] = (reg >> 8) & 0xFF;
    write_buf[1] = reg & 0xFF;
    std::memcpy(&write_buf[2], src, count);
    return i2c_master_transmit(dev_handle, write_buf.data(), write_buf.size(),
                              config.timeout_ms);
}

esp_err_t VL53L1::Impl::readMulti(uint16_t reg, uint8_t* dst, uint8_t count) {
    uint8_t reg_buf[2] = {
        static_cast<uint8_t>((reg >> 8) & 0xFF),
        static_cast<uint8_t>(reg & 0xFF)
    };
    return i2c_master_transmit_receive(dev_handle, reg_buf, 2, dst, count,
                                      config.timeout_ms);
}

// ===== PUBLIC API IMPLEMENTATIONS =====

VL53L1::Config vl53l1_default_config(
    i2c_port_t i2c_port,
    gpio_num_t sda_pin,
    gpio_num_t scl_pin,
    gpio_num_t xshut_pin,
    uart_port_t uart_port,
    gpio_num_t uart_tx_pin,
    gpio_num_t uart_rx_pin
) {
    VL53L1::Config config;

    // I2C configuration
    config.i2c_port = i2c_port;
    config.sda_pin = sda_pin;
    config.scl_pin = scl_pin;
    config.xshut_pin = xshut_pin;
    config.i2c_address = 0x29;
    config.i2c_freq_hz = 400000;

    // UART configuration (for initial setup)
    config.uart_port = uart_port;
    config.uart_tx_pin = uart_tx_pin;
    config.uart_rx_pin = uart_rx_pin;
    config.uart_baud_rate = 115200;

    // Sensor configuration
    config.ranging_mode = VL53L1::RangingMode::HIGH_PRECISION;
    config.timeout_ms = 500;
    config.enable_i2c_mode = false;

    return config;
}

VL53L1::VL53L1(const Config& config) : pimpl_(std::make_unique<Impl>()) {
    pimpl_->config = config;
}

VL53L1::~VL53L1() {
    if (pimpl_) {
        if (pimpl_->dev_handle) {
            i2c_master_bus_rm_device(pimpl_->dev_handle);
        }
        if (pimpl_->i2c_owned && pimpl_->bus_handle) {
            i2c_del_master_bus(pimpl_->bus_handle);
        }
    }
}

bool VL53L1::isReady() const {
    return pimpl_ && pimpl_->initialized;
}

VL53L1::VL53L1(VL53L1&& other) noexcept = default;
VL53L1& VL53L1::operator=(VL53L1&& other) noexcept = default;

const char* VL53L1::init() {
    if (!pimpl_) {
        return "Implementation not initialized";
    }

    ESP_LOGI(TAG, "VL53L1 I2C Initialization Starting");

    // Step 1: Handle XSHUT pin if configured
    if (pimpl_->config.xshut_pin != GPIO_NUM_NC) {
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
            gpio_set_level(pimpl_->config.xshut_pin, 0);
            vTaskDelay(pdMS_TO_TICKS(10));
            gpio_set_level(pimpl_->config.xshut_pin, 1);
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }

    // Step 2: Initialize I2C bus
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
        return "I2C bus creation failed";
    }
    pimpl_->i2c_owned = true;

    // Step 3: Add device to bus
    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = pimpl_->config.i2c_address,
        .scl_speed_hz = pimpl_->config.i2c_freq_hz,
        .scl_wait_us = 0,
        .flags = {0},
    };

    err = i2c_master_bus_add_device(pimpl_->bus_handle, &dev_config,
                                   &pimpl_->dev_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "FAILED to add I2C device: %s", esp_err_to_name(err));
        i2c_del_master_bus(pimpl_->bus_handle);
        pimpl_->bus_handle = nullptr;
        return "I2C device add failed";
    }

    // Step 4: Verify model ID
    uint8_t model_id;
    err = pimpl_->readReg8(Reg::IDENTIFICATION__MODEL_ID, &model_id);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read model ID: %s", esp_err_to_name(err));
        return "Model ID read failed";
    }

    if (model_id != 0xEA) {
        ESP_LOGE(TAG, "Model ID mismatch! Expected 0xEA (VL53L1), got 0x%02X",
                model_id);
        return "Model ID mismatch";
    }

    pimpl_->initialized = true;
    ESP_LOGI(TAG, "VL53L1 I2C Initialization Complete - SUCCESS");
    return nullptr;
}

// ===== MEASUREMENT IMPLEMENTATIONS =====

bool VL53L1::readSingle(MeasurementResult& result) {
    if (!isReady()) {
        return false;
    }

    result.timestamp_us = esp_timer_get_time();
    result.timeout_occurred = false;

    esp_err_t err = pimpl_->writeReg8(Reg::SYSTEM__MODE_START, 0x10);
    if (err != ESP_OK) {
        return false;
    }

    int64_t start = esp_timer_get_time();
    uint8_t gpio_status;

    do {
        err = pimpl_->readReg8(Reg::GPIO__TIO_HV_STATUS, &gpio_status);
        if (err != ESP_OK) {
            result.valid = false;
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
    } while ((gpio_status & 0x01) == 0);

    uint8_t range_status;
    err = pimpl_->readReg8(Reg::RESULT__RANGE_STATUS, &range_status);
    if (err != ESP_OK) {
        result.valid = false;
        return false;
    }

    uint16_t distance_mm;
    err = pimpl_->readReg16(Reg::RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0,
                           &distance_mm);
    if (err != ESP_OK) {
        result.valid = false;
        return false;
    }

    pimpl_->writeReg8(Reg::SYSTEM__INTERRUPT_CLEAR, 0x01);

    result.distance_mm = distance_mm;
    result.range_status = range_status & 0x1F;
    result.valid = (result.range_status == 0) && (distance_mm < 8190);

    return true;
}

bool VL53L1::startContinuous() {
    if (!isReady()) {
        return false;
    }

    esp_err_t err = pimpl_->writeReg8(Reg::SYSTEM__MODE_START, 0x40);
    if (err != ESP_OK) {
        return false;
    }

    pimpl_->current_mode = MeasurementMode::CONTINUOUS;
    return true;
}

bool VL53L1::readContinuous(MeasurementResult& result) {
    if (!isReady()) {
        return false;
    }

    result.timestamp_us = esp_timer_get_time();
    result.timeout_occurred = false;

    int64_t start = esp_timer_get_time();
    uint8_t gpio_status;

    do {
        esp_err_t err = pimpl_->readReg8(Reg::GPIO__TIO_HV_STATUS, &gpio_status);
        if (err != ESP_OK) {
            result.valid = false;
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
    } while ((gpio_status & 0x01) == 0);

    uint8_t range_status;
    esp_err_t err = pimpl_->readReg8(Reg::RESULT__RANGE_STATUS, &range_status);
    if (err != ESP_OK) {
        result.valid = false;
        return false;
    }

    uint16_t distance_mm;
    err = pimpl_->readReg16(Reg::RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0,
                           &distance_mm);
    if (err != ESP_OK) {
        result.valid = false;
        return false;
    }

    pimpl_->writeReg8(Reg::SYSTEM__INTERRUPT_CLEAR, 0x01);

    result.distance_mm = distance_mm;
    result.range_status = range_status & 0x1F;
    result.valid = (result.range_status == 0) && (distance_mm < 8190);

    return true;
}

bool VL53L1::stopContinuous() {
    if (!isReady()) {
        return false;
    }

    esp_err_t err = pimpl_->writeReg8(Reg::SYSTEM__MODE_START, 0x00);
    if (err != ESP_OK) {
        return false;
    }

    pimpl_->current_mode = MeasurementMode::SINGLE;
    return true;
}

// ===== CONFIGURATION IMPLEMENTATIONS =====

const char* VL53L1::setRangingMode(RangingMode mode) {
    if (!isReady()) {
        return "Sensor not initialized";
    }

    ESP_LOGW(TAG, "Ranging mode can only be set during initialization via MODBUS");
    return "Mode change requires re-initialization via MODBUS";
}

VL53L1::RangingMode VL53L1::getRangingMode() const {
    return pimpl_ ? pimpl_->config.ranging_mode : RangingMode::HIGH_PRECISION;
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

    esp_err_t err = pimpl_->writeReg8(Reg::I2C_SLAVE__DEVICE_ADDRESS,
                                     new_addr & 0x7F);
    if (err == ESP_OK) {
        pimpl_->config.i2c_address = new_addr;

        i2c_master_bus_rm_device(pimpl_->dev_handle);

        i2c_device_config_t dev_config = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = new_addr,
            .scl_speed_hz = pimpl_->config.i2c_freq_hz,
            .scl_wait_us = 0,
            .flags = {0},
        };

        err = i2c_master_bus_add_device(pimpl_->bus_handle, &dev_config,
                                       &pimpl_->dev_handle);
        return (err == ESP_OK);
    }

    return false;
}

uint8_t VL53L1::getAddress() const {
    return pimpl_ ? pimpl_->config.i2c_address : 0;
}

// ===== DIAGNOSTIC IMPLEMENTATIONS =====

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

    uint8_t model_id;
    esp_err_t err = pimpl_->readReg8(Reg::IDENTIFICATION__MODEL_ID, &model_id);
    if (err != ESP_OK) {
        return "Failed to read model ID";
    }

    if (model_id != 0xEA) {
        return "Model ID check failed";
    }

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

    ESP_LOGI(TAG, "Self-test passed (distance: %d mm, status: %d)",
            result.distance_mm, result.range_status);
    return nullptr;
}
