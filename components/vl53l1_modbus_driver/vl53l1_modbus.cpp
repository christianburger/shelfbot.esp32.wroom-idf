#include "vl53l1_modbus.hpp"

#include "vl53l1.hpp"
#include "duart_modbus.hpp"

#include <vector>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_task_wdt.h"

static const char* TAG = "VL53L1_Modbus";

namespace TOF400FReg {
    constexpr uint16_t RESTORE_DEFAULT     = 0x0001;
    constexpr uint16_t DEVICE_ADDRESS      = 0x0002;
    constexpr uint16_t BAUD_RATE           = 0x0003;
    constexpr uint16_t RANGE_MODE          = 0x0004;
    constexpr uint16_t AUTO_OUTPUT         = 0x0005;
    constexpr uint16_t LOAD_CALIBRATION    = 0x0006;
    constexpr uint16_t OFFSET_CALIBRATION  = 0x0007;
    constexpr uint16_t XTALK_CALIBRATION   = 0x0008;
    constexpr uint16_t I2C_ENABLE          = 0x0009;
    constexpr uint16_t MEASUREMENT_RESULT  = 0x0010;
}

struct VL53L1_Modbus::Impl {
    VL53L1_Modbus::Config config;
    std::unique_ptr<DuartModbus> modbus;
    std::unique_ptr<VL53L1> vl53l1;
    bool initialized;
    bool i2c_owned;
    bool did_timeout;

    Impl() : initialized(false), i2c_owned(false), did_timeout(false) {}

    bool sendModbusCommand(uint16_t reg, uint16_t value) {
        if (!modbus || !modbus->isReady()) {
            ESP_LOGE(TAG, "Modbus not ready");
            return false;
        }

        auto response = modbus->writeSingleRegister(config.modbus_slave_address, reg, value, true);
        if (!response.success) {
            ESP_LOGE(TAG, "Modbus command failed: reg=0x%04X, value=0x%04X", reg, value);
            return false;
        }

        ESP_LOGD(TAG, "Modbus success: reg=0x%04X, value=0x%04X", reg, value);
        return true;
    }

    bool verifyModbusRegister(uint16_t reg, uint16_t expected_value) {
        if (!modbus || !modbus->isReady()) {
            return false;
        }

        auto response = modbus->readHoldingRegisters(config.modbus_slave_address, reg, 1);
        if (!response.success || response.data.size() < 2) {
            return false;
        }

        uint16_t read_value = (response.data[0] << 8) | response.data[1];
        bool matches = (read_value == expected_value);

        if (!matches) {
            ESP_LOGW(TAG, "Register 0x%04X mismatch: expected 0x%04X, got 0x%04X",
                    reg, expected_value, read_value);
        }

        return matches;
    }

    bool configureModule() {
        ESP_LOGI(TAG, "Configuring TOF400F module via Modbus...");

        DuartModbus::Config modbus_config = {
            .uart_port = config.uart_port,
            .tx_pin = config.uart_tx_pin,
            .rx_pin = config.uart_rx_pin,
            .baud_rate = config.uart_baud_rate,
            .parity = 0,
            .stop_bits = 1,
            .data_bits = 8,
            .timeout_ms = config.timeout_ms,
            .max_retries = 3
        };

        modbus = std::make_unique<DuartModbus>(modbus_config);
        const char* modbus_err = modbus->init();
        if (modbus_err != nullptr) {
            ESP_LOGE(TAG, "Modbus initialization failed: %s", modbus_err);
            return false;
        }

        vTaskDelay(pdMS_TO_TICKS(100));

        ESP_LOGD(TAG, "Loading factory calibration...");
        if (!sendModbusCommand(TOF400FReg::LOAD_CALIBRATION, 0x0001)) {
            ESP_LOGE(TAG, "Failed to load calibration");
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(100));

        uint16_t mode_value = (config.ranging_mode == RangingMode::LONG_DISTANCE) ? 0x0001 : 0x0000;
        ESP_LOGD(TAG, "Setting ranging mode: %s",
                 (mode_value == 0x0001) ? "LONG_DISTANCE" : "HIGH_PRECISION");

        if (!sendModbusCommand(TOF400FReg::RANGE_MODE, mode_value)) {
            ESP_LOGE(TAG, "Failed to set ranging mode");
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(100));

        if (!sendModbusCommand(TOF400FReg::AUTO_OUTPUT, 0x0000)) {
            ESP_LOGW(TAG, "Failed to disable auto output (non-critical)");
        }
        vTaskDelay(pdMS_TO_TICKS(100));

        ESP_LOGD(TAG, "Enabling I2C mode...");
        if (!sendModbusCommand(TOF400FReg::I2C_ENABLE, 0x0001)) {
            ESP_LOGE(TAG, "Failed to enable I2C mode");
            return false;
        }

        ESP_LOGI(TAG, "TOF400F module configured successfully");
        return true;
    }

    bool waitForI2CMode(uint32_t timeout_ms = 1000) {
        ESP_LOGD(TAG, "Waiting for module to switch to I2C mode...");

        uint64_t start_time = esp_timer_get_time() / 1000;
        while ((esp_timer_get_time() / 1000 - start_time) < timeout_ms) {
            if (vl53l1 && vl53l1->probe()) {
                ESP_LOGD(TAG, "VL53L1 detected on I2C bus");
                return true;
            }
            vTaskDelay(pdMS_TO_TICKS(50));
        }

        ESP_LOGE(TAG, "Timeout waiting for I2C mode");
        return false;
    }

    bool initializeVL53L1() {
        ESP_LOGI(TAG, "Initializing VL53L1 via I2C...");

        // FIXED: Use the correct number of arguments for vl53l1_default_config
        VL53L1::Config vl53l1_config = vl53l1_default_config(
            config.i2c_port,
            config.sda_pin,
            config.scl_pin
        );

        // Manually set additional config parameters that aren't in the default config
        vl53l1_config.i2c_address = config.i2c_address;
        vl53l1_config.i2c_freq_hz = config.i2c_freq_hz;

        // Convert RangingMode
        vl53l1_config.ranging_mode = (config.ranging_mode == RangingMode::LONG_DISTANCE)
            ? VL53L1::RangingMode::LONG_DISTANCE
            : VL53L1::RangingMode::HIGH_PRECISION;

        vl53l1_config.timeout_ms = config.timeout_ms;

        vl53l1 = std::make_unique<VL53L1>(vl53l1_config);
        const char* err = vl53l1->init();
        if (err != nullptr) {
            ESP_LOGE(TAG, "VL53L1 initialization failed: %s", err);
            return false;
        }

        return true;
    }
};

VL53L1_Modbus::VL53L1_Modbus(const Config& config)
    : pimpl_(std::make_unique<Impl>()) {
    pimpl_->config = config;
}

VL53L1_Modbus::~VL53L1_Modbus() = default;

VL53L1_Modbus::VL53L1_Modbus(VL53L1_Modbus&& other) noexcept = default;
VL53L1_Modbus& VL53L1_Modbus::operator=(VL53L1_Modbus&& other) noexcept = default;

const char* VL53L1_Modbus::init() {
    if (pimpl_->initialized) {
        return nullptr;
    }

    ESP_LOGI(TAG, "===== Starting VL53L1_Modbus Initialization =====");

    esp_task_wdt_reset();

    if (!pimpl_->configureModule()) {
        ESP_LOGE(TAG, "TOF400F module configuration failed");
        return "TOF400F module configuration failed";
    }

    esp_task_wdt_reset();

    if (!pimpl_->waitForI2CMode()) {
        ESP_LOGE(TAG, "Failed to switch to I2C mode");
        return "Failed to switch to I2C mode";
    }

    esp_task_wdt_reset();

    if (!pimpl_->initializeVL53L1()) {
        ESP_LOGE(TAG, "VL53L1 initialization failed");
        return "VL53L1 initialization failed";
    }

    esp_task_wdt_reset();

    if (pimpl_->config.enable_continuous) {
        if (!startContinuous()) {
            ESP_LOGE(TAG, "Failed to start continuous mode");
            return "Failed to start continuous mode";
        }
    }

    pimpl_->initialized = true;

    ESP_LOGI(TAG, "===== VL53L1_Modbus Initialization SUCCESS =====");
    return nullptr;
}

bool VL53L1_Modbus::isReady() const {
    return pimpl_->initialized && pimpl_->vl53l1 && pimpl_->vl53l1->isReady();
}

bool VL53L1_Modbus::readSingle(Measurement& result) {
    return readContinuous(result);
}

bool VL53L1_Modbus::startContinuous() {
    if (!isReady()) {
        return false;
    }

    return pimpl_->vl53l1->startContinuous();
}

bool VL53L1_Modbus::stopContinuous() {
    if (!isReady()) {
        return false;
    }

    return pimpl_->vl53l1->stopContinuous();
}

bool VL53L1_Modbus::readContinuous(Measurement& result) {
    if (!isReady()) {
        return false;
    }

    VL53L1::MeasurementResult vl53_result;
    if (!pimpl_->vl53l1->readContinuous(vl53_result)) {
        return false;
    }

    result.distance_mm = vl53_result.distance_mm;
    result.valid = vl53_result.valid;
    result.range_status = vl53_result.range_status;
    result.timestamp_us = vl53_result.timestamp_us;

    return true;
}

const char* VL53L1_Modbus::setRangingMode(RangingMode mode) {
    if (!isReady()) {
        return "Sensor not ready";
    }

    VL53L1::RangingMode vl53_mode = (mode == RangingMode::LONG_DISTANCE)
        ? VL53L1::RangingMode::LONG_DISTANCE
        : VL53L1::RangingMode::HIGH_PRECISION;

    const char* err = pimpl_->vl53l1->setRangingMode(vl53_mode);
    if (err == nullptr) {
        pimpl_->config.ranging_mode = mode;
    }

    return err;
}

VL53L1_Modbus::RangingMode VL53L1_Modbus::getRangingMode() const {
    return pimpl_->config.ranging_mode;
}

void VL53L1_Modbus::setTimeout(uint16_t timeout_ms) {
    pimpl_->config.timeout_ms = timeout_ms;
    if (pimpl_->vl53l1) {
        pimpl_->vl53l1->setTimeout(timeout_ms);
    }
}

uint16_t VL53L1_Modbus::getTimeout() const {
    return pimpl_->config.timeout_ms;
}

bool VL53L1_Modbus::setAddress(uint8_t new_addr) {
    if (!isReady()) {
        return false;
    }

    bool success = pimpl_->vl53l1->setAddress(new_addr);
    if (success) {
        pimpl_->config.i2c_address = new_addr;
    }

    return success;
}

uint8_t VL53L1_Modbus::getAddress() const {
    return pimpl_->config.i2c_address;
}

bool VL53L1_Modbus::probe() {
    if (!pimpl_->vl53l1) {
        return false;
    }

    return pimpl_->vl53l1->probe();
}

const char* VL53L1_Modbus::selfTest() {
    if (!isReady()) {
        return "Sensor not ready";
    }

    return pimpl_->vl53l1->selfTest();
}

bool VL53L1_Modbus::timeoutOccurred() {
    if (!pimpl_->vl53l1) {
        return false;
    }

    return pimpl_->vl53l1->timeoutOccurred();
}

bool VL53L1_Modbus::loadFactoryCalibration() {
    if (!pimpl_->modbus || !pimpl_->modbus->isReady()) {
        return false;
    }

    return pimpl_->sendModbusCommand(TOF400FReg::LOAD_CALIBRATION, 0x0001);
}

bool VL53L1_Modbus::setAutoOutput(bool enable, uint16_t interval_ms) {
    if (!pimpl_->modbus || !pimpl_->modbus->isReady()) {
        return false;
    }

    uint16_t value = enable ? interval_ms : 0x0000;
    return pimpl_->sendModbusCommand(TOF400FReg::AUTO_OUTPUT, value);
}

bool VL53L1_Modbus::rebootModule() {
    if (!pimpl_->modbus || !pimpl_->modbus->isReady()) {
        return false;
    }

    return pimpl_->sendModbusCommand(TOF400FReg::RESTORE_DEFAULT, 0x1000);
}

VL53L1_Modbus::Config vl53l1_modbus_default_config(
    i2c_port_t i2c_port,
    gpio_num_t sda_pin,
    gpio_num_t scl_pin,
    uart_port_t uart_port,
    gpio_num_t uart_tx_pin,
    gpio_num_t uart_rx_pin) {

    return VL53L1_Modbus::Config{
        .i2c_port = i2c_port,
        .sda_pin = sda_pin,
        .scl_pin = scl_pin,
        .i2c_address = 0x29,
        .i2c_freq_hz = 400000,
        .uart_port = uart_port,
        .uart_tx_pin = uart_tx_pin,
        .uart_rx_pin = uart_rx_pin,
        .uart_baud_rate = 115200,
        .modbus_slave_address = 0x01,
        .ranging_mode = VL53L1_Modbus::RangingMode::LONG_DISTANCE,
        .timeout_ms = 500,
        .enable_continuous = true
    };
}