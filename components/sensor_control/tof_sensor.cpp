#include "tof_sensor.hpp"
#include "vl53l1_modbus.hpp"
#include <memory>
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/uart.h"

static const char* TAG = "TofSensor";

/**
 * @brief Concrete implementation for VL53L1-based ToF sensors using Modbus/I2C
 *
 * This class implements the TofSensor interface for VL53L1 sensors that
 * are accessed through Modbus (UART) commands for configuration and then
 * I2C for high-speed data reading.
 */
class VL53L1_ModbusSensor : public TofSensor {
private:
    std::unique_ptr<VL53L1_Modbus> driver_;  ///< Underlying driver instance
    Config config_;                          ///< Sensor configuration
    Mode current_mode_;                      ///< Current operation mode
    bool continuous_mode_;                   ///< Continuous mode state
    bool initialized_;                       ///< Initialization state

    /**
     * @brief Convert TofSensor mode to VL53L1_Modbus mode
     * @param mode TofSensor mode
     * @return Corresponding VL53L1_Modbus mode
     */
    VL53L1_Modbus::RangingMode to_driver_mode(Mode mode) const {
        return (mode == Mode::LONG_DISTANCE)
            ? VL53L1_Modbus::RangingMode::LONG_DISTANCE
            : VL53L1_Modbus::RangingMode::HIGH_PRECISION;
    }

    /**
     * @brief Convert VL53L1_Modbus mode to TofSensor mode
     * @param mode VL53L1_Modbus mode
     * @return Corresponding TofSensor mode
     */
    Mode from_driver_mode(VL53L1_Modbus::RangingMode mode) const {
        return (mode == VL53L1_Modbus::RangingMode::LONG_DISTANCE)
            ? Mode::LONG_DISTANCE
            : Mode::HIGH_PRECISION;
    }

    /**
     * @brief Create driver configuration from interface configuration
     * @return VL53L1_Modbus configuration
     */
    VL53L1_Modbus::Config create_driver_config() const {
        VL53L1_Modbus::Config driver_config = vl53l1_modbus_default_config(
            static_cast<i2c_port_t>(config_.pins.i2c_port),
            static_cast<gpio_num_t>(config_.pins.sda_pin),
            static_cast<gpio_num_t>(config_.pins.scl_pin),
            static_cast<uart_port_t>(config_.pins.uart_port),
            static_cast<gpio_num_t>(config_.pins.uart_tx_pin),
            static_cast<gpio_num_t>(config_.pins.uart_rx_pin)
        );

        // Override with user configuration
        driver_config.modbus_slave_address = config_.device_address;
        driver_config.ranging_mode = to_driver_mode(config_.mode);
        driver_config.timeout_ms = config_.timeout_ms;
        driver_config.enable_continuous = config_.enable_continuous;

        return driver_config;
    }

public:
    /**
     * @brief Construct a new VL53L1 Modbus Sensor
     * @param config Sensor configuration
     */
    explicit VL53L1_ModbusSensor(const Config& config)
        : config_(config),
          current_mode_(config.mode),
          continuous_mode_(false),
          initialized_(false) {

        ESP_LOGD(TAG, "Creating VL53L1_ModbusSensor");
    }

    /**
     * @brief Destroy the sensor instance
     */
    ~VL53L1_ModbusSensor() override {
        stop_continuous();
    }

    // ===== Core Interface Implementation =====

    esp_err_t initialize() override {
        if (initialized_) {
            ESP_LOGW(TAG, "Sensor already initialized");
            return ESP_OK;
        }

        ESP_LOGI(TAG, "Initializing VL53L1_Modbus sensor...");
        ESP_LOGI(TAG, "  Mode: %s",
                 (config_.mode == Mode::LONG_DISTANCE) ? "LONG_DISTANCE" : "HIGH_PRECISION");
        ESP_LOGI(TAG, "  Address: 0x%02X", config_.device_address);
        ESP_LOGI(TAG, "  Timeout: %d ms", config_.timeout_ms);

        // Create and initialize driver
        VL53L1_Modbus::Config driver_config = create_driver_config();
        driver_ = std::make_unique<VL53L1_Modbus>(driver_config);

        const char* err = driver_->init();
        if (err != nullptr) {
            ESP_LOGE(TAG, "Driver initialization failed: %s", err);
            driver_.reset();
            return ESP_FAIL;
        }

        // Start continuous mode if configured
        if (config_.enable_continuous) {
            esp_err_t err = start_continuous();
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "Failed to start continuous mode: %s", esp_err_to_name(err));
                driver_.reset();
                return err;
            }
        }

        initialized_ = true;
        ESP_LOGI(TAG, "VL53L1_Modbus sensor initialized successfully");
        return ESP_OK;
    }

    bool is_ready() const override {
        return initialized_ && driver_ && driver_->isReady();
    }

    esp_err_t read_measurement(Measurement& result) override {
        if (!is_ready()) {
            ESP_LOGE(TAG, "Sensor not ready");
            return ESP_ERR_INVALID_STATE;
        }

        VL53L1_Modbus::Measurement driver_result;
        if (!driver_->readContinuous(driver_result)) {
            ESP_LOGW(TAG, "Failed to read measurement");
            return ESP_ERR_INVALID_RESPONSE;
        }

        // Convert driver result to interface result
        result.distance_mm = driver_result.distance_mm;
        result.valid = driver_result.valid;
        result.status = driver_result.range_status;
        result.timestamp_us = driver_result.timestamp_us;
        result.timeout_occurred = driver_->timeoutOccurred();

        return ESP_OK;
    }

    // ===== Configuration Interface Implementation =====

    esp_err_t set_mode(Mode mode) override {
        if (!is_ready()) {
            ESP_LOGE(TAG, "Sensor not ready");
            return ESP_ERR_INVALID_STATE;
        }

        if (mode == current_mode_) {
            return ESP_OK;  // No change needed
        }

        VL53L1_Modbus::RangingMode driver_mode = to_driver_mode(mode);
        const char* err = driver_->setRangingMode(driver_mode);
        if (err != nullptr) {
            ESP_LOGE(TAG, "Failed to set ranging mode: %s", err);
            return ESP_FAIL;
        }

        current_mode_ = mode;
        config_.mode = mode;
        ESP_LOGI(TAG, "Sensor mode changed to: %s",
                 (mode == Mode::LONG_DISTANCE) ? "LONG_DISTANCE" : "HIGH_PRECISION");

        return ESP_OK;
    }

    Mode get_mode() const override {
        return current_mode_;
    }

    esp_err_t set_timeout(uint16_t timeout_ms) override {
        if (!driver_) {
            return ESP_ERR_INVALID_STATE;
        }

        driver_->setTimeout(timeout_ms);
        config_.timeout_ms = timeout_ms;
        ESP_LOGD(TAG, "Timeout set to %d ms", timeout_ms);

        return ESP_OK;
    }

    uint16_t get_timeout() const override {
        return config_.timeout_ms;
    }

    // ===== Continuous Mode Implementation =====

    esp_err_t start_continuous() override {
        if (!is_ready()) {
            return ESP_ERR_INVALID_STATE;
        }

        if (continuous_mode_) {
            return ESP_OK;  // Already in continuous mode
        }

        if (!driver_->startContinuous()) {
            ESP_LOGE(TAG, "Failed to start continuous mode");
            return ESP_FAIL;
        }

        continuous_mode_ = true;
        ESP_LOGI(TAG, "Continuous measurement mode started");

        return ESP_OK;
    }

    esp_err_t stop_continuous() override {
        if (!continuous_mode_) {
            return ESP_OK;  // Already stopped
        }

        if (!driver_->stopContinuous()) {
            ESP_LOGE(TAG, "Failed to stop continuous mode");
            return ESP_FAIL;
        }

        continuous_mode_ = false;
        ESP_LOGI(TAG, "Continuous measurement mode stopped");

        return ESP_OK;
    }

    bool is_continuous() const override {
        return continuous_mode_;
    }

    // ===== Diagnostic Interface Implementation =====

    esp_err_t self_test() override {
        if (!is_ready()) {
            return ESP_ERR_INVALID_STATE;
        }

        const char* err = driver_->selfTest();
        if (err != nullptr) {
            ESP_LOGE(TAG, "Self-test failed: %s", err);
            return ESP_FAIL;
        }

        ESP_LOGI(TAG, "Self-test passed");
        return ESP_OK;
    }

    bool probe() override {
        if (!driver_) {
            return false;
        }

        return driver_->probe();
    }

    bool timeout_occurred() override {
        if (!driver_) {
            return false;
        }

        return driver_->timeoutOccurred();
    }
};

// ===== Factory Function Implementation =====
std::unique_ptr<TofSensor> TofSensor::create(const Config& config) {
    return std::make_unique<VL53L1_ModbusSensor>(config);
}