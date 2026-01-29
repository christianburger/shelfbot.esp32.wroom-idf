#include "tof400f_modbus.hpp"
#include <sstream>
#include <iomanip>
#include "freertos/task.h"

const char* Tof400fModbus::TAG = "TOF400F_MODBUS";

Tof400fModbus::Tof400fModbus(const Config& config)
    : config_(config), initialized_(false) {

    // Set default register addresses if not configured
    if (config_.registers.device_address == 0) {
        config_.registers.device_address = 0x0002;
    }
    if (config_.registers.baud_rate == 0) {
        config_.registers.baud_rate = 0x0003;
    }
    if (config_.registers.range_mode == 0) {
        config_.registers.range_mode = 0x0004;
    }
    if (config_.registers.continuous_output == 0) {
        config_.registers.continuous_output = 0x0005;
    }
    if (config_.registers.disable_iic == 0) {
        config_.registers.disable_iic = 0x0009;
    }
    if (config_.registers.measurement_result == 0) {
        config_.registers.measurement_result = 0x0010;
    }

    // Set default MODBUS address if not configured
    if (config_.modbus_address == 0) {
        config_.modbus_address = 0x01;
    }
}

Tof400fModbus::~Tof400fModbus() {
    // modbus_ unique_ptr will automatically clean up
}

const char* Tof400fModbus::init() {
    if (initialized_) {
        return nullptr;
    }

    ESP_LOGI(TAG, "Initializing TOF400F MODBUS interface");
    ESP_LOGI(TAG, "  MODBUS address: 0x%02X", config_.modbus_address);
    ESP_LOGI(TAG, "  UART port: %d", config_.uart_config.uart_port);

    // Create and initialize MODBUS interface
    modbus_ = std::make_unique<DuartModbus>(config_.uart_config);
    const char* error = modbus_->init();
    if (error) {
        setError(std::string("MODBUS initialization failed: ") + error);
        return last_error_.c_str();
    }

    // Verify MODBUS communication
    ESP_LOGI(TAG, "Verifying MODBUS communication...");

    // Try to read device address register
    uint16_t device_addr = 0;
    if (!readAndLogRegister(config_.registers.device_address, device_addr, "device address")) {
        ESP_LOGW(TAG, "Failed to read device address, but continuing...");
    }

    // Try to read baud rate register
    uint16_t baud_rate = 0;
    if (!readAndLogRegister(config_.registers.baud_rate, baud_rate, "baud rate")) {
        ESP_LOGW(TAG, "Failed to read baud rate, but continuing...");
    }

    initialized_ = true;
    ESP_LOGI(TAG, "TOF400F MODBUS interface initialized successfully");
    return nullptr;
}

const char* Tof400fModbus::switchToI2CMode(bool verify) {
    if (!initialized_ || !modbus_->isReady()) {
        return "MODBUS not initialized";
    }

    ESP_LOGI(TAG, "Switching TOF400F to I2C mode via MODBUS");

    // First, read current value to log
    uint16_t current_value = 0;
    if (readAndLogRegister(config_.registers.disable_iic, current_value, "current I2C mode")) {
        ESP_LOGI(TAG, "Current I2C mode register value: 0x%04X", current_value);
    }

    // Write 1 to DISABLE_IIC register to enable I2C mode
    // Note: The register name is misleading. Writing 1 actually enables I2C mode.
    ESP_LOGI(TAG, "Writing 0x%04X to register 0x%04X (DISABLE_IIC)",
            ENABLE_I2C_VALUE, config_.registers.disable_iic);

    auto write_func = [this](uint16_t read_value) {
        // Verify that I2C mode is enabled (value should be 1)
        bool success = (read_value == ENABLE_I2C_VALUE);
        if (!success) {
            ESP_LOGE(TAG, "I2C mode verification failed: read 0x%04X, expected 0x%04X",
                    read_value, ENABLE_I2C_VALUE);
        }
        return success;
    };

    auto response = modbus_->writeSingleRegisterWithVerification(
        config_.modbus_address,
        config_.registers.disable_iic,
        ENABLE_I2C_VALUE,
        write_func,
        config_.uart_config.timeout_ms
    );

    if (!response.success) {
        std::stringstream ss;
        ss << "Failed to switch to I2C mode: ";
        ss << "ESP error: " << esp_err_to_name(response.esp_error);
        if (response.error_code != 0) {
            ss << ", MODBUS error: 0x" << std::hex << (int)response.error_code;
        }
        setError(ss.str());
        return last_error_.c_str();
    }

    ESP_LOGI(TAG, "TOF400F switched to I2C mode successfully");

    // Optional: Verify by trying to read the register again
    if (verify) {
        vTaskDelay(pdMS_TO_TICKS(200));  // Give module time to switch

        uint16_t verified_value = 0;
        if (readAndLogRegister(config_.registers.disable_iic, verified_value, "verify I2C mode")) {
            if (verified_value == ENABLE_I2C_VALUE) {
                ESP_LOGI(TAG, "I2C mode verified: register value = 0x%04X", verified_value);
            } else {
                ESP_LOGW(TAG, "I2C mode verification warning: unexpected value 0x%04X",
                        verified_value);
            }
        }
    }

    clearError();
    return nullptr;
}

const char* Tof400fModbus::setRangingMode(RangingMode mode, bool verify) {
    if (!initialized_ || !modbus_->isReady()) {
        return "MODBUS not initialized";
    }

    uint16_t mode_value = static_cast<uint16_t>(mode);
    const char* mode_name = (mode == RangingMode::HIGH_PRECISION) ?
                           "High Precision" : "Long Distance";

    ESP_LOGI(TAG, "Setting ranging mode to %s (value: 0x%04X)", mode_name, mode_value);

    // First, read current mode
    uint16_t current_mode = 0;
    if (readAndLogRegister(config_.registers.range_mode, current_mode, "current range mode")) {
        ESP_LOGI(TAG, "Current ranging mode: 0x%04X", current_mode);
    }

    // Write new mode
    auto write_func = [mode_value](uint16_t read_value) {
        bool success = (read_value == mode_value);
        if (!success) {
            ESP_LOGE(TAG, "Ranging mode verification failed: read 0x%04X, expected 0x%04X",
                    read_value, mode_value);
        }
        return success;
    };

    auto response = modbus_->writeSingleRegisterWithVerification(
        config_.modbus_address,
        config_.registers.range_mode,
        mode_value,
        write_func,
        config_.uart_config.timeout_ms
    );

    if (!response.success) {
        std::stringstream ss;
        ss << "Failed to set ranging mode: ";
        ss << "ESP error: " << esp_err_to_name(response.esp_error);
        if (response.error_code != 0) {
            ss << ", MODBUS error: 0x" << std::hex << (int)response.error_code;
        }
        setError(ss.str());
        return last_error_.c_str();
    }

    ESP_LOGI(TAG, "Ranging mode set to %s successfully", mode_name);

    // Optional additional verification
    if (verify) {
        vTaskDelay(pdMS_TO_TICKS(100));

        uint16_t verified_mode = 0;
        if (readAndLogRegister(config_.registers.range_mode, verified_mode, "verify range mode")) {
            if (verified_mode == mode_value) {
                ESP_LOGI(TAG, "Ranging mode verified: register value = 0x%04X", verified_mode);
            } else {
                ESP_LOGW(TAG, "Ranging mode verification warning: got 0x%04X, expected 0x%04X",
                        verified_mode, mode_value);
            }
        }
    }

    clearError();
    return nullptr;
}

const char* Tof400fModbus::getRangingMode(RangingMode& mode) {
    if (!initialized_ || !modbus_->isReady()) {
        return "MODBUS not initialized";
    }

    auto response = modbus_->readHoldingRegisters(
        config_.modbus_address,
        config_.registers.range_mode,
        1,
        config_.uart_config.timeout_ms
    );

    if (!response.success) {
        setError("Failed to read ranging mode");
        return last_error_.c_str();
    }

    if (response.data.size() >= 2) {
        uint16_t mode_value = (response.data[0] << 8) | response.data[1];
        mode = static_cast<RangingMode>(mode_value);
        clearError();
        return nullptr;
    }

    setError("Invalid response when reading ranging mode");
    return last_error_.c_str();
}

const char* Tof400fModbus::setDeviceAddress(uint8_t new_address, bool verify) {
    if (!initialized_ || !modbus_->isReady()) {
        return "MODBUS not initialized";
    }

    if (new_address < 1 || new_address > 247) {
        setError("Invalid MODBUS address (must be 1-247)");
        return last_error_.c_str();
    }

    ESP_LOGI(TAG, "Setting MODBUS address from 0x%02X to 0x%02X",
            config_.modbus_address, new_address);

    auto write_func = [new_address](uint16_t read_value) {
        bool success = (read_value == new_address);
        if (!success) {
            ESP_LOGE(TAG, "Address verification failed: read 0x%04X, expected 0x%04X",
                    read_value, new_address);
        }
        return success;
    };

    auto response = modbus_->writeSingleRegisterWithVerification(
        config_.modbus_address,
        config_.registers.device_address,
        new_address,
        write_func,
        config_.uart_config.timeout_ms
    );

    if (!response.success) {
        std::stringstream ss;
        ss << "Failed to set device address: ";
        ss << "ESP error: " << esp_err_to_name(response.esp_error);
        if (response.error_code != 0) {
            ss << ", MODBUS error: 0x" << std::hex << (int)response.error_code;
        }
        setError(ss.str());
        return last_error_.c_str();
    }

    // Update our configuration
    uint8_t old_address = config_.modbus_address;
    config_.modbus_address = new_address;

    ESP_LOGI(TAG, "MODBUS address changed from 0x%02X to 0x%02X",
            old_address, new_address);

    // Verify with new address if requested
    if (verify) {
        vTaskDelay(pdMS_TO_TICKS(200));

        // Try to read with new address - FIXED: removed unused variable
        if (modbus_->verifyRegisterValue(new_address, config_.registers.device_address,
                                        new_address, config_.uart_config.timeout_ms)) {
            ESP_LOGI(TAG, "New address verified successfully");
        } else {
            ESP_LOGW(TAG, "Could not verify new address, module may need power cycle");
        }
    }

    clearError();
    return nullptr;
}

bool Tof400fModbus::writeAndVerifyRegister(uint16_t reg, uint16_t value, const std::string& operation) {
    auto response = modbus_->writeSingleRegisterWithVerification(
        config_.modbus_address,
        reg,
        value,
        [value](uint16_t read_value) { return read_value == value; },
        config_.uart_config.timeout_ms
    );

    if (!response.success) {
        ESP_LOGE(TAG, "Failed to %s: ESP error %s", operation.c_str(),
                esp_err_to_name(response.esp_error));
        return false;
    }

    ESP_LOGI(TAG, "%s successful", operation.c_str());
    return true;
}

bool Tof400fModbus::readAndLogRegister(uint16_t reg, uint16_t& value, const std::string& operation) {
    auto response = modbus_->readHoldingRegisters(
        config_.modbus_address,
        reg,
        1,
        config_.uart_config.timeout_ms
    );

    if (!response.success) {
        ESP_LOGW(TAG, "Failed to read %s: %s", operation.c_str(),
                esp_err_to_name(response.esp_error));
        return false;
    }

    if (response.data.size() >= 2) {
        value = (response.data[0] << 8) | response.data[1];
        ESP_LOGI(TAG, "Read %s: 0x%04X", operation.c_str(), value);
        return true;
    }

    ESP_LOGW(TAG, "Invalid response when reading %s", operation.c_str());
    return false;
}

std::string Tof400fModbus::selfTest() {
    std::stringstream ss;

    if (!initialized_ || !modbus_->isReady()) {
        ss << "FAIL: MODBUS not initialized";
        return ss.str();
    }

    ss << "TOF400F MODBUS Self-Test Results:\n";
    ss << "================================\n";

    // Test 1: Read device address
    ss << "1. Reading device address register... ";
    uint16_t device_addr = 0;
    if (readAndLogRegister(config_.registers.device_address, device_addr, "device address")) {
        ss << "PASS (0x" << std::hex << std::setw(4) << std::setfill('0') << device_addr << ")\n";
    } else {
        ss << "FAIL\n";
    }

    // Test 2: Read baud rate
    ss << "2. Reading baud rate register... ";
    uint16_t baud_rate = 0;
    if (readAndLogRegister(config_.registers.baud_rate, baud_rate, "baud rate")) {
        ss << "PASS (0x" << std::hex << std::setw(4) << std::setfill('0') << baud_rate << ")\n";
    } else {
        ss << "FAIL\n";
    }

    // Test 3: Read ranging mode
    ss << "3. Reading ranging mode register... ";
    uint16_t range_mode = 0;
    if (readAndLogRegister(config_.registers.range_mode, range_mode, "range mode")) {
        ss << "PASS (0x" << std::hex << std::setw(4) << std::setfill('0') << range_mode << ")\n";
    } else {
        ss << "FAIL\n";
    }

    // Test 4: Test write/read cycle
    ss << "4. Testing write/read cycle... ";
    uint16_t test_value = 0x55AA;
    if (writeAndVerifyRegister(config_.registers.continuous_output, test_value, "test write")) {
        // Read back
        uint16_t read_value = 0;
        if (readAndLogRegister(config_.registers.continuous_output, read_value, "test read")) {
            if (read_value == test_value) {
                ss << "PASS\n";
            } else {
                ss << "FAIL (read 0x" << std::hex << read_value << ")\n";
            }
        } else {
            ss << "FAIL (read error)\n";
        }
    } else {
        ss << "FAIL (write error)\n";
    }

    // Test 5: Check I2C mode
    ss << "5. Checking I2C mode register... ";
    uint16_t i2c_mode = 0;
    if (readAndLogRegister(config_.registers.disable_iic, i2c_mode, "I2C mode")) {
        ss << "PASS (0x" << std::hex << std::setw(4) << std::setfill('0') << i2c_mode << ")\n";
        if (i2c_mode == ENABLE_I2C_VALUE) {
            ss << "   Note: Module is in I2C mode\n";
        } else {
            ss << "   Note: Module is in UART mode\n";
        }
    } else {
        ss << "FAIL\n";
    }

    ss << "\nSelf-test ";
    if (ss.str().find("FAIL") != std::string::npos) {
        ss << "completed with errors";
    } else {
        ss << "PASSED";
    }

    return ss.str();
}

std::string Tof400fModbus::getStatus() const {
    std::stringstream ss;

    ss << "TOF400F MODBUS Status:\n";
    ss << "  Initialized: " << (initialized_ ? "YES" : "NO") << "\n";
    ss << "  MODBUS ready: " << (modbus_ && modbus_->isReady() ? "YES" : "NO") << "\n";
    ss << "  MODBUS address: 0x" << std::hex << (int)config_.modbus_address << "\n";
    ss << "  UART port: " << config_.uart_config.uart_port << "\n";
    ss << "  Last error: " << (last_error_.empty() ? "None" : last_error_) << "\n";

    return ss.str();
}

void Tof400fModbus::setError(const std::string& error) {
    last_error_ = error;
    ESP_LOGE(TAG, "%s", error.c_str());
}

void Tof400fModbus::clearError() {
    last_error_.clear();
}
