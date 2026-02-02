// vl53l1_modbus.cpp - Pure Modbus implementation with extensive troubleshooting

#include "vl53l1_modbus.hpp"

const char* VL53L1_Modbus::TAG = "VL53L1_Modbus";

// Register addresses from TOF400F datasheet
constexpr uint16_t REG_SPECIAL = 0x0001;
constexpr uint16_t REG_DEVICE_ADDR = 0x0002;
constexpr uint16_t REG_BAUD_RATE = 0x0003;
constexpr uint16_t REG_RANGE_MODE = 0x0004;
constexpr uint16_t REG_CONTINUOUS_OUTPUT = 0x0005;
constexpr uint16_t REG_LOAD_CALIBRATION = 0x0006;
constexpr uint16_t REG_OFFSET_CORRECTION = 0x0007;
constexpr uint16_t REG_XTALK_CORRECTION = 0x0008;
constexpr uint16_t REG_DISABLE_IIC = 0x0009;
constexpr uint16_t REG_MEASUREMENT = 0x0010;
constexpr uint16_t REG_OFFSET_CALIBRATION = 0x0020;
constexpr uint16_t REG_XTALK_CALIBRATION = 0x0021;

VL53L1_Modbus::VL53L1_Modbus(const Config& config)
    : config_(config),
      modbus_(nullptr),
      initialized_(false),
      timeout_occurred_(false) {

    ESP_LOGI(TAG, "VL53L1_Modbus instance created");
    ESP_LOGI(TAG, "Configuration:");
    ESP_LOGI(TAG, "  UART: %d, TX: GPIO%d, RX: GPIO%d", config_.uart_port, config_.uart_tx_pin, config_.uart_rx_pin);
    ESP_LOGI(TAG, "  Slave address: 0x%02X", config_.modbus_slave_address);
    ESP_LOGI(TAG, "  Ranging mode: %s", config_.ranging_mode == RangingMode::HIGH_PRECISION ? "High Precision" : "Long Distance");
}

VL53L1_Modbus::~VL53L1_Modbus() {
    if (modbus_) {
        delete modbus_;
        modbus_ = nullptr;
    }
}

const char* VL53L1_Modbus::init() {
    ESP_LOGI(TAG, "===== Starting VL53L1_Modbus Initialization =====");

    // Step 1: Initialize Modbus/UART communication
    ESP_LOGI(TAG, "Step 1: Initializing Modbus UART communication...");
    const char* err = initModbus();
    if (err != nullptr) {
        ESP_LOGE(TAG, "FAILED at Step 1: %s", err);
        return err;
    }
    ESP_LOGI(TAG, "Step 1: SUCCESS - Modbus UART initialized");

    // Step 2: Test basic communication
    ESP_LOGI(TAG, "Step 2: Testing basic Modbus communication...");
    err = testCommunication();
    if (err != nullptr) {
        ESP_LOGE(TAG, "FAILED at Step 2: %s", err);
        return err;
    }
    ESP_LOGI(TAG, "Step 2: SUCCESS - Device is responding to Modbus commands");

    // Step 3: Read and verify current configuration
    ESP_LOGI(TAG, "Step 3: Reading current device configuration...");
    err = readCurrentConfiguration();
    if (err != nullptr) {
        ESP_LOGE(TAG, "FAILED at Step 3: %s", err);
        return err;
    }
    ESP_LOGI(TAG, "Step 3: SUCCESS - Current configuration read");

    // Step 4: Configure ranging mode
    ESP_LOGI(TAG, "Step 4: Configuring ranging mode...");
    err = configureRangingMode();
    if (err != nullptr) {
        ESP_LOGE(TAG, "FAILED at Step 4: %s", err);
        return err;
    }
    ESP_LOGI(TAG, "Step 4: SUCCESS - Ranging mode configured");

    // Step 5: Configure continuous measurement
    ESP_LOGI(TAG, "Step 5: Configuring continuous measurement...");
    err = configureContinuousMode();
    if (err != nullptr) {
        ESP_LOGE(TAG, "FAILED at Step 5: %s", err);
        return err;
    }
    ESP_LOGI(TAG, "Step 5: SUCCESS - Continuous mode configured");

    // Step 6: Verify final configuration
    ESP_LOGI(TAG, "Step 6: Verifying final configuration...");
    err = verifyConfiguration();
    if (err != nullptr) {
        ESP_LOGE(TAG, "FAILED at Step 6: %s", err);
        return err;
    }
    ESP_LOGI(TAG, "Step 6: SUCCESS - Configuration verified");

    initialized_ = true;
    ESP_LOGI(TAG, "===== VL53L1_Modbus Initialization Complete =====");

    return nullptr;
}

const char* VL53L1_Modbus::initModbus() {
    // Configure Modbus with TOF400F defaults from datasheet
    DuartModbus::Config modbus_config = {
        .uart_port = config_.uart_port,
        .tx_pin = config_.uart_tx_pin,
        .rx_pin = config_.uart_rx_pin,
        .baud_rate = 115200,  // TOF400F default from datasheet
        .parity = 0,          // No parity
        .stop_bits = 1,       // 1 stop bit
        .data_bits = 8,       // 8 data bits
        .timeout_ms = config_.timeout_ms,
        .max_retries = 3
    };

    ESP_LOGI(TAG, "  Creating Modbus instance with:");
    ESP_LOGI(TAG, "    Baud rate: %lu", (unsigned long)modbus_config.baud_rate);
    ESP_LOGI(TAG, "    Data bits: %d, Parity: %d, Stop bits: %d",
             modbus_config.data_bits, modbus_config.parity, modbus_config.stop_bits);
    ESP_LOGI(TAG, "    Timeout: %lu ms, Retries: %d",
             (unsigned long)modbus_config.timeout_ms, modbus_config.max_retries);

    modbus_ = new DuartModbus(modbus_config);
    const char* err = modbus_->init();
    if (err != nullptr) {
        ESP_LOGE(TAG, "  Modbus initialization failed: %s", err);
        return "Modbus init failed";
    }

    ESP_LOGI(TAG, "  Modbus driver initialized successfully");
    return nullptr;
}

const char* VL53L1_Modbus::testCommunication() {
    ESP_LOGI(TAG, "  Testing communication with slave 0x%02X...", config_.modbus_slave_address);

    // Test 1: Read special register (0x0001)
    ESP_LOGI(TAG, "  Test 1: Reading SPECIAL register (0x0001)...");
    auto response = modbus_->readHoldingRegisters(
        config_.modbus_slave_address,
        REG_SPECIAL,
        1,
        config_.timeout_ms
    );

    logModbusResponse("Read SPECIAL", response);

    if (!response.success) {
        ESP_LOGE(TAG, "  FAILED: Device not responding to read commands");
        ESP_LOGE(TAG, "  Troubleshooting hints:");
        ESP_LOGE(TAG, "    - Check TX/RX wiring (TX->RX, RX->TX)");
        ESP_LOGE(TAG, "    - Verify device has power (3.3V-5V)");
        ESP_LOGE(TAG, "    - Check slave address (default is 0x01)");
        ESP_LOGE(TAG, "    - Verify baud rate (default is 115200)");
        return "Device not responding";
    }

    // Test 2: Read device address register (0x0002)
    ESP_LOGI(TAG, "  Test 2: Reading DEVICE_ADDR register (0x0002)...");
    response = modbus_->readHoldingRegisters(
        config_.modbus_slave_address,
        REG_DEVICE_ADDR,
        1,
        config_.timeout_ms
    );

    logModbusResponse("Read DEVICE_ADDR", response);

    if (response.success && response.data.size() >= 2) {
        uint16_t device_addr = (response.data[0] << 8) | response.data[1];
        ESP_LOGI(TAG, "  Current device address: 0x%04X", device_addr);

        if (device_addr != config_.modbus_slave_address && device_addr != 0x0000) {
            ESP_LOGW(TAG, "  WARNING: Device address (0x%04X) doesn't match expected (0x%02X)",
                     device_addr, config_.modbus_slave_address);
        }
    }

    ESP_LOGI(TAG, "  Communication test PASSED");
    return nullptr;
}

const char* VL53L1_Modbus::readCurrentConfiguration() {
    ESP_LOGI(TAG, "  Reading current configuration from device...");

    // Read ranging mode (0x0004)
    ESP_LOGI(TAG, "  Reading RANGE_MODE register (0x0004)...");
    auto response = modbus_->readHoldingRegisters(
        config_.modbus_slave_address,
        REG_RANGE_MODE,
        1,
        config_.timeout_ms
    );

    logModbusResponse("Read RANGE_MODE", response);

    if (response.success && response.data.size() >= 2) {
        uint16_t mode = (response.data[0] << 8) | response.data[1];
        ESP_LOGI(TAG, "  Current ranging mode: 0x%04X (%s)",
                 mode, mode == 0x0000 ? "High Precision (30ms, 1.3m)" : "Long Distance (200ms, 4.0m)");
    }

    // Read continuous output setting (0x0005)
    ESP_LOGI(TAG, "  Reading CONTINUOUS_OUTPUT register (0x0005)...");
    response = modbus_->readHoldingRegisters(
        config_.modbus_slave_address,
        REG_CONTINUOUS_OUTPUT,
        1,
        config_.timeout_ms
    );

    logModbusResponse("Read CONTINUOUS_OUTPUT", response);

    if (response.success && response.data.size() >= 2) {
        uint16_t output = (response.data[0] << 8) | response.data[1];
        ESP_LOGI(TAG, "  Current continuous output: 0x%04X (%s)",
                 output, output == 0x0000 ? "Disabled" : "Enabled");
    }

    // Read IIC disable setting (0x0009)
    ESP_LOGI(TAG, "  Reading DISABLE_IIC register (0x0009)...");
    response = modbus_->readHoldingRegisters(
        config_.modbus_slave_address,
        REG_DISABLE_IIC,
        1,
        config_.timeout_ms
    );

    logModbusResponse("Read DISABLE_IIC", response);

    if (response.success && response.data.size() >= 2) {
        uint16_t iic_state = (response.data[0] << 8) | response.data[1];
        ESP_LOGI(TAG, "  Current IIC state: 0x%04X (%s mode)",
                 iic_state, iic_state == 0x0000 ? "UART/Modbus" : "I2C");

        if (iic_state != 0x0000) {
            ESP_LOGW(TAG, "  Device is currently in I2C mode");
            ESP_LOGI(TAG, "  Switching device back to UART/Modbus mode...");

            // Write 0x0000 to register 0x0009 to enable UART mode
            auto write_response = modbus_->writeSingleRegister(
                config_.modbus_slave_address,
                REG_DISABLE_IIC,
                0x0000,  // 0 = UART/Modbus mode
                false,
                config_.timeout_ms
            );

            logModbusResponse("Write DISABLE_IIC (enable UART)", write_response);

            if (!write_response.success) {
                ESP_LOGE(TAG, "  FAILED: Could not switch to UART mode");
                return "Failed to switch to UART mode";
            }

            // Small delay for mode switch
            vTaskDelay(pdMS_TO_TICKS(100));

            // Verify the switch
            ESP_LOGI(TAG, "  Verifying mode switch...");
            response = modbus_->readHoldingRegisters(
                config_.modbus_slave_address,
                REG_DISABLE_IIC,
                1,
                config_.timeout_ms
            );

            logModbusResponse("Read DISABLE_IIC (after switch)", response);

            if (response.success && response.data.size() >= 2) {
                uint16_t new_state = (response.data[0] << 8) | response.data[1];
                if (new_state != 0x0000) {
                    ESP_LOGE(TAG, "  VERIFICATION FAILED: Still in I2C mode (0x%04X)", new_state);
                    ESP_LOGE(TAG, "  Power cycle the device to reset");
                    return "Device stuck in I2C mode";
                }
                ESP_LOGI(TAG, "  Mode switch successful - now in UART mode");
            } else {
                ESP_LOGE(TAG, "  Could not verify mode switch");
                return "Mode verification failed";
            }
        }
    }

    return nullptr;
}

const char* VL53L1_Modbus::configureRangingMode() {
    uint16_t desired_mode = (config_.ranging_mode == RangingMode::HIGH_PRECISION) ? 0x0000 : 0x0001;
    const char* mode_name = (config_.ranging_mode == RangingMode::HIGH_PRECISION) ?
                            "High Precision (30ms, 1.3m)" : "Long Distance (200ms, 4.0m)";

    ESP_LOGI(TAG, "  Setting ranging mode to: %s (0x%04X)", mode_name, desired_mode);

    // Read current mode first
    ESP_LOGI(TAG, "  Reading current mode before write...");
    auto read_response = modbus_->readHoldingRegisters(
        config_.modbus_slave_address,
        REG_RANGE_MODE,
        1,
        config_.timeout_ms
    );

    logModbusResponse("Read RANGE_MODE (before)", read_response);

    uint16_t current_mode = 0xFFFF;
    if (read_response.success && read_response.data.size() >= 2) {
        current_mode = (read_response.data[0] << 8) | read_response.data[1];
        ESP_LOGI(TAG, "  Current mode: 0x%04X", current_mode);

        if (current_mode == desired_mode) {
            ESP_LOGI(TAG, "  Mode already set correctly, skipping write");
            return nullptr;
        }
    }

    // Write new mode
    ESP_LOGI(TAG, "  Writing new mode: 0x%04X...", desired_mode);
    auto write_response = modbus_->writeSingleRegister(
        config_.modbus_slave_address,
        REG_RANGE_MODE,
        desired_mode,
        false,  // Don't use driver's built-in verify, we'll do it manually
        config_.timeout_ms
    );

    logModbusResponse("Write RANGE_MODE", write_response);

    if (!write_response.success) {
        ESP_LOGE(TAG, "  FAILED: Could not write ranging mode");
        return "Failed to write ranging mode";
    }

    // Small delay for register to update
    vTaskDelay(pdMS_TO_TICKS(50));

    // Verify the write
    ESP_LOGI(TAG, "  Verifying write by reading back...");
    read_response = modbus_->readHoldingRegisters(
        config_.modbus_slave_address,
        REG_RANGE_MODE,
        1,
        config_.timeout_ms
    );

    logModbusResponse("Read RANGE_MODE (after)", read_response);

    if (read_response.success && read_response.data.size() >= 2) {
        uint16_t verified_mode = (read_response.data[0] << 8) | read_response.data[1];
        ESP_LOGI(TAG, "  Verified mode: 0x%04X", verified_mode);

        if (verified_mode != desired_mode) {
            ESP_LOGE(TAG, "  VERIFICATION FAILED: Read 0x%04X but expected 0x%04X",
                     verified_mode, desired_mode);
            return "Ranging mode verification failed";
        }

        ESP_LOGI(TAG, "  Verification PASSED");
    } else {
        ESP_LOGE(TAG, "  VERIFICATION FAILED: Could not read back");
        return "Could not verify ranging mode";
    }

    return nullptr;
}

const char* VL53L1_Modbus::configureContinuousMode() {
    // For continuous mode, we want the device to continuously measure
    // Set output interval based on ranging mode
    uint16_t output_interval;

    if (config_.ranging_mode == RangingMode::HIGH_PRECISION) {
        output_interval = 30;  // 30ms for high precision mode
    } else {
        output_interval = 200; // 200ms for long distance mode
    }

    ESP_LOGI(TAG, "  Setting continuous output interval to: %d ms", output_interval);

    // Read current setting first
    ESP_LOGI(TAG, "  Reading current continuous output setting...");
    auto read_response = modbus_->readHoldingRegisters(
        config_.modbus_slave_address,
        REG_CONTINUOUS_OUTPUT,
        1,
        config_.timeout_ms
    );

    logModbusResponse("Read CONTINUOUS_OUTPUT (before)", read_response);

    // Write new setting
    ESP_LOGI(TAG, "  Writing continuous output interval: %d ms...", output_interval);
    auto write_response = modbus_->writeSingleRegister(
        config_.modbus_slave_address,
        REG_CONTINUOUS_OUTPUT,
        output_interval,
        false,
        config_.timeout_ms
    );

    logModbusResponse("Write CONTINUOUS_OUTPUT", write_response);

    if (!write_response.success) {
        ESP_LOGE(TAG, "  FAILED: Could not write continuous output setting");
        return "Failed to configure continuous mode";
    }

    // Small delay
    vTaskDelay(pdMS_TO_TICKS(50));

    // Verify the write
    ESP_LOGI(TAG, "  Verifying write by reading back...");
    read_response = modbus_->readHoldingRegisters(
        config_.modbus_slave_address,
        REG_CONTINUOUS_OUTPUT,
        1,
        config_.timeout_ms
    );

    logModbusResponse("Read CONTINUOUS_OUTPUT (after)", read_response);

    if (read_response.success && read_response.data.size() >= 2) {
        uint16_t verified_interval = (read_response.data[0] << 8) | read_response.data[1];
        ESP_LOGI(TAG, "  Verified interval: %d ms", verified_interval);

        if (verified_interval != output_interval) {
            ESP_LOGE(TAG, "  VERIFICATION FAILED: Read %d but expected %d",
                     verified_interval, output_interval);
            return "Continuous mode verification failed";
        }

        ESP_LOGI(TAG, "  Verification PASSED");
    } else {
        ESP_LOGE(TAG, "  VERIFICATION FAILED: Could not read back");
        return "Could not verify continuous mode";
    }

    return nullptr;
}

const char* VL53L1_Modbus::verifyConfiguration() {
    ESP_LOGI(TAG, "  Performing final configuration verification...");

    // Verify ranging mode
    auto response = modbus_->readHoldingRegisters(
        config_.modbus_slave_address,
        REG_RANGE_MODE,
        1,
        config_.timeout_ms
    );

    if (!response.success) {
        return "Final verification failed - cannot read ranging mode";
    }

    uint16_t mode = (response.data[0] << 8) | response.data[1];
    uint16_t expected_mode = (config_.ranging_mode == RangingMode::HIGH_PRECISION) ? 0x0000 : 0x0001;

    if (mode != expected_mode) {
        ESP_LOGE(TAG, "  Final check: Ranging mode mismatch (got 0x%04X, expected 0x%04X)",
                 mode, expected_mode);
        return "Final ranging mode mismatch";
    }

    ESP_LOGI(TAG, "  Final verification: All settings correct");
    return nullptr;
}

void VL53L1_Modbus::logModbusResponse(const char* operation, const DuartModbus::ModbusResponse& response) {
    ESP_LOGI(TAG, "    %s:", operation);
    ESP_LOGI(TAG, "      Success: %s", response.success ? "YES" : "NO");
    ESP_LOGI(TAG, "      ESP Error: %s", esp_err_to_name(response.esp_error));

    if (response.success) {
        ESP_LOGI(TAG, "      Slave Address: 0x%02X", response.slave_address);
        ESP_LOGI(TAG, "      Function Code: 0x%02X", response.function_code);
        ESP_LOGI(TAG, "      Data bytes: %zu", response.data.size());

        if (!response.data.empty()) {
            char hex_str[256] = {0};
            size_t offset = 0;
            for (size_t i = 0; i < response.data.size() && offset < sizeof(hex_str) - 4; i++) {
                offset += snprintf(hex_str + offset, sizeof(hex_str) - offset, "%02X ", response.data[i]);
            }
            ESP_LOGI(TAG, "      Data: %s", hex_str);
        }
    } else {
        ESP_LOGE(TAG, "      Error Code: 0x%02X", response.error_code);
        ESP_LOGE(TAG, "      ESP Error: %s", esp_err_to_name(response.esp_error));
    }
}

bool VL53L1_Modbus::startContinuous() {
    if (!initialized_) {
        ESP_LOGE(TAG, "Cannot start continuous - not initialized");
        return false;
    }

    // Continuous mode is already configured in init()
    ESP_LOGI(TAG, "Continuous measurement already active");
    return true;
}

bool VL53L1_Modbus::stopContinuous() {
    if (!initialized_) {
        return false;
    }

    ESP_LOGI(TAG, "Stopping continuous measurement...");

    auto response = modbus_->writeSingleRegister(
        config_.modbus_slave_address,
        REG_CONTINUOUS_OUTPUT,
        0x0000,  // Disable continuous output
        false,
        config_.timeout_ms
    );

    logModbusResponse("Stop Continuous", response);

    return response.success;
}

bool VL53L1_Modbus::readContinuous(Measurement& result) {
    if (!initialized_) {
        ESP_LOGE(TAG, "Cannot read - not initialized");
        return false;
    }

    timeout_occurred_ = false;

    // Read measurement result from register 0x0010
    auto response = modbus_->readHoldingRegisters(
        config_.modbus_slave_address,
        REG_MEASUREMENT,
        1,
        config_.timeout_ms
    );

    if (!response.success) {
        ESP_LOGD(TAG, "Failed to read measurement");
        timeout_occurred_ = (response.esp_error == ESP_ERR_TIMEOUT);
        return false;
    }

    if (response.data.size() < 2) {
        ESP_LOGW(TAG, "Insufficient data in response");
        return false;
    }

    // Parse distance value (in mm)
    result.distance_mm = (response.data[0] << 8) | response.data[1];
    result.range_status = 0;  // Modbus interface doesn't provide detailed status
    result.valid = (result.distance_mm > 0 && result.distance_mm < 8190);  // 8190 = no target
    result.timestamp_us = esp_timer_get_time();

    ESP_LOGD(TAG, "Read distance: %d mm, valid: %s",
             result.distance_mm, result.valid ? "yes" : "no");

    return true;
}

bool VL53L1_Modbus::probe() {
    if (!modbus_) {
        return false;
    }

    // Try to read any register to check if device responds
    auto response = modbus_->readHoldingRegisters(
        config_.modbus_slave_address,
        REG_SPECIAL,
        1,
        config_.timeout_ms
    );

    return response.success;
}

const char* VL53L1_Modbus::setRangingMode(RangingMode mode) {
    if (!initialized_) {
        return "Not initialized";
    }

    config_.ranging_mode = mode;
    return configureRangingMode();
}

void VL53L1_Modbus::setTimeout(uint16_t timeout_ms) {
    config_.timeout_ms = timeout_ms;
}

const char* VL53L1_Modbus::selfTest() {
    if (!initialized_) {
        return "Not initialized";
    }

    ESP_LOGI(TAG, "Running self-test...");

    // Test 1: Read configuration registers
    auto response = modbus_->readHoldingRegisters(
        config_.modbus_slave_address,
        REG_RANGE_MODE,
        1,
        config_.timeout_ms
    );

    if (!response.success) {
        return "Self-test failed - cannot communicate";
    }

    ESP_LOGI(TAG, "Self-test PASSED");
    return nullptr;
}

VL53L1_Modbus::Config vl53l1_modbus_default_config() {
  VL53L1_Modbus::Config config;
  config.uart_port = VL53L1_MODBUS_UART_PORT;
  config.uart_tx_pin = VL53L1_MODBUS_TX_PIN;
  config.uart_rx_pin = VL53L1_MODBUS_RX_PIN;
  config.modbus_slave_address = VL53L1_MODBUS_SLAVE_ADDR;
  config.ranging_mode = VL53L1_Modbus::RangingMode::LONG_DISTANCE;
  config.timeout_ms = 500;
  config.enable_continuous = true;
  return config;
}