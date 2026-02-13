#include "vl53l1_modbus.hpp"

static const char* TAG = "TofDriver_VL53L1_Modbus";

// ═══════════════════════════════════════════════════════════════
// Constructor / Destructor
// ═══════════════════════════════════════════════════════════════
VL53L1_Modbus_Driver::VL53L1_Modbus_Driver()
    : uart_port_(VL53L1_MODBUS_UART_PORT),
      uart_tx_pin_(VL53L1_MODBUS_TX_PIN),
      uart_rx_pin_(VL53L1_MODBUS_RX_PIN),
      baud_rate_(VL53L1_MODBUS_BAUD_RATE),
      modbus_slave_address_(VL53L1_MODBUS_SLAVE_ADDR),
      timeout_ms_(VL53L1_MODBUS_TIMEOUT_MS),
      ranging_mode_(VL53L1_MODBUS_RANGING_MODE),
      enable_continuous_(VL53L1_MODBUS_CONTINUOUS),
      modbus_(nullptr),
      initialized_(false),
      timeout_occurred_(false) {
}

VL53L1_Modbus_Driver::~VL53L1_Modbus_Driver() {
    if (modbus_) {
        delete modbus_;
        modbus_ = nullptr;
    }
}

// ═══════════════════════════════════════════════════════════════
// Helper: Log Modbus Response
// ═══════════════════════════════════════════════════════════════
void VL53L1_Modbus_Driver::logModbusResponse(const char* operation, const DuartModbus::ModbusResponse& response) {
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
    }
}

// ═══════════════════════════════════════════════════════════════
// Initialization Helpers
// ═══════════════════════════════════════════════════════════════
const char* VL53L1_Modbus_Driver::initModbus() {
    DuartModbus::Config modbus_config = {
        .uart_port = uart_port_,
        .tx_pin = uart_tx_pin_,
        .rx_pin = uart_rx_pin_,
        .baud_rate = baud_rate_,
        .parity = 0,          // No parity
        .stop_bits = 1,       // 1 stop bit
        .data_bits = 8,       // 8 data bits
        .timeout_ms = timeout_ms_,
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

const char* VL53L1_Modbus_Driver::testCommunication() {
    ESP_LOGI(TAG, "  Testing communication with slave 0x%02X...", modbus_slave_address_);

    // Test: Read special register (0x0001)
    ESP_LOGI(TAG, "  Reading SPECIAL register (0x0001)...");
    auto response = modbus_->readHoldingRegisters(
        modbus_slave_address_,
        REG_SPECIAL,
        1,
        timeout_ms_
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

    ESP_LOGI(TAG, "  Communication test PASSED");
    return nullptr;
}

const char* VL53L1_Modbus_Driver::readCurrentConfiguration() {
    ESP_LOGI(TAG, "  Reading current configuration from device...");

    // Read IIC disable setting (0x0009)
    ESP_LOGI(TAG, "  Reading DISABLE_IIC register (0x0009)...");
    auto response = modbus_->readHoldingRegisters(
        modbus_slave_address_,
        REG_DISABLE_IIC,
        1,
        timeout_ms_
    );

    logModbusResponse("Read DISABLE_IIC", response);

    if (response.success && response.data.size() >= 2) {
        uint16_t iic_state = (response.data[0] << 8) | response.data[1];
        ESP_LOGI(TAG, "  Current IIC state: 0x%04X (%s mode)",
                 iic_state, iic_state == 0x0000 ? "UART/Modbus" : "I2C");

        if (iic_state != 0x0000) {
            ESP_LOGW(TAG, "  Device is currently in I2C mode");
            ESP_LOGI(TAG, "  Switching device to UART/Modbus mode...");

            auto write_response = modbus_->writeSingleRegister(
                modbus_slave_address_,
                REG_DISABLE_IIC,
                0x0000,
                false,
                timeout_ms_
            );

            logModbusResponse("Write DISABLE_IIC (enable UART)", write_response);

            if (!write_response.success) {
                ESP_LOGE(TAG, "  FAILED: Could not switch to UART mode");
                return "Failed to switch to UART mode";
            }

            vTaskDelay(pdMS_TO_TICKS(100));

            // Verify the switch
            response = modbus_->readHoldingRegisters(
                modbus_slave_address_,
                REG_DISABLE_IIC,
                1,
                timeout_ms_
            );

            if (response.success && response.data.size() >= 2) {
                uint16_t new_state = (response.data[0] << 8) | response.data[1];
                if (new_state != 0x0000) {
                    ESP_LOGE(TAG, "  VERIFICATION FAILED: Still in I2C mode");
                    return "Device stuck in I2C mode";
                }
                ESP_LOGI(TAG, "  Mode switch successful - now in UART mode");
            }
        }
    }

    return nullptr;
}

const char* VL53L1_Modbus_Driver::configureRangingMode() {
    uint16_t desired_mode = ranging_mode_;
    const char* mode_name = (ranging_mode_ == 0) ?
                            "High Precision (30ms, 1.3m)" : "Long Distance (200ms, 4.0m)";

    ESP_LOGI(TAG, "  Setting ranging mode to: %s (0x%04X)", mode_name, desired_mode);

    // Read current mode
    auto read_response = modbus_->readHoldingRegisters(
        modbus_slave_address_,
        REG_RANGE_MODE,
        1,
        timeout_ms_
    );

    uint16_t current_mode = 0xFFFF;
    if (read_response.success && read_response.data.size() >= 2) {
        current_mode = (read_response.data[0] << 8) | read_response.data[1];
        if (current_mode == desired_mode) {
            ESP_LOGI(TAG, "  Mode already set correctly");
            return nullptr;
        }
    }

    // Write new mode
    auto write_response = modbus_->writeSingleRegister(
        modbus_slave_address_,
        REG_RANGE_MODE,
        desired_mode,
        false,
        timeout_ms_
    );

    logModbusResponse("Write RANGE_MODE", write_response);

    if (!write_response.success) {
        ESP_LOGE(TAG, "  FAILED: Could not write ranging mode");
        return "Failed to write ranging mode";
    }

    vTaskDelay(pdMS_TO_TICKS(50));

    // Verify
    read_response = modbus_->readHoldingRegisters(
        modbus_slave_address_,
        REG_RANGE_MODE,
        1,
        timeout_ms_
    );

    if (read_response.success && read_response.data.size() >= 2) {
        uint16_t verified_mode = (read_response.data[0] << 8) | read_response.data[1];
        if (verified_mode != desired_mode) {
            ESP_LOGE(TAG, "  VERIFICATION FAILED");
            return "Ranging mode verification failed";
        }
        ESP_LOGI(TAG, "  Verification PASSED");
    }

    return nullptr;
}

const char* VL53L1_Modbus_Driver::configureContinuousMode() {
    if (!enable_continuous_) {
        ESP_LOGI(TAG, "  Continuous mode disabled by configuration");
        return nullptr;
    }

    uint16_t output_interval = (ranging_mode_ == 0) ? 30 : 200;
    ESP_LOGI(TAG, "  Setting continuous output interval to: %d ms", output_interval);

    auto write_response = modbus_->writeSingleRegister(
        modbus_slave_address_,
        REG_CONTINUOUS_OUTPUT,
        output_interval,
        false,
        timeout_ms_
    );

    logModbusResponse("Write CONTINUOUS_OUTPUT", write_response);

    if (!write_response.success) {
        ESP_LOGE(TAG, "  FAILED: Could not write continuous output setting");
        return "Failed to configure continuous mode";
    }

    vTaskDelay(pdMS_TO_TICKS(50));

    // Verify
    auto read_response = modbus_->readHoldingRegisters(
        modbus_slave_address_,
        REG_CONTINUOUS_OUTPUT,
        1,
        timeout_ms_
    );

    if (read_response.success && read_response.data.size() >= 2) {
        uint16_t verified_interval = (read_response.data[0] << 8) | read_response.data[1];
        if (verified_interval != output_interval) {
            ESP_LOGE(TAG, "  VERIFICATION FAILED");
            return "Continuous mode verification failed";
        }
        ESP_LOGI(TAG, "  Verification PASSED");
    }

    return nullptr;
}

const char* VL53L1_Modbus_Driver::verifyConfiguration() {
    ESP_LOGI(TAG, "  Performing final configuration verification...");

    auto response = modbus_->readHoldingRegisters(
        modbus_slave_address_,
        REG_RANGE_MODE,
        1,
        timeout_ms_
    );

    if (!response.success) {
        return "Final verification failed";
    }

    uint16_t mode = (response.data[0] << 8) | response.data[1];
    if (mode != ranging_mode_) {
        ESP_LOGE(TAG, "  Final check: Ranging mode mismatch");
        return "Final ranging mode mismatch";
    }

    ESP_LOGI(TAG, "  Final verification: All settings correct");
    return nullptr;
}

// ═══════════════════════════════════════════════════════════════
// Public Interface Implementation
// ═══════════════════════════════════════════════════════════════
const char* VL53L1_Modbus_Driver::configure() {
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "VL53L1_Modbus ToF Driver Configuration");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "UART Port: %d", uart_port_);
    ESP_LOGI(TAG, "TX: GPIO%d, RX: GPIO%d", uart_tx_pin_, uart_rx_pin_);
    ESP_LOGI(TAG, "Baud Rate: %lu", (unsigned long)baud_rate_);
    ESP_LOGI(TAG, "Slave Address: 0x%02X", modbus_slave_address_);
    ESP_LOGI(TAG, "Timeout: %d ms", timeout_ms_);
    ESP_LOGI(TAG, "Ranging Mode: %s", ranging_mode_ == 0 ? "High Precision" : "Long Distance");
    ESP_LOGI(TAG, "Continuous: %s", enable_continuous_ ? "Enabled" : "Disabled");
    ESP_LOGI(TAG, "========================================");

    return nullptr;
}

const char* VL53L1_Modbus_Driver::init() {
    if (initialized_) {
        return nullptr;
    }

    ESP_LOGI(TAG, "Initializing Modbus UART communication...");
    const char* err = initModbus();
    if (err != nullptr) {
        return err;
    }

    initialized_ = true;
    return nullptr;
}

const char* VL53L1_Modbus_Driver::setup() {
    if (!initialized_) {
        return "Not initialized";
    }

    ESP_LOGI(TAG, "Setting up device configuration...");

    const char* err = testCommunication();
    if (err != nullptr) {
        return err;
    }

    err = readCurrentConfiguration();
    if (err != nullptr) {
        return err;
    }

    err = configureRangingMode();
    if (err != nullptr) {
        return err;
    }

    err = configureContinuousMode();
    if (err != nullptr) {
        return err;
    }

    ESP_LOGI(TAG, "Setup complete");
    return nullptr;
}

const char* VL53L1_Modbus_Driver::calibrate() {
    if (!initialized_) {
        return "Not initialized";
    }

    ESP_LOGI(TAG, "Calibration (Modbus devices typically pre-calibrated)");
    // TOF400F modules come pre-calibrated
    // If custom calibration needed, implement here

    return nullptr;
}

const char* VL53L1_Modbus_Driver::check() {
    if (!initialized_) {
        return "Not initialized";
    }

    ESP_LOGI(TAG, "Performing health check...");

    auto response = modbus_->readHoldingRegisters(
        modbus_slave_address_,
        REG_SPECIAL,
        1,
        timeout_ms_
    );

    if (!response.success) {
        ESP_LOGE(TAG, "Health check FAILED: Device not responding");
        return "Health check failed";
    }

    ESP_LOGI(TAG, "Health check PASSED");
    return nullptr;
}

bool VL53L1_Modbus_Driver::read_sensor(MeasurementResult& result) {
    if (!initialized_) {
        return false;
    }

    timeout_occurred_ = false;

    auto response = modbus_->readHoldingRegisters(
        modbus_slave_address_,
        REG_MEASUREMENT,
        1,
        timeout_ms_
    );

    if (!response.success) {
        timeout_occurred_ = (response.esp_error == ESP_ERR_TIMEOUT);
        return false;
    }

    if (response.data.size() < 2) {
        return false;
    }

    result.distance_mm = (response.data[0] << 8) | response.data[1];
    result.range_status = 0;
    result.valid = (result.distance_mm > 0 && result.distance_mm < 8190);
    result.timeout_occurred = false;
    result.timestamp_us = esp_timer_get_time();

    return true;
}

bool VL53L1_Modbus_Driver::isReady() const {
    return initialized_;
}

void VL53L1_Modbus_Driver::setTimeout(uint16_t timeout_ms) {
    timeout_ms_ = timeout_ms;
}

bool VL53L1_Modbus_Driver::timeoutOccurred() {
    bool occurred = timeout_occurred_;
    timeout_occurred_ = false;
    return occurred;
}
