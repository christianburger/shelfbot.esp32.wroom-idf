#include "tof_sensor.hpp"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/task.h"
#include <cinttypes>
#include <cstring>

static const char* TAG = "ToF_Sensor";

// Known I2C devices database for identification
const ToFSensorArray::DeviceInfo ToFSensorArray::KNOWN_DEVICES[] = {
    {0x29, "VL53L0X / VL53L1X ToF Sensor"},
    {0x30, "VL53L1X ToF Sensor (alt)"},
    {0x48, "ADS1115 ADC"},
    {0x68, "MPU6050 / DS1307 RTC"},
    {0x76, "BMP280 / BME280"},
    {0x77, "BMP180 / BMP280 (alt)"},
    {0x3C, "SSD1306 OLED (128x64)"},
    {0x3D, "SSD1306 OLED (alt)"},
    {0x50, "AT24C32 EEPROM"},
    {0x57, "AT24C32 EEPROM (alt)"},
    {0x20, "PCF8574 I/O Expander"},
    {0x27, "PCF8574 I/O Expander (alt)"},
};

const size_t ToFSensorArray::NUM_KNOWN_DEVICES = sizeof(KNOWN_DEVICES) / sizeof(KNOWN_DEVICES[0]);

// ============================================================================
// ToFSensorArray Implementation
// ============================================================================

ToFSensorArray::ToFSensorArray(uint8_t num_sensors)
    : num_sensors_(num_sensors) {
    vl53l0x_sensors_.resize(num_sensors);
    vl53l1_sensors_.resize(num_sensors);
    detected_types_.resize(num_sensors, DetectedSensorType::NONE);
    
    ESP_LOGI(TAG, "ToFSensorArray created with %d sensor slots", num_sensors);
}

bool ToFSensorArray::add_sensor(uint8_t index, const ToFSensorConfig& cfg) {
    if (index >= num_sensors_) {
        ESP_LOGE(TAG, "Sensor index %d out of bounds (max: %d)", index, num_sensors_ - 1);
        return false;
    }

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "Configuring ToF sensor slot %d", index);
    ESP_LOGI(TAG, "I2C: Port %d, Address 0x%02X, SDA GPIO%d, SCL GPIO%d",
             cfg.i2c_port, cfg.i2c_address, cfg.sda_pin, cfg.scl_pin);
    ESP_LOGI(TAG, "========================================");

    // STEP 1: Apply UART->I2C mode switch workaround for TOF400F modules
    bool uart_success = apply_uart_to_i2c_workaround(cfg);

    // STEP 2: Full I2C bus scan to verify devices after UART mode switch
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "Performing full I2C bus scan after UART mode switch...");
    std::vector<uint8_t> found_devices;
    int64_t full_scan_start = esp_timer_get_time();
    
    if (scan_i2c_bus(cfg, found_devices)) {
        int64_t full_scan_duration = esp_timer_get_time() - full_scan_start;
        ESP_LOGI(TAG, "Full I2C scan completed in %" PRId64 " us (%.2f ms)",
                 full_scan_duration, full_scan_duration / 1000.0f);
        ESP_LOGI(TAG, "Found %zu device(s) on I2C bus:", found_devices.size());
        
        for (uint8_t addr : found_devices) {
            ESP_LOGI(TAG, "  - 0x%02X: %s", addr, get_device_name(addr));
        }
        
        if (found_devices.empty()) {
            ESP_LOGW(TAG, "No I2C devices found after UART mode switch!");
            ESP_LOGW(TAG, "Check:");
            ESP_LOGW(TAG, "  1. I2C pull-up resistors present");
            ESP_LOGW(TAG, "  2. Device power supply");
            ESP_LOGW(TAG, "  3. SDA/SCL connections");
            ESP_LOGW(TAG, "  4. UART connections if using TOF400F");
        }
    } else {
        ESP_LOGE(TAG, "Full I2C bus scan failed!");
    }
    ESP_LOGI(TAG, "========================================");

    // STEP 3: Scan I2C bus and detect sensor type at specific address
    ESP_LOGI(TAG, "Detecting sensor type at address 0x%02X...", cfg.i2c_address);
    int64_t scan_start_time = esp_timer_get_time();
    
    DetectedSensorType detected = scan_and_detect_sensor(cfg);
    detected_types_[index] = detected;
    
    int64_t scan_duration_us = esp_timer_get_time() - scan_start_time;
    ESP_LOGI(TAG, "Sensor type detection completed in %" PRId64 " us (%.2f ms)",
             scan_duration_us, scan_duration_us / 1000.0f);

    // STEP 4: Initialize appropriate driver based on detection
    switch (detected) {
        case DetectedSensorType::VL53L0X: {
            ESP_LOGI(TAG, "[OK] VL53L0X detected at address 0x%02X", cfg.i2c_address);
            ESP_LOGI(TAG, "Initializing VL53L0X driver...");

            VL53L0X::Config vl_config = vl53l0x_default_config(
                cfg.i2c_port,
                cfg.sda_pin,
                cfg.scl_pin,
                cfg.xshut_pin
            );

            vl_config.i2c_address = cfg.i2c_address;
            vl_config.io_2v8 = cfg.io_2v8;
            vl_config.timeout_ms = cfg.timeout_ms;
            vl_config.timing_budget_us = cfg.timing_budget_us;
            vl_config.signal_rate_limit_mcps = cfg.signal_rate_limit_mcps;

            vl53l0x_sensors_[index] = std::make_unique<VL53L0X>(vl_config);

            const char* error = vl53l0x_sensors_[index]->init();
            if (error != nullptr) {
                ESP_LOGE(TAG, "[X] VL53L0X initialization failed: %s", error);
                vl53l0x_sensors_[index].reset();
                detected_types_[index] = DetectedSensorType::NONE;
                return false;
            }

            ESP_LOGI(TAG, "[OK] VL53L0X sensor %d initialized successfully", index);
            ESP_LOGI(TAG, "========================================");
            return true;
        }

        case DetectedSensorType::VL53L1: {
            ESP_LOGI(TAG, "[OK] VL53L1 detected at address 0x%02X", cfg.i2c_address);
            ESP_LOGI(TAG, "Initializing VL53L1 driver...");

            VL53L1::Config vl_config = vl53l1_default_config(
                cfg.i2c_port,
                cfg.sda_pin,
                cfg.scl_pin,
                cfg.xshut_pin,
                cfg.uart_port,
                cfg.uart_tx_pin,
                cfg.uart_rx_pin
            );

            vl_config.i2c_address = cfg.i2c_address;
            vl_config.timeout_ms = cfg.timeout_ms;
            vl_config.ranging_mode = cfg.ranging_mode;
            vl_config.enable_i2c_mode = false;  // Already switched via workaround

            vl53l1_sensors_[index] = std::make_unique<VL53L1>(vl_config);

            const char* error = vl53l1_sensors_[index]->init();
            if (error != nullptr) {
                ESP_LOGE(TAG, "[X] VL53L1 initialization failed: %s", error);
                vl53l1_sensors_[index].reset();
                detected_types_[index] = DetectedSensorType::NONE;
                return false;
            }

            ESP_LOGI(TAG, "[OK] VL53L1 sensor %d initialized successfully", index);
            ESP_LOGI(TAG, "========================================");
            return true;
        }

        case DetectedSensorType::NONE:
        default: {
            ESP_LOGW(TAG, "[X] No ToF sensor detected at address 0x%02X", cfg.i2c_address);
            ESP_LOGW(TAG, "Troubleshooting:");
            ESP_LOGW(TAG, "  1. Check power supply and ground connections");
            ESP_LOGW(TAG, "  2. Verify I2C pull-up resistors (typically 4.7kÎ©)");
            ESP_LOGW(TAG, "  3. Check SDA/SCL wiring");
            ESP_LOGW(TAG, "  4. Verify sensor address (default 0x29 for both VL53L0X/VL53L1)");
            ESP_LOGW(TAG, "  5. For TOF400F: Ensure UART pins are correctly connected");
            ESP_LOGW(TAG, "========================================");
            return false;
        }
    }
}

bool ToFSensorArray::update_readings(std::vector<SensorCommon::Reading>& readings) {
    readings.resize(num_sensors_);
    bool any_new_data = false;

    for (uint8_t i = 0; i < num_sensors_; ++i) {
        // Initialize to invalid reading
        readings[i].distance_cm = SensorCommon::MAX_DISTANCE_CM;
        readings[i].valid = false;
        readings[i].status = 255;
        readings[i].timestamp_us = esp_timer_get_time();

        switch (detected_types_[i]) {
            case DetectedSensorType::VL53L1: {
                if (!vl53l1_sensors_[i] || !vl53l1_sensors_[i]->isReady()) {
                    ESP_LOGD(TAG, "VL53L1 sensor %d not ready", i);
                    continue;
                }

                VL53L1::MeasurementResult result;
                bool success = vl53l1_sensors_[i]->readSingle(result);

                readings[i].timestamp_us = result.timestamp_us;

                if (!success) {
                    ESP_LOGD(TAG, "VL53L1[%d] read failed", i);
                    readings[i].status = 2;
                    continue;
                }

                if (result.timeout_occurred) {
                    ESP_LOGD(TAG, "VL53L1[%d] timeout", i);
                    readings[i].status = 1;
                    continue;
                }

                float distance_cm = static_cast<float>(result.distance_mm) / 10.0f;
                bool is_in_valid_range = (distance_cm >= SensorCommon::MIN_DISTANCE_CM &&
                                          distance_cm <= SensorCommon::MAX_DISTANCE_CM);
                bool is_special_value = (result.distance_mm >= 8190);

                readings[i].distance_cm = distance_cm;
                readings[i].valid = result.valid && is_in_valid_range && !is_special_value;
                readings[i].status = result.valid ? 0 : (is_special_value ? 255 : 1);

                if (readings[i].valid) {
                    any_new_data = true;
                    ESP_LOGD(TAG, "[VL53L1-%d] %.1f cm (VALID)", i, distance_cm);
                } else if (is_special_value) {
                    ESP_LOGD(TAG, "[VL53L1-%d] No target", i);
                } else {
                    ESP_LOGD(TAG, "[VL53L1-%d] %.1f cm (INVALID)", i, distance_cm);
                }
                break;
            }

            case DetectedSensorType::VL53L0X: {
                if (!vl53l0x_sensors_[i] || !vl53l0x_sensors_[i]->isReady()) {
                    ESP_LOGD(TAG, "VL53L0X sensor %d not ready", i);
                    continue;
                }

                VL53L0X::MeasurementResult result;
                bool success = vl53l0x_sensors_[i]->readSingle(result);

                readings[i].timestamp_us = result.timestamp_us;

                if (!success) {
                    ESP_LOGD(TAG, "VL53L0X[%d] read failed", i);
                    readings[i].status = 2;
                    continue;
                }

                if (result.timeout_occurred) {
                    ESP_LOGD(TAG, "VL53L0X[%d] timeout", i);
                    readings[i].status = 1;
                    continue;
                }

                float distance_cm = static_cast<float>(result.distance_mm) / 10.0f;
                bool is_in_valid_range = (distance_cm >= SensorCommon::MIN_DISTANCE_CM &&
                                          distance_cm <= SensorCommon::MAX_DISTANCE_CM);
                bool is_special_value = (result.distance_mm >= SensorCommon::NO_TARGET_DISTANCE_MM);

                readings[i].distance_cm = distance_cm;
                readings[i].valid = result.valid && is_in_valid_range && !is_special_value;
                readings[i].status = result.valid ? 0 : (is_special_value ? 255 : 1);

                if (readings[i].valid) {
                    any_new_data = true;
                    ESP_LOGD(TAG, "[VL53L0X-%d] %.1f cm (VALID)", i, distance_cm);
                } else if (is_special_value) {
                    ESP_LOGD(TAG, "[VL53L0X-%d] No target", i);
                } else {
                    ESP_LOGD(TAG, "[VL53L0X-%d] %.1f cm (INVALID)", i, distance_cm);
                }
                break;
            }

            case DetectedSensorType::NONE:
            default:
                // Sensor not configured - skip
                ESP_LOGD(TAG, "Sensor %d not configured", i);
                break;
        }
    }

    return any_new_data;
}

// ============================================================================
// Private Helper Methods
// ============================================================================

bool ToFSensorArray::apply_uart_to_i2c_workaround(const ToFSensorConfig& cfg) {
    ESP_LOGI(TAG, "[WORKAROUND] Attempting UART->I2C mode switch for TOF400F modules");
    ESP_LOGI(TAG, "[WORKAROUND] This is safe even if no UART device is connected");
    ESP_LOGI(TAG, "[WORKAROUND] UART: Port %d, TX GPIO%d, RX GPIO%d",
             cfg.uart_port, cfg.uart_tx_pin, cfg.uart_rx_pin);
    
    int64_t start_time = esp_timer_get_time();

    // Configure UART (115200 baud, 8N1 as per datasheet)
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_DEFAULT,
        .flags = {0}
    };

    esp_err_t err = uart_param_config(cfg.uart_port, &uart_config);
    if (err != ESP_OK) {
        ESP_LOGD(TAG, "[WORKAROUND] UART param config failed: %s", esp_err_to_name(err));
        return false;
    }

    err = uart_set_pin(cfg.uart_port, cfg.uart_tx_pin, cfg.uart_rx_pin,
                       UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
        ESP_LOGD(TAG, "[WORKAROUND] UART set pin failed: %s", esp_err_to_name(err));
        return false;
    }

    err = uart_driver_install(cfg.uart_port, 1024, 1024, 0, NULL, 0);
    if (err != ESP_OK) {
        ESP_LOGD(TAG, "[WORKAROUND] UART driver install failed: %s", esp_err_to_name(err));
        return false;
    }

    // Small delay for UART stability
    vTaskDelay(pdMS_TO_TICKS(50));

    // Try multiple slave addresses (common TOF400F addresses)
    // Datasheet default is 0x01, but user might have changed it
    uint8_t slave_addresses[] = {0x01, 0x02, 0x10, 0x11};
    bool command_acknowledged = false;
    uint8_t successful_slave_addr = 0;
    
    for (size_t i = 0; i < sizeof(slave_addresses); i++) {
        uint8_t slave_addr = slave_addresses[i];
        
        ESP_LOGD(TAG, "[WORKAROUND] Trying slave address 0x%02X...", slave_addr);
        
        // Build Modbus command: Write register 0x0009 = 0x0001 (Disable IIC enable register)
        // Per datasheet: 0x0001 = Prohibited (MCU releases IO) = switches to I2C mode
        uint8_t cmd[8];
        size_t cmd_len = build_modbus_write_command(cmd, slave_addr, 0x0009, 0x0001);
        
        // Flush RX buffer before sending
        uart_flush_input(cfg.uart_port);
        
        // Send command
        int written = uart_write_bytes(cfg.uart_port, cmd, cmd_len);
        
        if (written != cmd_len) {
            ESP_LOGD(TAG, "[WORKAROUND] UART write failed: %d/%zu bytes", written, cmd_len);
            continue;
        }
        
        ESP_LOGD(TAG, "[WORKAROUND] Command sent: 0x%02X 0x06 0x00 0x09 0x00 0x01 [CRC]", slave_addr);
        
        // Wait for response (per datasheet, module echoes command on success)
        // Expected response: Same as sent command (echo)
        uint8_t response[8];
        int len = uart_read_bytes(cfg.uart_port, response, sizeof(response),
                                 pdMS_TO_TICKS(500));
        
        if (len == 8) {
            // Verify response matches command (per datasheet: module echoes on success)
            bool response_valid = true;
            for (int j = 0; j < 8; j++) {
                if (response[j] != cmd[j]) {
                    response_valid = false;
                    break;
                }
            }
            
            if (response_valid) {
                ESP_LOGI(TAG, "[WORKAROUND] ✓ Module acknowledged at slave address 0x%02X", slave_addr);
                ESP_LOGD(TAG, "[WORKAROUND] Response: %02X %02X %02X %02X %02X %02X %02X %02X",
                         response[0], response[1], response[2], response[3],
                         response[4], response[5], response[6], response[7]);
                command_acknowledged = true;
                successful_slave_addr = slave_addr;
                break;
            } else {
                ESP_LOGD(TAG, "[WORKAROUND] Response mismatch from slave 0x%02X", slave_addr);
            }
        } else if (len > 0) {
            ESP_LOGD(TAG, "[WORKAROUND] Partial response (%d bytes) from slave 0x%02X", len, slave_addr);
        } else {
            ESP_LOGD(TAG, "[WORKAROUND] No response from slave 0x%02X (timeout)", slave_addr);
        }
    }
    
    int64_t duration_us = esp_timer_get_time() - start_time;
    
    // Cleanup UART
    uart_driver_delete(cfg.uart_port);
    
    if (command_acknowledged) {
        ESP_LOGI(TAG, "[WORKAROUND] Command acknowledged by slave 0x%02X", successful_slave_addr);
        ESP_LOGI(TAG, "[WORKAROUND] Waiting for module to switch to I2C mode...");
        
        // Per datasheet: Module needs time to switch modes and reinitialize
        // Allow 500ms for mode switch and VL53L1 chip boot
        vTaskDelay(pdMS_TO_TICKS(500));
        
        ESP_LOGI(TAG, "[WORKAROUND] Duration: %" PRId64 " us (%.2f ms)",
                 duration_us, duration_us / 1000.0f);
        ESP_LOGI(TAG, "[WORKAROUND] Result: SUCCESS - Module switched to I2C mode");
        return true;
    } else {
        ESP_LOGI(TAG, "[WORKAROUND] Duration: %" PRId64 " us (%.2f ms)",
                 duration_us, duration_us / 1000.0f);
        ESP_LOGI(TAG, "[WORKAROUND] Result: No response from any slave address");
        ESP_LOGI(TAG, "[WORKAROUND] This is normal if:");
        ESP_LOGI(TAG, "[WORKAROUND]   1. No TOF400F module connected");
        ESP_LOGI(TAG, "[WORKAROUND]   2. Module already in I2C mode");
        ESP_LOGI(TAG, "[WORKAROUND]   3. UART TX/RX not connected (or wrong pins)");
        
        // Still allow 200ms in case module was already in I2C mode
        vTaskDelay(pdMS_TO_TICKS(200));
        
        return false;  // Return false but don't abort - module might already be in I2C mode
    }
}

bool ToFSensorArray::scan_i2c_bus(const ToFSensorConfig& cfg, std::vector<uint8_t>& found_addresses) {
    found_addresses.clear();

    ESP_LOGD(TAG, "Creating I2C bus for full scan...");
    ESP_LOGD(TAG, "  Port: I2C_%d", cfg.i2c_port);
    ESP_LOGD(TAG, "  SDA: GPIO%d, SCL: GPIO%d", cfg.sda_pin, cfg.scl_pin);

    // Configure I2C bus with longer timeout for problematic buses
    i2c_master_bus_config_t bus_config = {
        .i2c_port = cfg.i2c_port,
        .sda_io_num = cfg.sda_pin,
        .scl_io_num = cfg.scl_pin,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 0,
        .flags = {.enable_internal_pullup = true},
    };

    i2c_master_bus_handle_t bus_handle;
    esp_err_t err = i2c_new_master_bus(&bus_config, &bus_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create I2C bus for scan: %s", esp_err_to_name(err));
        return false;
    }

    ESP_LOGD(TAG, "Scanning addresses 0x08 to 0x77...");

    // Scan all valid 7-bit I2C addresses with verification
    for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
        // Verify device responds twice to reduce false positives
        bool first_probe = probe_i2c_address(bus_handle, addr, 100000);
        if (first_probe) {
            vTaskDelay(1); // Small delay
            bool second_probe = probe_i2c_address(bus_handle, addr, 100000);
            
            // Only add if device responds both times
            if (second_probe) {
                found_addresses.push_back(addr);
                ESP_LOGD(TAG, "  0x%02X: ACK (verified)", addr);
            }
        }
        vTaskDelay(1); // Small delay between probes
    }

    // Cleanup
    i2c_del_master_bus(bus_handle);

    return true;
}

DetectedSensorType ToFSensorArray::scan_and_detect_sensor(const ToFSensorConfig& cfg) {
    ESP_LOGD(TAG, "Creating temporary I2C bus for scanning...");

    // Configure I2C bus
    i2c_master_bus_config_t bus_config = {
        .i2c_port = cfg.i2c_port,
        .sda_io_num = cfg.sda_pin,
        .scl_io_num = cfg.scl_pin,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 0,
        .flags = {.enable_internal_pullup = true},
    };

    i2c_master_bus_handle_t bus_handle;
    esp_err_t err = i2c_new_master_bus(&bus_config, &bus_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create I2C bus: %s", esp_err_to_name(err));
        return DetectedSensorType::NONE;
    }

    ESP_LOGD(TAG, "I2C bus created, probing address 0x%02X...", cfg.i2c_address);

    // Probe the configured address
    bool device_present = probe_i2c_address(bus_handle, cfg.i2c_address);

    DetectedSensorType result = DetectedSensorType::NONE;

    if (device_present) {
        ESP_LOGI(TAG, "Device detected at 0x%02X - attempting identification...", cfg.i2c_address);
        
        // Both VL53L0X and VL53L1 use address 0x29 by default
        // We need to attempt reading device ID to distinguish them
        
        // Try VL53L1 identification register first (0x010F returns 0xEA for VL53L1)
        i2c_device_config_t dev_config = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = cfg.i2c_address,
            .scl_speed_hz = 100000,
            .scl_wait_us = 0,
            .flags = {0},
        };

        i2c_master_dev_handle_t dev_handle;
        err = i2c_master_bus_add_device(bus_handle, &dev_config, &dev_handle);
        
        if (err == ESP_OK) {
            // Try VL53L1 identification (register 0x010F)
            uint8_t reg_addr[2] = {0x01, 0x0F};
            uint8_t id_data = 0;
            
            err = i2c_master_transmit_receive(dev_handle, reg_addr, 2, &id_data, 1, 100);
            
            if (err == ESP_OK && id_data == 0xEA) {
                ESP_LOGI(TAG, "Identified as VL53L1 (ID: 0x%02X)", id_data);
                result = DetectedSensorType::VL53L1;
            } else {
                // Try VL53L0X identification (register 0xC0)
                uint8_t vl0_reg = 0xC0;
                err = i2c_master_transmit_receive(dev_handle, &vl0_reg, 1, &id_data, 1, 100);
                
                if (err == ESP_OK && id_data == 0xEE) {
                    ESP_LOGI(TAG, "Identified as VL53L0X (ID: 0x%02X)", id_data);
                    result = DetectedSensorType::VL53L0X;
                } else {
                    ESP_LOGW(TAG, "Device at 0x%02X responded but ID unknown (0x%02X)", 
                             cfg.i2c_address, id_data);
                    // Default to VL53L0X for backward compatibility
                    result = DetectedSensorType::VL53L0X;
                }
            }
            
            i2c_master_bus_rm_device(dev_handle);
        }
    } else {
        ESP_LOGD(TAG, "No device found at address 0x%02X", cfg.i2c_address);
    }

    // Cleanup bus
    i2c_del_master_bus(bus_handle);

    return result;
}

bool ToFSensorArray::probe_i2c_address(i2c_master_bus_handle_t bus_handle,
                                       uint8_t address,
                                       uint32_t freq_hz) {
    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = address,
        .scl_speed_hz = freq_hz,
        .scl_wait_us = 0,
        .flags = {0},
    };

    i2c_master_dev_handle_t dev_handle;
    esp_err_t err = i2c_master_bus_add_device(bus_handle, &dev_config, &dev_handle);
    if (err != ESP_OK) {
        return false;
    }

    // Try to read a single byte
    uint8_t dummy;
    err = i2c_master_receive(dev_handle, &dummy, 1, 50);

    i2c_master_bus_rm_device(dev_handle);
    
    return (err == ESP_OK);
}

uint16_t ToFSensorArray::calculate_modbus_crc(const uint8_t* data, size_t length) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

const char* ToFSensorArray::get_device_name(uint8_t address) {
    for (size_t i = 0; i < NUM_KNOWN_DEVICES; i++) {
        if (KNOWN_DEVICES[i].address == address) {
            return KNOWN_DEVICES[i].name;
        }
    }
    return "Unknown device";
}

size_t ToFSensorArray::build_modbus_write_command(uint8_t* buffer,
                                                   uint8_t slave_addr,
                                                   uint16_t reg_addr,
                                                   uint16_t value) {
    buffer[0] = slave_addr;
    buffer[1] = 0x06;  // Function: Write Single Register
    buffer[2] = (reg_addr >> 8) & 0xFF;
    buffer[3] = reg_addr & 0xFF;
    buffer[4] = (value >> 8) & 0xFF;
    buffer[5] = value & 0xFF;
    
    uint16_t crc = calculate_modbus_crc(buffer, 6);
    buffer[6] = crc & 0xFF;
    buffer[7] = (crc >> 8) & 0xFF;
    
    return 8;
}

// ============================================================================
// ToFSensorManager Implementation
// ============================================================================

ToFSensorManager& ToFSensorManager::instance() {
    static ToFSensorManager instance;
    return instance;
}

ToFSensorManager::ToFSensorManager()
    : array_(nullptr)
    , data_mutex_(nullptr)
    , task_handle_(nullptr)
    , running_(false)
    , paused_(false) {
    data_mutex_ = xSemaphoreCreateMutex();
    if (!data_mutex_) {
        ESP_LOGE(TAG, "Failed to create data mutex");
    }
}

ToFSensorManager::~ToFSensorManager() {
    stop();
    if (array_) delete array_;
    if (data_mutex_) vSemaphoreDelete(data_mutex_);
}

bool ToFSensorManager::configure(const ToFSensorConfig* sensor_configs, uint8_t num_sensors) {
    if (array_) {
        ESP_LOGW(TAG, "ToFSensorManager already configured");
        return false;
    }

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "ToFSensorManager: Configuring %d sensor(s)", num_sensors);
    ESP_LOGI(TAG, "========================================");

    array_ = new ToFSensorArray(num_sensors);

    uint8_t successfully_configured = 0;
    for (uint8_t i = 0; i < num_sensors; i++) {
        if (array_->add_sensor(i, sensor_configs[i])) {
            successfully_configured++;
        }
    }

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "ToFSensorManager: %d/%d sensor(s) configured successfully",
             successfully_configured, num_sensors);
    ESP_LOGI(TAG, "========================================");

    if (successfully_configured == 0) {
        ESP_LOGE(TAG, "No ToF sensors configured - aborting");
        delete array_;
        array_ = nullptr;
        return false;
    }

    latest_readings_.resize(num_sensors);
    return true;
}

bool ToFSensorManager::start_reading_task(uint32_t read_interval_ms, UBaseType_t priority) {
  if (task_handle_) return false;

  // Dynamically allocate TaskParams to avoid stack invalidation
  TaskParams* params = new TaskParams{this, read_interval_ms};
  if (!params) {
    ESP_LOGE(TAG, "Failed to allocate TaskParams");
    return false;
  }

  running_ = true;
  paused_ = false;

  BaseType_t res = xTaskCreate(reading_task, "tof_read", 4096, params, priority, &task_handle_);
  if (res != pdPASS) {
    delete params;  // Clean up on failure
    running_ = false;
    return false;
  }

  ESP_LOGI(TAG, "ToF reading task started (interval: %lu ms)", read_interval_ms);
  return true;
}

void ToFSensorManager::reading_task(void* param) {
  TaskParams* params = static_cast<TaskParams*>(param);
  ToFSensorManager* self = params->manager;
  uint32_t interval_ms = params->interval_ms;

  ESP_LOGI(TAG, "ToF reading task running");

  while (self->running_) {
    if (!self->paused_) {
      std::vector<SensorCommon::Reading> readings(self->latest_readings_.size());  // ADD THIS LINE

      if (self->array_->update_readings(readings)) {
        // Update latest readings with mutex protection
        if (xSemaphoreTake(self->data_mutex_, pdMS_TO_TICKS(10)) == pdTRUE) {
          self->latest_readings_ = readings;
          xSemaphoreGive(self->data_mutex_);
        }
      }
    }

    vTaskDelay(pdMS_TO_TICKS(interval_ms));
  }

  ESP_LOGI(TAG, "ToF reading task exiting");

  // Clean up the dynamically allocated params
  delete params;

  vTaskDelete(NULL);
}

bool ToFSensorManager::get_latest_readings(std::vector<SensorCommon::Reading>& readings) {
    if (xSemaphoreTake(data_mutex_, pdMS_TO_TICKS(100)) == pdTRUE) {
        readings = latest_readings_;
        xSemaphoreGive(data_mutex_);
        return true;
    }
    return false;
}

void ToFSensorManager::pause() {
    paused_ = true;
    ESP_LOGI(TAG, "ToF reading paused");
}

void ToFSensorManager::resume() {
    paused_ = false;
    ESP_LOGI(TAG, "ToF reading resumed");
}

void ToFSensorManager::stop() {
    running_ = false;
    if (task_handle_) {
        vTaskDelay(pdMS_TO_TICKS(200));
        task_handle_ = nullptr;
    }
    ESP_LOGI(TAG, "ToF reading stopped");
}

std::vector<DetectedSensorType> ToFSensorManager::get_detected_types() const {
    if (array_) {
        std::vector<DetectedSensorType> types;
        for (uint8_t i = 0; i < latest_readings_.size(); i++) {
            types.push_back(array_->get_detected_type(i));
        }
        return types;
    }
    return {};
}
