#include "include/i2c_scanner.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cstring>

const char* I2CScanner::TAG = "I2C_SCANNER";

const I2CScanner::DeviceInfo I2CScanner::KNOWN_DEVICES[] = {
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

const size_t I2CScanner::NUM_KNOWN_DEVICES = sizeof(KNOWN_DEVICES) / sizeof(KNOWN_DEVICES[0]);

bool I2CScanner::scan(i2c_port_t port, std::vector<uint8_t>& found_addresses,
                      gpio_num_t sda_pin, gpio_num_t scl_pin, uint32_t freq_hz) {
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "Starting I2C Bus Scan");
    ESP_LOGI(TAG, "Port: I2C_%d, SDA: GPIO%d, SCL: GPIO%d", port, sda_pin, scl_pin);
    ESP_LOGI(TAG, "Frequency: %lu Hz (%lu kHz)", freq_hz, freq_hz / 1000);
    ESP_LOGI(TAG, "========================================");

    found_addresses.clear();

    ESP_LOGD(TAG, "Step 1: Configuring I2C bus...");
    i2c_master_bus_config_t bus_config = {
        .i2c_port = port,
        .sda_io_num = sda_pin,
        .scl_io_num = scl_pin,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 0,
        .flags = {.enable_internal_pullup = true},
    };

    i2c_master_bus_handle_t bus_handle;
    esp_err_t err = i2c_new_master_bus(&bus_config, &bus_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "FAILED to create I2C bus: %s", esp_err_to_name(err));
        ESP_LOGE(TAG, "Troubleshooting:");
        ESP_LOGE(TAG, "  1. Check if I2C port %d is already in use", port);
        ESP_LOGE(TAG, "  2. Verify GPIO pins are not conflicting");
        ESP_LOGE(TAG, "  3. Check for hardware I2C peripheral availability");
        return false;
    }
    ESP_LOGI(TAG, "Step 1: I2C bus created successfully");

    ESP_LOGD(TAG, "Step 2: Scanning addresses 0x08 to 0x77...");
    uint16_t attempts = 0;

    // Use i2c_master_probe() - the correct way to scan!
    for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
        attempts++;

        err = i2c_master_probe(bus_handle, addr, 50);

        if (err == ESP_OK) {
            found_addresses.push_back(addr);
            ESP_LOGI(TAG, "  0x%02X: FOUND - %s", addr, getDeviceName(addr));
        } else {
            ESP_LOGD(TAG, "  0x%02X: No response", addr);
        }

        vTaskDelay(1);
    }

    i2c_del_master_bus(bus_handle);

    ESP_LOGI(TAG, "Step 2: Scan complete");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "Scan Results:");
    ESP_LOGI(TAG, "  Addresses scanned: %d", attempts);
    ESP_LOGI(TAG, "  Devices found: %zu", found_addresses.size());
    ESP_LOGI(TAG, "========================================");

    if (found_addresses.empty()) {
        ESP_LOGW(TAG, "NO DEVICES FOUND!");
        ESP_LOGW(TAG, "Troubleshooting:");
        ESP_LOGW(TAG, "  1. Check device power supply (3.3V or 5V as required)");
        ESP_LOGW(TAG, "  2. Verify pull-up resistors (typically 4.7kΩ) on SDA and SCL");
        ESP_LOGW(TAG, "  3. Check wire connections and soldering");
        ESP_LOGW(TAG, "  4. Ensure device is not in reset or sleep mode");
        ESP_LOGW(TAG, "  5. Verify correct I2C address for your device");
        ESP_LOGW(TAG, "  6. Try lower frequency (e.g., 100kHz instead of 400kHz)");
        ESP_LOGW(TAG, "  7. For TOF400F: Module may be in UART mode - use switchTOF400FToI2CMode()");
    } else {
        printResults(found_addresses);
    }

    return true;
}

bool I2CScanner::quickScan(i2c_port_t port, gpio_num_t sda_pin, gpio_num_t scl_pin, uint32_t freq_hz) {
    ESP_LOGI(TAG, "Quick scan initiated on I2C_%d", port);
    std::vector<uint8_t> addresses;
    bool result = scan(port, addresses, sda_pin, scl_pin, freq_hz);

    if (!result) {
        ESP_LOGE(TAG, "Quick scan FAILED - bus configuration error");
    }

    return result;
}

bool I2CScanner::probe(i2c_port_t port, uint8_t address,
                       gpio_num_t sda_pin, gpio_num_t scl_pin, uint32_t freq_hz) {
    if (address < 0x08 || address > 0x77) {
        ESP_LOGE(TAG, "Invalid address: 0x%02X (valid range: 0x08-0x77)", address);
        return false;
    }

    ESP_LOGD(TAG, "Probing single address 0x%02X on I2C_%d", address, port);
    ESP_LOGD(TAG, "  SDA: GPIO%d, SCL: GPIO%d, Freq: %lu Hz", sda_pin, scl_pin, freq_hz);

    i2c_master_bus_config_t bus_config = {
        .i2c_port = port,
        .sda_io_num = sda_pin,
        .scl_io_num = scl_pin,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 0,
        .flags = {.enable_internal_pullup = true},
    };

    i2c_master_bus_handle_t bus_handle;
    esp_err_t err = i2c_new_master_bus(&bus_config, &bus_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create I2C bus for probe: %s", esp_err_to_name(err));
        return false;
    }

    bool found = probeWithBus(bus_handle, address, freq_hz);
    i2c_del_master_bus(bus_handle);

    if (found) {
        ESP_LOGI(TAG, "✓ Device 0x%02X responded - %s", address, getDeviceName(address));
    } else {
        ESP_LOGW(TAG, "✗ Device 0x%02X did not respond", address);
        ESP_LOGW(TAG, "  Troubleshooting:");
        ESP_LOGW(TAG, "    - Verify device address (check datasheet)");
        ESP_LOGW(TAG, "    - Check power and ground connections");
        ESP_LOGW(TAG, "    - Verify pull-up resistors present");
        ESP_LOGW(TAG, "    - Try different I2C speed");
        if (address == 0x29) {
            ESP_LOGW(TAG, "    - For TOF400F: Module may be in UART mode");
        }
    }

    return found;
}

bool I2CScanner::probeWithBus(i2c_master_bus_handle_t bus_handle, uint8_t address, uint32_t freq_hz) {
    if (!bus_handle) {
        ESP_LOGE(TAG, "Invalid bus handle (NULL)");
        return false;
    }

    if (address < 0x08 || address > 0x77) {
        ESP_LOGE(TAG, "Address 0x%02X out of valid range (0x08-0x77)", address);
        return false;
    }

    ESP_LOGD(TAG, "probeWithBus: Testing address 0x%02X at %lu Hz", address, freq_hz);

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
        ESP_LOGD(TAG, "  Cannot add device to bus: %s", esp_err_to_name(err));
        return false;
    }

    uint8_t dummy;
    err = i2c_master_receive(dev_handle, &dummy, 1, 50);

    if (err == ESP_OK) {
        ESP_LOGD(TAG, "  Device 0x%02X ACK received", address);
    } else if (err == ESP_ERR_TIMEOUT) {
        ESP_LOGD(TAG, "  Device 0x%02X timeout (no response)", address);
    } else {
        ESP_LOGD(TAG, "  Device 0x%02X error: %s", address, esp_err_to_name(err));
    }

    i2c_master_bus_rm_device(dev_handle);
    return (err == ESP_OK);
}

void I2CScanner::printResults(const std::vector<uint8_t>& addresses) {
    if (addresses.empty()) {
        ESP_LOGI(TAG, "No devices found");
        return;
    }

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "I2C Devices Found (%zu):", addresses.size());
    ESP_LOGI(TAG, "Address | Device");
    ESP_LOGI(TAG, "--------|---------------------------");

    for (auto addr : addresses) {
        ESP_LOGI(TAG, "0x%02X    | %s", addr, getDeviceName(addr));
    }

    ESP_LOGI(TAG, "========================================");
}

const char* I2CScanner::getDeviceName(uint8_t address) {
    for (size_t i = 0; i < NUM_KNOWN_DEVICES; i++) {
        if (KNOWN_DEVICES[i].address == address) {
            return KNOWN_DEVICES[i].name;
        }
    }
    return "Unknown device";
}

// ===== MODBUS/TOF400F SUPPORT =====

uint16_t I2CScanner::calculateModbusCRC(const uint8_t* data, size_t length) {
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

size_t I2CScanner::buildModbusWriteCommand(uint8_t* buffer, uint8_t slave_addr,
                                          uint16_t reg_addr, uint16_t value) {
    buffer[0] = slave_addr;
    buffer[1] = 0x06;  // Function: Write Single Register
    buffer[2] = (reg_addr >> 8) & 0xFF;
    buffer[3] = reg_addr & 0xFF;
    buffer[4] = (value >> 8) & 0xFF;
    buffer[5] = value & 0xFF;

    uint16_t crc = calculateModbusCRC(buffer, 6);
    buffer[6] = crc & 0xFF;
    buffer[7] = (crc >> 8) & 0xFF;

    return 8;
}

bool I2CScanner::switchTOF400FToI2CMode(uart_port_t uart_port,
                                        gpio_num_t tx_pin,
                                        gpio_num_t rx_pin,
                                        uint32_t baud_rate,
                                        uint8_t slave_addr) {
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "TOF400F: Switching from UART to I2C mode");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "UART Port: %d", uart_port);
    ESP_LOGI(TAG, "TX Pin: GPIO%d (to module RX)", tx_pin);
    ESP_LOGI(TAG, "RX Pin: GPIO%d (from module TX)", rx_pin);
    ESP_LOGI(TAG, "Baud Rate: %lu", baud_rate);
    ESP_LOGI(TAG, "Slave Address: 0x%02X", slave_addr);

    // Step 1: Configure UART
    ESP_LOGI(TAG, "Step 1: Configuring UART...");

    uart_config_t uart_config = {
        .baud_rate = (int)baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t err = uart_param_config(uart_port, &uart_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "UART param config failed: %s", esp_err_to_name(err));
        return false;
    }

    err = uart_set_pin(uart_port, tx_pin, rx_pin,
                       UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "UART set pin failed: %s", esp_err_to_name(err));
        return false;
    }

    err = uart_driver_install(uart_port, 1024, 1024, 0, NULL, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "UART driver install failed: %s", esp_err_to_name(err));
        return false;
    }

    ESP_LOGI(TAG, "Step 1: UART configured - OK");

    // Small delay for module to be ready
    vTaskDelay(pdMS_TO_TICKS(100));

    // Step 2: Build and send Modbus command
    ESP_LOGI(TAG, "Step 2: Sending I2C mode enable command...");
    ESP_LOGI(TAG, "  Command: Write register 0x0009 = 0x0001");
    ESP_LOGI(TAG, "  (Register name 'DISABLE_IIC' enables I2C when set to 1)");

    uint8_t cmd[8];
    size_t cmd_len = buildModbusWriteCommand(cmd, slave_addr, 0x0009, 0x0001);

    // Log command bytes
    ESP_LOGD(TAG, "  Modbus command:");
    for (size_t i = 0; i < cmd_len; i++) {
        ESP_LOGD(TAG, "    [%zu] 0x%02X", i, cmd[i]);
    }

    // Flush RX buffer before sending
    uart_flush_input(uart_port);

    // Send command
    int written = uart_write_bytes(uart_port, cmd, cmd_len);
    if (written != cmd_len) {
        ESP_LOGE(TAG, "UART write failed: wrote %d/%zu bytes", written, cmd_len);
        uart_driver_delete(uart_port);
        return false;
    }

    ESP_LOGI(TAG, "  Command sent (%zu bytes)", cmd_len);

    // Step 3: Wait for response
    ESP_LOGI(TAG, "Step 3: Waiting for response...");

    uint8_t response[8];
    int len = uart_read_bytes(uart_port, response, sizeof(response),
                             pdMS_TO_TICKS(500));

    if (len < 0) {
        ESP_LOGW(TAG, "No response from module (timeout)");
        uart_driver_delete(uart_port);
        return false;
    }

    ESP_LOGI(TAG, "  Received %d bytes", len);

    // Log response bytes
    if (len > 0) {
        ESP_LOGD(TAG, "  Response:");
        for (int i = 0; i < len; i++) {
            ESP_LOGD(TAG, "    [%d] 0x%02X", i, response[i]);
        }
    }

    // Verify response (should echo the command)
    bool response_valid = false;
    if (len == 8) {
        // Verify CRC
        uint16_t received_crc = response[6] | (response[7] << 8);
        uint16_t calculated_crc = calculateModbusCRC(response, 6);

        if (received_crc == calculated_crc) {
            // Check if response matches command (successful write echoes command)
            if (memcmp(cmd, response, 8) == 0) {
                response_valid = true;
                ESP_LOGI(TAG, "  Response valid: Command echoed successfully");
            } else {
                ESP_LOGW(TAG, "  Response CRC valid but content differs");
            }
        } else {
            ESP_LOGW(TAG, "  CRC mismatch: received 0x%04X, calculated 0x%04X",
                    received_crc, calculated_crc);
        }
    } else {
        ESP_LOGW(TAG, "  Unexpected response length: %d (expected 8)", len);
    }

    // Step 4: Cleanup UART
    uart_driver_delete(uart_port);
    ESP_LOGI(TAG, "Step 4: UART cleaned up");

    if (response_valid) {
        ESP_LOGI(TAG, "========================================");
        ESP_LOGI(TAG, "TOF400F: Successfully switched to I2C mode");
        ESP_LOGI(TAG, "VL53L1 sensor should now be accessible at I2C address 0x29");
        ESP_LOGI(TAG, "Wait ~200ms before attempting I2C communication");
        ESP_LOGI(TAG, "========================================");

        // Wait for mode switch to complete
        vTaskDelay(pdMS_TO_TICKS(200));
        return true;
    } else {
        ESP_LOGW(TAG, "========================================");
        ESP_LOGW(TAG, "TOF400F: Mode switch command sent but response unclear");
        ESP_LOGW(TAG, "Module may have switched modes anyway - verify with I2C scan");
        ESP_LOGW(TAG, "========================================");

        // Still wait in case it worked
        vTaskDelay(pdMS_TO_TICKS(200));
        return false;
    }
}
