#include "i2c_scanner.hpp"
#include "freertos/FreeRTOS.h"  // Add this line
#include <vector>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

const char* I2CScanner::TAG = "I2C_SCANNER";

// Common I2C devices for reference
const I2CScanner::DeviceInfo I2CScanner::KNOWN_DEVICES[] = {
    {0x29, "VL53L0X ToF Sensor"},
    {0x30, "VL53L1X ToF Sensor"},
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

bool I2CScanner::scan(i2c_port_t port,
                      std::vector<uint8_t>& found_addresses,
                      gpio_num_t sda_pin,
                      gpio_num_t scl_pin,
                      uint32_t freq_hz) {

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "SCANNING I2C PORT %d", port);
    ESP_LOGI(TAG, "SDA: GPIO%d, SCL: GPIO%d, Freq: %lu Hz", sda_pin, scl_pin, freq_hz);
    ESP_LOGI(TAG, "========================================");

    found_addresses.clear();

    // Configure I2C master bus
    i2c_master_bus_config_t bus_config = {
        .i2c_port = port,
        .sda_io_num = sda_pin,
        .scl_io_num = scl_pin,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,  // Added
        .trans_queue_depth = 0,  // Added
        .flags = {
            .enable_internal_pullup = true,
        },
    };

    i2c_master_bus_handle_t bus_handle;
    esp_err_t err = i2c_new_master_bus(&bus_config, &bus_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create I2C bus: %s", esp_err_to_name(err));
        return false;
    }

    // Scan all valid 7-bit addresses (0x08 to 0x77)
    for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
        // Create a temporary device handle for probing
        i2c_device_config_t dev_config = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = addr,
            .scl_speed_hz = freq_hz,
            .scl_wait_us = 0,  // Added
            .flags = {0},  // Added
        };

        i2c_master_dev_handle_t dev_handle;
        err = i2c_master_bus_add_device(bus_handle, &dev_config, &dev_handle);
        if (err != ESP_OK) {
            continue;  // Skip this address
        }

        // Try to probe the device with a simple read
        uint8_t dummy;
        err = i2c_master_receive(dev_handle, &dummy, 1, 50);  // 50ms timeout

        // Remove the device handle
        i2c_master_bus_rm_device(dev_handle);

        if (err == ESP_OK) {
            found_addresses.push_back(addr);
            const char* device_name = getDeviceName(addr);
            ESP_LOGI(TAG, "Found device at 0x%02X - %s", addr, device_name);
        }

        vTaskDelay(pdMS_TO_TICKS(1));  // Small delay between probes
    }

    // Clean up bus
    i2c_del_master_bus(bus_handle);

    if (found_addresses.empty()) {
        ESP_LOGW(TAG, "NO I2C DEVICES FOUND!");
        ESP_LOGW(TAG, "Check:");
        ESP_LOGW(TAG, "  - Device power supply");
        ESP_LOGW(TAG, "  - Pull-up resistors (4.7kΩ typical)");
        ESP_LOGW(TAG, "  - Wire connections");
    } else {
        ESP_LOGI(TAG, "========================================");
        ESP_LOGI(TAG, "Found %zu device(s)", found_addresses.size());
        ESP_LOGI(TAG, "========================================");
    }

    return true;
}

bool I2CScanner::quickScan(i2c_port_t port,
                           gpio_num_t sda_pin,
                           gpio_num_t scl_pin,
                           uint32_t freq_hz) {
    std::vector<uint8_t> addresses;
    return scan(port, addresses, sda_pin, scl_pin, freq_hz);
}

bool I2CScanner::probe(i2c_port_t port,
                       uint8_t address,
                       gpio_num_t sda_pin,
                       gpio_num_t scl_pin,
                       uint32_t freq_hz) {

    if (address < 0x08 || address > 0x77) {
        ESP_LOGE(TAG, "Invalid I2C address: 0x%02X", address);
        return false;
    }

    // Configure I2C master bus
    i2c_master_bus_config_t bus_config = {
        .i2c_port = port,
        .sda_io_num = sda_pin,
        .scl_io_num = scl_pin,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,  // Added
        .trans_queue_depth = 0,  // Added
        .flags = {
            .enable_internal_pullup = true,
        },
    };

    i2c_master_bus_handle_t bus_handle;
    esp_err_t err = i2c_new_master_bus(&bus_config, &bus_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create I2C bus: %s", esp_err_to_name(err));
        return false;
    }

    bool found = probeWithBus(bus_handle, address, freq_hz);

    // Clean up
    i2c_del_master_bus(bus_handle);

    return found;
}

bool I2CScanner::probeWithBus(i2c_master_bus_handle_t bus_handle,
                              uint8_t address,
                              uint32_t freq_hz) {

    if (!bus_handle) {
        ESP_LOGE(TAG, "Invalid bus handle");
        return false;
    }

    if (address < 0x08 || address > 0x77) {
        ESP_LOGE(TAG, "Invalid I2C address: 0x%02X", address);
        return false;
    }

    // Create device handle
    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = address,
        .scl_speed_hz = freq_hz,
        .scl_wait_us = 0,  // Added
        .flags = {0},  // Added
    };

    i2c_master_dev_handle_t dev_handle;
    esp_err_t err = i2c_master_bus_add_device(bus_handle, &dev_config, &dev_handle);
    if (err != ESP_OK) {
        ESP_LOGD(TAG, "Failed to add device 0x%02X: %s", address, esp_err_to_name(err));
        return false;
    }

    // Try to probe the device
    uint8_t dummy;
    err = i2c_master_receive(dev_handle, &dummy, 1, 50);

    // Clean up device handle
    i2c_master_bus_rm_device(dev_handle);

    if (err == ESP_OK) {
        ESP_LOGD(TAG, "Device 0x%02X responded", address);
        return true;
    } else {
        ESP_LOGD(TAG, "Device 0x%02X no response: %s", address, esp_err_to_name(err));
        return false;
    }
}

void I2CScanner::printResults(const std::vector<uint8_t>& addresses) {
    if (addresses.empty()) {
        ESP_LOGI(TAG, "No I2C devices found");
        return;
    }

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "I2C Devices Found (%zu):", addresses.size());
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "Address | Device");
    ESP_LOGI(TAG, "--------|---------------------------");

    for (auto addr : addresses) {
        const char* device_name = getDeviceName(addr);
        ESP_LOGI(TAG, "0x%02X    | %s", addr, device_name);
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
