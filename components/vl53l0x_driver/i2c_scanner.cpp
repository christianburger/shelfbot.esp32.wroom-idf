#include "i2c_scanner.hpp"
#include <vector>

const char* I2CScanner::TAG = "I2C_SCANNER";

bool I2CScanner::scan(i2c_port_t port,
                     std::vector<uint8_t>& found_addresses,
                     gpio_num_t sda_pin,
                     gpio_num_t scl_pin) {

    // Configure I2C for scanning
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sda_pin,
        .scl_io_num = scl_pin,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master = {.clk_speed = 100000},
        .clk_flags = 0,
    };

    esp_err_t err = i2c_param_config(port, &conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C param config failed: %s", esp_err_to_name(err));
        return false;
    }

    err = i2c_driver_install(port, I2C_MODE_MASTER, 0, 0, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C driver install failed: %s", esp_err_to_name(err));
        return false;
    }

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "SCANNING I2C PORT %d", port);
    ESP_LOGI(TAG, "========================================");

    found_addresses.clear();

    for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);

        err = i2c_master_cmd_begin(port, cmd, 100 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);

        if (err == ESP_OK) {
            found_addresses.push_back(addr);
            ESP_LOGI(TAG, "Found device at 0x%02X", addr);
        }
    }

    if (found_addresses.empty()) {
        ESP_LOGW(TAG, "NO I2C DEVICES FOUND!");
    } else {
        ESP_LOGI(TAG, "Found %zu device(s)", found_addresses.size());
    }

    ESP_LOGI(TAG, "========================================");

    i2c_driver_delete(port);
    return true;
}

bool I2CScanner::quickScan(i2c_port_t port,
                          gpio_num_t sda_pin,
                          gpio_num_t scl_pin) {
    std::vector<uint8_t> addresses;
    return scan(port, addresses, sda_pin, scl_pin);
}

bool I2CScanner::probe(i2c_port_t port,
                      uint8_t address,
                      gpio_num_t sda_pin,
                      gpio_num_t scl_pin) {

    if (address < 0x08 || address > 0x77) {
        return false;
    }

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sda_pin,
        .scl_io_num = scl_pin,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master = {.clk_speed = 100000},
        .clk_flags = 0,
    };

    esp_err_t err = i2c_param_config(port, &conf);
    if (err != ESP_OK) return false;

    err = i2c_driver_install(port, I2C_MODE_MASTER, 0, 0, 0);
    if (err != ESP_OK) return false;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, true);
    i2c_master_stop(cmd);

    err = i2c_master_cmd_begin(port, cmd, 100 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    i2c_driver_delete(port);

    return (err == ESP_OK);
}

void I2CScanner::printResults(const std::vector<uint8_t>& addresses) {
    if (addresses.empty()) {
        ESP_LOGI(TAG, "No I2C devices found");
        return;
    }

    ESP_LOGI(TAG, "I2C Devices found (%zu):", addresses.size());
    for (auto addr : addresses) {
        ESP_LOGI(TAG, "  0x%02X", addr);
    }
}
