#include "include/vl53l1.hpp"
#include <cstring>
#include <algorithm>
#include <vector>
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"

static const char* TAG = "VL53L1";

// ===== VL53L1 REGISTER ADDRESSES =====
// Based on VL53L1 datasheet - core registers
namespace Reg {
    constexpr uint8_t IDENTIFICATION__MODEL_ID = 0x010F;
    constexpr uint8_t VHV_CONFIG__TIMEOUT_MACROP = 0x00B2;
    constexpr uint8_t RANGE_CONFIG__VCSEL_PERIOD_A = 0x0060;
    constexpr uint8_t RANGE_CONFIG__VCSEL_PERIOD_B = 0x0063;
    constexpr uint8_t RANGE_CONFIG__TIMEOUT_MACROP_A = 0x005E;
    constexpr uint8_t RANGE_CONFIG__TIMEOUT_MACROP_B = 0x0061;
    constexpr uint8_t SYSTEM__INTERRUPT_CLEAR = 0x0086;
    constexpr uint8_t SYSTEM__MODE_START = 0x0087;
    constexpr uint8_t RESULT__RANGE_STATUS = 0x0089;
    constexpr uint8_t RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0 = 0x0096;
    constexpr uint8_t GPIO__TIO_HV_STATUS = 0x0031;
    constexpr uint8_t I2C_SLAVE__DEVICE_ADDRESS = 0x0001;
    constexpr uint8_t SYSTEM__INTERMEASUREMENT_PERIOD = 0x006C;
}

// ===== MODBUS REGISTER ADDRESSES (for UART control) =====
namespace ModbusReg {
    constexpr uint16_t DEVICE_ADDRESS = 0x0002;
    constexpr uint16_t BAUD_RATE = 0x0003;
    constexpr uint16_t RANGE_MODE = 0x0004;
    constexpr uint16_t CONTINUOUS_OUTPUT = 0x0005;
    constexpr uint16_t DISABLE_IIC = 0x0009;  // Write 1 to enable I2C mode
    constexpr uint16_t MEASUREMENT_RESULT = 0x0010;
}

// ===== MODBUS HELPER FUNCTIONS =====
namespace Modbus {
    // Calculate CRC-16/MODBUS
    uint16_t crc16(const uint8_t* data, size_t length) {
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

    // Build Modbus write command (function 0x06)
    size_t buildWriteCommand(uint8_t* buffer, uint8_t slave_addr, 
                            uint16_t reg_addr, uint16_t value) {
        buffer[0] = slave_addr;
        buffer[1] = 0x06;  // Function: Write Single Register
        buffer[2] = (reg_addr >> 8) & 0xFF;
        buffer[3] = reg_addr & 0xFF;
        buffer[4] = (value >> 8) & 0xFF;
        buffer[5] = value & 0xFF;
        
        uint16_t crc = crc16(buffer, 6);
        buffer[6] = crc & 0xFF;
        buffer[7] = (crc >> 8) & 0xFF;
        
        return 8;
    }

    // Build Modbus read command (function 0x03)
    size_t buildReadCommand(uint8_t* buffer, uint8_t slave_addr,
                           uint16_t reg_addr, uint16_t num_regs) {
        buffer[0] = slave_addr;
        buffer[1] = 0x03;  // Function: Read Holding Registers
        buffer[2] = (reg_addr >> 8) & 0xFF;
        buffer[3] = reg_addr & 0xFF;
        buffer[4] = (num_regs >> 8) & 0xFF;
        buffer[5] = num_regs & 0xFF;
        
        uint16_t crc = crc16(buffer, 6);
        buffer[6] = crc & 0xFF;
        buffer[7] = (crc >> 8) & 0xFF;
        
        return 8;
    }
}

// ===== IMPLEMENTATION CLASS =====
struct VL53L1::Impl {
    // Configuration
    VL53L1::Config config;

    // I2C master bus and device handles
    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t dev_handle;
    
    // UART handle
    bool uart_configured;

    // State
    bool initialized;
    bool i2c_owned;  // True if we created the I2C bus
    bool did_timeout;
    MeasurementMode current_mode;

    // Constructors
    Impl() : bus_handle(nullptr), dev_handle(nullptr), uart_configured(false),
             initialized(false), i2c_owned(false), did_timeout(false), 
             current_mode(MeasurementMode::SINGLE) {}

    // UART helper methods
    esp_err_t initUART();
    esp_err_t sendModbusCommand(const uint8_t* cmd, size_t cmd_len, 
                               uint8_t* response, size_t* resp_len, 
                               uint32_t timeout_ms);
    esp_err_t switchToI2CMode();
    esp_err_t setModbusRangingMode(RangingMode mode);
    void cleanupUART();

    // I2C helper methods
    esp_err_t writeReg8(uint16_t reg, uint8_t value);
    esp_err_t writeReg16(uint16_t reg, uint16_t value);
    esp_err_t writeReg32(uint16_t reg, uint32_t value);
    esp_err_t readReg8(uint16_t reg, uint8_t* value);
    esp_err_t readReg16(uint16_t reg, uint16_t* value);
    esp_err_t readReg32(uint16_t reg, uint32_t* value);
    esp_err_t writeMulti(uint16_t reg, const uint8_t* src, uint8_t count);
    esp_err_t readMulti(uint16_t reg, uint8_t* dst, uint8_t count);

    // Initialization helpers
    esp_err_t performI2CInit();
    esp_err_t configureRangingMode(RangingMode mode);
};

// ===== UART HELPER IMPLEMENTATIONS =====

esp_err_t VL53L1::Impl::initUART() {
    if (uart_configured) {
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initializing UART%d (TX: GPIO%d, RX: GPIO%d, Baud: %lu)",
             config.uart_port, config.uart_tx_pin, config.uart_rx_pin, 
             config.uart_baud_rate);

    uart_config_t uart_config = {
        .baud_rate = (int)config.uart_baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t err = uart_param_config(config.uart_port, &uart_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "UART param config failed: %s", esp_err_to_name(err));
        return err;
    }

    err = uart_set_pin(config.uart_port, config.uart_tx_pin, config.uart_rx_pin,
                       UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "UART set pin failed: %s", esp_err_to_name(err));
        return err;
    }

    // Install UART driver (using 1KB for RX and TX buffers)
    err = uart_driver_install(config.uart_port, 1024, 1024, 0, NULL, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "UART driver install failed: %s", esp_err_to_name(err));
        return err;
    }

    uart_configured = true;
    ESP_LOGI(TAG, "UART configured successfully");
    return ESP_OK;
}

esp_err_t VL53L1::Impl::sendModbusCommand(const uint8_t* cmd, size_t cmd_len,
                                         uint8_t* response, size_t* resp_len,
                                         uint32_t timeout_ms) {
    if (!uart_configured) {
        ESP_LOGE(TAG, "UART not configured");
        return ESP_FAIL;
    }

    // Flush RX buffer
    uart_flush_input(config.uart_port);

    // Send command
    int written = uart_write_bytes(config.uart_port, cmd, cmd_len);
    if (written != cmd_len) {
        ESP_LOGE(TAG, "UART write failed: wrote %d/%zu bytes", written, cmd_len);
        return ESP_FAIL;
    }

    ESP_LOGD(TAG, "Sent Modbus command (%zu bytes)", cmd_len);

    // Wait for response
    if (response && resp_len) {
        int len = uart_read_bytes(config.uart_port, response, *resp_len,
                                 pdMS_TO_TICKS(timeout_ms));
        if (len < 0) {
            ESP_LOGW(TAG, "UART read failed");
            *resp_len = 0;
            return ESP_ERR_TIMEOUT;
        }

        *resp_len = len;
        ESP_LOGD(TAG, "Received response (%d bytes)", len);

        // Verify CRC if response is long enough
        if (len >= 3) {
            uint16_t received_crc = response[len-2] | (response[len-1] << 8);
            uint16_t calculated_crc = Modbus::crc16(response, len - 2);
            
            if (received_crc != calculated_crc) {
                ESP_LOGW(TAG, "CRC mismatch: received 0x%04X, calculated 0x%04X",
                        received_crc, calculated_crc);
                return ESP_ERR_INVALID_CRC;
            }
        }
    }

    return ESP_OK;
}

esp_err_t VL53L1::Impl::switchToI2CMode() {
    ESP_LOGI(TAG, "Switching TOF400F module to I2C mode via UART...");

    // Build Modbus command: Write 1 to register 0x0009 (DISABLE_IIC)
    // Note: Register name is misleading - writing 1 actually ENABLES I2C mode
    uint8_t cmd[8];
    size_t cmd_len = Modbus::buildWriteCommand(cmd, 0x01,  // Slave address 1
                                               ModbusReg::DISABLE_IIC, 0x0001);

    uint8_t response[8];
    size_t resp_len = sizeof(response);

    esp_err_t err = sendModbusCommand(cmd, cmd_len, response, &resp_len, 500);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send I2C mode switch command: %s", 
                esp_err_to_name(err));
        return err;
    }

    // Verify response (should echo the command)
    if (resp_len == 8 && memcmp(cmd, response, 8) == 0) {
        ESP_LOGI(TAG, "Successfully switched to I2C mode");
        return ESP_OK;
    } else {
        ESP_LOGW(TAG, "I2C mode switch response unexpected (len=%zu)", resp_len);
        return ESP_FAIL;
    }
}

esp_err_t VL53L1::Impl::setModbusRangingMode(RangingMode mode) {
    ESP_LOGI(TAG, "Setting ranging mode via UART: %s",
             mode == RangingMode::HIGH_PRECISION ? "High Precision" : "Long Distance");

    uint8_t cmd[8];
    uint16_t mode_value = (mode == RangingMode::HIGH_PRECISION) ? 0x0000 : 0x0001;
    
    size_t cmd_len = Modbus::buildWriteCommand(cmd, 0x01,  // Slave address 1
                                               ModbusReg::RANGE_MODE, mode_value);

    uint8_t response[8];
    size_t resp_len = sizeof(response);

    esp_err_t err = sendModbusCommand(cmd, cmd_len, response, &resp_len, 500);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set ranging mode: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "Ranging mode set successfully");
    return ESP_OK;
}

void VL53L1::Impl::cleanupUART() {
    if (uart_configured) {
        uart_driver_delete(config.uart_port);
        uart_configured = false;
        ESP_LOGD(TAG, "UART cleaned up");
    }
}

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

// ===== INITIALIZATION HELPERS =====

esp_err_t VL53L1::Impl::performI2CInit() {
    ESP_LOGI(TAG, "Performing VL53L1 I2C initialization...");

    // VL53L1 has a more complex initialization sequence than VL53L0X
    // For the TOF400F module, the firmware handles most of this
    // We just need to verify communication

    // Read and verify model ID (VL53L1 = 0xEA)
    uint8_t model_id;
    esp_err_t err = readReg8(Reg::IDENTIFICATION__MODEL_ID, &model_id);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read model ID: %s", esp_err_to_name(err));
        return err;
    }

    if (model_id != 0xEA) {
        ESP_LOGE(TAG, "Model ID mismatch! Expected 0xEA (VL53L1), got 0x%02X", 
                model_id);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "VL53L1 model ID verified: 0x%02X", model_id);
    
    // The TOF400F module firmware handles ranging mode configuration
    // No additional I2C initialization needed
    
    return ESP_OK;
}

esp_err_t VL53L1::Impl::configureRangingMode(RangingMode mode) {
    // For TOF400F, ranging mode is configured via UART/Modbus
    // This is a no-op in I2C mode
    ESP_LOGD(TAG, "Ranging mode is pre-configured by module firmware");
    return ESP_OK;
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
    config.i2c_freq_hz = 400000;  // 400kHz
    
    // UART configuration
    config.uart_port = uart_port;
    config.uart_tx_pin = uart_tx_pin;
    config.uart_rx_pin = uart_rx_pin;
    config.uart_baud_rate = 115200;  // Default from datasheet
    
    // Sensor configuration
    config.ranging_mode = VL53L1::RangingMode::HIGH_PRECISION;
    config.timeout_ms = 500;
    config.enable_i2c_mode = true;
    
    return config;
}

VL53L1::VL53L1(const Config& config) : pimpl_(std::make_unique<Impl>()) {
    pimpl_->config = config;
}

VL53L1::~VL53L1() {
    if (pimpl_) {
        pimpl_->cleanupUART();
        
        if (pimpl_->dev_handle) {
            i2c_master_bus_rm_device(pimpl_->dev_handle);
        }
        if (pimpl_->i2c_owned && pimpl_->bus_handle) {
            i2c_del_master_bus(pimpl_->bus_handle);
        }
    }
}

VL53L1::VL53L1(VL53L1&& other) noexcept : pimpl_(std::move(other.pimpl_)) {}

VL53L1& VL53L1::operator=(VL53L1&& other) noexcept {
    pimpl_ = std::move(other.pimpl_);
    return *this;
}

const char* VL53L1::init() {
    if (!pimpl_) {
        return "Implementation not initialized";
    }

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "VL53L1 (TOF400F) Initialization Starting");
    ESP_LOGI(TAG, "========================================");

    // Step 1: Handle XSHUT pin if configured
    if (pimpl_->config.xshut_pin != GPIO_NUM_NC) {
        ESP_LOGD(TAG, "Step 1: Configuring XSHUT pin (GPIO%d)", 
                pimpl_->config.xshut_pin);
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
            // Reset sensor via XSHUT
            gpio_set_level(pimpl_->config.xshut_pin, 0);
            vTaskDelay(pdMS_TO_TICKS(10));
            gpio_set_level(pimpl_->config.xshut_pin, 1);
            vTaskDelay(pdMS_TO_TICKS(10));
            ESP_LOGI(TAG, "Step 1: Sensor reset via XSHUT - OK");
        }
    } else {
        ESP_LOGD(TAG, "Step 1: XSHUT pin not configured - skipping reset");
    }

    // Step 2: Configure UART if needed
    if (pimpl_->config.enable_i2c_mode) {
        ESP_LOGD(TAG, "Step 2: Initializing UART for mode switch...");
        esp_err_t err = pimpl_->initUART();
        if (err != ESP_OK) {
            return "UART initialization failed";
        }
        ESP_LOGI(TAG, "Step 2: UART initialized - OK");

        // Small delay for module to be ready
        vTaskDelay(pdMS_TO_TICKS(100));

        // Step 3: Set ranging mode via UART
        ESP_LOGD(TAG, "Step 3: Configuring ranging mode via UART...");
        err = pimpl_->setModbusRangingMode(pimpl_->config.ranging_mode);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Failed to set ranging mode (continuing anyway)");
        } else {
            ESP_LOGI(TAG, "Step 3: Ranging mode configured - OK");
        }

        // Step 4: Switch to I2C mode
        ESP_LOGD(TAG, "Step 4: Switching to I2C mode...");
        err = pimpl_->switchToI2CMode();
        if (err != ESP_OK) {
            return "Failed to switch to I2C mode";
        }
        ESP_LOGI(TAG, "Step 4: Switched to I2C mode - OK");

        // Cleanup UART as it's no longer needed
        pimpl_->cleanupUART();

        // Wait for module to complete mode switch
        vTaskDelay(pdMS_TO_TICKS(200));
    } else {
        ESP_LOGD(TAG, "Steps 2-4: I2C mode already enabled - skipping UART setup");
    }

    // Step 5: Initialize I2C bus
    ESP_LOGD(TAG, "Step 5: Creating I2C bus...");
    ESP_LOGD(TAG, "  Port: I2C_%d", pimpl_->config.i2c_port);
    ESP_LOGD(TAG, "  SDA: GPIO%d, SCL: GPIO%d", 
            pimpl_->config.sda_pin, pimpl_->config.scl_pin);
    ESP_LOGD(TAG, "  Frequency: %lu Hz", pimpl_->config.i2c_freq_hz);

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
    ESP_LOGI(TAG, "Step 5: I2C bus created - OK");

    // Step 6: Add device to bus
    ESP_LOGD(TAG, "Step 6: Adding device at address 0x%02X to bus...", 
            pimpl_->config.i2c_address);
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
    ESP_LOGI(TAG, "Step 6: Device added to bus - OK");

    // Step 7: Verify device presence
    ESP_LOGI(TAG, "Step 7: Verifying device presence...");
    ESP_LOGI(TAG, "  Expected address: 0x%02X (VL53L1 in TOF400F module)", 
            pimpl_->config.i2c_address);

    if (!I2CScanner::probeWithBus(pimpl_->bus_handle, pimpl_->config.i2c_address,
                                 pimpl_->config.i2c_freq_hz)) {
        ESP_LOGE(TAG, "FAILED: Device not responding at address 0x%02X", 
                pimpl_->config.i2c_address);
        ESP_LOGE(TAG, "Possible causes:");
        ESP_LOGE(TAG, "  - Module still in UART mode (enable_i2c_mode=false?)");
        ESP_LOGE(TAG, "  - Wrong I2C address configured");
        ESP_LOGE(TAG, "  - Hardware connection issue");
        ESP_LOGE(TAG, "Running full I2C bus scan for diagnostics...");

        std::vector<uint8_t> found_addresses;
        I2CScanner::scan(pimpl_->config.i2c_port, found_addresses,
                        pimpl_->config.sda_pin, pimpl_->config.scl_pin,
                        100000);

        return "Device not responding at expected address";
    }
    ESP_LOGI(TAG, "Step 7: Device verified at address 0x%02X - OK", 
            pimpl_->config.i2c_address);

    // Step 8: Perform I2C initialization
    ESP_LOGD(TAG, "Step 8: Performing VL53L1 I2C initialization...");
    err = pimpl_->performI2CInit();
    if (err != ESP_OK) {
        return "VL53L1 I2C initialization failed";
    }
    ESP_LOGI(TAG, "Step 8: VL53L1 initialized - OK");

    pimpl_->initialized = true;
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "VL53L1 Initialization Complete - SUCCESS");
    ESP_LOGI(TAG, "Ranging mode: %s", 
            pimpl_->config.ranging_mode == RangingMode::HIGH_PRECISION 
            ? "High Precision (30ms, 1.3m)" : "Long Distance (200ms, 4.0m)");
    ESP_LOGI(TAG, "========================================");
    
    return nullptr;
}

bool VL53L1::isReady() const {
    return pimpl_ && pimpl_->initialized;
}

// ===== MEASUREMENT IMPLEMENTATIONS =====

bool VL53L1::readSingle(MeasurementResult& result) {
    if (!isReady()) {
        return false;
    }

    result.timestamp_us = esp_timer_get_time();
    result.timeout_occurred = false;

    // Start single ranging measurement
    esp_err_t err = pimpl_->writeReg8(Reg::SYSTEM__MODE_START, 0x10);
    if (err != ESP_OK) {
        return false;
    }

    // Poll for measurement ready
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

    // Read range status
    uint8_t range_status;
    err = pimpl_->readReg8(Reg::RESULT__RANGE_STATUS, &range_status);
    if (err != ESP_OK) {
        result.valid = false;
        return false;
    }

    // Read distance
    uint16_t distance_mm;
    err = pimpl_->readReg16(Reg::RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0,
                           &distance_mm);
    if (err != ESP_OK) {
        result.valid = false;
        return false;
    }

    // Clear interrupt
    pimpl_->writeReg8(Reg::SYSTEM__INTERRUPT_CLEAR, 0x01);

    result.distance_mm = distance_mm;
    result.range_status = range_status & 0x1F;
    // VL53L1 range status: 0 = valid, others indicate various error conditions
    result.valid = (result.range_status == 0) && (distance_mm < 8190);

    return true;
}

bool VL53L1::startContinuous() {
    if (!isReady()) {
        return false;
    }

    // Start continuous ranging
    esp_err_t err = pimpl_->writeReg8(Reg::SYSTEM__MODE_START, 0x40);
    if (err != ESP_OK) {
        return false;
    }

    pimpl_->current_mode = MeasurementMode::CONTINUOUS;
    ESP_LOGI(TAG, "Started continuous measurement mode");
    return true;
}

bool VL53L1::readContinuous(MeasurementResult& result) {
    if (!isReady()) {
        return false;
    }

    result.timestamp_us = esp_timer_get_time();
    result.timeout_occurred = false;

    // Wait for data ready
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

    // Read range status
    uint8_t range_status;
    esp_err_t err = pimpl_->readReg8(Reg::RESULT__RANGE_STATUS, &range_status);
    if (err != ESP_OK) {
        result.valid = false;
        return false;
    }

    // Read distance
    uint16_t distance_mm;
    err = pimpl_->readReg16(Reg::RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0,
                           &distance_mm);
    if (err != ESP_OK) {
        result.valid = false;
        return false;
    }

    // Clear interrupt
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

    // Stop ranging
    esp_err_t err = pimpl_->writeReg8(Reg::SYSTEM__MODE_START, 0x00);
    if (err != ESP_OK) {
        return false;
    }

    pimpl_->current_mode = MeasurementMode::SINGLE;
    ESP_LOGI(TAG, "Stopped continuous measurement mode");
    return true;
}

// ===== CONFIGURATION IMPLEMENTATIONS =====

const char* VL53L1::setRangingMode(RangingMode mode) {
    if (!isReady()) {
        return "Sensor not initialized";
    }

    // For TOF400F, ranging mode must be set via UART before switching to I2C
    // Cannot change mode after initialization
    ESP_LOGW(TAG, "Ranging mode can only be set during initialization");
    return "Mode change requires re-initialization";
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

    // VL53L1 uses 7-bit addressing
    esp_err_t err = pimpl_->writeReg8(Reg::I2C_SLAVE__DEVICE_ADDRESS, 
                                     new_addr & 0x7F);
    if (err == ESP_OK) {
        pimpl_->config.i2c_address = new_addr;

        // Recreate device handle with new address
        i2c_master_bus_rm_device(pimpl_->dev_handle);

        i2c_device_config_t dev_config = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = new_addr,
            .scl_speed_hz = pimpl_->config.i2c_freq_hz,
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

    // Verify we can still read model ID
    uint8_t model_id;
    esp_err_t err = pimpl_->readReg8(Reg::IDENTIFICATION__MODEL_ID, &model_id);
    if (err != ESP_OK) {
        return "Failed to read model ID";
    }

    if (model_id != 0xEA) {
        return "Model ID check failed";
    }

    // Perform a test measurement
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
