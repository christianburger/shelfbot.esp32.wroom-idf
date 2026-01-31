#pragma once

#include <cstdint>
#include <memory>
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/uart.h"
#include "esp_err.h"

class VL53L1_Modbus {
public:
    enum class RangingMode : uint8_t {
        HIGH_PRECISION = 0,  // ~1.3m range, 30ms period (Mode 0)
        LONG_DISTANCE = 1    // ~4.0m range, 200ms period (Mode 1)
    };

    struct Config {
        // I2C configuration for VL53L1 chip
        i2c_port_t i2c_port = I2C_NUM_0;
        gpio_num_t sda_pin = GPIO_NUM_21;
        gpio_num_t scl_pin = GPIO_NUM_22;
        uint8_t i2c_address = 0x29;
        uint32_t i2c_freq_hz = 400000;

        // UART/Modbus configuration for TOF400F module
        uart_port_t uart_port = UART_NUM_1;
        gpio_num_t uart_tx_pin = GPIO_NUM_17;
        gpio_num_t uart_rx_pin = GPIO_NUM_16;
        uint32_t uart_baud_rate = 115200;

        // TOF400F module configuration
        uint8_t modbus_slave_address = 0x01;
        RangingMode ranging_mode = RangingMode::LONG_DISTANCE;
        uint16_t timeout_ms = 500;
        bool enable_continuous = true;
    };

    struct Measurement {
        uint16_t distance_mm;
        bool valid;
        uint8_t range_status;
        int64_t timestamp_us;

        Measurement() : distance_mm(0), valid(false), range_status(0), timestamp_us(0) {}
    };

    explicit VL53L1_Modbus(const Config& config);
    ~VL53L1_Modbus();

    VL53L1_Modbus(const VL53L1_Modbus&) = delete;
    VL53L1_Modbus& operator=(const VL53L1_Modbus&) = delete;

    VL53L1_Modbus(VL53L1_Modbus&& other) noexcept;
    VL53L1_Modbus& operator=(VL53L1_Modbus&& other) noexcept;

    const char* init();
    bool isReady() const;

    bool readSingle(Measurement& result);
    bool startContinuous();
    bool stopContinuous();
    bool readContinuous(Measurement& result);

    const char* setRangingMode(RangingMode mode);
    RangingMode getRangingMode() const;

    void setTimeout(uint16_t timeout_ms);
    uint16_t getTimeout() const;

    bool setAddress(uint8_t new_addr);
    uint8_t getAddress() const;

    bool probe();
    const char* selfTest();
    bool timeoutOccurred();

    bool loadFactoryCalibration();
    bool setAutoOutput(bool enable, uint16_t interval_ms = 0);
    bool rebootModule();

private:
    struct Impl;
    std::unique_ptr<Impl> pimpl_;
};

VL53L1_Modbus::Config vl53l1_modbus_default_config(
    i2c_port_t i2c_port = I2C_NUM_0,
    gpio_num_t sda_pin = GPIO_NUM_21,
    gpio_num_t scl_pin = GPIO_NUM_22,
    uart_port_t uart_port = UART_NUM_1,
    gpio_num_t uart_tx_pin = GPIO_NUM_17,
    gpio_num_t uart_rx_pin = GPIO_NUM_16
);