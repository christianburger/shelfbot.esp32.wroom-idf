#pragma once

#include <cstdint>
#include <vector>
#include <functional>
#include "driver/uart.h"
#include "driver/gpio.h"  // ADDED: for gpio_num_t
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

class DuartModbus {
public:
    struct Config {
        uart_port_t uart_port;
        gpio_num_t tx_pin;
        gpio_num_t rx_pin;
        uint32_t baud_rate;
        uint8_t parity;           // UART_PARITY_DISABLE, UART_PARITY_EVEN, UART_PARITY_ODD
        uint8_t stop_bits;        // UART_STOP_BITS_1, UART_STOP_BITS_2
        uint8_t data_bits;        // UART_DATA_8_BITS
        uint32_t timeout_ms;
        uint8_t max_retries;
    };

    struct ModbusRequest {
        uint8_t slave_address;
        uint8_t function_code;
        uint16_t register_address;
        uint16_t value;
        uint8_t expected_response_len;
    };

    struct ModbusResponse {
        bool success;
        uint8_t slave_address;
        uint8_t function_code;
        std::vector<uint8_t> data;
        uint8_t error_code;
        esp_err_t esp_error;
    };

    DuartModbus(const Config& config);
    ~DuartModbus();

    const char* init();
    bool isReady() const { return initialized_; }

    ModbusResponse readHoldingRegisters(uint8_t slave_address,
                                       uint16_t start_register,
                                       uint16_t num_registers,
                                       uint32_t timeout_ms = 500);

    ModbusResponse writeSingleRegister(uint8_t slave_address,
                                      uint16_t register_address,
                                      uint16_t value,
                                      bool verify = true,
                                      uint32_t timeout_ms = 500);

    ModbusResponse writeSingleRegisterWithVerification(
        uint8_t slave_address,
        uint16_t register_address,
        uint16_t value,
        std::function<bool(uint16_t)> verification_func,
        uint32_t timeout_ms = 500);

    bool verifyRegisterValue(uint8_t slave_address,
                            uint16_t register_address,
                            uint16_t expected_value,
                            uint32_t timeout_ms = 500);

    uart_port_t getUartPort() const { return config_.uart_port; }
    void flushBuffers();

private:
    Config config_;
    bool initialized_;
    static const char* TAG;

    uint16_t calculateCRC(const uint8_t* data, size_t length);
    std::vector<uint8_t> buildModbusFrame(uint8_t slave_address,
                                         uint8_t function_code,
                                         uint16_t register_address,
                                         uint16_t value = 0,
                                         uint16_t count = 0);
    ModbusResponse sendRequest(const ModbusRequest& request, uint32_t timeout_ms);
    bool parseModbusResponse(const std::vector<uint8_t>& response,
                            ModbusResponse& parsed_response,
                            uint8_t expected_slave_address);
    bool validateModbusResponse(const std::vector<uint8_t>& frame);

    esp_err_t sendData(const uint8_t* data, size_t length);
    esp_err_t receiveData(std::vector<uint8_t>& buffer, size_t expected_len, uint32_t timeout_ms);
    esp_err_t receiveDataWithTimeout(std::vector<uint8_t>& buffer, uint32_t timeout_ms);
};
