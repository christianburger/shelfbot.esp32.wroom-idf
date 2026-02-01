#include "duart_modbus.hpp"
#include "include/duart_modbus.hpp"

#include <algorithm>
#include <string>
#include "esp_timer.h"

const char* DuartModbus::TAG = "DUART_MODBUS";

DuartModbus::DuartModbus(const Config& config)
    : config_(config), initialized_(false) {}

DuartModbus::~DuartModbus() {
    if (initialized_) {
        uart_driver_delete(config_.uart_port);
        initialized_ = false;
        ESP_LOGD(TAG, "UART%d driver deleted", config_.uart_port);
    }
}

const char* DuartModbus::init() {
    if (initialized_) {
        return nullptr;
    }

    ESP_LOGI(TAG, "Initializing DUART_MODBUS on UART%d", config_.uart_port);

    // Convert uint8_t to proper ESP-IDF enum types
    uart_word_length_t data_bits;
    switch (config_.data_bits) {
        case 5: data_bits = UART_DATA_5_BITS; break;
        case 6: data_bits = UART_DATA_6_BITS; break;
        case 7: data_bits = UART_DATA_7_BITS; break;
        case 8: data_bits = UART_DATA_8_BITS; break;
        default: data_bits = UART_DATA_8_BITS; break;
    }

    uart_parity_t parity;
    switch (config_.parity) {
        case 0: parity = UART_PARITY_DISABLE; break;
        case 1: parity = UART_PARITY_ODD; break;
        case 2: parity = UART_PARITY_EVEN; break;
        default: parity = UART_PARITY_DISABLE; break;
    }

    uart_stop_bits_t stop_bits;
    switch (config_.stop_bits) {
        case 1: stop_bits = UART_STOP_BITS_1; break;
        case 2: stop_bits = UART_STOP_BITS_2; break;
        case 3: stop_bits = UART_STOP_BITS_1_5; break;
        default: stop_bits = UART_STOP_BITS_1; break;
    }

    uart_config_t uart_config = {
        .baud_rate = (int)config_.baud_rate,
        .data_bits = data_bits,
        .parity = parity,
        .stop_bits = stop_bits,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_DEFAULT,
        .flags = {0}
    };

    esp_err_t err = uart_param_config(config_.uart_port, &uart_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "UART parameter config failed: %s", esp_err_to_name(err));
        return "UART parameter config failed";
    }

    err = uart_set_pin(config_.uart_port, config_.tx_pin, config_.rx_pin,
                      UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "UART pin config failed: %s", esp_err_to_name(err));
        return "UART pin config failed";
    }

    err = uart_driver_install(config_.uart_port, 1024, 1024, 0, NULL, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "UART driver install failed: %s", esp_err_to_name(err));
        return "UART driver install failed";
    }

    initialized_ = true;
    ESP_LOGI(TAG, "DUART_MODBUS initialized successfully");
    return nullptr;
}

uint16_t DuartModbus::calculateCRC(const uint8_t* data, size_t length) {
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

std::vector<uint8_t> DuartModbus::buildModbusFrame(uint8_t slave_address,
                                                  uint8_t function_code,
                                                  uint16_t register_address,
                                                  uint16_t value,
                                                  uint16_t count) {
    std::vector<uint8_t> frame;

    frame.push_back(slave_address);
    frame.push_back(function_code);

    frame.push_back((register_address >> 8) & 0xFF);
    frame.push_back(register_address & 0xFF);

    if (function_code == 0x03 || function_code == 0x04) {
        frame.push_back((count >> 8) & 0xFF);
        frame.push_back(count & 0xFF);
    } else if (function_code == 0x06) {
        frame.push_back((value >> 8) & 0xFF);
        frame.push_back(value & 0xFF);
    }

    uint16_t crc = calculateCRC(frame.data(), frame.size());
    frame.push_back(crc & 0xFF);        // LSB first (Modbus RTU standard)
    frame.push_back((crc >> 8) & 0xFF); // MSB second

    return frame;
}

bool DuartModbus::validateModbusResponse(const std::vector<uint8_t>& frame) {
    if (frame.size() < 5) {
        ESP_LOGW(TAG, "Frame too short: %zu bytes", frame.size());
        return false;
    }

    // Log the raw frame bytes
    std::string hex_str;
    for (size_t i = 0; i < frame.size(); i++) {
        char buf[8];
        snprintf(buf, sizeof(buf), "%02X ", frame[i]);
        hex_str += buf;
    }
    ESP_LOGI(TAG, "Raw RX frame (%zu bytes): %s", frame.size(), hex_str.c_str());

    // Modbus RTU CRC is little-endian (LSB first, MSB second)
    uint16_t received_crc = frame[frame.size() - 2] | (frame[frame.size() - 1] << 8);

    // Calculate CRC on all bytes except the last 2 (which are the CRC itself)
    uint16_t calculated_crc = calculateCRC(frame.data(), frame.size() - 2);

    ESP_LOGI(TAG, "CRC check:");
    ESP_LOGI(TAG, "  Last 2 bytes: [%zu]=%02X [%zu]=%02X",
             frame.size()-2, frame[frame.size()-2],
             frame.size()-1, frame[frame.size()-1]);
    ESP_LOGI(TAG, "  Received CRC (LSB|MSB<<8): 0x%04X", received_crc);
    ESP_LOGI(TAG, "  Calculated CRC: 0x%04X", calculated_crc);
    ESP_LOGI(TAG, "  Match: %s", (received_crc == calculated_crc) ? "YES" : "NO");

    if (received_crc != calculated_crc) {
        ESP_LOGW(TAG, "CRC MISMATCH!");
        return false;
    }

    ESP_LOGI(TAG, "CRC validation PASSED");
    return true;
}

bool DuartModbus::parseModbusResponse(const std::vector<uint8_t>& response,
                                     ModbusResponse& parsed_response,
                                     uint8_t expected_slave_address) {
    parsed_response.success = false;

    if (response.size() < 5) {
        ESP_LOGW(TAG, "Response too short: %zu bytes", response.size());
        return false;
    }

    if (!validateModbusResponse(response)) {
        return false;
    }

    parsed_response.slave_address = response[0];
    if (parsed_response.slave_address != expected_slave_address) {
        ESP_LOGW(TAG, "Slave address mismatch: expected 0x%02X, got 0x%02X",
                expected_slave_address, parsed_response.slave_address);
        return false;
    }

    parsed_response.function_code = response[1];

    if (parsed_response.function_code & 0x80) {
        parsed_response.error_code = response[2];
        ESP_LOGE(TAG, "MODBUS exception: function 0x%02X, error 0x%02X",
                parsed_response.function_code & 0x7F, parsed_response.error_code);
        return false;
    }

    if (parsed_response.function_code == 0x03 || parsed_response.function_code == 0x04) {
        uint8_t byte_count = response[2];
        if (response.size() != (3 + byte_count + 2)) {
            ESP_LOGW(TAG, "Invalid byte count: %d, frame size: %zu",
                    byte_count, response.size());
            return false;
        }

        parsed_response.data.assign(response.begin() + 3, response.begin() + 3 + byte_count);

    } else if (parsed_response.function_code == 0x06) {
        if (response.size() != 8) {
            ESP_LOGW(TAG, "Invalid write response size: %zu", response.size());
            return false;
        }

        parsed_response.data.push_back(response[4]);
        parsed_response.data.push_back(response[5]);
    }

    parsed_response.success = true;
    return true;
}

esp_err_t DuartModbus::sendData(const uint8_t* data, size_t length) {
    if (!initialized_) {
        return ESP_ERR_INVALID_STATE;
    }

    flushBuffers();

    // Log what we're sending
    std::string hex_str;
    for (size_t i = 0; i < length; i++) {
        char buf[8];
        snprintf(buf, sizeof(buf), "%02X ", data[i]);
        hex_str += buf;
    }
    ESP_LOGI(TAG, "TX (%zu bytes): %s", length, hex_str.c_str());

    int written = uart_write_bytes(config_.uart_port, data, length);
    if (written != length) {
        ESP_LOGE(TAG, "UART write failed: wrote %d/%zu bytes", written, length);
        return ESP_FAIL;
    }

    return uart_wait_tx_done(config_.uart_port, pdMS_TO_TICKS(config_.timeout_ms));
}

esp_err_t DuartModbus::receiveDataWithTimeout(std::vector<uint8_t>& buffer, uint32_t timeout_ms) {
    if (!initialized_) {
        return ESP_ERR_INVALID_STATE;
    }

    buffer.clear();

    int64_t start_time = esp_timer_get_time();
    uint32_t elapsed_ms = 0;

    while (elapsed_ms < timeout_ms) {
        size_t available = 0;
        esp_err_t err = uart_get_buffered_data_len(config_.uart_port, &available);
        if (err != ESP_OK) {
            return err;
        }

        if (available > 0) {
            size_t old_size = buffer.size();
            buffer.resize(old_size + available);

            int read = uart_read_bytes(config_.uart_port,
                                      buffer.data() + old_size,
                                      available,
                                      pdMS_TO_TICKS(timeout_ms - elapsed_ms));

            if (read < 0) {
                return ESP_ERR_TIMEOUT;
            }

            buffer.resize(old_size + read);

            if (read < available) {
                break;
            }
        }

        vTaskDelay(1);
        elapsed_ms = (esp_timer_get_time() - start_time) / 1000;
    }

    return buffer.empty() ? ESP_ERR_TIMEOUT : ESP_OK;
}

esp_err_t DuartModbus::receiveData(std::vector<uint8_t>& buffer, size_t expected_len, uint32_t timeout_ms) {
    if (!initialized_) {
        return ESP_ERR_INVALID_STATE;
    }

    buffer.resize(expected_len);

    int read = uart_read_bytes(config_.uart_port, buffer.data(), expected_len,
                              pdMS_TO_TICKS(timeout_ms));

    if (read != expected_len) {
        ESP_LOGW(TAG, "Read %d bytes, expected %zu", read, expected_len);
        return ESP_ERR_TIMEOUT;
    }

    buffer.resize(read);
    return ESP_OK;
}

DuartModbus::ModbusResponse DuartModbus::sendRequest(const ModbusRequest& request,
                                                    uint32_t timeout_ms) {
    ModbusResponse response;
    response.esp_error = ESP_OK;

    if (!initialized_) {
        response.esp_error = ESP_ERR_INVALID_STATE;
        return response;
    }

    std::vector<uint8_t> request_frame;
    if (request.function_code == 0x03) {
        request_frame = buildModbusFrame(request.slave_address, request.function_code,
                                        request.register_address, 0, request.value);
    } else if (request.function_code == 0x06) {
        request_frame = buildModbusFrame(request.slave_address, request.function_code,
                                        request.register_address, request.value);
    } else {
        response.esp_error = ESP_ERR_NOT_SUPPORTED;
        return response;
    }

    ESP_LOGI(TAG, "Modbus request: slave=0x%02X func=0x%02X reg=0x%04X",
            request.slave_address, request.function_code, request.register_address);

    response.esp_error = sendData(request_frame.data(), request_frame.size());
    if (response.esp_error != ESP_OK) {
        ESP_LOGE(TAG, "TX failed: %s", esp_err_to_name(response.esp_error));
        return response;
    }

    std::vector<uint8_t> response_frame;
    if (request.expected_response_len > 0) {
        response.esp_error = receiveData(response_frame, request.expected_response_len, timeout_ms);
    } else {
        response.esp_error = receiveDataWithTimeout(response_frame, timeout_ms);
    }

    if (response.esp_error != ESP_OK) {
        ESP_LOGE(TAG, "RX failed: %s", esp_err_to_name(response.esp_error));
        return response;
    }

    ESP_LOGI(TAG, "Received %zu bytes", response_frame.size());

    if (!parseModbusResponse(response_frame, response, request.slave_address)) {
        response.success = false;
        response.esp_error = ESP_ERR_INVALID_RESPONSE;
    }

    return response;
}

DuartModbus::ModbusResponse DuartModbus::readHoldingRegisters(uint8_t slave_address,
                                                             uint16_t start_register,
                                                             uint16_t num_registers,
                                                             uint32_t timeout_ms) {
    ModbusRequest request = {
        .slave_address = slave_address,
        .function_code = 0x03,
        .register_address = start_register,
        .value = num_registers,
        .expected_response_len = 0  // Use dynamic receive
    };

    return sendRequest(request, timeout_ms);
}

DuartModbus::ModbusResponse DuartModbus::writeSingleRegister(uint8_t slave_address,
                                                            uint16_t register_address,
                                                            uint16_t value,
                                                            bool verify,
                                                            uint32_t timeout_ms) {
    ModbusRequest write_request = {
        .slave_address = slave_address,
        .function_code = 0x06,
        .register_address = register_address,
        .value = value,
        .expected_response_len = 8
    };

    ModbusResponse response = sendRequest(write_request, timeout_ms);

    if (!response.success || !verify) {
        return response;
    }

    vTaskDelay(pdMS_TO_TICKS(50));

    auto verification_func = [value](uint16_t read_value) {
        return read_value == value;
    };

    return writeSingleRegisterWithVerification(slave_address, register_address,
                                              value, verification_func, timeout_ms);
}

DuartModbus::ModbusResponse DuartModbus::writeSingleRegisterWithVerification(
    uint8_t slave_address,
    uint16_t register_address,
    uint16_t value,
    std::function<bool(uint16_t)> verification_func,
    uint32_t timeout_ms) {

    ModbusResponse final_response;
    final_response.success = false;

    for (uint8_t retry = 0; retry <= config_.max_retries; retry++) {
        if (retry > 0) {
            ESP_LOGW(TAG, "Write verification failed, retry %d/%d",
                    retry, config_.max_retries);
            vTaskDelay(pdMS_TO_TICKS(100 * retry));
        }

        auto write_response = writeSingleRegister(slave_address, register_address,
                                                 value, false, timeout_ms);

        if (!write_response.success) {
            final_response = write_response;
            continue;
        }

        vTaskDelay(pdMS_TO_TICKS(50));

        auto read_response = readHoldingRegisters(slave_address, register_address, 1, timeout_ms);

        if (!read_response.success) {
            final_response = read_response;
            continue;
        }

        if (read_response.data.size() >= 2) {
            uint16_t read_value = (read_response.data[0] << 8) | read_response.data[1];

            ESP_LOGD(TAG, "Write verification: wrote 0x%04X, read 0x%04X",
                    value, read_value);

            if (verification_func(read_value)) {
                final_response = write_response;
                final_response.success = true;
                final_response.data.push_back((read_value >> 8) & 0xFF);
                final_response.data.push_back(read_value & 0xFF);
                break;
            } else {
                ESP_LOGW(TAG, "Verification failed: expected condition not met");
                final_response.esp_error = ESP_ERR_INVALID_RESPONSE;
            }
        }
    }

    return final_response;
}

bool DuartModbus::verifyRegisterValue(uint8_t slave_address,
                                     uint16_t register_address,
                                     uint16_t expected_value,
                                     uint32_t timeout_ms) {
    auto response = readHoldingRegisters(slave_address, register_address, 1, timeout_ms);

    if (!response.success || response.data.size() < 2) {
        return false;
    }

    uint16_t read_value = (response.data[0] << 8) | response.data[1];
    bool matches = (read_value == expected_value);

    if (!matches) {
        ESP_LOGW(TAG, "Register 0x%04X: expected 0x%04X, got 0x%04X",
                register_address, expected_value, read_value);
    }

    return matches;
}

void DuartModbus::flushBuffers() {
    if (initialized_) {
        uart_flush(config_.uart_port);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/*
 * #include "duart_modbus.hpp"
#include "include/duart_modbus.hpp"

#include <algorithm>
#include <string>
#include "esp_timer.h"

const char* DuartModbus::TAG = "DUART_MODBUS";

DuartModbus::DuartModbus(const Config& config)
    : config_(config), initialized_(false) {}

DuartModbus::~DuartModbus() {
    if (initialized_) {
        uart_driver_delete(config_.uart_port);
        initialized_ = false;
        ESP_LOGD(TAG, "UART%d driver deleted", config_.uart_port);
    }
}

const char* DuartModbus::init() {
    if (initialized_) {
        return nullptr;
    }

    ESP_LOGI(TAG, "Initializing DUART_MODBUS on UART%d", config_.uart_port);

    // Convert uint8_t to proper ESP-IDF enum types
    uart_word_length_t data_bits;
    switch (config_.data_bits) {
        case 5: data_bits = UART_DATA_5_BITS; break;
        case 6: data_bits = UART_DATA_6_BITS; break;
        case 7: data_bits = UART_DATA_7_BITS; break;
        case 8: data_bits = UART_DATA_8_BITS; break;
        default: data_bits = UART_DATA_8_BITS; break;
    }

    uart_parity_t parity;
    switch (config_.parity) {
        case 0: parity = UART_PARITY_DISABLE; break;
        case 1: parity = UART_PARITY_ODD; break;
        case 2: parity = UART_PARITY_EVEN; break;
        default: parity = UART_PARITY_DISABLE; break;
    }

    uart_stop_bits_t stop_bits;
    switch (config_.stop_bits) {
        case 1: stop_bits = UART_STOP_BITS_1; break;
        case 2: stop_bits = UART_STOP_BITS_2; break;
        case 3: stop_bits = UART_STOP_BITS_1_5; break;
        default: stop_bits = UART_STOP_BITS_1; break;
    }

    uart_config_t uart_config = {
        .baud_rate = (int)config_.baud_rate,
        .data_bits = data_bits,
        .parity = parity,
        .stop_bits = stop_bits,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_DEFAULT,
        .flags = {0}  // Initialize flags
    };

    esp_err_t err = uart_param_config(config_.uart_port, &uart_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "UART parameter config failed: %s", esp_err_to_name(err));
        return "UART parameter config failed";
    }

    err = uart_set_pin(config_.uart_port, config_.tx_pin, config_.rx_pin,
                      UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "UART pin config failed: %s", esp_err_to_name(err));
        return "UART pin config failed";
    }

    err = uart_driver_install(config_.uart_port, 1024, 1024, 0, NULL, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "UART driver install failed: %s", esp_err_to_name(err));
        return "UART driver install failed";
    }

    initialized_ = true;
    ESP_LOGI(TAG, "DUART_MODBUS initialized successfully");
    return nullptr;
}

uint16_t DuartModbus::calculateCRC(const uint8_t* data, size_t length) {
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

std::vector<uint8_t> DuartModbus::buildModbusFrame(uint8_t slave_address,
                                                  uint8_t function_code,
                                                  uint16_t register_address,
                                                  uint16_t value,
                                                  uint16_t count) {
    std::vector<uint8_t> frame;

    frame.push_back(slave_address);
    frame.push_back(function_code);

    frame.push_back((register_address >> 8) & 0xFF);
    frame.push_back(register_address & 0xFF);

    if (function_code == 0x03 || function_code == 0x04) {
        frame.push_back((count >> 8) & 0xFF);
        frame.push_back(count & 0xFF);
    } else if (function_code == 0x06) {
        frame.push_back((value >> 8) & 0xFF);
        frame.push_back(value & 0xFF);
    }

    uint16_t crc = calculateCRC(frame.data(), frame.size());
    frame.push_back(crc & 0xFF);
    frame.push_back((crc >> 8) & 0xFF);

    return frame;
}

bool DuartModbus::validateModbusResponse(const std::vector<uint8_t>& frame) {
    if (frame.size() < 5) {
        return false;
    }

    uint16_t received_crc = (frame[frame.size() - 1] << 8) | frame[frame.size() - 2];
    uint16_t calculated_crc = calculateCRC(frame.data(), frame.size() - 2);

    if (received_crc != calculated_crc) {
        ESP_LOGW(TAG, "CRC mismatch: received 0x%04X, calculated 0x%04X",
                received_crc, calculated_crc);
        return false;
    }

    return true;
}

bool DuartModbus::parseModbusResponse(const std::vector<uint8_t>& response,
                                     ModbusResponse& parsed_response,
                                     uint8_t expected_slave_address) {
    parsed_response.success = false;

    if (response.size() < 5) {
        ESP_LOGW(TAG, "Response too short: %zu bytes", response.size());
        return false;
    }

    if (!validateModbusResponse(response)) {
        return false;
    }

    parsed_response.slave_address = response[0];
    if (parsed_response.slave_address != expected_slave_address) {
        ESP_LOGW(TAG, "Slave address mismatch: expected 0x%02X, got 0x%02X",
                expected_slave_address, parsed_response.slave_address);
        return false;
    }

    parsed_response.function_code = response[1];

    if (parsed_response.function_code & 0x80) {
        parsed_response.error_code = response[2];
        ESP_LOGE(TAG, "MODBUS exception: function 0x%02X, error 0x%02X",
                parsed_response.function_code & 0x7F, parsed_response.error_code);
        return false;
    }

    if (parsed_response.function_code == 0x03 || parsed_response.function_code == 0x04) {
        uint8_t byte_count = response[2];
        if (response.size() != (3 + byte_count + 2)) {
            ESP_LOGW(TAG, "Invalid byte count: %d, frame size: %zu",
                    byte_count, response.size());
            return false;
        }

        parsed_response.data.assign(response.begin() + 3, response.begin() + 3 + byte_count);

    } else if (parsed_response.function_code == 0x06) {
        if (response.size() != 8) {
            ESP_LOGW(TAG, "Invalid write response size: %zu", response.size());
            return false;
        }

        parsed_response.data.push_back(response[4]);
        parsed_response.data.push_back(response[5]);
    }

    parsed_response.success = true;
    return true;
}

esp_err_t DuartModbus::sendData(const uint8_t* data, size_t length) {
    if (!initialized_) {
        return ESP_ERR_INVALID_STATE;
    }

    flushBuffers();

    int written = uart_write_bytes(config_.uart_port, data, length);
    if (written != length) {
        ESP_LOGE(TAG, "UART write failed: wrote %d/%zu bytes", written, length);
        return ESP_FAIL;
    }

    return uart_wait_tx_done(config_.uart_port, pdMS_TO_TICKS(config_.timeout_ms));
}

esp_err_t DuartModbus::receiveDataWithTimeout(std::vector<uint8_t>& buffer, uint32_t timeout_ms) {
    if (!initialized_) {
        return ESP_ERR_INVALID_STATE;
    }

    buffer.clear();

    int64_t start_time = esp_timer_get_time();
    uint32_t elapsed_ms = 0;

    while (elapsed_ms < timeout_ms) {
        size_t available = 0;
        esp_err_t err = uart_get_buffered_data_len(config_.uart_port, &available);
        if (err != ESP_OK) {
            return err;
        }

        if (available > 0) {
            size_t old_size = buffer.size();
            buffer.resize(old_size + available);

            int read = uart_read_bytes(config_.uart_port,
                                      buffer.data() + old_size,
                                      available,
                                      pdMS_TO_TICKS(timeout_ms - elapsed_ms));

            if (read < 0) {
                return ESP_ERR_TIMEOUT;
            }

            buffer.resize(old_size + read);

            if (read < available) {
                break;
            }
        }

        vTaskDelay(1);
        elapsed_ms = (esp_timer_get_time() - start_time) / 1000;
    }

    return buffer.empty() ? ESP_ERR_TIMEOUT : ESP_OK;
}

esp_err_t DuartModbus::receiveData(std::vector<uint8_t>& buffer, size_t expected_len, uint32_t timeout_ms) {
    if (!initialized_) {
        return ESP_ERR_INVALID_STATE;
    }

    buffer.resize(expected_len);

    int read = uart_read_bytes(config_.uart_port, buffer.data(), expected_len,
                              pdMS_TO_TICKS(timeout_ms));

    if (read != expected_len) {
        ESP_LOGW(TAG, "Read %d bytes, expected %zu", read, expected_len);
        return ESP_ERR_TIMEOUT;
    }

    buffer.resize(read);
    return ESP_OK;
}

DuartModbus::ModbusResponse DuartModbus::sendRequest(const ModbusRequest& request,
                                                    uint32_t timeout_ms) {
    ModbusResponse response;
    response.esp_error = ESP_OK;

    if (!initialized_) {
        response.esp_error = ESP_ERR_INVALID_STATE;
        return response;
    }

    std::vector<uint8_t> request_frame;
    if (request.function_code == 0x03) {
        request_frame = buildModbusFrame(request.slave_address, request.function_code,
                                        request.register_address, 0, request.value);
    } else if (request.function_code == 0x06) {
        request_frame = buildModbusFrame(request.slave_address, request.function_code,
                                        request.register_address, request.value);
    } else {
        response.esp_error = ESP_ERR_NOT_SUPPORTED;
        return response;
    }

    ESP_LOGD(TAG, "Sending MODBUS frame to slave 0x%02X, function 0x%02X, reg 0x%04X",
            request.slave_address, request.function_code, request.register_address);

    response.esp_error = sendData(request_frame.data(), request_frame.size());
    if (response.esp_error != ESP_OK) {
        return response;
    }

    // Note: expected_response_len is the minimum expected size. We'll use the dynamic receive.
    // Removed unused variable to fix warning

    std::vector<uint8_t> response_frame;
    if (request.expected_response_len > 0) {
        response.esp_error = receiveData(response_frame, request.expected_response_len, timeout_ms);
    } else {
        response.esp_error = receiveDataWithTimeout(response_frame, timeout_ms);
    }

    if (response.esp_error != ESP_OK) {
        ESP_LOGW(TAG, "Failed to receive response: %s", esp_err_to_name(response.esp_error));
        return response;
    }

    // Build hex dump string
    std::string hex_dump;
    for (uint8_t byte : response_frame) {
        char buf[4];
        snprintf(buf, sizeof(buf), "%02X ", byte);
        hex_dump += buf;
    }
    ESP_LOGD(TAG, "Received response (%zu bytes): %s", response_frame.size(), hex_dump.c_str());

    if (!parseModbusResponse(response_frame, response, request.slave_address)) {
        response.success = false;
        response.esp_error = ESP_ERR_INVALID_RESPONSE;
    }

    return response;
}

DuartModbus::ModbusResponse DuartModbus::readHoldingRegisters(uint8_t slave_address,
                                                             uint16_t start_register,
                                                             uint16_t num_registers,
                                                             uint32_t timeout_ms) {
    // Note: expected_response_len is the minimum expected size. We'll use the dynamic receive.
    ModbusRequest request = {
        .slave_address = slave_address,
        .function_code = 0x03,
        .register_address = start_register,
        .value = num_registers,
        .expected_response_len = 0  // We'll use dynamic receive
    };

    return sendRequest(request, timeout_ms);
}

DuartModbus::ModbusResponse DuartModbus::writeSingleRegister(uint8_t slave_address,
                                                            uint16_t register_address,
                                                            uint16_t value,
                                                            bool verify,
                                                            uint32_t timeout_ms) {
    ModbusRequest write_request = {
        .slave_address = slave_address,
        .function_code = 0x06,
        .register_address = register_address,
        .value = value,
        .expected_response_len = 8  // Fixed response size for write single register
    };

    ModbusResponse response = sendRequest(write_request, timeout_ms);

    if (!response.success || !verify) {
        return response;
    }

    vTaskDelay(pdMS_TO_TICKS(50));

    auto verification_func = [value](uint16_t read_value) {
        return read_value == value;
    };

    return writeSingleRegisterWithVerification(slave_address, register_address,
                                              value, verification_func, timeout_ms);
}

DuartModbus::ModbusResponse DuartModbus::writeSingleRegisterWithVerification(
    uint8_t slave_address,
    uint16_t register_address,
    uint16_t value,
    std::function<bool(uint16_t)> verification_func,
    uint32_t timeout_ms) {

    ModbusResponse final_response;
    final_response.success = false;

    for (uint8_t retry = 0; retry <= config_.max_retries; retry++) {
        if (retry > 0) {
            ESP_LOGW(TAG, "Write verification failed, retry %d/%d",
                    retry, config_.max_retries);
            vTaskDelay(pdMS_TO_TICKS(100 * retry));
        }

        auto write_response = writeSingleRegister(slave_address, register_address,
                                                 value, false, timeout_ms);

        if (!write_response.success) {
            final_response = write_response;
            continue;
        }

        vTaskDelay(pdMS_TO_TICKS(50));

        auto read_response = readHoldingRegisters(slave_address, register_address, 1, timeout_ms);

        if (!read_response.success) {
            final_response = read_response;
            continue;
        }

        if (read_response.data.size() >= 2) {
            uint16_t read_value = (read_response.data[0] << 8) | read_response.data[1];

            ESP_LOGD(TAG, "Write verification: wrote 0x%04X, read 0x%04X",
                    value, read_value);

            if (verification_func(read_value)) {
                final_response = write_response;
                final_response.success = true;
                final_response.data.push_back((read_value >> 8) & 0xFF);
                final_response.data.push_back(read_value & 0xFF);
                break;
            } else {
                ESP_LOGW(TAG, "Verification failed: expected condition not met");
                final_response.esp_error = ESP_ERR_INVALID_RESPONSE;
            }
        }
    }

    return final_response;
}

bool DuartModbus::verifyRegisterValue(uint8_t slave_address,
                                     uint16_t register_address,
                                     uint16_t expected_value,
                                     uint32_t timeout_ms) {
    auto response = readHoldingRegisters(slave_address, register_address, 1, timeout_ms);

    if (!response.success || response.data.size() < 2) {
        return false;
    }

    uint16_t read_value = (response.data[0] << 8) | response.data[1];
    bool matches = (read_value == expected_value);

    if (!matches) {
        ESP_LOGW(TAG, "Register 0x%04X: expected 0x%04X, got 0x%04X",
                register_address, expected_value, read_value);
    }

    return matches;
}

void DuartModbus::flushBuffers() {
    if (initialized_) {
        uart_flush(config_.uart_port);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
*/