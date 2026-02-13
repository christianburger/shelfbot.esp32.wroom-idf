#include "duart_modbus.hpp"

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

    err = uart_driver_install(config_.uart_port, 2048, 2048, 0, NULL, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "UART driver install failed: %s", esp_err_to_name(err));
        return "UART driver install failed";
    }

    initialized_ = true;
    ESP_LOGI(TAG, "DUART_MODBUS initialized successfully");

    // CRITICAL: Check for automatic output on startup
    detectAndDisableAutomaticOutput();

    return nullptr;
}

uint16_t DuartModbus::calculateCRC(const uint8_t* data, size_t length) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < length; i++) {
        crc ^= (uint16_t)data[i];
        for (int j = 8; j != 0; j--) {
            if ((crc & 0x0001) != 0) {
                crc >>= 1;
                crc ^= 0xA001;
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
    if (function_code == 0x06) {
        frame.push_back((value >> 8) & 0xFF);
        frame.push_back(value & 0xFF);
    } else if (function_code == 0x03) {
        frame.push_back((count >> 8) & 0xFF);
        frame.push_back(count & 0xFF);
    }
    uint16_t crc = calculateCRC(frame.data(), frame.size());
    frame.push_back(crc & 0xFF);
    frame.push_back((crc >> 8) & 0xFF);
    return frame;
}

esp_err_t DuartModbus::sendData(const uint8_t* data, size_t length) {
    ESP_LOGI(TAG, "Sending %zu bytes:", length);
    for (size_t i = 0; i < length; i++) {
        ESP_LOGI(TAG, "  0x%02X", data[i]);
    }
    int len = uart_write_bytes(config_.uart_port, data, length);
    if (len != static_cast<int>(length)) {
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t DuartModbus::receiveData(std::vector<uint8_t>& buffer, size_t expected_len, uint32_t timeout_ms) {
    buffer.resize(expected_len);
    int len = uart_read_bytes(config_.uart_port, buffer.data(), expected_len, pdMS_TO_TICKS(timeout_ms));
    if (len < 0) {
        return ESP_FAIL;
    }
    buffer.resize(len);
    ESP_LOGI(TAG, "Received %d bytes:", len);
    for (size_t i = 0; i < buffer.size(); i++) {
        ESP_LOGI(TAG, "  0x%02X", buffer[i]);
    }
    if (len != static_cast<int>(expected_len)) {
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}

esp_err_t DuartModbus::receiveDataWithTimeout(std::vector<uint8_t>& buffer, uint32_t timeout_ms) {
    buffer.clear();
    uint8_t byte;
    uint64_t start = esp_timer_get_time();
    uint64_t end_time = start + static_cast<uint64_t>(timeout_ms) * 1000ULL;
    while (esp_timer_get_time() < end_time) {
        int len = uart_read_bytes(config_.uart_port, &byte, 1, pdMS_TO_TICKS(10));
        if (len == 1) {
            buffer.push_back(byte);
        }
    }
    ESP_LOGI(TAG, "Received %zu bytes during timeout period:", buffer.size());
    for (size_t i = 0; i < buffer.size(); i++) {
        ESP_LOGI(TAG, "  0x%02X", buffer[i]);
    }
    return buffer.empty() ? ESP_ERR_TIMEOUT : ESP_OK;
}

bool DuartModbus::parseModbusResponse(const std::vector<uint8_t>& response,
                                      ModbusResponse& parsed_response,
                                      uint8_t expected_slave_address) {
    if (response.size() < 4) return false;
    parsed_response.slave_address = response[0];
    parsed_response.function_code = response[1];
    if (parsed_response.slave_address != expected_slave_address) return false;
    if (parsed_response.function_code & 0x80) {
        if (response.size() < 5) return false;
        parsed_response.error_code = response[2];
        return false;
    }
    size_t data_len;
    if (parsed_response.function_code == 0x03) {
        if (response.size() < 3) return false;
        data_len = response[2];
        if (response.size() != 3 + data_len + 2) return false;
        parsed_response.data.assign(response.begin() + 3, response.begin() + 3 + data_len);
    } else if (parsed_response.function_code == 0x06) {
        if (response.size() != 8) return false;
    } else {
        return false;
    }
    uint16_t calc_crc = calculateCRC(response.data(), response.size() - 2);
    uint16_t recv_crc = (response[response.size() - 1] << 8) | response[response.size() - 2];
    if (calc_crc != recv_crc) return false;
    return true;
}

bool DuartModbus::validateModbusResponse(const std::vector<uint8_t>& frame) {
    if (frame.size() < 4) return false;
    uint16_t calc_crc = calculateCRC(frame.data(), frame.size() - 2);
    uint16_t recv_crc = (frame[frame.size() - 1] << 8) | frame[frame.size() - 2];
    return calc_crc == recv_crc;
}

DuartModbus::ModbusResponse DuartModbus::sendRequest(const ModbusRequest& request, uint32_t timeout_ms) {
    ModbusResponse resp;
    resp.success = false;
    resp.slave_address = request.slave_address;
    resp.function_code = request.function_code;
    resp.error_code = 0;
    resp.esp_error = ESP_OK;

    std::vector<uint8_t> frame = buildModbusFrame(request.slave_address, request.function_code,
                                                  request.register_address, request.value, request.value);  // value as count for read

    esp_err_t err = sendData(frame.data(), frame.size());
    if (err != ESP_OK) {
        resp.esp_error = err;
        return resp;
    }

    std::vector<uint8_t> rx_buf;
    err = receiveData(rx_buf, request.expected_response_len, timeout_ms);
    if (err != ESP_OK) {
        resp.esp_error = err;
        return resp;
    }

    if (parseModbusResponse(rx_buf, resp, request.slave_address)) {
        resp.success = true;
    } else if (resp.function_code & 0x80) {
        resp.success = false;
    } else {
        resp.esp_error = ESP_ERR_INVALID_RESPONSE;
    }

    return resp;
}

DuartModbus::ModbusResponse DuartModbus::readHoldingRegisters(uint8_t slave_address,
                                                              uint16_t start_register,
                                                              uint16_t num_registers,
                                                              uint32_t timeout_ms) {
    ModbusRequest req;
    req.slave_address = slave_address;
    req.function_code = 0x03;
    req.register_address = start_register;
    req.value = num_registers;  // count
    req.expected_response_len = 3 + (num_registers * 2) + 2;
    return sendRequest(req, timeout_ms);
}

DuartModbus::ModbusResponse DuartModbus::writeSingleRegister(uint8_t slave_address,
                                                             uint16_t register_address,
                                                             uint16_t value,
                                                             bool verify,
                                                             uint32_t timeout_ms) {
    ModbusRequest req;
    req.slave_address = slave_address;
    req.function_code = 0x06;
    req.register_address = register_address;
    req.value = value;
    req.expected_response_len = 8;
    auto resp = sendRequest(req, timeout_ms);
    if (verify && resp.success) {
        auto vresp = readHoldingRegisters(slave_address, register_address, 1, timeout_ms);
        if (vresp.success && vresp.data.size() >= 2) {
            uint16_t rvalue = (vresp.data[0] << 8) | vresp.data[1];
            if (rvalue != value) {
                resp.success = false;
                resp.esp_error = ESP_ERR_INVALID_RESPONSE;
            }
        } else {
            resp.success = false;
            resp.esp_error = vresp.esp_error;
        }
    }
    return resp;
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

DuartModbus::HealthCheckResult DuartModbus::performHealthCheck() {
    HealthCheckResult result = {};
    result.uart_initialized = initialized_;

    if (!initialized_) {
        result.diagnosis = "UART not initialized";
        result.recommended_action = "Call init() first";
        return result;
    }

    // Passive listen for burst data
    ESP_LOGI(TAG, "Health Check: Passive listening for 2 seconds...");
    flushBuffers();
    std::vector<uint8_t> passive_data;
    receiveDataWithTimeout(passive_data, 2000);
    result.burst_data_size = passive_data.size();
    result.raw_burst_data = passive_data;

    // Use hypotheses tests
    bool no_data = testHypothesis_NoDataReceived();
    result.device_detected = !no_data || !testHypothesis_DeviceInactive();
    result.automatic_output_detected = testHypothesis_ContinuousOutput();
    result.crc_validation_passing = !testHypothesis_NonModbusBursts();

    analyzeRawBurst(passive_data, result);

    // Test communication
    ESP_LOGI(TAG, "Health Check: Testing Modbus communication...");
    uint8_t slave_addr = 0x01;  // Default
    ModbusResponse resp = readHoldingRegisters(slave_addr, 0x0001, 1, config_.timeout_ms);
    if (resp.success) {
        result.modbus_communication_working = true;
        result.detected_slave_address = slave_addr;
        result.crc_validation_passing = true;
    } else {
        // Scan for address
        ESP_LOGI(TAG, "Health Check: Scanning for slave address...");
        for (uint8_t addr = 1; addr <= 247; addr++) {
            resp = readHoldingRegisters(addr, 0x0001, 1, 200);
            if (resp.success) {
                result.detected_slave_address = addr;
                result.modbus_communication_working = true;
                result.crc_validation_passing = true;
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }

    if (result.modbus_communication_working) {
        result.device_detected = true;
    }

    // Set diagnosis based on tests
    if (testHypothesis_WrongBaudRate()) {
        result.diagnosis = "Baud rate mismatch likely";
        result.recommended_action = "Try different baud rates (115200, 38400, 9600)";
    } else if (testHypothesis_WrongSlaveAddress()) {
        result.diagnosis = "Wrong slave address";
        result.recommended_action = "Use detected address if found, or check device config";
    } else if (result.automatic_output_detected) {
        result.diagnosis = "Automatic continuous output detected";
        result.recommended_action = "Disable by writing 0x0000 to register 0x0005";
    } else if (!result.device_detected) {
        result.diagnosis = "No device detected";
        result.recommended_action = "Check power, wiring, UART pins, device mode";
    } else if (!result.modbus_communication_working) {
        result.diagnosis = "Modbus communication failing";
        result.recommended_action = "Verify parity, stop bits, data bits";
    } else {
        result.diagnosis = "Device healthy";
        result.recommended_action = "None needed";
    }

    return result;
}

void DuartModbus::detectAndDisableAutomaticOutput() {
    ESP_LOGI(TAG, "Checking for automatic output...");
    HealthCheckResult health = performHealthCheck();
    if (health.automatic_output_detected) {
        ESP_LOGW(TAG, "Automatic output detected - attempting to disable");
        uint8_t slave = health.detected_slave_address ? health.detected_slave_address : 0x01;
        auto resp = writeSingleRegister(slave, 0x0005, 0x0000, false, config_.timeout_ms);
        if (resp.success) {
            ESP_LOGI(TAG, "Successfully disabled automatic output");
            vTaskDelay(pdMS_TO_TICKS(100));
            flushBuffers();
        } else {
            ESP_LOGE(TAG, "Failed to disable automatic output - manual intervention needed");
        }
    } else {
        ESP_LOGI(TAG, "No automatic output detected");
    }
}

bool DuartModbus::testHypothesis_NoDataReceived() {
    flushBuffers();
    std::vector<uint8_t> buf;
    receiveDataWithTimeout(buf, 1000);
    return buf.empty();
}

bool DuartModbus::testHypothesis_DeviceInactive() {
    ModbusResponse resp = readHoldingRegisters(0x01, 0x0001, 1, 500);
    return !resp.success && resp.esp_error == ESP_ERR_TIMEOUT;
}

bool DuartModbus::testHypothesis_NonModbusBursts() {
    std::vector<uint8_t> buf;
    receiveDataWithTimeout(buf, 1000);
    if (buf.empty()) return false;
    return !validateModbusResponse(buf);
}

bool DuartModbus::testHypothesis_ContinuousOutput() {
    flushBuffers();
    vTaskDelay(pdMS_TO_TICKS(500));
    std::vector<uint8_t> buf1;
    receiveDataWithTimeout(buf1, 100);
    if (buf1.empty()) return false;
    flushBuffers();
    vTaskDelay(pdMS_TO_TICKS(500));
    std::vector<uint8_t> buf2;
    receiveDataWithTimeout(buf2, 100);
    return !buf2.empty();
}

bool DuartModbus::testHypothesis_WrongSlaveAddress() {
    ModbusResponse resp = readHoldingRegisters(0x01, 0x0001, 1, 500);
    if (resp.success) return false;
    for (uint8_t addr = 2; addr <= 247; addr++) {
        resp = readHoldingRegisters(addr, 0x0001, 1, 200);
        if (resp.success) return true;  // Found different address
    }
    return false;
}

bool DuartModbus::testHypothesis_WrongBaudRate() {
    std::vector<uint8_t> buf;
    receiveDataWithTimeout(buf, 1000);
    if (buf.empty()) return false;
    bool has_valid_crc = false;
    for (size_t len = 4; len <= buf.size(); ++len) {
        uint16_t calc = calculateCRC(buf.data(), len - 2);
        uint16_t recv = (buf[len - 1] << 8) | buf[len - 2];
        if (calc == recv) {
            has_valid_crc = true;
            break;
        }
    }
    return !has_valid_crc;
}

void DuartModbus::analyzeRawBurst(const std::vector<uint8_t>& data, HealthCheckResult& result) {
    if (data.empty()) return;
    size_t pos = 0;
    int valid_count = 0;
    int invalid_crc_count = 0;
    while (pos + 4 < data.size()) {
        // Guess frame length assuming func 0x03
        if (data[pos + 1] == 0x03 && pos + 2 < data.size()) {
            uint8_t byte_count = data[pos + 2];
            size_t frame_len = 3 + byte_count + 2;
            if (pos + frame_len <= data.size()) {
                std::vector<uint8_t> frame(data.begin() + pos, data.begin() + pos + frame_len);
                if (validateModbusResponse(frame)) {
                    valid_count++;
                } else {
                    invalid_crc_count++;
                }
                pos += frame_len;
                continue;
            }
        }
        pos++;
    }
    if (valid_count > 0) {
        result.crc_validation_passing = true;
    }
    if (valid_count > 1 || invalid_crc_count > 0) {
        result.automatic_output_detected = true;
    }
    if (valid_count == 0) {
        result.diagnosis += " - Non-Modbus data detected";
    }
}
