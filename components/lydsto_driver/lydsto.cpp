#include "lydsto.hpp"

static const char* TAG = "TofDriver_LYDSTO";

LYDSTO_Driver::LYDSTO_Driver()
    : uart_port_(LYDSTO_UART_PORT), uart_tx_pin_(LYDSTO_TX_PIN), uart_rx_pin_(LYDSTO_RX_PIN),
      baud_rate_(LYDSTO_BAUD_RATE), timeout_ms_(LYDSTO_TIMEOUT_MS),
      initialized_(false), timeout_occurred_(false) {}

LYDSTO_Driver::~LYDSTO_Driver() {
    uart_driver_delete(uart_port_);
}

const char* LYDSTO_Driver::configure() { return nullptr; }

const char* LYDSTO_Driver::init() {
    uart_config_t cfg{};
    cfg.baud_rate = baud_rate_;
    cfg.data_bits = UART_DATA_8_BITS;
    cfg.parity = UART_PARITY_DISABLE;
    cfg.stop_bits = UART_STOP_BITS_1;
    cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    if (uart_param_config(uart_port_, &cfg) != ESP_OK) return "uart_param_config failed";
    if (uart_set_pin(uart_port_, uart_tx_pin_, uart_rx_pin_, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) != ESP_OK) return "uart_set_pin failed";
    if (uart_driver_install(uart_port_, 4096, 0, 0, nullptr, 0) != ESP_OK) return "uart_driver_install failed";
    initialized_ = true;
    return nullptr;
}

const char* LYDSTO_Driver::setup() { return nullptr; }
const char* LYDSTO_Driver::calibrate() { return nullptr; }
const char* LYDSTO_Driver::check() { return initialized_ ? nullptr : "Not initialized"; }
bool LYDSTO_Driver::isReady() const { return initialized_; }
void LYDSTO_Driver::setTimeout(uint16_t timeout_ms) { timeout_ms_ = timeout_ms; }
bool LYDSTO_Driver::timeoutOccurred() { return timeout_occurred_; }

bool LYDSTO_Driver::validPacket(const uint8_t* p) const {
    if (p[0] != COMMAND || p[1] < INDEX_LO || p[1] > 0xF9) return false;
    uint32_t chk32 = 0;
    for (int i = 0; i < 20; i += 2) {
        uint16_t w = p[i] | (static_cast<uint16_t>(p[i + 1]) << 8);
        chk32 = (chk32 << 1) + w;
    }
    uint16_t checksum = (chk32 & 0x7FFF) + (chk32 >> 15);
    checksum &= 0x7FFF;
    return (p[20] == (checksum & 0xFF)) && (p[21] == (checksum >> 8));
}

bool LYDSTO_Driver::extractMinDistance(const uint8_t* p, uint16_t& min_mm) const {
    min_mm = 0xFFFF;
    for (int i = 0; i < 4; ++i) {
        int off = 4 + i * 4;
        uint8_t dataL = p[off];
        uint8_t dataM = p[off + 1];
        if (dataM & 0xC0) continue; // invalid or warning
        uint16_t mm = dataL | ((dataM & 0x3F) << 8);
        if (mm > 0 && mm < min_mm) min_mm = mm;
    }
    return min_mm != 0xFFFF;
}

bool LYDSTO_Driver::readPacket(uint8_t* packet) {
    timeout_occurred_ = false;
    int64_t deadline = esp_timer_get_time() + static_cast<int64_t>(timeout_ms_) * 1000;
    uint8_t b;
    while (esp_timer_get_time() < deadline) {
        if (uart_read_bytes(uart_port_, &b, 1, pdMS_TO_TICKS(20)) == 1 && b == COMMAND) {
            packet[0] = b;
            int n = uart_read_bytes(uart_port_, packet + 1, PACKET_LEN - 1, pdMS_TO_TICKS(timeout_ms_));
            if (n == PACKET_LEN - 1) return true;
        }
    }
    timeout_occurred_ = true;
    return false;
}

bool LYDSTO_Driver::read_sensor(MeasurementResult& result) {
    result = {};
    result.timestamp_us = esp_timer_get_time();
    uint8_t p[PACKET_LEN];
    if (!readPacket(p) || !validPacket(p)) {
        result.valid = false;
        result.timeout_occurred = timeout_occurred_;
        result.range_status = 1;
        return false;
    }
    uint16_t mm;
    if (!extractMinDistance(p, mm)) {
        result.valid = false;
        result.range_status = 2;
        result.timeout_occurred = false;
        return false;
    }
    result.distance_mm = mm;
    result.valid = true;
    result.range_status = 0;
    result.timeout_occurred = false;
    return true;
}
