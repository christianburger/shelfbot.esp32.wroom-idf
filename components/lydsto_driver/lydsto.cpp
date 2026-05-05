#include "lydsto.hpp"

static const char* TAG = "LidarDriver_LYDSTO";

static void log_hex_preview(const char* prefix, const uint8_t* data, size_t len) {
    char line[3 * 24 + 1] = {0};
    size_t n = (len < 24) ? len : 24;
    size_t off = 0;
    for (size_t i = 0; i < n && off + 4 < sizeof(line); ++i) {
        int wrote = snprintf(line + off, sizeof(line) - off, "%02X ", data[i]);
        if (wrote <= 0) break;
        off += static_cast<size_t>(wrote);
    }
    ESP_LOGW(TAG, "%s (%uB): %s", prefix, static_cast<unsigned>(len), line);
}

static uint8_t crc8_poly4d(const uint8_t* data, size_t len) {
    uint8_t crc = 0x00;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int b = 0; b < 8; ++b) {
            crc = (crc & 0x80) ? static_cast<uint8_t>((crc << 1) ^ 0x4D) : static_cast<uint8_t>(crc << 1);
        }
    }
    return crc;
}

LYDSTO_Driver::LYDSTO_Driver(uart_port_t uart_port, int uart_tx_pin, int uart_rx_pin, uint32_t baud_rate)
    : uart_port_(uart_port), uart_tx_pin_(uart_tx_pin), uart_rx_pin_(uart_rx_pin),
      baud_rate_(baud_rate), timeout_ms_(LYDSTO_TIMEOUT_MS),
      initialized_(false), timeout_occurred_(false), parser_len_(0),
      total_rx_bytes_(0), header_fa_hits_(0), valid_packets_(0), failed_reads_(0), has_last_packet_(false) {}

LYDSTO_Driver::~LYDSTO_Driver() {
    uart_driver_delete(uart_port_);
}

const char* LYDSTO_Driver::configure() { return nullptr; }

const char* LYDSTO_Driver::init() {
    ESP_LOGI(TAG, "UART init start (UART=%d RX=%d TX=%d BAUD=%lu)",
             static_cast<int>(uart_port_), uart_rx_pin_, uart_tx_pin_, static_cast<unsigned long>(baud_rate_));
    uart_config_t cfg{};
    cfg.baud_rate = baud_rate_;
    cfg.data_bits = UART_DATA_8_BITS;
    cfg.parity = UART_PARITY_DISABLE;
    cfg.stop_bits = UART_STOP_BITS_1;
    cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    if (uart_param_config(uart_port_, &cfg) != ESP_OK) return "uart_param_config failed";
    if (uart_set_pin(uart_port_, uart_tx_pin_, uart_rx_pin_, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) != ESP_OK) return "uart_set_pin failed";
    if (uart_driver_install(uart_port_, 4096, 0, 0, nullptr, 0) != ESP_OK) return "uart_driver_install failed";
    uart_flush_input(uart_port_);
    initialized_ = true;
    ESP_LOGI(TAG, "UART init done");
    return nullptr;
}

const char* LYDSTO_Driver::setup() { return nullptr; }
const char* LYDSTO_Driver::calibrate() { return nullptr; }
const char* LYDSTO_Driver::check() { return initialized_ ? nullptr : "Not initialized"; }
bool LYDSTO_Driver::isReady() const { return initialized_; }
void LYDSTO_Driver::setTimeout(uint16_t timeout_ms) { timeout_ms_ = timeout_ms; }
bool LYDSTO_Driver::timeoutOccurred() { return timeout_occurred_; }
bool LYDSTO_Driver::get_last_packet(uint8_t* out, size_t len) const {
    if (!has_last_packet_ || !out || len < PACKET_LEN) return false;
    memcpy(out, last_packet_, PACKET_LEN);
    return true;
}

bool LYDSTO_Driver::validPacket(const uint8_t* p) const {
    if (p[0] != COMMAND || p[1] != LENGTH_BYTE) return false;
    uint16_t start_angle = p[4] | (static_cast<uint16_t>(p[5]) << 8);
    uint16_t end_angle = p[42] | (static_cast<uint16_t>(p[43]) << 8);
    if (start_angle > 36000 || end_angle > 36000) return false;
    uint8_t expected_crc = crc8_poly4d(p, PACKET_LEN - 1);
    uint8_t frame_crc = p[PACKET_LEN - 1];
    if (expected_crc != frame_crc) {
        return false;
    }
    return true;
}

bool LYDSTO_Driver::extractMinDistance(const uint8_t* p, uint16_t& min_mm) const {
    min_mm = 0xFFFF;
    for (int i = 0; i < 12; ++i) {
        int off = 6 + i * 3;
        uint16_t mm = p[off] | (static_cast<uint16_t>(p[off + 1]) << 8);
        if (mm > 0 && mm < min_mm) min_mm = mm;
    }
    return min_mm != 0xFFFF;
}

bool LYDSTO_Driver::readPacket(uint8_t* packet) {
    timeout_occurred_ = false;
    int64_t deadline = esp_timer_get_time() + static_cast<int64_t>(timeout_ms_) * 1000;
    uint8_t rx_tmp[128];
    static int sample_log_budget = 8;
    while (esp_timer_get_time() < deadline) {
        int n = uart_read_bytes(uart_port_, rx_tmp, sizeof(rx_tmp), pdMS_TO_TICKS(20));
        if (n > 0) {
            total_rx_bytes_ += static_cast<uint32_t>(n);
            for (int i = 0; i < n; ++i) {
                if (rx_tmp[i] == COMMAND) header_fa_hits_++;
            }
            if (sample_log_budget > 0) {
                log_hex_preview("RX chunk", rx_tmp, static_cast<size_t>(n));
                sample_log_budget--;
            }
            size_t space = sizeof(parser_buf_) - parser_len_;
            size_t copy_n = (static_cast<size_t>(n) < space) ? static_cast<size_t>(n) : space;
            memcpy(parser_buf_ + parser_len_, rx_tmp, copy_n);
            parser_len_ += copy_n;

            for (size_t i = 0; i + PACKET_LEN <= parser_len_; ++i) {
                if (parser_buf_[i] != COMMAND) continue;
                if (validPacket(parser_buf_ + i)) {
                    log_hex_preview("Valid packet", parser_buf_ + i, PACKET_LEN);
                    memcpy(packet, parser_buf_ + i, PACKET_LEN);
                    memcpy(last_packet_, packet, PACKET_LEN);
                    has_last_packet_ = true;
                    valid_packets_++;
                    size_t remain = parser_len_ - (i + PACKET_LEN);
                    memmove(parser_buf_, parser_buf_ + i + PACKET_LEN, remain);
                    parser_len_ = remain;
                    return true;
                }
            }

            if (parser_len_ > PACKET_LEN) {
                size_t keep = PACKET_LEN - 1;
                memmove(parser_buf_, parser_buf_ + (parser_len_ - keep), keep);
                parser_len_ = keep;
            }
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
        failed_reads_++;
        size_t buffered = 0;
        uart_get_buffered_data_len(uart_port_, &buffered);
        ESP_LOGW(TAG, "read_sensor failed (timeout=%d buffered=%u)",
                 static_cast<int>(timeout_occurred_), static_cast<unsigned>(buffered));
        if ((failed_reads_ % 20) == 0) {
            ESP_LOGW(TAG,
                     "RX stats: bytes=%lu fa_headers=%lu valid_packets=%lu failed_reads=%lu",
                     static_cast<unsigned long>(total_rx_bytes_),
                     static_cast<unsigned long>(header_fa_hits_),
                     static_cast<unsigned long>(valid_packets_),
                     static_cast<unsigned long>(failed_reads_));
            if (total_rx_bytes_ > 5000 && valid_packets_ == 0) {
                ESP_LOGW(TAG, "No valid frames despite traffic. Check UART mismatch: baud/parity/stop bits/voltage levels.");
            }
        }
        if (parser_len_ > 0) {
            log_hex_preview("Parser tail", parser_buf_, parser_len_);
        }
        if (buffered > 1024) {
            ESP_LOGW(TAG, "input backlog detected, flushing UART RX buffer");
            uart_flush_input(uart_port_);
            parser_len_ = 0;
        }
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
        return true;
    }
    result.distance_mm = mm;
    result.valid = true;
    result.range_status = 0;
    result.timeout_occurred = false;
    return true;
}
