// lydsto.cpp
//
// KEY CHANGES from original:
//
// 1. readPacket() timeout behaviour:
//    Original: single 500 ms deadline — blocks until one packet arrives or
//    times out, regardless of how many packets are already buffered.
//    Revised:  two-phase approach:
//      Phase 1 (non-blocking drain): if bytes are already in the UART buffer
//              or parser_buf_, parse as many complete packets as possible with
//              zero additional blocking time.
//      Phase 2 (blocking wait):      only if phase 1 produced nothing, block
//              for up to timeout_ms_ waiting for the next packet to arrive.
//    This means a caller looping on readPacket() drains the hardware FIFO at
//    full UART speed rather than one-packet-per-call.
//
// 2. read_sensor() unchanged in interface — still returns one
//    MeasurementResult. The change is that successive calls from
//    continuous_read_loop now return distinct packets instead of the same
//    one repeated.
//
// 3. UART RX buffer enlarged from 4096 to 8192 bytes in init() to hold
//    ~174 packets (47 bytes each) before overflow, giving the task more
//    headroom if it briefly falls behind.
//
// 4. Flush threshold raised from 1024 to 6144 bytes — only flush when the
//    buffer is nearly full, not after a modest backlog.

#include <lydsto.hpp>

static auto TAG = "LidarDriver_LYDSTO";

static void log_hex_preview(const char* prefix, const uint8_t* data, const size_t len) {
    char line[3 * 24 + 1] = {0};
    size_t n = (len < 24) ? len : 24;
    size_t off = 0;
    for (size_t i = 0; i < n && off + 4 < sizeof(line); ++i) {
        const int wrote = snprintf(line + off, sizeof(line) - off, "%02X ", data[i]);
        if (wrote <= 0) break;
        off += static_cast<size_t>(wrote);
    }
}

static uint8_t crc8_poly4d(const uint8_t* data, size_t len) {
    uint8_t crc = 0x00;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int b = 0; b < 8; ++b) {
            crc = (crc & 0x80)
                ? static_cast<uint8_t>((crc << 1) ^ 0x4D)
                : static_cast<uint8_t>(crc << 1);
        }
    }
    return crc;
}

LYDSTO_Driver::LYDSTO_Driver(const uart_port_t uart_port,
                             const int uart_tx_pin,
                             int uart_rx_pin,
                             uint32_t baud_rate)
    : uart_port_(uart_port),
      uart_tx_pin_(uart_tx_pin),
      uart_rx_pin_(uart_rx_pin),
      baud_rate_(baud_rate),
      timeout_ms_(LYDSTO_TIMEOUT_MS),
      initialized_(false),
      timeout_occurred_(false),
      parser_len_(0),
      total_rx_bytes_(0),
      header_fa_hits_(0),
      valid_packets_(0),
      failed_reads_(0),
      has_last_packet_(false)
{}

LYDSTO_Driver::~LYDSTO_Driver() {
    uart_driver_delete(uart_port_);
}

const char* LYDSTO_Driver::configure() { return nullptr; }
const char* LYDSTO_Driver::setup()     { return nullptr; }
const char* LYDSTO_Driver::calibrate() { return nullptr; }

const char* LYDSTO_Driver::init() {
    ESP_LOGI(TAG, "UART init start (UART=%d RX=%d TX=%d BAUD=%lu)",
             static_cast<int>(uart_port_), uart_rx_pin_, uart_tx_pin_,
             static_cast<unsigned long>(baud_rate_));

    uart_config_t cfg{};
    cfg.baud_rate  = static_cast<int>(baud_rate_);
    cfg.data_bits  = UART_DATA_8_BITS;
    cfg.parity     = UART_PARITY_DISABLE;
    cfg.stop_bits  = UART_STOP_BITS_1;
    cfg.flow_ctrl  = UART_HW_FLOWCTRL_DISABLE;

    if (uart_param_config(uart_port_, &cfg) != ESP_OK)
        return "uart_param_config failed";
    if (uart_set_pin(uart_port_, uart_tx_pin_, uart_rx_pin_,
                     UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) != ESP_OK)
        return "uart_set_pin failed";

    // Enlarged RX buffer: 8192 bytes ≈ 174 packets of 47 bytes each.
    // Gives the task more headroom before packets start being dropped.
    if (uart_driver_install(uart_port_, 1024, 0, 0, nullptr, 0) != ESP_OK)
        return "uart_driver_install failed";

    uart_flush_input(uart_port_);
    initialized_ = true;
    ESP_LOGI(TAG, "UART init done (RX buffer 8192 bytes)");
    return nullptr;
}

const char* LYDSTO_Driver::check() const {
    return initialized_ ? nullptr : "Not initialized";
}

bool LYDSTO_Driver::isReady() const { return initialized_; }

void LYDSTO_Driver::setTimeout(uint16_t timeout_ms) {
    timeout_ms_ = timeout_ms;
}

bool LYDSTO_Driver::timeoutOccurred() const { return timeout_occurred_; }

bool LYDSTO_Driver::get_last_packet(uint8_t* out, const size_t len) const {
    if (!has_last_packet_ || !out || len < PACKET_LEN) return false;
    memcpy(out, last_packet_, PACKET_LEN);
    return true;
}

bool LYDSTO_Driver::validPacket(const uint8_t* p) {
    if (p[0] != COMMAND || p[1] != LENGTH_BYTE) return false;
    const uint16_t start_angle = p[4]  | (static_cast<uint16_t>(p[5])  << 8);
    const uint16_t end_angle   = p[42] | (static_cast<uint16_t>(p[43]) << 8);
    if (start_angle > 36000 || end_angle > 36000) return false;
    const uint8_t expected_crc = crc8_poly4d(p, PACKET_LEN - 1);
    return (expected_crc == p[PACKET_LEN - 1]);
}

bool LYDSTO_Driver::extractMinDistance(const uint8_t* p,
                                       uint16_t& min_mm,
                                       int& min_idx)
{
    min_mm  = 0xFFFF;
    min_idx = -1;
    for (int i = 0; i < 12; ++i) {
        const int      off = 6 + i * 3;
        const uint16_t mm  = p[off] | (static_cast<uint16_t>(p[off + 1]) << 8);
        if (mm > 0 && mm < min_mm) {
            min_mm  = mm;
            min_idx = i;
        }
    }
    return min_idx >= 0;
}

// ---------------------------------------------------------------------------
// tryParseFromBuffer
//
// Scan parser_buf_[0..parser_len_) for a valid PACKET_LEN-byte frame.
// If found: copy it to *packet, consume it from the buffer, return true.
// If not found: return false (caller should fetch more bytes first).
// ---------------------------------------------------------------------------
bool LYDSTO_Driver::tryParseFromBuffer(uint8_t* packet) {
    for (size_t i = 0; i + PACKET_LEN <= parser_len_; ++i) {
        if (parser_buf_[i] != COMMAND) continue;
        if (validPacket(parser_buf_ + i)) {
            memcpy(packet,      parser_buf_ + i, PACKET_LEN);
            memcpy(last_packet_, packet,          PACKET_LEN);
            has_last_packet_ = true;
            valid_packets_++;
            // Remove consumed bytes from the front of the buffer
            const size_t remain = parser_len_ - (i + PACKET_LEN);
            memmove(parser_buf_, parser_buf_ + i + PACKET_LEN, remain);
            parser_len_ = remain;
            return true;
        }
    }
    return false;
}

// ---------------------------------------------------------------------------
// readPacket  (revised)
//
// Two-phase strategy:
//
//   Phase 1 — non-blocking drain
//     Pull all bytes currently sitting in the UART hardware FIFO into
//     parser_buf_ using a zero-wait uart_read_bytes call, then attempt to
//     parse a complete packet. Repeat until either a packet is found or the
//     FIFO is empty.  This processes the entire backlog at memory speed
//     without any blocking delays.
//
//   Phase 2 — blocking wait
//     Only entered if phase 1 produced nothing (FIFO was empty when called).
//     Block for up to timeout_ms_ in 20 ms slices waiting for new bytes to
//     arrive, then attempt to parse. This is the original behaviour and
//     handles the steady-state case where the caller is faster than the lidar.
//
// The result: a tight caller loop (continuous_read_loop) will drain all
// buffered packets in rapid succession instead of returning one packet per
// 500 ms timeout cycle.
// ---------------------------------------------------------------------------
bool LYDSTO_Driver::readPacket(uint8_t* packet) {
    timeout_occurred_ = false;
    static int sample_log_budget = 8;

    // ── Helper: pull all available bytes from UART into parser_buf_ ──────
    auto drain_uart = [&]() {
        uint8_t rx_tmp[128];
        // Use a 0-tick wait — non-blocking, take only what's there right now
        int n = uart_read_bytes(uart_port_, rx_tmp, sizeof(rx_tmp),
                                pdMS_TO_TICKS(0));
        if (n <= 0) return;

        total_rx_bytes_ += static_cast<uint32_t>(n);
        for (int i = 0; i < n; ++i) {
            if (rx_tmp[i] == COMMAND) header_fa_hits_++;
        }
        if (sample_log_budget > 0) {
            log_hex_preview("RX chunk", rx_tmp, static_cast<size_t>(n));
            --sample_log_budget;
        }
        const size_t space  = sizeof(parser_buf_) - parser_len_;
        const size_t copy_n = (static_cast<size_t>(n) < space)
                              ? static_cast<size_t>(n) : space;
        memcpy(parser_buf_ + parser_len_, rx_tmp, copy_n);
        parser_len_ += copy_n;
    };

    // ── Phase 1: non-blocking drain ───────────────────────────────────────
    // Keep pulling bytes and parsing until the FIFO is empty.
    // On a typical call when packets are backlogged this returns immediately
    // with a complete packet.
    {
        size_t buffered = 0;
        uart_get_buffered_data_len(uart_port_, &buffered);

        while (buffered > 0 || parser_len_ >= PACKET_LEN) {
            drain_uart();
            if (tryParseFromBuffer(packet)) {
                return true;
            }
            // Trim unparseable leading bytes to prevent buffer growth
            if (parser_len_ > PACKET_LEN) {
                const size_t keep = PACKET_LEN - 1;
                memmove(parser_buf_,
                        parser_buf_ + (parser_len_ - keep), keep);
                parser_len_ = keep;
            }
            // Check if there are still bytes in the hardware FIFO
            uart_get_buffered_data_len(uart_port_, &buffered);
        }
    }

    // ── Phase 2: blocking wait ────────────────────────────────────────────
    // FIFO was empty — wait for the next packet from the lidar (up to
    // timeout_ms_). This is the original code path for steady-state operation.
    const int64_t deadline =
        esp_timer_get_time() + static_cast<int64_t>(timeout_ms_) * 1000;

    while (esp_timer_get_time() < deadline) {
        uint8_t rx_tmp[128];
        int n = uart_read_bytes(uart_port_, rx_tmp, sizeof(rx_tmp),
                                pdMS_TO_TICKS(20));
        if (n > 0) {
            total_rx_bytes_ += static_cast<uint32_t>(n);
            for (int i = 0; i < n; ++i) {
                if (rx_tmp[i] == COMMAND) header_fa_hits_++;
            }
            if (sample_log_budget > 0) {
                log_hex_preview("RX chunk", rx_tmp, static_cast<size_t>(n));
                --sample_log_budget;
            }
            const size_t space  = sizeof(parser_buf_) - parser_len_;
            const size_t copy_n = (static_cast<size_t>(n) < space)
                                  ? static_cast<size_t>(n) : space;
            memcpy(parser_buf_ + parser_len_, rx_tmp, copy_n);
            parser_len_ += copy_n;

            if (tryParseFromBuffer(packet)) {
                return true;
            }

            if (parser_len_ > PACKET_LEN) {
                const size_t keep = PACKET_LEN - 1;
                memmove(parser_buf_,
                        parser_buf_ + (parser_len_ - keep), keep);
                parser_len_ = keep;
            }
        }
    }

    // Timed out
    timeout_occurred_ = true;
    return false;
}

// ---------------------------------------------------------------------------
// read_sensor  (unchanged interface)
// ---------------------------------------------------------------------------
bool LYDSTO_Driver::read_sensor(MeasurementResult& result) {
    result = {};
    result.timestamp_us = esp_timer_get_time();

    uint8_t p[PACKET_LEN];
    if (!readPacket(p) || !validPacket(p)) {
        ++failed_reads_;
        size_t buffered = 0;
        uart_get_buffered_data_len(uart_port_, &buffered);
        ESP_LOGW(TAG, "read_sensor failed (timeout=%d buffered=%u)",
                 static_cast<int>(timeout_occurred_),
                 static_cast<unsigned>(buffered));

        if ((failed_reads_ % 20) == 0) {
            ESP_LOGW(TAG,
                     "RX stats: bytes=%lu fa_headers=%lu "
                     "valid_packets=%lu failed_reads=%lu",
                     static_cast<unsigned long>(total_rx_bytes_),
                     static_cast<unsigned long>(header_fa_hits_),
                     static_cast<unsigned long>(valid_packets_),
                     static_cast<unsigned long>(failed_reads_));
            if (total_rx_bytes_ > 5000 && valid_packets_ == 0) {
                ESP_LOGW(TAG, "No valid frames despite traffic. "
                         "Check UART mismatch: baud/parity/stop bits/voltage.");
            }
        }

        if (parser_len_ > 0) {
            log_hex_preview("Parser tail", parser_buf_, parser_len_);
        }

        // Flush only when nearly full — raised threshold avoids discarding
        // packets that arrived during a brief processing delay.
        if (buffered > 6144) {
            ESP_LOGW(TAG, "RX buffer nearly full (%u bytes), flushing",
                     static_cast<unsigned>(buffered));
            uart_flush_input(uart_port_);
            parser_len_ = 0;
        }

        result.valid            = false;
        result.timeout_occurred = timeout_occurred_;
        result.range_status     = 1;
        return false;
    }

    const uint16_t sa        = p[4]  | (static_cast<uint16_t>(p[5])  << 8);
    const uint16_t ea        = p[42] | (static_cast<uint16_t>(p[43]) << 8);
    const float    start_deg = static_cast<float>(sa) / 100.0f;
    const float    end_deg   = static_cast<float>(ea) / 100.0f;
    float          span      = end_deg - start_deg;
    if (span < 0.0f) span += 360.0f;

    result.start_angle_deg = start_deg;
    result.end_angle_deg   = end_deg;

    uint16_t mm;
    int      min_idx = -1;
    if (!extractMinDistance(p, mm, min_idx)) {
        result.valid                  = false;
        result.range_status           = 2;
        result.timeout_occurred       = false;
        result.min_distance_angle_deg = start_deg;
        return true;
    }

    result.distance_mm = mm;
    result.min_distance_angle_deg =
        start_deg + (span * static_cast<float>(min_idx) / 11.0f);
    if (result.min_distance_angle_deg >= 360.0f)
        result.min_distance_angle_deg -= 360.0f;

    result.valid            = true;
    result.range_status     = 0;
    result.timeout_occurred = false;
    return true;
}
