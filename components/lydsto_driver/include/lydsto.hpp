#pragma once
#include "idf_c_includes.hpp"

// LYDSTO LDS02RR UART DRIVER CONFIGURATION (ESP32-WROOM DevKitC 38-pin)
#define LYDSTO_UART_PORT      UART_NUM_2
#define LYDSTO_TX_PIN         UART_PIN_NO_CHANGE
#define LYDSTO_RX_PIN         GPIO_NUM_3
#define LYDSTO_BAUD_RATE      115200
#define LYDSTO_TIMEOUT_MS     500

class LYDSTO_Driver {
public:
    struct MeasurementResult {
        uint16_t distance_mm;
        uint8_t  range_status;
        bool     valid;
        bool     timeout_occurred;
        int64_t  timestamp_us;
        float    start_angle_deg;
        float    end_angle_deg;
        float    min_distance_angle_deg;
    };

    explicit LYDSTO_Driver(uart_port_t uart_port = LYDSTO_UART_PORT,
                           int uart_tx_pin = LYDSTO_TX_PIN,
                           int uart_rx_pin = LYDSTO_RX_PIN,
                           uint32_t baud_rate = LYDSTO_BAUD_RATE);
    ~LYDSTO_Driver();

    static const char* configure();
    const char* init();
    static const char* setup();
    static const char* calibrate();
    [[nodiscard]] const char* check() const;
    bool read_sensor(MeasurementResult& result);
    [[nodiscard]] bool isReady() const;

    void setTimeout(uint16_t timeout_ms);
    [[nodiscard]] bool timeoutOccurred() const;
    bool get_last_packet(uint8_t* out, size_t len) const;
    [[nodiscard]] uint32_t get_packet_count() const { return valid_packets_; }

    LYDSTO_Driver(const LYDSTO_Driver&) = delete;
    LYDSTO_Driver& operator=(const LYDSTO_Driver&) = delete;

private:
    static constexpr uint8_t PACKET_LEN   = 47;
    static constexpr uint8_t COMMAND      = 0x54;
    static constexpr uint8_t LENGTH_BYTE  = 0x2C;

    uart_port_t uart_port_;
    int         uart_tx_pin_;
    int         uart_rx_pin_;
    uint32_t    baud_rate_;
    uint16_t    timeout_ms_;

    bool     initialized_;
    bool     timeout_occurred_;
    uint8_t  parser_buf_[512]{};
    size_t   parser_len_;
    uint32_t total_rx_bytes_;
    uint32_t header_fa_hits_;
    uint32_t valid_packets_;
    uint32_t failed_reads_;
    uint8_t  last_packet_[47]{};
    bool     has_last_packet_;

    // Scans parser_buf_ for a complete valid packet.
    // On success: copies packet to *out, removes it from parser_buf_,
    // returns true. On failure: returns false, buffer unchanged.
    bool tryParseFromBuffer(uint8_t* out);

    bool readPacket(uint8_t* packet);
    static bool validPacket(const uint8_t* packet);
    static bool extractMinDistance(const uint8_t* packet,
                                   uint16_t& min_mm,
                                   int& min_idx);
};
