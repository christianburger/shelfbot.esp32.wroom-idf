#pragma once
#include <idf_c_includes.hpp>

// LYDSTO LDS02RR UART DRIVER CONFIGURATION
#define LYDSTO_UART_PORT      UART_NUM_1
#define LYDSTO_TX_PIN         GPIO_NUM_17
#define LYDSTO_RX_PIN         GPIO_NUM_16
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
    };

    LYDSTO_Driver();
    ~LYDSTO_Driver();

    const char* configure();
    const char* init();
    const char* setup();
    const char* calibrate();
    const char* check();
    bool read_sensor(MeasurementResult& result);
    bool isReady() const;

    void setTimeout(uint16_t timeout_ms);
    bool timeoutOccurred();

    LYDSTO_Driver(const LYDSTO_Driver&) = delete;
    LYDSTO_Driver& operator=(const LYDSTO_Driver&) = delete;

private:
    static constexpr uint8_t PACKET_LEN = 22;
    static constexpr uint8_t COMMAND = 0xFA;
    static constexpr uint8_t INDEX_LO = 0xA0;

    uart_port_t uart_port_;
    gpio_num_t uart_tx_pin_;
    gpio_num_t uart_rx_pin_;
    uint32_t baud_rate_;
    uint16_t timeout_ms_;

    bool initialized_;
    bool timeout_occurred_;

    bool readPacket(uint8_t* packet);
    bool validPacket(const uint8_t* packet) const;
    bool extractMinDistance(const uint8_t* packet, uint16_t& min_mm) const;
};

using TofDriver = LYDSTO_Driver;
