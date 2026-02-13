#pragma once
#include <idf_c_includes.hpp>
#include "duart_modbus.hpp"

// ═══════════════════════════════════════════════════════════════
// VL53L1_MODBUS DRIVER CONFIGURATION - EDIT ALL SETTINGS HERE
// ═══════════════════════════════════════════════════════════════
#define VL53L1_MODBUS_UART_PORT      UART_NUM_1
#define VL53L1_MODBUS_TX_PIN         GPIO_NUM_17
#define VL53L1_MODBUS_RX_PIN         GPIO_NUM_16
#define VL53L1_MODBUS_BAUD_RATE      115200
#define VL53L1_MODBUS_SLAVE_ADDR     0x01
#define VL53L1_MODBUS_TIMEOUT_MS     500
#define VL53L1_MODBUS_RANGING_MODE   1  // 0=High Precision (30ms, 1.3m), 1=Long Distance (200ms, 4.0m)
#define VL53L1_MODBUS_CONTINUOUS     true
// ═══════════════════════════════════════════════════════════════

/**
 * @brief VL53L1_Modbus ToF Driver - Simple and Explicit Implementation
 *
 * Communicates via Modbus/UART protocol (TOF400F module)
 * All configuration is defined above in #defines.
 * No external configuration accepted.
 */
class VL53L1_Modbus_Driver {
public:
    struct MeasurementResult {
        uint16_t distance_mm;
        uint8_t  range_status;
        bool     valid;
        bool     timeout_occurred;
        int64_t  timestamp_us;
    };

    // Constructor - uses #define configuration
    VL53L1_Modbus_Driver();
    ~VL53L1_Modbus_Driver();

    // ── Standardized Interface Methods ──
    const char* configure();
    const char* init();
    const char* setup();
    const char* calibrate();
    const char* check();
    bool read_sensor(MeasurementResult& result);
    bool isReady() const;

    // ── Support operations ──
    void setTimeout(uint16_t timeout_ms);
    bool timeoutOccurred();

    // Non-copyable
    VL53L1_Modbus_Driver(const VL53L1_Modbus_Driver&) = delete;
    VL53L1_Modbus_Driver& operator=(const VL53L1_Modbus_Driver&) = delete;

private:
    // ── Configuration (from #defines) ──
    uart_port_t uart_port_;
    gpio_num_t  uart_tx_pin_;
    gpio_num_t  uart_rx_pin_;
    uint32_t    baud_rate_;
    uint8_t     modbus_slave_address_;
    uint16_t    timeout_ms_;
    uint8_t     ranging_mode_;  // 0=High Precision, 1=Long Distance
    bool        enable_continuous_;

    // ── Hardware handle ──
    DuartModbus* modbus_;

    // ── State ──
    bool initialized_;
    bool timeout_occurred_;

    // ── TOF400F Register addresses ──
    static constexpr uint16_t REG_SPECIAL                = 0x0001;
    static constexpr uint16_t REG_DEVICE_ADDR            = 0x0002;
    static constexpr uint16_t REG_BAUD_RATE              = 0x0003;
    static constexpr uint16_t REG_RANGE_MODE             = 0x0004;
    static constexpr uint16_t REG_CONTINUOUS_OUTPUT      = 0x0005;
    static constexpr uint16_t REG_LOAD_CALIBRATION       = 0x0006;
    static constexpr uint16_t REG_OFFSET_CORRECTION      = 0x0007;
    static constexpr uint16_t REG_XTALK_CORRECTION       = 0x0008;
    static constexpr uint16_t REG_DISABLE_IIC            = 0x0009;
    static constexpr uint16_t REG_MEASUREMENT            = 0x0010;
    static constexpr uint16_t REG_OFFSET_CALIBRATION     = 0x0020;
    static constexpr uint16_t REG_XTALK_CALIBRATION      = 0x0021;

    // ── Initialization helpers ──
    const char* initModbus();
    const char* testCommunication();
    const char* readCurrentConfiguration();
    const char* configureRangingMode();
    const char* configureContinuousMode();
    const char* verifyConfiguration();

    // ── Helper functions ──
    void logModbusResponse(const char* operation, const DuartModbus::ModbusResponse& response);
};

// ═══════════════════════════════════════════════════════════════
// TYPEDEF FOR TOF_SENSOR INTERFACE
// ═══════════════════════════════════════════════════════════════
using TofDriver = VL53L1_Modbus_Driver;
// ═══════════════════════════════════════════════════════════════
