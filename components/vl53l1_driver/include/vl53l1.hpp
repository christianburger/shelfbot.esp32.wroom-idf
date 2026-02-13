#pragma once
#include <idf_c_includes.hpp>

// ==========================================================================
// VL53L0X Hardware Configuration - EDIT ONLY HERE
// ==========================================================================
#define TOF_I2C_PORT         I2C_NUM_0
#define TOF_SDA_PIN          GPIO_NUM_21
#define TOF_SCL_PIN          GPIO_NUM_22
#define TOF_I2C_FREQ_HZ      40000
#define TOF_I2C_ADDRESS      0x29
#define TOF_TIMEOUT_MS       500
#define TOF_TIMING_BUDGET_US 200000
#define TOF_SIGNAL_RATE_MCPS 0.25f
// ==========================================================================

/**
 * @brief VL53L0X ToF Driver - Standardized Interface
 * All configuration sequences and calibrations are internal to driver.
 */
class TofDriver {
public:
  struct MeasurementResult {
    uint16_t distance_mm;
    uint8_t  range_status;
    bool     valid;
    bool     timeout_occurred;
    int64_t  timestamp_us;
  };

  TofDriver();
  ~TofDriver();

  // ── Standardized operations for tof_sensor ──
  const char* configure();    // Log configuration
  const char* init();         // Initialize I2C hardware
  const char* setup();        // Load init sequences
  const char* calibrate();    // Perform sensor calibrations
  const char* check();        // Health check/probe
  bool        read_sensor(MeasurementResult& result);  // Read distance

  // ── Support operations ──
  bool isReady() const;
  void setTimeout(uint16_t timeout_ms);
  bool timeoutOccurred();

  // Non-copyable
  TofDriver(const TofDriver&) = delete;
  TofDriver& operator=(const TofDriver&) = delete;

private:
  struct Impl;
  std::unique_ptr<Impl> pimpl_;
};