#pragma once

#include <cstdint>
#include <memory>
#include <vector>
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/uart.h"

class LydstoLidar {
public:
    struct Point {
        float angle_deg;
        uint16_t distance_mm;
        uint8_t intensity;
        bool valid;
    };

    struct MeasurementResult {
        uint16_t distance_mm;       ///< Nearest valid distance in mm
        float distance_cm;          ///< Nearest valid distance in cm
        bool valid;                 ///< True if at least one valid point was found
        bool timeout_occurred;      ///< True if no complete frame available
        uint8_t status;             ///< 0=ok,1=no frame,2=no valid points
        uint32_t timestamp_us;      ///< Timestamp in microseconds
        float speed_dps;            ///< Motor speed from frame
        float start_angle_deg;      ///< Frame start angle
        float end_angle_deg;        ///< Frame end angle
    };

    struct Config {
        uart_port_t uart_port;
        int baud_rate;

        // ESP32-WROOM-32 DevKitC (38-pin) explicit pinout:
        // LiDAR TX -> ESP32 RX (remapped UART RX, GPIO18)
        // LiDAR PWM -> ESP32 PWM-capable output (GPIO19)
        gpio_num_t rx_pin;
        gpio_num_t tx_pin; // unused (device is TX-only from host perspective)
        gpio_num_t pwm_pin;

        uint32_t uart_rx_buffer_size;
        uint32_t uart_read_timeout_ms;

        bool enable_external_speed_control;
        uint32_t pwm_frequency_hz;
        float pwm_duty_cycle;

        uint16_t min_distance_mm;
        uint16_t max_distance_mm;
        uint8_t min_intensity;
    };

    explicit LydstoLidar(const Config& config);
    ~LydstoLidar();

    LydstoLidar(const LydstoLidar&) = delete;
    LydstoLidar& operator=(const LydstoLidar&) = delete;

    const char* init();
    bool isReady() const;

    bool readSingle(MeasurementResult& result);

    const char* selfTest();

private:
    struct Impl;
    std::unique_ptr<Impl> pimpl_;
};

LydstoLidar::Config lydsto_default_config(
    uart_port_t uart_port = UART_NUM_1,
    gpio_num_t rx_pin = GPIO_NUM_18,
    gpio_num_t pwm_pin = GPIO_NUM_19
);
