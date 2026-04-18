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
        std::vector<Point> points;
        bool valid;
        bool timeout_occurred;
        uint32_t timestamp_us;
        float speed_dps;
        float start_angle_deg;
        float end_angle_deg;
    };

    struct Config {
        uart_port_t uart_port;
        int baud_rate;

        // ESP32-WROOM-32 DevKitC (38-pin) explicit pinout:
        // LiDAR TX -> ESP32 RX (UART2 RXD GPIO16)
        // LiDAR PWM -> ESP32 PWM-capable output (GPIO17)
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
    gpio_num_t rx_pin = GPIO_NUM_16,
    gpio_num_t pwm_pin = GPIO_NUM_17
);
