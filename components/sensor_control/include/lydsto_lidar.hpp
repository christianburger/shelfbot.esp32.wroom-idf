#pragma once

#include <cstdint>
#include <vector>
#include <memory>
#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

struct LydstoLidarConfig {
    uart_port_t uart_port = UART_NUM_1;
    int baud_rate = 230400;
    gpio_num_t rx_pin = GPIO_NUM_16;
    gpio_num_t tx_pin = GPIO_NUM_NC;
    gpio_num_t pwm_pin = GPIO_NUM_17;
    uint32_t uart_rx_buffer_size = 4096;
    uint32_t uart_read_timeout_ms = 20;

    // LD19/LD14-style external speed control requirements
    bool enable_external_speed_control = false;
    uint32_t pwm_frequency_hz = 30000;
    float pwm_duty_cycle = 0.50f;

    // point filtering
    uint16_t min_distance_mm = 150;
    uint16_t max_distance_mm = 8000;
    uint8_t min_intensity = 1;
};

struct LydstoLidarPoint {
    float angle_deg = 0.0f;
    float distance_cm = 0.0f;
    uint8_t intensity = 0;
    bool valid = false;
    int64_t timestamp_us = 0;
};

struct LydstoLidarFrameMeta {
    float speed_dps = 0.0f;
    float start_angle_deg = 0.0f;
    float end_angle_deg = 0.0f;
    uint16_t timestamp_ms = 0;
    bool crc_ok = false;
};

class LydstoLidarArray {
public:
    LydstoLidarArray() = default;
    ~LydstoLidarArray();

    bool init(const LydstoLidarConfig& cfg);
    bool update_scan(std::vector<LydstoLidarPoint>& points, LydstoLidarFrameMeta* meta = nullptr);
    bool is_ready() const { return initialized_; }

private:
    static constexpr uint8_t kHeader = 0x54;
    static constexpr uint8_t kVerLen = 0x2C;
    static constexpr size_t kFrameSize = 47;
    static constexpr size_t kPointsPerFrame = 12;

    bool setup_uart_();
    bool setup_pwm_();
    bool ingest_uart_bytes_();
    bool parse_next_frame_(std::vector<LydstoLidarPoint>& points, LydstoLidarFrameMeta* meta);

    static uint16_t read_u16_le_(const uint8_t* p);
    static uint8_t crc8_(const uint8_t* data, size_t len);

    LydstoLidarConfig cfg_{};
    bool initialized_ = false;
    bool uart_ready_ = false;

    int ledc_timer_ = LEDC_TIMER_1;
    int ledc_channel_ = LEDC_CHANNEL_1;

    std::vector<uint8_t> rx_buffer_;
    std::vector<uint8_t> scratch_;
};

class LydstoLidarManager {
public:
    static LydstoLidarManager& instance();

    bool configure(const LydstoLidarConfig& config);
    bool start_reading_task(uint32_t read_interval_ms, UBaseType_t priority);
    bool get_latest_scan(std::vector<LydstoLidarPoint>& points, LydstoLidarFrameMeta* meta);

    void pause();
    void resume();
    void stop();

private:
    LydstoLidarManager();
    ~LydstoLidarManager();

    LydstoLidarManager(const LydstoLidarManager&) = delete;
    LydstoLidarManager& operator=(const LydstoLidarManager&) = delete;

    struct TaskParams {
        LydstoLidarManager* manager;
        uint32_t interval_ms;
    };

    static void reading_task(void* param);

    std::unique_ptr<LydstoLidarArray> lidar_;
    std::vector<LydstoLidarPoint> latest_points_;
    LydstoLidarFrameMeta latest_meta_{};

    SemaphoreHandle_t data_mutex_ = nullptr;
    TaskHandle_t task_handle_ = nullptr;
    bool running_ = false;
    bool paused_ = false;
};
