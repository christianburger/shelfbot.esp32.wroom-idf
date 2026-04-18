#include "lydsto_lidar.hpp"

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <algorithm>
#include <cmath>
#include <inttypes.h>

static const char* TAG = "LydstoLidar";

namespace {
constexpr uint8_t CRC_TABLE[256] = {
    0x00, 0x4d, 0x9a, 0xd7, 0x79, 0x34, 0xe3, 0xae, 0xf2, 0xbf, 0x68, 0x25, 0x8b, 0xc6, 0x11, 0x5c,
    0xa9, 0xe4, 0x33, 0x7e, 0xd0, 0x9d, 0x4a, 0x07, 0x5b, 0x16, 0xc1, 0x8c, 0x22, 0x6f, 0xb8, 0xf5,
    0x1f, 0x52, 0x85, 0xc8, 0x66, 0x2b, 0xfc, 0xb1, 0xed, 0xa0, 0x77, 0x3a, 0x94, 0xd9, 0x0e, 0x43,
    0xb6, 0xfb, 0x2c, 0x61, 0xcf, 0x82, 0x55, 0x18, 0x44, 0x09, 0xde, 0x93, 0x3d, 0x70, 0xa7, 0xea,
    0x3e, 0x73, 0xa4, 0xe9, 0x47, 0x0a, 0xdd, 0x90, 0xcc, 0x81, 0x56, 0x1b, 0xb5, 0xf8, 0x2f, 0x62,
    0x97, 0xda, 0x0d, 0x40, 0xee, 0xa3, 0x74, 0x39, 0x65, 0x28, 0xff, 0xb2, 0x1c, 0x51, 0x86, 0xcb,
    0x21, 0x6c, 0xbb, 0xf6, 0x58, 0x15, 0xc2, 0x8f, 0xd3, 0x9e, 0x49, 0x04, 0xaa, 0xe7, 0x30, 0x7d,
    0x88, 0xc5, 0x12, 0x5f, 0xf1, 0xbc, 0x6b, 0x26, 0x7a, 0x37, 0xe0, 0xad, 0x03, 0x4e, 0x99, 0xd4,
    0x7c, 0x31, 0xe6, 0xab, 0x05, 0x48, 0x9f, 0xd2, 0x8e, 0xc3, 0x14, 0x59, 0xf7, 0xba, 0x6d, 0x20,
    0xd5, 0x98, 0x4f, 0x02, 0xac, 0xe1, 0x36, 0x7b, 0x27, 0x6a, 0xbd, 0xf0, 0x5e, 0x13, 0xc4, 0x89,
    0x63, 0x2e, 0xf9, 0xb4, 0x1a, 0x57, 0x80, 0xcd, 0x91, 0xdc, 0x0b, 0x46, 0xe8, 0xa5, 0x72, 0x3f,
    0xca, 0x87, 0x50, 0x1d, 0xb3, 0xfe, 0x29, 0x64, 0x38, 0x75, 0xa2, 0xef, 0x41, 0x0c, 0xdb, 0x96,
    0x42, 0x0f, 0xd8, 0x95, 0x3b, 0x76, 0xa1, 0xec, 0xb0, 0xfd, 0x2a, 0x67, 0xc9, 0x84, 0x53, 0x1e,
    0xeb, 0xa6, 0x71, 0x3c, 0x92, 0xdf, 0x08, 0x45, 0x19, 0x54, 0x83, 0xce, 0x60, 0x2d, 0xfa, 0xb7,
    0x5d, 0x10, 0xc7, 0x8a, 0x24, 0x69, 0xbe, 0xf3, 0xaf, 0xe2, 0x35, 0x78, 0xd6, 0x9b, 0x4c, 0x01,
    0xf4, 0xb9, 0x6e, 0x23, 0x8d, 0xc0, 0x17, 0x5a, 0x06, 0x4b, 0x9c, 0xd1, 0x7f, 0x32, 0xe5, 0xa8,
};

constexpr uint8_t FRAME_HEADER = 0x54;
constexpr uint8_t FRAME_VERLEN = 0x2C;
constexpr size_t FRAME_SIZE = 47;
constexpr size_t POINTS_PER_FRAME = 12;

uint16_t read_u16_le(const uint8_t* p) {
    return static_cast<uint16_t>((static_cast<uint16_t>(p[1]) << 8) | p[0]);
}

uint8_t crc8(const uint8_t* data, size_t len) {
    uint8_t crc = 0;
    for (size_t i = 0; i < len; ++i) {
        crc = CRC_TABLE[(crc ^ data[i]) & 0xFF];
    }
    return crc;
}
} // namespace

struct LydstoLidar::Impl {
    explicit Impl(const Config& cfg) : config(cfg) {}

    Config config;
    bool initialized = false;
    bool uart_ready = false;

    ledc_timer_t ledc_timer = LEDC_TIMER_1;
    ledc_channel_t ledc_channel = LEDC_CHANNEL_1;

    std::vector<uint8_t> rx_buffer;
    std::vector<uint8_t> scratch;

    bool setup_uart() {
        uart_config_t uart_cfg = {};
        uart_cfg.baud_rate = config.baud_rate;
        uart_cfg.data_bits = UART_DATA_8_BITS;
        uart_cfg.parity = UART_PARITY_DISABLE;
        uart_cfg.stop_bits = UART_STOP_BITS_1;
        uart_cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
        uart_cfg.rx_flow_ctrl_thresh = 0;
        uart_cfg.source_clk = UART_SCLK_DEFAULT;
#if SOC_UART_SUPPORT_FSM_TX_WAIT_SEND
        uart_cfg.flags = 0;
#endif

        esp_err_t ret = uart_param_config(config.uart_port, &uart_cfg);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "uart_param_config failed: %s", esp_err_to_name(ret));
            return false;
        }

        ret = uart_set_pin(config.uart_port,
                           config.tx_pin,
                           config.rx_pin,
                           UART_PIN_NO_CHANGE,
                           UART_PIN_NO_CHANGE);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "uart_set_pin failed: %s", esp_err_to_name(ret));
            return false;
        }

        ret = uart_driver_install(config.uart_port,
                                  config.uart_rx_buffer_size,
                                  0,
                                  0,
                                  nullptr,
                                  0);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "uart_driver_install failed: %s", esp_err_to_name(ret));
            return false;
        }

        uart_ready = true;
        return true;
    }

    bool setup_pwm() {
        ledc_timer_config_t timer_cfg = {};
        timer_cfg.speed_mode = LEDC_LOW_SPEED_MODE;
        timer_cfg.duty_resolution = LEDC_TIMER_10_BIT;
        timer_cfg.timer_num = ledc_timer;
        timer_cfg.freq_hz = config.pwm_frequency_hz;
        timer_cfg.clk_cfg = LEDC_AUTO_CLK;

        esp_err_t ret = ledc_timer_config(&timer_cfg);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "ledc_timer_config failed: %s", esp_err_to_name(ret));
            return false;
        }

        uint32_t max_duty = (1u << LEDC_TIMER_10_BIT) - 1u;
        uint32_t duty = static_cast<uint32_t>(std::clamp(config.pwm_duty_cycle, 0.0f, 1.0f) * max_duty);

        ledc_channel_config_t ch_cfg = {};
        ch_cfg.gpio_num = config.pwm_pin;
        ch_cfg.speed_mode = LEDC_LOW_SPEED_MODE;
        ch_cfg.channel = ledc_channel;
        ch_cfg.intr_type = LEDC_INTR_DISABLE;
        ch_cfg.timer_sel = ledc_timer;
        ch_cfg.duty = duty;
        ch_cfg.hpoint = 0;

        ret = ledc_channel_config(&ch_cfg);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "ledc_channel_config failed: %s", esp_err_to_name(ret));
            return false;
        }

        ESP_LOGI(TAG, "PWM enabled at %" PRIu32 " Hz duty=%.1f%%",
                 config.pwm_frequency_hz, config.pwm_duty_cycle * 100.0f);
        return true;
    }

    bool ingest_uart() {
        size_t available = 0;
        esp_err_t ret = uart_get_buffered_data_len(config.uart_port, &available);
        if (ret != ESP_OK || available == 0) {
            return false;
        }

        const int to_read = static_cast<int>(std::min<size_t>(available, scratch.size()));
        const int got = uart_read_bytes(config.uart_port,
                                        scratch.data(),
                                        to_read,
                                        pdMS_TO_TICKS(config.uart_read_timeout_ms));
        if (got <= 0) {
            return false;
        }

        rx_buffer.insert(rx_buffer.end(), scratch.begin(), scratch.begin() + got);
        if (rx_buffer.size() > 8 * FRAME_SIZE) {
            rx_buffer.erase(rx_buffer.begin(), rx_buffer.begin() + (rx_buffer.size() - 8 * FRAME_SIZE));
        }

        return true;
    }

    bool parse_one_frame(MeasurementResult& result) {
        if (rx_buffer.size() < FRAME_SIZE) {
            return false;
        }

        size_t idx = 0;
        bool found = false;
        for (; idx + 1 < rx_buffer.size(); ++idx) {
            if (rx_buffer[idx] == FRAME_HEADER && rx_buffer[idx + 1] == FRAME_VERLEN) {
                found = true;
                break;
            }
        }

        if (!found) {
            rx_buffer.clear();
            return false;
        }

        if (idx > 0) {
            rx_buffer.erase(rx_buffer.begin(), rx_buffer.begin() + idx);
        }
        if (rx_buffer.size() < FRAME_SIZE) {
            return false;
        }

        const uint8_t* frame = rx_buffer.data();
        const bool crc_ok = (crc8(frame, FRAME_SIZE - 1) == frame[FRAME_SIZE - 1]);
        if (!crc_ok) {
            rx_buffer.erase(rx_buffer.begin());
            return false;
        }

        const uint16_t speed = read_u16_le(frame + 2);
        const uint16_t start_angle = read_u16_le(frame + 4);
        const uint16_t end_angle = read_u16_le(frame + 42);

        const uint16_t sweep = (end_angle >= start_angle)
            ? static_cast<uint16_t>(end_angle - start_angle)
            : static_cast<uint16_t>((36000 + end_angle) - start_angle);
        const float step = static_cast<float>(sweep) / static_cast<float>(POINTS_PER_FRAME - 1);

        result.points.clear();
        result.points.reserve(POINTS_PER_FRAME);
        result.timestamp_us = static_cast<uint32_t>(esp_timer_get_time());
        result.speed_dps = static_cast<float>(speed);
        result.start_angle_deg = static_cast<float>(start_angle) / 100.0f;
        result.end_angle_deg = static_cast<float>(end_angle) / 100.0f;
        result.timeout_occurred = false;

        bool has_valid = false;
        for (size_t i = 0; i < POINTS_PER_FRAME; ++i) {
            const size_t point_off = 6 + (i * 3);
            const uint16_t distance_mm = read_u16_le(frame + point_off);
            const uint8_t intensity = frame[point_off + 2];

            float angle_cdeg = std::fmod(static_cast<float>(start_angle) + (step * static_cast<float>(i)), 36000.0f);
            if (angle_cdeg < 0.0f) {
                angle_cdeg += 36000.0f;
            }

            const bool valid = (distance_mm >= config.min_distance_mm &&
                                distance_mm <= config.max_distance_mm &&
                                intensity >= config.min_intensity);

            LydstoLidar::Point p = {
                .angle_deg = angle_cdeg / 100.0f,
                .distance_mm = distance_mm,
                .intensity = intensity,
                .valid = valid,
            };
            result.points.push_back(p);
            has_valid = has_valid || valid;
        }

        result.valid = has_valid;
        rx_buffer.erase(rx_buffer.begin(), rx_buffer.begin() + FRAME_SIZE);
        return true;
    }
};

LydstoLidar::LydstoLidar(const Config& config)
    : pimpl_(std::make_unique<Impl>(config)) {}

LydstoLidar::~LydstoLidar() {
    if (pimpl_ && pimpl_->uart_ready) {
        uart_driver_delete(pimpl_->config.uart_port);
    }
}

const char* LydstoLidar::init() {
    pimpl_->rx_buffer.clear();
    pimpl_->rx_buffer.reserve(2048);
    pimpl_->scratch.resize(256);

    if (!pimpl_->setup_uart()) {
        return "UART setup failed";
    }

    if (pimpl_->config.enable_external_speed_control && !pimpl_->setup_pwm()) {
        return "PWM setup failed";
    }

    pimpl_->initialized = true;
    ESP_LOGI(TAG, "Initialized (uart=%d baud=%d rx=%d tx=%d pwm=%d)",
             static_cast<int>(pimpl_->config.uart_port),
             pimpl_->config.baud_rate,
             static_cast<int>(pimpl_->config.rx_pin),
             static_cast<int>(pimpl_->config.tx_pin),
             static_cast<int>(pimpl_->config.pwm_pin));
    return nullptr;
}

bool LydstoLidar::isReady() const {
    return pimpl_ && pimpl_->initialized;
}

bool LydstoLidar::readSingle(MeasurementResult& result) {
    if (!isReady()) {
        result.valid = false;
        result.timeout_occurred = true;
        return false;
    }

    pimpl_->ingest_uart();
    const bool ok = pimpl_->parse_one_frame(result);
    if (!ok) {
        result.timeout_occurred = true;
        result.valid = false;
    }
    return ok;
}

const char* LydstoLidar::selfTest() {
    if (!isReady()) {
        return "Driver not initialized";
    }

    MeasurementResult result;
    for (int i = 0; i < 20; ++i) {
        if (readSingle(result) && result.valid) {
            return nullptr;
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    return "No valid frame received from LiDAR";
}

LydstoLidar::Config lydsto_default_config(uart_port_t uart_port,
                                          gpio_num_t rx_pin,
                                          gpio_num_t pwm_pin) {
    return LydstoLidar::Config{
        .uart_port = uart_port,
        .baud_rate = 230400,
        .rx_pin = rx_pin,
        .tx_pin = GPIO_NUM_NC,
        .pwm_pin = pwm_pin,
        .uart_rx_buffer_size = 4096,
        .uart_read_timeout_ms = 20,
        .enable_external_speed_control = false,
        .pwm_frequency_hz = 30000,
        .pwm_duty_cycle = 0.50f,
        .min_distance_mm = 150,
        .max_distance_mm = 8000,
        .min_intensity = 1,
    };
}
