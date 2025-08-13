#include "ultrasonic_sensors.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_rx.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cstring>

static const char* TAG = "ultrasonic_sensors";

// --- Pin Definitions ---
#define SELECT_PIN_S0 GPIO_NUM_21
#define SELECT_PIN_S1 GPIO_NUM_22
#define SELECT_PIN_S2 GPIO_NUM_23
#define TRIGGER_PIN   GPIO_NUM_32
#define ECHO_PIN      GPIO_NUM_33

// --- RMT Configuration ---
#define RMT_RESOLUTION_HZ 1000000 // 1MHz resolution, 1 tick = 1us
#define RMT_TRIGGER_PULSE_US 10   // 10us trigger pulse

// --- Statically allocated handles ---
static rmt_channel_handle_t tx_channel = NULL;
static rmt_channel_handle_t rx_channel = NULL;
static rmt_encoder_handle_t copy_encoder = NULL;

void ultrasonic_sensors_init() {
    ESP_LOGI(TAG, "Initializing ultrasonic sensor array");

    // --- Configure GPIO pins ---
    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = (1ULL << SELECT_PIN_S0) | (1ULL << SELECT_PIN_S1) | (1ULL << SELECT_PIN_S2) | (1ULL << TRIGGER_PIN);
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io_conf);

    io_conf.pin_bit_mask = (1ULL << ECHO_PIN);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);

    // --- Configure RMT TX Channel ---
    rmt_tx_channel_config_t tx_chan_config = {};
    tx_chan_config.gpio_num = TRIGGER_PIN;
    tx_chan_config.clk_src = RMT_CLK_SRC_DEFAULT;
    tx_chan_config.resolution_hz = RMT_RESOLUTION_HZ;
    tx_chan_config.mem_block_symbols = 64;
    tx_chan_config.trans_queue_depth = 4;
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &tx_channel));

    // --- Configure RMT RX Channel ---
    rmt_rx_channel_config_t rx_chan_config = {};
    rx_chan_config.gpio_num = ECHO_PIN;
    rx_chan_config.clk_src = RMT_CLK_SRC_DEFAULT;
    rx_chan_config.resolution_hz = RMT_RESOLUTION_HZ;
    rx_chan_config.mem_block_symbols = 64;
    ESP_ERROR_CHECK(rmt_new_rx_channel(&rx_chan_config, &rx_channel));

    // --- Create RMT Copy Encoder for the trigger pulse ---
    rmt_copy_encoder_config_t copy_encoder_config = {};
    ESP_ERROR_CHECK(rmt_new_copy_encoder(&copy_encoder_config, &copy_encoder));

    // --- Enable RMT TX channel ---
    ESP_ERROR_CHECK(rmt_enable(tx_channel));
}

// Helper function to select a sensor
static void select_sensor(uint8_t sensor_index) {
    gpio_set_level(SELECT_PIN_S0, (sensor_index & 1) ? 1 : 0);
    gpio_set_level(SELECT_PIN_S1, (sensor_index & 2) ? 1 : 0);
    gpio_set_level(SELECT_PIN_S2, (sensor_index & 4) ? 1 : 0);
}

void ultrasonic_sensors_read_all(float* distances) {
    static rmt_symbol_word_t trigger_pulse = {};
    trigger_pulse.duration0 = RMT_TRIGGER_PULSE_US;
    trigger_pulse.level0 = 1;
    trigger_pulse.duration1 = 0;
    trigger_pulse.level1 = 0;

    rmt_transmit_config_t tx_config = {};
    tx_config.loop_count = 0;

    rmt_receive_config_t rx_config = {};
    rx_config.signal_range_min_ns = 1200;       // Filter out noise: 1.2us
    rx_config.signal_range_max_ns = 23200000; // Max sensor range ~4m (58us/cm * 400cm = 23200us)

    for (int i = 0; i < NUM_ULTRASONIC_SENSORS; i++) {
        select_sensor(i);
        vTaskDelay(pdMS_TO_TICKS(30)); // Wait for multiplexer to settle

        // Enable the receiver
        ESP_ERROR_CHECK(rmt_enable(rx_channel));

        // Send the trigger pulse
        ESP_ERROR_CHECK(rmt_transmit(tx_channel, copy_encoder, &trigger_pulse, sizeof(trigger_pulse), &tx_config));

        // Wait for the echo pulse (blocking with a timeout)
        rmt_symbol_word_t rx_symbol;
        if (rmt_receive(rx_channel, &rx_symbol, sizeof(rx_symbol), &rx_config) == ESP_OK) {
            // Echo received, calculate distance
            uint32_t duration_us = (rx_symbol.level0 == 1) ? rx_symbol.duration0 : rx_symbol.duration1;
            distances[i] = (float)duration_us / 58.0f;
        } else {
            // Timeout or error, report invalid distance
            distances[i] = -1.0f;
        }

        // Disable the receiver to ensure a clean state for the next reading
        ESP_ERROR_CHECK(rmt_disable(rx_channel));
    }
}
