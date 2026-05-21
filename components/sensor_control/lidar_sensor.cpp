#include <lidar_sensor.hpp>
#include <lidar_packet_parser.hpp>

LidarSensor::LidarSensor(uart_port_t uart_port, int tx_pin, int rx_pin, uint32_t baud_rate)
    : driver_(uart_port, tx_pin, rx_pin, baud_rate), initialized_(false),
      uart_port_(uart_port), tx_pin_(tx_pin), rx_pin_(rx_pin), baud_rate_(baud_rate) {}

esp_err_t LidarSensor::initialize() {
    const char* err = driver_.init();
    initialized_ = (err == nullptr);
    return initialized_ ? ESP_OK : ESP_FAIL;
}

esp_err_t LidarSensor::read(SensorCommon::LidarMeasurement& out) {
    if (!initialized_) return ESP_ERR_INVALID_STATE;
    LYDSTO_Driver::MeasurementResult m{};
    bool ok = driver_.read_sensor(m);
    out.active = true;
    out.valid = ok && m.valid;
    out.distance_mm = m.distance_mm;
    out.status = m.range_status;
    out.timestamp_us = m.timestamp_us;
    out.timeout_occurred = m.timeout_occurred;
    out.start_angle_deg = m.start_angle_deg;
    out.end_angle_deg = m.end_angle_deg;
    out.min_distance_angle_deg = m.min_distance_angle_deg;
    out.has_packet_points = false;
    uint8_t raw[47];
    if (driver_.get_last_packet(raw, sizeof(raw))) {
        LidarParsedPacket parsed{};
        if (LidarPacketParser::parse(raw, sizeof(raw), parsed)) {
            out.has_packet_points = true;
            out.packet_speed = parsed.speed;
            out.packet_timestamp = parsed.timestamp;
            out.packet_crc = parsed.crc;
            for (int i = 0; i < 12; ++i) {
                out.packet_distances_mm[i] = parsed.distances_mm[i];
                out.packet_confidences[i] = parsed.confidences[i];
            }
        }
    }
    out.health = out.valid ? 1 : 2;
    return ok ? ESP_OK : ESP_FAIL;
}