#pragma once

#include <idf_c_includes.hpp>
#include <sensor_common.hpp>
#include <sensor_interfaces.hpp>
#include <lydsto.hpp>

class LidarSensor : public ILidarSensor {
public:
    explicit LidarSensor(uart_port_t uart_port, int tx_pin, int rx_pin, uint32_t baud_rate);
    esp_err_t initialize() override;
    bool isReady() const override { return initialized_ && driver_.isReady(); }
    esp_err_t read(SensorCommon::LidarMeasurement& out) override;
    bool getLastRawPacket(uint8_t* out, size_t len) const override { return driver_.get_last_packet(out, len); }
    uint32_t getPacketCount() const override { return driver_.get_packet_count(); }
private:
    LYDSTO_Driver driver_;
    bool initialized_;
    uart_port_t uart_port_;
    int tx_pin_;
    int rx_pin_;
    uint32_t baud_rate_;
};