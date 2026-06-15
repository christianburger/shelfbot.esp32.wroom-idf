// lidar_packet_parser.cpp
//
// Parses raw 47-byte LYDSTO LDS02RR packets.
//
// Packet layout (47 bytes):
//   [0]     = 0x54  (header magic)
//   [1]     = 0x2C  (header magic / length)
//   [2-3]   = speed (uint16 LE, raw sensor units)
//   [4-5]   = start_angle (uint16 LE, centidegrees → divide by 100 for degrees)
//   [6-41]  = 12 × 3 bytes: distance_lo, distance_hi, confidence
//             distance is uint16 LE in mm; confidence is uint8
//   [42-43] = end_angle (uint16 LE, centidegrees)
//   [44-45] = timestamp (uint16 LE, ms, wraps at 65535)
//   [46]    = CRC (CRC-8/MAXIM polynomial 0x4D, over bytes [0..45])
//
// NOTE: The protocol transmits ONE start_angle and ONE end_angle for all 12
// measurements in the packet.  There is no per-measurement angle field.
// Per-measurement angles must be derived from the batch boundaries —
// see lidar_sensor.cpp for the timestamp-based method used here.

#include "lidar_packet_parser.hpp"
#include "esp_log.h"

static const char* TAG_RAW = "raw_lidar";

static uint8_t crc8_poly4d(const uint8_t* data, size_t len)
{
    uint8_t crc = 0x00;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int b = 0; b < 8; ++b) {
            crc = (crc & 0x80)
                ? static_cast<uint8_t>((crc << 1) ^ 0x4D)
                : static_cast<uint8_t>(crc << 1);
        }
    }
    return crc;
}

bool LidarPacketParser::parse(const uint8_t* p, size_t len, LidarParsedPacket& out)
{
    if (!p || len < 47 || p[0] != 0x54 || p[1] != 0x2C) {
        out.valid = false;
        return false;
    }

    out.valid = true;

    out.speed = static_cast<uint16_t>(p[2]) |
                (static_cast<uint16_t>(p[3]) << 8);

    const uint16_t sa = static_cast<uint16_t>(p[4]) |
                        (static_cast<uint16_t>(p[5]) << 8);
    const uint16_t ea = static_cast<uint16_t>(p[42]) |
                        (static_cast<uint16_t>(p[43]) << 8);
    out.start_angle_deg = static_cast<float>(sa) / 100.0f;
    out.end_angle_deg   = static_cast<float>(ea) / 100.0f;

    for (int i = 0; i < 12; ++i) {
        const int off = 6 + i * 3;
        out.distances_mm[i] = static_cast<uint16_t>(p[off]) |
                               (static_cast<uint16_t>(p[off + 1]) << 8);
        out.confidences[i]  = p[off + 2];
    }

    out.timestamp      = static_cast<uint16_t>(p[44]) |
                         (static_cast<uint16_t>(p[45]) << 8);
    out.crc            = p[46];
    out.crc_calculated = crc8_poly4d(p, 46);
    out.crc_valid      = (out.crc_calculated == out.crc);

    // ── Raw packet diagnostic (compiled out unless LOG_LOCAL_LEVEL >= DEBUG) ──
    // Enable with: idf.py menuconfig → Component config → Log output → Default
    // log verbosity → Debug, or set CONFIG_LOG_DEFAULT_LEVEL_DEBUG=y in sdkconfig.
    // Tag: raw_lidar  — grep for this in the serial output.
    //
    // Each line shows the complete decoded content of one 47-byte packet so
    // you can verify:
    //   - start/end angles increase monotonically within a revolution
    //   - angles wrap cleanly through 0° at each revolution boundary
    //   - all 12 distance and confidence values are non-zero where expected
    //   - CRC passes on every packet (FAIL here means wiring/baud problem)
    ESP_LOGD(TAG_RAW,
             "start=%6.2f end=%6.2f spd=%5u ts=%5u "
             "d=[%4u %4u %4u %4u %4u %4u %4u %4u %4u %4u %4u %4u] "
             "c=[%3u %3u %3u %3u %3u %3u %3u %3u %3u %3u %3u %3u] "
             "crc=%s",
             out.start_angle_deg,
             out.end_angle_deg,
             out.speed,
             out.timestamp,
             out.distances_mm[0],  out.distances_mm[1],  out.distances_mm[2],
             out.distances_mm[3],  out.distances_mm[4],  out.distances_mm[5],
             out.distances_mm[6],  out.distances_mm[7],  out.distances_mm[8],
             out.distances_mm[9],  out.distances_mm[10], out.distances_mm[11],
             out.confidences[0],   out.confidences[1],   out.confidences[2],
             out.confidences[3],   out.confidences[4],   out.confidences[5],
             out.confidences[6],   out.confidences[7],   out.confidences[8],
             out.confidences[9],   out.confidences[10],  out.confidences[11],
             out.crc_valid ? "OK" : "FAIL");

    return true;
}
