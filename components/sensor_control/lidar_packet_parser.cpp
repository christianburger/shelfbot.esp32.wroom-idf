#include <lidar_packet_parser.hpp>

static uint8_t crc8_poly4d(const uint8_t* data, size_t len) {
    uint8_t crc = 0x00;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int b = 0; b < 8; ++b) {
            crc = (crc & 0x80) ? static_cast<uint8_t>((crc << 1) ^ 0x4D) : static_cast<uint8_t>(crc << 1);
        }
    }
    return crc;
}

bool LidarPacketParser::parse(const uint8_t* p, size_t len, LidarParsedPacket& out) {
    if (!p || len < 47 || p[0] != 0x54 || p[1] != 0x2C) {
        return false;
    }

    out.valid = true;
    out.speed = p[2] | (static_cast<uint16_t>(p[3]) << 8);
    uint16_t sa = p[4] | (static_cast<uint16_t>(p[5]) << 8);
    uint16_t ea = p[42] | (static_cast<uint16_t>(p[43]) << 8);
    out.start_angle_deg = sa / 100.0f;
    out.end_angle_deg = ea / 100.0f;

    for (int i = 0; i < 12; ++i) {
        int off = 6 + i * 3;
        out.distances_mm[i] = p[off] | (static_cast<uint16_t>(p[off + 1]) << 8);
        out.confidences[i] = p[off + 2];
    }

    out.timestamp = p[44] | (static_cast<uint16_t>(p[45]) << 8);
    out.crc = p[46];
    out.crc_calculated = crc8_poly4d(p, 46);
    out.crc_valid = (out.crc_calculated == out.crc);

    return true;
}