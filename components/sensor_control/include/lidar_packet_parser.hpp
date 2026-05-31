#pragma once
#include <idf_c_includes.hpp>
#include <array>

struct LidarParsedPacket {
    bool valid;
    uint16_t speed;
    float start_angle_deg;
    float end_angle_deg;
    std::array<uint16_t, 12> distances_mm;
    std::array<uint8_t, 12> confidences;
    uint16_t timestamp;
    uint8_t crc;
    uint8_t crc_calculated;
    bool crc_valid;
    // No std::string – avoids heap allocation and exceptions
};

class LidarPacketParser {
public:
    static bool parse(const uint8_t* packet, size_t len, LidarParsedPacket& out);
};