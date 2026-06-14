#pragma once
#include <cstdint>
#include <cstddef>
#include <array>

/**
 * @brief Parsed representation of a single LYDSTO LDS02RR packet (47 bytes).
 *
 * Contains 12 distance/confidence samples spanning a small angular arc.
 * CRC validity is checked during parsing; callers should reject packets
 * where crc_valid == false.
 */
struct LidarParsedPacket {
    bool     valid;
    uint16_t speed;               // rotational speed (raw sensor units)
    float    start_angle_deg;     // angle of first sample  [0, 360)
    float    end_angle_deg;       // angle of last  sample  [0, 360)
    std::array<uint16_t, 12> distances_mm;
    std::array<uint8_t,  12> confidences;
    uint16_t timestamp;           // sensor-internal timestamp (ms)
    uint8_t  crc;                 // received CRC byte
    uint8_t  crc_calculated;      // locally computed CRC
    bool     crc_valid;
};

class LidarPacketParser {
public:
    /**
     * @brief Parse a raw 47-byte LYDSTO packet.
     *
     * @param packet  Pointer to exactly 47 bytes.
     * @param len     Must be >= 47; extra bytes are ignored.
     * @param out     Populated on success.
     * @return true if the header bytes match and the packet is well-formed
     *         (note: crc_valid inside @p out tells you whether the CRC passed).
     */
    static bool parse(const uint8_t* packet, size_t len, LidarParsedPacket& out);
};
