#pragma once

#include <idf_c_includes.hpp>   // now using idf_includes component

// Firmware Version Information – updated to v1.15.0
#define FIRMWARE_VERSION_MAJOR 1
#define FIRMWARE_VERSION_MINOR 15
#define FIRMWARE_VERSION_PATCH 0
#define FIRMWARE_VERSION_BUILD 20260522

// Stringify macros
#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)

// Complete version string
#define FIRMWARE_VERSION_STRING \
    "v" TOSTRING(FIRMWARE_VERSION_MAJOR) "." \
    TOSTRING(FIRMWARE_VERSION_MINOR) "." \
    TOSTRING(FIRMWARE_VERSION_PATCH) "." \
    TOSTRING(FIRMWARE_VERSION_BUILD)

class FirmwareVersion {
private:
    static const char* firmware_version_string;

public:
    FirmwareVersion() = default;

    static const char* get_firmware_version();
    static void print_firmware_version(const char* module_name);
    static const char* get_version_string();
    static void print_version(const char* module_name);

    static unsigned char get_major() { return FIRMWARE_VERSION_MAJOR; }
    static unsigned char get_minor() { return FIRMWARE_VERSION_MINOR; }
    static unsigned char get_patch() { return FIRMWARE_VERSION_PATCH; }
    static unsigned long get_build() { return FIRMWARE_VERSION_BUILD; }
};