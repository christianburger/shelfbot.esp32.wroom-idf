#pragma once

#ifndef FIRMWARE_VERSION_HPP
#define FIRMWARE_VERSION_HPP

#include "idf_c_includes.hpp"

// Firmware Version Information
#define FIRMWARE_VERSION_MAJOR 1
#define FIRMWARE_VERSION_MINOR 9
#define FIRMWARE_VERSION_PATCH 0
#define FIRMWARE_VERSION_BUILD 20260214

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
  // Static member for the version string
  static const char* firmware_version_string;

public:
  // Constructor
  FirmwareVersion() = default;

  // Get firmware version as string
  const char* get_firmware_version() const;

  // Print firmware version with module name
  void print_firmware_version(const char* module_name) const;

  // Static methods for accessing version without instantiation
  static const char* get_version_string();
  static void print_version(const char* module_name);

  // Get individual version components
  static uint8_t get_major() { return FIRMWARE_VERSION_MAJOR; }
  static uint8_t get_minor() { return FIRMWARE_VERSION_MINOR; }
  static uint8_t get_patch() { return FIRMWARE_VERSION_PATCH; }
  static uint32_t get_build() { return FIRMWARE_VERSION_BUILD; }
};

#endif // FIRMWARE_VERSION_HPP
