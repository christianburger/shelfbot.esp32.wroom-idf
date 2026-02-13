#include "firmware_version.hpp"

// Initialize static member
const char* FirmwareVersion::firmware_version_string = FIRMWARE_VERSION_STRING;

// Static TAG for logging
static const char* TAG = "FirmwareVersion";

// Instance method to get version string
const char* FirmwareVersion::get_firmware_version() const {
  return firmware_version_string;
}

// Instance method to print version
void FirmwareVersion::print_firmware_version(const char* module_name) const {
  ESP_LOGI(TAG, "%s initialized - Firmware Version: %s",
           module_name, firmware_version_string);
}

// Static method to get version string
const char* FirmwareVersion::get_version_string() {
  return firmware_version_string;
}

// Static method to print version
void FirmwareVersion::print_version(const char* module_name) {
  ESP_LOGI(TAG, "%s initialized - Firmware Version: %s",
           module_name, firmware_version_string);
}