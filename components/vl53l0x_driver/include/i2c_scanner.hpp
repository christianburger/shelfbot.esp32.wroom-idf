#pragma once

#include <vector>
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_log.h"

class I2CScanner {
public:
  static bool scan(i2c_port_t port,
                  std::vector<uint8_t>& found_addresses,
                  gpio_num_t sda_pin = GPIO_NUM_21,
                  gpio_num_t scl_pin = GPIO_NUM_22);

  static bool quickScan(i2c_port_t port,
                       gpio_num_t sda_pin = GPIO_NUM_21,
                       gpio_num_t scl_pin = GPIO_NUM_22);

  static bool probe(i2c_port_t port,
                   uint8_t address,
                   gpio_num_t sda_pin = GPIO_NUM_21,
                   gpio_num_t scl_pin = GPIO_NUM_22);

  static void printResults(const std::vector<uint8_t>& addresses);

private:
  static const char* TAG;
};