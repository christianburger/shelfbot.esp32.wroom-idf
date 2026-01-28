#pragma once

#include <vector>
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"

/**
 * @brief I2C Scanner utility for detecting devices on the I2C bus
 *
 * Uses the new ESP-IDF I2C master driver API for reliable device detection.
 * This is useful for debugging and verifying sensor connectivity.
 * 
 * Also includes support for TOF400F module mode switching from UART to I2C.
 */
class I2CScanner {
public:
    /**
     * @brief Scan the I2C bus for all devices
     *
     * @param port I2C port number
     * @param found_addresses Vector to store found device addresses
     * @param sda_pin SDA GPIO pin
     * @param scl_pin SCL GPIO pin
     * @param freq_hz I2C clock frequency (default: 100kHz for compatibility)
     * @return true if scan completed successfully, false on error
     */
    static bool scan(i2c_port_t port,
                     std::vector<uint8_t>& found_addresses,
                     gpio_num_t sda_pin = GPIO_NUM_21,
                     gpio_num_t scl_pin = GPIO_NUM_22,
                     uint32_t freq_hz = 100000);

    /**
     * @brief Quick scan without returning addresses (just logs)
     *
     * @param port I2C port number
     * @param sda_pin SDA GPIO pin
     * @param scl_pin SCL GPIO pin
     * @param freq_hz I2C clock frequency
     * @return true if scan completed, false on error
     */
    static bool quickScan(i2c_port_t port,
                          gpio_num_t sda_pin = GPIO_NUM_21,
                          gpio_num_t scl_pin = GPIO_NUM_22,
                          uint32_t freq_hz = 100000);

    /**
     * @brief Probe for a specific device address
     *
     * This is faster than a full scan when you know the expected address.
     * Creates a temporary I2C bus, probes the device, and cleans up.
     *
     * @param port I2C port number
     * @param address 7-bit I2C device address
     * @param sda_pin SDA GPIO pin
     * @param scl_pin SCL GPIO pin
     * @param freq_hz I2C clock frequency
     * @return true if device responds at this address, false otherwise
     */
    static bool probe(i2c_port_t port,
                      uint8_t address,
                      gpio_num_t sda_pin = GPIO_NUM_21,
                      gpio_num_t scl_pin = GPIO_NUM_22,
                      uint32_t freq_hz = 100000);

    /**
     * @brief Probe using an existing I2C bus handle
     *
     * Use this when you already have an I2C bus configured.
     * Does not create or destroy the bus.
     *
     * @param bus_handle Existing I2C master bus handle
     * @param address 7-bit I2C device address
     * @param freq_hz I2C clock frequency for the device
     * @return true if device responds, false otherwise
     */
    static bool probeWithBus(i2c_master_bus_handle_t bus_handle,
                             uint8_t address,
                             uint32_t freq_hz = 100000);

    /**
     * @brief Print scan results in a formatted table
     *
     * @param addresses Vector of found addresses
     */
    static void printResults(const std::vector<uint8_t>& addresses);

    /**
     * @brief Get the name of a common I2C device by address
     *
     * @param address 7-bit I2C address
     * @return Device name or "Unknown device"
     */
    static const char* getDeviceName(uint8_t address);

    /**
     * @brief Switch TOF400F module from UART to I2C mode
     * 
     * The TOF400F module (containing VL53L1 sensor) defaults to UART/Modbus mode.
     * This function sends a Modbus command via UART to enable I2C mode.
     * After this command, the VL53L1 sensor becomes accessible via I2C at 0x29.
     * 
     * Procedure:
     * 1. Initialize UART with specified pins and baud rate
     * 2. Send Modbus write command to register 0x0009 with value 0x0001
     * 3. Wait for module to switch modes (~200ms)
     * 4. Clean up UART resources
     * 
     * @param uart_port UART port number
     * @param tx_pin UART TX pin (connects to module RX)
     * @param rx_pin UART RX pin (connects to module TX)
     * @param baud_rate UART baud rate (default 115200 from datasheet)
     * @param slave_addr Modbus slave address (default 0x01)
     * @return true on success, false on failure
     */
    static bool switchTOF400FToI2CMode(uart_port_t uart_port,
                                       gpio_num_t tx_pin,
                                       gpio_num_t rx_pin,
                                       uint32_t baud_rate = 115200,
                                       uint8_t slave_addr = 0x01);

private:
    static const char* TAG;
    
    // Modbus helper functions for TOF400F
    static uint16_t calculateModbusCRC(const uint8_t* data, size_t length);
    static size_t buildModbusWriteCommand(uint8_t* buffer, uint8_t slave_addr,
                                         uint16_t reg_addr, uint16_t value);

    // Common I2C device addresses and names
    struct DeviceInfo {
        uint8_t address;
        const char* name;
    };

    static const DeviceInfo KNOWN_DEVICES[];
    static const size_t NUM_KNOWN_DEVICES;
};
