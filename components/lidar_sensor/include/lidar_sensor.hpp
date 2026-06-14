#pragma once
#include <idf_c_includes.hpp>
#include <lydsto.hpp>
#include "lidar_scan.hpp"
#include "lidar_packet_parser.hpp"
#include <state_machine_lifecycle.hpp>   // provides LidarSensorState and stateToString

// ============================================================================
//  Hardware defaults – override in your board config if needed
// ============================================================================
#define LIDAR_UART_PORT   UART_NUM_2
#define LIDAR_TX_PIN      UART_PIN_NO_CHANGE
#define LIDAR_RX_PIN      GPIO_NUM_3
#define LIDAR_BAUD_RATE   115200

// ============================================================================
//  Flat API (no state enum definition here – it's in state_machine_lifecycle.hpp)
// ============================================================================

void lidar_setup();
void lidar_stop();
LidarSensorState lidar_get_state();
bool lidar_is_running();
bool lidar_get_latest_scan(LidarScan& out);
bool lidar_get_last_raw_packet(uint8_t* out, size_t len);
uint32_t lidar_get_packet_count();
uint32_t lidar_get_scan_count();