#pragma once
#include <idf_c_includes.hpp>
#include <lydsto.hpp>
#include "lidar_scan.hpp"
#include "lidar_packet_parser.hpp"
#include <state_machine_lifecycle.hpp>

// ============================================================================
//  Hardware defaults – override in your board config if needed
// ============================================================================
#define LIDAR_UART_PORT   UART_NUM_2
#define LIDAR_TX_PIN      UART_PIN_NO_CHANGE
#define LIDAR_RX_PIN      GPIO_NUM_3
#define LIDAR_BAUD_RATE   115200

// ============================================================================
//  Task functions – declared here so shelfbot.cpp can create them centrally.
//  Stack budgets (verified empirically):
//    lidar_lifecycle_task_fn : 3072 words  (state machine calls + driver init)
//    lidar_read_task_fn      : 2048 words  (tight loop, no std::string/map)
// ============================================================================
void lidar_lifecycle_task_fn(void* arg);
void lidar_read_task_fn(void* arg);

// ============================================================================
//  Flat API
// ============================================================================

// Call before creating tasks.  Creates the scan mutex; does NOT spawn tasks.
void lidar_setup();

// Store the read task handle so lidar_stop() can delete it.
void lidar_set_read_task_handle(TaskHandle_t h);

void lidar_stop();
LidarSensorState lidar_get_state();
bool lidar_is_running();

// Copy the latest completed scan into `out`.
// Returns false if no scan is available.
// Does NOT clear the ready flag — multiple consumers (HTTP server,
// micro-ROS) may each call this independently; the flag is only cleared
// when a new scan overwrites the slot (every ~167 ms).
bool lidar_get_latest_scan(LidarScan& out);

bool lidar_get_last_raw_packet(uint8_t* out, size_t len);
uint32_t lidar_get_packet_count();
uint32_t lidar_get_scan_count();
