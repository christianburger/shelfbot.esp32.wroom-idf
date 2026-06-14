#pragma once
#include <idf_c_includes.hpp>
#include <lydsto.hpp>
#include "lidar_scan.hpp"
#include "lidar_packet_parser.hpp"

// ============================================================================
//  Hardware defaults – override in your board config if needed
// ============================================================================
#define LIDAR_UART_PORT   UART_NUM_2
#define LIDAR_TX_PIN      UART_PIN_NO_CHANGE
#define LIDAR_RX_PIN      GPIO_NUM_3
#define LIDAR_BAUD_RATE   115200

// ============================================================================
//  Component state (mirrors motor_control pattern)
// ============================================================================

/** Current operating state of the lidar component. */
enum class LidarState : uint8_t {
    UNINIT = 0,   ///< Before lidar_setup() is called
    IDLE,         ///< Initialised, not yet reading
    RUNNING,      ///< Continuous read task active
    ERROR         ///< Driver or UART failure
};

// ============================================================================
//  Flat API
// ============================================================================

/**
 * @brief Lifecycle – called once at system startup (spawns the lifecycle task).
 *        Mirrors motor_control_setup().
 */
void lidar_setup();

/** Tear down the read task and UART driver. */
void lidar_stop();

/** @return Current component state. */
LidarState lidar_get_state();

/** @return true when the read task is running and packets are flowing. */
bool lidar_is_running();

/**
 * @brief Copy the most-recently-completed 360° scan into @p out.
 *
 * Thread-safe; can be called from any task.
 *
 * @param out  Destination scan struct – only written when a new scan is available.
 * @return     true if a completed scan was available and copied.
 */
bool lidar_get_latest_scan(LidarScan& out);

/**
 * @brief Copy the raw last LYDSTO packet (47 bytes) into @p out.
 *
 * Useful for diagnostics and for callers that want to re-parse the packet
 * themselves.
 *
 * @param out  Buffer of at least 47 bytes.
 * @param len  Size of @p out; must be >= 47.
 * @return     true if a packet has been received.
 */
bool lidar_get_last_raw_packet(uint8_t* out, size_t len);

/** @return Total number of valid packets received since startup. */
uint32_t lidar_get_packet_count();

/** @return Total number of completed 360° scans since startup. */
uint32_t lidar_get_scan_count();
