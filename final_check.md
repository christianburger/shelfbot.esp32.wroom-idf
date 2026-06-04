# Shelfbot Firmware — Final Integration Check Guide

## Quick reference: QoS matrix

| Topic | Publisher QoS | Known subscriber | Subscriber QoS | Compatible |
|---|---|---|---|---|
| `heartbeat` | BEST_EFFORT | — | — | ✅ |
| `motor_positions` | BEST_EFFORT | `shelfbot_hardware_interface_microros_node` | BEST_EFFORT | ✅ |
| `distance_sensors` | BEST_EFFORT | — | — | ✅ |
| `led_state` | BEST_EFFORT | — | — | ✅ |
| `tof_distance` | BEST_EFFORT | — | — | ✅ |
| `laser_scan` | **RELIABLE** | `lidar_relay_node` | **RELIABLE** | ✅ |
| `motor_command` *(sub)* | hardware_interface RELIABLE | firmware RELIABLE | — | ✅ |
| `set_speed` *(sub)* | hardware_interface RELIABLE | firmware RELIABLE | — | ✅ |
| `led` *(sub)* | caller RELIABLE | firmware RELIABLE | — | ✅ |

> **Rule:** BEST_EFFORT pub + RELIABLE sub = **silent data loss** (incompatible under DDS).
> The previous firmware used `rclc_publisher_init_default` (RELIABLE) for all publishers.
> `motor_positions` was silently broken because the hardware interface subscribes BEST_EFFORT.
> Fixed: all publishers except `laser_scan` now use `rclc_publisher_init_best_effort`.
> `laser_scan` stays RELIABLE to match `lidar_relay_node`.

---

## Prerequisites

```bash
source /opt/ros/humble/setup.bash

# Start the micro-ROS agent with --time (required for clock sync)
docker run -it --rm --net=host microros/micro-ros-agent:humble udp4 --port 8888 --time -v6
```

Wait for the firmware log to show:
```
micro-ROS ready (time_synced=yes)
```
before running any checks below. If it shows `time_synced=no`, sensor topics will be silent.

---

## Section 1 — Node and topic discovery

```bash
# Confirm node is live
ros2 node list | grep shelfbot_firmware

# Full publisher/subscriber listing
ros2 node info /shelfbot_firmware

# All topics with types
ros2 topic list -t | grep shelfbot_firmware
```

Expected output from `node info`:
```
/shelfbot_firmware
  Subscribers:
    /shelfbot_firmware/led               std_msgs/msg/Bool
    /shelfbot_firmware/motor_command     std_msgs/msg/Float32MultiArray
    /shelfbot_firmware/set_speed         std_msgs/msg/Float32MultiArray
  Publishers:
    /shelfbot_firmware/distance_sensors  std_msgs/msg/Float32MultiArray
    /shelfbot_firmware/heartbeat         std_msgs/msg/Int32
    /shelfbot_firmware/laser_scan        sensor_msgs/msg/LaserScan
    /shelfbot_firmware/led_state         std_msgs/msg/Bool
    /shelfbot_firmware/motor_positions   std_msgs/msg/Float32MultiArray
    /shelfbot_firmware/tof_distance      std_msgs/msg/Float32
```

---

## Section 2 — QoS profile verification

Run `--verbose` on every publisher. Verify reliability matches the table above.

```bash
ros2 topic info /shelfbot_firmware/heartbeat        --verbose
ros2 topic info /shelfbot_firmware/motor_positions  --verbose
ros2 topic info /shelfbot_firmware/distance_sensors --verbose
ros2 topic info /shelfbot_firmware/led_state        --verbose
ros2 topic info /shelfbot_firmware/tof_distance     --verbose
ros2 topic info /shelfbot_firmware/laser_scan       --verbose
```

Expected reliability per topic:

```
heartbeat        → Reliability: BEST_EFFORT
motor_positions  → Reliability: BEST_EFFORT
distance_sensors → Reliability: BEST_EFFORT
led_state        → Reliability: BEST_EFFORT
tof_distance     → Reliability: BEST_EFFORT
laser_scan       → Reliability: RELIABLE
```

If any topic still shows `RELIABLE` where `BEST_EFFORT` is expected, the firmware has
not been rebuilt with the updated `microros_sync.cpp`.

### Check for QoS incompatibility events

```bash
# Global health check — reports any incompatible QoS pairings
ros2 doctor --report | grep -A10 "QoS"

# Or use rqt_graph: dashed lines between nodes = incompatible QoS
ros2 run rqt_graph rqt_graph
```

---

## Section 3 — Message receipt and publish rates

Since `heartbeat`, `motor_positions`, `distance_sensors`, `led_state`, and `tof_distance`
are BEST_EFFORT, use `--qos-reliability best_effort` when subscribing from the command line.
`laser_scan` is RELIABLE so no flag is needed (default).

```bash
# Rates
ros2 topic hz /shelfbot_firmware/heartbeat \
  --qos-reliability best_effort --qos-durability volatile
# expected: ~1 Hz

ros2 topic hz /shelfbot_firmware/motor_positions \
  --qos-reliability best_effort --qos-durability volatile
# expected: ~10 Hz  (only after clock sync)

ros2 topic hz /shelfbot_firmware/distance_sensors \
  --qos-reliability best_effort --qos-durability volatile
# expected: ~5 Hz  (only after clock sync)

ros2 topic hz /shelfbot_firmware/tof_distance \
  --qos-reliability best_effort --qos-durability volatile
# expected: ~5 Hz  (only after clock sync)

ros2 topic hz /shelfbot_firmware/laser_scan
# expected: ~5 Hz  (only after clock sync; no QoS flag — RELIABLE default)
```

---

## Section 4 — Message content

```bash
# Heartbeat — incrementing int32
ros2 topic echo /shelfbot_firmware/heartbeat \
  --qos-reliability best_effort --qos-durability volatile

# Motor positions — 5 floats in radians
ros2 topic echo /shelfbot_firmware/motor_positions \
  --qos-reliability best_effort --qos-durability volatile

# Distance sensors — 6 floats in cm: [us0, us1, us2, us3, tof_cm, lidar_min_cm]
# -1.0 = sensor invalid/disabled
ros2 topic echo /shelfbot_firmware/distance_sensors \
  --qos-reliability best_effort --qos-durability volatile

# ToF distance — single float in metres (-1.0 = invalid)
ros2 topic echo /shelfbot_firmware/tof_distance \
  --qos-reliability best_effort --qos-durability volatile

# LED state
ros2 topic echo /shelfbot_firmware/led_state \
  --qos-reliability best_effort --qos-durability volatile

# LaserScan — full message (no QoS flag needed)
ros2 topic echo /shelfbot_firmware/laser_scan --once
```

---

## Section 5 — Timestamp verification (laser_scan)

`laser_scan` is the only topic with a `header.stamp`. The stamp must be wall-clock epoch
(seconds since 1970), not boot-relative (seconds since power-on, which would be a small number).

```bash
# Extract stamp and compare to host clock in one pipeline
ros2 topic echo /shelfbot_firmware/laser_scan \
  --once 2>/dev/null \
  | awk '/^  sec:/{sec=$2} /^  nanosec:/{ns=$2}
         END{printf "Stamp epoch : %.3f\n", sec + ns/1e9}'

echo "Host epoch  : $(date +%s)"
```

Expected result — stamp and host should agree to within ~1 second:
```
Stamp epoch : 1748947312.481    ✅  (large Unix timestamp)
Host epoch  : 1748947312.000
```

Failure signature — stamp is boot-relative:
```
Stamp epoch : 47.123            ❌  (seconds since boot, not wall clock)
```

If you see the failure signature: the agent was not started with `--time`, or SNTP has
not settled yet. Check firmware logs for `time_synced=no`.

---

## Section 6 — Send commands to firmware subscriptions

```bash
# Move all 5 motors to 0 rad
ros2 topic pub --once /shelfbot_firmware/motor_command \
  std_msgs/msg/Float32MultiArray \
  "{layout: {dim: [], data_offset: 0}, data: [0.0, 0.0, 0.0, 0.0, 0.0]}"

# Set motor 0 to 1.0 rad/s continuous, all others stop
ros2 topic pub --once /shelfbot_firmware/set_speed \
  std_msgs/msg/Float32MultiArray \
  "{layout: {dim: [], data_offset: 0}, data: [1.0, 0.0, 0.0, 0.0, 0.0]}"

# LED ON — then verify led_state echoes true
ros2 topic pub --once /shelfbot_firmware/led std_msgs/msg/Bool "{data: true}"
ros2 topic echo /shelfbot_firmware/led_state \
  --qos-reliability best_effort --qos-durability volatile --once

# LED OFF
ros2 topic pub --once /shelfbot_firmware/led std_msgs/msg/Bool "{data: false}"
ros2 topic echo /shelfbot_firmware/led_state \
  --qos-reliability best_effort --qos-durability volatile --once
```

---

## Section 7 — Quick health one-liner

Paste this after connecting to confirm all topics are alive in a single shot:

```bash
source /opt/ros/humble/setup.bash && \
for topic in heartbeat motor_positions distance_sensors tof_distance; do
  printf "%-22s " "$topic:"
  timeout 5 ros2 topic echo /shelfbot_firmware/$topic \
    --qos-reliability best_effort --qos-durability volatile \
    --once 2>/dev/null | head -3 || echo "(no message in 5 s)"
  echo
done && \
printf "%-22s " "laser_scan:" && \
timeout 5 ros2 topic echo /shelfbot_firmware/laser_scan --once 2>/dev/null \
  | grep -E "sec:|nanosec:|frame_id:|ranges:" | head -4 || echo "(no message in 5 s)"
```

---

## Section 8 — Automated integration test

A standalone Python test covers all checks above programmatically and reports pass/fail.
No ROS 2 package build or workspace is required — it runs as a plain script.

### Run the test

```bash
source /opt/ros/humble/setup.bash
python3 shelfbot_integration_test.py
```

### What the test checks

| # | Check | How |
|---|---|---|
| 1 | `shelfbot_firmware` node is visible | `get_node_names()` |
| 2 | QoS profile per publisher matches expectation | `get_publishers_info_by_topic()` |
| 3 | All 5 publishers deliver at least one message within 8 s | subscription with timeout |
| 4 | Heartbeat counter increments monotonically | collect 3 s of samples, diff values |
| 5 | `motor_positions` has exactly 5 elements | `len(msg.data) == 5` |
| 6 | `distance_sensors` has exactly 6 elements, each in range or -1.0 | value bounds check |
| 7 | `tof_distance` is -1.0 or in 0–12 m range | value check |
| 8 | `laser_scan` has 12 range points, correct frame_id, range bounds | field inspection |
| 9 | `laser_scan` header.stamp is wall-clock epoch (> Nov 2023) | `stamp.sec > 1_700_000_000` |
| 10 | `laser_scan` stamp within 5 s of host clock | `abs(stamp - time.time()) < 5` |
| 11 | LED ON/OFF command round-trip via `led` → `led_state` | pub then wait for echo |
| 12 | No BEST_EFFORT pub + RELIABLE sub pairing on live topics | cross-check pub/sub infos |

### Expected output (all passing)

```
╔══════════════════════════════════════════════════╗
║        Shelfbot Firmware Integration Test        ║
╚══════════════════════════════════════════════════╝
  Timeout per topic : 8.0 s
  Host epoch        : 1748947300.123
  Epoch lower bound : 1700000000  (Nov 2023)

1. Node discovery
────────────────────────────────────────────────────────────
  [PASS] shelfbot_firmware node is visible

2. QoS profile verification
────────────────────────────────────────────────────────────
  [PASS] QoS heartbeat == BEST_EFFORT
  [PASS] QoS motor_positions == BEST_EFFORT
  [PASS] QoS distance_sensors == BEST_EFFORT
  [PASS] QoS led_state == BEST_EFFORT
  [PASS] QoS tof_distance == BEST_EFFORT
  [PASS] QoS laser_scan == RELIABLE

3. Message receipt (all topics must publish within timeout)
────────────────────────────────────────────────────────────
  [PASS] heartbeat receives messages — 3 message(s) received
  [PASS] motor_positions receives messages — 3 message(s) received
  [PASS] distance_sensors receives messages — 3 message(s) received
  [PASS] tof_distance receives messages — 3 message(s) received
  [PASS] laser_scan receives messages — 3 message(s) received

4. Heartbeat counter increments monotonically
────────────────────────────────────────────────────────────
  [PASS] Heartbeat counter increments

5. motor_positions payload
────────────────────────────────────────────────────────────
  [PASS] motor_positions has 5 elements
  [PASS] motor_positions values are finite

6. distance_sensors payload
────────────────────────────────────────────────────────────
  [PASS] distance_sensors has 6 elements
  [PASS] distance_sensors values in range or -1.0

7. tof_distance payload
────────────────────────────────────────────────────────────
  [PASS] tof_distance is -1.0 (no sensor) or in 0–12 m range

8. laser_scan payload and timestamp
────────────────────────────────────────────────────────────
  [PASS] laser_scan has 12 range points
  [PASS] laser_scan frame_id == 'lidar_frame'
  [PASS] laser_scan range_min == 0.02 m
  [PASS] laser_scan range_max == 12.0 m
  [PASS] laser_scan stamp is wall-clock epoch (not boot-relative)
  [PASS] laser_scan stamp within 5 s of host clock

9. LED command round-trip (publish led → receive led_state)
────────────────────────────────────────────────────────────
  [PASS] LED ON command reflected in led_state
  [PASS] LED OFF command reflected in led_state

10. QoS compatibility — no incompatible pairings
────────────────────────────────────────────────────────────
  [PASS] motor_positions pub=BEST_EFFORT sub=BEST_EFFORT compatible
  [PASS] laser_scan pub=RELIABLE sub=RELIABLE compatible

────────────────────────────────────────────────────────────
Result: 22/22 passed  ✓ All checks passed
```

### Common failure signatures and fixes

| Failure | Cause | Fix |
|---|---|---|
| `shelfbot_firmware node is visible — FAIL` | Node not connected | Check agent is running with `--time`; check Wi-Fi |
| `QoS motor_positions == BEST_EFFORT — FAIL, got RELIABLE` | Old firmware not rebuilt | Rebuild and flash with updated `microros_sync.cpp` |
| `heartbeat receives messages — FAIL` | Agent not reachable | Verify UDP port 8888, mDNS resolution |
| `motor_positions receives messages — FAIL` | Clock sync not completed | Wait for `time_synced=yes` in firmware log |
| `laser_scan stamp is wall-clock epoch — FAIL` | Agent missing `--time` or SNTP unreachable | Restart agent with `--time`; check NTP reachability |
| `laser_scan stamp within 5 s — FAIL` | Large clock drift | Restart agent; clock sync may have used stale SNTP value |
| `LED ON command reflected — FAIL` | led_state not published or QoS mismatch | Check firmware log for `led_cb` being called |
| `motor_positions pub=BEST_EFFORT sub=BEST_EFFORT — FAIL` | Still RELIABLE pub | Rebuild firmware |

---

## Section 9 — bag recording for offline analysis

```bash
# Create the QoS override file once
cat << 'EOF' > /tmp/shelfbot_qos.yaml
/shelfbot_firmware/heartbeat:
  reliability: best_effort
  durability: volatile
/shelfbot_firmware/motor_positions:
  reliability: best_effort
  durability: volatile
/shelfbot_firmware/distance_sensors:
  reliability: best_effort
  durability: volatile
/shelfbot_firmware/led_state:
  reliability: best_effort
  durability: volatile
/shelfbot_firmware/tof_distance:
  reliability: best_effort
  durability: volatile
/shelfbot_firmware/laser_scan:
  reliability: reliable
  durability: volatile
EOF

# Record all shelfbot topics
ros2 bag record \
  /shelfbot_firmware/heartbeat \
  /shelfbot_firmware/motor_positions \
  /shelfbot_firmware/distance_sensors \
  /shelfbot_firmware/led_state \
  /shelfbot_firmware/tof_distance \
  /shelfbot_firmware/laser_scan \
  --qos-profile-overrides-path /tmp/shelfbot_qos.yaml \
  -o shelfbot_bag

# Inspect the bag
ros2 bag info shelfbot_bag

# Replay and check laser_scan stamps from the bag
ros2 bag play shelfbot_bag &
ros2 topic echo /shelfbot_firmware/laser_scan --field header.stamp
```
