# MOTOR_SETUP

## Pin mapping (PULSE, DIR)
- Motor 0: GPIO27 (PULSE), GPIO26 (DIR)
- Motor 1: GPIO14 (PULSE), GPIO33 (DIR)
- Motor 2: GPIO13 (PULSE), GPIO19 (DIR)
- Motor 3: GPIO4  (PULSE), GPIO18 (DIR)
- Motor 4: GPIO12 (PULSE), GPIO23 (DIR)

Source: `components/motor_control/motor_control.cpp`.

## API alignment (micro-ROS)
- Position set/get uses radians (`position_rad`).
- Velocity set/get uses radians per second (`velocity_rad_s`).
- REST endpoint JSON:
  - GET `/api/motor/status` -> `{ "motors": [{"motor":0,"position_rad":...,"velocity_rad_s":...,"running":...}] }`
  - POST `/api/motor/set` with `{ "motor": 0, "position_rad": 1.57, "velocity_rad_s": 0.5 }`.
