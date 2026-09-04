# Firmware Architecture — Balancing Robot

Arduino sketch + `src/` layout (required: the Arduino builder only
compiles `.cpp` files under the sketch `src/` directory).

- `balancing_robot.ino` (~460 lines): includes, all global variable
  definitions, `setup()`, `loop()`. No logic lives here anymore —
  every function body has moved into `src/`.

## `src/config/` — compile-time configuration
- `config.h`: PID defaults (single source of truth), pins, thresholds,
  loop rates, feature flags (`ENABLE_CASCADE_PID`, `ENABLE_ESPNOW`).
- `board.h`: board-specific pin map. `sensor.h`: IMU profile selection
  (`IMU_Custom` alias for MPU6050 / MPU6500 I2C / SPI).

## `src/sensors/` — hardware drivers
- `MPU6050_Custom`, `MPU6500_I2C_Custom`, `MPU6500_SPI_Custom`
  (`.h`/`.cpp` pairs), `calibration.h/.cpp` (NVS bias storage),
  `battery_state.h` (`BatteryState` enum shared by battery + comms).

## `src/filters/` — attitude estimation (select ONE)
- `filter_selector.h`: uncomment one `USE_*_FILTER` define.
- Kalman, Mahony, Madgwick (active), Complementary (2D),
  Complementary-Quaternion (3D), EKF — each a `.h`/`.cpp` pair sharing
  the `pitch`/`roll`/`yaw` globals. `getActiveFilterName()` is `inline`.

## `src/control/` — PID + motors
- `pid_controller.h/.cpp`: single-PID pitch/roll/yaw, `applyBraking()`,
  `updateDualPID()`. `cascade_pid_controller.h/.cpp`: outer angle +
  inner rate loops. `motor_control.h/.cpp`: 2-motor PWM mix
  (`updateVehicleMotorControl()`), input limits, `initVehicleMotors()`.

## `src/comms/` — outside world
- `serial_commands.h/.cpp`: USB CLI (`status`, `load`, `save`, `i2c_scan`…).
- `wifi_ota.h/.cpp`: WiFi + OTA. NOTE: real SSID/password live here
  **uncommitted, local-only** — HEAD carries placeholders.
- `websocket_handler.h/.cpp`: WebSocket server on :81 (telemetry out,
  PID tuning in). `esp_now_handler.h/.cpp`: ESP-NOW joystick input.

## `src/system/` — cross-cutting runtime pieces (Phase 5)
- `prefs.h/.cpp`: PID save/load/reset via NVS (`Preferences`).
- `battery.h/.cpp`: voltage filtering + low-voltage LED logic.
- `imu.h/.cpp`: global `mpu` object + `initIMU()`.
- `control_task.h/.cpp`: the 1 kHz `controlLoopTask` (Core 0).

## `src/utils/`
- `timing.h/.cpp`: loop-time stats. `i2c_scan.h/.cpp`: bus scan helper.

## Conventions
- One `.h`/`.cpp` pair per module; shared state via `extern` in the
  header, defined once in the `.ino` (globals) or the `.cpp` (module
  objects like `mpu`). Include order: `config.h` first.
- WiFi creds never committed. `git add` explicitly — never `git add -A`.
