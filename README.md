# Balancing Robot

![ESP32](https://img.shields.io/badge/ESP32-C3_black)
![C++](https://img.shields.io/badge/Arduino-C%2B%2B-black)
![PID 1kHz](https://img.shields.io/badge/PID-1kHz-black)
![WebSocket](https://img.shields.io/badge/WebSocket-telemetry-black)

Self-balancing two-wheel robot on ESP32 + L298N + MPU IMU, with a serial
setup wizard, WiFi Web UI (telemetry + PID tuning), and optional ESP-NOW
joystick control. Runtime-configurable: pins, WiFi, IMU, and PID all live
in NVS via the serial CLI — `settings.h` holds seed defaults only.

> Built for real hardware: dual angle PID @ 1 kHz, Madgwick fusion, NVS-persisted
> config, embedded Web UI served from flash, OTA-ready. See `firmware/ARCHITECTURE.md`
> for the module map.

## Hardware

- Board: ESP32-C3 by default (ESP32 / ESP32-S3 selectable in `settings.h`)
- IMU: auto-detect — MPU6050 (I2C) or MPU6500 (SPI / I2C)
- Drivers: L298N dual H-bridge, N20 motors
- Filter: Madgwick (Kalman, Mahony, EKF, Complementary selectable)
- Control: dual angle PID @ 1 kHz, optional cascade angle+rate mode
- Printable chassis + N20 mounts in `assets/balancing_robot_STL/`

## Setup

1. Open `firmware/balancing_robot/balancing_robot.ino` in Arduino IDE.
   Install ESP32 core plus `WebSocketsServer` (Markus Sattler) and
   `ArduinoOTA` via the Library Manager.
2. In `src/config/settings.h` pick your board (`ACTIVE_BOARD`) and IMU
   (`ACTIVE_IMU`, 0 = auto-detect). Leave `WIFI_SSID` / `WIFI_PASSWORD`
   as placeholders — real credentials go in over serial, never in git.
3. Flash at 115200 baud, then open the Serial Monitor and run `setup`.
   The wizard walks through pins, IMU, motor, LED, link, WiFi, and
   calibration in 7 steps (`setup <step>` jumps straight to one).
4. Set WiFi without touching code: `wifi set ssid <name>`,
   `wifi set pass <password>`, then `wifi save`. Check with `wifi show`
   (passwords stay masked) and `wifi_status`.
5. In Arduino IDE set Tools → Partition Scheme to "No FS 4MB (2MB APP x2)"
   (the default 1.2MB app partition is too small for the embedded Web UI).
6. Connect to the same network — or to the robot's own `ESP32_BALANCING`
   AP — and open `http://<robot-ip>/` (AP mode: `http://192.168.4.1/`)
   for the Web UI (live attitude, battery, arm switch, stick control).
   Telemetry/tuning runs over WebSocket port 81.
7. Tune the balance: start with `DEFAULT_KP_PITCH` / `KI` / `KD` in
   `settings.h`, or live via serial (`pp` / `pi` / `pd` …) or the Web UI —
   every change auto-saves to NVS. `load` reloads saved values,
   `reset_pid` returns to compiled defaults.

Handy serial commands: `status`, `load`, `reset_pid`, `i2c_scan`,
`abort setup`, `reboot`. Type `help` on the robot for the full list.

## Web UI

Dark HUD theme, four tabs. Served by the robot itself on port 80 —
the UI (HTML/CSS/JS/fonts/uPlot) is embedded in the firmware flash via
`tools/gen_web_assets.py` → `src/web/web_assets.h`. No phone-side file or
internet connection needed once the phone is on the robot's WiFi. Re-run
`python3 firmware/balancing_robot/tools/gen_web_assets.py` after editing
anything under `UI/`, then recompile.

CONTROL — square stick, live pitch/roll/yaw, arm bar:

![CONTROL tab](docs/screenshots/ui-control.png)

STATUS — link, battery, IMU, motors:

![STATUS tab](docs/screenshots/ui-status.png)

PID TUNE — pitch/roll/yaw gains, load + reset:

![PID TUNE tab](docs/screenshots/ui-pid.png)

SETUP — vehicle, connection, stick range, calibration:

![SETUP tab](docs/screenshots/ui-setup.png)

## Layout

- `firmware/balancing_robot/` — Arduino sketch (`src/config`, `sensors`,
  `filters`, `control`, `comms`, `system`, `utils`; see
  `firmware/ARCHITECTURE.md`)
- `UI/` — Web UI (index.html + css/js), served by the robot on :80
- `assets/balancing_robot_STL/` — chassis + motor mount STLs

## Notes

- `settings.h` in git carries placeholder WiFi/MAC values on purpose.
  Keep it that way — `git add` files explicitly, never `git add -A`.
- Default AP mode: SSID `ESP32_BALANCING`, password `balancing123`.
  Change it on your board over serial (`wifi set ap_pass <yours>` +
  `wifi save`) before taking the robot to public places.
