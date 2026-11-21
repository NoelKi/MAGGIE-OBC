# MAGGIE OBC – Firmware Overview

On-board computer firmware for the MAGGIE REXUS experiment, Teensy 4.1
(PlatformIO, Arduino framework). This document describes the code as it is;
the source of truth for pins is `include/pin_config.hpp`, for wire formats
`include/hal/telemetry_hal.hpp` and `include/hal/uplink_hal.hpp`.

## Structure

```
src/main.cpp              setup()/loop()
src/boot_guard.cpp        hardware watchdog, hang breadcrumbs, reset-loop guard
src/obc_log.cpp           Log: debug output to USB and to the SD card (LOG.txt)
src/system.cpp            System: owns every module, runs the loop
src/statemachine/         ground and flight states
src/storage/              persistent state (EEPROM), SD-card log
src/hal/                  one file per hardware module (see below)
src/hardwareTest/         standalone test sketches, excluded from the build
include/                  headers, same layout; pin_config.hpp = all pins
```

`System` is the only class that talks to the modules. No module calls another,
and the state machine knows no hardware: `System` reads inputs, asks the state
machine, and drives the outputs.

## Modules

| Module | Hardware | Pins | Notes |
|---|---|---|---|
| `IMUHAL` | BMI088 (accel + gyro), SPI0 | CS accel 37, CS gyro 36, MOSI 11, MISO 12, SCK 13 | Chip IDs checked on every read; a dropout clears `IMU_VALID` |
| `MotorHAL` ×2 | DRV8871 + quadrature encoder | M1: PWM 14/18, enc 5/6 · M2: PWM 15/23, enc 7/8 | Open loop, 20 kHz hardware PWM. Encoder is a sensor and end switch for `TURN`/`GOTO`, not a controller |
| `ForceHAL` 1 | 3× HX711 (X, Y, Z), shared clock | DOUT 4, 3, 2 · CLK 27 | Own driver: all channels clocked in one pass. Tare runs in the background |
| `ForceHAL` 2 | 4× HX711 (A, B, C, D), shared clock | DOUT 30, 31, 32, 41 · CLK 9 | Vector maths happens on the ground |
| `UplinkReceiver` | Serial4 RX | 16 | 6-byte telecommand frame, CRC-8 |
| `TelemetryDownlink` | Serial4 TX, 38400 baud | 17 | 20-byte frame, CRC-8, no ACK |
| `REXUSHAL` | L0, SOE, SODS (open drain, active low) | 40, 39, 38 | Debounced, sent down as raw levels only. They trigger no transitions in this build |
| `CameraHAL` | 4× RunCam Split 4 V2 via CD4052B mux, Serial5 | TX 20, RX 21, mux A 19, B 22 | RunCam Device Protocol, CRC-8/DVB-S2. One camera at a time; probes one per second, status in `SYS/CAMERA` (03/07) |
| `ArmLink` | Robot arm control unit (RCU, STM32), Serial1 | RX 0, TX 1 | LISA-ODISEE ICD v0.9 frames, CRC-32/MPEG-2. Baud rate `rates::ARM_BAUDRATE` (115200, **unconfirmed**) |

Up- and downlink share one UART (`Serial4`) to the REXUS service module (RXSM).
The robot arm has its own UART (`Serial1`). `extern/lisa-if/` is the Python
reference parser for that protocol (a copy, see `extern/README.md`); it is not
part of the build.
USB `Serial` (115200) is for log output only. Firmware code prints through
`Log` (`include/obc_log.hpp`), which writes to USB and copies every line to
the SD card.

## Main loop and data rates

Everything runs cooperatively from `System::run()`; there is no RTOS. All rates
live in `include/data_rates.hpp`.

Downlink in three tiers (user decision, 19.11.2026; `DownlinkPrio`, limits in
`data_rates.hpp`):

| Tier | When | What | Frame (MSGID1/MSGID2) |
|---|---|---|---|
| 1 STATUS | 500 ms + on every state change | state, subsystems, REXUS levels, HDRM/arm flight flags (DATA[7]), live bits: arm link, SD, flight switch, minimal mode, cameras 1–4 (STATUS2) | `03/01` |
| 1 STATUS | one slot every 125 ms, each frame 1 Hz | uplink stats + raw, arm link, arm trajectory, flight trajectory, cameras + camera info, SD card, boot/watchdog | `03/02`–`03/0A` |
| 2 MOTION | one joint every 25 ms (each 5 Hz), only new values | arm joint state + angle | `05/01` |
| 3 BULK | `DL_IMU_INTERVAL_MS` (20 Hz) | IMU accel + gyro | `01/01`, `01/02` |
| 3 BULK | `DL_FORCE_INTERVAL_MS` (10 Hz each), or 1/s with `STALE` | force 1 + 2 | `04/01`, `04/02` |
| 3 BULK | `DL_MOTOR_INTERVAL_MS` (10 Hz) | motor 1 + 2 position, PWM, flags | `02/01`, `02/02` |
| 3 BULK | `ARM_IMU_INTERVAL_MS` (2 Hz) | RCU IMU accel + gyro | `05/04`, `05/05` |
| 3 BULK | one frame every 62 ms (each 1 Hz) | arm voltage, arm temperature per joint | `05/02`, `05/03` |

Rates are for 38400 baud (~3.8 KB/s, ~75 % load). Tier 1 may use the whole
UART buffer, tier 2 up to 80 ms of queue, tier 3 up to 60 ms; tiers 2 and 3
always leave `DL_STATUS_RESERVE` free for tier 1. A frame that does not fit is
dropped for the downlink only (counted per tier in `SYS.csv`); the loop never
waits. Nothing is sent in bursts, so no tier blocks another. Changing
`DOWNLINK_BAUDRATE` scales the tier-3 rates and queue limits automatically
(`DL_SPEED`). `tools/downlink_sim/run.sh` checks 38400 and 115200 baud: tiers 1
and 2 lose nothing, every tier-3 type arrives at its full rate.

The SD card does not take downlink frames. It gets every value straight from
its source (see "On-board storage").

## State machine

Timing lives in one file: `include/mission_timeline.hpp`.

```
Ground:  PRE_LAUNCH ──TEST_ENTER──► TEST ──TEST_EXIT──► PRE_LAUNCH
         any state ──STATE_RESET──► PRE_LAUNCH   (test tool)

Flight:  PRE_LAUNCH / TEST ──SODS──► ARMED ──LO──► ASCENT ──SOE or 70 s──► EXPERIMENT (t0)
         EXPERIMENT  t0+2.000 s HDRM 1 open, t0+2.010 s HDRM 2 open
                     ──both moves finished──► ROBOT_ARM (placeholder)
         ROBOT_ARM   ──t0+115 s──► DESCENT: HDRM 1 close, +10 ms HDRM 2 close,
                     then both motors locked (all motor pins HIGH)
```

| Value | State | Telecommands accepted |
|---|---|---|
| 0 | PRE_LAUNCH | all (motor moves only in TEST) |
| 6 | TEST | all |
| 1 | ARMED | MOTOR_OFF, ARM_STOP/INHIBIT, FORCE_TARE, CAMERA (no stop); launch scrub = STATE_RESET |
| 2, 3, 7 | ASCENT, EXPERIMENT, ROBOT_ARM | none except STATE_RESET |
| 4 | DESCENT | none except STATE_RESET (MOTOR_OFF would release the lock) |

STATE_RESET is accepted in **every** state. It is a test tool: there is no
uplink after launch. It stops both motors, cancels scheduled HDRM moves and
releases the DESCENT lock.

- HDRM moves use the same code as `MOTOR_TURN` in TEST (±165° relative).
- At power-on the motor pins are LOW (DRV8871 coasts, mechanical lock holds).
- **REXUS signals are ground truth.** A later signal implies the earlier ones:
  LO goes to ASCENT and SOE to EXPERIMENT from every ground state (PRE_LAUNCH,
  TEST) and from ARMED, even if SODS or LO was missed. The former ABORT state
  was removed on 2026-11-17 (value 5 and opcode 0x1F stay reserved);
  STATE_RESET does the same.
- Signals must be stable for 200 ms (`REXUSHAL::SIGNAL_STABLE_MS`) to count.
- On every return to PRE_LAUNCH, a signal that is still asserted only counts
  again after it has been released (`FLIGHT_IGNORE_*`, stored in EEPROM). So
  STATE_RESET after landing, with LO/SOE still held, does not start a second
  flight. A signal that was not asserted at that moment counts normally.

### Reboot in flight

- Every state change is stored in EEPROM, and so is the flight time (once per
  full second, from LO in ASCENT and from t0 after that). After a watchdog
  reset or power loss the sequence continues at the stored time, at most ~1 s
  plus the outage late.
- `StateMachine::stateAfterReboot()` is the single place that decides the start
  state: flight states are resumed, TEST (and an old stored ABORT) fall back to
  PRE_LAUNCH.
- Each HDRM move is started at most once per flight (flag in EEPROM, written
  before the move). A reboot during a move does not repeat it, because a
  second relative turn from an unknown position could drive into the end stop.
- No flash write happens while a motor runs: flash writes block interrupts, and
  a blocked encoder would miss the stop position.

## Telecommands

| Opcode | Command | Accepted in |
|---|---|---|
| `0x10` | TEST_ENTER | PRE_LAUNCH |
| `0x11` | TEST_EXIT | TEST |
| `0x12` | STATE_RESET | any |
| `0x20` | FORCE_TARE (arg 0 = both, 1, 2) | ground states, ARMED |
| `0x30 0x31` | ARM_STOP, ARM_INHIBIT (arg high byte = joint, 0 = all) | ground states, ARMED |
| `0x35` | ARM_BLINK (joint) | ground states |
| `0x40` | CAMERA (high byte = camera 1..4, 0 = all; low byte 1 start, 2 stop, 3 probe) | ground states; ARMED without stop |
| `0x32 0x33 0x34` | ARM_RELEASE, ARM_JOG (±1..30°), ARM_SET_VELOCITY (°/s) | TEST |

Arm commands carry the joint (node ID 1..8) in the high byte of the argument
and the value in the low byte. On every transition that stops the motors
(leaving TEST, flight start, return to PRE_LAUNCH) the OBC also sends
STOP and INHIBIT to all joints. The flight sequence in `ROBOT_ARM` is still a
placeholder.
| `0x00`, `0x08` | MOTOR_OFF, MOTOR2_OFF | ground states, ARMED |
| `0x01 0x05 0x06 0x07` | MOTOR_ON, ZERO, TURN, GOTO | TEST |
| `0x09 0x0A 0x0B 0x0C` | MOTOR2_ON, ZERO, TURN, GOTO | TEST |

Counterpart on the ground: `MAGGIE_SERVER/app/routes/command.py`.

## On-board storage

| What | Where | Survives |
|---|---|---|
| Mission state, flight time, HDRM progress, boot counter | EEPROM emulation in flash (`PersistentState`) | reset, power loss, reflash |
| Everything (see below) | SD card, CSV files (`SdLogger`) | power loss, except the last ~2 s |

- Layout: `/MAGGIE/<date>/<HHMMSS>_B<boot>/` with `INFO.txt`, `LOG.txt`,
  `EVENTS.csv`, `SYS.csv`, `IMU.csv`, `FORCE_T1.csv`, `FORCE_T2.csv`,
  `MOTOR.csv`, `ARM_JOINTS.csv`, `ARM_MODULE_IMU.csv`, `ARM_POWER.csv`,
  `ARM_RCU_IMU.csv`, `ARM_RAW.csv`, `CAMERA.csv`. `/MAGGIE/LIESMICH.TXT`
  describes every column. Each CSV starts with `t_s` (seconds since boot) and
  `zustand` (mission state). The date comes from the Teensy RTC, which without
  a coin cell restarts at the firmware build time after a power loss.
- Content: IMU 100 Hz incl. chip temperature, every force conversion with full
  24-bit counts, both motors at 100 Hz, every valid RCU frame in both
  directions (decoded into joints, module IMU, voltages/temperatures, RCU IMU,
  plus raw), cameras and a system overview at 1 Hz, state changes, REXUS
  signals, every telecommand (accepted or rejected), warnings, and the
  complete debug log.
- Values go as compact binary records into a 256 KB RAM ring first
  (`push*()`, never touches the card). `service()` turns them into text
  (per-file 2 KB buffers) and does at most one card operation per loop: create
  a folder/file, write 512 bytes, or sync one file. Files stay open and each is
  synced every 2 s. Nothing touches the card while a motor runs.
- The card is switched off until the next boot on any error, after one
  operation slower than 600 ms, or after three slower than 150 ms within 30 s.
  `tools/sd_sim/run.sh` tests all of this on the PC against a fake card.
- Flash writes block interrupts (µs normally, 45–400 ms for a rare sector
  erase), so EEPROM is written only at boot and on state changes.

## Safety mechanisms

- **Hardware watchdog** (WDOG1, `boot_guard.hpp`). Starts first thing in
  `setup()` with 5 s, so a hang during init also resets; 1 s in the loop. After
  the reset the motor pins are undriven (DRV8871 coasts) and the OBC restores
  its state with actuators locked.
- **Hang breadcrumbs.** Before each section that could hang, the current stage
  is written to `SNVS_LPGPR0` (survives a watchdog reset). After a watchdog
  reset the guilty optional subsystem (SD, IMU, force, cameras, arm UART)
  stays off for that boot instead of hanging again.
- **Reset-loop guard.** `SNVS_LPGPR1` counts watchdog resets in a row (cleared
  after 60 s of stable running). From 3 on, the OBC starts in minimal mode:
  SD card, cameras and force sensors off. State machine, uplink, downlink,
  motors, IMU and arm keep running. `SYS/BOOT` shows all of this on the ground.
- **No blocking calls in the loop.** Force tare is spread over the loop;
  downlink writes go into a buffer.
- **Actuator lock.** Motor commands other than OFF only run in TEST. From LO
  until DESCENT no telecommand is accepted at all.
- **Motor run-time limit.** A motor that runs for 30 s without a new command is
  stopped (`MOTOR_ON_TIMEOUT_MS`).
- **Turn stall detection.** `TURN`/`GOTO` stop after 1 s without encoder
  progress (150 ms near the target with soft approach).
- **Degraded start.** A missing subsystem is never fatal. Its bit in the
  subsystem byte stays 0, and `SYSTEM_HEALTHY` is set only when all subsystems
  are up.
- **Boot time.** About 0.4 s from reset to the first loop (300 ms USB
  enumeration in the core, ~110 ms IMU init). There is no wait for the serial
  monitor.

## Hardware tests

`src/hardwareTest/` holds standalone sketches for IMU, force and motor. Each
defines its own `setup()`/`loop()`. To run one, enable it in
`platformio.ini` (`build_src_filter`) and exclude `main.cpp`.
