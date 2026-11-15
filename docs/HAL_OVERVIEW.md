# MAGGIE OBC – Firmware Overview

On-board computer firmware for the MAGGIE REXUS experiment, Teensy 4.1
(PlatformIO, Arduino framework). This document describes the code as it is;
the source of truth for pins is `include/pin_config.hpp`, for wire formats
`include/hal/telemetry_hal.hpp` and `include/hal/uplink_hal.hpp`.

## Structure

```
src/main.cpp              setup()/loop(), hardware watchdog
src/system.cpp            System: owns every module, runs the loop
src/statemachine/         PRE_LAUNCH / TEST / ABORT
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

Up- and downlink share one UART (`Serial4`) to the REXUS service module (RXSM).
USB `Serial` (115200) is for log output only.

## Main loop and data rates

Everything runs cooperatively from `System::run()`; there is no RTOS. All rates
live in `include/data_rates.hpp`.

| When | What | Frame (MSGID1/MSGID2) |
|---|---|---|
| every loop | telecommands, REXUS signals, state machine, flight actions | – |
| every loop | motor `update()`, 30 s run-time limit | – |
| per HX711 conversion (80 Hz, set by the RATE pin) | force 1 + 2, or 1/s with `STALE` while a sensor hangs | `04/01`, `04/02` |
| 10 ms (100 Hz) | IMU accel + gyro (BMI088 runs at 400 Hz internally) | `01/01`, `01/02` |
| 50 ms (20 Hz) | motor 1 + 2 position, PWM, flags | `02/01`, `02/02` |
| 1 s | state, subsystem bits, uptime, REXUS levels, uplink statistics | `03/01`–`03/03` |
| on change | state or SODS/LO/SOE, ms-exact, **SD card only** | `10/01` |
| every loop | SD card: at most one write, never while a motor runs | – |

About 8 KB/s in total. Every frame goes to the SD card. It also goes to the
downlink UART if the UART's send buffer has room; otherwise it is dropped for
the downlink only (the ground sees a counter gap) and the loop never waits.
At 38400 baud (~3.8 KB/s) only about half the frames fit; the RXSM link needs
>= 115200 baud for all of them (`rates::DOWNLINK_BAUDRATE`).

## State machine

Timing lives in one file: `include/mission_timeline.hpp`.

```
Ground:  PRE_LAUNCH ──TEST_ENTER──► TEST ──TEST_EXIT──► PRE_LAUNCH
         PRE_LAUNCH / TEST / ARMED ──ABORT──► ABORT
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
| 5 | ABORT | all except motor moves; STATE_RESET leaves |
| 1 | ARMED | ABORT, MOTOR_OFF, FORCE_TARE (launch scrub) |
| 2, 3, 7 | ASCENT, EXPERIMENT, ROBOT_ARM | none except STATE_RESET |
| 4 | DESCENT | none except STATE_RESET (MOTOR_OFF would release the lock) |

STATE_RESET is accepted in **every** state. It is a test tool: there is no
uplink after launch. It stops both motors, cancels scheduled HDRM moves and
releases the DESCENT lock.

- HDRM moves use the same code as `MOTOR_TURN` in TEST (±165° relative).
- At power-on the motor pins are LOW (DRV8871 coasts, mechanical lock holds).
- **REXUS signals are ground truth.** A later signal implies the earlier ones:
  LO goes to ASCENT and SOE to EXPERIMENT from every ground state (PRE_LAUNCH,
  TEST, ABORT) and from ARMED, even if SODS or LO was missed. Only SODS
  respects ABORT, so a launch scrub on the pad still works before LO.
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
  state: flight states and ABORT are resumed, TEST falls back to PRE_LAUNCH.
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
| `0x1F` | ABORT | PRE_LAUNCH, TEST, ARMED |
| `0x12` | STATE_RESET | any |
| `0x20` | FORCE_TARE (arg 0 = both, 1, 2) | ground states, ARMED |
| `0x00`, `0x08` | MOTOR_OFF, MOTOR2_OFF | ground states, ARMED |
| `0x01 0x05 0x06 0x07` | MOTOR_ON, ZERO, TURN, GOTO | TEST |
| `0x09 0x0A 0x0B 0x0C` | MOTOR2_ON, ZERO, TURN, GOTO | TEST |

Counterpart on the ground: `MAGGIE_SERVER/app/routes/command.py`.

## On-board storage

| What | Where | Survives |
|---|---|---|
| Mission state, flight time, HDRM progress, boot counter | EEPROM emulation in flash (`PersistentState`) | reset, power loss, reflash |
| Every downlink frame | SD card, `/B<boot>/F<n>.BIN` (`SdLogger`) | power loss, except the open file |

- SD files hold the 20-byte frames back to back, the same format as the
  downlink, plus the SD-only event frames. A new file starts every 10 s.
  `MAGGIE-SERVER/tools/sd_to_csv.py` turns a card into CSV files with a
  timeline zeroed at SODS.
- Frames go into a 256 KB RAM buffer (~30 s of telemetry) first. The card is
  written in 512-byte chunks, one operation per loop, and not at all while a
  motor runs (card busy times of 100–250 ms would overshoot a turn).
- Any SD error switches logging off until the next boot. If a watchdog reset
  happened inside an SD call (marker in `SNVS_LPGPR0`), the card stays off for
  that boot instead of causing a reset loop.
- Flash writes block interrupts (µs normally, 45–400 ms for a rare sector
  erase), so EEPROM is written only at boot and on state changes.

## Safety mechanisms

- **Hardware watchdog** (WDOG1, 1 s). A hung loop resets the chip. After the
  reset the motor pins are undriven (DRV8871 coasts) and the OBC starts in
  PRE_LAUNCH with actuators locked. The ground sees the reset as uptime
  dropping back.
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
