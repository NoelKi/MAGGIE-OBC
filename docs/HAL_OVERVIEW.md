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

## Main loop

Everything runs cooperatively from `System::run()`; there is no RTOS. The only
interrupts are the encoder counters and the UART/USB drivers.

| When | What | Downlink frame (MSGID1/MSGID2) |
|---|---|---|
| every loop | read all queued telecommands | – |
| every loop | sample REXUS pins, update state machine | – |
| every loop | motor `update()` (stop at target, end brake pulse), 30 s run-time limit | – |
| every loop | poll both force sensors, send on each new sample (~10 Hz), or 1/s with `STALE` while a sensor hangs | `04/01`, `04/02` |
| 50 ms | IMU accel + gyro | `01/01`, `01/02` |
| 50 ms | motor 1 + 2 position, PWM, flags | `02/01`, `02/02` |
| 1 s | state, subsystem bits, uptime, REXUS levels | `03/01` |
| 1 s | uplink statistics, raw header of last uplink burst | `03/02`, `03/03` |

Downlink load is about 55 % of 38400 baud. Serial4 has an extra 512-byte send
buffer, so writes never wait on the UART.

## State machine

```
reset ─► PRE_LAUNCH ──TEST_ENTER──► TEST
             ▲  │                    │
             └──┼────TEST_EXIT───────┘
                │                    │
                └──ABORT──► ABORT ◄──ABORT      (terminal, only a reset leaves)
```

- Transitions come only from telecommands.
- Leaving TEST (by `TEST_EXIT` or `ABORT`) turns both motors off.
- State values on the wire are fixed: PRE_LAUNCH 0, ABORT 5, TEST 6. Values 1–4
  are reserved for the planned flight sequence in `maggie_state_machine.puml`.

## Telecommands

| Opcode | Command | Accepted in |
|---|---|---|
| `0x10` | TEST_ENTER | PRE_LAUNCH |
| `0x11` | TEST_EXIT | TEST |
| `0x1F` | ABORT | any |
| `0x20` | FORCE_TARE (arg 0 = both, 1, 2) | any |
| `0x00`, `0x08` | MOTOR_OFF, MOTOR2_OFF | any |
| `0x01 0x05 0x06 0x07` | MOTOR_ON, ZERO, TURN, GOTO | TEST |
| `0x09 0x0A 0x0B 0x0C` | MOTOR2_ON, ZERO, TURN, GOTO | TEST |

Counterpart on the ground: `MAGGIE_SERVER/app/routes/command.py`.

## Safety mechanisms

- **Hardware watchdog** (WDOG1, 1 s). A hung loop resets the chip. After the
  reset the motor pins are undriven (DRV8871 coasts) and the OBC starts in
  PRE_LAUNCH with actuators locked. The ground sees the reset as uptime
  dropping back.
- **No blocking calls in the loop.** Force tare is spread over the loop;
  downlink writes go into a buffer.
- **Actuator lock.** Motor commands other than OFF only run in TEST.
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
