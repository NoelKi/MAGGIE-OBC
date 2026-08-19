# MAGGIE — Bodentest (TEST-State) und Telemetrie-/Telecommand-Protokoll

Referenz für den Aufbau *Ground Station → Server → REXUS Test Module → OBC*
und zurück. Diese Datei ist die gemeinsame Quelle für alle drei Repos; die
Implementierungen dürfen nur gemeinsam mit ihr geändert werden.

| Rolle | Repo | Datei |
|---|---|---|
| Downlink-Frames senden | MAGGIE_OBC | `include/hal/telemetry_hal.hpp` |
| Telecommands empfangen | MAGGIE_OBC | `include/hal/uplink_hal.hpp` |
| Zustandsmaschine | MAGGIE_OBC | `include/statemachine/mission_state.hpp` |
| Downlink-Frames decodieren | MAGGIE_SERVER | `app/services/downlink_frame_parser.py` |
| Telecommands verpacken | MAGGIE_SERVER | `app/routes/command.py` |
| Anzeige + Bedienung | MAGGIE-GS | `src/app/features/telecommand.ts`, `telemetry.ts` |

---

## 1. Signalweg

```
GS  --HTTP-->  Server  --UART (24-B RXSM-SDC)-->  REXUS Test Module  --UART (6-B TC)-->  OBC
GS  <--HTTP--  Server  <--UART (20-B Downlink)--  REXUS Test Module  <--UART-----------  OBC
```

Das RXSM packt die SDC-Nutzbytes aus und reicht **nur** diese an den OBC
weiter — der OBC kennt das 24-Byte-RXSM-Format nicht. Down- und Uplink teilen
sich auf dem Teensy einen UART (`Serial8`, Pin 34 RX / 35 TX, 38400 8N1).

---

## 2. Zustandsmaschine

| Wert | Zustand | Aktoren | Übergang hinein |
|---:|---|---|---|
| 0 | `PRE_LAUNCH` | gesperrt | Start |
| 1 | `ARMED` | gesperrt | SODS = HIGH |
| 2 | `ASCENT` | gesperrt | LO = HIGH |
| 3 | `EXPERIMENT` | Flugsequenz | SOE = HIGH |
| 4 | `SAFE` | aus | Experimentfenster abgelaufen |
| 5 | `ABORT` | aus | Telecommand `ABORT` oder Kraftlimit |
| 6 | `TEST` | **frei** | Telecommand `TEST_ENTER` (nur aus `PRE_LAUNCH`) |

`TEST` ist der Bodentest-Zustand für Integration und Review. Nur dort führt der
OBC Motor- und HDRM-Telecommands aus; in jedem anderen Zustand werden sie mit
einer Warnung auf der Debug-Konsole verworfen. Kommt im `TEST` ein echtes
SODS-Signal, wechselt der OBC nach `ARMED` und stoppt dabei die Aktoren — der
Flug schlägt den Bodentest.

Die REXUS-Eingänge sind als `INPUT_PULLDOWN` konfiguriert, damit die offenen
Leitungen am Bodenaufbau nicht zufällig die Flugsequenz auslösen.

---

## 3. Downlink — 20-Byte-Frame

| Idx | Feld | Bytes | Beschreibung |
|---:|---|---:|---|
| 0 | START | 1 | `0x7E` |
| 1 | MSGID1 | 1 | Subsystem |
| 2 | MSGID2 | 1 | Nachrichtentyp |
| 3 | ACK | 1 | 0 bei reiner Telemetrie |
| 4–5 | COUNTER | 2 | Frame-Zähler, big-endian |
| 6–7 | TIME | 2 | untere 16 Bit von `millis()` |
| 8–15 | DATA | 8 | Nutzdaten je MSGID |
| 16 | STATUS1 | 1 | Bit0 `SYSTEM_HEALTHY`, Bit1 `IMU_VALID` |
| 17 | STATUS2 | 1 | reserviert |
| 18 | CRC | 1 | CRC-8, Poly `0x07`, Init `0x00`, über Bytes 1..17 |
| 19 | END | 1 | `0x7F` |

Alle Mehrbyte-Felder big-endian.

### DATA je Nachrichtentyp

| MSGID1 | MSGID2 | Name | DATA | Measurement |
|---:|---:|---|---|---|
| `0x01` | `0x01` | IMU / ACCEL | `ax ay az` (3× int16 BMI088-Counts) + 2 B frei | `imu` |
| `0x01` | `0x02` | IMU / GYRO | `gx gy gz` (3× int16) + 2 B frei | `imu` |
| `0x02` | `0x01` | MOTOR / STATE | `pos` int32, `pwm` int16, `state` uint8, 1 B frei | `motor` |
| `0x03` | `0x01` | SYS / STATE | `state` uint8, `subsys` uint8, `uptime_ms` uint32, 2 B frei | `obc` |

Skalierung in der Bodenstation:

```
accel [m/s²] = count · 9.80665 / 10920      (±3 g)
gyro  [°/s]  = count · 1 / 65.536           (±500 °/s)
Winkel [°]   = counts · 360 / 4600          (Encoder, COUNTS_PER_REV)
```

**MOTOR/STATE `state`-Bits**

| Bit | Maske | Bedeutung |
|---:|---:|---|
| 0 | `0x01` | Motor dauerhaft an (`on()`) |
| 1 | `0x02` | Closed-Loop-Fahrt aktiv |
| 2 | `0x04` | keine Fahrt aktiv / am Ziel |
| 3 | `0x08` | Position im Fenster „HDRM offen" |
| 4 | `0x10` | Position im Fenster „HDRM geschlossen" |

**SYS/STATE `subsys`-Bits**: `0x01` IMU, `0x02` Motor, `0x04` Downlink,
`0x08` Wiegesensor, `0x10` Kraftsensor 2 — jeweils gesetzt, wenn beim Start
initialisiert.

Senderaten: IMU und Motor mit 20 Hz, SYS mit 1 Hz. Zusammen ≈ 1220 B/s,
also rund ein Drittel der 38400 Baud.

---

## 4. Uplink — 6-Byte-Telecommand

| Idx | Feld | Bytes | Beschreibung |
|---:|---|---:|---|
| 0 | START | 1 | `0x7E` |
| 1 | OPCODE | 1 | siehe Tabelle |
| 2–3 | ARG | 2 | int16 big-endian, aktuell ungenutzt |
| 4 | CRC | 1 | CRC-8 wie im Downlink, über Bytes 1..3 |
| 5 | END | 1 | `0x7F` |

6 Bytes passen in die maximal 15 SDC-Nutzbytes des RXSM.

| Opcode | Name | Wirkung | Server-Endpunkt |
|---:|---|---|---|
| `0x00` | `MOTOR_OFF` | Motor aus, laufende Fahrt abbrechen | `POST /api/command/motor {"action":"off"}` |
| `0x01` | `MOTOR_ON` | Motor dauerhaft an (offene Steuerung) | `… {"action":"on"}` |
| `0x02` | `MOTOR_HALF_TURN` | **relative** halbe Umdrehung | `… {"action":"half_turn"}` |
| `0x03` | `HDRM_OPEN` | **absolut** auf +180° fahren | `… {"action":"hdrm_open"}` |
| `0x04` | `HDRM_CLOSE` | **absolut** auf die Nullposition fahren | `… {"action":"hdrm_close"}` |
| `0x05` | `MOTOR_ZERO` | aktuelle Position = „HDRM geschlossen" | `… {"action":"zero"}` |
| `0x10` | `TEST_ENTER` | Bodentest betreten (nur aus `PRE_LAUNCH`) | `POST /api/command/test {"action":"enter"}` |
| `0x11` | `TEST_EXIT` | Bodentest verlassen → `PRE_LAUNCH` | `… {"action":"exit"}` |
| `0x1F` | `ABORT` | Missionsabbruch, Aktoren stoppen | `POST /api/command/abort` |

HDRM-Fahrten sind **absolut** zur Encoder-Nullposition: mehrfaches „Öffnen"
dreht den Motor nicht weiter. Der Nullpunkt entsteht beim Boot und lässt sich
mit `MOTOR_ZERO` neu setzen, nachdem der Mechanismus von Hand in die
geschlossene Lage gebracht wurde.

---

## 5. Pinbelegung des Testaufbaus (vorläufig)

Alle Werte in `include/pin_config.hpp`. Für den Bodentest genügen:

| Funktion | Teensy-Pin | Konstante |
|---|---:|---|
| Motor Kanal A | 15 | `PIN_M1_A` |
| Motor Kanal B | 14 | `PIN_M1_B` |
| Encoder A | 16 | `PIN_M1_ENC_A` |
| Encoder B | 17 | `PIN_M1_ENC_B` |
| IMU CS Accel | 37 | `PIN_CS_ACCEL` |
| IMU CS Gyro | 36 | `PIN_CS_GYRO` |
| SPI MOSI/MISO/SCK | 11 / 12 / 13 | `PIN_SPI1_*`, `PIN_SCK` |
| Down-/Uplink (Serial8) | 34 RX / 35 TX | `PIN_UPDOWNLINK_*` |
| REXUS L0 / SOE / SODS | 40 / 39 / 38 | `PIN_L0_T`, `PIN_SOE_I`, `PIN_SODS_I` |

Offen: `COUNTS_PER_REV = 4600` in `motor_hal.hpp` stammt aus dem Prototyp
`hardwareTest/motor.cpp` und muss am eingebauten Pololu-Getriebemotor
nachgemessen werden — davon hängen Winkelanzeige und HDRM-Endlagen ab.

---

## 6. Ablauf im Review

1. Server-`.env` auf die beiden seriellen Ports zum RXSM-Testmodul setzen
   (`SERIAL_PORT` = Downlink, `TC_SERIAL_PORT` = Telecommand), Server starten.
2. Ground Station öffnen → *Telecommands*. Sobald Frames ankommen, zeigt der
   Kopf „Downlink aktiv" und der Zustand steht auf `PRE_LAUNCH`.
3. **Test-Modus betreten** — der Zustandschip springt auf `TEST`, die
   Aktor-Buttons werden frei.
4. **HDRM öffnen** → Wegbalken und Winkel laufen auf 180°, der Status wechselt
   auf „HDRM offen". **HDRM schließen** fährt zurück auf 0°.
5. *Telemetrie* zeigt IMU-Beschleunigung/Drehrate sowie Motorposition und PWM
   als Live-Verlauf.
6. **Test-Modus verlassen** — Aktoren werden wieder gesperrt.

Ohne Hardware lässt sich derselbe Ablauf mit
`MAGGIE_SERVER/tools/rxsm_obc_simulator.py` durchspielen.
