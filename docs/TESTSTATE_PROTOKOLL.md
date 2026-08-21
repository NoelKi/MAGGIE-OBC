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
| 5 | `ABORT` | aus | Telecommand `ABORT` (Endzustand, nur Reset führt heraus) |
| 6 | `TEST` | **frei** | Telecommand `TEST_ENTER` (nur aus `PRE_LAUNCH`) |

Die Werte 1..4 waren `ARMED`/`ASCENT`/`EXPERIMENT`/`SAFE` der Flugsequenz. Sie
sind in `mission_state.hpp` reserviert, aber nicht implementiert — der aktuelle
Ausbau kennt nur die drei Zustände oben, und gewechselt wird ausschließlich per
Telecommand.

`TEST` ist der Bodentest-Zustand für Integration und Review. Nur dort führt der
OBC Motor-Telecommands aus; in jedem anderen Zustand werden sie mit einer
Warnung auf der Debug-Konsole verworfen. Einzige Ausnahme ist `MOTOR_OFF` —
Ausschalten ist ein Sicherheitskommando und in jedem Zustand erlaubt.

Die REXUS-Eingänge (L0/SOE/SODS) werden eingelesen und in der Telemetrie
mitgeschickt, lösen aber **keine** Zustandsübergänge aus. Sie sind als
`INPUT_PULLUP` konfiguriert, weil `REXUS_ACTIVE_HIGH = false` gilt
(`pin_config.hpp`).

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
| `0x03` | `0x01` | SYS / STATE | `state` uint8, `subsys` uint8, `uptime_ms` uint32, `rexus` uint8, 1 B frei | `obc` |
| `0x03` | `0x02` | SYS / UPLINK | `rx_bytes` uint16, `frames_ok` uint16, `frames_bad` uint16, `last_opcode` uint8, 1 B frei | `obc` |
| `0x03` | `0x03` | SYS / UPLINK_RAW | bis zu 8 Rohbytes des letzten Uplink-Bursts; STATUS2 trägt die gültige Länge | `obc` |

Skalierung in der Bodenstation:

```
accel [m/s²] = count · 9.80665 / 10920      (±3 g)
gyro  [°/s]  = count · 1 / 65.536           (±500 °/s)
Winkel [°]   = counts · 360 / 4550          (Encoder, COUNTS_PER_REV)
```

`pos` in Encoder-Counts ist der direkt gemessene Wert. `angle_deg` und
`revolutions` leitet der Server daraus über `COUNTS_PER_REV = 4550` ab (siehe §5).

**MOTOR/STATE `state`-Bits**

| Bit | Maske | Bedeutung |
|---:|---:|---|
| 0 | `0x01` | Motor dreht (`on()`) |
| 1 | `0x02` | Encoder angehängt und wird gelesen |
| 2 | `0x04` | Winkeldrehung (`MOTOR_TURN`) läuft |

Bit 3 und 4 waren `HDRM_OPEN`/`HDRM_CLOSED` der früheren Positionsregelung. Sie
sind mit ihr entfallen und bleiben reserviert.

**SYS/STATE `subsys`-Bits**: `0x01` IMU, `0x02` Motor, `0x04` Downlink —
jeweils gesetzt, wenn beim Start initialisiert. Bit 3 und 4 waren Wiegesensor
bzw. Kraftsensor 2 und bleiben reserviert.

**SYS/STATE `rexus`-Bits** (DATA[6]): `0x01` L0, `0x02` SOE, `0x04` SODS —
rohe Leitungspegel, ohne Wirkung auf den Zustand.

Senderaten: IMU und Motor mit 20 Hz, SYS mit 1 Hz. Zusammen ≈ 1220 B/s,
also rund ein Drittel der 38400 Baud.

---

## 4. Uplink — 6-Byte-Telecommand

| Idx | Feld | Bytes | Beschreibung |
|---:|---|---:|---|
| 0 | START | 1 | `0x7E` |
| 1 | OPCODE | 1 | siehe Tabelle |
| 2–3 | ARG | 2 | int16 big-endian; bei `MOTOR_ON` die Geschwindigkeit |
| 4 | CRC | 1 | CRC-8 wie im Downlink, über Bytes 1..3 |
| 5 | END | 1 | `0x7F` |

6 Bytes passen in die maximal 15 SDC-Nutzbytes des RXSM.

| Opcode | Name | Wirkung | Server-Endpunkt |
|---:|---|---|---|
| `0x00` | `MOTOR_OFF` | Motor aus | `POST /api/command/motor {"action":"off"}` |
| `0x01` | `MOTOR_ON` | Motor dreht mit `ARG` als PWM, bis `MOTOR_OFF` kommt | `… {"action":"on","speed":220}` |
| `0x05` | `MOTOR_ZERO` | Encoder-Zähler auf 0 setzen (stoppt den Motor) | `… {"action":"zero"}` |
| `0x06` | `MOTOR_TURN` | Drehung um `ARG` Grad **relativ**, stoppt am Encoder-Ziel | `… {"action":"turn","angle":180}` |
| `0x07` | `MOTOR_GOTO` | Fahrt auf `ARG` Grad **absolut** zur Encoder-Null | `… {"action":"goto","angle":180}` |
| `0x10` | `TEST_ENTER` | Bodentest betreten (nur aus `PRE_LAUNCH`) | `POST /api/command/test {"action":"enter"}` |
| `0x11` | `TEST_EXIT` | Bodentest verlassen → `PRE_LAUNCH` | `… {"action":"exit"}` |
| `0x1F` | `ABORT` | Missionsabbruch, Aktoren stoppen | `POST /api/command/abort` |

`0x02`..`0x04` (`MOTOR_HALF_TURN`, `HALF_TURN_FWD`, `HALF_TURN_REV`) sind
stillgelegt: Die Positionsregelung ist entfallen, der Encoder ist nur noch
Sensor. Die Werte bleiben reserviert und dürfen nicht neu vergeben werden.

**`MOTOR_ON`-Argument.** `ARG` ist der PWM-Stellwert; das Vorzeichen gibt die
Drehrichtung vor. `ARG = 0` überlässt dem OBC seine `DEFAULT_ON_SPEED`
(vorwärts).

**Untergrenze `MIN_DRIVE_SPEED = 220`.** Am Aufbau gemessen: Unter etwa 150
läuft der Motor selbst **ohne Last** nicht an — Haftreibung plus
Getriebewiderstand, unter Last liegt die Schwelle eher höher. 220 hält Abstand
dazu. Gültig ist damit `0` (Default) oder ein Betrag von `220..255`.

Die Grenze wird zweifach durchgesetzt: Der Server weist kleinere Beträge mit
HTTP 400 ab, damit ein zu kleiner Wert auffällt statt still verändert zu werden;
`MotorHAL::setSpeed()` hebt sie zusätzlich an, falls ein Kommando anders
hereinkommt. Greift die Anhebung, meldet der OBC das auf der Debug-Konsole. Der
schlimmste Fall wäre ein Wert dazwischen: Strom fließt, der Motor steht, die
H-Brücke heizt — und bei einer Fahrt auf Encoder-Ziel liefe zusätzlich der
Stillstands-Abbruch los.

**`MOTOR_ON` stoppt nicht von selbst.** Beendet wird ein Dauerlauf durch
`MOTOR_OFF`, durch `TEST_EXIT`/`ABORT` (der OBC stoppt die Aktoren beim
Verlassen von `TEST`) — oder durch den Laufzeit-Watchdog `MOTOR_ON_TIMEOUT_MS`
in `system.hpp`, der den Motor nach 30 s abschaltet. Die Konstante auf `0` zu
setzen deaktiviert den Watchdog.

**`MOTOR_TURN` und `MOTOR_GOTO`.** `ARG` ist in beiden Fällen ein Winkel in
Grad, umgerechnet über `COUNTS_PER_REV`. Der Unterschied ist der Bezugspunkt:

| | Bezug | Ziel |
|---|---|---|
| `MOTOR_TURN` | aktuelle Position | `Position + ARG` |
| `MOTOR_GOTO` | Encoder-Null | `ARG` |

Der Server begrenzt beide auf ±3600°; `angle = 0` ist bei `goto` gültig (Fahrt
auf die Nullage), bei `turn` nicht.

**Für wiederholtes Auf/Zu ist `MOTOR_GOTO` zu nehmen.** Jede Fahrt endet ein
Stück hinter dem Ziel, und dieser Nachlauf ist richtungsabhängig — Federkraft,
Schwerkraft und Reibung wirken beim Öffnen anders als beim Schließen. Bei
`MOTOR_TURN` erbt jede Fahrt die Endlage der vorherigen, die Differenz addiert
sich also Zyklus für Zyklus auf und die Nullage wandert sichtbar davon. Bei
`MOTOR_GOTO` bezieht sich jedes Ziel auf denselben Nullpunkt, wodurch der
Fehler beschränkt bleibt statt zu wachsen.

`MOTOR_GOTO` fährt nicht, wenn die Position bereits innerhalb von
`POS_DEADBAND` (`motor_hal.hpp`) um das Ziel liegt. Das Fenster **muss größer
sein als der Nachlauf** — sonst korrigiert ein erneuter Befehl den Nachlauf
zurück, der nächste wieder vor, und der Motor pendelt um den Zielpunkt.

Beides ist **keine Regelung**. Der Motor läuft mit der festen `TURN_SPEED`
(bewusst unabhängig vom PWM-Schieber der Bodenstation, damit der Nachlauf
reproduzierbar bleibt), und `update()` schaltet ihn ab, sobald der Encoder das
Ziel in Fahrtrichtung überschritten hat — der Encoder wirkt als Endschalter.
Es wird weder die Geschwindigkeit nachgeführt noch am Ziel nachkorrigiert.

**Gebremst statt ausgelaufen.** Am Ende jeder Fahrt und bei `MOTOR_OFF` legt der
OBC beide Brückenkanäle auf HIGH — laut DRV8871-Datenblatt (Table 1) „Brake;
low-side slow decay". Vorher standen beide auf LOW, also „Coast": Der Motor lief
frei aus, was den Nachlauf um ein Vielfaches vergrößerte. Der Bremsimpuls endet
nach `BRAKE_MS`, danach geht die Brücke wieder auf Coast, damit sie schlafen
kann und sich die Mechanik von Hand bewegen lässt.

Ohne Encoder verwirft der OBC `MOTOR_TURN`/`MOTOR_GOTO`, statt ungebremst
loszulaufen. Bleibt der Encoder während der Fahrt stehen (Mechanik fest, Kanal
ab), greift `TURN_STALL_MS` nach einer Sekunde und setzt das Telemetriebit
`0x08`; danach fängt der Laufzeit-Watchdog den Rest ab.

Beide Fahrten laufen mit `TURN_SPEED = MIN_DRIVE_SPEED`. Nach unten ist dort
kein Spielraum, der Nachlauf lässt sich also **nicht** über die Drehzahl
verkleinern — dagegen arbeiten allein der Bremsimpuls und `POS_DEADBAND`.

---

## 5. Pinbelegung des Testaufbaus (vorläufig)

Alle Werte in `include/pin_config.hpp`. Für den Bodentest genügen:

| Funktion | Teensy-Pin | Konstante |
|---|---:|---|
| Motor Kanal A (vorwärts) | 18 | `PIN_M1_A` |
| Motor Kanal B (rückwärts) | 19 | `PIN_M1_B` |
| Encoder A | 0 | `PIN_M1_ENC_A` |
| Encoder B | 1 | `PIN_M1_ENC_B` |
| IMU CS Accel | 37 | `PIN_CS_ACCEL` |
| IMU CS Gyro | 36 | `PIN_CS_GYRO` |
| SPI MOSI/MISO/SCK | 11 / 12 / 13 | `PIN_SPI1_*`, `PIN_SCK` |
| Down-/Uplink (Serial8) | 34 RX / 35 TX | `PIN_UPDOWNLINK_*` |
| REXUS L0 / SOE / SODS | 21 / 39 / 38 | `PIN_L0_T`, `PIN_SOE_I`, `PIN_SODS_I` |

Die Motorpins 18/19 liegen auf QuadTimer3_1 bzw. QuadTimer3_0, haben also
echtes Hardware-PWM — `MotorHAL` taktet sie mit 20 kHz per `analogWrite()`.
Die Software-PWM (`IntervalTimer`, 1 kHz) bleibt als Rückfallebene im Code, für
den Fall, dass der Motor auf Pins ohne PWM-Timer umzieht (z. B. 40/41). Welcher
Modus aktiv ist, steht beim Booten auf der Debug-Konsole:
`Motor 1 bereit (Treiber Pin 18/19, Encoder Pin 0/1, Hardware-PWM)`.

Kanal A ist die Vorwärtsrichtung. Dreht der Motor verkehrt herum, `PIN_M1_A`
und `PIN_M1_B` in `pin_config.hpp` tauschen.

Abweichung zur Platine: In `docs/teensyPins/MAGGIE-OCB-PIN-BELEGUNG.txt` liegt
`M1_B` auf Pin 14, Pin 19 ist dort `CAMDIR1`. 18/19 ist die Verdrahtung des
Tischaufbaus — vor dem Flug abgleichen.

`COUNTS_PER_REV = 4550` ergibt sich direkt aus der Bestückung. Der Encoder
(12 CPR, back connector) sitzt auf der **Motorwelle**, nicht auf der
Abtriebswelle — also gilt
`Counts/Abtriebsumdrehung = Encoder-CPR × Getriebeübersetzung`:

```
12 CPR × 379.17 = 4550.04  →  COUNTS_PER_REV = 4550
```

Die 379.17:1 sind die exakte Übersetzung des verbauten Getriebes, nicht die
gerundete Typbezeichnung. Der Wert ist damit hergeleitet und nicht nur gemessen.

Der Wert steht doppelt — in `include/hal/motor_hal.hpp` und in
`MAGGIE_SERVER/app/services/downlink_frame_parser.py` — und muss an beiden
Stellen gleich sein. Er bestimmt die Winkelanzeige **und** das Ziel von
`MOTOR_TURN`; die rohen Encoder-Counts sind davon unabhängig.

---

## 6. Ablauf im Review

1. Server-`.env` auf die beiden seriellen Ports zum RXSM-Testmodul setzen
   (`SERIAL_PORT` = Downlink, `TC_SERIAL_PORT` = Telecommand), Server starten.
2. Ground Station öffnen → *Telecommands*. Sobald Frames ankommen, zeigt der
   Kopf „Downlink aktiv" und der Zustand steht auf `PRE_LAUNCH`.
3. **Test-Modus betreten** — der Zustandschip springt auf `TEST`, die
   Aktor-Buttons werden frei.
4. **Encoder nullen**, PWM-Sollwert auf 120 stellen, **Drehen** → der Motor
   läuft an, die Encoder-Counts steigen gleichmäßig, PWM ist zeigt 120.
   **Stopp** → Counts bleiben stehen, PWM ist geht auf 0.
5. **Richtung umkehren** und erneut **Drehen** → die Counts laufen rückwärts,
   PWM ist zeigt -120. Damit sind beide Drehrichtungen und das Vorzeichen des
   Encoders verifiziert.
6. **½ Umdrehung** → der Motor läuft an, der Chip zeigt „Drehung um 180° läuft",
   und der OBC stoppt nach 2300 Counts von selbst. Der Endwert liegt um den
   Auslauf über dem Ziel — genau das ist die Aussage des Tests.
7. *Telemetrie* zeigt IMU-Beschleunigung/Drehrate sowie Encoder-Counts, Winkel
   und PWM als Live-Verlauf.
8. **Test-Modus verlassen** — der Motor stoppt, die Aktoren sind wieder
   gesperrt.

Ohne Hardware lässt sich derselbe Ablauf mit
`MAGGIE_SERVER/tools/rxsm_obc_simulator.py` durchspielen.
