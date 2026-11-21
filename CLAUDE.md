# CLAUDE.md – MAGGIE OBC

Firmware für den On-Board-Computer (Teensy 4.1, PlatformIO). Überblick über
Module, Datenraten und Telecommands: `docs/HAL_OVERVIEW.md`.

## Feste Regeln

### 1. `extern/lisa-if/` niemals verändern

Der Ordner ist eine unveränderte Kopie des LISA-Referenzparsers aus einem
fremden Repo (Quelle und Stand: `extern/README.md`). Keine Datei darin
bearbeiten, umbenennen, formatieren, löschen oder ergänzen – auch keine
"kleinen Korrekturen".

- Nur lesen und ausführen ist erlaubt, z. B. um Firmware-Code dagegen zu
  testen. Dabei `PYTHONDONTWRITEBYTECODE=1` setzen, damit kein `__pycache__`
  im Ordner entsteht.
- Wenn etwas in lisa-if falsch aussieht: dem Nutzer melden, nicht selbst ändern.
  Änderungen macht das Team im Original-Repo und kopiert den Ordner dann neu.

### 2. Roboterarm nicht unter Schwerkraft fahren

Die Gelenke des Arms sind nur für Mikrogravitation ausgelegt. Am Boden können
sie das Gewicht des Arms nicht halten – wird der ganze Arm unter Schwerkraft
bewegt, gehen die Gelenke kaputt. Der vollständige Arm wird deshalb erst in
Mikrogravitation getestet.

- Am Boden sind nur zulässig:
  - der **Einzelgelenk-Test**: ein Gelenk ohne Armlast, in kleinen
    Jog-Schritten, langsam (TEST-Zustand, `ARM_JOG`);
  - der **Trajektorientest mit höchstens zwei Gelenken** im Testaufbau
    (`ARM_TRAJ_TEST`, `ArmTrajPlayer`, `tools/arm_trajectory.py`): nur in TEST,
    je Gelenk höchstens ±30° um den Startwinkel, Ende = Start, höchstens
    10 °/s und 4 °/s², Abbruch mit STOP + INHIBIT bei Schleppfehler oder fehlender
    Rückmeldung. Vom Team am 15.11.2026 für den aktuellen Testaufbau
    freigegeben (z. Zt. Gelenk 1 = Node 5, Gelenk 2 = Node 6).
- Keinen Code, keine Skripte und keine Befehle vorschlagen oder ausführen, die
  mehr als zwei Gelenke oder den ganzen Arm am Boden bewegen, oder zwei
  Gelenke außerhalb des Trajektorientests oben (z. B. Homing-Fahrten,
  Flugskript testweise in PRE_LAUNCH/TEST starten).
- Die Schutzmechanismen nicht lockern: Jog-Limit `ARM_JOG_MAX_DEG`, die
  Grenzen des `ArmTrajPlayer` (Gelenkzahl, Winkel, Geschwindigkeit), Freigabe
  nur in TEST, STOP + INHIBIT an alle Gelenke beim Verlassen von TEST und bei
  jeder Rueckkehr nach PRE_LAUNCH (`System::stopArm()`). Soll daran etwas geändert werden, vorher beim
  Nutzer nachfragen.

### 3. Flug-Trajektorie (`ArmFlightPlayer`)

Der ganze Arm fährt nur im Flug: `ArmFlightPlayer` spielt
`include/arm_flight_trajectory.hpp` (aus der Flug-CSV, `tools/arm_trajectory.py
flight`) im Zustand ROBOT_ARM ab. Vom Nutzer am 17.11.2026 so festgelegt:

- Freigabe in ROBOT_ARM: Flug-Schalter (`ARM_FLIGHT_ENABLE`, GUI) ODER
  gemessene Schwerelosigkeit (`MicrogravityDetector`, OBC- oder RCU-IMU).
  Der Schalter geht nur in PRE_LAUNCH an. TEST betreten und jede Rueckkehr
  nach PRE_LAUNCH (STATE_RESET, TEST_EXIT) schalten ihn aus, ein Neustart nicht.
  Schwelle, Haltezeit und die Schutzpruefungen gegen einen defekten Sensor
  (Nullvektor, eingefrorener Wert, Luecke) nicht lockern.
- Bei Gelenkproblemen weiterfahren, nicht abbrechen und nicht direkt in die
  Verstaupose (Kollisionsgefahr). Nach OBC-Neustart am gesicherten Schritt
  fortsetzen.
- Die Tabelle wird beim Booten geprueft (CRC, Grenzen, Ende = Start); faellt
  die Pruefung durch, faehrt der Arm nicht. Die Grenzen in `ArmFlightPlayer`
  und `FLIGHT_*` im Skript gehoeren zusammen.
- Den Flug-Player nie in TEST/PRE_LAUNCH starten und keine Abkuerzung bauen,
  die ihn am Boden ohne Schalter/Schwerelosigkeit startet.
- Aenderungen am Player erst in der Host-Simulation pruefen:
  `tools/flight_sim/run.sh` muss "ALLES OK" melden.

### 4. Kameras (`CameraHAL`)

4x RunCam Split 4 V2 an Serial5 ueber den CD4052B-Mux (Pins in
`pin_config.hpp`). Vom Nutzer am 17.11.2026 so festgelegt:

- Die Kameras nehmen selbst ab dem Einschalten auf (Kamera-Einstellung) und
  laufen bis Strom aus. Der OBC schickt im Flug **nie** STOP_RECORDING und
  nie die Power-Taste (schaltet um, ohne Rueckmeldung).
- Zur Sicherheit START_RECORDING an alle bei ARMED, ASCENT, EXPERIMENT und
  nach einem Neustart im Flug (`System::CAM_SAFETY_START`). Vor dem Flug am
  Boden pruefen, dass START eine laufende Aufnahme nicht beendet.
- STOP per Telecommand nur am Boden (PRE_LAUNCH/TEST).
- Aenderungen am Treiber erst im Host-Test pruefen:
  `tools/camera_sim/run.sh` muss "ALLES OK" melden.

### 5. Nie haengen bleiben, keine Reset-Schleife

Vom Nutzer am 19.11.2026 als "wirklich wichtig" festgelegt:

- Im Loop nichts Blockierendes. Was haengen koennte, bekommt eine Brotkrume
  (`bootguard::stage()` / `bootguard::Scope`, `boot_guard.hpp`), damit der
  Boot-Guard nach einem Watchdog-Reset das schuldige Subsystem auslaesst.
  Neue verzichtbare Subsysteme dort mit eigener Stage und `GUARD_OFF_*` anlegen.
- Der Watchdog startet als Erstes in `setup()`. Nicht wieder ans Ende von
  `init()` schieben. In `init()` zwischen langen Schritten `watchdogFeed()`.
- Minimalmodus (ab 3 Resets in Folge) und das Auslassen nach einem Haenger
  nicht entfernen oder lockern, ohne den Nutzer zu fragen.

### 6. SD-Karte (`SdLogger`)

- **Alles** kommt auf die Karte, unabhaengig vom Downlink, als CSV je
  Subsystem unter `/MAGGIE/<Datum>/<Uhrzeit>_B<Boot>/` (Nutzer, 19.11.2026).
  Neue Messwerte bekommen einen eigenen Datensatz-Typ und eine Spalte bzw.
  Datei, und `LIESMICH.TXT` (Text in `sd_logger.cpp`) wird mitgepflegt.
- `push*()` darf nie auf die Karte zugreifen; Kartenzugriffe nur in
  `service()`, hoechstens einer pro Loop, nie waehrend ein Motor laeuft.
- Debug-Ausgaben mit `Log.printf(...)` statt `Serial.printf(...)` - sonst
  fehlen sie in `LOG.txt`.
- Aenderungen am Logger erst im Host-Test pruefen:
  `tools/sd_sim/run.sh` muss "ALLES OK" melden.

### 7. Downlink-Prioritaeten

Vom Nutzer am 19.11.2026 festgelegt (`DownlinkPrio`, Raten in `data_rates.hpp`):

1. **Status** (harte Prioritaet): Zustand, HDRM offen, Arm/Kameras/Sensoren an
   - alles in SYS/STATE (2 Hz + bei jedem Wechsel), dazu die Diagnose-Frames.
2. **Bewegung**: Gelenkwinkel (ARM/JOINT) - man muss sehen, dass der Arm faehrt.
3. **Rest**: alle Sensordaten, ausgeduennt, sodass die Summe in die Leitung passt.

- Neue Frames einer Stufe zuordnen (`TelemetryDownlink::prioOf`) und keine
  Schuebe erzeugen (mehrere Frames auf einmal reihum verteilen).
- Aenderungen an Raten oder Stufen erst im Host-Test pruefen:
  `tools/downlink_sim/run.sh` muss fuer beide Baudraten "ALLES OK" melden.

### 8. HDRMs muessen aufgehen

Vom Nutzer am 19.11.2026: "es MUSS sich oeffnen".

- Jedes HDRM wird so oft angefahren, bis es offen ist
  (`System::updateHdrm`). "Offen" = mindestens `HDRM_OPEN_MIN_DEG` (140 Grad)
  in Oeffnungsrichtung, nicht die vollen 165 - ein paar Grad zu wenig duerfen
  den Arm nicht blockieren (Nutzer). Wiederholt wird nach jedem Fehlversuch mit Wackeln im
  Getriebespiel (`MotorHAL::wiggle`). Schliessen in DESCENT genauso.
- Der Arm faehrt erst, wenn BEIDE offen sind (`FLIGHT_HDRM_OPEN_OK`).
- Ein HDRM darf den Arm aber NIE dauerhaft blockieren ("Hauptsache es geht auf
  und der Arm startet", auch wenn dabei etwas kaputtgeht): Neustart mitten im
  Oeffnen -> ab aktueller Position weiter oeffnen, nach 3 Versuchen gilt es als
  offen. Encoder zaehlt nicht / fehlt -> auf Zeit fahren und als offen werten.
  Ohne Motor -> gilt als offen.
- NUR wenn ein HDRM so ohne Encoder-Beweis als offen gilt (`hdrm_blind_`):
  Arm-Trajektorie abbrechen und Arm stoppen, sobald ein antwortendes Gelenk
  `ARM_BLIND_STUCK_MS` (2 s) nicht vorankommt (Klappe vermutlich im Weg).
  Sonst gilt die normale Logik des Flug-Players (warten, dann weiterfahren).
- Der Boot-Guard schaltet den Arm NIE ab, auch nicht nach einem Haenger in
  der Arm-UART.
- Zeitvergleiche immer vorzeichenbehaftet (`static_cast<int32_t>(a - b)`),
  wenn eine der Zeiten aus `millis()` spaeter im Loop stammt als `now` -
  sonst wird die Differenz "negativ" = riesig (Fehler vom 19.11.2026: die
  30-s-Motorgrenze stoppte jede gerade gestartete HDRM-Fahrt).
