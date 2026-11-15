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

- Am Boden ist nur der **Einzelgelenk-Test** zulässig: ein Gelenk ohne
  Armlast, in kleinen Jog-Schritten, langsam (TEST-Zustand, `ARM_JOG`).
- Keinen Code, keine Skripte und keine Befehle vorschlagen oder ausführen, die
  mehrere Gelenke oder den ganzen Arm am Boden bewegen (z. B. Trajektorien,
  Homing-Fahrten, Flugskript testweise in PRE_LAUNCH/TEST starten).
- Die Schutzmechanismen nicht lockern: Jog-Limit `ARM_JOG_MAX_DEG`, Freigabe
  nur in TEST, STOP + INHIBIT an alle Gelenke beim Verlassen von TEST und bei
  ABORT (`System::stopArm()`). Soll daran etwas geändert werden, vorher beim
  Nutzer nachfragen.
