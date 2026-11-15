# LISA-ODISEE UART Protocol & Telemetry Parser

Python-Paket zur Dekodierung und Auswertung serieller UART-Telemetriedaten
gemaess LISA-ODISEE Interface Control Document (ICD) v0.7.

## Funktionsumfang

- CRC-32/MPEG-2-Pruefung
- Resynchronisation auf das Startbyte `0xAA`
- CRC-gepruefte Rekonstruktion eines verlorenen `0x0D`
- typisierte Payloadmodelle fuer RCU-IMU, Gelenkzustand, Spannung und Temperatur
- Parserdiagnostik fuer CRC-Fehler, unbekannte IDs und Teilframes
- inkrementeller Parser fuer blockweise UART-Daten
- JSON-, Hex-Text- und Binaereingaben

## Installation

Ab Python 3.10 kann das Projekt lokal installiert werden:

```powershell
py -m pip install -e .
```

Auf Systemen, auf denen der Interpreter als `python` verfuegbar ist:

```bash
python -m pip install -e .
```

Danach steht das Konsolenprogramm `lisa-parse` zur Verfuegung.

## Kommandozeile

```bash
# Mit den mitgelieferten Beispieldaten
lisa-parse

# Eigene Messdaten
lisa-parse messung.json
lisa-parse telemetry.hex
lisa-parse stream.bin

# Alle IMU-Zeilen anzeigen, CR-Recovery abschalten oder Diagramm erstellen
lisa-parse --all messung.json
lisa-parse --no-recover-cr messung.json
lisa-parse --plot messung.json
lisa-parse --save-plot imu_plot.png messung.json
```

Ohne Installation funktionieren aus der Projektwurzel weiterhin:

```bash
python parse_lisa_uart.py --save-plot imu_plot.png
python -m lisa
```

## Python-API

Die einfache API liefert wie bisher nur gueltige Frames:

```python
from lisa import RcuImuPayload, load_data, parse_stream, plot_imu_data

frames = parse_stream(load_data("messung.json"))
for frame in frames:
    payload = frame.decoded_payload
    if isinstance(payload, RcuImuPayload):
        print(payload.acc_g, payload.gyro_dps)

# IMU-Diagramm als PNG speichern
plot_imu_data(frames, save_path="imu_plot.png")
```


Fuer Auswertungen sollte die detaillierte API verwendet werden:

```python
from lisa import load_data, parse_stream_detailed

result = parse_stream_detailed(load_data("messung.json"))
print(f"Frames: {len(result.frames)}")
print(f"CRC-Fehler: {result.crc_errors}")
print(f"Rekonstruierte CR-Bytes: {result.restored_frames}")
print(f"Unvollstaendiger Rest: {len(result.incomplete_tail)} Bytes")
```

Blockweise gelesene UART-Daten koennen ohne Verlust von Teilframes verarbeitet
werden:

```python
from lisa import StreamParser

parser = StreamParser()
for chunk in uart_chunks:
    result = parser.feed(chunk)
    process(result.frames)

final_result = parser.flush()
```

## Projektstruktur

```text
lisa-if/
|-- lisa/
|   |-- cli.py          # Konsolenoberflaeche
|   |-- crc.py          # CRC-32/MPEG-2
|   |-- frame.py        # Frame-Modell
|   |-- loader.py       # Eingabeformate
|   |-- parser.py       # Batch- und Streamparser
|   |-- payloads.py     # Typisierte Payloadmodelle und Decoder
|   `-- protocol.py     # IDs und zentrale Message-Spezifikationen
|-- tests/              # Unit- und Integrationstests
|-- parse_lisa_uart.py  # kompatibler Einstiegspunkt
|-- pyproject.toml      # Paket- und CLI-Metadaten
`-- sample_data.json    # reale Regressionstestdaten
```

## Tests

Die Tests verwenden neben synthetischen Frames auch die vorhandene
`sample_data.json` als Regressionstest:

```bash
python -m unittest discover -s tests -v
```

Der Beispieldatensatz muss 273 gueltige RCU-IMU-Frames ergeben, von denen 51
durch das Einfuegen eines fehlenden `0x0D` rekonstruiert werden.

## Noch nicht fachlich dekodierte Payloads

Nachrichten, deren genaue Feldsemantik im Projekt noch nicht hinterlegt ist,
werden als `RawPayload` bereitgestellt. Damit werden keine ungesicherten
Annahmen ueber `IMU_DATA`, `REFERENCE_ANGLE_DATA`, `IDENTIFIED` oder
Steuerparameter getroffen; ihre Laengen und CRCs werden trotzdem validiert.
