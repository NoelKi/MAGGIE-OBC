"""Validierende Laderoutinen fuer JSON-, Hex- und Binaerdaten."""

from __future__ import annotations

import json
from os import PathLike
from pathlib import Path
from typing import Any


PathInput = str | PathLike[str]
_DATA_KEYS = ("raw_hex", "hex", "data", "payload", "bytes")
_BINARY_SUFFIXES = {".bin", ".dat", ".raw"}
_TEXT_SUFFIXES = {".hex", ".txt"}


def _parse_hex_text(text: str) -> bytes:
    """Extrahiert Bytes aus Hex-Texten mit optionalen 0x-Praefixen, Kommas oder Kommentaren."""
    lines: list[str] = []
    for raw_line in text.splitlines():
        # Kommentare (# oder //) abschneiden
        line = raw_line.split("#")[0].split("//")[0].strip()
        if line:
            lines.append(line)
    cleaned = " ".join(lines)
    cleaned = (
        cleaned.replace("0x", "")
        .replace("0X", "")
        .replace(",", " ")
        .replace(";", " ")
        .replace(":", " ")
    )
    raw_hex = "".join(cleaned.split())
    if not raw_hex:
        return b""
    try:
        return bytes.fromhex(raw_hex)
    except ValueError as error:
        raise ValueError("String enthaelt keine gueltige Hex-Bytefolge") from error


def _extract_hex_bytes(obj: Any) -> bytes:
    """Extrahiert Bytes aus Strings, Listen oder verschachtelten Objekten."""

    if isinstance(obj, str):
        return _parse_hex_text(obj)

    if isinstance(obj, list):
        if all(type(value) is int for value in obj):
            invalid = [value for value in obj if not 0 <= value <= 0xFF]
            if invalid:
                raise ValueError(f"Bytewert ausserhalb 0..255: {invalid[0]}")
            return bytes(obj)
        if all(isinstance(value, str) for value in obj):
            return _extract_hex_bytes(" ".join(obj))

        # Verschachtelte Listen werden in ihrer Reihenfolge zusammengefuegt.
        parts = [_extract_hex_bytes(value) for value in obj]
        return b"".join(parts)

    if isinstance(obj, dict):
        for key in _DATA_KEYS:
            if key in obj:
                return _extract_hex_bytes(obj[key])

        # Metadaten-Huellen duerfen die eigentlichen Daten tiefer enthalten.
        errors: list[ValueError] = []
        for value in obj.values():
            try:
                return _extract_hex_bytes(value)
            except ValueError as error:
                errors.append(error)
        raise ValueError(
            "JSON-Objekt enthaelt keines der Datenfelder: " + ", ".join(_DATA_KEYS)
        ) from (errors[-1] if errors else None)

    raise ValueError(f"Nicht unterstuetzte Datenstruktur: {type(obj).__name__}")


def load_data(file_path: PathInput | None = None) -> bytes:
    """Laedt UART-Rohdaten aus JSON, Hex-Text (.txt, .hex) oder einer Binaerdatei."""

    path = _resolve_path(file_path)
    suffix = path.suffix.lower()

    if suffix in _BINARY_SUFFIXES:
        return path.read_bytes()

    if suffix == ".json":
        try:
            parsed_json = json.loads(path.read_text(encoding="utf-8"))
        except json.JSONDecodeError as error:
            raise ValueError(
                f"Ungueltiges JSON in '{path}' (Zeile {error.lineno}, "
                f"Spalte {error.colno})"
            ) from error
        try:
            return _extract_hex_bytes(parsed_json)
        except ValueError as error:
            raise ValueError(f"Ungueltige UART-Daten in '{path}': {error}") from error

    if suffix in _TEXT_SUFFIXES:
        content = path.read_text(encoding="utf-8")
        try:
            return _parse_hex_text(content)
        except ValueError as error:
            raise ValueError(f"Ungueltige Hex-Daten in '{path}'") from error


    supported = ", ".join(sorted(_BINARY_SUFFIXES | _TEXT_SUFFIXES | {".json"}))
    raise ValueError(f"Nicht unterstuetzter Dateityp '{suffix}'; erwartet: {supported}")


def _resolve_path(file_path: PathInput | None) -> Path:
    if file_path is not None:
        path = Path(file_path).expanduser()
    else:
        candidates = (
            Path.cwd() / "sample_data.json",
            Path(__file__).resolve().parent.parent / "sample_data.json",
        )
        path = next((candidate for candidate in candidates if candidate.is_file()), candidates[0])

    if not path.is_file():
        raise FileNotFoundError(f"Datei nicht gefunden: '{path}'")
    return path
