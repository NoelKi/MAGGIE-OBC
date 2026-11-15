#!/usr/bin/env python3
"""Kompatibler Einstiegspunkt; bevorzugt wird ``lisa-parse``."""

from lisa.cli import main


if __name__ == "__main__":
    raise SystemExit(main())
