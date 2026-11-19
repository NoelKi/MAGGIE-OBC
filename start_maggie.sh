#!/usr/bin/env bash
# =============================================================================
#  MAGGIE Ground Station – Startskript
#
#  Startet: InfluxDB (Docker)
#           -> MAGGIE-Server (Flask, http://localhost:3000)
#           -> MAGGIE-GS GUI  (Angular, http://localhost:4200)
#
#  Aufruf:  ./start_maggie.sh            alles starten
#           ./start_maggie.sh server     nur InfluxDB + Server
#           ./start_maggie.sh gui        nur die GUI
#
#  Beenden: Strg+C  (stoppt Server und GUI, InfluxDB läuft weiter)
# =============================================================================
set -euo pipefail

# Das Skript liegt in MAGGIE-OBC/, Server und GUI liegen daneben in Software/.
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SERVER_DIR="$ROOT/MAGGIE-Server/MAGGIE-SERVER"
GUI_DIR="$ROOT/MAGGIE-GUI/MAGGIE-GS"
SERVER_PORT=3000
GUI_PORT=4200
GUI_URL="http://localhost:$GUI_PORT"

MODE="${1:-all}"
case "$MODE" in
    all|server|gui) ;;
    -h|--help) sed -n '2,14p' "$0"; exit 0 ;;
    *) echo "Unbekannte Option: $MODE  (erlaubt: all | server | gui)"; exit 1 ;;
esac

# ── Ausgabe-Helfer ───────────────────────────────────────────────────────────
c_blue=$'\e[34m'; c_green=$'\e[32m'; c_yellow=$'\e[33m'; c_red=$'\e[31m'
c_mag=$'\e[35m'; c_cyan=$'\e[36m'; c_off=$'\e[0m'
info() { echo "${c_blue}[maggie]${c_off} $*"; }
ok()   { echo "${c_green}[maggie]${c_off} $*"; }
warn() { echo "${c_yellow}[maggie]${c_off} $*"; }
die()  { echo "${c_red}[maggie] FEHLER:${c_off} $*" >&2; exit 1; }

port_in_use() { ss -ltn "sport = :$1" 2>/dev/null | grep -q LISTEN; }

wait_for_http() {   # wait_for_http <url> <timeout_s>
    local url=$1 timeout=$2 i
    for ((i = 0; i < timeout; i++)); do
        curl -fs -o /dev/null "$url" && return 0
        sleep 1
    done
    return 1
}

# ── Aufräumen bei Strg+C / Ende ──────────────────────────────────────────────
PIDS=()
cleanup() {
    trap - INT TERM EXIT
    echo
    info "Stoppe Server/GUI ..."
    for pid in "${PIDS[@]}"; do
        pkill -TERM -P "$pid" 2>/dev/null || true   # Kinder (Flask-Reloader, ng)
        kill -TERM "$pid" 2>/dev/null || true
    done
    wait 2>/dev/null || true
    ok "Beendet. (InfluxDB läuft weiter – stoppen mit:"
    ok "         docker stop maggie_influxdb)"
    exit 0
}
trap cleanup INT TERM EXIT

# ── 1. Datenbanken + Server ──────────────────────────────────────────────────
start_server() {
    [[ -d "$SERVER_DIR" ]] || die "Server-Ordner fehlt: $SERVER_DIR"
    [[ -f "$SERVER_DIR/.env" ]] || die "$SERVER_DIR/.env fehlt (Vorlage: .env.example)"

    if port_in_use "$SERVER_PORT"; then
        warn "Port $SERVER_PORT ist schon belegt – Server läuft vermutlich bereits, überspringe."
        return
    fi

    # InfluxDB: einen vorhandenen Container wiederverwenden (die Namen
    # unterscheiden sich je nach Rechner/Server-Stand), sonst docker compose.
    command -v docker >/dev/null || die "docker ist nicht installiert"
    if port_in_use 8086; then
        ok "InfluxDB läuft bereits (Port 8086)."
    else
        info "Starte InfluxDB ..."
        local c started=""
        for c in maggie_influxdb influxdb; do
            if docker start "$c" >/dev/null 2>&1; then started=$c; break; fi
        done
        if [[ -n "$started" ]]; then
            ok "Container '$started' gestartet."
        else
            (cd "$SERVER_DIR" && docker compose up -d) || die "InfluxDB konnte nicht gestartet werden"
        fi
    fi
    if wait_for_http "http://localhost:8086/health" 30; then
        ok "InfluxDB bereit."
    else
        warn "InfluxDB antwortet nicht auf :8086 – starte den Server trotzdem."
    fi

    if [[ ! -x "$SERVER_DIR/.venv/bin/python" ]]; then
        info "Lege Python-venv an und installiere Abhängigkeiten ..."
        python3 -m venv "$SERVER_DIR/.venv"
        "$SERVER_DIR/.venv/bin/pip" install -q -r "$SERVER_DIR/requirements.txt"
    fi

    info "Starte MAGGIE-Server auf http://localhost:$SERVER_PORT ..."
    (
        cd "$SERVER_DIR"
        PYTHONUNBUFFERED=1 exec .venv/bin/python run.py
    ) > >(sed -u "s/^/${c_mag}[server]${c_off} /") 2>&1 &
    PIDS+=($!)

    if wait_for_http "http://localhost:$SERVER_PORT/api/health" 30; then
        ok "Server läuft."
    else
        warn "Server antwortet nach 30 s nicht auf /api/health – siehe [server]-Ausgabe."
    fi
}

# ── 2. GUI ───────────────────────────────────────────────────────────────────
start_gui() {
    [[ -d "$GUI_DIR" ]] || die "GUI-Ordner fehlt: $GUI_DIR"

    if port_in_use "$GUI_PORT"; then
        warn "Port $GUI_PORT ist schon belegt – GUI läuft vermutlich bereits, überspringe."
        return
    fi

    command -v npm >/dev/null || die "npm/Node.js ist nicht installiert"
    if [[ ! -d "$GUI_DIR/node_modules" ]]; then
        info "Installiere npm-Pakete (einmalig) ..."
        (cd "$GUI_DIR" && npm ci)
    fi

    info "Starte GUI auf $GUI_URL (erster Build dauert etwas) ..."
    (
        cd "$GUI_DIR"
        NG_CLI_ANALYTICS=false exec npx ng serve --port "$GUI_PORT"
    ) > >(sed -u "s/^/${c_cyan}[gui]${c_off}    /") 2>&1 &
    PIDS+=($!)

    # Browser öffnen, sobald die GUI erreichbar ist (im Hintergrund).
    (
        if wait_for_http "$GUI_URL" 180; then
            command -v xdg-open >/dev/null && xdg-open "$GUI_URL" >/dev/null 2>&1 || true
        fi
    ) &
}

# ── Los geht's ───────────────────────────────────────────────────────────────
[[ "$MODE" == all || "$MODE" == server ]] && start_server
[[ "$MODE" == all || "$MODE" == gui    ]] && start_gui

if ((${#PIDS[@]} == 0)); then
    ok "Nichts zu starten – alles läuft schon."
    trap - EXIT
    exit 0
fi

echo
ok "──────────────────────────────────────────────"
[[ "$MODE" != gui    ]] && ok " Server:  http://localhost:$SERVER_PORT"
[[ "$MODE" != server ]] && ok " GUI:     $GUI_URL"
ok " Beenden mit Strg+C"
ok "──────────────────────────────────────────────"
echo

# Läuft, bis Strg+C kommt oder einer der Prozesse abstürzt.
wait -n "${PIDS[@]}" || true
warn "Ein Prozess hat sich beendet – fahre den Rest herunter."
