#!/bin/bash
# run.sh — Autonomous drone mission launcher.
#
# Manual 3-terminal workflow:
#
#   STEP 1 — SITL (ArduPilot + MAVProxy)
#   ──────────────────────────────────────
#   First-ever run (wipes EEPROM, loads no_gps.parm):
#     bash control/launch_sitl.sh --wipe
#     → Wait for "Received 1346 parameters", then type: reboot
#
#   All subsequent runs (EEPROM already has params):
#     bash control/launch_sitl.sh
#
#   STEP 2 — MAVROS2 bridge
#   ────────────────────────
#     bash control/launch_mavros.sh
#
#   STEP 3 — Flight commander (after MAVROS2 prints "CON: Got HEARTBEAT")
#   ──────────────────────────────────────────────────────────────────────
#     bash control/launch_commander.sh
#
# ──────────────────────────────────────────────────────────────────────────────
# Automated tmux mode (all 3 windows + auto-reboot on first run):
#
#   bash run.sh --tmux          — reuse existing EEPROM
#   bash run.sh --tmux --wipe   — wipe EEPROM and reload params (first run)
# ──────────────────────────────────────────────────────────────────────────────

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

print_usage() {
    echo ""
    echo "Usage:"
    echo "  bash run.sh                  — print this help"
    echo "  bash run.sh --tmux           — launch all 3 in tmux (existing EEPROM)"
    echo "  bash run.sh --tmux --wipe    — launch all 3 in tmux (wipe EEPROM first)"
    echo ""
}

if [[ "$1" == "--help" || "$1" == "-h" ]]; then
    print_usage
    exit 0
fi

# ── tmux mode ──────────────────────────────────────────────────────────────────
if [[ "$1" == "--tmux" || "$2" == "--tmux" ]]; then
    if ! command -v tmux &>/dev/null; then
        echo "[run.sh] tmux not found.  sudo apt install tmux"
        exit 1
    fi

    # Accept --wipe in either argument position
    WIPE=""
    [[ "$1" == "--wipe" || "$2" == "--wipe" ]] && WIPE="--wipe"

    SESSION="drone_mission"
    SITL_LOG="/tmp/drone_sitl_$$.log"
    MAVROS_LOG="/tmp/drone_mavros_$$.log"

    # ── kill any stale session ───────────────────────────────────────────────
    echo "[run.sh] Cleaning up old processes..."
    tmux kill-session -t "$SESSION" 2>/dev/null || true
    pkill -9 -f 'arducopter|mavproxy|mavros_node|flight_commander' 2>/dev/null || true
    sleep 2

    # ── Window 0: SITL ──────────────────────────────────────────────────────
    echo "[run.sh] Starting SITL${WIPE:+ (--wipe)}..."
    tmux new-session -d -s "$SESSION" -x 220 -y 50
    tmux rename-window -t "$SESSION:0" "SITL"
    tmux send-keys -t "$SESSION:0" \
        "bash '$SCRIPT_DIR/control/launch_sitl.sh' $WIPE 2>&1 | tee '$SITL_LOG'; exec bash" \
        Enter

    # Poll until MAVProxy has saved all parameters (up to 120 s)
    echo -n "[run.sh] Waiting for SITL params to load"
    WAITED=0
    while ! grep -q "Saved 1346 parameters" "$SITL_LOG" 2>/dev/null; do
        sleep 2; WAITED=$((WAITED+2)); echo -n "."
        if [[ $WAITED -ge 120 ]]; then
            echo ""
            echo "[run.sh] ERROR: Params not loaded after 120 s — check the SITL window."
            tmux attach-session -t "$SESSION"; exit 1
        fi
    done
    echo " done."

    # On --wipe runs: auto-send 'reboot' so params persist to eeprom
    if [[ -n "$WIPE" ]]; then
        echo "[run.sh] Sending 'reboot' to MAVProxy to persist params..."
        tmux send-keys -t "$SESSION:0" "reboot" Enter

        # Poll until SITL comes back online (second 'ArduPilot Ready' = post-reboot boot)
        echo -n "[run.sh] Waiting for SITL reboot"
        WAITED=0
        while true; do
            sleep 2; WAITED=$((WAITED+2)); echo -n "."
            COUNT=$(grep -c "ArduPilot Ready" "$SITL_LOG" 2>/dev/null || echo 0)
            [[ "$COUNT" -ge 2 ]] && break
            if [[ $WAITED -ge 90 ]]; then
                echo ""
                echo "[run.sh] ERROR: SITL didn't come back after reboot — check the SITL window."
                tmux attach-session -t "$SESSION" 2>/dev/null || true; exit 1
            fi
        done
        echo " done."
    fi
    sleep 2  # brief extra settle time

    # ── Window 1: MAVROS2 ────────────────────────────────────────────────────
    echo "[run.sh] Starting MAVROS2..."
    tmux new-window -t "$SESSION"
    tmux rename-window -t "$SESSION:1" "MAVROS2"
    tmux send-keys -t "$SESSION:1" \
        "bash '$SCRIPT_DIR/control/launch_mavros.sh' 2>&1 | tee '$MAVROS_LOG'; exec bash" \
        Enter

    # Poll until MAVROS2 gets a heartbeat (up to 60 s)
    echo -n "[run.sh] Waiting for MAVROS2 heartbeat"
    WAITED=0
    while ! grep -q "Got HEARTBEAT, connected" "$MAVROS_LOG" 2>/dev/null; do
        sleep 2; WAITED=$((WAITED+2)); echo -n "."
        if [[ $WAITED -ge 60 ]]; then
            echo ""
            echo "[run.sh] ERROR: MAVROS2 heartbeat not received — check the MAVROS2 window."
            tmux attach-session -t "$SESSION"; exit 1
        fi
    done
    echo " done."
    sleep 2  # brief extra settle time

    # ── Window 2: Flight Commander ───────────────────────────────────────────
    echo "[run.sh] Starting flight commander..."
    tmux new-window -t "$SESSION"
    tmux rename-window -t "$SESSION:2" "Commander"
    tmux send-keys -t "$SESSION:2" \
        "bash '$SCRIPT_DIR/control/launch_commander.sh'; exec bash" \
        Enter

    # Switch to SITL window so user sees the MAVProxy console first
    tmux select-window -t "$SESSION:0"
    echo "[run.sh] All services running. Attaching tmux (Ctrl-B 0/1/2 = SITL/MAVROS/Commander)"
    # Attach only when running in a real terminal
    if [ -t 1 ]; then
        tmux attach-session -t "$SESSION"
    else
        echo "[run.sh] Not a TTY — tmux session '$SESSION' is running in background."
        echo "[run.sh] Connect manually with:  tmux attach -t $SESSION"
    fi
    exit 0
fi

# ── print instructions mode (default) ─────────────────────────────────────────
echo ""
echo "════════════════════════════════════════════════"
echo "  Drone Mission — Launch Sequence"
echo "════════════════════════════════════════════════"
echo ""
echo "  Open 3 terminals and run in order:"
echo ""
echo "  [Terminal 1]  bash control/launch_sitl.sh --wipe    # first run"
echo "                → Wait for 'Saved 1346 parameters', then type: reboot"
echo "                bash control/launch_sitl.sh            # subsequent runs"
echo ""
echo "  [Terminal 2]  bash control/launch_mavros.sh"
echo "                → Wait for: CON: Got HEARTBEAT, connected."
echo ""
echo "  [Terminal 3]  bash control/launch_commander.sh"
echo ""
echo "════════════════════════════════════════════════"
print_usage
