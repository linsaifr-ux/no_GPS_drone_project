#!/bin/bash
# apply_px4_params.sh — set PX4 no-GPS + external-vision params on the running SITL.
# Uses the px4 client (px4-param) which talks to the px4 daemon.  Params persist in
# the rootfs parameters.bson, so a reboot after this keeps them.
#
# Usage: bash control/apply_px4_params.sh   (PX4 SITL must be running)

PX4_BIN="$HOME/PX4-Autopilot/build/px4_sitl_nolockstep/bin"
ROOTFS="$HOME/PX4-Autopilot/build/px4_sitl_nolockstep/rootfs"
PARAMS="$(dirname "$(readlink -f "$0")")/px4_no_gps.params"

cd "$ROOTFS" || exit 1
n=0
while read -r line; do
    # accept lines of the form: param set NAME VALUE   (# comments ignored)
    set -- $line
    if [ "$1" = "param" ] && [ "$2" = "set" ]; then
        "$PX4_BIN/px4-param" set "$3" "$4" >/dev/null 2>&1 && n=$((n+1))
    fi
done < "$PARAMS"
echo "[apply_px4_params] applied $n params"
"$PX4_BIN/px4-param" save >/dev/null 2>&1
echo "[apply_px4_params] saved. Reboot PX4 to apply EKF2 params cleanly."
