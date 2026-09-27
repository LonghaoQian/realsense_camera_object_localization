#!/usr/bin/env bash
# Bench-test launcher for the drone-mounted D435i: bottle detection + localization,
# published on ROS 2 so it can be checked and bagged, confined to the housekeeping CPUs.
#
# Checks the preconditions that otherwise fail silently or confusingly, then starts
# run_detection_tmux.sh (detached tmux session `rs_detect`) with the bench settings:
#   --classes bottle --conf 0.4 --ros --ros-image-hz 10 --stream 8080
# (q80 @ 10 Hz images is the setting settled on in commit 65d2f88: ~24 loops/s.)
#
# Does NOT start a rosbag -- run that separately, so its effect on the control loop
# can be measured as an A/B against this stack alone.
#
# Usage:
#   ./scripts/start_vision_bench.sh                # bench defaults
#   ./scripts/start_vision_bench.sh --conf 0.5     # extra args go to main.py
#   STREAM_PORT=8081 ./scripts/start_vision_bench.sh
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SESSION="${SESSION:-rs_detect}"
STREAM_PORT="${STREAM_PORT:-8080}"

ok()   { echo -e "\033[1;32m[ ok ]\033[0m $*"; }
warn() { echo -e "\033[1;33m[warn]\033[0m $*" >&2; }
die()  { echo -e "\033[1;31m[fail]\033[0m $*" >&2; exit 1; }

# --- 1. Camera present, and on a USB 3 link -----------------------------------------
# On USB 2 the D435i enumerates and streams, but depth/color at 640x480x30 degrade or
# drop -- which looks like a detection problem, not a cable problem.
CAM_SPEED=""
for dev in /sys/bus/usb/devices/*; do
  [[ -f "$dev/idVendor" && -f "$dev/idProduct" ]] || continue
  if [[ "$(cat "$dev/idVendor")" == "8086" && "$(cat "$dev/idProduct")" == "0b3a" ]]; then
    CAM_SPEED="$(cat "$dev/speed")"
    break
  fi
done
[[ -n "$CAM_SPEED" ]] || die "D435i (8086:0b3a) not found on USB. Check the cable; run lsusb."
if (( CAM_SPEED >= 5000 )); then
  ok "D435i on USB 3 (${CAM_SPEED} Mb/s)"
else
  warn "D435i is on a ${CAM_SPEED} Mb/s link (USB 2). Use a USB 3 port/cable before trusting results."
fi

# --- 2. Nothing else holds the camera -----------------------------------------------
# Only one process can open the D435i; the second fails with errno=16.
if tmux has-session -t "=$SESSION" 2>/dev/null; then
  die "tmux session '$SESSION' is already running. Stop it: tmux kill-session -t $SESSION"
fi
if pgrep -f realsense2_camera_node >/dev/null; then
  die "realsense2_camera_node is running and holds the camera. Stop it first."
fi
ok "camera is free"

# --- 3. CPU isolation active --------------------------------------------------------
ISOLATED="$(cat /sys/devices/system/cpu/isolated 2>/dev/null || true)"
if [[ "$ISOLATED" == "4-5" ]]; then
  ok "kernel isolated CPUs = 4-5; vision will be confined to 0-3"
else
  warn "kernel isolated CPUs = '${ISOLATED:-<none>}' (expected 4-5). Booted 'primary'?"
  warn "The CPU-isolation part of this bench test is meaningless until that is fixed."
fi

# --- 4. Launch ---------------------------------------------------------------------
SESSION="$SESSION" "$SCRIPT_DIR/run_detection_tmux.sh" \
  --classes bottle --conf 0.4 \
  --ros --ros-image-hz 10 \
  --stream "$STREAM_PORT" \
  "$@"

IP="$(hostname -I | awk '{print $1}')"
cat <<EOF

Vision is running (model load takes ~10-20 s before detections appear).

  watch detections:  tmux attach -t $SESSION             (Ctrl-b d to detach)
  view the feed:     http://${IP}:${STREAM_PORT}
  topics:            ros2 topic echo /vision/target/pose  (bottle XYZ, camera optical frame)
                     /vision/detections runs at ~24-25 Hz, one message per loop
  confirm placement: taskset -cp \$(pgrep -f '^python -u main.py')   (expect 0-3)
  stop:              tmux kill-session -t $SESSION
EOF
