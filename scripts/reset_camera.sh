#!/usr/bin/env bash
# Recovers a D435i that enumerates but will not stream -- the symptom is:
#
#   RuntimeError: Frame didn't arrive within 5000
#
# raised from the first wait_for_frames(), while `lsusb` still shows the camera
# happily on USB 3.2 and the pipeline claims to start. The device gets into
# this state after an unclean stop (a SIGKILLed process, a tmux kill-server, a
# crash mid-stream) which leaves the streaming interface open on the camera
# side with nothing reading it.
#
# Issues a firmware-level reset over USB, which is equivalent to replugging the
# cable but does not require physical access to the machine. Then confirms the
# camera actually delivers frames again, so a pass here means the next
# `python main.py` will work rather than merely that the device re-enumerated.
#
# Usage:
#   ./reset_camera.sh
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PYTHON="$REPO_ROOT/.venv/bin/python"

if [[ ! -x "$PYTHON" ]]; then
  echo "error: no venv at $REPO_ROOT/.venv -- see doc/jetson_orin_setup.md" >&2
  exit 1
fi

# A process still holding the device makes the reset pointless: it will grab
# the camera again as soon as it re-enumerates, or block the reset outright.
if pgrep -f "main.py" >/dev/null 2>&1; then
  echo "error: main.py is still running and holding the camera." >&2
  echo "       stop it first (Ctrl-C, or tmux kill-session -t rs_detect)" >&2
  exit 1
fi

cd "$REPO_ROOT"
exec "$PYTHON" - <<'PY'
import sys
import time

import pyrealsense2 as rs

from camera import RealSenseCamera

devs = rs.context().query_devices()
if not devs:
    sys.exit("No RealSense device found -- check the USB cable, then re-run.")

dev = devs[0]
print(f"Resetting {dev.get_info(rs.camera_info.name)} "
      f"(serial {dev.get_info(rs.camera_info.serial_number)})...")
dev.hardware_reset()

# The device drops off the bus and comes back; ~2s is typical, allow plenty.
for i in range(30):
    time.sleep(1)
    if rs.context().query_devices():
        print(f"  re-enumerated after {i + 1}s")
        break
else:
    sys.exit("Device did not come back on the bus -- replug the cable.")

time.sleep(2)  # settle before opening the pipeline

# Re-enumerating is not the same as streaming, which is the thing that was
# broken -- so prove frames actually flow before reporting success.
print("Verifying frames...")
cam = RealSenseCamera().start()
try:
    ok = sum(cam.get_aligned_frames()[0] is not None for _ in range(10))
finally:
    cam.stop()

if ok == 10:
    print("OK -- camera is streaming again.")
else:
    sys.exit(f"Still broken: only {ok}/10 frames arrived. Replug the cable, "
             "or try a different USB 3 port.")
PY
