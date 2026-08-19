#!/usr/bin/env bash
# Kills EVERY tmux session on this machine by shutting down the tmux server.
#
# This is not limited to this project's sessions -- anything else running under
# tmux (a rosbag recording, someone else's SSH session, a long build) dies too.
# It lists what it is about to kill and asks first; pass -y to skip the prompt
# in a script.
#
# To stop just the detection loop instead:
#   tmux kill-session -t rs_detect
#
# Usage:
#   ./tmux_kill_all.sh
#   ./tmux_kill_all.sh -y
set -euo pipefail

ASSUME_YES=0
[[ "${1:-}" == "-y" || "${1:-}" == "--yes" ]] && ASSUME_YES=1

if ! tmux list-sessions >/dev/null 2>&1; then
  echo "No tmux server running -- nothing to kill."
  exit 0
fi

echo "About to kill ALL tmux sessions on $(hostname):"
tmux list-sessions -F '  #{session_name}  [#{session_windows} window(s), #{?session_attached,attached,detached}]'

if [[ "$ASSUME_YES" -ne 1 ]]; then
  read -r -p "Kill them all? [y/N] " reply
  if [[ ! "$reply" =~ ^[Yy]$ ]]; then
    echo "Aborted -- nothing killed."
    exit 1
  fi
fi

tmux kill-server
echo "All tmux sessions killed."
