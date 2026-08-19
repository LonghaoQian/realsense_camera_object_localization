#!/usr/bin/env bash
# Launches the live detection + localization loop (main.py) in a *detached*
# tmux session, so it keeps running after you close the SSH connection.
#
# The loop runs headless -- there is no display to draw into from a detached
# session -- so its output is the per-detection console lines (label,
# confidence, XYZ in the color optical frame). Those go to the tmux pane and
# are also appended to logs/, which is the copy that survives if the process
# dies and the session with it.
#
# Any extra arguments are passed straight through to main.py.
#
# Usage:
#   ./run_detection_tmux.sh                    # start detached with defaults
#   ./run_detection_tmux.sh --conf 0.4         # forwarded to main.py
#   ./run_detection_tmux.sh --model yolov8n.engine
#   SESSION=my_run ./run_detection_tmux.sh     # override the session name
#
# Then:
#   tmux attach -t rs_detect     # watch it live; 'Ctrl-b d' detaches again
#   ./tmux_status.sh             # still running?
#   tmux kill-session -t rs_detect
set -euo pipefail

SESSION="${SESSION:-rs_detect}"
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VENV_ACTIVATE="$REPO_ROOT/.venv/bin/activate"
LOG_DIR="$REPO_ROOT/logs"
LOG_FILE="$LOG_DIR/${SESSION}_$(date +%Y%m%d_%H%M%S).log"

if [[ ! -f "$VENV_ACTIVATE" ]]; then
  echo "error: no venv at $REPO_ROOT/.venv" >&2
  echo "       see doc/jetson_orin_setup.md -- it must be created with --system-site-packages" >&2
  exit 1
fi

if tmux has-session -t "=$SESSION" 2>/dev/null; then
  echo "error: tmux session '$SESSION' is already running." >&2
  echo "       attach with:  tmux attach -t $SESSION" >&2
  echo "       or stop it:   tmux kill-session -t $SESSION" >&2
  exit 1
fi

mkdir -p "$LOG_DIR"

# Build the command with the extra args quoted individually, so things like
# --model "my model.pt" survive the trip through tmux's shell.
#
# `exec` and the process substitution are deliberate: a plain `python | tee`
# pipeline leaves the shell as the pane's foreground process, so tmux reports
# the pane's command as "bash" and tmux_status.sh cannot tell a live run from
# an idle shell. Exec'ing python in place makes the pane genuinely *be* the
# detection process, while >(tee) still puts output on both the pane and the
# log. Wrapped in an explicit `bash -c` because process substitution is a bash
# feature and tmux would otherwise use whatever default-shell is set to.
CMD="source $(printf %q "$VENV_ACTIVATE"); exec python -u main.py --headless"
for arg in "$@"; do
  CMD+=" $(printf %q "$arg")"
done
CMD+=" > >(tee $(printf %q "$LOG_FILE")) 2>&1"

tmux new-session -d -s "$SESSION" -c "$REPO_ROOT" "bash -c $(printf %q "$CMD")"

# Keep the pane around after the process exits so a crash is still readable
# on attach, rather than the whole session silently disappearing.
tmux set-option -t "$SESSION" remain-on-exit on >/dev/null

echo "Started detached tmux session '$SESSION'."
echo "  log:    $LOG_FILE"
echo "  attach: tmux attach -t $SESSION   (Ctrl-b d to detach)"
echo "  stop:   tmux kill-session -t $SESSION"
