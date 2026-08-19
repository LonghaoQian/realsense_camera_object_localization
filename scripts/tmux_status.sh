#!/usr/bin/env bash
# Reports which tmux sessions are alive on this machine, and what is actually
# running inside each one -- the useful question after reconnecting over SSH is
# usually "is my detection loop still going, or did it die?", which the session
# list alone does not answer.
#
# Exits 0 if the session(s) in question are alive, 1 if not, so it can gate
# other scripts:  ./tmux_status.sh rs_detect || ./run_detection_tmux.sh
#
# Usage:
#   ./tmux_status.sh              # all sessions; 0 if any exist
#   ./tmux_status.sh rs_detect    # just that one; 0 if it exists
set -euo pipefail

WANTED="${1:-}"

if ! tmux list-sessions >/dev/null 2>&1; then
  echo "No tmux server running -- no live sessions."
  exit 1
fi

if [[ -n "$WANTED" ]]; then
  if ! tmux has-session -t "=$WANTED" 2>/dev/null; then
    echo "Session '$WANTED' is not running."
    exit 1
  fi
fi

# Session line, then one line per pane showing the process it is running.
# A pane whose command is a bare shell means the program exited (or never
# started) and only the shell is left -- worth calling out explicitly.
#
# any_live stays 0 if every pane we looked at is dead: run_detection_tmux.sh
# sets remain-on-exit, so a crashed run leaves the session present but empty.
# Reporting that as success would make `tmux_status.sh rs_detect || relaunch`
# quietly do nothing after exactly the failure it is meant to catch.
any_live=0

while IFS=$'\t' read -r name attached windows created; do
  [[ -n "$WANTED" && "$name" != "$WANTED" ]] && continue
  printf '%s  [%s, %s window(s), started %s]\n' "$name" "$attached" "$windows" "$created"

  while IFS=$'\t' read -r idx cmd pid dead; do
    if [[ "$dead" == "1" ]]; then
      status="EXITED -- attach to read the error, or check logs/"
    else
      any_live=1
      if [[ "$cmd" =~ ^(bash|sh|zsh|fish)$ ]]; then
        # tmux reports the pane's foreground command, which is only the shell
        # when the pane runs a pipeline rather than the program directly.
        # Check the shell's children before calling it idle, otherwise a
        # perfectly healthy run looks dead.
        kids=$(ps --ppid "$pid" -o comm= 2>/dev/null | sort -u | tr '\n' ' ')
        if [[ -n "${kids// }" ]]; then
          status="$cmd (pid $pid) running: ${kids% }"
        else
          status="$cmd (pid $pid, idle shell -- nothing running)"
        fi
      else
        status="$cmd (pid $pid)"
      fi
    fi
    printf '    pane %s: %s\n' "$idx" "$status"
  done < <(tmux list-panes -t "=$name" -F '#{pane_index}'$'\t''#{pane_current_command}'$'\t''#{pane_pid}'$'\t''#{pane_dead}')
done < <(tmux list-sessions -F '#{session_name}'$'\t''#{?session_attached,attached,detached}'$'\t''#{session_windows}'$'\t''#{t:session_created}')

[[ "$any_live" -eq 1 ]]
