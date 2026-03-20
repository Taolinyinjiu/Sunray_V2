#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
EXPORT_SCRIPT="$SCRIPT_DIR/export_mocap_pose_csv.sh"

usage() {
  cat <<'EOF'
Manage a tmux session for mocap recording:
- pane 0: rosbag record (pose + twist)
- pane 1: CSV export from pose topic

Usage:
  record_mocap_tmux.sh start [options]
  record_mocap_tmux.sh stop [options]
  record_mocap_tmux.sh attach [options]
  record_mocap_tmux.sh status [options]

Options:
  --session <name>        tmux session name (default: sunray_mocap_rec)
  --out-dir <dir>         output directory
                         (default: /home/taolin/Desktop/Sunray_OpticalFlow_TestDir/MTF01/Mocap_Logs
                          or $SUNRAY_MOCAP_OUT_DIR if set)
  --bag-prefix <name>     rosbag -o prefix (default: Sunray_OpticalFlow_MTF01_Mocap_logs)
  --csv-prefix <name>     CSV file prefix (default: mocap_pose)
  --pose-topic <name>     pose topic (default: /vrpn_client_node_1/uav1/pose)
  --twist-topic <name>    twist topic (default: /vrpn_client_node_1/uav1/twist)
  --time-format <fmt>     CSV timestamp format (default: %Y%m%d_%H%M%S)
  --ros-setup <cmd>       command run before each pane command
                         (example: "source /opt/ros/noetic/setup.bash && source ~/catkin_ws/devel/setup.bash")
  --sync-panes            enable tmux synchronized panes (default: on)
                         With this on, pressing Ctrl+C in one pane sends to both panes.
  --no-sync-panes         disable tmux synchronized panes
  --auto-kill-on-exit     when either pane exits, stop peer pane and kill session (default: on)
  --no-auto-kill-on-exit  keep tmux session after a pane exits
  --no-attach             (start only) do not auto attach
  --kill                  (stop only) kill tmux session after Ctrl+C
  -h, --help              show help

Examples:
  ./record_mocap_tmux.sh start \
    --out-dir /home/taolin/Desktop/Sunray_OpticalFlow_TestDir/MTF01/Mocap_Logs

  ./record_mocap_tmux.sh stop --kill
EOF
}

err() {
  echo "[record_mocap_tmux] ERROR: $*" >&2
}

require_cmd() {
  if ! command -v "$1" >/dev/null 2>&1; then
    err "command not found: $1"
    exit 127
  fi
}

tmux_has_session() {
  local name="$1"
  tmux has-session -t "$name" 2>/dev/null
}

quote_cmd() {
  local out=""
  local arg
  for arg in "$@"; do
    out+="${out:+ }$(printf '%q' "$arg")"
  done
  echo "$out"
}

ACTION="${1:-}"
if [[ -z "$ACTION" ]]; then
  usage
  exit 2
fi
if [[ "$ACTION" == "-h" || "$ACTION" == "--help" ]]; then
  usage
  exit 0
fi
shift || true

SESSION="sunray_mocap_rec"
DEFAULT_OUT_DIR="${SUNRAY_MOCAP_OUT_DIR:-/home/taolin/Desktop/Sunray_OpticalFlow_TestDir/MTF01/Mocap_Logs}"
OUT_DIR="$DEFAULT_OUT_DIR"
BAG_PREFIX="Sunray_OpticalFlow_MTF01_Mocap_logs"
CSV_PREFIX="mocap_pose"
POSE_TOPIC="/vrpn_client_node_1/uav1/pose"
TWIST_TOPIC="/vrpn_client_node_1/uav1/twist"
TIME_FORMAT="%Y%m%d_%H%M%S"
ROS_SETUP=""
SYNC_PANES=1
AUTO_KILL_ON_EXIT=1
AUTO_ATTACH=1
KILL_SESSION=0

while [[ $# -gt 0 ]]; do
  case "$1" in
    --session)
      [[ $# -ge 2 ]] || { err "--session requires a value"; exit 2; }
      SESSION="$2"
      shift 2
      ;;
    --out-dir)
      [[ $# -ge 2 ]] || { err "--out-dir requires a value"; exit 2; }
      OUT_DIR="$2"
      shift 2
      ;;
    --bag-prefix)
      [[ $# -ge 2 ]] || { err "--bag-prefix requires a value"; exit 2; }
      BAG_PREFIX="$2"
      shift 2
      ;;
    --csv-prefix)
      [[ $# -ge 2 ]] || { err "--csv-prefix requires a value"; exit 2; }
      CSV_PREFIX="$2"
      shift 2
      ;;
    --pose-topic)
      [[ $# -ge 2 ]] || { err "--pose-topic requires a value"; exit 2; }
      POSE_TOPIC="$2"
      shift 2
      ;;
    --twist-topic)
      [[ $# -ge 2 ]] || { err "--twist-topic requires a value"; exit 2; }
      TWIST_TOPIC="$2"
      shift 2
      ;;
    --time-format)
      [[ $# -ge 2 ]] || { err "--time-format requires a value"; exit 2; }
      TIME_FORMAT="$2"
      shift 2
      ;;
    --ros-setup)
      [[ $# -ge 2 ]] || { err "--ros-setup requires a value"; exit 2; }
      ROS_SETUP="$2"
      shift 2
      ;;
    --sync-panes)
      SYNC_PANES=1
      shift
      ;;
    --no-sync-panes)
      SYNC_PANES=0
      shift
      ;;
    --auto-kill-on-exit)
      AUTO_KILL_ON_EXIT=1
      shift
      ;;
    --no-auto-kill-on-exit)
      AUTO_KILL_ON_EXIT=0
      shift
      ;;
    --no-attach)
      AUTO_ATTACH=0
      shift
      ;;
    --kill)
      KILL_SESSION=1
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      err "unknown option: $1"
      usage
      exit 2
      ;;
  esac
done

case "$ACTION" in
  start|stop|attach|status)
    ;;
  *)
    err "unknown action: $ACTION"
    usage
    exit 2
    ;;
esac

require_cmd tmux

if [[ "$ACTION" == "status" ]]; then
  if tmux_has_session "$SESSION"; then
    echo "[record_mocap_tmux] session '$SESSION' is running"
    tmux list-panes -t "$SESSION":0 -F "#I.#P #{pane_current_command} #{pane_active}"
    exit 0
  else
    echo "[record_mocap_tmux] session '$SESSION' is not running"
    exit 1
  fi
fi

if [[ "$ACTION" == "attach" ]]; then
  if ! tmux_has_session "$SESSION"; then
    err "session '$SESSION' not found"
    exit 1
  fi
  exec tmux attach -t "$SESSION"
fi

if [[ "$ACTION" == "stop" ]]; then
  if ! tmux_has_session "$SESSION"; then
    err "session '$SESSION' not found"
    exit 1
  fi

  echo "[record_mocap_tmux] stopping session: $SESSION"
  tmux send-keys -t "$SESSION":0.0 C-c || true
  tmux send-keys -t "$SESSION":0.1 C-c || true

  if [[ "$KILL_SESSION" -eq 1 ]]; then
    sleep 1
    tmux kill-session -t "$SESSION" || true
    echo "[record_mocap_tmux] session killed: $SESSION"
  else
    tmux setw -t "$SESSION":0 synchronize-panes off >/dev/null 2>&1 || true
    echo "[record_mocap_tmux] Ctrl+C sent to both panes (session kept)"
  fi
  exit 0
fi

# ACTION == start
require_cmd rosbag
if [[ ! -x "$EXPORT_SCRIPT" ]]; then
  err "missing helper script: $EXPORT_SCRIPT"
  exit 1
fi

if tmux_has_session "$SESSION"; then
  err "session '$SESSION' already exists"
  echo "[record_mocap_tmux] use: $0 attach --session $SESSION"
  exit 2
fi

mkdir -p "$OUT_DIR"

bag_cmd="$(quote_cmd rosbag record -o "$BAG_PREFIX" "$POSE_TOPIC" "$TWIST_TOPIC")"
bag_line="cd $(printf '%q' "$OUT_DIR") && $bag_cmd"

csv_cmd="$(quote_cmd "$EXPORT_SCRIPT" --live --topic "$POSE_TOPIC" --out-dir "$OUT_DIR" --prefix "$CSV_PREFIX" --time-format "$TIME_FORMAT")"
csv_line="$csv_cmd"

if [[ -n "$ROS_SETUP" ]]; then
  bag_line="$ROS_SETUP && $bag_line"
  csv_line="$ROS_SETUP && $csv_line"
fi

if [[ "$AUTO_KILL_ON_EXIT" -eq 1 ]]; then
  session_q="$(printf '%q' "$SESSION")"
  pane0_q="$(printf '%q' "$SESSION:0.0")"
  pane1_q="$(printf '%q' "$SESSION:0.1")"

  bag_line="$bag_line; rc=\$?; tmux send-keys -t $pane1_q C-c >/dev/null 2>&1 || true; sleep 0.3; tmux kill-session -t $session_q >/dev/null 2>&1 || true; exit \$rc"
  csv_line="$csv_line; rc=\$?; tmux send-keys -t $pane0_q C-c >/dev/null 2>&1 || true; sleep 0.3; tmux kill-session -t $session_q >/dev/null 2>&1 || true; exit \$rc"
fi

echo "[record_mocap_tmux] creating tmux session: $SESSION"
echo "[record_mocap_tmux] pane0 rosbag:  $bag_line"
echo "[record_mocap_tmux] pane1 csv:     $csv_line"

tmux new-session -d -s "$SESSION" -n "mocap"
tmux send-keys -t "$SESSION":0.0 "$bag_line" C-m
tmux split-window -h -t "$SESSION":0
tmux send-keys -t "$SESSION":0.1 "$csv_line" C-m
tmux select-layout -t "$SESSION":0 even-horizontal
if [[ "$SYNC_PANES" -eq 1 ]]; then
  tmux setw -t "$SESSION":0 synchronize-panes on
fi

echo "[record_mocap_tmux] started. Output dir: $OUT_DIR"
if [[ "$SYNC_PANES" -eq 1 ]]; then
  echo "[record_mocap_tmux] synchronized panes: ON (Ctrl+C in one pane stops both)"
else
  echo "[record_mocap_tmux] synchronized panes: OFF"
fi
if [[ "$AUTO_KILL_ON_EXIT" -eq 1 ]]; then
  echo "[record_mocap_tmux] auto kill session: ON (Ctrl+C in either pane closes whole session)"
else
  echo "[record_mocap_tmux] auto kill session: OFF"
fi
echo "[record_mocap_tmux] stop with:"
echo "  $0 stop --session $SESSION"

if [[ "$AUTO_ATTACH" -eq 1 ]]; then
  exec tmux attach -t "$SESSION"
fi
