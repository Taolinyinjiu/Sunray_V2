#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "$SCRIPT_DIR/../../.." && pwd)"

usage() {
  cat <<'USAGE_EOF'
Run localization_fusion and 3 topic monitors in one tmux session.

Usage:
  run_localization_fusion_only_tmux.sh start [options]
  run_localization_fusion_only_tmux.sh stop [options]
  run_localization_fusion_only_tmux.sh attach [options]
  run_localization_fusion_only_tmux.sh status [options]

Options:
  --session <name>         tmux session name (default: sunray_loc_fusion_only)
  --source-id <int>        localization_fusion source_id override
  --health-rate-hz <num>   localization_fusion health_rate_hz override
  --uav-name <name>        launch test_uav_name (default: uav)
  --uav-id <id>            launch test_uav_id (default: 1)

  --local-topic <topic>    local odom echo topic
  --global-topic <topic>   global odom echo topic
  --state-topic <topic>    odom_status echo topic

  --fusion-cmd <cmd>       override the roslaunch command
  --ros-setup <cmd>        setup command before each pane command
  --no-attach              start only: do not auto attach
  -h, --help               show help

Behavior:
  This script launches localization_fusion with test:=true by default so it can
  run standalone, then opens panes for local_odom, global_odom, and odom_status.
  Press Ctrl+C in any pane to close the whole tmux session.
USAGE_EOF
}

err() {
  echo "[loc_fusion_tmux] ERROR: $*" >&2
}

require_cmd() {
  if ! command -v "$1" >/dev/null 2>&1; then
    err "command not found: $1"
    exit 127
  fi
}

tmux_has_session() {
  tmux has-session -t "$1" 2>/dev/null
}

stop_session() {
  local session="$1"
  while read -r pane_id; do
    [[ -n "$pane_id" ]] && tmux send-keys -t "$pane_id" C-c || true
  done < <(tmux list-panes -t "$session":0 -F "#{pane_id}" 2>/dev/null || true)
  sleep 0.3
  tmux kill-session -t "$session" >/dev/null 2>&1 || true
}

build_pane_line() {
  local label="$1"
  local cmd="$2"
  echo "$ROS_SETUP && $cmd; rc=\$?; if [[ \$rc -eq 0 || \$rc -eq 130 ]]; then tmux kill-session -t $SESSION_Q >/dev/null 2>&1 || true; else echo \"[loc_fusion_tmux:$label] command exited with rc=\$rc (session kept)\"; fi"
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

SESSION="sunray_loc_fusion_only"
SOURCE_ID=""
HEALTH_RATE_HZ=""
UAV_NAME="uav"
UAV_ID="1"
LOCAL_ODOM_TOPIC=""
GLOBAL_ODOM_TOPIC=""
ODOM_STATE_TOPIC=""
FUSION_CMD_OVERRIDE=""
ROS_SETUP=""
AUTO_ATTACH=1

while [[ $# -gt 0 ]]; do
  case "$1" in
    --session)
      [[ $# -ge 2 ]] || { err "--session requires a value"; exit 2; }
      SESSION="$2"
      shift 2
      ;;
    --source-id)
      [[ $# -ge 2 ]] || { err "--source-id requires a value"; exit 2; }
      SOURCE_ID="$2"
      shift 2
      ;;
    --health-rate-hz)
      [[ $# -ge 2 ]] || { err "--health-rate-hz requires a value"; exit 2; }
      HEALTH_RATE_HZ="$2"
      shift 2
      ;;
    --uav-name)
      [[ $# -ge 2 ]] || { err "--uav-name requires a value"; exit 2; }
      UAV_NAME="$2"
      shift 2
      ;;
    --uav-id)
      [[ $# -ge 2 ]] || { err "--uav-id requires a value"; exit 2; }
      UAV_ID="$2"
      shift 2
      ;;
    --local-topic)
      [[ $# -ge 2 ]] || { err "--local-topic requires a value"; exit 2; }
      LOCAL_ODOM_TOPIC="$2"
      shift 2
      ;;
    --global-topic)
      [[ $# -ge 2 ]] || { err "--global-topic requires a value"; exit 2; }
      GLOBAL_ODOM_TOPIC="$2"
      shift 2
      ;;
    --state-topic)
      [[ $# -ge 2 ]] || { err "--state-topic requires a value"; exit 2; }
      ODOM_STATE_TOPIC="$2"
      shift 2
      ;;
    --fusion-cmd)
      [[ $# -ge 2 ]] || { err "--fusion-cmd requires a value"; exit 2; }
      FUSION_CMD_OVERRIDE="$2"
      shift 2
      ;;
    --ros-setup)
      [[ $# -ge 2 ]] || { err "--ros-setup requires a value"; exit 2; }
      ROS_SETUP="$2"
      shift 2
      ;;
    --no-attach)
      AUTO_ATTACH=0
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
    echo "[loc_fusion_tmux] session '$SESSION' is running"
    tmux list-panes -t "$SESSION":0 -F "#S:#I.#P #{pane_current_command} #{pane_active} #{pane_title}"
    exit 0
  fi
  echo "[loc_fusion_tmux] session '$SESSION' is not running"
  exit 1
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
  echo "[loc_fusion_tmux] stopping session: $SESSION"
  stop_session "$SESSION"
  echo "[loc_fusion_tmux] session killed: $SESSION"
  exit 0
fi

if tmux_has_session "$SESSION"; then
  err "session '$SESSION' already exists"
  echo "[loc_fusion_tmux] use: $0 attach --session $SESSION"
  exit 2
fi

UAV_NS="/${UAV_NAME}${UAV_ID}"
LOCAL_ODOM_TOPIC="${LOCAL_ODOM_TOPIC:-$UAV_NS/sunray/localization/local_odom}"
GLOBAL_ODOM_TOPIC="${GLOBAL_ODOM_TOPIC:-$UAV_NS/sunray/localization/global_odom}"
ODOM_STATE_TOPIC="${ODOM_STATE_TOPIC:-$UAV_NS/sunray/localization/odom_status}"

if [[ -z "$ROS_SETUP" ]]; then
  ROS_SETUP="source /opt/ros/noetic/setup.bash"
  if [[ -f "$WORKSPACE_ROOT/devel/setup.bash" ]]; then
    ROS_SETUP="$ROS_SETUP && source \"$WORKSPACE_ROOT/devel/setup.bash\""
  fi
fi

SESSION_Q="$(printf '%q' "$SESSION")"
WAIT_MASTER_CMD="until rostopic list >/dev/null 2>&1; do sleep 0.2; done"

if [[ -n "$FUSION_CMD_OVERRIDE" ]]; then
  FUSION_CMD="$FUSION_CMD_OVERRIDE"
else
  FUSION_CMD="roslaunch localization_fusion localization_fusion.launch test:=true test_uav_name:=$UAV_NAME test_uav_id:=$UAV_ID"
  [[ -n "$SOURCE_ID" ]] && FUSION_CMD="$FUSION_CMD source_id:=$SOURCE_ID"
  [[ -n "$HEALTH_RATE_HZ" ]] && FUSION_CMD="$FUSION_CMD health_rate_hz:=$HEALTH_RATE_HZ"
fi

LOCAL_ECHO_CMD="$WAIT_MASTER_CMD; rostopic echo \"$LOCAL_ODOM_TOPIC\""
GLOBAL_ECHO_CMD="$WAIT_MASTER_CMD; rostopic echo \"$GLOBAL_ODOM_TOPIC\""
STATE_ECHO_CMD="$WAIT_MASTER_CMD; rostopic echo \"$ODOM_STATE_TOPIC\""

echo "[loc_fusion_tmux] creating session: $SESSION"
echo "[loc_fusion_tmux] fusion:      $FUSION_CMD"
echo "[loc_fusion_tmux] echo_local:  $LOCAL_ODOM_TOPIC"
echo "[loc_fusion_tmux] echo_global: $GLOBAL_ODOM_TOPIC"
echo "[loc_fusion_tmux] echo_state:  $ODOM_STATE_TOPIC"

tmux new-session -d -s "$SESSION" -n "loc_test"

pane_fusion="$(tmux display-message -p -t "$SESSION":0.0 "#{pane_id}")"
pane_local="$(tmux split-window -h -t "$pane_fusion" -p 75 -P -F "#{pane_id}")"
pane_global="$(tmux split-window -h -t "$pane_local" -p 66 -P -F "#{pane_id}")"
pane_state="$(tmux split-window -h -t "$pane_global" -p 50 -P -F "#{pane_id}")"

tmux set-option -t "$SESSION" -g pane-border-status top

PANES=("$pane_fusion" "$pane_local" "$pane_global" "$pane_state")
TITLES=("localization_fusion" "echo local_odom" "echo global_odom" "echo odom_status")
LINES=(
  "$(build_pane_line fusion "$FUSION_CMD")"
  "$(build_pane_line local_odom "$LOCAL_ECHO_CMD")"
  "$(build_pane_line global_odom "$GLOBAL_ECHO_CMD")"
  "$(build_pane_line odom_status "$STATE_ECHO_CMD")"
)

for i in "${!PANES[@]}"; do
  tmux select-pane -t "${PANES[$i]}" -T "${TITLES[$i]}"
done

tmux send-keys -t "$pane_fusion" "${LINES[0]}" C-m
sleep 1.0
tmux send-keys -t "$pane_local" "${LINES[1]}" C-m
tmux send-keys -t "$pane_global" "${LINES[2]}" C-m
tmux send-keys -t "$pane_state" "${LINES[3]}" C-m

tmux select-pane -t "$pane_fusion"

echo "[loc_fusion_tmux] started. Press Ctrl+C in any pane to stop all."
echo "[loc_fusion_tmux] stop manually: $0 stop --session $SESSION"

if [[ "$AUTO_ATTACH" -eq 1 ]]; then
  exec tmux attach -t "$SESSION"
fi
