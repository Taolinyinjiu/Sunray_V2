#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "$SCRIPT_DIR/../../.." && pwd)"

usage() {
  cat <<'USAGE_EOF'
Run localization_fusion + topic monitors in one tmux session.

Layout (single window, 4 panes):
  left       -> localization_fusion.launch
  mid-left   -> rostopic echo local_odom
  mid-right  -> rostopic echo global_odom
  right      -> rostopic echo odom_status

Usage:
  run_localization_fusion_only_tmux.sh start [options]
  run_localization_fusion_only_tmux.sh stop [options]
  run_localization_fusion_only_tmux.sh attach [options]
  run_localization_fusion_only_tmux.sh status [options]

Options:
  --session <name>         tmux session name (default: sunray_loc_fusion_only)
  --source-id <int>        localization_fusion source_id (default: 1)
  --loop <mode>            localization_fusion loop mode (default: disable)
  --health-rate-hz <num>   localization health check rate (default: 10.0)

  --local-topic <topic>    topic for local odom echo (default: /sunray/localization/local_odom)
  --global-topic <topic>   topic for global odom echo (default: /sunray/localization/global_odom)
  --state-topic <topic>    topic for odom_status echo (default: /sunray/localization/odom_status)

  --fusion-cmd <cmd>       override fusion launch command
  --ros-setup <cmd>        setup command before run
                           (default: source /opt/ros/noetic/setup.bash && source <workspace>/devel/setup.bash if exists)
  --no-attach              start only: do not auto attach
  -h, --help               show help

Behavior:
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
  local name="$1"
  tmux has-session -t "$name" 2>/dev/null
}

send_ctrl_c_all_panes() {
  local session="$1"
  while read -r pane_id; do
    if [[ -n "$pane_id" ]]; then
      tmux send-keys -t "$pane_id" C-c || true
    fi
  done < <(tmux list-panes -t "$session":0 -F "#{pane_id}" 2>/dev/null || true)
}

build_pane_line() {
  local label="$1"
  local cmd="$2"
  echo "$ROS_SETUP && $cmd; rc=\$?; if [[ \$rc -eq 0 || \$rc -eq 130 ]]; then tmux kill-session -t $session_q >/dev/null 2>&1 || true; else echo \"[loc_fusion_tmux:$label] command exited with rc=\$rc (session kept)\"; fi"
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
SOURCE_ID=1
LOOP_MODE="disable"
HEALTH_RATE_HZ="10.0"
AUTO_ATTACH=1
ROS_SETUP=""

LOCAL_ODOM_TOPIC="/sunray/localization/local_odom"
GLOBAL_ODOM_TOPIC="/sunray/localization/global_odom"
ODOM_STATE_TOPIC="/sunray/localization/odom_status"

FUSION_CMD_OVERRIDE=""

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
    --loop)
      [[ $# -ge 2 ]] || { err "--loop requires a value"; exit 2; }
      LOOP_MODE="$2"
      shift 2
      ;;
    --health-rate-hz)
      [[ $# -ge 2 ]] || { err "--health-rate-hz requires a value"; exit 2; }
      HEALTH_RATE_HZ="$2"
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
  else
    echo "[loc_fusion_tmux] session '$SESSION' is not running"
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
  echo "[loc_fusion_tmux] stopping session: $SESSION"
  send_ctrl_c_all_panes "$SESSION"
  sleep 0.3
  tmux kill-session -t "$SESSION" || true
  echo "[loc_fusion_tmux] session killed: $SESSION"
  exit 0
fi

if tmux_has_session "$SESSION"; then
  err "session '$SESSION' already exists"
  echo "[loc_fusion_tmux] use: $0 attach --session $SESSION"
  exit 2
fi

if [[ -z "$ROS_SETUP" ]]; then
  ROS_SETUP="source /opt/ros/noetic/setup.bash"
  if [[ -f "$WORKSPACE_ROOT/devel/setup.bash" ]]; then
    ROS_SETUP="$ROS_SETUP && source \"$WORKSPACE_ROOT/devel/setup.bash\""
  fi
fi

session_q="$(printf '%q' "$SESSION")"

WAIT_MASTER_CMD="until rostopic list >/dev/null 2>&1; do sleep 0.2; done"

if [[ -n "$FUSION_CMD_OVERRIDE" ]]; then
  FUSION_CMD="$FUSION_CMD_OVERRIDE"
else
  FUSION_CMD="roslaunch localization_fusion localization_fusion.launch source_id:=$SOURCE_ID loop:=$LOOP_MODE health_rate_hz:=$HEALTH_RATE_HZ"
fi

LOCAL_ECHO_CMD="$WAIT_MASTER_CMD; rostopic echo \"$LOCAL_ODOM_TOPIC\""
GLOBAL_ECHO_CMD="$WAIT_MASTER_CMD; rostopic echo \"$GLOBAL_ODOM_TOPIC\""
STATE_ECHO_CMD="$WAIT_MASTER_CMD; rostopic echo \"$ODOM_STATE_TOPIC\""

echo "[loc_fusion_tmux] creating session: $SESSION"
echo "[loc_fusion_tmux] layout: single-row(fusion|local_odom|global_odom|odom_status)"
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

tmux select-pane -t "$pane_fusion" -T "localization_fusion"
tmux select-pane -t "$pane_local" -T "echo local_odom"
tmux select-pane -t "$pane_global" -T "echo global_odom"
tmux select-pane -t "$pane_state" -T "echo odom_status"

FUSION_LINE="$(build_pane_line fusion "$FUSION_CMD")"
LOCAL_ECHO_LINE="$(build_pane_line local_odom "$LOCAL_ECHO_CMD")"
GLOBAL_ECHO_LINE="$(build_pane_line global_odom "$GLOBAL_ECHO_CMD")"
STATE_ECHO_LINE="$(build_pane_line odom_state "$STATE_ECHO_CMD")"

tmux send-keys -t "$pane_fusion" "$FUSION_LINE" C-m
sleep 1.0

tmux send-keys -t "$pane_local" "$LOCAL_ECHO_LINE" C-m
tmux send-keys -t "$pane_global" "$GLOBAL_ECHO_LINE" C-m
tmux send-keys -t "$pane_state" "$STATE_ECHO_LINE" C-m

tmux select-pane -t "$pane_fusion"

echo "[loc_fusion_tmux] started. Press Ctrl+C in any pane to stop all."
echo "[loc_fusion_tmux] stop manually: $0 stop --session $SESSION"

if [[ "$AUTO_ATTACH" -eq 1 ]]; then
  exec tmux attach -t "$SESSION"
fi
