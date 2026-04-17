#!/bin/bash

set -euo pipefail

SESSION_NAME="${SESSION_NAME:-uav_formation}"
UAV_NUM="${1:-${UAV_NUM:-6}}"
UAV_NAME="${UAV_NAME:-uav}"
SOURCE_ID="${SOURCE_ID:-3}"
HEALTH_RATE_HZ="${HEALTH_RATE_HZ:-10.0}"
USE_RECEIVE_TIME="${USE_RECEIVE_TIME:-true}"
WORKSPACE_ROOT="/home/yyf/Sunray_v2"

if ! command -v tmux >/dev/null 2>&1; then
  echo "tmux not found"
  exit 1
fi

if [ "${UAV_NUM}" -lt 1 ] || [ "${UAV_NUM}" -gt 6 ]; then
  echo "UAV_NUM must be in [1, 6]"
  exit 1
fi

attach_or_switch() {
  if [ -n "${TMUX:-}" ]; then
    tmux switch-client -t "${SESSION_NAME}"
  else
    exec tmux attach-session -t "${SESSION_NAME}"
  fi
}

send_window_command() {
  local window_name="$1"
  local command="$2"

  tmux send-keys -t "${SESSION_NAME}:${window_name}.0" "${command}" C-m
}

if tmux has-session -t "${SESSION_NAME}" 2>/dev/null; then
  echo "tmux session '${SESSION_NAME}' already exists"
  attach_or_switch
fi

tmux new-session -d -s "${SESSION_NAME}" -n roscore -c "${WORKSPACE_ROOT}"
send_window_command roscore "cd ${WORKSPACE_ROOT} && roscore"

tmux new-window -d -t "${SESSION_NAME}:" -n simulator -c "${WORKSPACE_ROOT}"
send_window_command simulator "cd ${WORKSPACE_ROOT} && sleep 3 && roslaunch sunray_simulator sunray_sim_6uav.launch"

tmux new-window -d -t "${SESSION_NAME}:" -n control -c "${WORKSPACE_ROOT}"
send_window_command control "cd ${WORKSPACE_ROOT} && sleep 8 && roslaunch sunray_uav_control uav_control_swarm.launch uav_num:=${UAV_NUM} uav_name:=${UAV_NAME}"

tmux new-window -d -t "${SESSION_NAME}:" -n localization -c "${WORKSPACE_ROOT}"
send_window_command localization "cd ${WORKSPACE_ROOT} && sleep 10 && roslaunch localization_fusion localization_fusion_swarm.launch uav_num:=${UAV_NUM} uav_name:=${UAV_NAME} source_id:=${SOURCE_ID} health_rate_hz:=${HEALTH_RATE_HZ} use_receive_time:=${USE_RECEIVE_TIME}"

tmux new-window -d -t "${SESSION_NAME}:" -n swarm -c "${WORKSPACE_ROOT}"
send_window_command swarm "cd ${WORKSPACE_ROOT} && sleep 12 && roslaunch sunray_swarm swarm_sim.launch agent_num:=${UAV_NUM} agent_name:=${UAV_NAME}"

tmux new-window -d -t "${SESSION_NAME}:" -n tui -c "${WORKSPACE_ROOT}"
send_window_command tui "cd ${WORKSPACE_ROOT} && sleep 14 && roslaunch sunray_swarm formation_tui.launch"

tmux select-window -t "${SESSION_NAME}:tui"
attach_or_switch
