#!/bin/bash

set -e

AGENT_NUM="${1:-6}"
UAV_NAME="${UAV_NAME:-uav}"
SWARM_PKG="sunray_swarm"
CONTROL_PKG="sunray_uav_control"
CONFIG_PATH="${CONFIG_PATH:-$(rospack find ${CONTROL_PKG})/config/sunray_control_config.yaml}"

launch_term() {
  local title="$1"
  local cmd="$2"
  gnome-terminal --title="${title}" -- bash -lc "${cmd}; exec bash" &
}

if ! command -v gnome-terminal >/dev/null 2>&1; then
  echo "gnome-terminal not found"
  exit 1
fi

if [ "${AGENT_NUM}" -lt 1 ] || [ "${AGENT_NUM}" -gt 6 ]; then
  echo "agent_num must be in [1, 6]"
  exit 1
fi

if [ ! -f "${CONFIG_PATH}" ]; then
  echo "sunray_uav_control config not found: ${CONFIG_PATH}"
  exit 1
fi

# This script now only uses launch/binaries that actually exist in Sunray_v2.
# The old simulator and external fusion launch chain referenced by the previous
# version is not present in this repository anymore.

launch_term "roscore" "roscore"

for ((i = 1; i <= AGENT_NUM; ++i)); do
  delay=$((4 + i * 2))
  launch_term \
    "uav_control_${i}" \
    "sleep ${delay}; rosparam set /uav_name ${UAV_NAME}; rosparam set /uav_id ${i}; sleep 1; rosrun ${CONTROL_PKG} uav_control_node __name:=uav_control_node_${i} _config_yamlfile_path:=${CONFIG_PATH}"
done

launch_term "swarm_sim" "sleep 4; roslaunch ${SWARM_PKG} swarm_sim.launch agent_num:=${AGENT_NUM} agent_name:=${UAV_NAME}"
launch_term "formation_tui" "sleep 6; roslaunch ${SWARM_PKG} formation_tui.launch"

if [ "${ENABLE_FORMATION_SWITCH:-0}" = "1" ]; then
  launch_term "formation_switch" "sleep 6; roslaunch ${SWARM_PKG} formation_switch.launch"
fi

cat <<EOF
Started:
  roscore
  ${AGENT_NUM} x sunray_uav_control
  roslaunch ${SWARM_PKG} swarm_sim.launch agent_num:=${AGENT_NUM} agent_name:=${UAV_NAME}
  roslaunch ${SWARM_PKG} formation_tui.launch

Note:
  This script does not launch Gazebo or the old external fusion pipeline.
  If you still need a simulator/localization source, start that stack separately.
EOF
