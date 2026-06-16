#!/bin/bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SUNRAY_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

cd "$SUNRAY_ROOT" || exit 1
source /opt/ros/noetic/setup.bash
source devel/setup.bash

gnome-terminal \
--window --title="sunray_mavros" --working-directory="$SUNRAY_ROOT" -e 'bash -c "source /opt/ros/noetic/setup.bash; source devel/setup.bash; roslaunch sunray_mavros mavros.launch; exec bash"' \
--window --title="joystick_ros_bridge" --working-directory="$SUNRAY_ROOT" -e 'bash -c "sleep 3.0; source /opt/ros/noetic/setup.bash; source devel/setup.bash; roslaunch joystick_ros_bridge joystick_ros_bridge.launch; exec bash"'
