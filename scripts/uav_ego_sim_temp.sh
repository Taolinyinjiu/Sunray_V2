#!/bin/bash
gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 5.0; roslaunch sunray_simulator sunray_sim_uav_planning.launch; exec bash"' \
--tab -e 'bash -c "sleep 5.0; roslaunch localization_fusion localization_fusion.launch; exec bash"' \
--tab -e 'bash -c "sleep 5.0; roslaunch sunray_uav_control uav_control.launch; exec bash"' \

gnome-terminal --window -e 'bash -c "sleep 3.0; roslaunch ego_planner single_run_in_gazebo.launch; exec bash"' \
