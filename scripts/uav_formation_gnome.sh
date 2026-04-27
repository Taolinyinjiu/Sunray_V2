#!/bin/bash
gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 5.0; roslaunch sunray_simulator sunray_sim_6uav.launch; exec bash"' \
--tab -e 'bash -c "sleep 5.0; roslaunch sunray_uav_control uav_control_swarm.launch uav_num:=6 uav_name:=uav; exec bash"' \
--tab -e 'bash -c "sleep 5.0; roslaunch localization_fusion localization_fusion_swarm.launch uav_num:=6 uav_name:=uav source_id:=3; exec bash"' \

gnome-terminal --window -e 'bash -c "sleep 3.0; roslaunch sunray_swarm_control swarm_sim.launch agent_num:=6; exec bash"' \
--tab -e 'bash -c "sleep 5.0; roslaunch sunray_swarm_control formation_tui.launch; exec bash"' \
