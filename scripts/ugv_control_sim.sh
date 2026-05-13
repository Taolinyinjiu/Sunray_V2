#!/bin/bash

gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 3.0; roslaunch sunray_simulator sunray_sim_1ugv.launch gui:=true; exec bash"' \
--tab -e 'bash -c "sleep 5.0; roslaunch localization_fusion localization_fusion.launch agent_name:=ugv agent_id:=1 source_id:=5; exec bash"' \
--tab -e 'bash -c "sleep 7.0; roslaunch sunray_ugv_control ugv_control.launch agent_name:=ugv agent_id:=1; exec bash"' \
