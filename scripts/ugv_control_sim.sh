#!/bin/bash

gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 3.0; roslaunch sunray_simulator sunray_sim_1ugv.launch gui:=true; exec bash"' \
--tab -e 'bash -c "sleep 5.0; roslaunch localization_fusion localiztion_ugv.launch ugv_name:=ugv ugv_id:=1 source_id:=3 node_name:=localization_fusion_ugv1; exec bash"' \
--tab -e 'bash -c "sleep 7.0; roslaunch sunray_ugv_control ugv_control.launch ugv_id:=ugv1 localization_ns:=ugv1; exec bash"' \
