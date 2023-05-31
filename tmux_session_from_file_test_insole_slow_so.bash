#!/usr/bin/env bash

tmux new-session -s mysession -d 
tmux set-option -s -t mysession default-command "bash --rcfile ~/.bashrc_ws.sh"
tmux send -t mysession:0.0 "roscore" C-m

tmux new-window
tmux select-window -t 1
tmux split-window -h 
tmux split-window -h 
tmux split-window -h 
tmux select-layout even-horizontal
tmux select-pane -t 3
tmux split-window -v -p 50 
tmux select-pane -t 2
tmux split-window -v -p 50 
tmux select-pane -t 1 
tmux split-window -v -p 50 
tmux select-pane -t 0
tmux split-window -v -p 50 
#tmux select-layout tiled
#tmux select-pane -t 0
tmux new-window
tmux select-window -t 2
tmux split-window -h
tmux new-window
tmux select-window -t 3
tmux split-window -h
tmux new-window
sleep 2
tmux send -t mysession:4.0 "rosparam set /use_sim_time true" C-m
tmux select-window -t 1

#sends keys to first and second terminals
tmux send -t mysession:1.0 "sleep 2; rosservice call /ik/outlabels" C-m
tmux send -t mysession:1.0 "rosrun osrt_ros graph_tau_id.bash" C-m
#tmux send -t mysession:1.0 "rostopic echo /id_node/output" C-m
#tmux send -t mysession:1.1 "rostopic echo /model_generic/joint_states" C-m
tmux send -t mysession:1.1 "rosrun osrt_ros graph_grfs.bash" C-m

tmux send -t mysession:1.2 "roslaunch osrt_ros id_combined_filtered.launch get_second_label:=false left_foot_tf_name:=left right_foot_tf_name:=right" C-m
tmux send -t mysession:1.3 "roslaunch moticon_insoles feet_wrench_and_ik_from_file.launch filename:=/catkin_ws/Data/ruoli/ViconData/Ruoli/Moticon_insole/RealTimekIDS2/walking01_header_corrected.txt" C-m
tmux send -t mysession:1.4 "roslaunch moticon_insoles play_ik.launch filename:=/catkin_ws/Data/ruoli/ViconData/Ruoli/Moticon_insole/RealTimekIDS2/2023-03-03-11-56-24walking012_ik_lower.sto" C-m
#tmux send -t mysession:1.5 "rosrun moticon_insoles service_mux.py" C-m
tmux send -t mysession:4.0 "roslaunch custom_clock custom_clock.launch slowdown_rate:=3" C-m
tmux send -t mysession:1.5 "rostopic echo /clock" C-m
tmux send -t mysession:1.6 "sleep 4; rosservice call /moticon_insoles/start_playback" C-m
tmux send -t mysession:1.7 "sleep 3; rosservice call /inverse_kinematics_from_file/start" C-m
#tmux send -t mysession:1.7 "sleep 3; rosservice call /start_multi/trigger" C-m
tmux send -t mysession:2.0 "rosbag record /id_node/output /left/force /right/force " C-m
#tmux send -t mysession:2.0 "rosrun osrt_ros graph_iks_old.bash" C-m
tmux send -t mysession:3.0 "sleep 2.9; rosservice call /id_node/set_name_and_path \"{name: 's2_id_walking', path: '/tmp/s2' }\" " C-m

#tmux send -t mysession:1.7 "ls -la" C-m
#tmux setw synchronize-panes on

tmux -2 a -t mysession


