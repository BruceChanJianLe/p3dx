#!/bin/bash
# This script starts another ros and gazebo of p3dx simulation
# Please change the port for Gazebo and ROS so that they do not conflict

envs="export export GAZEBO_MASTER_URI=http://localhost:11346; export ROS_MASTER_URI=http://localhost:11312"

tmux has-session -t p3dx_simulator

if [ $? != 0 ]; then
    # create a new session
    tmux new-session -s p3dx_simulator -n p3dx_simulator -d

    # First Pane Description
    tmux send-keys -t p3dx_simulator "$envs; source $HOME/temp_ws/devel/setup.zsh; roslaunch p3dx_gazebo p3dx_empty_world.launch" C-m
    tmux select-layout -t p3dx_simulator tiled

    # Second Pane Description
    tmux split-window -h -t p3dx_simulator
    tmux send-keys -t p3dx_simulator "$env" C-m
    tmux select-layout -t p3dx_simulator tiled

fi

if [ -z "$TMUX" ]; then
  tmux attach -t p3dx_simulator
else
  tmux switch-client -t p3dx_simulator
fi
