#!/bin/bash
# 启动explore自主建图
gnome-terminal \
--tab -e "zsh -c 'source $HOME/.zshrc;sudo systemctl stop start_app_node;killall -9 rosmaster;sleep 10;roslaunch slam slam.launch slam_methods:=explore robot_name:=/ master_name:=/ '" \
--tab -e "zsh -c 'source $HOME/.zshrc;sleep 20;rviz rviz -d  $HOME/ros_ws/src/slam/rviz/explore_desktop.rviz'" 

