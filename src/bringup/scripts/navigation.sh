#!/bin/bash
# 启动多点导航

gnome-terminal \
--tab -e "zsh -c 'source $HOME/.zshrc;sudo systemctl stop start_app_node;killall -9 rosmaster;sleep 10;roslaunch navigation navigation.launch map:=explore robot_name:=/ master_name:=/'" \
--tab -e "zsh -c 'source $HOME/.zshrc;sleep 20;roslaunch navigation publish_point.launch enable_navigation:=false robot_name:=/ master_name:=/ '" \
--tab -e "zsh -c 'source $HOME/.zshrc;sleep 20;rviz rviz -d  $HOME/ros_ws/src/navigation/rviz/navigation_desktop.rviz'" 
