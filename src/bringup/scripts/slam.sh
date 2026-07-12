#!/bin/bash
# 启动gmapping建图

gnome-terminal \
--tab -e "zsh -c 'source $HOME/.zshrc;sudo systemctl stop start_app_node;killall -9 rosmaster;sleep 10;roslaunch slam slam.launch robot_name:=/ master_name:=/'" \
--tab -e "zsh -c 'source $HOME/.zshrc;sleep 20;roscd slam/rviz;rviz rviz -d gmapping_desktop.rviz'" \
--tab -e "zsh -c 'source $HOME/.zshrc;sleep 20;roslaunch peripherals teleop_key_control.launch robot_name:=/'" \
--tab -e "zsh -c 'source $HOME/.zshrc;sleep 20;rosrun slam map_save.py'"
