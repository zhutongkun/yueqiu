#!/bin/bash

source $HOME/ros_ws/.typerc

export AUDIODRIVER=alsa
export OPENBLAS_CORETYPE=ARMV8

export CUDA_HOME=/usr/local/cuda
export PATH=$CUDA_HOME/bin:$PATH
export LD_LIBRARY_PATH=$CUDA_HOME/lib64:$LD_LIBRARY_PATH
export LD_PRELOAD=/usr/lib/aarch64-linux-gnu/libgomp.so.1:$LD_PRELOAD

export ROS_IP=localhost
export ROS_MASTER_URI=http://$ROS_IP:11311
export ROS_HOSTNAME=$ROS_IP

if [ $ZSH_VERSION ]; then
  . /opt/ros/noetic/setup.zsh
  . $HOME/ros_ws/devel/setup.zsh
elif [ $BASH_VERSION ]; then
  . /opt/ros/noetic/setup.bash
  . $HOME/ros_ws/devel/setup.bash
else
  . /opt/ros/noetic/setup.sh
  . $HOME/ros_ws/devel/setup.sh
fi

#sudo killall pulseaudio 
#pulseaudio --start

export DISPLAY=:0.0
exec "$@"
