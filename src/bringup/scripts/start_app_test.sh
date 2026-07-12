#!/bin/bash
sleep 30
#source $HOME/ros_ws/src/bringup/scripts/source_env.bash
#sudo date -s "20240420 11:18" &
#sudo killall pulseaudio 
#pulseaudio --start
#sleep 10
#roslaunch /home/ubuntu/ros_ws/src/peripherals/launch/depth_cam.launch &
#sleep 10
roslaunch /home/ubuntu/ros_ws/src/peripherals/launch/lidar.launch &
sleep 5
roslaunch /home/ubuntu/ros_ws/src/app/launch/start_app.launch & 
sleep 5
#roslaunch /home/ubuntu/ros_ws/src/controller/launch/controller.launch &
sleep 10
roslaunch /home/ubuntu/ros_ws/src/bringup/launch/bringup.launch & 
sleep 5
roslaunch /home/ubuntu/ros_ws/src/xf_mic_asr_offline/launch/startup_test.launch
