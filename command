# data:2023/12/05 by aiden
# 本文档只包含启动需要的指令，部分指令还需要配合其他设置才能生效
# 请结合教程文档使用. 特别说明: 每行指令需要单独开一个终端运行，
# 且有先后之分

# bringup app
#关闭app自启功能 
sudo systemctl disable start_app_node.service 

#停止app功能 
sudo systemctl stop start_app_node.service

#开启app自启功能 
sudo systemctl enable start_app_node.service

#开启app功能 
sudo systemctl start start_app_node.service

#重启app功能
sudo systemctl restart start_app_node.service

#查看app后台自启状态 
sudo systemctl status start_app_node.service

# calibration
#深度摄像头红外标定 
roslaunch calibration depth_cam_ir_calibration.launch

#深度摄像头RGB标定 
roslaunch calibration depth_cam_rgb_calibration.launch

#角速度校准 
roslaunch calibration calibrate_angular.launch angular:=true

#线速度校准 
roslaunch calibration calibrate_linear.launch linear:=true

#imu校准 
roslaunch calibration calibrate_imu.launch

# example
#apriltag检测 
roslaunch example apriltag_detect_demo.launch
 
#ar检测 
roslaunch example ar_detect_demo.launch

#深度摄像头红外可视化 
roslaunch example depth_cam_ir_view.launch

#深度摄像头点云可视化 
roslaunch example depth_cam_point_cloud_view.launch

#深度摄像头RGB图像可视化 
roslaunch example depth_cam_rgb_view.launch

#imu可视化 
roslaunch peripherals imu.launch debug:=true

#雷达可视化
roslaunch peripherals lidar_view.launch

#肢体姿态融合RGB控制 
roslaunch example body_and_rgb_control.launch

#肢体姿态控制 
roslaunch example body_control.launch

#人体跟踪 
roslaunch example body_track.launch    

#跌倒检测 
roslaunch example fall_down_detect.launch

#颜色识别 
roscd example/scripts/color_detect && python3 color_detect_demo.py    

#颜色追踪 
roslaunch example color_track_node.launch

#指尖轨迹 
roslaunch example hand_trajectory_node.launch    

#手部跟随 
roslaunch example hand_track_node.launch      

#二维码生成 
roscd example/scripts/qrcode && python3 qrcode_creater.py

#二维码检测 
roscd example/scripts/qrcode && python3 qrcode_detecter.py        

#物体跟踪 
roslaunch example object_tracking.launch

#颜色分拣
roslaunch example color_sorting_node.launch 
#roslaunch example color_sorting_node.launch debug:=true

#垃圾分类
roslaunch example garbage_classification.launch   
#roslaunch example garbage_classification.launch debug:=true

#循线清障
roslaunch example line_follow_clean_node.launch  
#roslaunch example line_follow_clean_node.launch debug:=true

#颜色夹取 
roslaunch example automatic_pick.launch
#roslaunch example automatic_pick.launch debug:=true
#开启夹取
rosservice call /automatic_pick/pick "{}"
#开启放置
rosservice call /automatic_pick/place "{}"

#导航搬运 
roslaunch example navigation_transport.launch map:=xxx

#人脸检测 
roscd example/scripts/mediapipe_example && python3 face_detect.py

#人脸网格 
roscd example/scripts/mediapipe_example && python3 face_mesh.py

#手关键点检测 
roscd example/scripts/mediapipe_example && python3 hand.py

#肢体关键点检测 
roscd example/scripts/mediapipe_example && python3 pose.py

#背景分割 
roscd example/scripts/mediapipe_example && python3 self_segmentation.py    

#整体检测 
roscd example/scripts/mediapipe_example  && python3 holistic.py   
#使用完毕后需要退出虚拟环境，输入指令deactivate

#3D物体检测 
roscd example/scripts/mediapipe_example && python3 objectron.py  
#使用完毕后需要退出虚拟环境，输入指令deactivate

#无人驾驶
roslaunch example self_driving.launch  

# slam
#建图 
roslaunch slam slam.launch

#rviz查看建图效果
roslaunch slam rviz_slam.launch

#键盘控制(可选)
roslaunch peripherals teleop_key_control.launch

#保存地图 
roscd slam/maps && rosrun map_server map_saver map:=/robot_1/map -f 保存名称

#app建图
roslaunch slam slam.launch app:=true
roscd slam/maps && rosrun map_server map_saver map:=/map -f 保存名称

# navigation
#导航
roslaunch navigation navigation.launch map:=地图名称

#rviz发布导航目标
roslaunch navigation rviz_navigation.launch

#多点导航
roslaunch navigation publish_point.launch

#app导航
roslaunch navigation navigation.launch map:=地图名称 app:=true

#3D建图
roslaunch slam slam.launch slam_methods:=rtabmap   
#rviz查看建图效果
roslaunch slam rviz_slam.launch slam_methods:=rtabmap  

#3D导航
roslaunch navigation rtabmap_navigation.launch   
#rviz发布导航目标
roslaunch navigation rviz_rtabmap_navigation.launch  

# simulations
#urdf可视化 
roslaunch roslander_description display.launch

#gazebo可视化 
roslaunch roslander_gazebo worlds.launch

#moveit
#仅仿真 
roslaunch roslander_moveit_config demo.launch

#和真实联动
roslaunch roslander_moveit_config demo.launch fake_execution:=false

#和gazebo联动
roslaunch roslander_moveit_config demo_gazebo.launch

#仿真建图 
#gazebo仿真
roslaunch roslander_gazebo room_worlds.launch  
roslaunch slam slam.launch sim:=true
roslaunch slam rviz_slam.launch sim:=true

#仿真导航 
roslaunch roslander_gazebo room_worlds.launch
roslaunch navigation navigation.launch sim:=true map:=地图名称
roslaunch navigation rviz_navigation.launch sim:=true

#xf_mic_asr_offline
#语音控制移动 
roslaunch xf_mic_asr_offline voice_control_move.launch

#语音控制导航 
roslaunch xf_mic_asr_offline voice_control_navigation.launch map:=地图名称

#语音控制颜色检测 
roslaunch xf_mic_asr_offline voice_control_color_detect.launch

#语音控制颜色跟踪 
roslaunch xf_mic_asr_offline voice_control_color_track.launch   

#语音控制颜色分拣
roslaunch xf_mic_asr_offline voice_control_color_sorting.launch   

#语音控制垃圾分类
roslaunch xf_mic_asr_offline voice_control_garbage_classification.launch  ##

#图像采集软件
roslaunch peripherals depth_cam.launch
python3 ~/software/collect_picture/main.py

#lab软件
roslaunch peripherals depth_cam.launch
python3 ~/software/lab_tool/main.py

#舵机调试软件
python3 ~/software/servo_tool/main.py

#图像标注软件
python3 ~/software/labelImg/labelImg.py

#手势控制
roslaunch example hand_gesture_control_node.launch    

#过独木桥
roslaunch example cross_bridge.launch

#防跌落
roslaunch example prevent_falling.launch

#物体跟踪
roslaunch example track_object.launch 

#跟踪夹取
roslaunch example track_and_grab.launch

#三维夹取
roslaunch example object_classification.launch

#语音控制跟踪夹取
roslaunch example vc_track_and_grab.launch

#语音控制三维夹取
roslaunch example vc_object_classification.launch
