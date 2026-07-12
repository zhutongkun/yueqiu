#!/usr/bin/env python3
# encoding: utf-8
# 语音控制导航
import os
import json
import rospy
import signal
import math
import subprocess
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger, TriggerResponse
from xf_mic_asr_offline import voice_play
from geometry_msgs.msg import Twist, PoseStamped, Pose
from move_base_msgs.msg import MoveBaseActionResult
from ros_robot_controller.msg import BuzzerState
from servo_controllers import bus_servo_control
from servo_msgs.msg import MultiRawIdPosDur


def rpy2qua(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = Pose()
    q.orientation.w = cy * cp * cr + sy * sp * sr
    q.orientation.x = cy * cp * sr - sy * sp * cr
    q.orientation.y = sy * cp * sr + cy * sp * cr
    q.orientation.z = sy * cp * cr - cy * sp * sr
    return q.orientation


class VoiceControlNavNode:
    def __init__(self, name):
        rospy.init_node(name)

        self.words = None
        self.running = True
        self.move_base_status = 1
        self.pick_location_time = rospy.get_param('/pick_location_time', 3)
        self.up_ramp_time = rospy.get_param('/up_ramp_time', 3.5)
        self.scene_card_results = []
        self.scene_card_timeout = float(rospy.get_param('~scene_card_timeout', 10.0))
        self.scene_card_retry = int(rospy.get_param('~scene_card_retry', 1))
        self.scene_card_settle_time = float(rospy.get_param('~scene_card_settle_time', 0.8))
        self.scene_card_stop_after_task = rospy.get_param('~scene_card_stop_after_task', True)
        self.scene_card_process = None
        self.auto_start_on_voice_timeout = rospy.get_param('~auto_start_on_voice_timeout', True)
        self.voice_init_timeout = float(rospy.get_param('~voice_init_timeout', 20.0))
        self.voice_command_timeout = float(rospy.get_param('~voice_command_timeout', 15.0))
        self.auto_start_used = False
        self.wakeup_detected = False

        rospy.Service('~pick', Trigger, self.start_pick_callback)
        rospy.Service('~place', Trigger, self.start_place_callback)
        rospy.Service('~detect', Trigger, self.start_detect_callback)
        rospy.Service('~scene_card', Trigger, self.start_scene_card_callback)
        rospy.Service('~back', Trigger, self.start_back_callback)
        rospy.Service('~test', Trigger, self.test_callback)
        rospy.Service('~aligning', Trigger, self.start_aligning_callback)

        rospy.set_param('~target_shape', 'None')
        rospy.set_param('~status', 'start')

        self.language = os.environ['ASR_LANGUAGE']
        self.costmap = '/move_base/local_costmap/costmap'
        self.map_frame = rospy.get_param('~map_frame', '/map')
        self.slope_surface = rospy.get_param('~slope_surface', True)
        self.mecanum_pub = rospy.Publisher('/controller/cmd_vel', Twist, queue_size=1)
        self.joints_pub = rospy.Publisher('/servo_controllers/port_id_1/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)

        while not rospy.is_shutdown():
            try:
                if rospy.get_param('/servo_manager/init_finish') and rospy.get_param(
                        '/joint_states_publisher/init_finish'):
                    break
            except:
                rospy.sleep(0.1)

        bus_servo_control.set_servos(self.joints_pub, 2,
                                     ((1, 500), (2, 760), (3, 15), (4, 150), (5, 500), (10, 200)))
        rospy.sleep(2)

        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        rospy.Subscriber('/move_base/result', MoveBaseActionResult, self.move_callback)

        voice_wait_start = rospy.Time.now()
        while not rospy.is_shutdown():
            try:
                if rospy.get_param('/voice_control/init_finish'):
                    break
            except:
                pass
            if self.auto_start_on_voice_timeout and (rospy.Time.now() - voice_wait_start).to_sec() >= self.voice_init_timeout:
                rospy.logwarn("Voice control init timeout, continue and auto-start later.")
                break
            rospy.sleep(0.1)

        self.vc_sub = rospy.Subscriber('/asr_node/voice_words', String, self.words_callback)
        rospy.loginfo('唤醒口令: 小迈小迈')
        rospy.loginfo('唤醒后15秒内可以不用再唤醒')
        rospy.loginfo('控制指令: 开始执行任务 / 开始安全任务')

        rospy.wait_for_message(self.costmap, OccupancyGrid)

        self.left_rear_dist = 2.0
        self.right_rear_dist = 2.0
        self.current_pose = Pose()
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)

        self.play('running')
        signal.signal(signal.SIGINT, self.shutdown)
        self.run()

    def scan_callback(self, msg):
        try:
            left_ranges = []
            for i, angle in enumerate(msg.ranges):
                deg = math.degrees(msg.angle_min + i * msg.angle_increment)
                if 150 <= deg <= 180 and msg.ranges[i] > 0.1:
                    left_ranges.append(msg.ranges[i])
            self.left_rear_dist = min(left_ranges) if left_ranges else 2.0

            right_ranges = []
            for i, angle in enumerate(msg.ranges):
                deg = math.degrees(msg.angle_min + i * msg.angle_increment)
                if deg < 0:
                    deg += 360
                if 180 <= deg <= 210 and msg.ranges[i] > 0.1:
                    right_ranges.append(msg.ranges[i])
            self.right_rear_dist = min(right_ranges) if right_ranges else 2.0
        except Exception as e:
            rospy.logwarn_throttle(5, "scan_callback error: %s", e)

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    def reverse_up_ramp_with_laser(self, distance=1.3, speed=0.25, Kp=0.6):
        rospy.loginfo("Starting reverse up ramp with laser correction, distance=%.2f m", distance)
        try:
            self.goal_pub.publish(PoseStamped())
            rospy.loginfo("Cancelled move_base goal")
        except Exception as e:
            rospy.logwarn("Failed to cancel move_base goal: %s", e)
        start_pose = self.current_pose
        traveled = 0.0
        rate = rospy.Rate(20)
        twist = Twist()
        twist.linear.x = -speed
        while not rospy.is_shutdown() and traveled < distance:
            dx = self.current_pose.position.x - start_pose.position.x
            dy = self.current_pose.position.y - start_pose.position.y
            traveled = math.sqrt(dx*dx + dy*dy)
            error = self.left_rear_dist - self.right_rear_dist
            twist.angular.z = max(-0.5, min(0.5, Kp * error))
            self.mecanum_pub.publish(twist)
            rate.sleep()
        self.mecanum_pub.publish(Twist())
        rospy.loginfo("Reverse up ramp finished, traveled %.3f m", traveled)

    def test_callback(self, msg):
        self.words = '开始安全任务'
        return TriggerResponse(success=True)

    def start_pick_callback(self, msg):
        rospy.loginfo("start_pick_callback called")
        self.safe_pick()
        return TriggerResponse(success=True)

    def start_place_callback(self, msg):
        self.control(0, 0, 0, "place")
        return TriggerResponse(success=True)

    def start_detect_callback(self, msg):
        self.control(0, 0, 0, "detect")
        return TriggerResponse(success=True)

    def start_scene_card_callback(self, msg):
        ok = self.run_scene_card_task(report=True)
        return TriggerResponse(success=ok)

    def start_aligning_callback(self, msg):
        rospy.loginfo("start_aligning_callback called")
        try:
            rospy.wait_for_service('/position_correction/start', timeout=3.0)
            rospy.wait_for_service('/position_correction/pick_1', timeout=3.0)
            rospy.wait_for_service('/position_correction/pick_2', timeout=3.0)
        except rospy.ROSException as e:
            rospy.logerr("Position correction services not available: %s", e)
            return TriggerResponse(success=False, message=str(e))

        try:
            rospy.ServiceProxy('/position_correction/start', Trigger)()
            rospy.ServiceProxy('/position_correction/pick_1', Trigger)()
            rospy.sleep(1)
            self.wait_correction_status()
            rospy.ServiceProxy('/position_correction/pick_2', Trigger)()
            rospy.sleep(1)
            self.wait_correction_status()
        except Exception as e:
            rospy.logerr("Aligning failed: %s", e)
            return TriggerResponse(success=False, message=str(e))

        twist = Twist()
        twist.linear.x = 0.05
        self.mecanum_pub.publish(twist)
        rospy.sleep(self.pick_location_time)
        self.mecanum_pub.publish(Twist())
        return TriggerResponse(success=True)

    def start_back_callback(self, msg):
        rospy.loginfo("start_back_callback called")
        self.control(0.1, -0.3, 0, "back")
        return TriggerResponse(success=True)

    def play(self, name):
        try:
            voice_play.play(name, language=self.language)
        except Exception as e:
            rospy.logwarn('语音播放失败: %s, %s', name, e)

    def call_trigger(self, service_name, timeout=1.5):
        try:
            rospy.wait_for_service(service_name, timeout=timeout)
            return rospy.ServiceProxy(service_name, Trigger)()
        except Exception as e:
            rospy.logwarn('服务调用跳过: %s, %s', service_name, e)
            return None

    def ensure_scene_card_node(self):
        try:
            rospy.wait_for_service('/yolov5_scene_card/start', timeout=0.5)
            return True
        except Exception:
            pass
        rospy.loginfo('启动月球环境识别节点 yolov5_scene_card')
        if self.scene_card_process is None or self.scene_card_process.poll() is not None:
            self.scene_card_process = subprocess.Popen(
                ['rosrun', 'competition', 'yolov5_scene_card_node.py']
            )
        try:
            rospy.wait_for_service('/yolov5_scene_card/start', timeout=70.0)
            return True
        except Exception as e:
            rospy.logerr('月球环境识别节点启动失败: %s', e)
            return False

    def stop_scene_card_node(self):
        self.call_trigger('/yolov5_scene_card/stop', timeout=1.0)
        if self.scene_card_process is not None and self.scene_card_process.poll() is None:
            try:
                self.scene_card_process.terminate()
                self.scene_card_process.wait(timeout=3.0)
            except Exception:
                try:
                    self.scene_card_process.kill()
                except Exception:
                    pass

    def shutdown(self, signum, frame):
        self.running = False
        rospy.loginfo('shutdown')
        rospy.signal_shutdown('shutdown')

    def words_callback(self, msg):
        self.words = json.dumps(msg.data, ensure_ascii=False)[1:-1]
        if self.language == 'Chinese':
            self.words = self.words.replace(' ', '')
        print('words:', self.words)

        if self.words == '唤醒成功(wake-up-success)':
            self.wakeup_detected = True
            self.play('awake')
        elif self.words == '休眠(Sleep)':
            buzzer = BuzzerState()
            buzzer.freq = 1900
            buzzer.on_time = 0.05
            buzzer.off_time = 0.01
            buzzer.repeat = 1
            self.buzzer_pub.publish(buzzer)

    def move_callback(self, msg):
        try:
            if msg.status.status == 3:
                self.move_base_status = 3
            elif msg.status.status > 3:
                self.move_base_status = -1
            else:
                self.move_base_status = 1
        except:
            self.move_base_status = 1

    def nav_position(self, x, y, w):
        pose = PoseStamped()
        pose.header.frame_id = self.map_frame
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation = rpy2qua(0, 0, math.radians(w))
        self.goal_pub.publish(pose)

    def wait_nav_status(self, timeout=30.0):
        start = rospy.Time.now()
        while not rospy.is_shutdown():
            if self.move_base_status == 3:
                self.move_base_status = 1
                return True
            if self.move_base_status == -1:
                rospy.logerr("Navigation failed!")
                self.move_base_status = 1
                return False
            if (rospy.Time.now() - start).to_sec() > timeout:
                rospy.logwarn("Navigation timeout")
                self.move_base_status = 1
                return False
            rospy.sleep(1)
        return False

    def wait_correction_status(self):
        while not rospy.is_shutdown():
            if rospy.get_param('/position_correction/status', "stop") == "stop":
                break
            rospy.sleep(0.5)

    def wait_ramp_status(self):
        while not rospy.is_shutdown():
            if rospy.get_param('/ramp/status', "stop") == "stop":
                break
            rospy.sleep(0.5)

    def wait_pick_status(self, timeout=20.0):
        start = rospy.Time.now()
        rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            if rospy.get_param('/shape_recognition/status', "start") == "stop":
                rospy.set_param('/shape_recognition/status', "start")
                return True
            if (rospy.Time.now() - start).to_sec() > timeout:
                rospy.logwarn("Pick timeout after %.1f s", timeout)
                rospy.set_param('/shape_recognition/status', "start")
                return False
            rate.sleep()

    def wait_yolo_status(self, timeout=10.0):
        start = rospy.Time.now()
        while not rospy.is_shutdown():
            shape = rospy.get_param('/yolov5/shape', 'None')
            if shape != "None":
                return shape
            if (rospy.Time.now() - start).to_sec() > timeout:
                rospy.logwarn("YOLO shape detection timeout")
                return None
            rospy.sleep(0.5)

    # ---- 单点环境扫描函数（供 detect 使用） ----
    def _single_scene_card_scan(self, rotate=False, reverse_rotate=False, restart_yolo=True):
        self.call_trigger('/yolov5/stop', timeout=1.0)
        if not self.ensure_scene_card_node():
            if restart_yolo:
                self._restart_yolo()
            return 'unknown'
        self.call_trigger('/yolov5_scene_card/start', timeout=3.0)

        if rotate:
            twist = Twist()
            twist.angular.z = -0.5
            self.mecanum_pub.publish(twist)
            rospy.sleep(math.pi)
            self.mecanum_pub.publish(Twist())

        result = self._detect_scene_card_once(timeout=self.scene_card_timeout)

        if rotate and reverse_rotate:
            twist = Twist()
            twist.angular.z = 0.5
            self.mecanum_pub.publish(twist)
            rospy.sleep(math.pi)
            self.mecanum_pub.publish(Twist())

        self.stop_scene_card_node()
        if restart_yolo:
            self._restart_yolo()
        return result

    def _detect_scene_card_once(self, timeout=10.0):
        rospy.set_param('/yolov5_scene_card/shape', 'None')
        rospy.sleep(self.scene_card_settle_time)
        start = rospy.Time.now()
        while not rospy.is_shutdown() and self.running:
            raw = rospy.get_param('/yolov5_scene_card/shape', 'None')
            shape = self.normalize_scene_card(raw)
            if shape:
                rospy.loginfo('环境识别结果: %s', shape)
                return shape
            if (rospy.Time.now() - start).to_sec() > timeout:
                rospy.logwarn('环境识别超时')
                break
            rospy.sleep(0.2)
        return 'unknown'

    def normalize_scene_card(self, raw_shape):
        if raw_shape is None:
            return None
        shape = str(raw_shape).strip().lower()
        if shape in ['', 'none', 'null', 'unknown', 'no']:
            return None
        known = [
            'astronaut', 'lunar_crater', 'meteorite', 'satellite', 'lunar_rover',
            'space_station', 'rocket', 'earth', 'moon', 'lunar_soil',
        ]
        if shape in known:
            return shape
        rospy.logwarn('未知月球环境类别: %s', raw_shape)
        return shape

    # ---- 完整的三个点环境识别任务（供 scene_card 服务使用） ----
    def run_scene_card_task(self, report=False):
        default_points = [
            {'x': 1.45, 'y': -1.05, 'w': 90},
            {'x': 1.45, 'y': -1.55, 'w': 90},
            {'x': 1.45, 'y': -2.05, 'w': 90},
        ]
        points = rospy.get_param('~scene_card_points', default_points)
        self.scene_card_results = []
        self.call_trigger('/yolov5/stop', timeout=1.0)
        if not self.ensure_scene_card_node():
            self.scene_card_results = ['unknown'] * 3
            return False
        self.call_trigger('/yolov5_scene_card/start', timeout=3.0)
        try:
            for idx, pt in enumerate(points[:3], 1):
                self.nav_position(pt['x'], pt['y'], pt['w'])
                rospy.sleep(2)
                self.wait_nav_status()
                result = self._detect_scene_card_once(timeout=10.0)
                self.scene_card_results.append(result)
        finally:
            if self.scene_card_stop_after_task:
                self.stop_scene_card_node()
        if report:
            self.report_scene_card_results()
        return len(self.scene_card_results) == 3

    def report_scene_card_results(self):
        if not self.scene_card_results:
            rospy.logwarn('没有月球环境识别结果，跳过播报')
            return
        for idx, res in enumerate(self.scene_card_results, 1):
            self.play('error')
            rospy.sleep(0.25)

    def control(self, x, y, w, set_status):
        # pick2 跳过通用导航，因为其在内部自行处理导航点
        if set_status != 'pick2':
            self.nav_position(x, y, w)
            rospy.sleep(2)
            self.wait_nav_status()

        # ========== pick1 任务 ==========
        if set_status == 'pick1':
            rospy.loginfo("Navigating to pick1 point (1.30, -3.12, 0)")
            self.nav_position(1.30, -3.12, 0)
            rospy.sleep(2)
            self.wait_nav_status()

            twist = Twist()
            twist.linear.x = 0.108
            self.mecanum_pub.publish(twist)
            rospy.sleep(2)
            self.mecanum_pub.publish(Twist())

            success = self.safe_pick()
            if success:
                self.play('7')

            # 新增：第二次环境扫描（后退 → 旋转扫描并回转 → 后退）
            rospy.loginfo("pick1: 执行第二次环境识别")
            # 先以 0.05 m/s 后退 1 秒
            twist = Twist()
            twist.linear.x = -0.05
            self.mecanum_pub.publish(twist)
            rospy.sleep(1.0)
            self.mecanum_pub.publish(Twist())

            # 旋转扫描并回转（内部处理 YOLO 启停）
            self._single_scene_card_scan(rotate=True, reverse_rotate=True)
            self.play('error')

            # 以 0.1 m/s 后退 1.5 秒
            twist = Twist()
            twist.linear.x = -0.1
            self.mecanum_pub.publish(twist)
            rospy.sleep(1.5)
            self.mecanum_pub.publish(Twist())

        # ========== pick2 任务 ==========
        elif set_status == 'pick2':
            rospy.loginfo("Navigating to pick2 point (0.94, -3.17)")

            target_yaw = -180.0
            yaw_tolerance = 10.0

            try:
                q = self.current_pose.orientation
                siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
                cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
                yaw_rad = math.atan2(siny_cosp, cosy_cosp)
                current_yaw = math.degrees(yaw_rad)
                diff = abs(current_yaw - target_yaw)
                diff = min(diff, 360.0 - diff)

                if diff <= yaw_tolerance:
                    rospy.loginfo("Current yaw %.2f is within tolerance (%.1f°), using it to avoid rotation.",
                                  current_yaw, diff)
                    nav_yaw = current_yaw
                else:
                    rospy.loginfo("Current yaw %.2f differs from target %.2f by %.1f°, will correct.",
                                  current_yaw, target_yaw, diff)
                    nav_yaw = target_yaw
            except Exception as e:
                rospy.logwarn("Failed to get current yaw, using target yaw as fallback: %s", e)
                nav_yaw = target_yaw

            self.nav_position(0.94, -3.17, nav_yaw)

            rospy.sleep(2)
            self.wait_nav_status()

            twist = Twist()
            twist.linear.x = 0.24
            self.mecanum_pub.publish(twist)
            rospy.sleep(2)
            self.mecanum_pub.publish(Twist())

            success = self.safe_pick()
            if success:
                self.play('7')

            # 新增：第三次环境扫描（原地，不旋转）
            rospy.loginfo("pick2: 执行第三次环境识别 (原地)")
            self._single_scene_card_scan(rotate=False, reverse_rotate=False)
            self.play('error')

            # 保留原有后退
            twist = Twist()
            twist.linear.x = -0.2
            self.mecanum_pub.publish(twist)
            rospy.sleep(3)
            self.mecanum_pub.publish(Twist())

        elif set_status == "place":
            twist = Twist()
            twist.linear.x = 0.2
            self.mecanum_pub.publish(twist)
            rospy.sleep(1.7)
            self.mecanum_pub.publish(Twist())
            try:
                rospy.ServiceProxy('/position_correction/place_3', Trigger)()
            except Exception as e:
                rospy.logwarn("Place correction failed: %s", e)
            rospy.sleep(1)
            self.wait_correction_status()
            self.play('9')
            twist = Twist()
            twist.linear.x = -0.2
            self.mecanum_pub.publish(twist)
            rospy.sleep(2)
            self.mecanum_pub.publish(Twist())

        elif set_status == "detect":
            if self.slope_surface is not True:
                self.play('reached_explosion-proof_warehouse')
            else:
                self.play('2')

            rospy.sleep(1)
            shape = self.wait_yolo_status(timeout=10.0)
            if shape is None:
                shape = "cube"
                rospy.logwarn("形状检测超时，默认正方体")

            if self.slope_surface is not True:
                if shape == "cube":
                    self.play('ruled_out_cuboid_explosive')
                elif shape == "box":
                    self.play('ruled_out_cube_explosive')
                else:
                    self.play('ruled_out_cylinder_explosive')
            else:
                if shape == "cube":
                    self.play('3')
                elif shape == "box":
                    self.play('4')
                else:
                    self.play('5')

            rospy.set_param('/shape_recognition/target_shape', shape)

            # 第一次环境扫描（旋转，但不回转）
            rospy.loginfo("detect: 执行环境识别 (旋转，不回转)")
            self._single_scene_card_scan(rotate=True, reverse_rotate=False)
            self.play('error')

            rospy.sleep(2)

        elif set_status == "back":
            rospy.set_param('~status', 'stop')
            if self.slope_surface:
                rospy.ServiceProxy('/ramp/up', Trigger)()
                rospy.sleep(1)
                self.wait_ramp_status()
                twist = Twist()
                twist.linear.y = -0.1
                self.mecanum_pub.publish(twist)
                rospy.sleep(0.6)
                twist.angular.z = -0.5
                self.mecanum_pub.publish(twist)
                rospy.sleep(0.1)
                twist.linear.x = 0.3
                self.mecanum_pub.publish(twist)
                rospy.sleep(self.up_ramp_time)
                twist.angular.z = 0.5
                self.mecanum_pub.publish(twist)
                rospy.sleep(6)
                self.mecanum_pub.publish(Twist())

    # ===== 夹取执行函数（修改：返回成功状态，不播报，超时不后退） =====
    def safe_pick(self):
        rospy.loginfo("=== safe_pick start ===")
        rospy.loginfo("Stopping depth camera (YOLOv5)...")
        try:
            rospy.ServiceProxy('/yolov5/stop', Trigger)()
        except Exception as e:
            rospy.logwarn("Failed to stop YOLO: %s", e)

        try:
            rospy.wait_for_service('/shape_recognition/start', timeout=5.0)
            rospy.wait_for_service('/shape_recognition/pick', timeout=5.0)
            rospy.wait_for_service('/shape_recognition/stop', timeout=5.0)
        except rospy.ROSException as e:
            rospy.logerr("Shape recognition services not available: %s", e)
            self._restart_yolo()
            return False

        current_status = rospy.get_param('/shape_recognition/status', 'start')
        if current_status != 'start':
            rospy.set_param('/shape_recognition/status', 'start')
            rospy.sleep(0.5)

        try:
            rospy.ServiceProxy('/shape_recognition/start', Trigger)()
        except Exception as e:
            rospy.logerr("Failed to call start: %s", e)
            self._restart_yolo()
            return False

        rospy.sleep(1.5)

        try:
            rospy.ServiceProxy('/shape_recognition/pick', Trigger)()
        except Exception as e:
            rospy.logerr("Failed to call pick: %s", e)
            self._restart_yolo()
            return False

        rospy.sleep(1.0)

        success = self.wait_pick_status(timeout=20.0)

        if success:
            # 成功，不在此处播报，由调用方播报
            pass
        else:
            rospy.logerr("Pick timeout, continuing without object...")
            # 只重启 YOLO，不后退（后退由调用方统一处理）
            self._restart_yolo()
            return False

        try:
            rospy.ServiceProxy('/shape_recognition/stop', Trigger)()
        except Exception as e:
            rospy.logerr("Failed to call stop: %s", e)

        rospy.set_param('/shape_recognition/status', 'start')
        rospy.sleep(0.5)
        self._restart_yolo()
        rospy.loginfo("=== safe_pick end ===")
        return True

    def _restart_yolo(self):
        rospy.loginfo("Restarting depth camera (YOLOv5)...")
        try:
            rospy.wait_for_service('/yolov5/start', timeout=5.0)
            rospy.ServiceProxy('/yolov5/start', Trigger)()
            rospy.loginfo("Depth camera restarted successfully.")
        except Exception as e:
            rospy.logerr("Failed to restart depth camera: %s", e)

    def run(self):
        voice_command_wait_start = rospy.Time.now()
        wakeup_timeout = rospy.Duration(15.0)
        start_time = rospy.Time.now()

        while not rospy.is_shutdown() and self.running:
            if (not self.wakeup_detected and not self.auto_start_used and
                    (rospy.Time.now() - start_time) > wakeup_timeout):
                if self.slope_surface is not True:
                    self.words = "开始安全任务"
                else:
                    self.words = "开始执行任务"
                self.auto_start_used = True
                rospy.logwarn("Wake-up timeout, auto-starting task.")

            if self.words is not None:
                if self.slope_surface is not True:
                    str_data = "开始安全任务"
                else:
                    str_data = "开始执行任务"

                if self.words == str_data:
                    print('>>>>>>>>>>>>>>>>>>>> 开始任务<<<<<<<<<<<<<<<<<')
                    self.vc_sub.unregister()

                    rospy.loginfo("启动摄像头检测...")
                    for attempt in range(2):
                        try:
                            rospy.ServiceProxy('/yolov5/start', Trigger)()
                            break
                        except Exception as e:
                            if attempt == 0:
                                rospy.logwarn("yolov5 start failed, retrying...")
                                rospy.sleep(1.0)
                            else:
                                rospy.logerr("启动YOLOv5服务失败: %s", e)

                    self.play('1')
                    twist = Twist()
                    twist.linear.x = 0.3
                    self.mecanum_pub.publish(twist)
                    rospy.sleep(3)
                    self.mecanum_pub.publish(Twist())
                    twist.angular.z = -0.5
                    self.mecanum_pub.publish(twist)
                    rospy.sleep(3)
                    self.mecanum_pub.publish(Twist())
                    print("go")
                    self.move_base_status = 1

                    self.control(1.5, -0.15, 90, "detect")
                    self.control(0.94, -3.124, 0, "pick1")
                    self.control(1.24, -0.22, 40, "place")
                    self.control(0.94, -3.124, -180, "pick2")
                    self.control(1.24, -0.22, 40, "place")

                    try:
                        rospy.ServiceProxy('/position_correction/close', Trigger)()
                    except:
                        rospy.logwarn("Failed to call /position_correction/close")
                    try:
                        rospy.ServiceProxy('/shape_recognition/close', Trigger)()
                    except:
                        rospy.logwarn("Failed to call /shape_recognition/close")

                    if self.slope_surface:
                        rospy.loginfo("导航至坡前点 (1.3, 0, 0)")
                        self.nav_position(1.3, 0, 0)
                        rospy.sleep(2)
                        self.wait_nav_status()

                        # 使用激光纠偏倒车上坡
                        rospy.loginfo("开始倒车上坡（激光纠偏）...")
                        self.reverse_up_ramp_with_laser(distance=1.3, speed=0.25)
                        rospy.loginfo("已到达上坡终点 (0, 0, 0)")
                        self.play('11')
                    else:
                        self.control(0.1, -0.3, 0, "back")
                        twist.linear.y = 0.1
                        self.mecanum_pub.publish(twist)
                        rospy.sleep(3)
                        self.mecanum_pub.publish(Twist())
                        self.play('mission_accomplished')

                    rospy.loginfo("关闭摄像头检测...")
                    try:
                        rospy.ServiceProxy('/yolov5/stop', Trigger)()
                    except Exception as e:
                        rospy.logerr("停止YOLOv5服务失败: %s", e)

                    print('>>>>>>>>>>>>>>>>>>>> 结束任务<<<<<<<<<<<<<<<<<')

                elif self.words == '休眠(Sleep)':
                    rospy.sleep(0.01)
                self.words = None
            else:
                if (self.auto_start_on_voice_timeout and not self.auto_start_used and
                        (rospy.Time.now() - voice_command_wait_start).to_sec() >= self.voice_command_timeout):
                    if self.slope_surface is not True:
                        self.words = "开始安全任务"
                    else:
                        self.words = "开始执行任务"
                    self.auto_start_used = True
                    rospy.logwarn("Voice command timeout, auto-starting task.")
                    continue
                rospy.sleep(0.01)

        self.mecanum_pub.publish(Twist())


if __name__ == "__main__":
    VoiceControlNavNode('voice_control_nav')
