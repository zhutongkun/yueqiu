#!/usr/bin/env python3
# encoding: utf-8
"""ROS1 competition controller for the ROSLander lunar mission."""

import datetime
import json
import math
import os
import signal
import subprocess
import threading
import time
import uuid

import actionlib
import rospy
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, PoseStamped, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseActionResult, MoveBaseGoal
from nav_msgs.msg import OccupancyGrid, Odometry
from ros_robot_controller.msg import BuzzerState
from sensor_msgs.msg import LaserScan
from servo_controllers import bus_servo_control
from servo_msgs.msg import MultiRawIdPosDur
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger, TriggerResponse
from xf_mic_asr_offline import voice_play

from moon_mission_core import (
    ANNOUNCING_RESULTS,
    COMPLETED,
    ERROR,
    RETURNING_TO_BASE,
    RUNNING,
    STARTING,
    STOPPED,
    WAITING_FOR_START,
    MissionLifecycle,
    atomic_write_json,
)


MOON_CLASSES = (
    'satellite',
    'space_station',
    'lunar_crater',
    'lunar_rover',
    'meteorite',
    'earth',
    'lunar_soil',
    'moon',
    'rocket',
    'astronaut',
)

MOON_CLASS_CN = {
    'satellite': '\u536b\u661f',
    'space_station': '\u7a7a\u95f4\u7ad9',
    'lunar_crater': '\u6708\u5751',
    'lunar_rover': '\u6708\u7403\u8f66',
    'meteorite': '\u9668\u77f3',
    'earth': '\u5730\u7403',
    'lunar_soil': '\u6708\u58e4',
    'moon': '\u6708\u7403',
    'rocket': '\u706b\u7bad',
    'astronaut': '\u5b87\u822a\u5458',
}

TASK_POINT_NAME = {
    1: '\u7b2c\u4e00\u4e2a\u4efb\u52a1\u70b9',
    2: '\u7b2c\u4e8c\u4e2a\u4efb\u52a1\u70b9',
    3: '\u7b2c\u4e09\u4e2a\u4efb\u52a1\u70b9',
}

START_TASK_COMMAND = '\u5f00\u59cb\u6267\u884c\u4efb\u52a1'
START_SECURITY_COMMAND = '\u5f00\u59cb\u5b89\u5168\u4efb\u52a1'
WAKEUP_SUCCESS = '\u5524\u9192\u6210\u529f(wake-up-success)'
SLEEP_COMMAND = '\u4f11\u7720(Sleep)'


class MissionAbort(RuntimeError):
    pass


def utc_timestamp():
    return datetime.datetime.utcnow().replace(microsecond=0).isoformat() + 'Z'


def rpy2qua(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    pose = Pose()
    pose.orientation.w = cy * cp * cr + sy * sp * sr
    pose.orientation.x = cy * cp * sr - sy * sp * cr
    pose.orientation.y = sy * cp * sr + cy * sp * cr
    pose.orientation.z = sy * cp * cr - cy * sp * sr
    return pose.orientation


class VoiceControlNavNode(MissionLifecycle):
    def __init__(self, name):
        rospy.init_node(name)
        MissionLifecycle.__init__(self)

        self.words = None
        self.running = True
        self.initialization_complete = False
        self.wakeup_detected = False
        self.move_base_status = 1
        self.current_pose = Pose()
        self.current_pose_received = False
        self.left_rear_dist = 2.0
        self.right_rear_dist = 2.0
        self.language = os.environ.get('ASR_LANGUAGE', 'Chinese')

        self.pick_location_time = rospy.get_param('/pick_location_time', 3)
        up_ramp_time = rospy.get_param('/up_ramp_time', 3.5)
        if isinstance(up_ramp_time, (list, tuple)):
            up_ramp_time = up_ramp_time[0] if up_ramp_time else 3.5
        self.up_ramp_time = float(up_ramp_time)
        self.map_frame = rospy.get_param('~map_frame', '/map')
        self.slope_surface = rospy.get_param('~slope_surface', True)
        self.costmap = rospy.get_param('~costmap_topic', '/move_base/local_costmap/costmap')

        self.enable_timeout_auto_start = self._mission_param(
            'enable_timeout_auto_start', True
        )
        self.enable_voice = bool(self._mission_param('enable_voice', True))
        self.start_timeout_seconds = float(
            self._mission_param('start_timeout_seconds', 15.0)
        )
        self.voice_init_timeout = float(self._mission_param('voice_init_timeout', 20.0))
        self.initialization_timeout = float(
            self._mission_param('initialization_timeout', 60.0)
        )
        self.navigation_timeout = float(
            self._mission_param('navigation_timeout', 60.0)
        )
        self.move_base_server_timeout = float(
            self._mission_param('move_base_server_timeout', 10.0)
        )
        self.moon_detection_timeout = float(
            self._mission_param('moon_detection_timeout', 18.0)
        )
        self.maximum_result_age = float(
            self._mission_param('maximum_result_age', 5.0)
        )
        self.place_service_timeout = float(
            self._mission_param('place_service_timeout', 8.0)
        )
        self.yolo_start_timeout = float(
            self._mission_param('yolo_start_timeout', 15.0)
        )
        self.yolo_unload_timeout = float(
            self._mission_param('yolo_unload_timeout', 10.0)
        )
        self.moon_start_timeout = float(
            self._mission_param('moon_start_timeout', 60.0)
        )
        self.moon_unload_timeout = float(
            self._mission_param('moon_unload_timeout', 10.0)
        )
        self.voice_volume = int(self._mission_param('voice_volume', 100))
        self.use_ramp_alignment_service = bool(
            self._mission_param('use_ramp_alignment_service', False)
        )
        self.enable_manual_debug_services = bool(
            self._mission_param('enable_manual_debug_services', True)
        )

        self.base_pose = self._pose_param(
            'base_pose', {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
        )
        self.ramp_approach_pose = self._pose_param(
            'ramp_approach_pose', {'x': 1.3, 'y': 0.0, 'yaw': 0.0}
        )
        self.ramp_return_distance = float(
            self._mission_param('ramp_return_distance', 1.3)
        )
        self.ramp_return_speed = float(
            self._mission_param('ramp_return_speed', 0.25)
        )
        self.ramp_return_timeout = float(
            self._mission_param('ramp_return_timeout', 20.0)
        )

        script_dir = os.path.dirname(os.path.realpath(__file__))
        self.package_root = os.path.abspath(os.path.join(script_dir, '..', '..'))
        workspace_root = os.path.abspath(os.path.join(self.package_root, '..', '..'))
        default_results_path = os.path.join(
            workspace_root, 'runtime', 'moon_task_results.json'
        )
        self.results_path = os.path.abspath(
            os.path.expanduser(self._mission_param('results_path', default_results_path))
        )
        self.moon_voice_dir = os.path.abspath(
            os.path.expanduser(
                self._mission_param(
                    'moon_voice_dir', os.path.join(self.package_root, 'voice', 'moon')
                )
            )
        )

        self.completed_pick_tasks = 0
        self.completed_place_tasks = 0
        self.navigation_tasks_finished = False
        self.ramp_task_finished = False
        self.other_competition_tasks_finished = False
        self.mechanical_arm_safe = False

        self._mission_thread = None
        self._start_timeout_timer = None
        self._shutdown_handled = False
        self._transition_lock = threading.Lock()
        self._gpu_lifecycle_lock = threading.Lock()
        self._debug_action_lock = threading.Lock()
        self._debug_action_active = False
        self._moon_result_lock = threading.Lock()
        self._moon_result_event = threading.Event()
        self._current_moon_request = None
        self._current_moon_result = None
        self.vc_sub = None
        self._yolo_unloaded_for_mission = False

        self.mecanum_pub = rospy.Publisher(
            '/controller/cmd_vel', Twist, queue_size=1
        )
        self.joints_pub = rospy.Publisher(
            '/servo_controllers/port_id_1/multi_id_pos_dur',
            MultiRawIdPosDur,
            queue_size=1,
        )
        self.buzzer_pub = rospy.Publisher(
            '/ros_robot_controller/set_buzzer', BuzzerState, queue_size=1
        )
        self.goal_pub = rospy.Publisher(
            '/move_base_simple/goal', PoseStamped, queue_size=1
        )

        self.move_base_action_name = rospy.get_param(
            '~move_base_action', '/move_base'
        )
        self.move_base_client = actionlib.SimpleActionClient(
            self.move_base_action_name, MoveBaseAction
        )

        rospy.Service('~pick', Trigger, self.start_pick_callback)
        rospy.Service('~place', Trigger, self.start_place_callback)
        rospy.Service('~detect', Trigger, self.start_detect_callback)
        rospy.Service('~scene_card', Trigger, self.start_scene_card_callback)
        rospy.Service('~back', Trigger, self.start_back_callback)
        rospy.Service('~test', Trigger, self.test_callback)
        rospy.Service('~aligning', Trigger, self.start_aligning_callback)
        rospy.Service('/competition/start_mission', Trigger, self.start_mission_callback)
        rospy.Service('/competition/stop_mission', Trigger, self.stop_mission_callback)
        rospy.Service('/competition/reset_mission', Trigger, self.reset_mission_callback)

        rospy.set_param('~target_shape', 'None')
        rospy.set_param('~status', 'start')
        rospy.set_param('/moon_task/returned_to_base', False)

        rospy.Subscriber('/move_base/result', MoveBaseActionResult, self.move_callback)
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/moon_detector/result', String, self.moon_result_callback)
        rospy.Subscriber('/moon_detector/finished', Bool, self.moon_finished_callback)

        rospy.on_shutdown(self.on_shutdown)
        signal.signal(signal.SIGINT, self.shutdown)
        try:
            self._initialise_arm()
            if self.enable_voice:
                self._wait_for_voice_initialisation()
            self._wait_for_navigation_initialisation()
            if self.enable_voice:
                self.vc_sub = rospy.Subscriber(
                    '/asr_node/voice_words', String, self.words_callback
                )
            else:
                rospy.logwarn(
                    'voice input and playback are disabled; use timeout or service start'
                )
            self.initialization_complete = True
        except Exception as exc:
            rospy.logfatal('competition controller initialisation failed: %s', exc)
            self.mark_error()
            self.cleanup_mission_execution()
            self.save_mission_results()
            raise

        self.play('running')
        self.schedule_start_timeout()
        self.save_mission_results()
        rospy.loginfo('Competition controller is waiting for its one start trigger')
        rospy.spin()

    def _mission_param(self, name, default):
        return rospy.get_param(
            '/competition_mission/' + name,
            rospy.get_param('~' + name, default),
        )

    def _pose_param(self, name, default):
        value = self._mission_param(name, default)
        return {
            'x': float(value.get('x', default['x'])),
            'y': float(value.get('y', default['y'])),
            'yaw': float(value.get('yaw', default['yaw'])),
        }

    def _initialise_arm(self):
        deadline = rospy.Time.now() + rospy.Duration(self.initialization_timeout)
        ready = False
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            try:
                ready = bool(rospy.get_param('/servo_manager/init_finish')) and bool(
                    rospy.get_param('/joint_states_publisher/init_finish')
                )
            except KeyError:
                ready = False
            if ready:
                break
            rospy.sleep(0.1)
        if not ready:
            raise RuntimeError('servo initialisation timed out')
        if not self.safe_arm_pose(wait_seconds=2.0):
            raise RuntimeError('failed to place arm in its initial safe pose')

    def _wait_for_voice_initialisation(self):
        deadline = rospy.Time.now() + rospy.Duration(self.voice_init_timeout)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            if rospy.get_param('/voice_control/init_finish', False):
                return
            rospy.sleep(0.1)
        rospy.logwarn('voice control init timeout; the one-shot auto start remains enabled')

    def _wait_for_navigation_initialisation(self):
        try:
            rospy.wait_for_message(
                self.costmap, OccupancyGrid, timeout=self.initialization_timeout
            )
        except rospy.ROSException as exc:
            raise RuntimeError('costmap initialisation timed out: %s' % exc)

    def schedule_start_timeout(self):
        self.cancel_start_timeout()
        if not self.enable_timeout_auto_start or self.start_timeout_seconds <= 0:
            return
        if self.mission_state != WAITING_FOR_START:
            return
        self._start_timeout_timer = rospy.Timer(
            rospy.Duration(self.start_timeout_seconds),
            self.start_timeout_callback,
            oneshot=True,
        )

    def cancel_start_timeout(self):
        timer = self._start_timeout_timer
        self._start_timeout_timer = None
        if timer is not None:
            try:
                timer.shutdown()
            except Exception:
                pass

    def start_timeout_callback(self, _event):
        self.request_mission_start('timeout')

    def request_mission_start(self, trigger_source):
        with self._transition_lock:
            if not self.initialization_complete:
                rospy.logwarn('Rejected mission start before controller initialisation completed')
                return False
            if self._debug_action_active:
                rospy.logwarn('Rejected mission start while a manual debug action is active')
                return False
            accepted = self.request_start_token(
                trigger_source, ros_shutdown=rospy.is_shutdown()
            )
            if not accepted:
                rospy.logwarn(
                    'Rejected mission start from %s in state %s',
                    trigger_source,
                    self.mission_state,
                )
                return False
            self._reset_run_data()
            self.cancel_start_timeout()
            self.save_mission_results()
            self._mission_thread = threading.Thread(
                target=self.run_mission_once,
                name='moon-competition-mission',
            )
            self._mission_thread.daemon = True
            self._mission_thread.start()
            return True

    def _reset_run_data(self):
        with self._lock:
            self.moon_task_results = {1: None, 2: None, 3: None}
            self.three_scene_recognitions_finished = False
            self.all_competition_tasks_finished = False
            self.return_to_base_requested = False
            self.returned_to_base = False
            self.results_announced = False
        self.completed_pick_tasks = 0
        self.completed_place_tasks = 0
        self.navigation_tasks_finished = False
        self.ramp_task_finished = False
        self.other_competition_tasks_finished = False
        for task_point_index in (1, 2, 3):
            try:
                rospy.delete_param('/moon_task/results/%d' % task_point_index)
            except KeyError:
                pass
        rospy.set_param('/moon_task/returned_to_base', False)

    def start_mission_callback(self, _request):
        success = self.request_mission_start('service')
        return TriggerResponse(success=success, message=self.mission_state)

    def stop_mission_callback(self, _request):
        with self._transition_lock:
            changed = self.mark_stopped()
        self.cleanup_mission_execution()
        self.save_mission_results()
        return TriggerResponse(success=changed, message=self.mission_state)

    def reset_mission_callback(self, _request):
        with self._transition_lock:
            active = self._mission_thread is not None and self._mission_thread.is_alive()
            if (
                self.mission_state not in (COMPLETED, STOPPED, ERROR)
                or active
                or self._debug_action_active
            ):
                return TriggerResponse(
                    success=False,
                    message='reset requires COMPLETED, STOPPED, or ERROR with no active thread',
                )
            self.safe_stop_robot()
            if not self.safe_arm_pose(wait_seconds=2.0):
                return TriggerResponse(
                    success=False,
                    message='reset rejected because the arm did not reach its safe pose',
                )
            if not self.unload_moon_detector():
                return TriggerResponse(
                    success=False,
                    message='reset rejected because the Moon model could not be unloaded',
                )
            if not self.reset(active_thread=active):
                return TriggerResponse(
                    success=False,
                    message='reset requires COMPLETED, STOPPED, or ERROR with no active thread',
                )
            self._reset_run_data()
            with self._gpu_lifecycle_lock:
                self._yolo_unloaded_for_mission = False
            self.words = None
            self.wakeup_detected = False
            rospy.set_param('/moon_task/returned_to_base', False)
            self.schedule_start_timeout()
            self.save_mission_results()
            return TriggerResponse(success=True, message=self.mission_state)

    def test_callback(self, _request):
        success = self.request_mission_start('test_service')
        return TriggerResponse(success=success, message=self.mission_state)

    def start_pick_callback(self, _request):
        return self._run_debug_service(
            'pick', lambda: TriggerResponse(success=self.safe_pick())
        )

    def start_place_callback(self, _request):
        return self._run_debug_service(
            'place', lambda: TriggerResponse(success=self.control(0, 0, 0, 'place'))
        )

    def start_detect_callback(self, _request):
        return self._run_debug_service(
            'detect', lambda: TriggerResponse(success=self.control(0, 0, 0, 'detect'))
        )

    def start_scene_card_callback(self, _request):
        return self._run_debug_service('scene_card', self._debug_scene_card)

    def _debug_scene_card(self):
        task_point_index = int(rospy.get_param('~debug_task_point_index', 1))
        result = self.detect_moon_scene_at_task_point(task_point_index)
        return TriggerResponse(
            success=result.get('status') == 'success',
            message=json.dumps(result, ensure_ascii=False),
        )

    def start_aligning_callback(self, _request):
        return self._run_debug_service('aligning', self._debug_aligning)

    def _debug_aligning(self):
        for service_name in (
            '/position_correction/start',
            '/position_correction/pick_1',
        ):
            response = self.call_trigger(service_name, timeout=5.0)
            if response is None or not response.success:
                return TriggerResponse(success=False, message=service_name + ' failed')
        if not self.wait_correction_status():
            return TriggerResponse(success=False, message='pick_1 alignment timeout')
        response = self.call_trigger('/position_correction/pick_2', timeout=5.0)
        if response is None or not response.success:
            return TriggerResponse(success=False, message='pick_2 failed')
        if not self.wait_correction_status():
            return TriggerResponse(success=False, message='pick_2 alignment timeout')
        return TriggerResponse(
            success=self._drive_for(linear_x=0.05, duration=self.pick_location_time)
        )

    def start_back_callback(self, _request):
        return self._run_debug_service(
            'back', lambda: TriggerResponse(success=self.control(0.1, -0.3, 0, 'back'))
        )

    def _run_debug_service(self, action_name, action):
        if not self.enable_manual_debug_services:
            return TriggerResponse(success=False, message='manual debug services are disabled')
        if not self._debug_action_lock.acquire(False):
            return TriggerResponse(success=False, message='another debug action is active')
        try:
            with self._transition_lock:
                if (
                    self.mission_state != WAITING_FOR_START
                    or self.mission_started
                    or self.stop_requested
                    or rospy.is_shutdown()
                ):
                    return TriggerResponse(
                        success=False,
                        message='%s requires an idle WAITING_FOR_START mission' % action_name,
                    )
                self._debug_action_active = True
                self.cancel_start_timeout()
            try:
                return action()
            except Exception as exc:
                rospy.logerr('manual debug action %s failed: %s', action_name, exc)
                return TriggerResponse(success=False, message=str(exc))
            finally:
                self.cleanup_mission_execution()
                with self._transition_lock:
                    self._debug_action_active = False
                    if self.mission_state == WAITING_FOR_START and not self.stop_requested:
                        # Debug cleanup leaves both GPU models unloaded; the next
                        # real mission may load the mineral detector again.
                        with self._gpu_lifecycle_lock:
                            self._yolo_unloaded_for_mission = False
                        self.schedule_start_timeout()
        finally:
            self._debug_action_lock.release()

    def words_callback(self, msg):
        words = json.dumps(msg.data, ensure_ascii=False)[1:-1]
        if self.language == 'Chinese':
            words = words.replace(' ', '')
        self.words = words
        if words == WAKEUP_SUCCESS:
            self.wakeup_detected = True
            self.play('awake')
            return
        if words == SLEEP_COMMAND:
            buzzer = BuzzerState()
            buzzer.freq = 1900
            buzzer.on_time = 0.05
            buzzer.off_time = 0.01
            buzzer.repeat = 1
            self.buzzer_pub.publish(buzzer)
            return
        expected = START_SECURITY_COMMAND if not self.slope_surface else START_TASK_COMMAND
        if words == expected:
            self.request_mission_start('voice')

    def scan_callback(self, msg):
        try:
            left_ranges = []
            right_ranges = []
            for index, distance in enumerate(msg.ranges):
                if not math.isfinite(distance) or distance <= 0.1:
                    continue
                degrees = math.degrees(msg.angle_min + index * msg.angle_increment)
                if 150 <= degrees <= 180:
                    left_ranges.append(distance)
                if degrees < 0:
                    degrees += 360
                if 180 <= degrees <= 210:
                    right_ranges.append(distance)
            self.left_rear_dist = min(left_ranges) if left_ranges else 2.0
            self.right_rear_dist = min(right_ranges) if right_ranges else 2.0
        except Exception as exc:
            rospy.logwarn_throttle(5.0, 'scan callback failed: %s', exc)

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose
        self.current_pose_received = True

    def move_callback(self, msg):
        try:
            status = msg.status.status
            if status == GoalStatus.SUCCEEDED:
                self.move_base_status = 3
            elif status > GoalStatus.SUCCEEDED:
                self.move_base_status = -1
            else:
                self.move_base_status = 1
        except Exception:
            self.move_base_status = 1

    def play(self, name):
        if not self.enable_voice:
            return True
        try:
            voice_play.play(name, volume=self.voice_volume, language=self.language)
            return True
        except Exception as exc:
            rospy.logwarn('voice playback failed for %s: %s', name, exc)
            return False

    def play_moon_asset(self, asset_name):
        if not self.enable_voice:
            return True
        path = os.path.join(self.moon_voice_dir, asset_name + '.wav')
        if not os.path.isfile(path):
            rospy.logerr('offline voice asset is missing: %s', path)
            return False
        try:
            subprocess.call(
                [
                    'amixer',
                    '-q',
                    '-D',
                    'pulse',
                    'set',
                    'Master',
                    '%d%%' % self.voice_volume,
                ],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
        except OSError:
            pass
        try:
            return subprocess.call(['aplay', '-q', path], timeout=20.0) == 0
        except Exception as exc:
            rospy.logerr('offline voice playback failed for %s: %s', path, exc)
            return False

    def call_trigger(self, service_name, timeout=2.0):
        timeout = max(0.1, float(timeout))
        started_at = time.monotonic()
        try:
            rospy.wait_for_service(service_name, timeout=timeout)
        except Exception as exc:
            rospy.logwarn('service %s unavailable: %s', service_name, exc)
            return None

        remaining = timeout - (time.monotonic() - started_at)
        if remaining <= 0.0:
            rospy.logwarn('service %s discovery exhausted its %.1fs timeout', service_name, timeout)
            return None

        completed = threading.Event()
        abandoned = threading.Event()
        outcome = {}

        def neutralise_late_response(response):
            if not getattr(response, 'success', False):
                return
            cleanup_service = {
                '/moon_detector/start': '/moon_detector/stop',
                '/ramp/start': '/ramp/stop',
                '/ramp/up': '/ramp/stop',
                '/shape_recognition/start': '/shape_recognition/stop',
                '/yolov5/start': '/yolov5/stop',
            }.get(service_name)
            if not cleanup_service:
                return
            try:
                rospy.ServiceProxy(cleanup_service, Trigger)()
            except Exception as cleanup_exc:
                rospy.logwarn(
                    'late service %s could not be neutralised through %s: %s',
                    service_name,
                    cleanup_service,
                    cleanup_exc,
                )

        def invoke():
            try:
                response = rospy.ServiceProxy(service_name, Trigger)()
                outcome['response'] = response
                if abandoned.is_set():
                    neutralise_late_response(response)
            except Exception as exc:
                outcome['error'] = exc
            finally:
                completed.set()

        worker = threading.Thread(target=invoke, name='trigger-' + service_name.strip('/').replace('/', '-'))
        worker.daemon = True
        worker.start()
        if not completed.wait(remaining):
            abandoned.set()
            if 'response' in outcome:
                cleanup_worker = threading.Thread(
                    target=neutralise_late_response,
                    args=(outcome['response'],),
                    name='cleanup-' + service_name.strip('/').replace('/', '-'),
                )
                cleanup_worker.daemon = True
                cleanup_worker.start()
            rospy.logwarn('service %s call exceeded its %.1fs timeout', service_name, timeout)
            return None

        if 'error' in outcome:
            rospy.logwarn('service %s failed: %s', service_name, outcome['error'])
            return None
        response = outcome.get('response')
        if response is None:
            rospy.logwarn('service %s returned no response', service_name)
            return None
        if not response.success:
            rospy.logwarn('service %s rejected request: %s', service_name, response.message)
        return response

    def call_trigger_compat(self, preferred_name, legacy_name):
        response = self.call_trigger(preferred_name, timeout=1.0)
        if response is not None:
            return response
        return self.call_trigger(legacy_name, timeout=1.0)

    def safe_stop_robot(self):
        try:
            self.mecanum_pub.publish(Twist())
        except Exception:
            pass

    def cancel_navigation_and_stop(self):
        try:
            self.move_base_client.cancel_all_goals()
        except Exception:
            try:
                self.move_base_client.cancel_goal()
            except Exception:
                pass
        for _index in range(3):
            self.safe_stop_robot()

    def cleanup_mission_execution(self):
        self.cancel_start_timeout()
        self.cancel_navigation_and_stop()
        self.stop_moon_detector()
        self.unload_moon_detector()
        if self.slope_surface:
            self.call_trigger('/ramp/stop', timeout=1.0)
        self.call_trigger('/position_correction/stop', timeout=1.0)
        self.call_trigger('/shape_recognition/stop', timeout=1.0)
        self.call_trigger('/yolov5/stop', timeout=1.0)
        self._unload_yolo_for_mission()
        self.cancel_navigation_and_stop()

    def _drive_for(self, linear_x=0.0, linear_y=0.0, angular_z=0.0, duration=0.0):
        twist = Twist()
        twist.linear.x = linear_x
        twist.linear.y = linear_y
        twist.angular.z = angular_z
        deadline = rospy.Time.now() + rospy.Duration(max(0.0, duration))
        rate = rospy.Rate(20)
        try:
            while not rospy.is_shutdown() and rospy.Time.now() < deadline:
                if self.stop_requested:
                    return False
                self.mecanum_pub.publish(twist)
                rate.sleep()
            return not self.stop_requested and not rospy.is_shutdown()
        finally:
            self.safe_stop_robot()

    def safe_arm_pose(self, wait_seconds=1.0):
        command_duration = 2.0
        try:
            bus_servo_control.set_servos(
                self.joints_pub,
                command_duration,
                ((1, 500), (2, 760), (3, 15), (4, 150), (5, 500), (10, 200)),
            )
            rospy.sleep(max(command_duration, float(wait_seconds)))
            self.mechanical_arm_safe = True
            return True
        except Exception as exc:
            self.mechanical_arm_safe = False
            rospy.logerr('failed to move arm to safe pose: %s', exc)
            return False

    def make_move_base_goal(self, x, y, yaw_degrees):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = self.map_frame
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = float(x)
        goal.target_pose.pose.position.y = float(y)
        goal.target_pose.pose.orientation = rpy2qua(
            0.0, 0.0, math.radians(float(yaw_degrees))
        )
        return goal

    def nav_position(self, x, y, yaw_degrees):
        pose = PoseStamped()
        pose.header.frame_id = self.map_frame
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.orientation = rpy2qua(0.0, 0.0, math.radians(float(yaw_degrees)))
        self.goal_pub.publish(pose)
        return pose

    def navigate_and_wait(self, x, y, yaw_degrees, timeout=None):
        timeout = float(timeout or self.navigation_timeout)
        self.safe_stop_robot()
        if not self.move_base_client.wait_for_server(
            rospy.Duration(self.move_base_server_timeout)
        ):
            rospy.logerr('move_base action server is unavailable')
            return False
        goal = self.make_move_base_goal(x, y, yaw_degrees)
        self.move_base_client.send_goal(goal)
        deadline = rospy.Time.now() + rospy.Duration(timeout)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            if self.stop_requested:
                self.move_base_client.cancel_goal()
                self.safe_stop_robot()
                return False
            if self.move_base_client.wait_for_result(rospy.Duration(0.2)):
                state = self.move_base_client.get_state()
                if state == GoalStatus.SUCCEEDED:
                    self.safe_stop_robot()
                    return True
                rospy.logerr('navigation failed with move_base state %s', state)
                self.safe_stop_robot()
                return False
        self.move_base_client.cancel_goal()
        self.safe_stop_robot()
        rospy.logerr('navigation timed out after %.1f seconds', timeout)
        return False

    def wait_nav_status(self, timeout=30.0):
        start = rospy.Time.now()
        while not rospy.is_shutdown() and not self.stop_requested:
            if self.move_base_status == 3:
                self.move_base_status = 1
                return True
            if self.move_base_status == -1:
                self.move_base_status = 1
                self.safe_stop_robot()
                return False
            if (rospy.Time.now() - start).to_sec() > timeout:
                self.move_base_status = 1
                self.safe_stop_robot()
                return False
            rospy.sleep(0.1)
        return False

    def wait_correction_status(self, timeout=30.0):
        start = rospy.Time.now()
        while not rospy.is_shutdown() and not self.stop_requested:
            if rospy.get_param('/position_correction/status', 'stop') == 'stop':
                return True
            if (rospy.Time.now() - start).to_sec() >= timeout:
                self.safe_stop_robot()
                return False
            rospy.sleep(0.2)
        return False

    def wait_ramp_status(self, timeout=30.0, require_active_transition=True):
        start = rospy.Time.now()
        active_seen = not require_active_transition
        while not rospy.is_shutdown() and not self.stop_requested:
            status = rospy.get_param('/ramp/status', 'stop')
            if status != 'stop':
                active_seen = True
            elif active_seen:
                return True
            if (rospy.Time.now() - start).to_sec() >= timeout:
                self.safe_stop_robot()
                return False
            rospy.sleep(0.2)
        return False

    def run_ramp_alignment(self):
        if self.stop_requested or rospy.is_shutdown():
            return False
        start_response = self.call_trigger('/ramp/start', timeout=5.0)
        if start_response is None or not start_response.success:
            return False
        if self.stop_requested or rospy.is_shutdown():
            self.call_trigger('/ramp/stop', timeout=2.0)
            return False
        try:
            up_response = self.call_trigger('/ramp/up', timeout=5.0)
            if up_response is None or not up_response.success:
                return False
            if self.stop_requested or rospy.is_shutdown():
                return False
            return self.wait_ramp_status(require_active_transition=True)
        finally:
            self.call_trigger('/ramp/stop', timeout=2.0)

    def wait_pick_status(self, timeout=20.0):
        start = rospy.Time.now()
        while not rospy.is_shutdown() and not self.stop_requested:
            if rospy.get_param('/shape_recognition/status', 'start') == 'stop':
                rospy.set_param('/shape_recognition/status', 'start')
                return True
            if (rospy.Time.now() - start).to_sec() >= timeout:
                rospy.set_param('/shape_recognition/status', 'start')
                self.safe_stop_robot()
                return False
            rospy.sleep(0.2)
        return False

    def wait_yolo_status(self, timeout=10.0):
        start = rospy.Time.now()
        while not rospy.is_shutdown() and not self.stop_requested:
            shape = rospy.get_param('/yolov5/shape', 'None')
            if shape != 'None':
                return shape
            if (rospy.Time.now() - start).to_sec() >= timeout:
                return None
            rospy.sleep(0.2)
        return None

    def reverse_up_ramp_with_laser(self, distance, speed, timeout):
        if not self.current_pose_received:
            rospy.logerr('cannot confirm ramp return without odometry')
            return False
        try:
            self.move_base_client.cancel_goal()
        except Exception:
            pass
        start_x = self.current_pose.position.x
        start_y = self.current_pose.position.y
        start_time = rospy.Time.now()
        rate = rospy.Rate(20)
        traveled = 0.0
        try:
            while not rospy.is_shutdown() and not self.stop_requested:
                dx = self.current_pose.position.x - start_x
                dy = self.current_pose.position.y - start_y
                traveled = math.sqrt(dx * dx + dy * dy)
                if traveled >= distance:
                    return True
                if (rospy.Time.now() - start_time).to_sec() >= timeout:
                    rospy.logerr(
                        'ramp return timed out at %.3f of %.3f metres', traveled, distance
                    )
                    return False
                twist = Twist()
                twist.linear.x = -abs(speed)
                error = self.left_rear_dist - self.right_rear_dist
                twist.angular.z = max(-0.5, min(0.5, 0.6 * error))
                self.mecanum_pub.publish(twist)
                rate.sleep()
            return False
        finally:
            self.safe_stop_robot()

    def moon_result_callback(self, msg):
        try:
            payload = json.loads(msg.data)
        except Exception as exc:
            rospy.logwarn('invalid moon detector JSON: %s', exc)
            return
        with self._moon_result_lock:
            request = self._current_moon_request
            if request is None:
                return
            if payload.get('session_id') != request['session_id']:
                return
            if int(payload.get('task_point_index', 0)) != request['task_point_index']:
                return
            timestamp_ros = float(payload.get('timestamp_ros', 0.0))
            if timestamp_ros < request['started_at_ros']:
                return
            if rospy.Time.now().to_sec() - timestamp_ros > self.maximum_result_age:
                return
            self._current_moon_result = payload
            self._moon_result_event.set()

    def moon_finished_callback(self, msg):
        if not msg.data:
            return
        with self._moon_result_lock:
            if self._current_moon_request is not None and self._current_moon_result is not None:
                self._moon_result_event.set()

    def _moon_failure_result(self, task_point_index, session_id, status, error):
        return {
            'task_point_index': task_point_index,
            'task_point_name': TASK_POINT_NAME[task_point_index],
            'session_id': session_id,
            'status': status,
            'class_id': None,
            'class_name_en': '',
            'class_name_cn': '',
            'confidence': 0.0,
            'detection_count': 0,
            'timestamp': utc_timestamp(),
            'timestamp_ros': rospy.Time.now().to_sec(),
            'error': str(error),
        }

    def _normalise_moon_result(self, task_point_index, session_id, payload):
        status = str(payload.get('status', 'invalid_result'))
        result = self._moon_failure_result(
            task_point_index,
            session_id,
            status,
            payload.get('error', ''),
        )
        result['timestamp'] = str(payload.get('timestamp', result['timestamp']))
        result['timestamp_ros'] = float(
            payload.get('timestamp_ros', result['timestamp_ros'])
        )
        result['detection_count'] = int(payload.get('detection_count', 0))
        if status != 'success':
            return result
        try:
            class_id = int(payload['class_id'])
            class_name_en = str(payload['class_name_en'])
            if class_id < 0 or class_id >= len(MOON_CLASSES):
                raise ValueError('class_id out of range')
            if MOON_CLASSES[class_id] != class_name_en:
                raise ValueError('class mapping mismatch')
            result.update(
                {
                    'status': 'success',
                    'class_id': class_id,
                    'class_name_en': class_name_en,
                    'class_name_cn': MOON_CLASS_CN[class_name_en],
                    'confidence': float(payload.get('confidence', 0.0)),
                    'error': '',
                }
            )
            return result
        except Exception as exc:
            result['status'] = 'invalid_result'
            result['error'] = str(exc)
            return result

    def detect_moon_scene_at_task_point(
        self, task_point_index, rotate=False, reverse_rotate=False
    ):
        if task_point_index not in (1, 2, 3):
            raise ValueError('task_point_index must be 1, 2, or 3')
        self.safe_stop_robot()
        if not self._unload_yolo_for_mission():
            result = self._moon_failure_result(
                task_point_index,
                uuid.uuid4().hex,
                'detector_unavailable',
                'existing mineral-card YOLO could not release its GPU context',
            )
            self._store_moon_result(task_point_index, result)
            return result
        if rotate and not self._drive_for(angular_z=-0.5, duration=math.pi):
            result = self._moon_failure_result(
                task_point_index, uuid.uuid4().hex, 'stopped', 'rotation interrupted'
            )
            self._store_moon_result(task_point_index, result)
            return result

        session_id = uuid.uuid4().hex
        started_at_ros = rospy.Time.now().to_sec()
        request = {
            'task_point_index': task_point_index,
            'session_id': session_id,
            'started_at_ros': started_at_ros,
        }
        with self._moon_result_lock:
            self._current_moon_request = request
            self._current_moon_result = None
            self._moon_result_event.clear()

        result = None
        try:
            reset_response = self.call_trigger('/moon_detector/reset', timeout=3.0)
            if reset_response is None or not reset_response.success:
                result = self._moon_failure_result(
                    task_point_index, session_id, 'detector_unavailable', 'reset failed'
                )
            else:
                rospy.set_param('/moon_detector/request/task_point_index', task_point_index)
                rospy.set_param('/moon_detector/request/session_id', session_id)
                rospy.set_param('/moon_detector/request/started_at_ros', started_at_ros)
                start_response = self.call_trigger(
                    '/moon_detector/start', timeout=self.moon_start_timeout
                )
                if start_response is None or not start_response.success:
                    result = self._moon_failure_result(
                        task_point_index,
                        session_id,
                        'model_error',
                        start_response.message if start_response is not None else 'start failed',
                    )
                else:
                    deadline = rospy.Time.now() + rospy.Duration(
                        self.moon_detection_timeout
                    )
                    rate = rospy.Rate(20)
                    while not rospy.is_shutdown() and rospy.Time.now() < deadline:
                        if self.stop_requested:
                            break
                        if self._moon_result_event.is_set():
                            break
                        rate.sleep()
                    with self._moon_result_lock:
                        payload = self._current_moon_result
                    if payload is None:
                        result = self._moon_failure_result(
                            task_point_index,
                            session_id,
                            'timeout',
                            'no fresh result before controller timeout',
                        )
                    else:
                        result = self._normalise_moon_result(
                            task_point_index, session_id, payload
                        )
        finally:
            self.stop_moon_detector()
            with self._moon_result_lock:
                self._current_moon_request = None
                self._current_moon_result = None
                self._moon_result_event.clear()
            if (
                rotate
                and reverse_rotate
                and not self.stop_requested
                and not rospy.is_shutdown()
            ):
                self._drive_for(angular_z=0.5, duration=math.pi)
            self._restart_yolo()
            self.safe_stop_robot()

        if result is None:
            result = self._moon_failure_result(
                task_point_index, session_id, 'error', 'detector ended without a result'
            )
        self._store_moon_result(task_point_index, result)
        return result

    def _store_moon_result(self, task_point_index, result):
        stored = self.record_scene_result(task_point_index, result)
        rospy.set_param('/moon_task/results/%d' % task_point_index, stored)
        self.save_mission_results()

    def stop_moon_detector(self):
        self.call_trigger('/moon_detector/stop', timeout=1.0)

    def unload_moon_detector(self):
        response = self.call_trigger(
            '/moon_detector/unload', timeout=self.moon_unload_timeout
        )
        return response is not None and response.success

    def safe_pick(self):
        self.call_trigger('/yolov5/stop', timeout=2.0)
        try:
            rospy.set_param('/shape_recognition/status', 'start')
            start_response = self.call_trigger('/shape_recognition/start', timeout=5.0)
            if start_response is None or not start_response.success:
                return False
            rospy.sleep(1.5)
            pick_response = self.call_trigger('/shape_recognition/pick', timeout=5.0)
            if pick_response is None or not pick_response.success:
                return False
            rospy.sleep(1.0)
            success = self.wait_pick_status(timeout=20.0)
            stop_response = self.call_trigger('/shape_recognition/stop', timeout=2.0)
            if stop_response is None or not stop_response.success:
                return False
            rospy.set_param('/shape_recognition/status', 'start')
            rospy.sleep(0.5)
            return success
        except Exception as exc:
            rospy.logerr('pick task failed: %s', exc)
            self.safe_stop_robot()
            return False
        finally:
            self.call_trigger('/shape_recognition/stop', timeout=1.0)
            try:
                rospy.set_param('/shape_recognition/status', 'start')
            except Exception:
                pass
            self._restart_yolo()

    def _restart_yolo(self):
        with self._gpu_lifecycle_lock:
            if (
                self._yolo_unloaded_for_mission
                or self.stop_requested
                or rospy.is_shutdown()
                or self.mission_state in (STOPPED, ERROR, COMPLETED)
            ):
                return False
            response = self.call_trigger(
                '/yolov5/start', timeout=self.yolo_start_timeout
            )
            return response is not None and response.success

    def _unload_yolo_for_mission(self):
        with self._gpu_lifecycle_lock:
            if self._yolo_unloaded_for_mission:
                return True
            response = self.call_trigger(
                '/yolov5/unload', timeout=self.yolo_unload_timeout
            )
            if response is None or not response.success:
                return False
            self._yolo_unloaded_for_mission = True
            return True

    def control(self, x, y, yaw, set_status):
        if self.stop_requested or rospy.is_shutdown():
            return False
        if set_status != 'pick2':
            if not self.navigate_and_wait(x, y, yaw):
                return False

        if set_status == 'pick1':
            if not self.navigate_and_wait(1.30, -3.12, 0.0):
                return False
            if not self._drive_for(linear_x=0.108, duration=2.0):
                return False
            if not self.safe_pick():
                return False
            self.play('7')
            if not self._drive_for(linear_x=-0.05, duration=1.0):
                return False
            self.detect_moon_scene_at_task_point(
                2, rotate=True, reverse_rotate=True
            )
            return self._drive_for(linear_x=-0.1, duration=1.5)

        if set_status == 'pick2':
            target_yaw = -180.0
            navigation_yaw = target_yaw
            if self.current_pose_received:
                quaternion = self.current_pose.orientation
                siny_cosp = 2.0 * (
                    quaternion.w * quaternion.z + quaternion.x * quaternion.y
                )
                cosy_cosp = 1.0 - 2.0 * (
                    quaternion.y * quaternion.y + quaternion.z * quaternion.z
                )
                current_yaw = math.degrees(math.atan2(siny_cosp, cosy_cosp))
                difference = abs(current_yaw - target_yaw)
                difference = min(difference, 360.0 - difference)
                if difference <= 10.0:
                    navigation_yaw = current_yaw
            if not self.navigate_and_wait(0.94, -3.17, navigation_yaw):
                return False
            if not self._drive_for(linear_x=0.24, duration=2.0):
                return False
            if not self.safe_pick():
                return False
            self.play('7')
            self.detect_moon_scene_at_task_point(3)
            return self._drive_for(linear_x=-0.2, duration=3.0)

        if set_status == 'place':
            if not self._drive_for(linear_x=0.2, duration=1.7):
                return False
            response = self.call_trigger(
                '/position_correction/place_3',
                timeout=self.place_service_timeout,
            )
            if response is None or not response.success:
                self.safe_stop_robot()
                return False
            rospy.sleep(1.0)
            if not self.wait_correction_status():
                return False
            self.play('9')
            return self._drive_for(linear_x=-0.2, duration=2.0)

        if set_status == 'detect':
            self.play('reached_explosion-proof_warehouse' if not self.slope_surface else '2')
            rospy.sleep(1.0)
            shape = self.wait_yolo_status(timeout=10.0)
            if shape is None:
                rospy.logerr('shape detection timed out; refusing to invent a mineral class')
                return False
            if shape not in ('cube', 'box', 'cylinder'):
                rospy.logerr('shape detector returned unsupported class: %s', shape)
                return False
            if not self.slope_surface:
                audio = {
                    'cube': 'ruled_out_cuboid_explosive',
                    'box': 'ruled_out_cube_explosive',
                }.get(shape, 'ruled_out_cylinder_explosive')
            else:
                audio = {'cube': '3', 'box': '4'}.get(shape, '5')
            self.play(audio)
            rospy.set_param('/shape_recognition/target_shape', shape)
            self.detect_moon_scene_at_task_point(
                1, rotate=True, reverse_rotate=False
            )
            rospy.sleep(2.0)
            return True

        if set_status == 'back':
            rospy.set_param('~status', 'stop')
            if self.slope_surface:
                if not self.run_ramp_alignment():
                    return False
                if not self._drive_for(linear_y=-0.1, duration=0.6):
                    return False
                if not self._drive_for(angular_z=-0.5, duration=0.1):
                    return False
                if not self._drive_for(linear_x=0.3, duration=self.up_ramp_time):
                    return False
                return self._drive_for(angular_z=0.5, duration=6.0)
            return True

        return True

    def _prepare_remaining_tasks(self):
        position_close = self.call_trigger_compat(
            '/position_correction/close', '/position_correction/colse'
        )
        shape_close = self.call_trigger_compat(
            '/shape_recognition/close', '/shape_recognition/colse'
        )
        if (
            position_close is None
            or not position_close.success
            or shape_close is None
            or not shape_close.success
        ):
            return False
        if not self.safe_arm_pose(wait_seconds=1.0):
            return False
        self.other_competition_tasks_finished = True
        if self.slope_surface:
            if not self.navigate_and_wait(
                self.ramp_approach_pose['x'],
                self.ramp_approach_pose['y'],
                self.ramp_approach_pose['yaw'],
            ):
                return False
            if self.use_ramp_alignment_service and not self.run_ramp_alignment():
                return False
            if not self.reverse_up_ramp_with_laser(
                distance=self.ramp_return_distance,
                speed=self.ramp_return_speed,
                timeout=self.ramp_return_timeout,
            ):
                return False
        else:
            if not self.control(0.1, -0.3, 0.0, 'back'):
                return False
            if not self._drive_for(linear_y=0.1, duration=3.0):
                return False
            self.play('mission_accomplished')
        if not self.safe_arm_pose(wait_seconds=1.0):
            return False
        self.ramp_task_finished = True
        return True

    def _body_tasks_complete(self):
        return (
            self.completed_pick_tasks >= 2
            and self.completed_place_tasks >= 2
            and self.navigation_tasks_finished
            and self.ramp_task_finished
            and self.other_competition_tasks_finished
            and self.mechanical_arm_safe
        )

    def begin_return_to_base(self):
        if not self._body_tasks_complete():
            rospy.logerr('return gate rejected incomplete competition body tasks')
            return False
        if not self.mark_returning(ros_shutdown=rospy.is_shutdown()):
            return False
        self.save_mission_results()
        if not self.navigate_and_wait(
            self.base_pose['x'], self.base_pose['y'], self.base_pose['yaw']
        ):
            return False
        if not self.safe_arm_pose(wait_seconds=1.0):
            return False
        self.safe_stop_robot()
        if not self.mark_returned():
            return False
        rospy.set_param('/moon_task/returned_to_base', True)
        self.save_mission_results()
        return True

    def announce_all_task_results_at_base(self):
        if not self.begin_announcing():
            return False
        self.safe_stop_robot()
        playback_succeeded = True
        for task_point_index in (1, 2, 3):
            if self.stop_requested or rospy.is_shutdown():
                return False
            result = self.moon_task_results.get(task_point_index)
            if result is not None and result.get('status') == 'success':
                class_name_en = result.get('class_name_en', '')
                class_name_cn = MOON_CLASS_CN.get(class_name_en, '')
                if class_name_cn:
                    rospy.loginfo(
                        '%s\u8bc6\u522b\u5230%s',
                        TASK_POINT_NAME[task_point_index],
                        class_name_cn,
                    )
                    prefix_ok = self.play_moon_asset(
                        'point_%d_prefix' % task_point_index
                    )
                    if self.stop_requested or rospy.is_shutdown():
                        return False
                    class_ok = self.play_moon_asset('class_' + class_name_en)
                    playback_succeeded = prefix_ok and class_ok and playback_succeeded
                    if self.stop_requested or rospy.is_shutdown():
                        return False
                    continue
            rospy.loginfo(
                '%s\u672a\u8bc6\u522b\u5230\u6709\u6548\u573a\u666f\u5143\u7d20',
                TASK_POINT_NAME[task_point_index],
            )
            playback_succeeded = (
                self.play_moon_asset('point_%d_failure' % task_point_index)
                and playback_succeeded
            )
            if self.stop_requested or rospy.is_shutdown():
                return False
        if not playback_succeeded:
            rospy.logerr('one or more result announcements failed at the base')
            return False
        if not self.mark_announced():
            return False
        self.save_mission_results()
        # The 2026 rules separately require this exact base-completion phrase.
        return self.play('mission_completed')

    def run_mission_once(self):
        try:
            if not self.mark_running():
                return
            self.save_mission_results()
            if not self._restart_yolo():
                raise MissionAbort('existing mineral-card YOLO failed to start')
            self.play('1')
            if not self._drive_for(linear_x=0.3, duration=3.0):
                raise MissionAbort('initial forward motion interrupted')
            if not self._drive_for(angular_z=-0.5, duration=3.0):
                raise MissionAbort('initial rotation interrupted')

            if not self.control(1.5, -0.15, 90.0, 'detect'):
                raise MissionAbort('resource-library detect stage failed')
            if not self.control(0.94, -3.124, 0.0, 'pick1'):
                raise MissionAbort('first pick stage failed')
            self.completed_pick_tasks = 1
            if not self.control(1.24, -0.22, 40.0, 'place'):
                raise MissionAbort('first place stage failed')
            self.completed_place_tasks = 1
            if not self.control(0.94, -3.124, -180.0, 'pick2'):
                raise MissionAbort('second pick stage failed')
            self.completed_pick_tasks = 2
            if not self.control(1.24, -0.22, 40.0, 'place'):
                raise MissionAbort('second place stage failed')
            self.completed_place_tasks = 2
            self.navigation_tasks_finished = True

            if not self._prepare_remaining_tasks():
                raise MissionAbort('post-recognition competition tasks failed')
            if not self._body_tasks_complete():
                raise MissionAbort('competition body-task completion flags are incomplete')
            if not self.mark_all_tasks_finished():
                raise MissionAbort('failed to mark all competition tasks finished')
            self.save_mission_results()

            if not self.can_begin_return(ros_shutdown=rospy.is_shutdown()):
                raise MissionAbort('return gate rejected the mission state')
            if not self.begin_return_to_base():
                raise MissionAbort('return to base failed')
            if not self.announce_all_task_results_at_base():
                raise MissionAbort('base result announcement failed')
            if not self.mark_completed():
                raise MissionAbort('failed to enter COMPLETED state')

            self.cancel_start_timeout()
            self.cleanup_mission_execution()
            self.save_mission_results()
            rospy.loginfo('Competition mission completed exactly once')
        except MissionAbort as exc:
            rospy.logerr('competition mission aborted: %s', exc)
            if self.mission_state not in (STOPPED, ERROR, COMPLETED):
                self.mark_error()
            self.cleanup_mission_execution()
            self.save_mission_results()
        except Exception as exc:
            rospy.logerr('unhandled competition mission error: %s', exc)
            if self.mission_state not in (STOPPED, ERROR, COMPLETED):
                self.mark_error()
            self.cleanup_mission_execution()
            self.save_mission_results()

    def save_mission_results(self):
        payload = self.snapshot()
        payload.update(
            {
                'completed_pick_tasks': self.completed_pick_tasks,
                'completed_place_tasks': self.completed_place_tasks,
                'navigation_tasks_finished': self.navigation_tasks_finished,
                'ramp_task_finished': self.ramp_task_finished,
                'other_competition_tasks_finished': self.other_competition_tasks_finished,
                'mechanical_arm_safe': self.mechanical_arm_safe,
                'updated_at': utc_timestamp(),
            }
        )
        try:
            atomic_write_json(self.results_path, payload)
            rospy.set_param('/moon_task/mission_state', payload['mission_state'])
            rospy.set_param(
                '/moon_task/mission_execution_count',
                payload['mission_execution_count'],
            )
        except Exception as exc:
            rospy.logerr('failed to persist mission results: %s', exc)

    def shutdown(self, _signum, _frame):
        rospy.signal_shutdown('SIGINT')

    def on_shutdown(self):
        if self._shutdown_handled:
            return
        self._shutdown_handled = True
        self.running = False
        self.cancel_start_timeout()
        with self._lock:
            self.stop_requested = True
            if self.mission_state not in (COMPLETED, STOPPED, ERROR):
                self.mission_state = STOPPED
                self.mission_finished = True
                self.start_trigger_consumed = True
        self.cleanup_mission_execution()
        self.save_mission_results()


if __name__ == '__main__':
    VoiceControlNavNode('voice_control_nav')
