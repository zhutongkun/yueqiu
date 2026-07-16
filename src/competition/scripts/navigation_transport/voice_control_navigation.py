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
from sensor_msgs.msg import Imu, LaserScan
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
    MissionTimeBudget,
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


def quaternion_roll_pitch_degrees(quaternion):
    sinr_cosp = 2.0 * (
        quaternion.w * quaternion.x + quaternion.y * quaternion.z
    )
    cosr_cosp = 1.0 - 2.0 * (
        quaternion.x * quaternion.x + quaternion.y * quaternion.y
    )
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (
        quaternion.w * quaternion.y - quaternion.z * quaternion.x
    )
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)
    return math.degrees(roll), math.degrees(pitch)


def quaternion_yaw_radians(quaternion):
    siny_cosp = 2.0 * (
        quaternion.w * quaternion.z + quaternion.x * quaternion.y
    )
    cosy_cosp = 1.0 - 2.0 * (
        quaternion.y * quaternion.y + quaternion.z * quaternion.z
    )
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_gravity_vector(quaternion):
    norm = math.sqrt(
        quaternion.x * quaternion.x
        + quaternion.y * quaternion.y
        + quaternion.z * quaternion.z
        + quaternion.w * quaternion.w
    )
    if norm <= 1e-9:
        raise ValueError('IMU orientation quaternion has zero length')
    x = quaternion.x / norm
    y = quaternion.y / norm
    z = quaternion.z / norm
    w = quaternion.w / norm
    return (
        2.0 * (x * z - w * y),
        2.0 * (y * z + w * x),
        w * w - x * x - y * y + z * z,
    )


def vector_angle_degrees(first, second):
    dot = sum(left * right for left, right in zip(first, second))
    dot = max(-1.0, min(1.0, dot))
    return math.degrees(math.acos(dot))


def wrapped_angle_delta_degrees(value, reference):
    return (value - reference + 180.0) % 360.0 - 180.0


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
        self._tilt_lock = threading.Lock()
        self.imu_received = False
        self.tilt_reference_ready = False
        self._tilt_reference_roll_degrees = 0.0
        self._tilt_reference_pitch_degrees = 0.0
        self._tilt_reference_gravity = (0.0, 0.0, 1.0)
        self.current_roll_degrees = 0.0
        self.current_pitch_degrees = 0.0
        self.current_tilt_degrees = 0.0
        self._controlled_ramp_motion = False
        self._tilt_violation_count = 0
        self._tilt_fault = False
        self._tilt_recovery_requested = threading.Event()
        self._tilt_recovery_active = False
        self._command_lock = threading.Lock()
        self._last_motion_linear_x = 0.0
        self._last_motion_linear_y = 0.0
        self._last_motion_command_time = 0.0
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
        self.mission_timeout_seconds = float(
            self._mission_param('mission_timeout_seconds', 450.0)
        )
        self.return_reserve_seconds = float(
            self._mission_param('return_reserve_seconds', 60.0)
        )
        self.announcement_reserve_seconds = float(
            self._mission_param('announcement_reserve_seconds', 20.0)
        )
        self.deadline_warning_seconds = float(
            self._mission_param('deadline_warning_seconds', 90.0)
        )
        self.service_discovery_timeout = float(
            self._mission_param('service_discovery_timeout', 2.0)
        )
        self.move_base_server_timeout = float(
            self._mission_param('move_base_server_timeout', 10.0)
        )
        self.moon_session_timeout = float(
            self._mission_param('moon_session_timeout', 25.0)
        )
        self.moon_rotation_degrees = abs(
            float(self._mission_param('moon_rotation_degrees', 90.0))
        )
        self.moon_rotation_speed = abs(
            float(self._mission_param('moon_rotation_speed', 0.5))
        )
        if self.moon_rotation_degrees <= 0.0:
            raise ValueError('moon_rotation_degrees must be positive')
        if self.moon_rotation_speed <= 0.0:
            raise ValueError('moon_rotation_speed must be positive')
        self.moon_rotation_duration = math.radians(
            self.moon_rotation_degrees
        ) / self.moon_rotation_speed
        self.moon_detection_timeout = float(
            self._mission_param('moon_detection_timeout', 12.0)
        )
        self.maximum_result_age = float(
            self._mission_param('maximum_result_age', 5.0)
        )
        self.place_service_timeout = float(
            self._mission_param('place_service_timeout', 8.0)
        )
        self.correction_status_timeout = float(
            self._mission_param('correction_status_timeout', 15.0)
        )
        self.shape_start_timeout = float(
            self._mission_param('shape_start_timeout', 3.0)
        )
        self.shape_pick_service_timeout = float(
            self._mission_param('shape_pick_service_timeout', 4.0)
        )
        self.shape_pick_timeout = float(
            self._mission_param('shape_pick_timeout', 18.0)
        )
        self.shape_pick_stage_timeout = float(
            self._mission_param('shape_pick_stage_timeout', 25.0)
        )
        self.shape_stop_timeout = float(
            self._mission_param('shape_stop_timeout', 1.0)
        )
        self.shape_warmup_seconds = float(
            self._mission_param('shape_warmup_seconds', 0.5)
        )
        self.yolo_detection_timeout = float(
            self._mission_param('yolo_detection_timeout', 8.0)
        )
        self.yolo_start_timeout = float(
            self._mission_param('yolo_start_timeout', 12.0)
        )
        self.prewarm_yolo_before_start = bool(
            self._mission_param('prewarm_yolo_before_start', True)
        )
        self.yolo_prewarm_timeout = float(
            self._mission_param('yolo_prewarm_timeout', 90.0)
        )
        self.yolo_unload_timeout = float(
            self._mission_param('yolo_unload_timeout', 5.0)
        )
        self.moon_start_timeout = float(
            self._mission_param('moon_start_timeout', 12.0)
        )
        self.prewarm_moon_before_start = bool(
            self._mission_param('prewarm_moon_before_start', True)
        )
        self.moon_prewarm_timeout = float(
            self._mission_param('moon_prewarm_timeout', 60.0)
        )
        self.moon_unload_timeout = float(
            self._mission_param('moon_unload_timeout', 5.0)
        )
        self.audio_playback_timeout = float(
            self._mission_param('audio_playback_timeout', 4.0)
        )
        self.initial_stage_timeout = float(
            self._mission_param('initial_stage_timeout', 20.0)
        )
        self.detect_stage_timeout = float(
            self._mission_param('detect_stage_timeout', 50.0)
        )
        self.pick_stage_timeout = float(
            self._mission_param('pick_stage_timeout', 85.0)
        )
        self.place_stage_timeout = float(
            self._mission_param('place_stage_timeout', 45.0)
        )
        self.remaining_tasks_timeout = float(
            self._mission_param('remaining_tasks_timeout', 60.0)
        )
        self.return_stage_timeout = float(
            self._mission_param('return_stage_timeout', 40.0)
        )
        self.announcement_stage_timeout = float(
            self._mission_param('announcement_stage_timeout', 20.0)
        )
        self.voice_volume = int(self._mission_param('voice_volume', 100))
        self.use_ramp_alignment_service = bool(
            self._mission_param('use_ramp_alignment_service', False)
        )
        self.enable_manual_debug_services = bool(
            self._mission_param('enable_manual_debug_services', True)
        )
        self.enable_tilt_guard = bool(
            self._mission_param('enable_tilt_guard', True)
        )
        self.imu_topic = str(
            self._mission_param('imu_topic', '/imu')
        )
        self.unexpected_tilt_limit_degrees = float(
            self._mission_param('unexpected_tilt_limit_degrees', 8.0)
        )
        self.controlled_ramp_tilt_limit_degrees = float(
            self._mission_param('controlled_ramp_tilt_limit_degrees', 25.0)
        )
        self.tilt_violation_samples = max(
            1, int(self._mission_param('tilt_violation_samples', 5))
        )
        self.tilt_recovery_speed = abs(
            float(self._mission_param('tilt_recovery_speed', 0.12))
        )
        self.tilt_recovery_timeout = float(
            self._mission_param('tilt_recovery_timeout', 3.0)
        )
        self.tilt_recovery_min_duration = float(
            self._mission_param('tilt_recovery_min_duration', 0.6)
        )
        self.tilt_recovery_level_degrees = float(
            self._mission_param('tilt_recovery_level_degrees', 5.0)
        )
        self.tilt_recovery_level_samples = max(
            1, int(self._mission_param('tilt_recovery_level_samples', 3))
        )
        self.tilt_recovery_max_attempts = max(
            1, int(self._mission_param('tilt_recovery_max_attempts', 2))
        )
        self.tilt_command_max_age = float(
            self._mission_param('tilt_command_max_age', 0.75)
        )
        self.enable_ramp_clearance_waypoints = bool(
            self._mission_param('enable_ramp_clearance_waypoints', True)
        )

        self.base_pose = self._pose_param(
            'base_pose', {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
        )
        self.ramp_approach_pose = self._pose_param(
            'ramp_approach_pose', {'x': 1.3, 'y': 0.0, 'yaw': 0.0}
        )
        self.departure_ramp_clearance_pose = self._pose_param(
            'departure_ramp_clearance_pose', {'x': 0.9, 'y': -0.7, 'yaw': -90.0}
        )
        self.place_ramp_clearance_pose = self._pose_param(
            'place_ramp_clearance_pose', {'x': 1.45, 'y': -0.75, 'yaw': 90.0}
        )
        self.pick_ramp_clearance_pose = self._pose_param(
            'pick_ramp_clearance_pose', {'x': 1.45, 'y': -0.75, 'yaw': -90.0}
        )
        self.ramp_return_distance = float(
            self._mission_param('ramp_return_distance', 1.3)
        )
        self.ramp_return_speed = float(
            self._mission_param('ramp_return_speed', 0.25)
        )
        self.ramp_return_timeout = float(
            self._mission_param('ramp_return_timeout', 15.0)
        )
        self.ramp_alignment_timeout = float(
            self._mission_param('ramp_alignment_timeout', 20.0)
        )

        self.mission_budget = MissionTimeBudget(
            total_seconds=self.mission_timeout_seconds,
            return_reserve_seconds=self.return_reserve_seconds,
            announcement_reserve_seconds=self.announcement_reserve_seconds,
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
        self._service_worker_lock = threading.Lock()
        self._pending_service_workers = set()
        self._moon_result_event = threading.Event()
        self._current_moon_request = None
        self._current_moon_result = None
        self._active_stage_deadline = None
        self._active_stage_name = None
        self._active_stage_worker = None
        self._deadline_warning_emitted = False
        self._shape_pick_prepared = False
        self.vc_sub = None
        self._yolo_unloaded_for_mission = False
        self._yolo_preloaded = False

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
        self.move_base_client = actionlib.ActionClient(
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
        rospy.Subscriber('/controller/cmd_vel', Twist, self.cmd_vel_callback)
        rospy.Subscriber(self.imu_topic, Imu, self.imu_callback)
        rospy.Subscriber('/moon_detector/result', String, self.moon_result_callback)
        rospy.Subscriber('/moon_detector/finished', Bool, self.moon_finished_callback)

        rospy.on_shutdown(self.on_shutdown)
        signal.signal(signal.SIGINT, self.shutdown)
        try:
            self._initialise_arm()
            if self.enable_voice:
                self._wait_for_voice_initialisation()
            self._wait_for_navigation_initialisation()
            self._prewarm_yolo_for_mission()
            self._prewarm_moon_detector_for_mission()
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
        self._log_startup_ready_banner()
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

    def _default_mission_reserve(self):
        if self.mission_state in (STARTING, RUNNING):
            return self.return_reserve_seconds
        if self.mission_state == RETURNING_TO_BASE:
            return self.announcement_reserve_seconds
        return 0.0

    def _effective_stage_deadline(self, deadline=None):
        deadlines = [
            value
            for value in (self._active_stage_deadline, deadline)
            if value is not None
        ]
        return min(deadlines) if deadlines else None

    def _has_pending_service_workers(self):
        with self._service_worker_lock:
            self._pending_service_workers = {
                worker
                for worker in self._pending_service_workers
                if worker.is_alive()
            }
            return bool(self._pending_service_workers)

    def _bounded_timeout(
        self,
        requested_seconds,
        reserve_seconds=None,
        deadline=None,
        enforce_mission_budget=True,
    ):
        requested = max(0.0, float(requested_seconds))
        effective_deadline = self._effective_stage_deadline(deadline)
        if not enforce_mission_budget:
            return requested
        reserve = (
            self._default_mission_reserve()
            if reserve_seconds is None
            else max(0.0, float(reserve_seconds))
        )
        allowed = self.mission_budget.bounded_timeout(
            requested,
            reserve_seconds=reserve,
            stage_deadline=effective_deadline,
        )
        if self.mission_budget.active:
            remaining = self.mission_budget.remaining()
            if (
                remaining <= self.deadline_warning_seconds
                and not self._deadline_warning_emitted
            ):
                self._deadline_warning_emitted = True
                rospy.logwarn(
                    'mission deadline warning: %.1f seconds remain before the hard stop',
                    remaining,
                )
        return allowed

    def _ensure_time_remaining(
        self,
        stage_name,
        minimum_seconds=0.1,
        reserve_seconds=None,
        deadline=None,
        enforce_mission_budget=True,
    ):
        minimum = max(0.0, float(minimum_seconds))
        if enforce_mission_budget and (self.stop_requested or rospy.is_shutdown()):
            self.safe_stop_robot()
            return False
        allowed = self._bounded_timeout(
            minimum,
            reserve_seconds=reserve_seconds,
            deadline=deadline,
            enforce_mission_budget=enforce_mission_budget,
        )
        if allowed + 1e-6 >= minimum:
            return True
        rospy.logerr(
            'time budget exhausted before %s (needed %.1fs, available %.1fs)',
            stage_name,
            minimum,
            allowed,
        )
        self.safe_stop_robot()
        return False

    def _make_local_deadline(
        self, requested_seconds, reserve_seconds=None, parent_deadline=None
    ):
        reserve = (
            self._default_mission_reserve()
            if reserve_seconds is None
            else max(0.0, float(reserve_seconds))
        )
        parent = self._effective_stage_deadline(parent_deadline)
        return self.mission_budget.make_stage_deadline(
            requested_seconds,
            reserve_seconds=reserve,
            parent_deadline=parent,
        )

    def _run_timed_stage(
        self, stage_name, timeout_seconds, operation, reserve_seconds=None
    ):
        if not self._ensure_time_remaining(
            stage_name,
            minimum_seconds=0.1,
            reserve_seconds=reserve_seconds,
        ):
            return False
        previous_name = self._active_stage_name
        previous_deadline = self._active_stage_deadline
        stage_deadline = self._make_local_deadline(
            timeout_seconds,
            reserve_seconds=reserve_seconds,
            parent_deadline=previous_deadline,
        )
        self._active_stage_name = stage_name
        self._active_stage_deadline = stage_deadline
        rospy.loginfo(
            'stage %s started with %.1f seconds available',
            stage_name,
            max(0.0, stage_deadline - time.monotonic()),
        )
        completed = threading.Event()
        outcome = {}

        def invoke():
            try:
                outcome['succeeded'] = bool(operation())
            except Exception as exc:
                outcome['error'] = exc
            finally:
                completed.set()
                if self._active_stage_worker is threading.current_thread():
                    self._active_stage_worker = None

        worker = threading.Thread(
            target=invoke,
            name='mission-stage-' + stage_name.replace(' ', '-'),
        )
        worker.daemon = True
        self._active_stage_worker = worker
        worker.start()
        wait_seconds = max(0.0, stage_deadline - time.monotonic())
        if not completed.wait(wait_seconds):
            rospy.logerr('stage %s exceeded its aggregate timeout', stage_name)
            with self._lock:
                self.stop_requested = True
            self.cancel_navigation_and_stop()
            self._active_stage_name = previous_name
            self._active_stage_deadline = previous_deadline
            return False
        self._active_stage_name = previous_name
        self._active_stage_deadline = previous_deadline
        if 'error' in outcome:
            raise outcome['error']
        return outcome.get('succeeded', False)

    def _interruptible_sleep(
        self,
        seconds,
        label='sleep',
        reserve_seconds=None,
        deadline=None,
        enforce_mission_budget=True,
        allow_stopped=False,
    ):
        duration = max(0.0, float(seconds))
        if duration <= 0.0:
            return True
        if not self._ensure_time_remaining(
            label,
            minimum_seconds=duration,
            reserve_seconds=reserve_seconds,
            deadline=deadline,
            enforce_mission_budget=enforce_mission_budget,
        ):
            return False
        end_time = time.monotonic() + duration
        while time.monotonic() < end_time:
            if (self.stop_requested and not allow_stopped) or rospy.is_shutdown():
                return False
            time.sleep(min(0.05, max(0.0, end_time - time.monotonic())))
        return (
            (allow_stopped or not self.stop_requested) and not rospy.is_shutdown()
        )

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
        if self.enable_tilt_guard:
            try:
                imu_message = rospy.wait_for_message(
                    self.imu_topic, Imu, timeout=self.initialization_timeout
                )
                self._set_tilt_reference(imu_message)
            except rospy.ROSException as exc:
                raise RuntimeError('tilt-guard IMU initialisation timed out: %s' % exc)
            with self._tilt_lock:
                if self._tilt_fault:
                    raise RuntimeError('tilt guard rejected the initial robot attitude')
                roll = self.current_roll_degrees
                pitch = self.current_pitch_degrees
                tilt = self.current_tilt_degrees
            rospy.loginfo(
                'Tilt guard armed relative to startup gravity: '
                'roll_delta=%.2f deg pitch_delta=%.2f deg tilt=%.2f deg limit=%.1f deg',
                roll,
                pitch,
                tilt,
                self.unexpected_tilt_limit_degrees,
            )

    def _prewarm_yolo_for_mission(self):
        """Load the legacy TensorRT engine before the one-shot start window opens."""
        if not self.prewarm_yolo_before_start:
            rospy.logwarn(
                'YOLOv5 prewarm is disabled; the first mission stage may include '
                'TensorRT cold-start latency'
            )
            return
        self.safe_stop_robot()
        rospy.loginfo(
            'Preloading YOLOv5 TensorRT before mission start; the robot remains stopped'
        )
        response = self.call_trigger(
            '/yolov5/start',
            timeout=self.yolo_prewarm_timeout,
            enforce_mission_budget=False,
        )
        if response is None or not response.success:
            message = response.message if response is not None else 'service timed out'
            raise RuntimeError('YOLOv5 TensorRT prewarm failed: %s' % message)
        pause_response = self.call_trigger(
            '/yolov5/stop',
            timeout=max(2.0, self.shape_stop_timeout),
            enforce_mission_budget=False,
        )
        if pause_response is None or not pause_response.success:
            raise RuntimeError('YOLOv5 prewarm completed but pause failed')
        self._yolo_preloaded = True
        self.safe_stop_robot()
        rospy.loginfo('YOLOv5 TensorRT prewarm complete')

    def _prewarm_moon_detector_for_mission(self):
        """Load the Moon CPU model before accepting the one-shot mission trigger."""
        if not self.prewarm_moon_before_start:
            rospy.logwarn(
                'Moon detector prewarm is disabled; the first scene session may '
                'include model cold-start latency'
            )
            return
        self.safe_stop_robot()
        rospy.loginfo(
            'Preloading Moon scene-card model before mission start; the robot remains stopped'
        )
        response = self.call_trigger(
            '/moon_detector/preload',
            timeout=self.moon_prewarm_timeout,
            enforce_mission_budget=False,
        )
        if response is None or not response.success:
            message = response.message if response is not None else 'service timed out'
            raise RuntimeError('Moon detector prewarm failed: %s' % message)
        self.safe_stop_robot()
        rospy.loginfo('Moon scene-card model prewarm complete')

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

    def _log_startup_ready_banner(self):
        separator = '=' * 72
        rospy.loginfo(separator)
        rospy.loginfo('系统启动完成：导航、机械臂和视觉模块已就绪，小车当前保持停车')
        if self.enable_voice:
            rospy.loginfo('请现在说“小麦小麦”唤醒，然后说“开始执行任务”')
        else:
            rospy.loginfo('语音输入已关闭，请使用启动服务或人工调试方式启动')
        if self.enable_timeout_auto_start and self.start_timeout_seconds > 0:
            rospy.loginfo(
                '%.0f 秒自动启动倒计时已开始；未收到开始指令时只启动一次',
                self.start_timeout_seconds,
            )
        else:
            rospy.loginfo('自动启动已关闭，小车将继续保持停车并等待人工启动')
        rospy.loginfo(separator)

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
            if self._has_pending_service_workers():
                rospy.logwarn('Rejected mission start while a timed-out service is still active')
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
            self.mission_budget.start()
            self._deadline_warning_emitted = False
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
        self.mission_budget.reset()
        self._active_stage_deadline = None
        self._active_stage_name = None
        self._active_stage_worker = None
        self._deadline_warning_emitted = False
        self._shape_pick_prepared = False
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
        with self._tilt_lock:
            self._controlled_ramp_motion = False
            self._tilt_violation_count = 0
            self._tilt_fault = False
            self._tilt_recovery_active = False
        self._tilt_recovery_requested.clear()
        with self._command_lock:
            self._last_motion_linear_x = 0.0
            self._last_motion_linear_y = 0.0
            self._last_motion_command_time = 0.0
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
            active = (
                self._mission_thread is not None and self._mission_thread.is_alive()
            ) or (
                self._active_stage_worker is not None
                and self._active_stage_worker.is_alive()
            ) or self._has_pending_service_workers()
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
            if not self.safe_arm_pose(wait_seconds=2.0, allow_stopped=True):
                return TriggerResponse(
                    success=False,
                    message='reset rejected because the arm did not reach its safe pose',
                )
            if not self.unload_moon_detector(enforce_mission_budget=False):
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

    def cmd_vel_callback(self, msg):
        with self._tilt_lock:
            if self._tilt_recovery_active:
                return
        if math.hypot(msg.linear.x, msg.linear.y) < 0.01:
            return
        with self._command_lock:
            self._last_motion_linear_x = float(msg.linear.x)
            self._last_motion_linear_y = float(msg.linear.y)
            self._last_motion_command_time = time.monotonic()

    def _set_tilt_reference(self, msg):
        roll, pitch = quaternion_roll_pitch_degrees(msg.orientation)
        gravity = quaternion_gravity_vector(msg.orientation)
        with self._tilt_lock:
            self.imu_received = True
            self.tilt_reference_ready = True
            self._tilt_reference_roll_degrees = roll
            self._tilt_reference_pitch_degrees = pitch
            self._tilt_reference_gravity = gravity
            self.current_roll_degrees = 0.0
            self.current_pitch_degrees = 0.0
            self.current_tilt_degrees = 0.0
            self._tilt_violation_count = 0
            self._tilt_fault = False
        self._tilt_recovery_requested.clear()
        rospy.loginfo(
            'Tilt reference captured from mounted IMU: raw_roll=%.2f raw_pitch=%.2f',
            roll,
            pitch,
        )

    def imu_callback(self, msg):
        try:
            raw_roll, raw_pitch = quaternion_roll_pitch_degrees(msg.orientation)
            gravity = quaternion_gravity_vector(msg.orientation)
        except Exception as exc:
            rospy.logwarn_throttle(5.0, 'tilt guard IMU conversion failed: %s', exc)
            return

        trip_guard = False
        with self._tilt_lock:
            self.imu_received = True
            if not self.tilt_reference_ready:
                return
            roll = wrapped_angle_delta_degrees(
                raw_roll, self._tilt_reference_roll_degrees
            )
            pitch = wrapped_angle_delta_degrees(
                raw_pitch, self._tilt_reference_pitch_degrees
            )
            tilt = vector_angle_degrees(self._tilt_reference_gravity, gravity)
            self.current_roll_degrees = roll
            self.current_pitch_degrees = pitch
            self.current_tilt_degrees = tilt
            if (
                not self.enable_tilt_guard
                or self._tilt_fault
                or self._tilt_recovery_active
                or self._tilt_recovery_requested.is_set()
            ):
                return
            limit = (
                self.controlled_ramp_tilt_limit_degrees
                if self._controlled_ramp_motion
                else self.unexpected_tilt_limit_degrees
            )
            if tilt > limit:
                self._tilt_violation_count += 1
            else:
                self._tilt_violation_count = 0
            if self._tilt_violation_count >= self.tilt_violation_samples:
                self._tilt_violation_count = 0
                self._tilt_recovery_requested.set()
                trip_guard = True

        if trip_guard:
            rospy.logwarn(
                'Unexpected tilt detected: roll_delta=%.2f deg '
                'pitch_delta=%.2f deg tilt=%.2f deg; '
                'canceling this goal and requesting reverse escape',
                roll,
                pitch,
                tilt,
            )
            try:
                self.move_base_client.cancel_all_goals()
            except Exception:
                pass

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

    def play(self, name, deadline=None):
        if not self.enable_voice:
            return True
        return self._play_audio_path(
            voice_play.get_path(name, language=self.language), deadline=deadline
        )

    def play_moon_asset(self, asset_name, deadline=None):
        if not self.enable_voice:
            return True
        path = os.path.join(self.moon_voice_dir, asset_name + '.wav')
        return self._play_audio_path(path, deadline=deadline)

    def _play_audio_path(self, path, deadline=None):
        if self.stop_requested or rospy.is_shutdown():
            return False
        if not os.path.isfile(path):
            rospy.logerr('offline voice asset is missing: %s', path)
            return False
        mixer_timeout = self._bounded_timeout(1.0, deadline=deadline)
        try:
            if mixer_timeout > 0.0:
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
                    timeout=mixer_timeout,
                )
        except (OSError, subprocess.TimeoutExpired):
            pass
        playback_timeout = self._bounded_timeout(
            self.audio_playback_timeout, deadline=deadline
        )
        if playback_timeout <= 0.0:
            rospy.logerr('offline voice playback skipped because its deadline expired')
            return False
        try:
            return (
                subprocess.call(
                    ['aplay', '-q', path],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                    timeout=playback_timeout,
                )
                == 0
            )
        except Exception as exc:
            rospy.logerr('offline voice playback failed for %s: %s', path, exc)
            return False

    def call_trigger(
        self,
        service_name,
        timeout=2.0,
        discovery_timeout=None,
        reserve_seconds=None,
        deadline=None,
        enforce_mission_budget=True,
    ):
        requested_timeout = max(0.1, float(timeout))
        timeout = self._bounded_timeout(
            requested_timeout,
            reserve_seconds=reserve_seconds,
            deadline=deadline,
            enforce_mission_budget=enforce_mission_budget,
        )
        if (
            enforce_mission_budget
            and (self.stop_requested or rospy.is_shutdown())
        ):
            rospy.logwarn('service %s skipped after mission cancellation', service_name)
            return None
        if timeout <= 0.0:
            rospy.logwarn('service %s skipped because its deadline expired', service_name)
            return None
        discovery_requested = (
            self.service_discovery_timeout
            if discovery_timeout is None
            else max(0.1, float(discovery_timeout))
        )
        discovery_allowed = min(timeout, discovery_requested)
        started_at = time.monotonic()
        try:
            rospy.wait_for_service(service_name, timeout=discovery_allowed)
        except Exception as exc:
            rospy.logwarn('service %s unavailable: %s', service_name, exc)
            return None

        remaining = timeout - (time.monotonic() - started_at)
        if remaining <= 0.0:
            rospy.logwarn('service %s discovery exhausted its %.1fs timeout', service_name, timeout)
            return None

        completed = threading.Event()
        abandoned = threading.Event()
        cleanup_lock = threading.Lock()
        cleanup_started = [False]
        outcome = {}

        def neutralise_late_response(response):
            if not getattr(response, 'success', False):
                return
            with cleanup_lock:
                if cleanup_started[0]:
                    return
                cleanup_started[0] = True
            cleanup_service = {
                '/moon_detector/start': '/moon_detector/stop',
                '/ramp/start': '/ramp/stop',
                '/ramp/up': '/ramp/stop',
                '/shape_recognition/start': '/shape_recognition/stop',
                '/shape_recognition/pick': '/shape_recognition/stop',
                '/position_correction/start': '/position_correction/stop',
                '/position_correction/pick_1': '/position_correction/stop',
                '/position_correction/pick_2': '/position_correction/stop',
                '/position_correction/pick_3': '/position_correction/stop',
                '/position_correction/place_1': '/position_correction/stop',
                '/position_correction/place_2': '/position_correction/stop',
                '/position_correction/place_3': '/position_correction/stop',
                '/yolov5/start': '/yolov5/stop',
            }.get(service_name)
            if not cleanup_service:
                return
            try:
                self.call_trigger(
                    cleanup_service,
                    timeout=1.0,
                    discovery_timeout=0.5,
                    enforce_mission_budget=False,
                )
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
                with self._service_worker_lock:
                    self._pending_service_workers.discard(
                        threading.current_thread()
                    )
                completed.set()

        worker = threading.Thread(target=invoke, name='trigger-' + service_name.strip('/').replace('/', '-'))
        worker.daemon = True
        with self._service_worker_lock:
            self._pending_service_workers.add(worker)
        worker.start()
        if not completed.wait(remaining):
            abandoned.set()
            if 'response' in outcome:
                def cleanup_late_response():
                    try:
                        neutralise_late_response(outcome['response'])
                    finally:
                        with self._service_worker_lock:
                            self._pending_service_workers.discard(
                                threading.current_thread()
                            )

                cleanup_worker = threading.Thread(
                    target=cleanup_late_response,
                    name='cleanup-' + service_name.strip('/').replace('/', '-'),
                )
                cleanup_worker.daemon = True
                with self._service_worker_lock:
                    self._pending_service_workers.add(cleanup_worker)
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
        self.stop_moon_detector(enforce_mission_budget=False)
        self.unload_moon_detector(enforce_mission_budget=False)
        if self.slope_surface:
            self.call_trigger(
                '/ramp/stop', timeout=1.0, enforce_mission_budget=False
            )
        self.call_trigger(
            '/position_correction/stop',
            timeout=1.0,
            enforce_mission_budget=False,
        )
        self.call_trigger(
            '/shape_recognition/stop',
            timeout=1.0,
            enforce_mission_budget=False,
        )
        self.call_trigger(
            '/yolov5/stop', timeout=1.0, enforce_mission_budget=False
        )
        self._unload_yolo_for_mission()
        self.cancel_navigation_and_stop()

    def _drive_for(
        self,
        linear_x=0.0,
        linear_y=0.0,
        angular_z=0.0,
        duration=0.0,
        deadline=None,
    ):
        duration = max(0.0, float(duration))
        if not self._ensure_time_remaining(
            'timed chassis motion', minimum_seconds=duration, deadline=deadline
        ):
            return False
        twist = Twist()
        twist.linear.x = linear_x
        twist.linear.y = linear_y
        twist.angular.z = angular_z
        motion_deadline = time.monotonic() + duration
        try:
            while not rospy.is_shutdown() and time.monotonic() < motion_deadline:
                if self.stop_requested:
                    return False
                if self._tilt_recovery_requested.is_set():
                    with self._tilt_lock:
                        controlled_ramp_motion = self._controlled_ramp_motion
                    recovered = self._recover_from_unexpected_tilt()
                    return recovered and not controlled_ramp_motion
                self.mecanum_pub.publish(twist)
                time.sleep(min(0.05, max(0.0, motion_deadline - time.monotonic())))
            return not self.stop_requested and not rospy.is_shutdown()
        finally:
            self.safe_stop_robot()

    def safe_arm_pose(self, wait_seconds=1.0, allow_stopped=False):
        command_duration = 2.0
        total_wait = max(command_duration, float(wait_seconds))
        if not self._ensure_time_remaining(
            'safe arm pose',
            minimum_seconds=total_wait,
            enforce_mission_budget=not allow_stopped,
        ):
            self.mechanical_arm_safe = False
            return False
        try:
            bus_servo_control.set_servos(
                self.joints_pub,
                command_duration,
                ((1, 500), (2, 760), (3, 15), (4, 150), (5, 500), (10, 200)),
            )
            if not self._interruptible_sleep(
                total_wait,
                label='safe arm settling',
                enforce_mission_budget=not allow_stopped,
                allow_stopped=allow_stopped,
            ):
                self.mechanical_arm_safe = False
                return False
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
        if self.stop_requested or rospy.is_shutdown():
            self.safe_stop_robot()
            return False
        # Navigation has no independent path timeout. The current task-stage
        # deadline and the global mission budget remain the hard safety bounds.
        requested_timeout = float('inf') if timeout is None else float(timeout)
        timeout = self._bounded_timeout(requested_timeout)
        if timeout <= 0.0:
            rospy.logerr('navigation skipped because the task deadline expired')
            self.safe_stop_robot()
            return False
        navigation_deadline = time.monotonic() + timeout
        self.safe_stop_robot()
        server_timeout = min(
            self.move_base_server_timeout,
            max(0.0, navigation_deadline - time.monotonic()),
        )
        if server_timeout <= 0.0:
            return False
        if not self.move_base_client.wait_for_server(
            rospy.Duration(server_timeout)
        ):
            rospy.logerr('move_base action server is unavailable')
            return False
        recovery_attempts = 0
        while not rospy.is_shutdown() and time.monotonic() < navigation_deadline:
            if self._tilt_recovery_requested.is_set():
                if recovery_attempts >= self.tilt_recovery_max_attempts:
                    rospy.logerr('navigation exceeded its tilt-recovery retry limit')
                    return False
                if not self._recover_from_unexpected_tilt():
                    return False
                recovery_attempts += 1

            goal = self.make_move_base_goal(x, y, yaw_degrees)
            goal_handle = self.move_base_client.send_goal(goal)
            retry_after_recovery = False
            while not rospy.is_shutdown() and time.monotonic() < navigation_deadline:
                if self.stop_requested:
                    goal_handle.cancel()
                    self.safe_stop_robot()
                    return False
                if self._tilt_recovery_requested.is_set():
                    goal_handle.cancel()
                    if recovery_attempts >= self.tilt_recovery_max_attempts:
                        rospy.logerr('navigation exceeded its tilt-recovery retry limit')
                        return False
                    if not self._recover_from_unexpected_tilt():
                        return False
                    recovery_attempts += 1
                    retry_after_recovery = True
                    break
                if goal_handle.get_comm_state() == actionlib.CommState.DONE:
                    state = goal_handle.get_goal_status()
                    if state == GoalStatus.SUCCEEDED:
                        self.safe_stop_robot()
                        return True
                    rospy.logerr(
                        'navigation failed with move_base state %s: %s',
                        state,
                        goal_handle.get_goal_status_text(),
                    )
                    self.safe_stop_robot()
                    return False
                time.sleep(0.1)
            if retry_after_recovery:
                rospy.logwarn(
                    'Resending navigation goal after reverse escape '
                    '(attempt %d/%d)',
                    recovery_attempts,
                    self.tilt_recovery_max_attempts,
                )
                continue
            goal_handle.cancel()
            break
        self.safe_stop_robot()
        rospy.logerr(
            'navigation stopped when the current task deadline expired after %.1f seconds',
            timeout,
        )
        return False

    def wait_nav_status(self, timeout=30.0, deadline=None):
        timeout = self._bounded_timeout(timeout, deadline=deadline)
        end_time = time.monotonic() + timeout
        while not rospy.is_shutdown() and not self.stop_requested:
            if self.move_base_status == 3:
                self.move_base_status = 1
                return True
            if self.move_base_status == -1:
                self.move_base_status = 1
                self.safe_stop_robot()
                return False
            if time.monotonic() >= end_time:
                self.move_base_status = 1
                self.safe_stop_robot()
                return False
            time.sleep(0.1)
        return False

    def wait_correction_status(self, timeout=None, deadline=None):
        requested = self.correction_status_timeout if timeout is None else timeout
        timeout = self._bounded_timeout(requested, deadline=deadline)
        end_time = time.monotonic() + timeout
        while not rospy.is_shutdown() and not self.stop_requested:
            if rospy.get_param('/position_correction/status', 'stop') == 'stop':
                return True
            if time.monotonic() >= end_time:
                self.safe_stop_robot()
                return False
            time.sleep(0.2)
        return False

    def wait_ramp_status(
        self, timeout=None, require_active_transition=True, deadline=None
    ):
        requested = self.ramp_alignment_timeout if timeout is None else timeout
        timeout = self._bounded_timeout(requested, deadline=deadline)
        end_time = time.monotonic() + timeout
        active_seen = not require_active_transition
        while not rospy.is_shutdown() and not self.stop_requested:
            status = rospy.get_param('/ramp/status', 'stop')
            if status != 'stop':
                active_seen = True
            elif active_seen:
                return True
            if time.monotonic() >= end_time:
                self.safe_stop_robot()
                return False
            time.sleep(0.2)
        return False

    def run_ramp_alignment(self):
        if self.stop_requested or rospy.is_shutdown():
            return False
        deadline = self._make_local_deadline(self.ramp_alignment_timeout)
        start_response = self.call_trigger(
            '/ramp/start', timeout=5.0, deadline=deadline
        )
        if start_response is None or not start_response.success:
            return False
        if self.stop_requested or rospy.is_shutdown():
            self.call_trigger(
                '/ramp/stop', timeout=1.0, enforce_mission_budget=False
            )
            return False
        try:
            up_response = self.call_trigger(
                '/ramp/up', timeout=5.0, deadline=deadline
            )
            if up_response is None or not up_response.success:
                return False
            if self.stop_requested or rospy.is_shutdown():
                return False
            return self.wait_ramp_status(
                require_active_transition=True, deadline=deadline
            )
        finally:
            self.call_trigger(
                '/ramp/stop', timeout=1.0, enforce_mission_budget=False
            )

    def wait_pick_status(self, timeout=None, deadline=None):
        requested = self.shape_pick_timeout if timeout is None else timeout
        timeout = self._bounded_timeout(requested, deadline=deadline)
        end_time = time.monotonic() + timeout
        active_seen = False
        while not rospy.is_shutdown() and not self.stop_requested:
            status = rospy.get_param('/shape_recognition/status', 'start')
            if status in ('start', 'active', 'pick'):
                active_seen = True
            elif status == 'stop' and active_seen:
                return True
            elif status in ('timeout', 'error'):
                self.safe_stop_robot()
                return False
            if time.monotonic() >= end_time:
                self.safe_stop_robot()
                return False
            time.sleep(0.2)
        return False

    def wait_yolo_status(self, timeout=None, deadline=None):
        requested = self.yolo_detection_timeout if timeout is None else timeout
        timeout = self._bounded_timeout(requested, deadline=deadline)
        end_time = time.monotonic() + timeout
        while not rospy.is_shutdown() and not self.stop_requested:
            shape = rospy.get_param('/yolov5/shape', 'None')
            if shape != 'None':
                return shape
            if time.monotonic() >= end_time:
                return None
            time.sleep(0.2)
        return None

    def _set_controlled_ramp_motion(self, enabled):
        with self._tilt_lock:
            self._controlled_ramp_motion = bool(enabled)
            self._tilt_violation_count = 0
            if enabled:
                self._tilt_fault = False
        if enabled:
            self._tilt_recovery_requested.clear()

    def _tilt_snapshot(self):
        with self._tilt_lock:
            return (
                self.imu_received,
                self.current_roll_degrees,
                self.current_pitch_degrees,
                self.current_tilt_degrees,
                self._tilt_fault,
            )

    def _opposite_tilt_escape_twist(self):
        speed = max(0.05, self.tilt_recovery_speed)
        with self._command_lock:
            command_age = time.monotonic() - self._last_motion_command_time
            linear_x = self._last_motion_linear_x
            linear_y = self._last_motion_linear_y

        magnitude = math.hypot(linear_x, linear_y)
        if command_age <= self.tilt_command_max_age and magnitude >= 0.01:
            escape_x = -linear_x / magnitude * speed
            escape_y = -linear_y / magnitude * speed
            source = 'opposite of the latest chassis command'
        else:
            yaw = (
                quaternion_yaw_radians(self.current_pose.orientation)
                if self.current_pose_received
                else 0.0
            )
            # Negative map Y is the verified flat-side escape corridor beside the ramp.
            escape_x = -math.sin(yaw) * speed
            escape_y = -math.cos(yaw) * speed
            source = 'negative-map-Y fallback'

        twist = Twist()
        twist.linear.x = escape_x
        twist.linear.y = escape_y
        return twist, source

    def _recover_from_unexpected_tilt(self):
        timeout = self._bounded_timeout(self.tilt_recovery_timeout)
        if timeout <= 0.0 or self.stop_requested or rospy.is_shutdown():
            return False
        with self._tilt_lock:
            if self._tilt_recovery_active:
                return False
            self._tilt_recovery_active = True
            self._tilt_violation_count = 0
            self._tilt_fault = False
        self._tilt_recovery_requested.clear()
        try:
            self.move_base_client.cancel_all_goals()
        except Exception:
            pass

        twist, source = self._opposite_tilt_escape_twist()
        rospy.logwarn(
            'Tilt recovery moving away at x=%.3f y=%.3f using %s; mission remains active',
            twist.linear.x,
            twist.linear.y,
            source,
        )
        deadline = time.monotonic() + timeout
        minimum_end = time.monotonic() + min(
            timeout, max(0.0, self.tilt_recovery_min_duration)
        )
        level_samples = 0
        try:
            while not rospy.is_shutdown() and not self.stop_requested:
                imu_received, roll, pitch, tilt, _tilt_fault = self._tilt_snapshot()
                if self.enable_tilt_guard and not imu_received:
                    rospy.logerr('tilt recovery lost IMU data')
                    break
                if (
                    time.monotonic() >= minimum_end
                    and tilt <= self.tilt_recovery_level_degrees
                ):
                    level_samples += 1
                    if level_samples >= self.tilt_recovery_level_samples:
                        rospy.loginfo(
                            'Tilt recovery cleared the ramp edge: '
                            'roll_delta=%.2f pitch_delta=%.2f tilt=%.2f; '
                            'retrying navigation',
                            roll,
                            pitch,
                            tilt,
                        )
                        with self._tilt_lock:
                            self._tilt_fault = False
                        return True
                else:
                    level_samples = 0
                if time.monotonic() >= deadline:
                    break
                self.mecanum_pub.publish(twist)
                time.sleep(0.05)
            with self._tilt_lock:
                self._tilt_fault = True
            rospy.logerr(
                'tilt recovery could not return the chassis to level within %.1f seconds',
                timeout,
            )
            return False
        finally:
            self.safe_stop_robot()
            self._tilt_recovery_requested.clear()
            with self._tilt_lock:
                self._tilt_recovery_active = False
                self._tilt_violation_count = 0

    def reverse_up_ramp_with_laser(self, distance, speed, timeout):
        if not self.current_pose_received:
            rospy.logerr('cannot confirm ramp return without odometry')
            return False
        try:
            self.move_base_client.cancel_all_goals()
        except Exception:
            pass
        imu_received, _roll, _pitch, _tilt, tilt_fault = self._tilt_snapshot()
        if self.enable_tilt_guard and (not imu_received or tilt_fault):
            rospy.logerr('cannot start controlled ramp return without a healthy IMU')
            return False
        start_x = self.current_pose.position.x
        start_y = self.current_pose.position.y
        timeout = self._bounded_timeout(timeout)
        if timeout <= 0.0:
            self.safe_stop_robot()
            return False
        end_time = time.monotonic() + timeout
        traveled = 0.0
        self._set_controlled_ramp_motion(True)
        try:
            while not rospy.is_shutdown() and not self.stop_requested:
                if self._tilt_recovery_requested.is_set():
                    self._recover_from_unexpected_tilt()
                    return False
                dx = self.current_pose.position.x - start_x
                dy = self.current_pose.position.y - start_y
                traveled = math.sqrt(dx * dx + dy * dy)
                if traveled >= distance:
                    return True
                if time.monotonic() >= end_time:
                    rospy.logerr(
                        'ramp return timed out at %.3f of %.3f metres', traveled, distance
                    )
                    return False
                twist = Twist()
                twist.linear.x = -abs(speed)
                error = self.left_rear_dist - self.right_rear_dist
                twist.angular.z = max(-0.5, min(0.5, 0.6 * error))
                self.mecanum_pub.publish(twist)
                time.sleep(0.05)
            return False
        finally:
            self.safe_stop_robot()
            self._set_controlled_ramp_motion(False)

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
            'class_id': -1,
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

    def _rotate_for_moon_scene(
        self, task_point_index, reverse=False, deadline=None
    ):
        direction = 'right' if reverse else 'left'
        angular_z = self.moon_rotation_speed if reverse else -self.moon_rotation_speed
        rospy.loginfo(
            'Moon task point %d: rotate %s %.1f degrees',
            task_point_index,
            direction,
            self.moon_rotation_degrees,
        )
        return self._drive_for(
            angular_z=angular_z,
            duration=self.moon_rotation_duration,
            deadline=deadline,
        )

    def detect_moon_scene_at_task_point(
        self, task_point_index, rotate=False, reverse_rotate=False
    ):
        if task_point_index not in (1, 2, 3):
            raise ValueError('task_point_index must be 1, 2, or 3')
        session_id = uuid.uuid4().hex
        session_deadline = self._make_local_deadline(self.moon_session_timeout)
        reverse_cleanup_reserve = (
            self.moon_rotation_duration + 1.5
            if rotate and reverse_rotate
            else 0.0
        )
        detector_deadline = session_deadline - reverse_cleanup_reserve
        self.safe_stop_robot()
        if not self._ensure_time_remaining(
            'Moon scene session', minimum_seconds=0.1, deadline=detector_deadline
        ):
            result = self._moon_failure_result(
                task_point_index,
                session_id,
                'timeout',
                'mission or Moon-session deadline expired before detection',
            )
            self._store_moon_result(task_point_index, result)
            return result

        rotated = False
        if rotate:
            if not self._rotate_for_moon_scene(
                task_point_index, reverse=False, deadline=detector_deadline
            ):
                result = self._moon_failure_result(
                    task_point_index, session_id, 'stopped', 'rotation interrupted'
                )
                self._store_moon_result(task_point_index, result)
                return result
            rotated = True

        result = None
        detector_touched = False
        try:
            if not self._unload_yolo_for_mission(deadline=detector_deadline):
                result = self._moon_failure_result(
                    task_point_index,
                    session_id,
                    'detector_unavailable',
                    'existing mineral-card YOLO could not release its GPU context',
                )
            else:
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

                detector_touched = True
                reset_response = self.call_trigger(
                    '/moon_detector/reset', timeout=3.0, deadline=detector_deadline
                )
                if reset_response is None or not reset_response.success:
                    result = self._moon_failure_result(
                        task_point_index,
                        session_id,
                        'detector_unavailable',
                        'reset failed',
                    )
                else:
                    rospy.set_param(
                        '/moon_detector/request/task_point_index', task_point_index
                    )
                    rospy.set_param('/moon_detector/request/session_id', session_id)
                    rospy.set_param(
                        '/moon_detector/request/started_at_ros', started_at_ros
                    )
                    start_response = self.call_trigger(
                        '/moon_detector/start',
                        timeout=self.moon_start_timeout,
                        deadline=detector_deadline,
                    )
                    if start_response is None or not start_response.success:
                        result = self._moon_failure_result(
                            task_point_index,
                            session_id,
                            'model_error',
                            start_response.message
                            if start_response is not None
                            else 'start failed',
                        )
                    else:
                        detection_timeout = self._bounded_timeout(
                            self.moon_detection_timeout, deadline=detector_deadline
                        )
                        detection_deadline = time.monotonic() + detection_timeout
                        while (
                            not rospy.is_shutdown()
                            and time.monotonic() < detection_deadline
                        ):
                            if self.stop_requested or self._moon_result_event.is_set():
                                break
                            self._moon_result_event.wait(
                                min(
                                    0.05,
                                    max(
                                        0.0,
                                        detection_deadline - time.monotonic(),
                                    ),
                                )
                            )
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
            if detector_touched:
                self.stop_moon_detector(enforce_mission_budget=False)
            with self._moon_result_lock:
                self._current_moon_request = None
                self._current_moon_result = None
                self._moon_result_event.clear()
            if (
                rotated
                and reverse_rotate
                and not self.stop_requested
                and not rospy.is_shutdown()
            ):
                self._rotate_for_moon_scene(
                    task_point_index, reverse=True, deadline=session_deadline
                )
            self._restart_yolo(deadline=session_deadline)
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

    def stop_moon_detector(self, enforce_mission_budget=True):
        self.call_trigger(
            '/moon_detector/stop',
            timeout=1.0,
            enforce_mission_budget=enforce_mission_budget,
        )

    def unload_moon_detector(self, deadline=None, enforce_mission_budget=True):
        response = self.call_trigger(
            '/moon_detector/unload',
            timeout=self.moon_unload_timeout,
            deadline=deadline,
            enforce_mission_budget=enforce_mission_budget,
        )
        return response is not None and response.success

    def _prepare_shape_pick(self, deadline=None):
        self.safe_stop_robot()
        if self._shape_pick_prepared:
            return True
        if not self._yolo_unloaded_for_mission:
            self.call_trigger('/yolov5/stop', timeout=2.0, deadline=deadline)
        try:
            start_response = self.call_trigger(
                '/shape_recognition/start',
                timeout=self.shape_start_timeout,
                deadline=deadline,
            )
            if start_response is None or not start_response.success:
                self.call_trigger(
                    '/shape_recognition/stop',
                    timeout=self.shape_stop_timeout,
                    enforce_mission_budget=False,
                )
                self._restart_yolo(deadline=deadline)
                return False
            self._shape_pick_prepared = True
            return True
        except Exception as exc:
            rospy.logerr('shape-recognition prewarm failed: %s', exc)
            self.safe_stop_robot()
            self.call_trigger(
                '/shape_recognition/stop',
                timeout=self.shape_stop_timeout,
                enforce_mission_budget=False,
            )
            self._restart_yolo(deadline=deadline)
            return False

    def _finish_shape_pick(self, deadline=None):
        stop_response = self.call_trigger(
            '/shape_recognition/stop',
            timeout=self.shape_stop_timeout,
            enforce_mission_budget=False,
        )
        self._shape_pick_prepared = False
        self._restart_yolo(deadline=deadline)
        return stop_response is not None and stop_response.success

    def safe_pick(self, prepared=False, deadline=None):
        pick_deadline = self._make_local_deadline(
            self.shape_pick_stage_timeout, parent_deadline=deadline
        )
        prepared_here = False
        success = False
        cleanup_succeeded = True
        try:
            if prepared:
                if not self._shape_pick_prepared:
                    rospy.logerr('prepared pick requested without an active prewarm session')
                    success = False
                else:
                    success = True
            else:
                if not self._prepare_shape_pick(deadline=pick_deadline):
                    success = False
                else:
                    prepared_here = True
                    success = self._interruptible_sleep(
                        self.shape_warmup_seconds,
                        label='shape-recognition warmup',
                        deadline=pick_deadline,
                    )
            if success:
                self.safe_stop_robot()
                pick_response = self.call_trigger(
                    '/shape_recognition/pick',
                    timeout=self.shape_pick_service_timeout,
                    deadline=pick_deadline,
                )
                if pick_response is not None and pick_response.success:
                    success = self.wait_pick_status(
                        timeout=self.shape_pick_timeout, deadline=pick_deadline
                    )
                else:
                    success = False
        except Exception as exc:
            rospy.logerr('pick task failed: %s', exc)
            self.safe_stop_robot()
            success = False
        finally:
            if prepared_here or prepared or self._shape_pick_prepared:
                cleanup_succeeded = self._finish_shape_pick(deadline=pick_deadline)
        return success and cleanup_succeeded

    def _restart_yolo(self, deadline=None):
        with self._gpu_lifecycle_lock:
            if (
                self._yolo_unloaded_for_mission
                or self.stop_requested
                or rospy.is_shutdown()
                or self.mission_state in (STOPPED, ERROR, COMPLETED)
            ):
                return False
            response = self.call_trigger(
                '/yolov5/start',
                timeout=self.yolo_start_timeout,
                deadline=deadline,
            )
            return response is not None and response.success

    def _unload_yolo_for_mission(self, deadline=None):
        with self._gpu_lifecycle_lock:
            if self._yolo_unloaded_for_mission:
                return True
            response = self.call_trigger(
                '/yolov5/unload',
                timeout=self.yolo_unload_timeout,
                deadline=deadline,
            )
            if response is None or not response.success:
                return False
            self._yolo_unloaded_for_mission = True
            return True

    def _navigate_ramp_clearance(self, pose, label):
        if not self.enable_ramp_clearance_waypoints:
            return True
        rospy.loginfo(
            'Ramp-clearance route %s via x=%.2f y=%.2f yaw=%.1f',
            label,
            pose['x'],
            pose['y'],
            pose['yaw'],
        )
        return self.navigate_and_wait(pose['x'], pose['y'], pose['yaw'])

    def control(self, x, y, yaw, set_status):
        if self.stop_requested or rospy.is_shutdown():
            return False
        if set_status == 'detect':
            if not self._navigate_ramp_clearance(
                self.departure_ramp_clearance_pose, 'after initial turn'
            ):
                return False
        elif set_status == 'place':
            if not self._navigate_ramp_clearance(
                self.place_ramp_clearance_pose, 'before placement'
            ):
                return False
        elif set_status == 'pick2':
            if not self._navigate_ramp_clearance(
                self.pick_ramp_clearance_pose, 'leaving placement for second pickup'
            ):
                return False
        if set_status != 'pick2':
            if not self.navigate_and_wait(x, y, yaw):
                return False

        if set_status == 'pick1':
            if not self.navigate_and_wait(
                1.30, -3.12, 0.0
            ):
                return False
            pick_deadline = self._make_local_deadline(
                self.shape_pick_stage_timeout
            )
            if not self._prepare_shape_pick(deadline=pick_deadline):
                return False
            try:
                if not self._drive_for(
                    linear_x=0.108, duration=2.0, deadline=pick_deadline
                ):
                    return False
                if not self.safe_pick(prepared=True, deadline=pick_deadline):
                    return False
            finally:
                if self._shape_pick_prepared:
                    self._finish_shape_pick(deadline=pick_deadline)
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
            pick_deadline = self._make_local_deadline(
                self.shape_pick_stage_timeout
            )
            if not self._prepare_shape_pick(deadline=pick_deadline):
                return False
            try:
                if not self._drive_for(
                    linear_x=0.24, duration=2.0, deadline=pick_deadline
                ):
                    return False
                if not self.safe_pick(prepared=True, deadline=pick_deadline):
                    return False
            finally:
                if self._shape_pick_prepared:
                    self._finish_shape_pick(deadline=pick_deadline)
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
            # place_3 is synchronous: its response arrives after the arm has released
            # the object and returned, so a shared stale status parameter is not waited.
            self.play('9')
            return self._drive_for(linear_x=-0.2, duration=2.0)

        if set_status == 'detect':
            self.play('reached_explosion-proof_warehouse' if not self.slope_surface else '2')
            if not self._interruptible_sleep(1.0, label='mineral detector settling'):
                return False
            shape = self.wait_yolo_status(timeout=self.yolo_detection_timeout)
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
            return self._interruptible_sleep(
                2.0, label='post-detection camera settling'
            )

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
            self.base_pose['x'],
            self.base_pose['y'],
            self.base_pose['yaw'],
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
        announcement_deadline = self._make_local_deadline(
            self.announcement_stage_timeout, reserve_seconds=0.0
        )
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
                        'point_%d_prefix' % task_point_index,
                        deadline=announcement_deadline,
                    )
                    if self.stop_requested or rospy.is_shutdown():
                        return False
                    class_ok = self.play_moon_asset(
                        'class_' + class_name_en, deadline=announcement_deadline
                    )
                    playback_succeeded = prefix_ok and class_ok and playback_succeeded
                    if self.stop_requested or rospy.is_shutdown():
                        return False
                    continue
            rospy.loginfo(
                '%s\u672a\u8bc6\u522b\u5230\u6709\u6548\u573a\u666f\u5143\u7d20',
                TASK_POINT_NAME[task_point_index],
            )
            playback_succeeded = (
                self.play_moon_asset(
                    'point_%d_failure' % task_point_index,
                    deadline=announcement_deadline,
                )
                and playback_succeeded
            )
            if self.stop_requested or rospy.is_shutdown():
                return False
        if not playback_succeeded:
            rospy.logerr(
                'one or more result announcements failed at the base; '
                'keeping the completed mission stopped instead of re-entering ERROR'
            )
        if not self.mark_announced():
            return False
        self.save_mission_results()
        # The 2026 rules separately require this exact base-completion phrase.
        if not self.play('mission_completed', deadline=announcement_deadline):
            rospy.logerr(
                'mission completion voice playback failed after result announcements; '
                'mission remains complete and the robot stays stopped'
            )
        return True

    def _run_initial_departure(self):
        if not self._restart_yolo():
            return False
        self.play('1')
        self._set_controlled_ramp_motion(True)
        try:
            if not self._drive_for(linear_x=0.3, duration=3.0):
                return False
            return self._drive_for(angular_z=-0.5, duration=3.0)
        finally:
            self._set_controlled_ramp_motion(False)

    def run_mission_once(self):
        try:
            if not self.mark_running():
                return
            self.save_mission_results()
            if not self._run_timed_stage(
                'initial departure',
                self.initial_stage_timeout,
                self._run_initial_departure,
            ):
                raise MissionAbort('initial departure timed out or failed')

            if not self._run_timed_stage(
                'resource-library detection',
                self.detect_stage_timeout,
                lambda: self.control(1.5, -0.15, 90.0, 'detect'),
            ):
                raise MissionAbort('resource-library detect stage failed')
            if not self._run_timed_stage(
                'first pickup',
                self.pick_stage_timeout,
                lambda: self.control(0.94, -3.124, 0.0, 'pick1'),
            ):
                raise MissionAbort('first pick stage failed')
            self.completed_pick_tasks = 1
            if not self._run_timed_stage(
                'first placement',
                self.place_stage_timeout,
                lambda: self.control(1.24, -0.22, 40.0, 'place'),
            ):
                raise MissionAbort('first place stage failed')
            self.completed_place_tasks = 1
            if not self._run_timed_stage(
                'second pickup',
                self.pick_stage_timeout,
                lambda: self.control(0.94, -3.124, -180.0, 'pick2'),
            ):
                raise MissionAbort('second pick stage failed')
            self.completed_pick_tasks = 2
            if not self._run_timed_stage(
                'second placement',
                self.place_stage_timeout,
                lambda: self.control(1.24, -0.22, 40.0, 'place'),
            ):
                raise MissionAbort('second place stage failed')
            self.completed_place_tasks = 2
            self.navigation_tasks_finished = True

            if not self._run_timed_stage(
                'post-recognition tasks',
                self.remaining_tasks_timeout,
                self._prepare_remaining_tasks,
            ):
                raise MissionAbort('post-recognition competition tasks failed')
            if not self._body_tasks_complete():
                raise MissionAbort('competition body-task completion flags are incomplete')
            if not self.mark_all_tasks_finished():
                raise MissionAbort('failed to mark all competition tasks finished')
            self.save_mission_results()

            if not self.can_begin_return(ros_shutdown=rospy.is_shutdown()):
                raise MissionAbort('return gate rejected the mission state')
            if not self._run_timed_stage(
                'return to base',
                self.return_stage_timeout,
                self.begin_return_to_base,
                reserve_seconds=self.announcement_reserve_seconds,
            ):
                raise MissionAbort('return to base failed')
            if not self._run_timed_stage(
                'announce results',
                self.announcement_stage_timeout,
                self.announce_all_task_results_at_base,
                reserve_seconds=0.0,
            ):
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
                'time_budget': self.mission_budget.snapshot(),
                'active_stage': self._active_stage_name,
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
