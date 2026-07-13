#!/usr/bin/env python3
import ast
import pathlib
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]
CONTROLLER_PATH = ROOT / 'scripts' / 'navigation_transport' / 'voice_control_navigation.py'
DETECTOR_PATH = ROOT / 'scripts' / 'navigation_transport' / 'moon_detector.py'
YOLO_NODE_PATH = (
    ROOT / 'scripts' / 'navigation_transport' / 'yolo5' / 'yolov5_node.py'
)
YOLO_TRT_PATH = (
    ROOT / 'scripts' / 'navigation_transport' / 'yolo5' / 'yolov5_trt_6_2.py'
)
RAMP_PATH = ROOT / 'scripts' / 'navigation_transport' / 'ramp' / 'ramp.py'
POSITION_PATH = (
    ROOT / 'scripts' / 'navigation_transport' / 'calibration_position' / 'automatic_pick.py'
)
SHAPE_PATH = (
    ROOT / 'scripts' / 'navigation_transport' / 'shape_recognition' / 'shape_recognition_down.py'
)
LAUNCH_PATH = ROOT / 'scripts' / 'navigation_transport' / 'position_correction_pick.launch'
VENDORED_YOLO_ROOT = ROOT / 'third_party' / 'yolov5'


def load_source(path):
    return path.read_text(encoding='utf-8')


def class_methods(path, class_name):
    tree = ast.parse(load_source(path), filename=str(path))
    for node in tree.body:
        if isinstance(node, ast.ClassDef) and node.name == class_name:
            return {item.name: item for item in node.body if isinstance(item, ast.FunctionDef)}
    raise AssertionError('class %s not found' % class_name)


def called_attributes(function_node):
    names = []
    for node in ast.walk(function_node):
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute):
            names.append(node.func.attr)
    return names


class ControllerContractTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source = load_source(CONTROLLER_PATH)
        cls.methods = class_methods(CONTROLLER_PATH, 'VoiceControlNavNode')

    def test_timeout_uses_one_shot_rospy_timer(self):
        method_source = ast.get_source_segment(
            self.source, self.methods['schedule_start_timeout']
        )
        self.assertIn('rospy.Timer', method_source)
        self.assertIn('oneshot=True', method_source)

    def test_voice_enable_parameter_gates_input_and_playback(self):
        self.assertIn("self.enable_voice = bool(self._mission_param('enable_voice', True))", self.source)
        init_source = ast.get_source_segment(
            self.source, self.methods['__init__']
        )
        self.assertIn('if self.enable_voice:', init_source)
        self.assertIn("'/asr_node/voice_words'", init_source)
        for method_name in ('play', 'play_moon_asset'):
            method_source = ast.get_source_segment(self.source, self.methods[method_name])
            self.assertIn('if not self.enable_voice:', method_source)

    def test_timeout_callback_only_requests_start(self):
        calls = called_attributes(self.methods['start_timeout_callback'])
        self.assertEqual(['request_mission_start'], calls)

    def test_voice_callback_uses_unified_start_entry(self):
        calls = called_attributes(self.methods['words_callback'])
        self.assertIn('request_mission_start', calls)
        self.assertNotIn('run_mission_once', calls)

    def test_service_start_uses_unified_start_entry(self):
        calls = called_attributes(self.methods['start_mission_callback'])
        self.assertIn('request_mission_start', calls)
        self.assertNotIn('run_mission_once', calls)

    def test_start_stop_reset_share_controller_transition_lock(self):
        for method_name in (
            'request_mission_start',
            'stop_mission_callback',
            'reset_mission_callback',
        ):
            method_source = ast.get_source_segment(self.source, self.methods[method_name])
            self.assertIn('self._transition_lock', method_source)

    def test_manual_debug_services_use_idle_action_gate(self):
        for method_name in (
            'start_pick_callback',
            'start_place_callback',
            'start_detect_callback',
            'start_scene_card_callback',
            'start_aligning_callback',
            'start_back_callback',
        ):
            calls = called_attributes(self.methods[method_name])
            self.assertIn('_run_debug_service', calls)
        gate_source = ast.get_source_segment(
            self.source, self.methods['_run_debug_service']
        )
        self.assertIn('WAITING_FOR_START', gate_source)
        self.assertIn('cancel_start_timeout', gate_source)
        self.assertIn('schedule_start_timeout', gate_source)

    def test_scene_detection_function_never_announces_or_returns(self):
        calls = called_attributes(self.methods['detect_moon_scene_at_task_point'])
        self.assertNotIn('play', calls)
        self.assertNotIn('play_moon_asset', calls)
        self.assertNotIn('begin_return_to_base', calls)
        self.assertNotIn('mark_all_tasks_finished', calls)

    def test_third_scene_scan_precedes_original_reverse(self):
        method_source = ast.get_source_segment(self.source, self.methods['control'])
        third_scan = method_source.index('detect_moon_scene_at_task_point(3)')
        original_reverse = method_source.index(
            "return self._drive_for(linear_x=-0.2, duration=3.0)"
        )
        self.assertLess(third_scan, original_reverse)

    def test_second_place_remains_after_second_pick(self):
        method_source = ast.get_source_segment(
            self.source, self.methods['run_mission_once']
        )
        second_pick = method_source.index("'pick2'")
        second_place = method_source.index(
            "self.control(1.24, -0.22, 40.0, 'place')", second_pick
        )
        remaining_tasks = method_source.index('self._prepare_remaining_tasks()', second_place)
        mark_all = method_source.index('self.mark_all_tasks_finished()', remaining_tasks)
        begin_return = method_source.index('self.begin_return_to_base()', mark_all)
        announce = method_source.index('self.announce_all_task_results_at_base()', begin_return)
        self.assertLess(second_pick, second_place)
        self.assertLess(second_place, remaining_tasks)
        self.assertLess(remaining_tasks, mark_all)
        self.assertLess(mark_all, begin_return)
        self.assertLess(begin_return, announce)

    def test_return_requires_body_task_flags(self):
        calls = called_attributes(self.methods['begin_return_to_base'])
        self.assertIn('_body_tasks_complete', calls)
        self.assertIn('mark_returning', calls)

    def test_prepare_does_not_fake_ramp_completion(self):
        method_source = ast.get_source_segment(
            self.source, self.methods['_prepare_remaining_tasks']
        )
        self.assertNotIn("'/ramp/start'", method_source)
        self.assertNotIn('wait_ramp_status', method_source)
        traversal = method_source.index('self.reverse_up_ramp_with_laser(')
        ramp_finished = method_source.index('self.ramp_task_finished = True', traversal)
        self.assertLess(traversal, ramp_finished)

    def test_slope_task_traverses_once_before_return_is_requested(self):
        prepare_source = ast.get_source_segment(
            self.source, self.methods['_prepare_remaining_tasks']
        )
        return_source = ast.get_source_segment(
            self.source, self.methods['begin_return_to_base']
        )
        traversal = prepare_source.index('self.reverse_up_ramp_with_laser(')
        ramp_finished = prepare_source.index('self.ramp_task_finished = True', traversal)
        final_goal = return_source.index("self.base_pose['x']")
        self.assertEqual(1, self.source.count('self.reverse_up_ramp_with_laser('))
        self.assertLess(traversal, ramp_finished)
        self.assertGreaterEqual(final_goal, 0)
        self.assertNotIn('self.reverse_up_ramp_with_laser(', return_source)

    def test_optional_ramp_alignment_has_start_up_wait_stop_sequence(self):
        method_source = ast.get_source_segment(
            self.source, self.methods['run_ramp_alignment']
        )
        start = method_source.index("'/ramp/start'")
        up = method_source.index("'/ramp/up'", start)
        wait = method_source.index('self.wait_ramp_status(', up)
        stop = method_source.rindex("'/ramp/stop'")
        self.assertLess(start, up)
        self.assertLess(up, wait)
        self.assertLess(wait, stop)

    def test_base_return_uses_move_base_action_wait(self):
        method_source = ast.get_source_segment(
            self.source, self.methods['navigate_and_wait']
        )
        self.assertIn('send_goal', method_source)
        self.assertIn('wait_for_result', method_source)
        self.assertIn('GoalStatus.SUCCEEDED', method_source)

    def test_announcement_order_is_fixed(self):
        method_source = ast.get_source_segment(
            self.source, self.methods['announce_all_task_results_at_base']
        )
        self.assertIn('for task_point_index in (1, 2, 3):', method_source)
        self.assertIn('begin_announcing', method_source)
        self.assertIn('mark_announced', method_source)
        self.assertIn('self.stop_requested', method_source)
        self.assertIn('playback_succeeded', method_source)

    def test_non_slope_original_back_and_lateral_finish_are_preserved(self):
        method_source = ast.get_source_segment(
            self.source, self.methods['_prepare_remaining_tasks']
        )
        self.assertIn("self.control(0.1, -0.3, 0.0, 'back')", method_source)
        self.assertIn('self._drive_for(linear_y=0.1, duration=3.0)', method_source)

    def test_mineral_detection_has_no_invented_cube_fallback(self):
        method_source = ast.get_source_segment(self.source, self.methods['control'])
        self.assertNotIn("shape = 'cube'", method_source)
        self.assertIn('refusing to invent a mineral class', method_source)

    def test_long_running_place_service_uses_configured_timeout(self):
        method_source = ast.get_source_segment(self.source, self.methods['control'])
        self.assertIn('timeout=self.place_service_timeout', method_source)

    def test_all_mission_exit_paths_share_cleanup(self):
        method_source = ast.get_source_segment(
            self.source, self.methods['run_mission_once']
        )
        self.assertGreaterEqual(method_source.count('self.cleanup_mission_execution()'), 3)

    def test_cleanup_cancels_navigation_and_stops_ramp(self):
        method_source = ast.get_source_segment(
            self.source, self.methods['cleanup_mission_execution']
        )
        self.assertIn('cancel_navigation_and_stop', method_source)
        self.assertIn("'/ramp/stop'", method_source)
        self.assertIn("'/shape_recognition/stop'", method_source)
        self.assertIn("'/yolov5/stop'", method_source)
        self.assertIn('unload_moon_detector', method_source)
        self.assertIn('_unload_yolo_for_mission', method_source)

    def test_scene_detection_unloads_old_yolo_before_starting_moon_model(self):
        method_source = ast.get_source_segment(
            self.source, self.methods['detect_moon_scene_at_task_point']
        )
        unload = method_source.index('self._unload_yolo_for_mission()')
        start = method_source.index("'/moon_detector/start'")
        self.assertLess(unload, start)
        self.assertIn('timeout=self.moon_start_timeout', method_source)

    def test_old_yolo_cannot_restart_after_mission_gpu_handoff(self):
        restart_source = ast.get_source_segment(
            self.source, self.methods['_restart_yolo']
        )
        unload_source = ast.get_source_segment(
            self.source, self.methods['_unload_yolo_for_mission']
        )
        self.assertIn('self._yolo_unloaded_for_mission', restart_source)
        self.assertIn('with self._gpu_lifecycle_lock:', restart_source)
        self.assertIn("'/yolov5/unload'", unload_source)
        self.assertIn('with self._gpu_lifecycle_lock:', unload_source)
        self.assertIn('self._yolo_unloaded_for_mission = True', unload_source)

    def test_reset_unloads_moon_before_allowing_old_yolo_again(self):
        method_source = ast.get_source_segment(
            self.source, self.methods['reset_mission_callback']
        )
        unload = method_source.index('self.unload_moon_detector()')
        allow_yolo = method_source.index(
            'self._yolo_unloaded_for_mission = False', unload
        )
        self.assertLess(unload, allow_yolo)

    def test_debug_cleanup_does_not_poison_next_mission_gpu_handoff(self):
        method_source = ast.get_source_segment(
            self.source, self.methods['_run_debug_service']
        )
        cleanup = method_source.index('self.cleanup_mission_execution()')
        allow_yolo = method_source.index(
            'self._yolo_unloaded_for_mission = False', cleanup
        )
        self.assertLess(cleanup, allow_yolo)

    def test_trigger_timeout_bounds_service_invocation(self):
        method_source = ast.get_source_segment(self.source, self.methods['call_trigger'])
        self.assertIn('threading.Event()', method_source)
        self.assertIn('completed.wait(remaining)', method_source)
        self.assertIn('abandoned.set()', method_source)

    def test_reset_moves_arm_before_lifecycle_reset(self):
        method_source = ast.get_source_segment(
            self.source, self.methods['reset_mission_callback']
        )
        safe_pose = method_source.index('self.safe_arm_pose(')
        lifecycle_reset = method_source.index('self.reset(', safe_pose)
        self.assertLess(safe_pose, lifecycle_reset)

    def test_safe_arm_wait_covers_full_command_duration(self):
        method_source = ast.get_source_segment(self.source, self.methods['safe_arm_pose'])
        self.assertIn('command_duration = 2.0', method_source)
        self.assertIn('max(command_duration, float(wait_seconds))', method_source)

    def test_result_persistence_uses_atomic_writer(self):
        method_source = ast.get_source_segment(
            self.source, self.methods['save_mission_results']
        )
        self.assertIn('atomic_write_json(', method_source)

    def test_shutdown_cancels_timer_detector_goal_and_motion(self):
        calls = called_attributes(self.methods['on_shutdown'])
        self.assertIn('cancel_start_timeout', calls)
        self.assertIn('cleanup_mission_execution', calls)


class DetectorContractTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source = load_source(DETECTOR_PATH)
        cls.methods = class_methods(DETECTOR_PATH, 'MoonDetector')

    def test_detector_has_no_navigation_or_voice_calls(self):
        self.assertNotIn('move_base', self.source)
        self.assertNotIn('voice_play', self.source)
        self.assertNotIn("['aplay'", self.source)

    def test_detector_result_is_session_scoped(self):
        method_source = ast.get_source_segment(self.source, self.methods['_result_message'])
        for key in (
            'task_point_index',
            'session_id',
            'status',
            'class_id',
            'class_name_en',
            'class_name_cn',
            'confidence',
            'detection_count',
            'timestamp',
            'timestamp_ros',
            'error',
        ):
            self.assertIn('"%s"' % key, method_source)

    def test_detector_finishes_only_once_per_generation(self):
        method_source = ast.get_source_segment(self.source, self.methods['_finish_session'])
        self.assertIn('self.result_published', method_source)
        self.assertIn('generation != self.session_generation', method_source)

    def test_detector_and_vendored_runtime_never_run_pip_install(self):
        self.assertNotIn('pip install', self.source)
        for relative_path in ('models/common.py', 'utils/general.py'):
            source = load_source(VENDORED_YOLO_ROOT / relative_path)
            self.assertNotIn('os.system("pip install', source)

    def test_detector_is_lazy_loaded_on_first_start(self):
        init_source = ast.get_source_segment(self.source, self.methods['__init__'])
        start_source = ast.get_source_segment(self.source, self.methods['handle_start'])
        self.assertNotIn('self._load_model()', init_source)
        self.assertIn('self._publish_status("unloaded")', init_source)
        self.assertIn('self._load_model()', start_source)

    def test_detector_unload_serializes_with_inference_and_clears_cuda(self):
        unload_source = ast.get_source_segment(
            self.source, self.methods['_unload_model']
        )
        cache_source = ast.get_source_segment(
            self.source, self.methods['_empty_cuda_cache']
        )
        self.assertIn('with self.inference_lock:', unload_source)
        self.assertIn('self.model = None', unload_source)
        self.assertIn('self._unsubscribe_images()', unload_source)
        self.assertIn('torch.cuda.empty_cache()', cache_source)
        self.assertIn('torch.cuda.ipc_collect()', cache_source)

    def test_detector_rechecks_session_after_acquiring_inference_lock(self):
        callback_source = ast.get_source_segment(
            self.source, self.methods['image_callback']
        )
        acquire = callback_source.index('self.inference_lock.acquire(False)')
        generation_check = callback_source.index(
            'generation != self.session_generation', acquire
        )
        model_check = callback_source.index('self.model is None', acquire)
        infer = callback_source.index('self._infer(image)', acquire)
        self.assertLess(generation_check, infer)
        self.assertLess(model_check, infer)


class LegacyYoloGpuLifecycleTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.node_source = load_source(YOLO_NODE_PATH)
        cls.node_methods = class_methods(YOLO_NODE_PATH, 'Yolov5Node')
        cls.trt_source = load_source(YOLO_TRT_PATH)
        cls.trt_methods = class_methods(YOLO_TRT_PATH, 'YoLov5TRT')

    def test_node_does_not_create_process_lifetime_autoinit_context(self):
        self.assertNotIn('pycuda.autoinit', self.node_source)
        self.assertNotIn('pycuda.autoinit', self.trt_source)
        init_source = ast.get_source_segment(
            self.trt_source, self.trt_methods['__init__']
        )
        self.assertIn('cuda.init()', init_source)
        self.assertIn('make_context()', init_source)

    def test_inference_and_unload_use_the_same_lock(self):
        image_proc = ast.get_source_segment(
            self.node_source, self.node_methods['image_proc']
        )
        unload = ast.get_source_segment(
            self.node_source, self.node_methods['unload_model']
        )
        stop = ast.get_source_segment(
            self.node_source, self.node_methods['stop_srv_callback']
        )
        callback = ast.get_source_segment(
            self.node_source, self.node_methods['image_callback']
        )
        self.assertIn('with self.inference_lock:', image_proc)
        self.assertIn('with self.inference_lock:', unload)
        self.assertIn('with self.inference_lock:', stop)
        self.assertIn('with self.inference_lock:', callback)
        self.assertIn('self.image_sub.unregister()', unload)

    def test_unload_releases_buffers_and_detaches_cuda_context(self):
        release = ast.get_source_segment(
            self.trt_source, self.trt_methods['_release_allocations']
        )
        destroy = ast.get_source_segment(
            self.trt_source, self.trt_methods['destroy']
        )
        self.assertIn('allocation.free()', release)
        self.assertIn('context.detach()', destroy)
        self.assertIn('self.ctx = None', destroy)

    def test_constructor_failure_detaches_partial_cuda_context(self):
        init_source = ast.get_source_segment(
            self.trt_source, self.trt_methods['__init__']
        )
        self.assertIn('except Exception:', init_source)
        self.assertIn('self._release_allocations()', init_source)
        self.assertIn('self.ctx.detach()', init_source)
        self.assertIn('self.ctx = None', init_source)

    def test_inference_always_pops_explicit_cuda_context(self):
        infer = ast.get_source_segment(
            self.trt_source, self.trt_methods['infer']
        )
        self.assertIn('finally:', infer)
        self.assertIn('self.ctx.pop()', infer)


class SupportingNodeSafetyTests(unittest.TestCase):
    def test_ramp_stop_is_idempotent_and_publishes_zero(self):
        methods = class_methods(CONTROLLER_PATH, 'VoiceControlNavNode')
        self.assertIn('run_ramp_alignment', methods)
        ramp_source = load_source(RAMP_PATH)
        tree = ast.parse(ramp_source, filename=str(RAMP_PATH))
        functions = {
            node.name: node for node in tree.body if isinstance(node, ast.FunctionDef)
        }
        stop_source = ast.get_source_segment(ramp_source, functions['stop_callback'])
        self.assertIn('if image_sub is not None', stop_source)
        self.assertIn('start_pick = False', stop_source)
        self.assertIn("rospy.set_param('~status', 'stop')", stop_source)
        self.assertIn('mecnum_pub.publish(Twist())', stop_source)

    def test_ramp_callback_cannot_publish_motion_after_stop(self):
        ramp_source = load_source(RAMP_PATH)
        tree = ast.parse(ramp_source, filename=str(RAMP_PATH))
        functions = {
            node.name: node for node in tree.body if isinstance(node, ast.FunctionDef)
        }
        align_source = ast.get_source_segment(ramp_source, functions['ramp_align'])
        self.assertIn('if not start_pick or ramp_cancel_requested:', align_source)

    def test_position_correction_uses_generation_guard_before_motion_publish(self):
        source = load_source(POSITION_PATH)
        self.assertIn('operation_generation', source)
        self.assertIn('operation_cancel_requested', source)
        self.assertIn('generation != operation_generation', source)
        self.assertIn('mecnum_pub.publish(twist)', source)

    def test_shape_arm_thread_checks_cancellation_generation(self):
        source = load_source(SHAPE_PATH)
        self.assertIn('operation_generation', source)
        self.assertIn('_set_servos_if_active', source)
        self.assertIn('_sleep_if_active', source)
        self.assertIn('move_thread.join(timeout=1.0)', source)

    def test_close_services_quiesce_nodes_for_manual_reset(self):
        position_source = load_source(POSITION_PATH)
        shape_source = load_source(SHAPE_PATH)
        self.assertIn('stop_callback(msg)', position_source)
        self.assertIn('self.stop_callback(msg)', shape_source)
        self.assertIn('unregister()', position_source)
        self.assertIn('unregister()', shape_source)
        self.assertNotIn('signal_shutdown', position_source)
        self.assertNotIn('signal_shutdown', shape_source)
        launch_source = load_source(LAUNCH_PATH)
        self.assertNotIn('respawn="true"', launch_source)


if __name__ == '__main__':
    unittest.main()
