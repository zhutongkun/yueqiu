#!/usr/bin/env python3
import pathlib
import re
import unittest
import xml.etree.ElementTree as ET


PACKAGE_ROOT = pathlib.Path(__file__).resolve().parents[1]
WORKSPACE_ROOT = PACKAGE_ROOT.parents[1]
MAIN_LAUNCH = (
    PACKAGE_ROOT / 'scripts' / 'navigation_transport' / 'position_correction_pick.launch'
)
MOON_LAUNCH = PACKAGE_ROOT / 'launch' / 'moon_detector.launch'
DETECTOR_CONFIG = PACKAGE_ROOT / 'config' / 'moon_detector.yaml'
MISSION_CONFIG = PACKAGE_ROOT / 'config' / 'moon_competition.yaml'
CMAKE = PACKAGE_ROOT / 'CMakeLists.txt'
PACKAGE_XML = PACKAGE_ROOT / 'package.xml'
SETUP_SCRIPT = WORKSPACE_ROOT / 'setup_moon_runtime.sh'
RUN_SCRIPT = WORKSPACE_ROOT / 'run_moon_competition.sh'
STOP_SCRIPT = WORKSPACE_ROOT / 'stop_moon_competition.sh'
YOLO_GENERAL = PACKAGE_ROOT / 'third_party' / 'yolov5' / 'utils' / 'general.py'


def source(path):
    return path.read_text(encoding='utf-8')


def numeric_config_value(path, name):
    match = re.search(
        r'^%s:\s*([0-9]+(?:\.[0-9]+)?)\s*$' % re.escape(name),
        source(path),
        flags=re.MULTILINE,
    )
    if match is None:
        raise AssertionError('numeric config key %s was not found' % name)
    return float(match.group(1))


class LaunchAndConfigContractTests(unittest.TestCase):
    def test_launch_files_are_well_formed_xml(self):
        ET.parse(str(MAIN_LAUNCH))
        ET.parse(str(MOON_LAUNCH))

    def test_detector_yaml_remains_the_default_parameter_source(self):
        detector_text = source(DETECTOR_CONFIG)
        launch_root = ET.parse(str(MOON_LAUNCH)).getroot()
        args = {item.attrib['name']: item.attrib.get('default') for item in launch_root.findall('arg')}
        for name in (
            'model_path',
            'yolov5_repo',
            'image_topic',
            'publish_debug_image',
            'save_debug_image',
        ):
            self.assertIn('%s:' % name, detector_text)
            self.assertEqual('', args[name])

        node = launch_root.find('node')
        conditional_params = {item.attrib['name']: item.attrib.get('if', '') for item in node.findall('param')}
        self.assertEqual(set(args), set(conditional_params))
        self.assertTrue(all("!= ''" in condition for condition in conditional_params.values()))

    def test_main_launch_forwards_all_moon_overrides(self):
        text = source(MAIN_LAUNCH)
        for name in (
            'moon_model_path',
            'moon_yolov5_repo',
            'moon_image_topic',
            'moon_publish_debug_image',
            'moon_save_debug_image',
        ):
            self.assertIn('<arg name="%s" default=""/>' % name, text)
            self.assertIn('$(arg %s)' % name, text)

    def test_voice_setting_is_competition_scoped(self):
        self.assertIn('enable_voice: true', source(MISSION_CONFIG))
        self.assertNotIn('enable_voice:', source(DETECTOR_CONFIG))
        main_launch = source(MAIN_LAUNCH)
        self.assertIn('/competition_mission/enable_voice', main_launch)
        self.assertIn("str(arg('enable_voice')).lower() == 'false'", main_launch)
        self.assertIn('<include unless=', main_launch)
        self.assertIn('competition)/launch/mic_init.launch', main_launch)

    def test_start_window_opens_only_after_yolo_prewarm(self):
        mission_config = source(MISSION_CONFIG)
        self.assertIn('prewarm_yolo_before_start: true', mission_config)
        self.assertGreaterEqual(
            numeric_config_value(MISSION_CONFIG, 'yolo_prewarm_timeout'),
            60.0,
        )
        main_launch = source(MAIN_LAUNCH)
        self.assertIn('<arg name="enable_timeout_auto_start" default=""/>', main_launch)
        self.assertIn('/competition_mission/enable_timeout_auto_start', main_launch)

    def test_stage_caps_fit_exactly_inside_the_body_budget(self):
        mission_timeout = numeric_config_value(
            MISSION_CONFIG, 'mission_timeout_seconds'
        )
        return_reserve = numeric_config_value(
            MISSION_CONFIG, 'return_reserve_seconds'
        )
        stage_total = sum(
            numeric_config_value(MISSION_CONFIG, name)
            for name in (
                'initial_stage_timeout',
                'detect_stage_timeout',
                'pick_stage_timeout',
                'place_stage_timeout',
                'pick_stage_timeout',
                'place_stage_timeout',
                'remaining_tasks_timeout',
            )
        )
        self.assertEqual(mission_timeout - return_reserve, stage_total)

    def test_return_and_announcement_fit_before_hard_deadline(self):
        return_reserve = numeric_config_value(
            MISSION_CONFIG, 'return_reserve_seconds'
        )
        return_stage = numeric_config_value(MISSION_CONFIG, 'return_stage_timeout')
        announcement_stage = numeric_config_value(
            MISSION_CONFIG, 'announcement_stage_timeout'
        )
        self.assertLessEqual(return_stage + announcement_stage, return_reserve)
        self.assertLess(
            numeric_config_value(MISSION_CONFIG, 'mission_timeout_seconds'),
            480.0,
        )

    def test_first_two_scene_rotation_defaults_are_explicit(self):
        self.assertEqual(
            90.0,
            numeric_config_value(MISSION_CONFIG, 'moon_rotation_degrees'),
        )
        self.assertEqual(
            0.5,
            numeric_config_value(MISSION_CONFIG, 'moon_rotation_speed'),
        )


class BuildMetadataContractTests(unittest.TestCase):
    def test_runtime_catkin_dependencies_are_declared(self):
        required = {'interfaces', 'kinematics', 'message_filters', 'sdk'}
        package_root = ET.parse(str(PACKAGE_XML)).getroot()
        declared = {item.text for item in package_root.findall('depend')}
        self.assertTrue(required.issubset(declared))
        cmake = source(CMAKE)
        for dependency in required:
            self.assertIn(dependency, cmake)

    def test_install_rules_preserve_python3_shebangs_and_runtime_assets(self):
        cmake = source(CMAKE)
        self.assertNotIn('catkin_install_python(', cmake)
        for relative_path in (
            'scripts/navigation_transport/position_correction_pick.launch',
            'scripts/navigation_transport/calibration_position/automatic_pick.py',
            'scripts/navigation_transport/ramp/ramp.py',
            'scripts/navigation_transport/shape_recognition/shape_recognition_down.py',
            'scripts/navigation_transport/shape_recognition/tone.py',
            'scripts/navigation_transport/yolo5/yolov5_trt_6_2.py',
            'scripts/navigation_transport/yolo5/shape_models.engine',
            'scripts/navigation_transport/yolo5/shape_models_libmyplugins.so',
        ):
            self.assertIn(relative_path, cmake)
            self.assertTrue((PACKAGE_ROOT / relative_path).is_file())


class DeploymentScriptContractTests(unittest.TestCase):
    def test_setup_supports_explicit_underlay_and_existing_build_tool(self):
        text = source(SETUP_SCRIPT)
        self.assertIn('MOON_ROS_SETUP', text)
        self.assertIn('/opt/ros_ws/melodic/setup.bash', text)
        self.assertIn('MOON_CATKIN_BUILD_TOOL', text)
        self.assertIn('.catkin_tools', text)
        self.assertIn('catkin build', text)
        self.assertIn('catkin_make', text)

    def test_setup_requires_python_38_and_pinned_ultralytics(self):
        text = source(SETUP_SCRIPT)
        self.assertIn("sys.version_info >= (3, 8)", text)
        self.assertIn('ultralytics==8.4.83', text)
        self.assertNotIn('ultralytics>=', text)

    def test_setup_checks_all_robot_python_dependencies(self):
        text = source(SETUP_SCRIPT)
        for import_name in (
            'pycuda.driver',
            'tensorrt',
            'sklearn',
            'transforms3d',
            'scipy',
            'seaborn',
            'message_filters',
            'interfaces.msg',
            'sdk.common',
            'kinematics.transform',
        ):
            self.assertIn(import_name, text)

    def test_runtime_disables_network_installation(self):
        run_text = source(RUN_SCRIPT)
        self.assertIn('YOLOv5_AUTOINSTALL=false', run_text)
        self.assertIn('PIP_NO_INDEX=1', run_text)
        self.assertIn('ULTRALYTICS_OFFLINE=true', run_text)

        general_text = source(YOLO_GENERAL)
        self.assertIn('AUTOINSTALL = False', general_text)
        self.assertIn('install=False', general_text)

    def test_stop_script_releases_both_visual_models(self):
        text = source(STOP_SCRIPT)
        self.assertIn('call_stop_service /moon_detector/unload', text)
        self.assertIn('call_stop_service /yolov5/unload', text)

    def test_all_ros_scripts_support_the_explicit_underlay(self):
        for path in (SETUP_SCRIPT, RUN_SCRIPT, STOP_SCRIPT):
            text = source(path)
            self.assertIn('MOON_ROS_SETUP', text)
            self.assertIn('/opt/ros_ws/melodic/setup.bash', text)


if __name__ == '__main__':
    unittest.main()
