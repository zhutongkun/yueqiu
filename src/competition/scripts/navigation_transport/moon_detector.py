#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""ROS1 service-driven detector for lunar scene-element cards."""

from __future__ import print_function

import datetime
import gc
import json
import os
import pathlib
import sys
import threading
import traceback
import types

import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger, TriggerResponse

from moon_mission_core import MultiFrameVoter

try:
    import cv2
    import numpy as np
    import torch
    INFERENCE_IMPORT_ERROR = ""
except Exception as exc:
    cv2 = None
    np = None
    torch = None
    INFERENCE_IMPORT_ERROR = "%s: %s" % (type(exc).__name__, exc)


CLASS_NAMES_EN = [
    "satellite",
    "space_station",
    "lunar_crater",
    "lunar_rover",
    "meteorite",
    "earth",
    "lunar_soil",
    "moon",
    "rocket",
    "astronaut",
]

CLASS_NAMES_CN = [
    "卫星",
    "空间站",
    "月坑",
    "月球车",
    "陨石",
    "地球",
    "月壤",
    "月球",
    "火箭",
    "宇航员",
]


def _install_checkpoint_pathlib_compatibility():
    """Map Python 3.13 cross-platform checkpoint paths to this host's path type."""
    if hasattr(pathlib, "__path__"):
        try:
            __import__("pathlib._local")
            return False
        except ImportError:
            pass

    module_name = "pathlib._local"
    compat = sys.modules.get(module_name)
    if compat is None:
        compat = types.ModuleType(module_name)
        sys.modules[module_name] = compat

    native_path = pathlib.WindowsPath if os.name == "nt" else pathlib.PosixPath
    native_pure_path = (
        pathlib.PureWindowsPath if os.name == "nt" else pathlib.PurePosixPath
    )
    compat.Path = pathlib.Path
    compat.PosixPath = native_path
    compat.WindowsPath = native_path
    compat.PurePath = pathlib.PurePath
    compat.PurePosixPath = native_pure_path
    compat.PureWindowsPath = native_pure_path
    pathlib._local = compat
    if not hasattr(pathlib, "__path__"):
        pathlib.__path__ = []
    return True


class MoonDetector(object):
    """Run independent, bounded recognition sessions requested by the mission node."""

    def __init__(self):
        self.lock = threading.RLock()
        self.inference_lock = threading.RLock()
        self.bridge = CvBridge()

        self.model_path = os.path.abspath(os.path.expanduser(
            rospy.get_param("~model_path", "")
        ))
        self.yolov5_repo = os.path.abspath(os.path.expanduser(
            rospy.get_param("~yolov5_repo", "")
        ))
        self.image_topic = rospy.get_param(
            "~image_topic", "/astra_camera/rgb/image_raw"
        )
        self.confidence_threshold = float(rospy.get_param(
            "~confidence_threshold", 0.70
        ))
        self.iou_threshold = float(rospy.get_param("~iou_threshold", 0.45))
        self.vote_window = max(1, int(rospy.get_param("~vote_window", 7)))
        self.minimum_votes = min(
            self.vote_window,
            max(1, int(rospy.get_param("~minimum_votes", 4))),
        )
        self.minimum_consecutive_frames = min(
            self.vote_window,
            max(1, int(rospy.get_param("~minimum_consecutive_frames", 1))),
        )
        self.timeout_seconds = max(0.1, float(rospy.get_param(
            "~timeout_seconds", 15.0
        )))
        self.maximum_result_age = max(0.0, float(rospy.get_param(
            "~maximum_result_age", 2.0
        )))
        self.minimum_box_area = max(0.0, float(rospy.get_param(
            "~minimum_box_area", 400.0
        )))
        self.maximum_box_area = max(0.0, float(rospy.get_param(
            "~maximum_box_area", 250000.0
        )))
        self.enable_roi = bool(rospy.get_param("~enable_roi", False))
        self.roi_x = int(rospy.get_param("~roi_x", 0))
        self.roi_y = int(rospy.get_param("~roi_y", 0))
        self.roi_width = int(rospy.get_param("~roi_width", 0))
        self.roi_height = int(rospy.get_param("~roi_height", 0))
        self.image_size = max(32, int(rospy.get_param("~image_size", 640)))
        self.device_setting = str(rospy.get_param("~device", "auto"))
        self.half = bool(rospy.get_param("~half", False))
        self.save_debug_image = bool(rospy.get_param(
            "~save_debug_image", False
        ))
        self.publish_debug_image = bool(rospy.get_param(
            "~publish_debug_image", False
        ))
        self.debug_image_directory = os.path.abspath(os.path.expanduser(
            rospy.get_param("~debug_image_directory", "/tmp/moon_detector")
        ))

        self.model = None
        self.device = None
        self.stride = 32
        self.letterbox = None
        self.non_max_suppression = None
        self.scale_boxes = None
        self.model_error = ""
        self.model_loading = False
        self.image_sub = None

        self.session_generation = 0
        self.active = False
        self.result_published = False
        self.task_point_index = 0
        self.session_id = ""
        self.session_started_at_ros = 0.0
        self.timeout_timer = None
        self.last_candidate = None
        self.last_error = ""
        self.last_frame_ros = 0.0
        self.voter = self._new_voter()

        self.result_pub = rospy.Publisher(
            "/moon_detector/result", String, queue_size=1, latch=False
        )
        self.finished_pub = rospy.Publisher(
            "/moon_detector/finished", Bool, queue_size=1, latch=True
        )
        self.status_pub = rospy.Publisher(
            "/moon_detector/status", String, queue_size=1, latch=True
        )
        self.debug_pub = None
        if self.publish_debug_image:
            self.debug_pub = rospy.Publisher(
                "/moon_detector/debug_image", Image, queue_size=1
            )

        self.finished_pub.publish(Bool(data=False))
        self._publish_status("unloaded")

        self.reset_service = rospy.Service(
            "/moon_detector/reset", Trigger, self.handle_reset
        )
        self.preload_service = rospy.Service(
            "/moon_detector/preload", Trigger, self.handle_preload
        )
        self.start_service = rospy.Service(
            "/moon_detector/start", Trigger, self.handle_start
        )
        self.stop_service = rospy.Service(
            "/moon_detector/stop", Trigger, self.handle_stop
        )
        self.unload_service = rospy.Service(
            "/moon_detector/unload", Trigger, self.handle_unload
        )
        rospy.on_shutdown(self.on_shutdown)

    def _new_voter(self):
        return MultiFrameVoter(
            vote_window=self.vote_window,
            minimum_votes=self.minimum_votes,
            minimum_consecutive_frames=self.minimum_consecutive_frames,
            confidence_threshold=self.confidence_threshold,
            timeout_seconds=self.timeout_seconds,
        )

    def _publish_status(self, status):
        self.status_pub.publish(String(data=str(status)))

    @staticmethod
    def _utc_timestamp():
        return datetime.datetime.now(datetime.timezone.utc).isoformat()

    def _load_model(self):
        with self.inference_lock:
            with self.lock:
                if self.model is not None:
                    return True
                self.model_loading = True
                self.model_error = ""
            self._publish_status("loading_model")

            model = None
            device = None
            try:
                if INFERENCE_IMPORT_ERROR:
                    raise ImportError(
                        "inference dependencies unavailable: %s"
                        % INFERENCE_IMPORT_ERROR
                    )
                if not self.yolov5_repo or not os.path.isdir(self.yolov5_repo):
                    raise IOError("YOLOv5 repository not found: %s" % self.yolov5_repo)
                if not self.model_path or not os.path.isfile(self.model_path):
                    raise IOError("model file not found: %s" % self.model_path)
                try:
                    import ultralytics
                    if not getattr(ultralytics, "__version__", ""):
                        raise ImportError("ultralytics has no version metadata")
                except Exception as exc:
                    raise ImportError(
                        "the local YOLOv5 repository requires a working ultralytics "
                        "package installed during setup; runtime installation is disabled: %s"
                        % exc
                    )

                os.environ["YOLOv5_AUTOINSTALL"] = "false"
                if self.yolov5_repo in sys.path:
                    sys.path.remove(self.yolov5_repo)
                sys.path.insert(0, self.yolov5_repo)

                from models.common import DetectMultiBackend
                try:
                    from utils.augmentations import letterbox
                except ImportError:
                    from utils.datasets import letterbox
                from utils.general import non_max_suppression
                try:
                    from utils.general import scale_boxes
                except ImportError:
                    from utils.general import scale_coords as scale_boxes
                from utils.torch_utils import select_device

                if _install_checkpoint_pathlib_compatibility():
                    rospy.logwarn(
                        "Enabled pathlib checkpoint compatibility for the target Python runtime"
                    )

                requested_device = self.device_setting.strip().lower()
                if requested_device in ("", "auto"):
                    requested_device = "0" if torch.cuda.is_available() else "cpu"
                device = select_device(requested_device)
                model = DetectMultiBackend(
                    self.model_path,
                    device=device,
                    dnn=False,
                    data=None,
                    fp16=self.half,
                )
                model_stride = model.stride
                if hasattr(model_stride, "max"):
                    stride = int(model_stride.max())
                elif isinstance(model_stride, (tuple, list)):
                    stride = int(max(model_stride))
                else:
                    stride = int(model_stride)
                image_size = int(np.ceil(
                    float(self.image_size) / float(stride)
                ) * stride)

                model_names = model.names
                if isinstance(model_names, dict):
                    normalized_names = [
                        model_names[index] for index in range(len(model_names))
                    ]
                else:
                    normalized_names = list(model_names)
                if normalized_names != CLASS_NAMES_EN:
                    raise ValueError(
                        "model class mapping mismatch: expected %s, got %s"
                        % (CLASS_NAMES_EN, normalized_names)
                    )

                try:
                    model.warmup(imgsz=(1, 3, image_size, image_size))
                except Exception as exc:
                    rospy.logwarn("Moon detector warmup skipped: %s", exc)

                with self.lock:
                    self.model = model
                    self.device = device
                    self.stride = stride
                    self.image_size = image_size
                    self.letterbox = letterbox
                    self.non_max_suppression = non_max_suppression
                    self.scale_boxes = scale_boxes
                    self.model_error = ""
                    self.model_loading = False
                self._publish_status("ready")
                rospy.loginfo(
                    "Moon detector ready: model=%s device=%s image_topic=%s",
                    self.model_path,
                    self.device,
                    self.image_topic,
                )
                return True
            except Exception as exc:
                with self.lock:
                    self.model = None
                    self.device = None
                    self.letterbox = None
                    self.non_max_suppression = None
                    self.scale_boxes = None
                    self.model_error = "%s: %s" % (type(exc).__name__, exc)
                    self.model_loading = False
                if model is not None:
                    del model
                if device is not None and str(device).lower() != "cpu":
                    self._empty_cuda_cache()
                else:
                    gc.collect()
                self._publish_status("model_error")
                rospy.logerr("Moon detector model load failed: %s", self.model_error)
                rospy.logdebug(traceback.format_exc())
                return False

    @staticmethod
    def _empty_cuda_cache():
        gc.collect()
        if torch is None:
            return
        try:
            if torch.cuda.is_available():
                torch.cuda.empty_cache()
                if hasattr(torch.cuda, "ipc_collect"):
                    torch.cuda.ipc_collect()
        except Exception as exc:
            rospy.logwarn("Moon detector CUDA cache cleanup failed: %s", exc)

    def _subscribe_images(self):
        if self.image_sub is None:
            self.image_sub = rospy.Subscriber(
                self.image_topic,
                Image,
                self.image_callback,
                queue_size=1,
                buff_size=2 ** 24,
            )

    def _unsubscribe_images(self):
        subscriber = self.image_sub
        self.image_sub = None
        if subscriber is not None:
            try:
                subscriber.unregister()
            except Exception as exc:
                rospy.logwarn("Moon detector subscriber cleanup failed: %s", exc)

    def _unload_model(self, publish_status=True):
        with self.inference_lock:
            with self.lock:
                self._reset_session_locked()
                model = self.model
                device = self.device
                self.model = None
                self.device = None
                self.letterbox = None
                self.non_max_suppression = None
                self.scale_boxes = None
                self.model_error = ""
                self.model_loading = False
            self._unsubscribe_images()
            clear_cuda_cache = (
                model is not None
                and device is not None
                and str(device).lower() != "cpu"
            )
            if model is not None:
                del model
            if clear_cuda_cache:
                self._empty_cuda_cache()
            else:
                gc.collect()
        if publish_status:
            self.finished_pub.publish(Bool(data=False))
            self._publish_status("unloaded")
        return True

    def _cancel_timeout_locked(self):
        timer = self.timeout_timer
        self.timeout_timer = None
        if timer is not None:
            try:
                timer.shutdown()
            except Exception:
                pass

    def _reset_session_locked(self):
        self._cancel_timeout_locked()
        self.session_generation += 1
        self.active = False
        self.result_published = False
        self.task_point_index = 0
        self.session_id = ""
        self.session_started_at_ros = 0.0
        self.last_candidate = None
        self.last_error = ""
        self.last_frame_ros = 0.0
        self.voter = self._new_voter()
        self.voter.reset(now=rospy.Time.now().to_sec())

    def handle_reset(self, _request):
        with self.inference_lock:
            with self.lock:
                self._reset_session_locked()
                status = "ready" if self.model is not None else (
                    "model_error" if self.model_error else "unloaded"
                )
            self._unsubscribe_images()
        self.finished_pub.publish(Bool(data=False))
        self._publish_status(status)
        return TriggerResponse(success=True, message="moon detector session reset")

    def handle_preload(self, _request):
        with self.lock:
            if self.active:
                return TriggerResponse(
                    success=False,
                    message="cannot preload during an active detection session",
                )
        if not self._load_model():
            with self.lock:
                message = self.model_error
            return TriggerResponse(success=False, message=message)
        return TriggerResponse(success=True, message="moon detector model preloaded")

    def handle_start(self, _request):
        try:
            task_point_index = int(rospy.get_param(
                "/moon_detector/request/task_point_index", 0
            ))
            session_id = str(rospy.get_param(
                "/moon_detector/request/session_id", ""
            )).strip()
            started_at_ros = float(rospy.get_param(
                "/moon_detector/request/started_at_ros", 0.0
            ))
        except (TypeError, ValueError) as exc:
            return TriggerResponse(
                success=False,
                message="invalid detector request parameters: %s" % exc,
            )

        if task_point_index not in (1, 2, 3):
            return TriggerResponse(
                success=False,
                message="task_point_index must be 1, 2, or 3",
            )
        if not session_id:
            return TriggerResponse(success=False, message="session_id is required")
        if started_at_ros <= 0.0:
            started_at_ros = rospy.Time.now().to_sec()

        with self.inference_lock:
            with self.lock:
                self._reset_session_locked()
                self.task_point_index = task_point_index
                self.session_id = session_id
                self.session_started_at_ros = started_at_ros
                self.active = True
                generation = self.session_generation
                self.voter.reset(now=started_at_ros)

            self.finished_pub.publish(Bool(data=False))
            if not self._load_model():
                with self.lock:
                    model_error = self.model_error
                self._finish_session(
                    generation=generation,
                    status="model_error",
                    error=model_error,
                )
                return TriggerResponse(success=False, message=model_error)

            with self.lock:
                if not self.active or generation != self.session_generation:
                    return TriggerResponse(success=False, message="session was cancelled")
                self._subscribe_images()
                self.timeout_timer = rospy.Timer(
                    rospy.Duration(self.timeout_seconds),
                    lambda event: self._timeout_callback(event, generation),
                    oneshot=True,
                )
        self._publish_status("running")
        return TriggerResponse(
            success=True,
            message="started task point %d session %s"
            % (task_point_index, session_id),
        )

    def handle_stop(self, _request):
        with self.lock:
            if not self.active:
                self._cancel_timeout_locked()
                generation = None
            else:
                generation = self.session_generation
        if generation is None:
            self._unsubscribe_images()
            return TriggerResponse(success=True, message="detector already stopped")
        self._finish_session(
            generation=generation,
            status="stopped",
            error="recognition session stopped",
        )
        self._unsubscribe_images()
        return TriggerResponse(success=True, message="moon detector stopped")

    def handle_unload(self, _request):
        self._unload_model()
        return TriggerResponse(success=True, message="moon detector unloaded")

    def _timeout_callback(self, _event, generation):
        with self.lock:
            if generation != self.session_generation or not self.active:
                return
            evaluation = self.voter.evaluate(now=rospy.Time.now().to_sec())
            last_error = self.last_error
        error = "no stable result before %.3f second timeout" % self.timeout_seconds
        if last_error:
            error = "%s; last frame error: %s" % (error, last_error)
        self._finish_session(
            generation=generation,
            status="timeout",
            detection_count=int(evaluation.get("accepted_detections", 0)),
            error=error,
        )

    def _result_message(self, status, class_id, confidence, detection_count, error):
        if class_id in range(len(CLASS_NAMES_EN)):
            class_name_en = CLASS_NAMES_EN[class_id]
            class_name_cn = CLASS_NAMES_CN[class_id]
        else:
            class_id = -1
            class_name_en = ""
            class_name_cn = ""
        now_ros = rospy.Time.now().to_sec()
        return {
            "task_point_index": int(self.task_point_index),
            "session_id": self.session_id,
            "status": str(status),
            "class_id": int(class_id),
            "class_name_en": class_name_en,
            "class_name_cn": class_name_cn,
            "confidence": float(confidence),
            "detection_count": int(detection_count),
            "timestamp": self._utc_timestamp(),
            "timestamp_ros": float(now_ros),
            "error": str(error),
        }

    def _finish_session(
        self,
        generation,
        status,
        class_id=-1,
        confidence=0.0,
        detection_count=0,
        error="",
    ):
        with self.lock:
            if generation != self.session_generation:
                return False
            if not self.active or self.result_published:
                return False
            self.result_published = True
            self.active = False
            self.last_error = str(error)
            self._cancel_timeout_locked()
            result = self._result_message(
                status,
                class_id,
                confidence,
                detection_count,
                error,
            )
        self.result_pub.publish(String(data=json.dumps(
            result, ensure_ascii=False, sort_keys=True
        )))
        self.finished_pub.publish(Bool(data=True))
        self._publish_status(status)
        rospy.loginfo(
            "Moon detector finished task_point=%d session=%s status=%s class=%s confidence=%.3f",
            result["task_point_index"],
            result["session_id"],
            result["status"],
            result["class_name_en"],
            result["confidence"],
        )
        return True

    def _frame_is_current(self, msg, now_ros, started_at_ros):
        stamp_ros = msg.header.stamp.to_sec()
        if stamp_ros <= 0.0:
            return True
        if stamp_ros + 1e-6 < started_at_ros:
            return False
        if self.maximum_result_age > 0.0:
            if now_ros - stamp_ros > self.maximum_result_age:
                return False
        return True

    def _crop_roi(self, image):
        if not self.enable_roi:
            return image, (0, 0)
        height, width = image.shape[:2]
        x1 = min(max(self.roi_x, 0), width)
        y1 = min(max(self.roi_y, 0), height)
        requested_width = self.roi_width if self.roi_width > 0 else width - x1
        requested_height = self.roi_height if self.roi_height > 0 else height - y1
        x2 = min(max(x1 + requested_width, x1), width)
        y2 = min(max(y1 + requested_height, y1), height)
        if x2 <= x1 or y2 <= y1:
            raise ValueError("configured ROI is empty")
        return image[y1:y2, x1:x2], (x1, y1)

    def _infer(self, image):
        roi_image, offset = self._crop_roi(image)
        prepared = self.letterbox(
            roi_image,
            new_shape=self.image_size,
            stride=self.stride,
            auto=True,
        )[0]
        prepared = prepared[:, :, ::-1].transpose(2, 0, 1)
        prepared = np.ascontiguousarray(prepared)
        tensor = torch.from_numpy(prepared).to(self.device)
        tensor = tensor.half() if self.model.fp16 else tensor.float()
        tensor /= 255.0
        if tensor.ndimension() == 3:
            tensor = tensor.unsqueeze(0)

        with torch.no_grad():
            prediction = self.model(tensor, augment=False, visualize=False)
            prediction = self.non_max_suppression(
                prediction,
                self.confidence_threshold,
                self.iou_threshold,
                classes=None,
                agnostic=False,
                max_det=20,
            )

        candidates = []
        detections = prediction[0]
        if detections is not None and len(detections):
            detections[:, :4] = self.scale_boxes(
                tensor.shape[2:], detections[:, :4], roi_image.shape
            ).round()
            for row in detections:
                x1, y1, x2, y2, confidence, class_id = row[:6].tolist()
                class_id = int(class_id)
                box_area = max(0.0, x2 - x1) * max(0.0, y2 - y1)
                if class_id not in range(len(CLASS_NAMES_EN)):
                    continue
                if box_area < self.minimum_box_area:
                    continue
                if self.maximum_box_area > 0.0 and box_area > self.maximum_box_area:
                    continue
                candidates.append({
                    "class_id": class_id,
                    "confidence": float(confidence),
                    "box": (
                        int(x1 + offset[0]),
                        int(y1 + offset[1]),
                        int(x2 + offset[0]),
                        int(y2 + offset[1]),
                    ),
                })
        if not candidates:
            return None
        return max(candidates, key=lambda item: item["confidence"])

    def image_callback(self, msg):
        with self.lock:
            if not self.active or self.result_published or self.model is None:
                return
            generation = self.session_generation
            started_at_ros = self.session_started_at_ros

        now_ros = rospy.Time.now().to_sec()
        if not self._frame_is_current(msg, now_ros, started_at_ros):
            return
        with self.lock:
            if generation != self.session_generation or not self.active:
                return
            self.last_frame_ros = now_ros
        if not self.inference_lock.acquire(False):
            return
        try:
            with self.lock:
                if (
                    generation != self.session_generation
                    or not self.active
                    or self.model is None
                ):
                    return
            try:
                image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
                candidate = self._infer(image)
            except Exception as exc:
                rospy.logwarn_throttle(2.0, "Moon detector frame error: %s", exc)
                with self.lock:
                    if generation == self.session_generation and self.active:
                        self.last_error = "%s: %s" % (type(exc).__name__, exc)
                        self.voter.add_frame([], timestamp=now_ros)
                return

            with self.lock:
                if generation != self.session_generation or not self.active:
                    return
                self.last_candidate = candidate
                voter_candidates = []
                if candidate is not None:
                    voter_candidates.append({
                        "class_id": candidate["class_id"],
                        "confidence": candidate["confidence"],
                    })
                vote_result = self.voter.add_frame(
                    voter_candidates, timestamp=now_ros
                )

            if self.debug_pub is not None or self.save_debug_image:
                self._publish_debug(image, msg, candidate)

            if vote_result and vote_result.get("status") == "success":
                self._finish_session(
                    generation=generation,
                    status="success",
                    class_id=int(vote_result["class_id"]),
                    confidence=float(vote_result.get("confidence", 0.0)),
                    detection_count=int(vote_result.get(
                        "detection_count", vote_result.get("vote_count", 0)
                    )),
                    error="",
                )
        finally:
            self.inference_lock.release()

    def _publish_debug(self, image, source_msg, candidate):
        debug_image = image.copy()
        if self.enable_roi:
            height, width = debug_image.shape[:2]
            x1 = min(max(self.roi_x, 0), width)
            y1 = min(max(self.roi_y, 0), height)
            x2 = min(x1 + (self.roi_width or width - x1), width)
            y2 = min(y1 + (self.roi_height or height - y1), height)
            cv2.rectangle(debug_image, (x1, y1), (x2, y2), (255, 180, 0), 2)
        if candidate is not None:
            x1, y1, x2, y2 = candidate["box"]
            label = "%s %.2f" % (
                CLASS_NAMES_EN[candidate["class_id"]],
                candidate["confidence"],
            )
            cv2.rectangle(debug_image, (x1, y1), (x2, y2), (0, 220, 0), 2)
            cv2.putText(
                debug_image,
                label,
                (x1, max(20, y1 - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 220, 0),
                2,
                cv2.LINE_AA,
            )
        if self.debug_pub is not None:
            try:
                debug_msg = self.bridge.cv2_to_imgmsg(debug_image, encoding="bgr8")
                debug_msg.header = source_msg.header
                self.debug_pub.publish(debug_msg)
            except CvBridgeError as exc:
                rospy.logwarn_throttle(2.0, "Debug image conversion failed: %s", exc)
        if self.save_debug_image:
            try:
                if not os.path.isdir(self.debug_image_directory):
                    os.makedirs(self.debug_image_directory)
                safe_session_id = "".join(
                    character if character.isalnum() or character in "-_" else "_"
                    for character in self.session_id
                )
                filename = "%s_task_%d_%s.jpg" % (
                    safe_session_id,
                    self.task_point_index,
                    datetime.datetime.utcnow().strftime("%Y%m%dT%H%M%S_%f"),
                )
                cv2.imwrite(os.path.join(self.debug_image_directory, filename), debug_image)
            except Exception as exc:
                rospy.logwarn_throttle(2.0, "Debug image save failed: %s", exc)

    def on_shutdown(self):
        self._unload_model(publish_status=False)


def main():
    rospy.init_node("moon_detector", anonymous=False)
    MoonDetector()
    rospy.spin()


if __name__ == "__main__":
    main()
