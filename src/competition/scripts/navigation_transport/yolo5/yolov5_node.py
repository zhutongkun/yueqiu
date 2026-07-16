#!/usr/bin/env python3
# encoding: utf-8
import gc
import os
import queue
import signal
import threading

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger, TriggerResponse

from sdk import common
from yolov5_trt_6_2 import YoLov5TRT


MODEL_PATH = os.path.dirname(os.path.realpath(__file__))


class Yolov5Node:
    """TensorRT shape detector with explicit load, pause, and unload services."""

    def __init__(self, name):
        rospy.init_node(name)
        self.start = False
        self.running = True
        self.yolov5 = None
        self.image_sub = None
        self.image_queue = queue.Queue(maxsize=1)
        self.result_image_pub = None
        self.inference_lock = threading.RLock()

        self.engine = rospy.get_param('~engine')
        self.calibration = rospy.get_param('~calibration', False)
        self.lib = rospy.get_param('~lib')
        self.conf_thresh = rospy.get_param('~conf_thresh', 0.8)
        self.image_topic = rospy.get_param('~image_topic', '/astra_camera/rgb/image_raw')
        self.classes = rospy.get_param('/classes')

        rospy.set_param('~shape', 'None')
        try:
            rospy.wait_for_service('/astra_camera/set_ldp', timeout=30.0)
        except rospy.ROSException:
            rospy.logwarn('/astra_camera/set_ldp was not available during YOLO startup')

        rospy.Service('/yolov5/start', Trigger, self.start_srv_callback)
        rospy.Service('/yolov5/stop', Trigger, self.stop_srv_callback)
        rospy.Service('/yolov5/unload', Trigger, self.unload_srv_callback)
        rospy.Service('/yolov5/shutdown', Trigger, self.shutdown_srv_callback)
        rospy.Service('/yolov5/calibration', Trigger, self.calibration_srv_callback)
        rospy.set_param('~init_finish', True)

        signal.signal(signal.SIGINT, self.shutdown)
        rospy.on_shutdown(self.on_shutdown)

    def calibration_srv_callback(self, _request):
        self.calibration = True
        if self.result_image_pub is None:
            self.result_image_pub = rospy.Publisher('~object_image', Image, queue_size=1)
        return TriggerResponse(success=True)

    def start_srv_callback(self, _request):
        rospy.loginfo('start yolov5 detect')
        with self.inference_lock:
            try:
                if self.yolov5 is None:
                    self.yolov5 = YoLov5TRT(
                        os.path.join(MODEL_PATH, self.engine),
                        os.path.join(MODEL_PATH, self.lib),
                        self.classes,
                        self.conf_thresh,
                    )
                if self.image_sub is None:
                    self.image_sub = rospy.Subscriber(
                        self.image_topic,
                        Image,
                        self.image_callback,
                        queue_size=1,
                    )
            except Exception as exc:
                self.start = False
                self.unload_model()
                rospy.logerr('failed to start yolov5: %s', exc)
                return TriggerResponse(success=False, message=str(exc))

            self.clear_image_queue()
            rospy.set_param('~shape', 'None')
            self.start = True
        return TriggerResponse(success=True)

    def stop_srv_callback(self, _request):
        """Pause inference; keep the node and TensorRT context reusable."""
        rospy.loginfo('pause yolov5 detect')
        with self.inference_lock:
            self.start = False
            self.clear_image_queue()
            rospy.set_param('~shape', 'None')
        return TriggerResponse(success=True)

    def unload_srv_callback(self, _request):
        success, message = self.unload_model()
        return TriggerResponse(success=success, message=message)

    def unload_model(self):
        """Stop callbacks and release the detector-owned TensorRT/CUDA resources."""
        with self.inference_lock:
            self.start = False
            self.clear_image_queue()
            rospy.set_param('~shape', 'None')
            if self.image_sub is not None:
                try:
                    self.image_sub.unregister()
                except Exception as exc:
                    rospy.logwarn('failed to unregister yolov5 image subscriber: %s', exc)
                self.image_sub = None

            model = self.yolov5
            self.yolov5 = None
            if model is None:
                return True, 'yolov5 already unloaded'
            try:
                if hasattr(model, 'destroy'):
                    model.destroy()
            except Exception as exc:
                rospy.logerr('failed to unload yolov5 TensorRT context: %s', exc)
                return False, str(exc)
            finally:
                del model
                gc.collect()
        rospy.loginfo('unloaded yolov5 TensorRT model and CUDA context')
        return True, 'yolov5 unloaded'

    def shutdown_srv_callback(self, _request):
        self.start = False
        rospy.signal_shutdown('requested by /yolov5/shutdown')
        return TriggerResponse(success=True)

    def clear_image_queue(self):
        while not self.image_queue.empty():
            try:
                self.image_queue.get_nowait()
            except queue.Empty:
                break

    def image_callback(self, ros_image):
        with self.inference_lock:
            if not self.start or self.yolov5 is None:
                return
            rgb_image = np.ndarray(
                shape=(ros_image.height, ros_image.width, 3),
                dtype=np.uint8,
                buffer=ros_image.data,
            )
            bgr_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)
            if self.image_queue.full():
                try:
                    self.image_queue.get_nowait()
                except queue.Empty:
                    pass
            self.image_queue.put_nowait(bgr_image)
            self.image_proc()

    def shutdown(self, _signum, _frame):
        self.running = False
        rospy.signal_shutdown('SIGINT')

    def on_shutdown(self):
        success, message = self.unload_model()
        if not success:
            rospy.logwarn('yolov5 shutdown cleanup failed: %s', message)

    def image_proc(self):
        with self.inference_lock:
            try:
                if not self.start or self.yolov5 is None:
                    return
                image = self.image_queue.get_nowait()
                boxes, scores, class_ids = self.yolov5.infer(image)
                for box, confidence, class_id in zip(boxes, scores, class_ids):
                    class_id = int(class_id)
                    rospy.set_param('~shape', self.classes[class_id])
                    if self.calibration and self.result_image_pub is not None:
                        color = common.colors(class_id, True)
                        common.plot_one_box(
                            box,
                            image,
                            color=color,
                            label='{}:{:.2f}'.format(self.classes[class_id], confidence),
                        )
                if self.calibration and self.result_image_pub is not None:
                    self.result_image_pub.publish(common.cv2_image2ros(image, frame_id='yolov5'))
            except queue.Empty:
                return
            except Exception as exc:
                rospy.logerr_throttle(2.0, 'yolov5 inference failed: %s', exc)


if __name__ == '__main__':
    node = Yolov5Node('yolov5')
    try:
        rospy.spin()
    except Exception as exc:
        rospy.logerr(str(exc))
