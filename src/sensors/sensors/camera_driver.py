from pathlib import Path

import cv2
import rclpy
import yaml
from camera_info_manager import CameraInfoManager
from camera_info_manager.camera_info_manager import (
    URL_file,
    default_camera_info_url,
    parseURL,
    resolveURL,
)
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from sensor_msgs.srv import SetCameraInfo


class CameraDriver(Node):
    def __init__(self):
        super().__init__("camera_driver")

        self.declare_parameter("device",          "/dev/video0")
        self.declare_parameter("topic",           "/image_raw")
        self.declare_parameter("frame_id",        "front_camera")
        self.declare_parameter("camera_info_url", "")

        device          = self.get_parameter("device").get_parameter_value().string_value
        topic           = self.get_parameter("topic").get_parameter_value().string_value
        self._frame_id  = self.get_parameter("frame_id").get_parameter_value().string_value
        camera_info_url = self.get_parameter("camera_info_url").get_parameter_value().string_value

        self._available = False
        self.cap = cv2.VideoCapture(device)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if not self.cap.isOpened():
            self.get_logger().warn(f"Cannot open {device} — running in degraded mode")
        else:
            self._available = True
            self.get_logger().info(f"Camera opened: {device} → {topic}")

        self._cim = CameraInfoManager(self, cname=self._frame_id, url=camera_info_url)
        if camera_info_url:
            self._cim.loadCameraInfo()

        # camera_info_manager's auto-registered "set_camera_info" service uses the
        # legacy single-argument callback signature, which this rclpy no longer
        # supports (it always calls back with (request, response)). Replace it
        # with a compliant service that reuses the same save logic.
        self.destroy_service(self._cim.svc)
        self.create_service(SetCameraInfo, "set_camera_info", self._set_camera_info_cb)

        self.pub      = self.create_publisher(Image,      topic,                  10)
        self.info_pub = self.create_publisher(CameraInfo, topic + "/camera_info", 10)
        self.bridge   = CvBridge()
        self.create_timer(1 / 30, self._cb)

    def _cb(self):
        if not self._available:
            return
        ret, frame = self.cap.read()
        if not ret:
            return
        stamp = self.get_clock().now().to_msg()

        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        msg.header.stamp    = stamp
        msg.header.frame_id = self._frame_id
        self.pub.publish(msg)

        try:
            info = self._cim.getCameraInfo()
        except Exception:
            from sensor_msgs.msg import CameraInfo
            info = CameraInfo()
            h, w = frame.shape[:2]
            info.height = h
            info.width = w
        info.header.stamp    = stamp
        info.header.frame_id = self._frame_id
        self.info_pub.publish(info)

    def _set_camera_info_cb(self, request, response):
        info = request.camera_info
        # CameraInfo's k/d/r/p fields re-coerce any assignment back into a
        # numpy.ndarray via their property setters, so converting them on the
        # message and delegating to camera_info_manager's saveCalibrationFile()
        # (which hands them straight to yaml.safe_dump) still fails — it can't
        # represent numpy arrays. Build the YAML dict ourselves instead, only
        # reusing the URL-resolution helpers. list(ndarray) alone isn't enough:
        # it plainifies the container but each element stays a numpy.float64
        # scalar, which yaml.safe_dump also can't represent — cast per element.
        d = [float(x) for x in info.d]
        k = [float(x) for x in info.k]
        r = [float(x) for x in info.r]
        p = [float(x) for x in info.p]
        calib = {
            "image_width": info.width,
            "image_height": info.height,
            "camera_name": self._cim.cname,
            "distortion_model": info.distortion_model,
            "distortion_coefficients": {"data": d, "rows": 1, "cols": len(d)},
            "camera_matrix": {"data": k, "rows": 3, "cols": 3},
            "rectification_matrix": {"data": r, "rows": 3, "cols": 3},
            "projection_matrix": {"data": p, "rows": 3, "cols": 4},
        }

        url = self._cim.url or default_camera_info_url
        resolved = resolveURL(url, self._cim.cname)
        if parseURL(resolved) != URL_file:
            response.success = False
            response.status_message = f"Unsupported calibration URL: {resolved}"
            return response

        path = Path(resolved[len("file://"):])
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("w") as f:
            yaml.safe_dump(calib, f)

        self._cim.camera_info = info
        response.success = True
        return response

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
