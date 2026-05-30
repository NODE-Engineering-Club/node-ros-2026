"""YOLO26n-seg inference node using ONNX Runtime (CPU).

Subscribes to /image_raw.
Publishes:
  /yolo/detections — vision_msgs/Detection2DArray
  /yolo/seg_mask   — sensor_msgs/Image (mono8)
                     pixel value = detection index + 1  (0 = background)
                     paired to /yolo/detections by header stamp

Output tensor layout (YOLO26 NMS-free):
  output0: (1, 300, 38) — col 0-3: x1 y1 x2 y2 (letterbox px),
                           col 4:   confidence (0-1, already sigmoid'd),
                           col 5:   class_id (integer 0-5),
                           col 6-37: 32 mask coefficients
  output1: (1, 32, 160, 160) — prototype masks

ROS parameters:
  confidence   (double, default 0.5)
"""

import os
import cv2
import numpy as np
import onnxruntime as ort
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from vision_msgs.msg import (
    BoundingBox2D,
    Detection2D,
    Detection2DArray,
    ObjectHypothesisWithPose,
)

MODEL_PATH = "/workspace/models/yolo26n-seg-navier.onnx"
INPUT_SIZE = 640
MASK_PROTO_SIZE = 160


class VisionNode(Node):
    def __init__(self):
        super().__init__("vision_node")

        self.declare_parameter("confidence", 0.5)
        self._confidence = self.get_parameter("confidence").get_parameter_value().double_value

        self._available = False

        if not os.path.isfile(MODEL_PATH):
            self.get_logger().error(f"Model not found at {MODEL_PATH}")
            return

        self.session = ort.InferenceSession(
            MODEL_PATH, providers=["CPUExecutionProvider"]
        )
        self._input_name = self.session.get_inputs()[0].name
        self.bridge = CvBridge()

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.det_pub = self.create_publisher(Detection2DArray, "/yolo/detections", qos)
        self.mask_pub = self.create_publisher(Image, "/yolo/seg_mask", qos)
        self.create_subscription(Image, "/front_camera_driver/image_raw", self._cb, qos)

        self._available = True
        self.get_logger().info("Vision node ready")

    def _preprocess(self, frame):
        """Letterbox resize to INPUT_SIZE x INPUT_SIZE, normalise to [0,1], NCHW."""
        h, w = frame.shape[:2]
        scale = INPUT_SIZE / max(h, w)
        nh, nw = int(h * scale), int(w * scale)
        canvas = np.zeros((INPUT_SIZE, INPUT_SIZE, 3), dtype=np.uint8)
        canvas[:nh, :nw] = cv2.resize(frame, (nw, nh))
        blob = canvas.astype(np.float32) / 255.0
        return blob.transpose(2, 0, 1)[np.newaxis], scale, (h, w)  # (1, 3, H, W)

    def _decode_mask(self, coeffs, protos, scale, orig_hw):
        """Decode a single instance mask to original image resolution.

        coeffs: (32,) mask coefficients for one detection
        protos: (32, 160, 160) prototype masks
        scale:  letterbox scale factor (INPUT_SIZE / max(orig_h, orig_w))
        orig_hw: (h, w) of the original camera frame

        Returns a binary uint8 mask of shape (orig_h, orig_w).
        """
        # (32,) @ (32, 160*160) -> (160, 160)
        mask = (coeffs @ protos.reshape(32, -1)).reshape(MASK_PROTO_SIZE, MASK_PROTO_SIZE)
        mask = 1.0 / (1.0 + np.exp(-mask))  # sigmoid

        # The mask covers the letterboxed region — crop to the scaled image area
        orig_h, orig_w = orig_hw
        nh, nw = int(orig_h * scale), int(orig_w * scale)
        # proto coords map to letterbox coords via scale factor MASK_PROTO_SIZE/INPUT_SIZE
        proto_scale = MASK_PROTO_SIZE / INPUT_SIZE
        mh, mw = int(nh * proto_scale), int(nw * proto_scale)
        mask = mask[:mh, :mw]

        # Resize to original frame resolution
        mask = cv2.resize(mask, (orig_w, orig_h), interpolation=cv2.INTER_LINEAR)
        return (mask > 0.5).astype(np.uint8)

    def _postprocess(self, raw, scale, orig_hw):
        """Decode YOLO26 NMS-free output.

        Returns list of (x1, y1, x2, y2, score, class_id, mask_uint8).
        """
        preds = raw[0][0]           # (300, 38)
        protos = raw[1][0]          # (32, 160, 160)

        scores = preds[:, 4]
        keep = scores > self._confidence
        if not keep.any():
            return []

        boxes = preds[keep, :4] / scale
        scores = scores[keep]
        class_ids = preds[keep, 5].astype(int)
        coeffs = preds[keep, 6:38]  # (N, 32)

        results = []
        for i in range(len(scores)):
            mask = self._decode_mask(coeffs[i], protos, scale, orig_hw)
            results.append((boxes[i], scores[i], class_ids[i], mask))
        return results

    def _cb(self, img_msg):
        if not self._available:
            return
        frame = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")
        blob, scale, orig_hw = self._preprocess(frame)
        outputs = self.session.run(None, {self._input_name: blob})
        detections = self._postprocess(outputs, scale, orig_hw)

        det_msg = Detection2DArray()
        det_msg.header = img_msg.header

        orig_h, orig_w = orig_hw
        seg_map = np.zeros((orig_h, orig_w), dtype=np.uint8)

        for idx, ((x1, y1, x2, y2), score, class_id, mask) in enumerate(detections):
            det = Detection2D()
            det.header = img_msg.header
            det.bbox = BoundingBox2D(size_x=float(x2 - x1), size_y=float(y2 - y1))
            det.bbox.center.position.x = float((x1 + x2) / 2)
            det.bbox.center.position.y = float((y1 + y2) / 2)
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = str(class_id)
            hyp.hypothesis.score = float(score)
            det.results.append(hyp)
            det_msg.detections.append(det)

            # detection index + 1 so background stays 0; later detections overwrite
            # earlier ones at overlapping pixels (acceptable for buoy use case)
            seg_map[mask > 0] = idx + 1

        self.det_pub.publish(det_msg)

        mask_msg = self.bridge.cv2_to_imgmsg(seg_map, encoding="mono8")
        mask_msg.header = img_msg.header
        self.mask_pub.publish(mask_msg)


def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
