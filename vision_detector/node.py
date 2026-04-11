"""YOLO26n-seg inference node using ONNX Runtime (CPU).

Subscribes to /image_raw, publishes Detection2DArray on /yolo/detections.
The model is loaded from MODEL_PATH (bind-mounted at runtime).

Output tensor layout (standard ultralytics segmentation ONNX export):
  output0: (1, 4 + nc + 32, num_anchors)  — box + class scores + mask coeff
  output1: (1, 32, mask_h, mask_w)        — prototype masks (unused here)
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

MODEL_PATH = "/models/yolo26n-seg-navier.onnx"
CONFIDENCE = 0.5
NMS_IOU = 0.45
INPUT_SIZE = 640


class VisionNode(Node):
    def __init__(self):
        super().__init__("vision_node")

        self._available = False

        if not os.path.isfile(MODEL_PATH):
            self.get_logger().warn(
                f"Model not found at {MODEL_PATH} — vision_node running in degraded mode (no detections published)"
            )
            return

        self.session = ort.InferenceSession(
            MODEL_PATH, providers=["CPUExecutionProvider"]
        )
        self._input_name = self.session.get_inputs()[0].name
        # Auto-detect output layout: (1, 4+nc+32, anchors) vs (1, anchors, 4+nc+32)
        s = self.session.get_outputs()[0].shape
        self._feat_first = s[1] < s[2]  # True → (1, feats, anchors)
        self._nc = (s[1] if self._feat_first else s[2]) - 36
        self.bridge = CvBridge()

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.det_pub = self.create_publisher(Detection2DArray, "/yolo/detections", qos)
        self.create_subscription(Image, "/image_raw", self._cb, qos)

        self._available = True
        self.get_logger().info(f"Vision node ready — {self._nc} classes")

    def _preprocess(self, frame):
        """Letterbox resize to INPUT_SIZE x INPUT_SIZE, normalise to [0,1], NCHW."""
        h, w = frame.shape[:2]
        scale = INPUT_SIZE / max(h, w)
        nh, nw = int(h * scale), int(w * scale)
        canvas = np.zeros((INPUT_SIZE, INPUT_SIZE, 3), dtype=np.uint8)
        canvas[:nh, :nw] = cv2.resize(frame, (nw, nh))
        blob = canvas.astype(np.float32) / 255.0
        return blob.transpose(2, 0, 1)[np.newaxis], scale  # (1, 3, H, W)

    def _postprocess(self, raw, scale):
        """Decode raw output tensor, apply confidence filter + NMS.

        Returns list of (x1, y1, x2, y2, score, class_id) in original image coords.
        """
        preds = raw[0][0]  # strip batch dim
        if self._feat_first:
            preds = preds.T  # (anchors, 4+nc+32)
        # else already (anchors, 4+nc+32)
        box_cxcywh = preds[:, :4]
        class_scores = preds[:, 4 : 4 + self._nc]

        class_ids = np.argmax(class_scores, axis=1)
        confidences = class_scores[np.arange(len(class_scores)), class_ids]

        keep = confidences > CONFIDENCE
        if not keep.any():
            return []

        box_cxcywh = box_cxcywh[keep]
        confidences = confidences[keep]
        class_ids = class_ids[keep]

        # cx, cy, w, h (letterbox coords) -> x1, y1, x2, y2 (original coords)
        cx, cy, bw, bh = box_cxcywh.T
        x1 = (cx - bw / 2) / scale
        y1 = (cy - bh / 2) / scale
        x2 = (cx + bw / 2) / scale
        y2 = (cy + bh / 2) / scale
        boxes = np.stack([x1, y1, x2, y2], axis=1)

        indices = cv2.dnn.NMSBoxes(
            boxes.tolist(), confidences.tolist(), CONFIDENCE, NMS_IOU
        )
        if len(indices) == 0:
            return []

        return [
            (boxes[i], float(confidences[i]), int(class_ids[i]))
            for i in indices.flatten()
        ]

    def _cb(self, img_msg):
        if not self._available:
            return
        frame = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")
        blob, scale = self._preprocess(frame)
        outputs = self.session.run(None, {self._input_name: blob})
        detections = self._postprocess(outputs, scale)

        msg = Detection2DArray()
        msg.header = img_msg.header
        for (x1, y1, x2, y2), score, class_id in detections:
            det = Detection2D()
            det.header = msg.header
            det.bbox = BoundingBox2D(size_x=float(x2 - x1), size_y=float(y2 - y1))
            det.bbox.center.position.x = float((x1 + x2) / 2)
            det.bbox.center.position.y = float((y1 + y2) / 2)
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = str(class_id)
            hyp.hypothesis.score = score
            det.results.append(hyp)
            msg.detections.append(det)

        self.det_pub.publish(msg)


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
