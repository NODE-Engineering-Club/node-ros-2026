import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose2D
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from ultralytics import YOLO
from vision_msgs.msg import BoundingBox2D, Detection2D, Detection2DArray, ObjectHypothesisWithPose

MODEL = "/models/yolo26n-seg-navier.onnx"
CONFIDENCE = 0.5


class VisionNode(Node):
    def __init__(self):
        super().__init__("vision_node")

        self.model = YOLO(MODEL)
        self.bridge = CvBridge()

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
        self.det_pub = self.create_publisher(Detection2DArray, "/yolo/detections", qos)
        self.img_pub = self.create_publisher(Image, "/yolo/image_annotated", qos)
        self.create_subscription(Image, "/image_raw", self._cb, qos)

    def _cb(self, img_msg):
        frame = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")
        results = self.model.predict(source=frame, conf=CONFIDENCE, verbose=False)

        msg = Detection2DArray()
        msg.header = img_msg.header

        for result in results:
            for box in result.boxes:
                x1, y1, x2, y2 = box.xyxy[0].tolist()

                det = Detection2D()
                det.header = msg.header
                det.bbox = BoundingBox2D(
                    center=Pose2D(x=(x1 + x2) / 2, y=(y1 + y2) / 2, theta=0.0),
                    size_x=float(x2 - x1),
                    size_y=float(y2 - y1),
                )
                hyp = ObjectHypothesisWithPose()
                hyp.hypothesis.class_id = result.names[int(box.cls[0])]
                hyp.hypothesis.score = float(box.conf[0])
                det.results.append(hyp)
                msg.detections.append(det)

        self.det_pub.publish(msg)
        self.img_pub.publish(self.bridge.cv2_to_imgmsg(results[0].plot() if results else frame, encoding="bgr8"))


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
