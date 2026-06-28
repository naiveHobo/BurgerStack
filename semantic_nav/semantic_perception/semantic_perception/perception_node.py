"""ROS 2 node: RGB-D + detector -> map-frame 3D detections.

Synchronizes RGB and depth, runs a (pluggable, mockable) detector, deprojects each
detection to a 3D point using the cached camera intrinsics, transforms it from the
camera optical frame into the target frame (normally `map`) at the image timestamp,
crops the object's pixels, and publishes a DetectionArray. The deprojection math and
detector are pure (unit-tested); this node is the thin ROS shell around them.
"""
from __future__ import annotations

import numpy as np
import rclpy
import tf2_geometry_msgs  # noqa: F401  (registers do_transform_point for PointStamped)
import tf2_ros
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import TransformException
from visualization_msgs.msg import Marker, MarkerArray

import message_filters

from semantic_nav_msgs.msg import Detection, DetectionArray
from semantic_perception import deprojection
from semantic_perception.detector import MockDetector

DEFAULT_VOCABULARY = [
    "chair", "table", "desk", "door", "person", "monitor", "keyboard",
    "trash can", "fire extinguisher", "sofa", "potted plant", "bookshelf",
]


class PerceptionNode(Node):
    def __init__(self):
        super().__init__("perception_node")

        self.declare_parameter("rgb_topic", "/camera/image_raw")
        self.declare_parameter("depth_topic", "/camera/depth/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/depth/camera_info")
        self.declare_parameter("detections_topic", "/semantic/detections")
        self.declare_parameter("markers_topic", "/semantic/detection_markers")
        self.declare_parameter("target_frame", "map")
        self.declare_parameter("detector", "mock")            # mock | yolo_world
        self.declare_parameter("vocabulary", DEFAULT_VOCABULARY)
        self.declare_parameter("confidence_threshold", 0.3)
        self.declare_parameter("process_period_sec", 0.5)
        self.declare_parameter("depth_min", 0.1)
        self.declare_parameter("depth_max", 8.0)
        self.declare_parameter("sync_slop_sec", 0.1)
        self.declare_parameter("tf_timeout_sec", 0.1)
        self.declare_parameter("publish_markers", True)
        self.declare_parameter("mock_box_frac", 0.33)

        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.target_frame = g("target_frame")
        self.conf_thresh = float(g("confidence_threshold"))
        self.depth_min = float(g("depth_min"))
        self.depth_max = float(g("depth_max"))
        self.process_period = float(g("process_period_sec"))
        self.tf_timeout = float(g("tf_timeout_sec"))

        self.bridge = CvBridge()
        self.k = None
        self._last_stamp = None
        self.detector = self._make_detector(
            g("detector"), list(g("vocabulary")), float(g("mock_box_frac")))

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.det_pub = self.create_publisher(DetectionArray, g("detections_topic"), 10)
        self.marker_pub = (
            self.create_publisher(MarkerArray, g("markers_topic"), 10)
            if bool(g("publish_markers")) else None
        )

        self.create_subscription(
            CameraInfo, g("camera_info_topic"), self._on_camera_info, 10)
        rgb_sub = message_filters.Subscriber(
            self, Image, g("rgb_topic"), qos_profile=qos_profile_sensor_data)
        depth_sub = message_filters.Subscriber(
            self, Image, g("depth_topic"), qos_profile=qos_profile_sensor_data)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [rgb_sub, depth_sub], queue_size=10, slop=float(g("sync_slop_sec")))
        self.sync.registerCallback(self._on_rgbd)

        self.get_logger().info(
            f"perception_node up: detector={g('detector')}, "
            f"target_frame={self.target_frame}")

    def _make_detector(self, name, vocabulary, box_frac):
        if name == "mock":
            return MockDetector(box_frac=box_frac)
        if name in ("yolo_world", "openvocab"):
            from semantic_perception.openvocab_detector import OpenVocabDetector
            vocab = [v for v in vocabulary if v]
            return OpenVocabDetector(
                vocabulary=vocab, confidence_threshold=self.conf_thresh)
        raise ValueError(f"unknown detector backend '{name}'")

    def _on_camera_info(self, msg: CameraInfo):
        if self.k is None:
            self.k = list(msg.k)
            self.get_logger().info("camera intrinsics received")

    def _throttled(self, stamp) -> bool:
        now = Time.from_msg(stamp)
        if self._last_stamp is not None:
            dt = (now - self._last_stamp).nanoseconds * 1e-9
            if 0.0 <= dt < self.process_period:
                return True
        self._last_stamp = now
        return False

    def _on_rgbd(self, rgb_msg: Image, depth_msg: Image):
        if self.k is None or self._throttled(rgb_msg.header.stamp):
            return
        try:
            rgb = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
            depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="32FC1")
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f"cv_bridge conversion failed: {e}")
            return

        out = DetectionArray()
        out.header.stamp = rgb_msg.header.stamp
        out.header.frame_id = self.target_frame

        for d in self.detector.detect(rgb):
            if d.score < self.conf_thresh:
                continue
            optical = deprojection.deproject_detection(
                (d.x, d.y, d.width, d.height), depth, self.k,
                depth_min=self.depth_min, depth_max=self.depth_max)
            if optical is None:
                continue
            point = self._transform(optical, depth_msg.header)
            if point is None:
                continue
            det = Detection()
            det.header.stamp = rgb_msg.header.stamp
            det.header.frame_id = self.target_frame
            det.label = d.label
            det.confidence = float(d.score)
            det.position = point
            det.x, det.y = int(d.x), int(d.y)
            det.width, det.height = int(d.width), int(d.height)
            det.crop = self._crop(rgb, d, rgb_msg.header)
            out.detections.append(det)

        if out.detections:
            self.det_pub.publish(out)
            if self.marker_pub is not None:
                self.marker_pub.publish(self._markers(out))

    def _transform(self, optical, src_header):
        ps = PointStamped()
        ps.header = src_header  # camera optical frame + depth stamp
        ps.point.x, ps.point.y, ps.point.z = (
            float(optical[0]), float(optical[1]), float(optical[2]))
        try:
            tf = self.tf_buffer.lookup_transform(
                self.target_frame, src_header.frame_id,
                Time.from_msg(src_header.stamp),
                timeout=Duration(seconds=self.tf_timeout))
            return tf2_geometry_msgs.do_transform_point(ps, tf).point
        except TransformException as e:
            self.get_logger().debug(
                f"TF {self.target_frame} <- {src_header.frame_id} failed: {e}")
            return None

    def _crop(self, rgb, d, src_header) -> Image:
        patch = rgb[d.y:d.y + d.height, d.x:d.x + d.width]
        msg = self.bridge.cv2_to_imgmsg(np.ascontiguousarray(patch), encoding="bgr8")
        msg.header = src_header
        return msg

    def _markers(self, det_array: DetectionArray) -> MarkerArray:
        markers = MarkerArray()
        for i, det in enumerate(det_array.detections):
            sphere = Marker()
            sphere.header = det_array.header
            sphere.ns = "semantic_detections"
            sphere.id = i
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position = det.position
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.2
            sphere.color.r, sphere.color.g, sphere.color.b, sphere.color.a = (
                0.1, 0.9, 0.2, 0.9)
            sphere.lifetime = Duration(seconds=5.0).to_msg()
            markers.markers.append(sphere)

            text = Marker()
            text.header = det_array.header
            text.ns = "semantic_labels"
            text.id = i
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position = det.position
            text.pose.position.z += 0.25
            text.pose.orientation.w = 1.0
            text.scale.z = 0.2
            text.color.r = text.color.g = text.color.b = text.color.a = 1.0
            text.text = det.label
            text.lifetime = Duration(seconds=5.0).to_msg()
            markers.markers.append(text)
        return markers


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
