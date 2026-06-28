"""ROS 2 node: Phase-1 spatial semantic map builder.

Subscribes to the per-frame ``DetectionArray`` stream from ``semantic_perception``,
fuses each detection into a :class:`~semantic_store.store.SpatialSemanticStore`
(spatial + label association), and keeps the best image crop per consolidated entity.
On a ``BuildSemanticMap`` request it runs the batch enrichment pass (VLM description +
image embedding + region clustering) and persists the map to disk. Optionally
publishes the growing map (``SemanticObjectArray`` + RViz markers) during exploration.

This is the thin ROS shell; the fusion logic is ``semantic_store`` and the
conversions/enrichment are pure modules in this package.
"""
from __future__ import annotations

import os

import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node

from semantic_nav_msgs.msg import DetectionArray, SemanticObjectArray
from semantic_nav_msgs.srv import BuildSemanticMap
from visualization_msgs.msg import MarkerArray

from semantic_store.store import SpatialSemanticStore

from semantic_mapping import conversions
from semantic_mapping.enrichment import MockDescriber, MockEmbedder, enrich_store

DEFAULT_MAP_PATH = "~/.semantic_nav/semantic_map.json"


class MappingNode(Node):
    def __init__(self):
        super().__init__("mapping_node")

        self.declare_parameter("detections_topic", "/semantic/detections")
        self.declare_parameter("semantic_map_topic", "/semantic/map")
        self.declare_parameter("markers_topic", "/semantic/map_markers")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("map_output_path", DEFAULT_MAP_PATH)
        self.declare_parameter("assoc_radius", 0.5)
        self.declare_parameter("embedding_sim_threshold", 0.85)
        self.declare_parameter("describer", "mock")      # mock | ollama | none
        self.declare_parameter("embedder", "mock")       # mock | clip  | none
        self.declare_parameter("embedding_dim", 512)
        self.declare_parameter("region_eps", 1.5)
        self.declare_parameter("publish_markers", True)
        self.declare_parameter("publish_period_sec", 1.0)
        # Real-backend knobs (used only when describer/embedder are not mock/none).
        self.declare_parameter("vlm_model", "moondream")
        self.declare_parameter("clip_model", "ViT-B-32")
        self.declare_parameter("ollama_host", "http://localhost:11434")

        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.map_frame = g("map_frame")
        self.map_output_path = os.path.expanduser(g("map_output_path"))
        self.region_eps = float(g("region_eps"))

        self.bridge = CvBridge()
        self.store = SpatialSemanticStore(
            assoc_radius=float(g("assoc_radius")),
            embedding_sim_threshold=float(g("embedding_sim_threshold")))
        self.crops_by_id = {}
        self._crop_conf = {}

        self.describer = self._make_describer(g("describer"))
        self.embedder = self._make_embedder(g("embedder"), int(g("embedding_dim")))

        self.map_pub = self.create_publisher(
            SemanticObjectArray, g("semantic_map_topic"), 10)
        self.marker_pub = (
            self.create_publisher(MarkerArray, g("markers_topic"), 10)
            if bool(g("publish_markers")) else None
        )
        self.create_subscription(
            DetectionArray, g("detections_topic"), self._on_detections, 10)
        self.build_srv = self.create_service(
            BuildSemanticMap, "~/build_semantic_map", self._on_build)

        period = float(g("publish_period_sec"))
        if period > 0:
            self.create_timer(period, self._publish_map)

        self.get_logger().info(
            f"mapping_node up: describer={g('describer')}, embedder={g('embedder')}, "
            f"map_frame={self.map_frame}, output={self.map_output_path}")

    # --- backend factory (lazy import for real backends) -----------------

    def _make_describer(self, name):
        if name in ("none", ""):
            return None
        if name == "mock":
            return MockDescriber()
        if name == "ollama":
            from semantic_mapping.ollama_describer import OllamaDescriber
            return OllamaDescriber(
                model=self.get_parameter("vlm_model").value,
                host=self.get_parameter("ollama_host").value)
        raise ValueError(f"unknown describer backend '{name}'")

    def _make_embedder(self, name, dim):
        if name in ("none", ""):
            return None
        if name == "mock":
            return MockEmbedder(dim=dim)
        if name == "clip":
            from semantic_mapping.clip_embedder import ClipEmbedder
            return ClipEmbedder(model=self.get_parameter("clip_model").value)
        raise ValueError(f"unknown embedder backend '{name}'")

    # --- live capture ----------------------------------------------------

    def _on_detections(self, msg: DetectionArray):
        for det in msg.detections:
            entity = self.store.add_observation(conversions.detection_to_observation(det))
            self._retain_crop(entity.id, det)

    def _retain_crop(self, entity_id: int, det) -> None:
        """Keep the highest-confidence crop seen for this entity (for batch enrich)."""
        if det.crop.width == 0 or det.crop.height == 0:
            return
        if det.confidence <= self._crop_conf.get(entity_id, -1.0):
            return
        try:
            crop = self.bridge.imgmsg_to_cv2(det.crop, desired_encoding="bgr8")
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f"crop conversion failed: {e}")
            return
        self.crops_by_id[entity_id] = crop
        self._crop_conf[entity_id] = float(det.confidence)

    # --- batch enrichment + persistence ----------------------------------

    def _on_build(self, request, response):
        enriched = enrich_store(
            self.store, self.crops_by_id,
            describer=self.describer, embedder=self.embedder,
            region_eps=self.region_eps)
        path = os.path.expanduser(request.output_path) if request.output_path \
            else self.map_output_path
        try:
            self.store.save(path)
        except OSError as e:
            self.get_logger().error(f"failed to save semantic map to {path}: {e}")
            response.success = False
            response.object_count = len(self.store.entities)
            response.path = path
            return response

        self.get_logger().info(
            f"semantic map: {len(self.store.entities)} entities "
            f"({enriched} enriched) -> {path}")
        response.success = True
        response.object_count = len(self.store.entities)
        response.path = path
        return response

    # --- live visualization ----------------------------------------------

    def _publish_map(self):
        objects = [conversions.entity_to_msg(e, self.map_frame)
                   for e in self.store.entities]
        out = SemanticObjectArray()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.map_frame
        out.objects = objects
        self.map_pub.publish(out)
        if self.marker_pub is not None:
            self.marker_pub.publish(conversions.objects_to_markers(objects, self.map_frame))


def main(args=None):
    rclpy.init(args=args)
    node = MappingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
