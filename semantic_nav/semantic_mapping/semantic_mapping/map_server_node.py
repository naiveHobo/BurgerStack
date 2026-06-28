"""ROS 2 node: Phase-2 semantic map server.

Loads a semantic map persisted by the Phase-1 builder, serves ``QuerySemanticMap``
(the retrieval interface the agentic reasoner drives), and publishes the map as a
latched ``SemanticObjectArray`` + RViz markers. Text queries are embedded with the
same ``Embedder`` family the builder used, so a real CLIP places the query text in the
same space as the stored crop embeddings.
"""
from __future__ import annotations

import os

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile

from semantic_nav_msgs.msg import SemanticObjectArray
from semantic_nav_msgs.srv import QuerySemanticMap
from visualization_msgs.msg import MarkerArray

from semantic_store.store import SpatialSemanticStore

from semantic_mapping import conversions
from semantic_mapping.enrichment import MockEmbedder


def _latched(depth: int = 1) -> QoSProfile:
    qos = QoSProfile(depth=depth)
    qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
    return qos


class MapServerNode(Node):
    def __init__(self):
        super().__init__("map_server_node")

        self.declare_parameter("map_path", "~/.semantic_nav/semantic_map.json")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("semantic_map_topic", "/semantic/map")
        self.declare_parameter("markers_topic", "/semantic/map_markers")
        self.declare_parameter("query_service", "~/query_semantic_map")
        self.declare_parameter("embedder", "mock")       # mock | clip | none
        self.declare_parameter("embedding_dim", 512)
        self.declare_parameter("publish_markers", True)
        self.declare_parameter("default_top_k", 10)
        self.declare_parameter("clip_model", "ViT-B-32")

        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.map_frame = g("map_frame")
        self.default_top_k = int(g("default_top_k"))

        self.store = self._load_store(os.path.expanduser(g("map_path")))
        self.embedder = self._make_embedder(g("embedder"), int(g("embedding_dim")))

        self.create_service(
            QuerySemanticMap, g("query_service"), self._on_query)

        self.map_pub = self.create_publisher(
            SemanticObjectArray, g("semantic_map_topic"), _latched())
        self.marker_pub = (
            self.create_publisher(MarkerArray, g("markers_topic"), _latched())
            if bool(g("publish_markers")) else None
        )
        self._publish_map()

        self.get_logger().info(
            f"map_server_node up: {len(self.store.entities)} entities, "
            f"embedder={g('embedder')}, frame={self.map_frame}")

    def _load_store(self, path: str) -> SpatialSemanticStore:
        if not os.path.exists(path):
            self.get_logger().warn(
                f"semantic map not found at {path}; serving an empty map")
            return SpatialSemanticStore()
        return SpatialSemanticStore.load(path)

    def _make_embedder(self, name, dim):
        if name in ("none", ""):
            return None
        if name == "mock":
            return MockEmbedder(dim=dim)
        if name == "clip":
            from semantic_mapping.clip_embedder import ClipEmbedder
            return ClipEmbedder(model=self.get_parameter("clip_model").value)
        raise ValueError(f"unknown embedder backend '{name}'")

    def _on_query(self, request, response):
        kwargs = conversions.query_kwargs_from_request(request)
        if kwargs.get("top_k") is None:
            kwargs["top_k"] = self.default_top_k
        if request.text_query and self.embedder is not None:
            kwargs["text_embedding"] = self.embedder.embed_text(request.text_query)

        results = self.store.query(**kwargs)
        response.results = [conversions.entity_to_msg(e, self.map_frame) for e in results]
        self.get_logger().info(
            f"query (text='{request.text_query}', label='{request.label}', "
            f"region='{request.region}') -> {len(response.results)} results")
        return response

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
    node = MapServerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
