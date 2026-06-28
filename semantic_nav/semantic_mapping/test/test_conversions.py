"""msg <-> store-record conversions. These touch ROS message types (built, not
running), so the workspace must be sourced; no rclpy.init is needed."""
import numpy as np

from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point
from semantic_nav_msgs.msg import Detection
from semantic_nav_msgs.srv import QuerySemanticMap
from visualization_msgs.msg import Marker

from semantic_store.schema import SemanticEntity

from semantic_mapping.conversions import (
    detection_to_observation,
    entity_to_msg,
    objects_to_markers,
    query_kwargs_from_request,
    seconds_to_stamp,
    stamp_to_seconds,
)


# --- time helpers --------------------------------------------------------

def test_seconds_to_stamp_splits_sec_and_nanosec():
    stamp = seconds_to_stamp(2.5)
    assert stamp.sec == 2
    assert stamp.nanosec == 500_000_000


def test_stamp_to_seconds_reads_sec_and_nanosec():
    stamp = Time(sec=3, nanosec=250_000_000)
    assert stamp_to_seconds(stamp) == 3.25


def test_time_roundtrip():
    assert stamp_to_seconds(seconds_to_stamp(1234.75)) == 1234.75


# --- detection -> observation -------------------------------------------

def _detection(label="chair", x=1.0, y=2.0, z=3.0, conf=0.8, sec=10, nanosec=0):
    det = Detection()
    det.header.stamp = Time(sec=sec, nanosec=nanosec)
    det.label = label
    det.confidence = conf
    det.position = Point(x=x, y=y, z=z)
    return det


def test_detection_to_observation_maps_fields():
    obs = detection_to_observation(_detection(label="table", x=1.0, y=-2.0, z=0.5,
                                              conf=0.6, sec=7, nanosec=500_000_000))
    assert obs.label == "table"
    np.testing.assert_allclose(obs.position, [1.0, -2.0, 0.5])
    assert obs.confidence == 0.6
    assert obs.stamp == 7.5


def test_detection_to_observation_leaves_embedding_and_size_unset():
    obs = detection_to_observation(_detection())
    assert obs.embedding is None
    assert obs.size is None


# --- entity -> SemanticObject msg ---------------------------------------

def _entity(**kw):
    base = dict(id=4, label="sofa", position=np.array([1.0, 2.0, 0.3]),
                confidence=0.9, observation_count=5, last_seen=12.0)
    base.update(kw)
    return SemanticEntity(**base)


def test_entity_to_msg_maps_core_fields():
    e = _entity(aliases=["couch"], description="a comfy seat", region="region_0")
    msg = entity_to_msg(e, frame_id="map")
    assert msg.id == 4
    assert msg.label == "sofa"
    assert list(msg.aliases) == ["couch"]
    assert msg.description == "a comfy seat"
    assert msg.region == "region_0"
    assert msg.confidence == 0.9
    assert msg.observation_count == 5
    assert msg.last_seen.sec == 12
    assert msg.pose.header.frame_id == "map"
    np.testing.assert_allclose(
        [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z],
        [1.0, 2.0, 0.3])
    assert msg.pose.pose.orientation.w == 1.0


def test_entity_to_msg_embedding_none_is_empty_list():
    msg = entity_to_msg(_entity(embedding=None), frame_id="map")
    assert list(msg.embedding) == []


def test_entity_to_msg_embedding_is_serialized():
    msg = entity_to_msg(_entity(embedding=np.array([0.1, 0.2, 0.3])), frame_id="map")
    np.testing.assert_allclose(list(msg.embedding), [0.1, 0.2, 0.3])


def test_entity_to_msg_size_none_is_zero_vector():
    msg = entity_to_msg(_entity(size=None), frame_id="map")
    assert [msg.size.x, msg.size.y, msg.size.z] == [0.0, 0.0, 0.0]


def test_entity_to_msg_size_is_serialized():
    msg = entity_to_msg(_entity(size=np.array([0.5, 0.6, 0.7])), frame_id="map")
    np.testing.assert_allclose([msg.size.x, msg.size.y, msg.size.z], [0.5, 0.6, 0.7])


# --- QuerySemanticMap request -> store.query kwargs ----------------------

def test_query_kwargs_empty_request_is_all_none():
    kw = query_kwargs_from_request(QuerySemanticMap.Request())
    assert kw == {"label": None, "region": None, "near": None,
                  "radius": None, "top_k": None}


def test_query_kwargs_normalizes_filters():
    req = QuerySemanticMap.Request()
    req.label = "chair"
    req.region = "kitchen"
    req.top_k = 3
    kw = query_kwargs_from_request(req)
    assert kw["label"] == "chair"
    assert kw["region"] == "kitchen"
    assert kw["top_k"] == 3
    # No radius -> near is ignored even if present.
    assert kw["near"] is None
    assert kw["radius"] is None


def test_query_kwargs_near_only_when_radius_positive():
    req = QuerySemanticMap.Request()
    req.near = Point(x=1.0, y=2.0, z=0.0)
    req.radius = 2.5
    kw = query_kwargs_from_request(req)
    np.testing.assert_allclose(kw["near"], [1.0, 2.0, 0.0])
    assert kw["radius"] == 2.5


# --- SemanticObject[] -> RViz markers ------------------------------------

def test_objects_to_markers_two_markers_per_object():
    objs = [entity_to_msg(_entity(id=0, label="chair"), "map"),
            entity_to_msg(_entity(id=1, label="table"), "map")]
    markers = objects_to_markers(objs, frame_id="map")
    assert len(markers.markers) == 4  # one sphere + one text label per object
    spheres = [m for m in markers.markers if m.type == Marker.SPHERE]
    texts = [m for m in markers.markers if m.type == Marker.TEXT_VIEW_FACING]
    assert len(spheres) == 2 and len(texts) == 2
    assert {m.text for m in texts} == {"chair", "table"}
    assert all(m.header.frame_id == "map" for m in markers.markers)
