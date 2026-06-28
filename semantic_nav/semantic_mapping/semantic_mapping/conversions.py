"""Conversions between ROS messages and ``semantic_store`` records.

These are the only place that knows both vocabularies: the per-frame ``Detection``
message and the store's ``Observation`` (Phase 1), and the store's ``SemanticEntity``
and the ``SemanticObject`` message (Phase 2). Kept ROS-free of rclpy — they only
construct/read message *types*, so they unit-test without a running node.
"""
from __future__ import annotations

from typing import Optional

import numpy as np

from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point, Vector3
from semantic_nav_msgs.msg import SemanticObject
from visualization_msgs.msg import Marker, MarkerArray

from semantic_store.schema import Observation, SemanticEntity

_NANOSEC = 1_000_000_000


def stamp_to_seconds(stamp) -> float:
    """builtin_interfaces/Time (or any obj with .sec/.nanosec) -> float seconds."""
    return float(stamp.sec) + float(stamp.nanosec) / _NANOSEC


def seconds_to_stamp(t: float) -> Time:
    """Float seconds -> builtin_interfaces/Time."""
    sec = int(t)
    nanosec = int(round((t - sec) * _NANOSEC))
    if nanosec >= _NANOSEC:          # rounding spilled into the next second
        sec += 1
        nanosec -= _NANOSEC
    return Time(sec=sec, nanosec=nanosec)


def detection_to_observation(det) -> Observation:
    """One per-frame ``Detection`` -> one raw ``Observation`` for the store.

    ``embedding``/``size`` are left unset; they are filled later in the batch
    enrichment pass, not per frame.
    """
    return Observation(
        label=det.label,
        position=[det.position.x, det.position.y, det.position.z],
        confidence=float(det.confidence),
        stamp=stamp_to_seconds(det.header.stamp),
    )


def entity_to_msg(e: SemanticEntity, frame_id: str) -> SemanticObject:
    """A consolidated ``SemanticEntity`` -> a ``SemanticObject`` message."""
    msg = SemanticObject()
    msg.id = int(e.id)
    msg.label = e.label
    msg.aliases = list(e.aliases)
    msg.description = e.description
    msg.region = e.region

    msg.pose.header.frame_id = frame_id
    msg.pose.pose.position = Point(
        x=float(e.position[0]), y=float(e.position[1]), z=float(e.position[2]))
    msg.pose.pose.orientation.w = 1.0

    if e.size is None:
        msg.size = Vector3(x=0.0, y=0.0, z=0.0)
    else:
        msg.size = Vector3(x=float(e.size[0]), y=float(e.size[1]), z=float(e.size[2]))

    msg.confidence = float(e.confidence)
    msg.observation_count = int(e.observation_count)
    msg.last_seen = seconds_to_stamp(e.last_seen)
    msg.embedding = [] if e.embedding is None else [float(v) for v in e.embedding]
    return msg


def objects_to_markers(objects, frame_id: str, *, label_height: float = 0.25) -> MarkerArray:
    """Build an RViz ``MarkerArray`` (a sphere + a floating text label per object).

    Shared by the Phase-1 builder (live map) and the Phase-2 server (loaded map).
    """
    markers = MarkerArray()
    for i, obj in enumerate(objects):
        pos = obj.pose.pose.position

        sphere = Marker()
        sphere.header.frame_id = frame_id
        sphere.ns = "semantic_objects"
        sphere.id = i
        sphere.type = Marker.SPHERE
        sphere.action = Marker.ADD
        sphere.pose.position = Point(x=pos.x, y=pos.y, z=pos.z)
        sphere.pose.orientation.w = 1.0
        sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.2
        sphere.color.r, sphere.color.g, sphere.color.b, sphere.color.a = 0.1, 0.6, 1.0, 0.9
        markers.markers.append(sphere)

        text = Marker()
        text.header.frame_id = frame_id
        text.ns = "semantic_labels"
        text.id = i
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position = Point(x=pos.x, y=pos.y, z=pos.z + label_height)
        text.pose.orientation.w = 1.0
        text.scale.z = 0.2
        text.color.r = text.color.g = text.color.b = text.color.a = 1.0
        text.text = obj.label
        markers.markers.append(text)
    return markers


def query_kwargs_from_request(req) -> dict:
    """Normalize a ``QuerySemanticMap`` request into ``store.query`` kwargs.

    Empty strings, a non-positive radius, and ``top_k == 0`` mean "unset" -> ``None``.
    ``near`` is only honoured when ``radius`` is positive.
    """
    radius: Optional[float] = req.radius if req.radius and req.radius > 0 else None
    near = ([req.near.x, req.near.y, req.near.z] if radius is not None else None)
    return {
        "label": req.label or None,
        "region": req.region or None,
        "near": near,
        "radius": radius,
        "top_k": int(req.top_k) if req.top_k and req.top_k > 0 else None,
    }
