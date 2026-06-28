"""ROS-backed implementation of ``ToolContext``.

Wraps the rclpy clients the tools need — a ``QuerySemanticMap`` service client, a
``NavigateToPose`` action client, and a TF lookup for the robot pose — behind the
pure tool interface. All future-waiting uses ``threading.Event`` + ``add_done_callback``
(never ``client.call()`` / ``spin_until_future_complete``), so it is safe to call from
inside the action server's execute callback under a MultiThreadedExecutor with the
clients on a ReentrantCallbackGroup. See the execute_task_node for the executor setup.
"""
from __future__ import annotations

import math
import threading
from typing import Optional

from action_msgs.msg import GoalStatus
from rclpy.time import Time

from nav2_msgs.action import NavigateToPose
from semantic_nav_msgs.srv import QuerySemanticMap

from semantic_reasoning.tools import ToolContext

_STATUS_NAMES = {
    GoalStatus.STATUS_SUCCEEDED: "succeeded",
    GoalStatus.STATUS_CANCELED: "canceled",
    GoalStatus.STATUS_ABORTED: "aborted",
}


def _yaw_to_quat(yaw: float):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def _quat_to_yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class RobotTools(ToolContext):
    def __init__(self, node, *, query_client, nav_client, tf_buffer, goal_handle,
                 map_frame="map", robot_base_frame="base_link",
                 query_timeout=5.0, nav_timeout=120.0, feedback_fn=None):
        self.node = node
        self.query_client = query_client
        self.nav_client = nav_client
        self.tf_buffer = tf_buffer
        self.goal_handle = goal_handle          # the ExecuteTask goal handle
        self.map_frame = map_frame
        self.robot_base_frame = robot_base_frame
        self.query_timeout = float(query_timeout)
        self.nav_timeout = float(nav_timeout)
        self.feedback_fn = feedback_fn
        self._lock = threading.Lock()
        self._active_nav = None

    # --- future waiting (deadlock-safe) ----------------------------------

    @staticmethod
    def _wait(future, timeout):
        ev = threading.Event()
        future.add_done_callback(lambda _f: ev.set())
        return future.result() if ev.wait(timeout) else None

    # --- ToolContext: query ----------------------------------------------

    def query_semantic_map(self, text_query="", label="", region="",
                           near=None, radius=0.0, top_k=0):
        if not self.query_client.wait_for_service(timeout_sec=2.0):
            self.node.get_logger().warn("QuerySemanticMap service unavailable")
            return []
        req = QuerySemanticMap.Request()
        req.text_query = text_query or ""
        req.label = label or ""
        req.region = region or ""
        if near is not None:
            req.near.x = float(near.get("x", 0.0))
            req.near.y = float(near.get("y", 0.0))
            req.near.z = float(near.get("z", 0.0))
        req.radius = float(radius or 0.0)
        req.top_k = int(top_k or 0)

        resp = self._wait(self.query_client.call_async(req), self.query_timeout)
        if resp is None:
            return []
        return [self._object_to_dict(o) for o in resp.results]

    @staticmethod
    def _object_to_dict(o) -> dict:
        p = o.pose.pose.position
        return {
            "id": int(o.id), "label": o.label, "description": o.description,
            "region": o.region, "confidence": float(o.confidence),
            "position": {"x": float(p.x), "y": float(p.y), "z": float(p.z)},
        }

    # --- ToolContext: navigate -------------------------------------------

    def navigate_to_pose(self, x, y, yaw=0.0, frame="map"):
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            return {"success": False, "status": "no_nav_server"}

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = frame or self.map_frame
        goal.pose.header.stamp = self.node.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        _, _, qz, qw = _yaw_to_quat(float(yaw))
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        gh = self._wait(
            self.nav_client.send_goal_async(goal, feedback_callback=self._on_nav_feedback),
            5.0)
        if gh is None or not gh.accepted:
            return {"success": False, "status": "rejected"}

        with self._lock:
            self._active_nav = gh
        try:
            status = self._await_nav_result(gh)
        finally:
            with self._lock:
                self._active_nav = None
        return {"success": status == GoalStatus.STATUS_SUCCEEDED,
                "status": _STATUS_NAMES.get(status, "unknown")}

    def _await_nav_result(self, gh) -> int:
        """Wait for the nav result, polling so cancel/timeout stay responsive."""
        ev = threading.Event()
        result_future = gh.get_result_async()
        result_future.add_done_callback(lambda _f: ev.set())
        elapsed = 0.0
        while not ev.wait(0.5):
            if self.is_cancelled():
                self._wait(gh.cancel_goal_async(), 3.0)
                return GoalStatus.STATUS_CANCELED
            elapsed += 0.5
            if elapsed >= self.nav_timeout:
                self._wait(gh.cancel_goal_async(), 3.0)
                return GoalStatus.STATUS_ABORTED
        return result_future.result().status

    def _on_nav_feedback(self, msg):
        if self.feedback_fn is None:
            return
        try:
            d = msg.feedback.distance_remaining
            self.feedback_fn("navigating", f"distance_remaining={d:.2f} m", None)
        except Exception:  # noqa: BLE001 — feedback is best-effort
            pass

    # --- ToolContext: pose + cancellation --------------------------------

    def get_robot_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.robot_base_frame, Time())
        except Exception as e:  # noqa: BLE001
            self.node.get_logger().debug(f"pose TF lookup failed: {e}")
            return None
        t = tf.transform.translation
        return {"x": float(t.x), "y": float(t.y),
                "yaw": _quat_to_yaw(tf.transform.rotation)}

    def is_cancelled(self) -> bool:
        return self.goal_handle is not None and self.goal_handle.is_cancel_requested

    def cancel_active_nav(self) -> None:
        with self._lock:
            gh = self._active_nav
        if gh is not None:
            self._wait(gh.cancel_goal_async(), 3.0)
