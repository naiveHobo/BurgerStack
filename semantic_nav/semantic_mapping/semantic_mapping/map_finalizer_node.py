"""ROS 2 node: the Phase-1 -> Phase-2 handoff orchestrator.

One mechanism, two triggers: it saves the Nav2 occupancy map (via the map_saver
``SaveMap`` service) AND persists the enriched semantic map (via ``mapping_node``'s
``BuildSemanticMap`` service). Fire it automatically when the explorer finishes
(``auto_finalize:=true`` -> subscribe to ``exploration_complete``) or manually via the
``~/finalize`` Trigger service. Idempotent (one-shot latch).

Like execute_task_node, its callbacks block on service calls, so it runs under a
MultiThreadedExecutor with the service/clients on a ReentrantCallbackGroup and waits
on futures via threading.Event + add_done_callback (never client.call()).
"""
from __future__ import annotations

import os
import threading

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from nav2_msgs.srv import SaveMap
from std_msgs.msg import Empty
from std_srvs.srv import Trigger

from semantic_nav_msgs.srv import BuildSemanticMap


class MapFinalizerNode(Node):
    def __init__(self):
        super().__init__("map_finalizer")

        self.declare_parameter("semantic_map_path", "~/.semantic_nav/semantic_map.json")
        self.declare_parameter("occupancy_map_path", "~/.semantic_nav/occupancy_map")
        self.declare_parameter("build_service", "/mapping_node/build_semantic_map")
        self.declare_parameter("save_map_service", "/map_saver/save_map")
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("completion_topic", "/exploration_complete")
        self.declare_parameter("auto_finalize", False)
        self.declare_parameter("service_timeout_sec", 30.0)
        self.declare_parameter("free_thresh", 0.25)
        self.declare_parameter("occupied_thresh", 0.65)

        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.semantic_map_path = os.path.expanduser(g("semantic_map_path"))
        self.occupancy_map_path = os.path.expanduser(g("occupancy_map_path"))
        self.map_topic = g("map_topic")
        self.timeout = float(g("service_timeout_sec"))
        self.free_thresh = float(g("free_thresh"))
        self.occupied_thresh = float(g("occupied_thresh"))

        self._done = False
        self._lock = threading.Lock()
        self.cb = ReentrantCallbackGroup()

        self.build_client = self.create_client(
            BuildSemanticMap, g("build_service"), callback_group=self.cb)
        self.save_client = self.create_client(
            SaveMap, g("save_map_service"), callback_group=self.cb)
        self.create_service(Trigger, "~/finalize", self._on_trigger, callback_group=self.cb)

        if bool(g("auto_finalize")):
            # The explorer publishes the completion event latched (reliable +
            # transient_local, KeepLast(1)). Match it so we still receive the
            # one-shot event across discovery races or a finalizer restart, not
            # just while both ends happen to be connected.
            completion_qos = QoSProfile(
                depth=1, history=HistoryPolicy.KEEP_LAST,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(
                Empty, g("completion_topic"), self._on_complete,
                completion_qos, callback_group=self.cb)
            self.get_logger().info(
                f"auto-finalize armed on '{g('completion_topic')}'")

        self.get_logger().info(
            f"map_finalizer up: occupancy->{self.occupancy_map_path}.yaml, "
            f"semantic->{self.semantic_map_path}")

    @staticmethod
    def _wait(future, timeout):
        ev = threading.Event()
        future.add_done_callback(lambda _f: ev.set())
        return future.result() if ev.wait(timeout) else None

    def _on_trigger(self, request, response):
        ok, msg = self.finalize()
        response.success = ok
        response.message = msg
        return response

    def _on_complete(self, _msg: Empty):
        self.get_logger().info("exploration_complete received -> finalizing maps")
        self.finalize()

    def finalize(self):
        with self._lock:
            if self._done:
                return True, "already finalized"
            # Both savers write into these dirs; create them before either runs
            # (map_saver_server will not create a missing output directory).
            for path in (self.occupancy_map_path, self.semantic_map_path):
                os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
            occ_ok = self._save_occupancy_map()
            sem_ok, detail = self._build_semantic_map()
            self._done = occ_ok and sem_ok
            msg = (f"occupancy {'OK' if occ_ok else 'FAILED'} "
                   f"({self.occupancy_map_path}.yaml); semantic {detail}")
            self.get_logger().info(f"finalize: {msg}")
            return (occ_ok and sem_ok), msg

    def _save_occupancy_map(self) -> bool:
        if not self.save_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("map_saver SaveMap service unavailable")
            return False
        req = SaveMap.Request()
        req.map_topic = self.map_topic
        req.map_url = self.occupancy_map_path
        req.image_format = "pgm"
        req.map_mode = "trinary"
        req.free_thresh = self.free_thresh
        req.occupied_thresh = self.occupied_thresh
        resp = self._wait(self.save_client.call_async(req), self.timeout)
        return bool(resp is not None and resp.result)

    def _build_semantic_map(self):
        if not self.build_client.wait_for_service(timeout_sec=5.0):
            return False, "BuildSemanticMap service unavailable"
        os.makedirs(os.path.dirname(self.semantic_map_path) or ".", exist_ok=True)
        req = BuildSemanticMap.Request()
        req.output_path = self.semantic_map_path
        resp = self._wait(self.build_client.call_async(req), self.timeout)
        if resp is None:
            return False, "FAILED (no response)"
        return bool(resp.success), f"{'OK' if resp.success else 'FAILED'} ({resp.object_count} objects -> {resp.path})"


def main(args=None):
    rclpy.init(args=args)
    node = MapFinalizerNode()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
