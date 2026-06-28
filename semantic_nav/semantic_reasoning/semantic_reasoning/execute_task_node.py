"""ROS 2 node: the ExecuteTask action server — the local (ollama) reasoning frontend.

Accepts a natural-language ``command``, runs the shared reasoning loop
(``agent.run_agent``) over the shared tool registry against ``RobotTools`` (real ROS
clients), streams progress as ExecuteTask feedback, and reports the outcome.

This is the first rclpy ActionServer in the workspace. Concurrency is the key risk:
the execute callback BLOCKS on a service call and a Nav2 action from inside itself, so
it runs under a MultiThreadedExecutor with the action server and all clients on a
ReentrantCallbackGroup, and every wait uses a future + threading.Event (never
client.call()/spin_until_future_complete). Otherwise the sub-call futures would never
be serviced and the callback would self-deadlock.
"""
from __future__ import annotations

import rclpy
import tf2_ros
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from nav2_msgs.action import NavigateToPose
from semantic_nav_msgs.action import ExecuteTask
from semantic_nav_msgs.srv import QuerySemanticMap

from semantic_reasoning.agent import DEFAULT_SYSTEM_PROMPT, MockAgentBackend, run_agent
from semantic_reasoning.robot_tools import RobotTools, _yaw_to_quat


class ExecuteTaskNode(Node):
    def __init__(self):
        super().__init__("execute_task_node")

        self.declare_parameter("backend", "mock")          # mock | ollama
        self.declare_parameter("query_service", "/map_server_node/query_semantic_map")
        self.declare_parameter("nav_action", "/navigate_to_pose")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("robot_base_frame", "base_link")
        self.declare_parameter("query_timeout_sec", 5.0)
        self.declare_parameter("nav_timeout_sec", 120.0)
        self.declare_parameter("max_steps", 8)
        self.declare_parameter("system_prompt", DEFAULT_SYSTEM_PROMPT)
        self.declare_parameter("ollama_model", "qwen2.5:7b")
        self.declare_parameter("ollama_host", "http://localhost:11434")
        self.declare_parameter("approach_standoff_m", 0.6)
        self.declare_parameter("costmap_topic", "/global_costmap/costmap")
        self.declare_parameter("costmap_free_threshold", 99)

        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.map_frame = g("map_frame")
        self.robot_base_frame = g("robot_base_frame")
        self.query_timeout = float(g("query_timeout_sec"))
        self.nav_timeout = float(g("nav_timeout_sec"))
        self.max_steps = int(g("max_steps"))
        self.system_prompt = g("system_prompt")
        self.approach_standoff = float(g("approach_standoff_m"))
        self.costmap_free_threshold = int(g("costmap_free_threshold"))

        self.backend = self._make_backend(g("backend"))

        # One shared reentrant group: the action server's execute callback blocks on
        # the clients below, so their responses must be serviceable concurrently.
        self.cb = ReentrantCallbackGroup()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.query_client = self.create_client(
            QuerySemanticMap, g("query_service"), callback_group=self.cb)
        self.nav_client = ActionClient(
            self, NavigateToPose, g("nav_action"), callback_group=self.cb)

        # Latest global costmap, used to validate/repair object approach poses. Nav2
        # publishes it latched (transient_local), so match that to get the current map.
        self._costmap = None
        costmap_qos = QoSProfile(
            depth=1, history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            OccupancyGrid, g("costmap_topic"), self._on_costmap, costmap_qos,
            callback_group=self.cb)

        self.action_server = ActionServer(
            self, ExecuteTask, "~/execute_task",
            execute_callback=self._execute,
            goal_callback=lambda goal: GoalResponse.ACCEPT,
            cancel_callback=lambda goal_handle: CancelResponse.ACCEPT,
            callback_group=self.cb)

        self.get_logger().info(
            f"execute_task_node up: backend={g('backend')}, "
            f"query_service={g('query_service')}, nav_action={g('nav_action')}")

    def _on_costmap(self, msg: OccupancyGrid):
        self._costmap = msg

    def _make_backend(self, name):
        if name == "mock":
            return MockAgentBackend()
        if name == "ollama":
            from semantic_reasoning.ollama_backend import OllamaAgentBackend
            return OllamaAgentBackend(
                model=self.get_parameter("ollama_model").value,
                host=self.get_parameter("ollama_host").value)
        raise ValueError(f"unknown reasoning backend '{name}'")

    def _make_feedback_fn(self, goal_handle: ServerGoalHandle):
        def feedback_fn(status, reasoning_step, goal):
            fb = ExecuteTask.Feedback()
            fb.status = status
            fb.reasoning_step = reasoning_step
            if goal is not None:
                ps = PoseStamped()
                ps.header.frame_id = self.map_frame
                ps.header.stamp = self.get_clock().now().to_msg()
                ps.pose.position.x, ps.pose.position.y = float(goal[0]), float(goal[1])
                _, _, qz, qw = _yaw_to_quat(float(goal[2]))
                ps.pose.orientation.z, ps.pose.orientation.w = qz, qw
                fb.current_goal = ps
            goal_handle.publish_feedback(fb)
        return feedback_fn

    def _execute(self, goal_handle: ServerGoalHandle):
        command = goal_handle.request.command
        self.get_logger().info(f"ExecuteTask: '{command}'")
        feedback_fn = self._make_feedback_fn(goal_handle)

        ctx = RobotTools(
            self, query_client=self.query_client, nav_client=self.nav_client,
            tf_buffer=self.tf_buffer, goal_handle=goal_handle,
            map_frame=self.map_frame, robot_base_frame=self.robot_base_frame,
            query_timeout=self.query_timeout, nav_timeout=self.nav_timeout,
            feedback_fn=feedback_fn,
            approach_standoff=self.approach_standoff,
            costmap_getter=lambda: self._costmap,
            costmap_free_threshold=self.costmap_free_threshold)

        result = run_agent(
            self.backend, ctx, command,
            system_prompt=self.system_prompt, max_steps=self.max_steps,
            feedback=feedback_fn)

        out = ExecuteTask.Result()
        out.success = result.success
        out.summary = result.summary

        if goal_handle.is_cancel_requested:
            ctx.cancel_active_nav()
            goal_handle.canceled()
        elif result.success:
            goal_handle.succeed()
        else:
            goal_handle.abort()

        self.get_logger().info(f"ExecuteTask done: success={out.success}, summary='{out.summary}'")
        return out


def main(args=None):
    rclpy.init(args=args)
    node = ExecuteTaskNode()
    executor = MultiThreadedExecutor(num_threads=4)
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
