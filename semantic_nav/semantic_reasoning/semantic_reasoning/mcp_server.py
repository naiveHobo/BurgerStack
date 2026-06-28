"""MCP server frontend — the Claude side of the shared tool layer.

Exposes the SAME three tools as the ollama frontend (``query_semantic_map`` /
``navigate_to_pose`` / ``get_robot_pose``) over the Model Context Protocol, so Claude (Desktop
or Code) can drive the robot against a live Phase-2 stack. Both frontends sit on one
``ToolRegistry`` and one ``RobotTools`` implementation — Claude supplies its own reasoning loop,
so this process only serves tools; it does not run ``agent.run_agent``.

Architecture (validated): an rclpy node owning the query-service / Nav2-action / TF clients is
spun by a MultiThreadedExecutor on a daemon thread (same ReentrantCallbackGroup invariant as
``execute_task_node`` — required so ``RobotTools``' future waits get serviced). The MCP server
runs the asyncio event loop on the main thread; because ``registry.dispatch`` BLOCKS on ROS
futures, each tool call is offloaded with ``run_in_executor`` so it never stalls the MCP loop.

``mcp`` is imported inside ``amain`` (not at module top) so this module imports under the
default env without the ``mcp`` dependency. stdout is the MCP transport — all logging is on
stderr (rclpy's default), and nothing here writes to stdout.

Run (needs the `ai` pixi env + a live Phase-2 stack):
    pixi run -e ai ros2 run semantic_reasoning mcp_server
"""
from __future__ import annotations

import asyncio
import json
import threading

import rclpy
import tf2_ros
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from nav2_msgs.action import NavigateToPose
from semantic_nav_msgs.srv import QuerySemanticMap

from semantic_reasoning.robot_tools import RobotTools
from semantic_reasoning.tools import ToolRegistry


class ToolNode(Node):
    """Holds the ROS clients the MCP tools execute against (no action server here)."""

    def __init__(self):
        super().__init__("semantic_mcp_server")
        self.declare_parameter("query_service", "/map_server_node/query_semantic_map")
        self.declare_parameter("nav_action", "/navigate_to_pose")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("robot_base_frame", "base_link")
        self.declare_parameter("query_timeout_sec", 5.0)
        self.declare_parameter("nav_timeout_sec", 120.0)
        g = lambda n: self.get_parameter(n).value  # noqa: E731

        self.map_frame = g("map_frame")
        self.robot_base_frame = g("robot_base_frame")
        self.query_timeout = float(g("query_timeout_sec"))
        self.nav_timeout = float(g("nav_timeout_sec"))

        # Same shared reentrant group as execute_task_node: tool calls block on these
        # clients, so their responses must be serviceable concurrently by the executor.
        self.cb = ReentrantCallbackGroup()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.query_client = self.create_client(
            QuerySemanticMap, g("query_service"), callback_group=self.cb)
        self.nav_client = ActionClient(
            self, NavigateToPose, g("nav_action"), callback_group=self.cb)


async def amain():
    import mcp.server.stdio
    import mcp.types as types
    from mcp.server.lowlevel import NotificationOptions, Server
    from mcp.server.models import InitializationOptions

    rclpy.init()
    node = ToolNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    registry = ToolRegistry()
    # goal_handle=None: there is no ROS action goal here, so is_cancelled() is always False;
    # navigate_to_pose still self-bounds via nav_timeout, so a tool call cannot hang forever.
    ctx = RobotTools(
        node, query_client=node.query_client, nav_client=node.nav_client,
        tf_buffer=node.tf_buffer, goal_handle=None,
        map_frame=node.map_frame, robot_base_frame=node.robot_base_frame,
        query_timeout=node.query_timeout, nav_timeout=node.nav_timeout)

    server = Server("semantic-nav")

    @server.list_tools()
    async def list_tools():
        return [types.Tool(name=t["name"], description=t["description"],
                           inputSchema=t["inputSchema"])
                for t in registry.to_mcp_tools()]

    @server.call_tool()
    async def call_tool(name: str, arguments: dict):
        # dispatch() blocks on ROS futures -> run off the event loop so MCP stays responsive.
        loop = asyncio.get_running_loop()
        result = await loop.run_in_executor(None, registry.dispatch, ctx, name, arguments)
        return [types.TextContent(type="text", text=json.dumps(result))]

    node.get_logger().info("MCP server up (stdio); serving query/navigate/pose tools")
    try:
        async with mcp.server.stdio.stdio_server() as (read, write):
            await server.run(read, write, InitializationOptions(
                server_name="semantic-nav", server_version="0.1.0",
                capabilities=server.get_capabilities(
                    notification_options=NotificationOptions(),
                    experimental_capabilities={})))
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


def main(args=None):
    asyncio.run(amain())


if __name__ == "__main__":
    main()
