"""The shared tool layer — the single source of truth for "what the robot can do".

Both reasoning frontends consume this one registry: the local ollama agent
(``agent.run_agent`` over ``to_ollama_tools()``) and the deferred MCP server
(over ``to_mcp_tools()``). Tools talk to the robot only through the abstract
``ToolContext``; the real implementation (``robot_tools.RobotTools``) wraps the
rclpy clients, while tests inject an in-memory fake. Tool results are plain
JSON-serializable dicts so they drop straight into LLM tool-results / MCP responses
— no ROS types leak past this boundary.
"""
from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Callable, List, Optional


class ToolContext(ABC):
    """The robot capabilities the tools are built on. Pure interface — no ROS here."""

    @abstractmethod
    def query_semantic_map(self, text_query: str = "", label: str = "", region: str = "",
                           near: Optional[dict] = None, radius: float = 0.0,
                           top_k: int = 0) -> List[dict]:
        ...

    @abstractmethod
    def navigate_to_pose(self, x: float, y: float, yaw: float = 0.0,
                         frame: str = "map") -> dict:
        ...

    @abstractmethod
    def get_robot_pose(self) -> Optional[dict]:
        ...

    @abstractmethod
    def is_cancelled(self) -> bool:
        ...


@dataclass
class ToolSpec:
    """One tool: its name, model-facing description, JSON-Schema params, and handler."""

    name: str
    description: str
    parameters: dict
    handler: Callable[[ToolContext, dict], dict]


# --- handlers (thin adapters from a JSON args dict to a ToolContext call) ----

def _h_query(ctx: ToolContext, args: dict) -> dict:
    objects = ctx.query_semantic_map(
        text_query=args.get("text_query", ""),
        label=args.get("label", ""),
        region=args.get("region", ""),
        near=args.get("near"),
        radius=float(args.get("radius", 0.0) or 0.0),
        top_k=int(args.get("top_k", 0) or 0),
    )
    return {"objects": objects, "count": len(objects)}


def _h_navigate(ctx: ToolContext, args: dict) -> dict:
    return ctx.navigate_to_pose(
        x=float(args["x"]), y=float(args["y"]),
        yaw=float(args.get("yaw", 0.0) or 0.0),
        frame=args.get("frame", "map"),
    )


def _h_get_pose(ctx: ToolContext, args: dict) -> dict:
    return {"pose": ctx.get_robot_pose()}


TOOL_SPECS: List[ToolSpec] = [
    ToolSpec(
        name="query_semantic_map",
        description=(
            "Search the spatial semantic map for objects. Use a free-text query for "
            "open-vocabulary search, or filter by label/region, or by proximity to a "
            "point. Returns objects with their map-frame position, label, description "
            "and region, ranked best-first."),
        parameters={
            "type": "object",
            "properties": {
                "text_query": {"type": "string", "description": "free-text / open-vocabulary query"},
                "label": {"type": "string", "description": "exact or fuzzy label filter"},
                "region": {"type": "string", "description": "restrict to a region / room"},
                "near": {
                    "type": "object",
                    "description": "proximity center in the map frame",
                    "properties": {"x": {"type": "number"}, "y": {"type": "number"},
                                   "z": {"type": "number"}},
                },
                "radius": {"type": "number", "description": "max distance from `near` (m); 0 = ignore"},
                "top_k": {"type": "integer", "description": "max results; 0 = server default"},
            },
            "required": [],
        },
        handler=_h_query,
    ),
    ToolSpec(
        name="navigate_to_pose",
        description=(
            "Drive the robot to a goal pose in the map frame using Nav2. Blocks until "
            "navigation finishes and reports whether the goal was reached."),
        parameters={
            "type": "object",
            "properties": {
                "x": {"type": "number", "description": "goal x in the map frame (m)"},
                "y": {"type": "number", "description": "goal y in the map frame (m)"},
                "yaw": {"type": "number", "description": "goal heading (rad); default 0"},
                "frame": {"type": "string", "description": "goal frame; default 'map'"},
            },
            "required": ["x", "y"],
        },
        handler=_h_navigate,
    ),
    ToolSpec(
        name="get_robot_pose",
        description="Get the robot's current pose (x, y, yaw) in the map frame.",
        parameters={"type": "object", "properties": {}, "required": []},
        handler=_h_get_pose,
    ),
]


class ToolRegistry:
    """Exposes the tool specs in each frontend's schema format and dispatches calls."""

    def __init__(self, specs: Optional[List[ToolSpec]] = None):
        self._specs = list(specs if specs is not None else TOOL_SPECS)
        self._by_name = {s.name: s for s in self._specs}

    @property
    def specs(self) -> List[ToolSpec]:
        return self._specs

    def to_ollama_tools(self) -> List[dict]:
        return [
            {"type": "function",
             "function": {"name": s.name, "description": s.description,
                          "parameters": s.parameters}}
            for s in self._specs
        ]

    def to_mcp_tools(self) -> List[dict]:
        return [
            {"name": s.name, "description": s.description, "inputSchema": s.parameters}
            for s in self._specs
        ]

    def dispatch(self, ctx: ToolContext, name: str, arguments: dict) -> dict:
        spec = self._by_name.get(name)
        if spec is None:
            return {"error": f"unknown tool '{name}'"}
        try:
            return spec.handler(ctx, arguments or {})
        except Exception as e:  # noqa: BLE001 — surface tool errors to the agent, don't crash
            return {"error": f"{type(e).__name__}: {e}"}
