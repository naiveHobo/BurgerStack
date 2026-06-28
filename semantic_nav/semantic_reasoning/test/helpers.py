"""Shared test doubles for semantic_reasoning unit tests (ROS-free)."""
from __future__ import annotations

from semantic_reasoning.agent import AgentBackend
from semantic_reasoning.tools import ToolContext


class FakeToolContext(ToolContext):
    """In-memory ToolContext that records calls and returns canned data."""

    def __init__(self, objects=None, pose=None):
        self.objects = objects or []
        self.pose = pose
        self.cancelled = False
        self.fail_navigation = False
        self.calls = []  # list of (tool_name, kwargs)

    def query_semantic_map(self, text_query="", label="", region="",
                           near=None, radius=0.0, top_k=0):
        self.calls.append(("query_semantic_map", dict(
            text_query=text_query, label=label, region=region,
            near=near, radius=radius, top_k=top_k)))
        return list(self.objects)

    def navigate_to_pose(self, x, y, yaw=0.0, frame="map"):
        self.calls.append(("navigate_to_pose", dict(x=x, y=y, yaw=yaw, frame=frame)))
        if self.fail_navigation:
            return {"success": False, "status": "aborted"}
        return {"success": True, "status": "succeeded"}

    def get_robot_pose(self):
        self.calls.append(("get_robot_pose", {}))
        return self.pose

    def is_cancelled(self):
        return self.cancelled


class ScriptedBackend(AgentBackend):
    """Returns a predetermined sequence of AssistantTurns (for loop mechanics tests).

    When ``loop`` is set and the script is exhausted, the last turn repeats — handy
    for driving the max_steps path.
    """

    def __init__(self, turns, *, loop=False):
        self.turns = list(turns)
        self.loop = loop
        self.steps = 0

    def step(self, messages, tools):
        self.steps += 1
        if self.turns:
            turn = self.turns.pop(0)
            self._last = turn
            return turn
        if self.loop:
            return self._last
        raise AssertionError("ScriptedBackend ran out of turns")
