"""The shared reasoning loop for the ollama frontend.

A backend proposes the next turn (tool calls or a final answer); this loop executes
any tool calls through the ``ToolRegistry`` and feeds results back until the backend
finishes, the step budget runs out, or the task is cancelled. The loop is ROS-free
and backend-agnostic: ``MockAgentBackend`` is a deterministic, reactive stand-in (no
LLM); the deferred ``OllamaAgentBackend`` will plug in real tool-calling. Claude uses
the MCP server instead and supplies its own loop, so it does NOT use this module.
"""
from __future__ import annotations

import json
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Callable, List, Optional

from semantic_reasoning.tools import ToolContext, ToolRegistry

DEFAULT_SYSTEM_PROMPT = (
    "You are the navigation brain of a mobile robot. You have a spatial semantic map "
    "of the environment built during exploration. Use the provided tools to find "
    "objects in the map and drive the robot to them. Think step by step: query the "
    "map to locate the relevant object, then call navigate_to_object with that object's "
    "map-frame position to approach it — the robot stops a safe distance away and faces "
    "it, so never aim a goal at the object's exact cell. Use navigate_to_pose only for "
    "explicit free-space coordinates. When the task is complete, reply with a short "
    "natural-language summary of what you did."
)

# Tool names that move the robot toward a goal (for feedback labelling).
_NAV_TOOLS = ("navigate_to_pose", "navigate_to_object")

# A feedback sink: (status, reasoning_step, goal) where goal is (x, y, yaw) | None.
FeedbackFn = Callable[[str, str, Optional[tuple]], None]


@dataclass
class ToolCall:
    id: str
    name: str
    arguments: dict


@dataclass
class AssistantTurn:
    """One backend turn: either tool calls to execute, or a final ``content`` answer."""

    tool_calls: Optional[List[ToolCall]] = None
    content: Optional[str] = None


@dataclass
class AgentResult:
    success: bool          # True iff the agent finished normally (not cancelled / out of steps)
    summary: str
    steps: int = 0
    messages: List[dict] = field(default_factory=list)


class AgentBackend(ABC):
    """Proposes the next assistant turn given the conversation and the tool schemas."""

    @abstractmethod
    def step(self, messages: List[dict], tools: List[dict]) -> AssistantTurn:
        ...


def _goal_of(call: ToolCall) -> Optional[tuple]:
    if call.name not in _NAV_TOOLS:
        return None
    a = call.arguments
    return (float(a.get("x", 0.0)), float(a.get("y", 0.0)), float(a.get("yaw", 0.0)))


def run_agent(
    backend: AgentBackend,
    ctx: ToolContext,
    command: str,
    *,
    system_prompt: str = DEFAULT_SYSTEM_PROMPT,
    max_steps: int = 8,
    feedback: Optional[FeedbackFn] = None,
    registry: Optional[ToolRegistry] = None,
) -> AgentResult:
    registry = registry or ToolRegistry()
    tools = registry.to_ollama_tools()
    messages: List[dict] = [
        {"role": "system", "content": system_prompt},
        {"role": "user", "content": command},
    ]

    for step in range(max_steps):
        if ctx.is_cancelled():
            return AgentResult(False, "Task cancelled.", step, messages)

        turn = backend.step(messages, tools)

        if not turn.tool_calls:
            return AgentResult(True, turn.content or "", step + 1, messages)

        messages.append({
            "role": "assistant", "content": turn.content or "",
            "tool_calls": [{"id": c.id, "name": c.name, "arguments": c.arguments}
                           for c in turn.tool_calls],
        })
        for call in turn.tool_calls:
            status = "navigating" if call.name in _NAV_TOOLS else "reasoning"
            if feedback is not None:
                feedback(status, f"{call.name}({call.arguments})", _goal_of(call))
            result = registry.dispatch(ctx, call.name, call.arguments)
            messages.append({
                "role": "tool", "name": call.name, "tool_call_id": call.id,
                "content": json.dumps(result),
            })

    return AgentResult(False, f"Gave up after {max_steps} reasoning steps.", max_steps, messages)


class MockAgentBackend(AgentBackend):
    """Deterministic, reactive agent with no LLM: query the map, then drive to the
    first match. Doubles as the integration-demo agent (it really performs the task)."""

    def step(self, messages: List[dict], tools: List[dict]) -> AssistantTurn:
        results = {m["name"]: json.loads(m["content"])
                   for m in messages if m.get("role") == "tool"}

        if "query_semantic_map" not in results:
            command = next((m["content"] for m in messages if m.get("role") == "user"), "")
            return AssistantTurn(tool_calls=[ToolCall(
                "q1", "query_semantic_map", {"text_query": command})])

        objects = results["query_semantic_map"].get("objects", [])
        if not objects:
            return AssistantTurn(content="I could not find anything matching that in the semantic map.")

        target = objects[0]
        label = target.get("label", "target")
        if "navigate_to_object" not in results:
            pos = target.get("position", {})
            return AssistantTurn(tool_calls=[ToolCall(
                "n1", "navigate_to_object", {"x": pos.get("x", 0.0), "y": pos.get("y", 0.0)})])

        if results["navigate_to_object"].get("success"):
            return AssistantTurn(content=f"Arrived at the {label}.")
        return AssistantTurn(content=f"I found the {label} but could not reach it.")
