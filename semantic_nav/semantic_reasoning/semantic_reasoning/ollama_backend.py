"""Tool-calling agent backend over a local ollama server (the real ``backend: ollama``).

Plugs a real LLM into the shared ``run_agent`` loop: it proposes the next ``AssistantTurn``
(tool calls or a final answer) from the conversation so far, using ollama's native tool-calling.
Claude does NOT use this — it drives the same tools through the MCP server (see ``mcp_server``).

The ``ollama`` client imports lazily and is injectable, so the package imports and the unit
tests run with no server/model. All library-shape knowledge lives in two pure functions:

``to_ollama_messages`` / ``from_ollama_response`` bridge the gaps between the loop's internal
message format and ollama's, which differ in three ways that matter:
  * assistant tool calls nest under ``{"function": {...}}`` (not a flat ``{id,name,arguments}``);
  * tool results use ``tool_name`` (not ``name``) and carry no ``tool_call_id``;
  * response tool calls carry no id (we synthesize one) and ``arguments`` is already a dict.
"""
from __future__ import annotations

from typing import List, Optional

from semantic_reasoning.agent import AgentBackend, AssistantTurn, ToolCall


def to_ollama_messages(messages: List[dict]) -> List[dict]:
    """Internal loop message format -> ollama chat message format."""
    out: List[dict] = []
    for m in messages:
        role = m.get("role")
        if role == "assistant" and m.get("tool_calls"):
            out.append({
                "role": "assistant",
                "content": m.get("content", "") or "",
                "tool_calls": [
                    {"function": {"name": c["name"], "arguments": c["arguments"]}}
                    for c in m["tool_calls"]
                ],
            })
        elif role == "tool":
            # internal {name, tool_call_id, content} -> ollama {tool_name, content}
            out.append({
                "role": "tool",
                "tool_name": m.get("name", ""),
                "content": m.get("content", ""),
            })
        else:  # system / user / plain assistant
            out.append({"role": role, "content": m.get("content", "") or ""})
    return out


def from_ollama_response(message, *, step: int = 0) -> AssistantTurn:
    """ollama ``response.message`` -> ``AssistantTurn`` (synthesizing tool-call ids)."""
    raw = getattr(message, "tool_calls", None) or []
    calls = [
        ToolCall(id=f"call_{step}_{i}",
                 name=tc.function.name,
                 arguments=dict(tc.function.arguments or {}))  # Mapping -> plain dict
        for i, tc in enumerate(raw)
    ]
    content = getattr(message, "content", None)
    return AssistantTurn(tool_calls=calls or None, content=content or None)


class OllamaAgentBackend(AgentBackend):
    """Proposes turns via ollama tool-calling chat. Injectable client for testing."""

    def __init__(self, model: str = "qwen2.5:7b", host: str = "http://localhost:11434",
                 client=None):
        self.model = model
        self.host = host
        self._client = client
        self._step = 0

    @property
    def client(self):
        if self._client is None:
            import ollama  # lazy: only when a real backend is selected
            self._client = ollama.Client(host=self.host)
        return self._client

    def step(self, messages: List[dict], tools: List[dict]) -> AssistantTurn:
        resp = self.client.chat(
            model=self.model,
            messages=to_ollama_messages(messages),
            tools=tools,                     # to_ollama_tools() shape passes through unchanged
        )
        turn = from_ollama_response(resp.message, step=self._step)
        self._step += 1
        return turn
