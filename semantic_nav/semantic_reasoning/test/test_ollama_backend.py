"""OllamaAgentBackend: internal<->ollama message translation + response parsing.

The ollama client is injected (a fake), so no server/model/network is needed at test time.
The two pure translation functions carry all the library-shape knowledge and are tested
directly — the format mismatches they bridge (tool_name vs name, no tool-call id, nested
function, dict args) are exactly the regressions most likely to bite against a live model.
"""
from types import SimpleNamespace

from semantic_reasoning.agent import AssistantTurn
from semantic_reasoning.ollama_backend import (
    OllamaAgentBackend,
    from_ollama_response,
    to_ollama_messages,
)


def test_to_ollama_messages_passes_system_and_user_through():
    msgs = [{"role": "system", "content": "sys"}, {"role": "user", "content": "go"}]
    assert to_ollama_messages(msgs) == msgs


def test_to_ollama_messages_nests_assistant_tool_calls_under_function():
    msgs = [{"role": "assistant", "content": "",
             "tool_calls": [{"id": "q1", "name": "query_semantic_map",
                             "arguments": {"text_query": "chair"}}]}]
    tc = to_ollama_messages(msgs)[0]["tool_calls"][0]
    assert tc == {"function": {"name": "query_semantic_map",
                               "arguments": {"text_query": "chair"}}}
    assert "id" not in tc  # ollama tool calls carry no id


def test_to_ollama_messages_maps_tool_result_to_tool_name_and_drops_id():
    msgs = [{"role": "tool", "name": "query_semantic_map",
             "tool_call_id": "q1", "content": '{"objects": []}'}]
    out = to_ollama_messages(msgs)[0]
    assert out["role"] == "tool"
    assert out["tool_name"] == "query_semantic_map"   # NOT "name"
    assert out["content"] == '{"objects": []}'
    assert "name" not in out and "tool_call_id" not in out


def test_from_ollama_response_parses_tool_calls_with_synthesized_unique_ids():
    msg = SimpleNamespace(content="", tool_calls=[
        SimpleNamespace(function=SimpleNamespace(
            name="navigate_to_pose", arguments={"x": 1.0, "y": 2.0})),
        SimpleNamespace(function=SimpleNamespace(
            name="get_robot_pose", arguments={})),
    ])
    turn = from_ollama_response(msg, step=3)
    assert isinstance(turn, AssistantTurn) and turn.content is None
    c0, c1 = turn.tool_calls
    assert c0.name == "navigate_to_pose"
    assert c0.arguments == {"x": 1.0, "y": 2.0} and isinstance(c0.arguments, dict)
    assert c0.id and c1.id and c0.id != c1.id


def test_from_ollama_response_content_only_turn():
    msg = SimpleNamespace(content="Arrived at the chair.", tool_calls=None)
    turn = from_ollama_response(msg, step=0)
    assert turn.tool_calls is None
    assert turn.content == "Arrived at the chair."


class _FakeClient:
    def __init__(self, message):
        self._message = message
        self.calls = []

    def chat(self, model, messages, tools):
        self.calls.append((model, messages, tools))
        return SimpleNamespace(message=self._message)


def test_step_round_trips_a_tool_call_and_forwards_tools_unchanged():
    msg = SimpleNamespace(content="", tool_calls=[
        SimpleNamespace(function=SimpleNamespace(
            name="query_semantic_map", arguments={"text_query": "mug"}))])
    fake = _FakeClient(msg)
    backend = OllamaAgentBackend(model="qwen2.5:7b", client=fake)
    tools = [{"type": "function", "function": {"name": "query_semantic_map"}}]
    turn = backend.step([{"role": "user", "content": "find the mug"}], tools)
    assert turn.tool_calls[0].name == "query_semantic_map" and turn.tool_calls[0].id
    model, _msgs, fwd_tools = fake.calls[0]
    assert model == "qwen2.5:7b" and fwd_tools == tools


def test_step_synthesized_ids_differ_across_calls():
    msg = SimpleNamespace(content="", tool_calls=[
        SimpleNamespace(function=SimpleNamespace(name="get_robot_pose", arguments={}))])
    backend = OllamaAgentBackend(client=_FakeClient(msg))
    t1 = backend.step([], [])
    t2 = backend.step([], [])
    assert t1.tool_calls[0].id != t2.tool_calls[0].id
