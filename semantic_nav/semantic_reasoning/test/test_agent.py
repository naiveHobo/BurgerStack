"""The reasoning loop: tool dispatch, termination, cancellation, feedback. Pure."""
from helpers import FakeToolContext, ScriptedBackend

from semantic_reasoning.agent import (
    AssistantTurn,
    MockAgentBackend,
    ToolCall,
    run_agent,
)

CHAIR = {"id": 0, "label": "chair", "description": "a wooden chair", "region": "region_0",
         "position": {"x": 1.0, "y": 2.0, "z": 0.0}, "confidence": 0.9}


def _query_turn(**args):
    return AssistantTurn(tool_calls=[ToolCall(id="c1", name="query_semantic_map", arguments=args)])


def _nav_turn(x, y):
    return AssistantTurn(tool_calls=[ToolCall(id="c2", name="navigate_to_pose",
                                              arguments={"x": x, "y": y})])


# --- loop mechanics (ScriptedBackend) ------------------------------------

def test_run_agent_executes_tool_then_finishes():
    ctx = FakeToolContext(objects=[CHAIR])
    backend = ScriptedBackend([_query_turn(label="chair"),
                               AssistantTurn(content="done")])
    result = run_agent(backend, ctx, "find the chair", max_steps=5)
    assert result.success is True
    assert result.summary == "done"
    assert ctx.calls[0][0] == "query_semantic_map"


def test_run_agent_threads_multiple_tool_calls():
    ctx = FakeToolContext(objects=[CHAIR])
    backend = ScriptedBackend([_query_turn(label="chair"), _nav_turn(1.0, 2.0),
                               AssistantTurn(content="arrived")])
    result = run_agent(backend, ctx, "go to the chair", max_steps=5)
    assert [c[0] for c in ctx.calls] == ["query_semantic_map", "navigate_to_pose"]
    assert ctx.calls[1][1]["x"] == 1.0 and ctx.calls[1][1]["y"] == 2.0
    assert result.success is True and result.summary == "arrived"


def test_run_agent_respects_max_steps():
    ctx = FakeToolContext(objects=[CHAIR])
    backend = ScriptedBackend([_query_turn(label="chair")], loop=True)  # never finishes
    result = run_agent(backend, ctx, "loop forever", max_steps=3)
    assert result.success is False
    assert backend.steps == 3


def test_run_agent_cancelled_returns_early():
    ctx = FakeToolContext(objects=[CHAIR])
    ctx.cancelled = True
    backend = ScriptedBackend([_query_turn(label="chair"), AssistantTurn(content="done")])
    result = run_agent(backend, ctx, "go to the chair")
    assert result.success is False
    assert "cancel" in result.summary.lower()
    assert ctx.calls == []  # nothing dispatched


def test_run_agent_emits_feedback():
    ctx = FakeToolContext(objects=[CHAIR])
    backend = ScriptedBackend([_nav_turn(1.0, 2.0), AssistantTurn(content="arrived")])
    events = []
    run_agent(backend, ctx, "go", feedback=lambda status, step, goal: events.append((status, step, goal)))
    assert any("navigate_to_pose" in step for _, step, _ in events)
    # the navigate step should surface the goal coordinates for current_goal feedback
    assert any(goal == (1.0, 2.0, 0.0) for _, _, goal in events)


# --- MockAgentBackend (reactive, no LLM) ---------------------------------

def test_mock_backend_queries_then_navigates_to_first_result():
    ctx = FakeToolContext(objects=[CHAIR])
    result = run_agent(MockAgentBackend(), ctx, "go to the chair")
    names = [c[0] for c in ctx.calls]
    assert names == ["query_semantic_map", "navigate_to_pose"]
    nav_kwargs = ctx.calls[1][1]
    assert (nav_kwargs["x"], nav_kwargs["y"]) == (1.0, 2.0)
    assert result.success is True
    assert "chair" in result.summary.lower()


def test_mock_backend_handles_no_results():
    ctx = FakeToolContext(objects=[])
    result = run_agent(MockAgentBackend(), ctx, "go to the unicorn")
    assert [c[0] for c in ctx.calls] == ["query_semantic_map"]  # no navigation attempted
    assert result.success is True  # agent finished gracefully
    assert "could not" in result.summary.lower() or "couldn't" in result.summary.lower()


def test_mock_backend_reports_navigation_failure():
    ctx = FakeToolContext(objects=[CHAIR])
    ctx.fail_navigation = True
    result = run_agent(MockAgentBackend(), ctx, "go to the chair")
    assert [c[0] for c in ctx.calls] == ["query_semantic_map", "navigate_to_pose"]
    assert "could not reach" in result.summary.lower() or "not reach" in result.summary.lower()
