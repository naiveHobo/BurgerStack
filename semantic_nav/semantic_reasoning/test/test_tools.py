"""The shared tool layer: schema generation + dispatch. Pure (no ROS, no LLM)."""
from helpers import FakeToolContext

from semantic_reasoning.tools import TOOL_SPECS, ToolRegistry

TOOL_NAMES = {"query_semantic_map", "navigate_to_pose", "get_robot_pose"}


def test_tool_specs_cover_the_expected_tools():
    assert {s.name for s in TOOL_SPECS} == TOOL_NAMES


# --- schema generation ---------------------------------------------------

def test_to_ollama_tools_shape():
    tools = ToolRegistry().to_ollama_tools()
    assert {t["function"]["name"] for t in tools} == TOOL_NAMES
    for t in tools:
        assert t["type"] == "function"
        params = t["function"]["parameters"]
        assert params["type"] == "object"
        assert "properties" in params


def test_navigate_tool_requires_x_and_y():
    tools = {t["function"]["name"]: t for t in ToolRegistry().to_ollama_tools()}
    required = tools["navigate_to_pose"]["function"]["parameters"].get("required", [])
    assert "x" in required and "y" in required


def test_to_mcp_tools_uses_input_schema():
    tools = ToolRegistry().to_mcp_tools()
    assert {t["name"] for t in tools} == TOOL_NAMES
    for t in tools:
        assert "description" in t
        assert t["inputSchema"]["type"] == "object"


# --- dispatch ------------------------------------------------------------

def test_dispatch_unknown_tool_returns_error():
    out = ToolRegistry().dispatch(FakeToolContext(), "fly", {})
    assert "error" in out


def test_dispatch_query_calls_context_and_returns_objects():
    ctx = FakeToolContext(objects=[{"label": "chair", "position": {"x": 1.0, "y": 2.0, "z": 0.0}}])
    out = ToolRegistry().dispatch(ctx, "query_semantic_map", {"label": "chair"})
    assert ("query_semantic_map", {"text_query": "", "label": "chair", "region": "",
                                    "near": None, "radius": 0.0, "top_k": 0}) in ctx.calls
    assert out["objects"][0]["label"] == "chair"


def test_dispatch_navigate_passes_coordinates():
    ctx = FakeToolContext()
    out = ToolRegistry().dispatch(ctx, "navigate_to_pose", {"x": 1.5, "y": -0.5, "yaw": 1.0})
    name, kwargs = ctx.calls[-1]
    assert name == "navigate_to_pose"
    assert (kwargs["x"], kwargs["y"], kwargs["yaw"]) == (1.5, -0.5, 1.0)
    assert out["success"] is True


def test_dispatch_get_robot_pose():
    ctx = FakeToolContext(pose={"x": 0.0, "y": 0.0, "yaw": 0.0})
    out = ToolRegistry().dispatch(ctx, "get_robot_pose", {})
    assert ctx.calls[-1][0] == "get_robot_pose"
    assert out["pose"] == {"x": 0.0, "y": 0.0, "yaw": 0.0}


def test_dispatch_handler_exception_is_caught():
    ctx = FakeToolContext()
    # Missing required x/y -> handler raises -> dispatch should return an error dict.
    out = ToolRegistry().dispatch(ctx, "navigate_to_pose", {})
    assert "error" in out
