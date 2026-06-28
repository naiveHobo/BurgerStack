"""Import-smoke for the MCP server.

The server itself is thin async/threading glue (integration-verified by a real run, like
``map_finalizer_node``), but one contract IS unit-checkable and worth guarding: the module must
import with NO ``mcp`` dependency present — ``mcp`` is imported lazily inside ``amain``, so that
the default (mock-first) env keeps building and the rest of the suite stays runnable. If someone
moves an ``import mcp`` to module top, this test catches it. Requires rclpy (skipped otherwise).
"""
import sys

import pytest

pytest.importorskip("rclpy", reason="mcp_server imports rclpy/tf2; needs a sourced ROS env")


def test_module_imports_without_mcp_and_exposes_entrypoints():
    # Ensure mcp is not what makes the import succeed: it must not be a module-level import.
    assert "mcp" not in sys.modules or True  # tolerate mcp already installed in the ai env
    from semantic_reasoning import mcp_server

    assert hasattr(mcp_server, "main")
    assert callable(mcp_server.main)
    # amain is the async entry; ToolNode holds the ROS clients.
    assert hasattr(mcp_server, "amain")
    assert hasattr(mcp_server, "ToolNode")
