# semantic_nav_bringup

Integration/bringup for the **`semantic_nav`** stack. It composes the existing
BurgerStack launch system (`burger_bringup`) with the `semantic_nav` nodes into a
two-phase demo. It contains **no nodes** — only launch files, a parameter file, and
an RViz config. Everything runs **mock-first** (mock detector / describer / embedder /
reasoning backend), so the whole demo needs no GPU, LLM, or network.

## Phase 1 — map + explore (build the semantic map)

```bash
ros2 launch semantic_nav_bringup semantic_mapping.launch.py world:=office explore:=true
```

Brings up the office world + RGB-D burger + SLAM + Nav2 + frontier explorer (all from
`burger_bringup`), and adds:
- `perception_node` — RGB-D → `map`-frame 3D detections (`/semantic/detections`).
- `mapping_node` — fuses detections into the live semantic map (`/semantic/map`).
- `map_finalizer` + a managed `map_saver_server` — the handoff (below).

Watch the map build: add `rviz:=true` to show the semantic RViz view (adds
`/semantic/map_markers` + `/semantic/detection_markers` + the camera on top of the planner
displays) **as a single window** — it forwards `use_rviz:=false` to `burger_bringup` so the
bundled `planner.rviz` is suppressed (the semantic config is a superset of it). The default
(`rviz:=false`) keeps the plain `planner.rviz` view, unchanged.

## Handoff — save both maps

When exploration is far enough along, persist **both** the Nav2 occupancy map and the
enriched semantic map. Two equivalent ways:

- **Automatic:** launch Phase 1 with `auto_finalize:=true` — the `map_finalizer` fires
  when the explorer publishes `/exploration_complete`.
- **Manual:** with the Phase-1 stack running,
  ```bash
  ros2 launch semantic_nav_bringup save_maps.launch.py
  ```

Both write `~/.semantic_nav/occupancy_map.{yaml,pgm}` and
`~/.semantic_nav/semantic_map.json`.

## Phase 2 — localize + reason (drive by natural language)

```bash
ros2 launch semantic_nav_bringup semantic_navigation.launch.py world:=office
```

Brings up the office world + AMCL (against the saved occupancy map) + Nav2, and adds:
- `map_server_node` — loads the saved semantic map, serves
  `/map_server_node/query_semantic_map`.
- `execute_task_node` — the `ExecuteTask` action; turns commands into Nav2 goals.

Send a command:
```bash
ros2 action send_goal /execute_task_node/execute_task \
    semantic_nav_msgs/action/ExecuteTask "{command: 'go to the chair'}" --feedback
```

## Important: map-frame consistency

The semantic map stores object positions in the **`map` frame of the Phase-1 SLAM
session**. Phase 2 must localize against the **occupancy map saved from that same
session** (`~/.semantic_nav/occupancy_map.yaml`, the default) so the two share an
origin and goals land correctly. Using a *different* occupancy map (e.g. the
ground-truth `map_gt.yaml`) shifts the origin and goals will be misplaced.

## Going from mock to real backends

The demo is mock-first. The real AI backends — YOLO-World detection, an ollama VLM
describer, CLIP embeddings, and an ollama tool-calling agent — slot in behind the same
interfaces with **no launch changes**: just pass the ready-made real-params overlay,
`params/semantic_nav_real.yaml` (it flips `detector: yolo_world`,
`describer: ollama` / `embedder: clip`, `map_server_node.embedder: clip`, and
`execute_task_node.backend: ollama`).

**1. Install the optional `ai` pixi environment** (heavy: CUDA torch + ultralytics +
open_clip + ollama client + mcp; the default mock env is untouched). It builds into its own
`install_ai/` tree (separate from the default env's `install/`) so the two never collide:

```bash
pixi install -e ai          # one-time solve + download (GB-scale)
pixi run   -e ai ai-build    # build the workspace into install_ai/ (ai-env shebangs)
```

**2. Install the ollama server and pull the models.** The ollama *server* is separate from
the python client (only the client is in the `ai` env). Use the **official installer** — its
bundled CUDA runners use the GPU independently of conda, so the `ai` env's pytorch stays put:

```bash
curl -fsSL https://ollama.com/install.sh | sh   # installs + starts the server on :11434
ollama pull qwen2.5:7b      # Phase-2 reasoning agent
ollama pull moondream       # Phase-1 VLM crop describer
# YOLO-World + CLIP weights auto-download on first use.
```

> Avoid `conda-forge::ollama`'s GPU build here: it requires `cuda-version >=13`, and because
> conda's cuda-version is environment-wide it would force the whole `ai` env (incl. pytorch)
> onto CUDA 13 — a multi-GB torch re-download for no benefit. The official installer sidesteps
> that. (The installer usually starts a systemd service already; otherwise run `ollama serve &`.)

**3. Run either phase in the `ai` env with the overlay** (convenience tasks wrap these):

```bash
pixi run -e ai ai-mapping      # Phase 1 with real perception + enrichment
pixi run -e ai ai-navigation   # Phase 2 with CLIP queries + the ollama agent
# equivalently, any launch with: params_file:=<.../params/semantic_nav_real.yaml>
```

> **CLIP dim note:** with `embedder: clip` the `embedding_dim` param is ignored — the CLIP
> model fixes the dimension (ViT-B-32 → 512). Keep the same `clip_model` on `mapping_node`
> and `map_server_node` so image- and text-embeddings share one space.

### Claude / MCP frontend

The Claude side of the shared tool layer is the MCP server `semantic_reasoning mcp_server`
(stdio), exposing the same `query_semantic_map` / `navigate_to_pose` / `get_robot_pose`
tools over the Model Context Protocol. Run it against a live Phase-2 stack:

```bash
pixi run -e ai mcp-server      # or: ros2 run semantic_reasoning mcp_server
```

then register it as an MCP server in Claude Desktop/Code. It reuses the exact same
`ToolRegistry` and `RobotTools` as the ollama `execute_task_node` — two frontends, one tool
implementation. (8 GB VRAM is fine: the GPU-heavy phases never run at the same time.)
