# rrt_planner

ROS 2 integration for the [rrt_core](../rrt_core/README.md) planning library. It provides two ways to run RRT / RRT* planning on a robot:

- A **Nav2 global planner plugin** (`rrt_planner/RRTGlobalPlanner`) that plans over the Nav2 global costmap and returns a `nav_msgs/Path`.
- A **standalone planner node** (`rrt_planner_node`) that plans over a `nav_msgs/OccupancyGrid` map topic, intended for development and debugging outside the full Nav2 stack.

Both share the same algorithm core; they differ only in where the map comes from and how planning is triggered.

---

## How it works

Both entry points wrap `rrt_core::RRT` and feed it a `rrt_core::CollisionChecker` implementation. The only real difference between them is the collision checker:

- The Nav2 plugin uses a **`CostmapCollisionChecker`**, which queries the live `nav2_costmap_2d::Costmap2D`. A cell is free when its cost is below the configured `lethal_cost`; cells with no information are treated as free only when `allow_unknown` is set. Line checks are done by sampling along the segment at half the costmap resolution.
- The standalone node uses `rrt_core::GridCollisionChecker` directly on the incoming occupancy grid, with its own occupancy threshold, unknown-cell handling, and obstacle inflation.

Both also publish a `visualization_msgs/MarkerArray` on `rrt_tree` for RViz: the search tree as blue edges and the final path as a green line strip.

---

## Nav2 global planner plugin

The plugin implements `nav2_core::GlobalPlanner` and is exported through `plugins.xml`, so Nav2 can load it like any other global planner. On every `createPlan` call it builds a `CostmapCollisionChecker` over the current global costmap (holding the costmap lock), runs the planner from the start pose to the goal pose, and converts the resulting waypoints into a `nav_msgs/Path`. Each pose's orientation is set to face the next waypoint, and the final pose inherits the goal's orientation. If no path is found, an empty path is returned and a warning is logged.

### Usage

Reference the plugin from your Nav2 planner server configuration:

```yaml
planner_server:
  ros__parameters:
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "rrt_planner/RRTGlobalPlanner"
      step_size: 0.3
      goal_bias: 0.1
      goal_tolerance: 0.25
      max_iterations: 50000
      use_rrt_star: true
      rewire_radius: 0.75
      smooth: true
      seed: 42
      lethal_cost: 253
      allow_unknown: false
```

### Parameters

All parameters are namespaced under the planner's name (e.g. `GridBased.step_size`).

| Parameter | Default | Description |
| --- | --- | --- |
| `step_size` | `0.3` | Maximum tree extension length per iteration (meters). |
| `goal_bias` | `0.1` | Probability of sampling the goal instead of a random point. |
| `goal_tolerance` | `0.25` | Distance to the goal at which it is considered reached (meters). |
| `max_iterations` | `50000` | Maximum iterations before giving up. |
| `use_rrt_star` | `true` | Use RRT* (parent selection + rewiring) instead of vanilla RRT. |
| `rewire_radius` | `0.75` | Neighbor radius used for RRT* parent selection and rewiring (meters). |
| `smooth` | `true` | Run greedy smoothing on the path before densification. |
| `seed` | `42` | Seed for the random number generator (reproducible runs). |
| `lethal_cost` | `253` | Costmap cost at or above which a cell is treated as an obstacle. |
| `allow_unknown` | `false` | Treat unknown (no-information) costmap cells as free. |

---

## Standalone planner node

`rrt_planner_node` runs RRT / RRT* against a map topic without the rest of Nav2. Set a start with `initialpose`, send a goal on `goal_pose`, and the node plans and publishes the path plus a tree visualization. If no start has been received yet, it defaults to `(0, 0)`; if no map has arrived, the goal is ignored with a warning.

### Usage

Run it bare against whatever is publishing `/map`:

```bash
ros2 run rrt_planner rrt_planner_node
```

Or use the one-command demo that serves the saved occupancy map and opens RViz with the right config (the Section-3 "grid + start + goal → path" demo, no Gazebo):

```bash
pixi run plan-demo        # = ros2 launch burger_bringup plan_demo.launch.py
```

Then send a goal with RViz's *2D Goal Pose* tool. Set the start by publishing a `PoseStamped` on `/initialpose` (RViz's *2D Pose Estimate* publishes a `PoseWithCovarianceStamped`, a different type, so it won't set this node's start):

```bash
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseStamped \
    '{header: {frame_id: map}, pose: {position: {x: 0.0, y: 10.0}}}'
```

### Topics

| Topic | Type | Direction | Description |
| --- | --- | --- | --- |
| `map` | `nav_msgs/OccupancyGrid` | subscribe | Occupancy grid used for collision checking (transient-local, latched). |
| `initialpose` | `geometry_msgs/PoseStamped` | subscribe | Sets the planning start point. |
| `goal_pose` | `geometry_msgs/PoseStamped` | subscribe | Goal pose; receiving one triggers a plan. |
| `rrt_path` | `nav_msgs/Path` | publish | The planned path. |
| `rrt_tree` | `visualization_msgs/MarkerArray` | publish | Search tree (blue) and final path (green) for RViz. |

### Parameters

| Parameter | Default | Description |
| --- | --- | --- |
| `global_frame` | `map` | Frame ID stamped on the published path. |
| `occupied_threshold` | `50` | Occupancy value at or above which a cell is an obstacle. |
| `unknown_is_obstacle` | `true` | Treat unknown (`-1`) cells as obstacles. |
| `inflation_radius` | `0.2` | Radius (meters) by which obstacles are inflated before planning. |
| `step_size` | `0.3` | Maximum tree extension length per iteration (meters). |
| `goal_bias` | `0.1` | Probability of sampling the goal instead of a random point. |
| `goal_tolerance` | `0.25` | Distance to the goal at which it is considered reached (meters). |
| `max_iterations` | `50000` | Maximum iterations before giving up. |
| `use_rrt_star` | `true` | Use RRT* instead of vanilla RRT. |
| `neighbor_radius` | `0.75` | Neighbor radius used for RRT* parent selection and rewiring (meters). |
| `smooth` | `true` | Run greedy smoothing on the path before densification. |
| `seed` | `42` | Seed for the random number generator. |
