# rrt_core

Provides a standalone library implementing RRT and RRT* path planning in 2D space using an abstract collision checker to test whether points and line segments are collision-free.

---

## Algorithm

### Step 1: Sampling a new point
Sample a new random point within the world bounds and with a certain predefined probability, we instead choose the goal point as the new point.

### Step 2: Find nearest node in our tree
- Find the nearest node to the sampled point in our current tree.
- If the distance between the sampled point and the nearest node is within the chosen step size, the sampled point can be chosen as the new node to test. Otherwise, we create a new point step_size distance away from the nearest node towards the sampled point and choose this as the new node to test.

### Step 3: Check if new node is free
- Check if the new node's location is collision free and the straight line path between the nearest node and the new node is also collision free.
- Skip this new node if collisions are found, go back to step 1.

### Step 4: Choose new node's parent
- Vanilla RRT chooses the nearest node found in step 2 as the new node's parent. The new node's cost is the nearest node's cost plus the distance between the nearest node and the new node.
- RRT* does the following:
  - Start with the nearest node from step 2 as the baseline parent (with the cost described above).
  - Find neighboring nodes from our current tree within a pre-defined radius of the new node.
  - For each neighbor, compute the new node's total cost-to-come if that neighbor were its parent: the neighbor's own cost plus the distance between the neighbor and the new node.
  - Pick the neighbor that yields the lowest total cost, provided the straight line between that neighbor and the new node is collision free. This neighbor becomes the new node's parent, and its corresponding total cost becomes the new node's cost.

### Step 5: Insert new node to tree
- Vanilla RRT just inserts the new node to the tree with its parent and cost decided in step 4.
- RRT* does the following:
  - Find nearby neighboring nodes in our tree within a pre-defined radius to the newly inserted node
  - Check if there is a collision free line between new node and neighbor node
  - For each neighboring node, check if this node's cost would improve if the newly inserted node was its parent
  - Each neighboring node's potential cost is calculated as the cost of the new node + the distance between the new node and the neighbor. If this is less than the current cost of the neighbor node and there is a collision-free line between the two nodes, the neighbor is rewired to have the new node as its parent and its cost is updated accordingly

### Step 6: Check if goal was reached
- Check the newly inserted node to see if it falls within the chosen tolerance around the goal point *and* that the straight line between the new node and the goal is collision free.
- If both hold, the goal point itself is added to the tree as a node whose parent is the newly inserted node. RRT has found a path!
- Otherwise, go back to step 1.

### Step 7: Traverse the tree backwards from goal to find the path
- Start with the goal node added in step 6 and add it to the path.
- Move to its parent node and add it to the path, the parent node now becomes the current node.
- Keep repeating the process for each current node until we reach a node that doesn't have a parent, ie, the start point.
- Reverse the path to get a collision-free path from the start point to the goal.

---

## Improvements

### Greedy smoothing
Once a path has been found, greedy smoothing can be used to produce straighter smoother paths avoiding the inherent jittery paths produced by RRT.
- Starting from the first point in the path, check the farthest point in the path which has a collision-free straight line.
- If such a point is found, we can remove all points in between and continue the same process for this point.
- The result is a sparse path with longer straight line segments.

### Densification
Greedy smoothing leaves a sparse path of long segments, which is awkward for downstream consumers (controllers, trackers) that expect evenly spaced waypoints. After smoothing, the path is re-sampled at a fixed spacing so consecutive waypoints stay close together.
- The spacing is the smaller of the configured step size and the collision checker's grid resolution, so the path is never sampled coarser than the map it was planned on.
- Each segment is split into evenly spaced intermediate points, leaving the segment endpoints intact.
- The final returned path is therefore dense and evenly spaced, not sparse.

---

## Usage

The planner is decoupled from any particular map representation through the abstract `CollisionChecker` interface. To use the library, supply a collision checker, configure `RRTParams`, and call `plan()`:

```cpp
#include "rrt_core/rrt.hpp"
#include "rrt_core/grid_collision_checker.hpp"

using namespace rrt_core;

// Provide a collision checker. GridCollisionChecker is shipped with the
// package as a ready-made implementation backed by an occupancy grid.
auto collision_checker = std::make_shared<GridCollisionChecker>(
  grid, width, height, resolution, origin_x, origin_y);

RRTParams params;
params.use_rrt_star = true;   // RRT* instead of vanilla RRT
params.step_size = 0.3;
params.goal_tolerance = 0.1;

RRT planner(params, collision_checker);

const RRTResult result = planner.plan(Point2D{0.0, 0.0}, Point2D{5.0, 5.0});
if (result.success) {
  for (const Point2D & p : result.path) {
    // consume waypoint (p.x, p.y)
  }
}
```

To plan against a different map representation, subclass `CollisionChecker` and implement the world-bounds getters (`getMinX`/`getMaxX`/`getMinY`/`getMaxY`), `getResolution`, `isFree`, and `isLineFree`.

`RRTResult` also exposes the full search tree (`tree_nodes` and `tree_parents`) alongside `iterations` and `tree_size` for visualization and debugging.

---

## Parameters

Configured via `RRTParams`:

| Parameter | Default | Description |
| --- | --- | --- |
| `step_size` | `0.3` | Maximum extension length per iteration (meters). |
| `goal_bias` | `0.05` | Probability `[0, 1]` of sampling the goal instead of a random point. |
| `goal_tolerance` | `0.1` | Distance to the goal at which it is considered reached (meters). |
| `max_iterations` | `5000` | Maximum number of iterations before giving up. |
| `use_rrt_star` | `false` | If true, use RRT* (parent selection + rewiring); otherwise vanilla RRT. |
| `neighbor_radius` | `0.5` | Radius used to gather neighbors for RRT* parent selection and rewiring (meters). |
| `greedy_smoothing` | `true` | If true, run greedy smoothing on the final path before densification. |
| `seed` | `42` | Seed for the random number generator (reproducible runs). |
