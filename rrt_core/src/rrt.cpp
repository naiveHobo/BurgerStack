#include "rrt_core/rrt.hpp"

#include "rrt_core/spatial_hash.hpp"
#include "rrt_core/types.hpp"
#include "rrt_core/utils.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <random>
#include <stdexcept>
#include <vector>

namespace rrt_core
{

RRT::RRT(const RRTParams & params, const std::shared_ptr<CollisionChecker> & collision_checker)
: params_(params), collision_checker_(collision_checker)
{
  if (!collision_checker_) {
    throw std::invalid_argument("CollisionChecker pointer cannot be null");
  }
}

RRTResult RRT::plan(const Point2D & start, const Point2D & goal) const
{
  RRTResult result;

  if (!collision_checker_->isFree(start)) {
    std::cerr << "Start point is in collision" << std::endl;
    result.success = false;
    return result;
  }

  if (!collision_checker_->isFree(goal)) {
    std::cerr << "Goal point is in collision" << std::endl;
    result.success = false;
    return result;
  }

  std::mt19937 rng(params_.seed);
  std::uniform_real_distribution<double> sample_x(
    collision_checker_->getMinX(), collision_checker_->getMaxX());
  std::uniform_real_distribution<double> sample_y(
    collision_checker_->getMinY(), collision_checker_->getMaxY());
  std::uniform_real_distribution<double> bias(0.0, 1.0);

  std::vector<Point2D> nodes;
  std::vector<int> parents;
  std::vector<double> costs;

  nodes.push_back(start);
  parents.push_back(-1);
  costs.push_back(0.0);

  SpatialHash spatial_hash(params_.step_size);
  spatial_hash.insert(start, 0);

  int goal_index = -1;
  std::size_t iterations = 0;

  for (; iterations < params_.max_iterations; ++iterations) {
    // Sample a random point or the goal point based on goal bias
    const Point2D sample =
      (bias(rng) < params_.goal_bias) ? goal : Point2D{sample_x(rng), sample_y(rng)};

    // Find the nearest node in the tree to the sampled point
    int nearest_index = spatial_hash.nearest(sample, nodes);
    if (nearest_index == -1) {
      continue;  // No nodes in the tree yet
    }
    const Point2D & nearest_node = nodes[nearest_index];

    // Steer towards the sampled point by a maximum of step_size
    const double dist = distance(nearest_node, sample);
    const Point2D new_node =
      (dist <= params_.step_size)
        ? sample
        : Point2D{
            nearest_node.x + (params_.step_size * (sample.x - nearest_node.x) / dist),
            nearest_node.y + (params_.step_size * (sample.y - nearest_node.y) / dist)};

    // Check if the new node is free of obstacles
    if (
      !collision_checker_->isFree(new_node) ||
      !collision_checker_->isLineFree(nearest_node, new_node)) {
      continue;
    }

    // Choose the parent for the new node
    // RRT uses the nearest node as the parent
    // RRT* picks the collision-free node with the lowest cost within neighbor_radius
    int parent_index = nearest_index;
    double min_cost = costs[nearest_index] + distance(nearest_node, new_node);

    if (params_.use_rrt_star) {
      for (const int nidx : spatial_hash.withinRadius(new_node, params_.neighbor_radius, nodes)) {
        const double cost = costs[nidx] + distance(nodes[nidx], new_node);
        if (cost < min_cost && collision_checker_->isLineFree(nodes[nidx], new_node)) {
          min_cost = cost;
          parent_index = nidx;
        }
      }
    }

    // Add the new node to the tree
    const int new_index = static_cast<int>(nodes.size());
    nodes.push_back(new_node);
    parents.push_back(parent_index);
    costs.push_back(min_cost);
    spatial_hash.insert(new_node, new_index);

    // RRT* rewiring: check if the new node can be a better parent for its neighbors
    if (params_.use_rrt_star) {
      for (const int nidx : spatial_hash.withinRadius(new_node, params_.neighbor_radius, nodes)) {
        // Skip the new node and its parent to avoid self-loop
        if (nidx == parent_index || nidx == new_index) {
          continue;
        }
        const double new_cost = min_cost + distance(new_node, nodes[nidx]);
        if (new_cost < costs[nidx] && collision_checker_->isLineFree(new_node, nodes[nidx])) {
          parents[nidx] = new_index;
          costs[nidx] = new_cost;
        }
      }
    }

    // Check if the goal was reached
    if (
      distance(new_node, goal) <= params_.goal_tolerance &&
      collision_checker_->isLineFree(new_node, goal)) {
      goal_index = static_cast<int>(nodes.size());
      nodes.push_back(goal);
      parents.push_back(new_index);
      costs.push_back(min_cost + distance(new_node, goal));
      break;
    }
  }

  result.iterations = iterations;
  result.tree_size = nodes.size();
  result.tree_nodes = std::move(nodes);
  result.tree_parents = std::move(parents);

  if (goal_index != -1) {
    std::vector<Point2D> path;
    for (int idx = goal_index; idx != -1; idx = result.tree_parents[idx]) {
      path.push_back(result.tree_nodes[idx]);
    }

    std::reverse(path.begin(), path.end());

    if (params_.greedy_smoothing) {
      path = smoothPath(path);
    }

    const double densify_spacing = std::min(params_.step_size, collision_checker_->getResolution());
    path = densifyPath(path, densify_spacing);

    result.path = std::move(path);
    result.success = true;
  }

  return result;
}

std::vector<Point2D> RRT::smoothPath(const std::vector<Point2D> & path) const
{
  // No smoothing needed for paths with less than 2 points
  if (path.size() < 2) {
    return path;
  }

  std::vector<Point2D> smoothed_path;
  smoothed_path.push_back(path.front());

  std::size_t i = 0;
  while (i + 1 < path.size()) {
    // Find the farthest point that can be connected to the current point without collision
    std::size_t j = path.size() - 1;
    for (; j > i + 1; --j) {
      if (collision_checker_->isLineFree(path[i], path[j])) {
        break;
      }
    }
    smoothed_path.push_back(path[j]);
    i = j;
  }

  return smoothed_path;
}

std::vector<Point2D> RRT::densifyPath(const std::vector<Point2D> & path, double step_size) const
{
  std::vector<Point2D> densified_path;

  if (path.empty()) {
    return densified_path;
  }

  densified_path.push_back(path.front());

  for (std::size_t i = 0; i < path.size() - 1; ++i) {
    const Point2D & start = path[i];
    const Point2D & end = path[i + 1];
    const double segment_length = distance(start, end);
    const int num_steps = static_cast<int>(std::ceil(segment_length / step_size));

    for (int step = 1; step < num_steps; ++step) {
      const double t = static_cast<double>(step) / num_steps;
      Point2D intermediate_point{start.x + t * (end.x - start.x), start.y + t * (end.y - start.y)};
      densified_path.push_back(intermediate_point);
    }
    densified_path.push_back(end);
  }

  return densified_path;
}

}  // namespace rrt_core
