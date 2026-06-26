#ifndef RRT_CORE__TYPES_HPP_
#define RRT_CORE__TYPES_HPP_

#include <cstddef>
#include <vector>
#include <utility>
#include <cstdint>

namespace rrt_core
{

typedef std::pair<int, int> GridCell;

/// @brief A simple 2D point structure to represent coordinates in a 2D space (meters).
struct Point2D
{
  double x{0.0};
  double y{0.0};
};

/// @brief A hash function for GridCell to be used in unordered containers
struct CellHash
{
  std::size_t operator()(const GridCell & cell) const
  {
    // Pack two 32-bit ints into a single 64-bit int,
    const int64_t key = (static_cast<int64_t>(cell.first) <<
      32) ^ static_cast<uint32_t>(cell.second);
    return std::hash<int64_t>()(key);
  }
};

/// @brief Parameters for configuring the RRT / RRT* algorithm
struct RRTParams
{
  /// Maximum extension length per iteration (meters)
  double step_size{0.3};
  /// Probability [0, 1] of sampling the goal point over a random point
  double goal_bias{0.05};
  /// Distance to goal to consider it reached (meters)
  double goal_tolerance{0.1};
  /// Maximum number of iterations before giving up
  std::size_t max_iterations{5000};
  /// If true, use RRT*; otherwise, use RRT
  bool use_rrt_star{false};
  /// Radius to consider for rewiring in RRT* (meters)
  double neighbor_radius{0.5};
  /// If true, perform greedy smoothing on the final path
  bool greedy_smoothing{true};
  /// Seed for random number generation
  unsigned int seed{42};
};

/// @brief Result of an RRT / RRT* planning attempt
struct RRTResult
{
  /// Flag indicating whether a valid path was found
  bool success{false};
  /// The path from start to goal as a sequence of 2D points, empty on failure
  std::vector<Point2D> path;
  /// Number of iterations taken to find the path
  std::size_t iterations{0};
  /// Size of the tree (number of nodes) explored during planning
  std::size_t tree_size{0};
  /// Visualization/debugging data
  /// A list of all nodes in the RRT tree
  std::vector<Point2D> tree_nodes;
  /// Index of parent node for each node in tree_nodes
  /// tree_parents[i] is the index of node i's parent, or -1 for the root
  std::vector<int> tree_parents;
};

}  // namespace rrt_core

#endif  // RRT_CORE__TYPES_HPP_
