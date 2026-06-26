#ifndef RRT_CORE__RRT_HPP_
#define RRT_CORE__RRT_HPP_

#include "rrt_core/collision_checker.hpp"
#include "rrt_core/types.hpp"

#include <memory>
#include <vector>

namespace rrt_core
{

/// @brief Abstract base class for RRT / RRT* path planning
class RRT
{
public:
  /// @brief Construct an RRT planner with a given collision checker and parameters
  /// @param params The parameters for configuring the RRT / RRT* algorithm
  /// @param collision_checker A shared pointer to a CollisionChecker instance for obstacle checking
  RRT(const RRTParams & params, const std::shared_ptr<CollisionChecker> & collision_checker);

  /// @brief Plan a path from start to goal using the RRT / RRT* algorithm
  /// @param start The starting point of the path
  /// @param goal The goal point of the path
  /// @return An RRTResult containing the planning outcome, including success flag, path,
  ///         number of iterations, tree size, and visualization/debugging data
  RRTResult plan(const Point2D & start, const Point2D & goal) const;

private:
  /// @brief Smooth the given path using a greedy approach
  /// @param path The original path to be smoothed
  /// @return A new path that is a smoothed version of the original path
  std::vector<Point2D> smoothPath(const std::vector<Point2D> & path) const;

  RRTParams params_;
  std::shared_ptr<CollisionChecker> collision_checker_;
};

}  // namespace rrt_core

#endif  // RRT_CORE__RRT_HPP_
