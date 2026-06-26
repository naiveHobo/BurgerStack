#ifndef RRT_CORE__COLLISION_CHECKER_HPP_
#define RRT_CORE__COLLISION_CHECKER_HPP_

#include "rrt_core/types.hpp"

namespace rrt_core
{

/// @brief Abstract base class for collision checking in a 2D space
class CollisionChecker
{
public:
  virtual ~CollisionChecker() = default;

  /// World-frame sampling bounds
  virtual double getMinX() const = 0;
  virtual double getMaxX() const = 0;
  virtual double getMinY() const = 0;
  virtual double getMaxY() const = 0;
  virtual double getResolution() const = 0;

  /// @brief Check if a given point is free of obstacles
  /// @param p The 2D point to check
  /// @return True if the point is free, false if it is in collision
  virtual bool isFree(const Point2D & p) const = 0;

  /// @brief Check if a line segment between two points is free of obstacles
  /// @param a The starting point of the line segment
  /// @param b The ending point of the line segment
  /// @return True if the line segment is free, false if it intersects any obstacles
  virtual bool isLineFree(const Point2D & a, const Point2D & b) const = 0;
};

}  // namespace rrt_core

#endif  // RRT_CORE__COLLISION_CHECKER_HPP_
