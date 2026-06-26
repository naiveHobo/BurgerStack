#ifndef RRT_CORE__UTILS_HPP_
#define RRT_CORE__UTILS_HPP_

#include <cmath>

#include "rrt_core/types.hpp"

namespace rrt_core
{

/// @brief Compute the Euclidean distance between two 2D points
/// @param a The first point
/// @param b The second point
/// @return The Euclidean distance between points a and b
inline double distance(const Point2D & a, const Point2D & b)
{
  return std::hypot(b.x - a.x, b.y - a.y);
}

}

#endif  // RRT_CORE__UTILS_HPP_
