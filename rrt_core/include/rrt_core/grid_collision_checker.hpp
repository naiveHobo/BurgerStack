#ifndef RRT_CORE__GRID_COLLISION_CHECKER_HPP_
#define RRT_CORE__GRID_COLLISION_CHECKER_HPP_

#include "rrt_core/collision_checker.hpp"
#include "rrt_core/types.hpp"

#include <cstdint>
#include <vector>

namespace rrt_core
{

/// @brief A collision checker that uses a 2D grid to determine
/// whether points and line segments are free of obstacles.
class GridCollisionChecker : public CollisionChecker
{
public:
  /// @brief Construct a new GridCollisionChecker object
  /// @param grid The occupancy grid to use for collision checking
  /// @param width The width of the grid (number of cells in the x-direction)
  /// @param height The height of the grid (number of cells in the y-direction)
  /// @param resolution The resolution of the grid (meters per cell)
  /// @param origin_x The x-coordinate of the grid's origin in world coordinates
  /// @param origin_y The y-coordinate of the grid's origin in world coordinates
  /// @param occupied_threshold The threshold value above which a cell is considered occupied
  /// (default: 50)
  /// @param unknown_is_free Whether to treat unknown cells as free (default: false)
  /// @param inflation_radius The radius around occupied cells to consider as inflated (default:
  /// 0.0)
  GridCollisionChecker(
    const std::vector<int8_t> & grid, uint32_t width, uint32_t height, double resolution,
    double origin_x, double origin_y, int occupied_threshold = 50, bool unknown_is_free = false,
    double inflation_radius = 0.0);

  double getMinX() const override { return origin_x_; }

  double getMaxX() const override { return origin_x_ + width_ * resolution_; }

  double getMinY() const override { return origin_y_; }

  double getMaxY() const override { return origin_y_ + height_ * resolution_; }

  double getResolution() const override { return resolution_; }

  bool isFree(const Point2D & p) const override;
  bool isLineFree(const Point2D & a, const Point2D & b) const override;

  /// @brief Convert a point in world coordinates to grid coordinates
  /// @param world_point The point in world coordinates
  /// @param grid_x The x-coordinate in grid coordinates (output)
  /// @param grid_y The y-coordinate in grid coordinates (output)
  /// @return True if the conversion was successful (the point is within the grid bounds), false
  /// otherwise
  bool worldToGrid(const Point2D & world_point, int & grid_x, int & grid_y) const;

  /// @brief Convert grid coordinates to a point in world coordinates
  /// @param grid_x The x-coordinate in grid coordinates
  /// @param grid_y The y-coordinate in grid coordinates
  /// @return The corresponding point in world coordinates
  Point2D gridToWorld(int grid_x, int grid_y) const;

  uint32_t width() const { return width_; }

  uint32_t height() const { return height_; }

  double resolution() const { return resolution_; }

  double originX() const { return origin_x_; }

  double originY() const { return origin_y_; }

private:
  std::size_t index(int grid_x, int grid_y) const
  {
    return static_cast<std::size_t>(grid_y) * width_ + static_cast<std::size_t>(grid_x);
  }

  std::vector<int8_t> blocked_;
  uint32_t width_;
  uint32_t height_;
  double resolution_;
  double origin_x_;
  double origin_y_;
  int occupied_threshold_;
  bool unknown_is_free_;
  double inflation_radius_;
};

}  // namespace rrt_core

#endif  // RRT_CORE__GRID_COLLISION_CHECKER_HPP_
