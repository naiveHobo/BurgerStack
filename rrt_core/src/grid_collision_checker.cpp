#include <vector>
#include <cmath>

#include "rrt_core/types.hpp"
#include "rrt_core/collision_checker.hpp"
#include "rrt_core/grid_collision_checker.hpp"

namespace rrt_core
{

GridCollisionChecker::GridCollisionChecker(
  const std::vector<int8_t> & grid,
  unsigned int width,
  unsigned int height,
  double resolution,
  double origin_x,
  double origin_y,
  int occupied_threshold,
  bool unknown_is_free,
  double inflation_radius)
: CollisionChecker(),
  width_(width),
  height_(height),
  resolution_(resolution),
  origin_x_(origin_x),
  origin_y_(origin_y),
  occupied_threshold_(occupied_threshold),
  unknown_is_free_(unknown_is_free),
  inflation_radius_(inflation_radius)
{
  const std::size_t n = static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
  blocked_.assign(n, 0);

  // First pass: mark occupied cells
  std::vector<int8_t> occupied(n, 0);
  const std::size_t limit = std::min(n, grid.size());
  for (std::size_t i = 0; i < limit; ++i) {
    const int8_t cell_value = grid[i];
    if (cell_value >= occupied_threshold_) {
      occupied[i] = 1;
      blocked_[i] = 1;
    } else if (cell_value == -1 && !unknown_is_free_) {
      blocked_[i] = 1;
    }
  }

  // Second pass: inflate occupied cells by the specified inflation radius
  const int inflation_cells = static_cast<int>(std::ceil(inflation_radius_ / resolution_));
  if (inflation_cells > 0) {
    const int r2 = inflation_cells * inflation_cells;
    const int width_int = static_cast<int>(width_);
    const int height_int = static_cast<int>(height_);
    for (int cy = 0; cy < height_int; ++cy) {
      for (int cx = 0; cx < width_int; ++cx) {
        if (!occupied[index(cx, cy)]) {
          continue;
        }

        for (int dy = -inflation_cells; dy <= inflation_cells; ++dy) {
          for (int dx = -inflation_cells; dx <= inflation_cells; ++dx) {
            if (dx * dx + dy * dy > r2) {
              continue;
            }

            const int nx = cx + dx;
            const int ny = cy + dy;
            if (nx >= 0 && nx < width_int && ny >= 0 && ny < height_int) {
              blocked_[index(nx, ny)] = 1;
            }
          }
        }
      }
    }
  }
}

bool GridCollisionChecker::worldToGrid(
  const Point2D & world_point,
  int & grid_x,
  int & grid_y) const
{
  grid_x = static_cast<int>((world_point.x - origin_x_) / resolution_);
  grid_y = static_cast<int>((world_point.y - origin_y_) / resolution_);
  return grid_x >= 0 && grid_y >= 0 && grid_x < static_cast<int>(width_) &&
         grid_y < static_cast<int>(height_);
}

Point2D GridCollisionChecker::gridToWorld(int grid_x, int grid_y) const
{
  return {
    origin_x_ + (grid_x + 0.5) * resolution_,
    origin_y_ + (grid_y + 0.5) * resolution_
  };
}

bool GridCollisionChecker::isFree(const Point2D & point) const
{
  int grid_x, grid_y;

  if (!worldToGrid(point, grid_x, grid_y)) {
    return false;
  }

  return blocked_[index(grid_x, grid_y)] == 0;
}

bool GridCollisionChecker::isLineFree(const Point2D & a, const Point2D & b) const
{
  const double dx = b.x - a.x;
  const double dy = b.y - a.y;
  const double length = std::hypot(dx, dy);

  if (length < 1e-6) {
    return isFree(a);
  }

  // Step size is set to half the grid resolution to ensure that we
  // check points along the line at a finer granularity than the grid cells.
  const double step_size = resolution_ / 0.5;
  const int samples = static_cast<int>(std::ceil(length / step_size));

  for (int i = 0; i <= samples; ++i) {
    const double t = static_cast<double>(i) / samples;
    if (!isFree({a.x + t * dx, a.y + t * dy})) {
      return false;
    }
  }

  return true;
}

}  // namespace rrt_core
