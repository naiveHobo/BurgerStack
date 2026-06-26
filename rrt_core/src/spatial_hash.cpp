#include "rrt_core/spatial_hash.hpp"

#include "rrt_core/types.hpp"
#include "rrt_core/utils.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_map>
#include <utility>
#include <vector>

namespace rrt_core
{

SpatialHash::SpatialHash(double cell_size) : cell_size_(cell_size)
{
}

void SpatialHash::insert(const Point2D & point, int index)
{
  hash_table_[getCell(point)].push_back(index);
}

GridCell SpatialHash::getCell(const Point2D & point) const
{
  int cell_x = static_cast<int>(std::floor(point.x / cell_size_));
  int cell_y = static_cast<int>(std::floor(point.y / cell_size_));
  return std::make_pair(cell_x, cell_y);
}

int SpatialHash::nearest(const Point2D & point, const std::vector<Point2D> & nodes) const
{
  const auto grid_cell = getCell(point);
  int nearest_index = -1;
  double min_distance = std::numeric_limits<double>::max();

  for (int ring = 0; ring <= max_ring_; ++ring) {
    for (int dx = grid_cell.first - ring; dx <= grid_cell.first + ring; ++dx) {
      for (int dy = grid_cell.second - ring; dy <= grid_cell.second + ring; ++dy) {
        // Only check the cells on the current ring (Chebyshev distance)
        if (std::max(std::abs(dx - grid_cell.first), std::abs(dy - grid_cell.second)) != ring) {
          continue;
        }

        auto it = hash_table_.find({dx, dy});
        if (it == hash_table_.end()) {
          continue;
        }

        for (const int index : it->second) {
          const double dist = distance(point, nodes[index]);
          if (dist < min_distance) {
            min_distance = dist;
            nearest_index = index;
          }
        }
      }
    }

    // Any node sitting in ring r+1 or beyond is at least r * cell_size away from the
    // query point (need to cross at least r full cells to get there), so once
    // current best distance is ≤ ring * cell_size, no unexplored ring could possibly
    // contain anything closer, it's safe to stop.
    if (nearest_index >= 0 && static_cast<double>(ring) * cell_size_ >= min_distance) {
      break;
    }
  }

  return nearest_index;
}

std::vector<int> SpatialHash::withinRadius(
  const Point2D & point, double radius, const std::vector<Point2D> & nodes) const
{
  std::vector<int> indices;
  const auto grid_cell = getCell(point);
  const int max_ring = static_cast<int>(std::ceil(radius / cell_size_));

  for (int dx = grid_cell.first - max_ring; dx <= grid_cell.first + max_ring; ++dx) {
    for (int dy = grid_cell.second - max_ring; dy <= grid_cell.second + max_ring; ++dy) {
      const auto it = hash_table_.find({dx, dy});
      if (it == hash_table_.end()) {
        continue;
      }

      for (const int index : it->second) {
        if (distance(point, nodes[index]) <= radius) {
          indices.push_back(index);
        }
      }
    }
  }

  return indices;
}

}  // namespace rrt_core
