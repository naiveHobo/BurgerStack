#ifndef RRT_CORE__SPATIAL_HASH_HPP_
#define RRT_CORE__SPATIAL_HASH_HPP_

#include <unordered_map>
#include <vector>

#include "rrt_core/types.hpp"

namespace rrt_core
{

class SpatialHash
{
public:
  explicit SpatialHash(double cell_size);

  /// @brief Insert a point into the spatial hash
  /// @param point The 2D point to insert
  /// @param index The index of the point in the original point list
  void insert(const Point2D & point, int index);

  /// @brief Find the index of the nearest point to a given query point
  /// @param point The query point for which to find the nearest neighbor
  /// @param nodes The list of points to search for the nearest neighbor
  /// @return The index of the nearest point in the nodes vector, or -1 if no points are found
  int nearest(const Point2D & point, const std::vector<Point2D> & nodes) const;

  /// @brief Find the indices of all points within a given radius of a query point
  /// @param point The query point for which to find nearby points
  /// @param radius The radius within which to search for points
  /// @param nodes The list of points to search for nearby points
  /// @return A vector of indices of points in the nodes vector that are within the specified
  ///         radius of the query point
  std::vector<int> withinRadius(
    const Point2D & point,
    double radius,
    const std::vector<Point2D> & nodes) const;

private:
  /// @brief Get the grid cell corresponding to a given point in 2D space
  /// @param point The 2D point for which to find the corresponding grid cell
  /// @return The grid cell (as a pair of integers) that contains the point
  GridCell getCell(const Point2D & point) const;

  /// Size of each cell in the spatial hash
  double cell_size_;
  /// Maximum number of cells to store in the hash table before resizing
  int max_ring_{10000};
  /// Hash table mapping grid cells to indices of points in those cells
  std::unordered_map<GridCell, std::vector<int>, CellHash> hash_table_;
};

}  // namespace rrt_core

#endif  // RRT_CORE__SPATIAL_HASH_HPP_
