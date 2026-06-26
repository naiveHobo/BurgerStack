#ifndef RRT_PLANNER__COSTMAP_COLLISION_CHECKER_HPP_
#define RRT_PLANNER__COSTMAP_COLLISION_CHECKER_HPP_

#include <cmath>
#include <memory>

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"

#include "rrt_core/collision_checker.hpp"
#include "rrt_core/types.hpp"

namespace rrt_planner {

class CostmapCollisionChecker : public rrt_core::CollisionChecker {
public:
  CostmapCollisionChecker(const nav2_costmap_2d::Costmap2D *costmap,
                          unsigned char lethal_cost, bool allow_unknown)
      : costmap_(costmap), lethal_cost_(lethal_cost),
        allow_unknown_(allow_unknown) {}

  double getMinX() const override { return costmap_->getOriginX(); }

  double getMinY() const override { return costmap_->getOriginY(); }

  double getMaxX() const override {
    return costmap_->getOriginX() +
           costmap_->getSizeInCellsX() * costmap_->getResolution();
  }

  double getMaxY() const override {
    return costmap_->getOriginY() +
           costmap_->getSizeInCellsY() * costmap_->getResolution();
  }

  bool isFree(const rrt_core::Point2D &p) const override {
    uint32_t mx = 0;
    uint32_t my = 0;
    if (!costmap_->worldToMap(p.x, p.y, mx, my)) {
      return false;
    }
    const auto cost = costmap_->getCost(mx, my);
    if (cost == nav2_costmap_2d::NO_INFORMATION) {
      return allow_unknown_;
    }
    return cost < lethal_cost_;
  }

  bool isLineFree(const rrt_core::Point2D &a,
                  const rrt_core::Point2D &b) const override {
    const double dx = (b.x - a.x);
    const double dy = (b.y - a.y);
    const double length = std::hypot(dx, dy);
    if (length < 1e-6) {
      return isFree(a);
    }
    const double step_size = costmap_->getResolution() * 0.5;
    const int samples = static_cast<int>(std::ceil(length / step_size));
    for (int i = 0; i <= samples; ++i) {
      const double t = static_cast<double>(i) / samples;
      if (!isFree({a.x + t * dx, a.y + t * dy})) {
        return false;
      }
    }
    return true;
  }

private:
  const nav2_costmap_2d::Costmap2D *costmap_;
  unsigned char lethal_cost_;
  bool allow_unknown_;
};

} // namespace rrt_planner

#endif // RRT_PLANNER__COSTMAP_COLLISION_CHECKER_HPP_
