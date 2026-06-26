#ifndef RRT_PLANNER__RRT_GLOBAL_PLANNER_HPP_
#define RRT_PLANNER__RRT_GLOBAL_PLANNER_HPP_

#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"

#include "rrt_core/types.hpp"

namespace rrt_planner {

/// @brief RRT Global Planner implemented as a Nav2 Global Planner plugin
/// wrapping rrt_core library
class RRTGlobalPlanner : public nav2_core::GlobalPlanner {
public:
  void configure(
      const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent, std::string name,
      std::shared_ptr<tf2_ros::Buffer> tf,
      std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  nav_msgs::msg::Path
  createPlan(const geometry_msgs::msg::PoseStamped &start,
             const geometry_msgs::msg::PoseStamped &goal) override;

private:
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::string global_frame_;
  std::string name_;

  rrt_core::RRTParams params_;
  unsigned char lethal_cost_{nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE};
  bool allow_unknown_{false};

  rclcpp::Logger logger_{rclcpp::get_logger("RRTGlobalPlanner")};
};

} // namespace rrt_planner

#endif // RRT_PLANNER__RRT_GLOBAL_PLANNER_HPP_
