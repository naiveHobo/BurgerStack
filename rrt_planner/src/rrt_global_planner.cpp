#include <cmath>
#include <memory>
#include <string>

#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rrt_core/rrt.hpp"

#include "rrt_planner/costmap_collision_checker.hpp"
#include "rrt_planner/rrt_global_planner.hpp"
#include "rrt_planner/utils.hpp"

namespace rrt_planner {

void RRTGlobalPlanner::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) {
  node_ = parent.lock();
  name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  global_frame_ = costmap_ros_->getGlobalFrameID();
  logger_ = node_->get_logger();

  nav2_util::declare_parameter_if_not_declared(node_, name_ + ".step_size",
                                               rclcpp::ParameterValue(0.3));
  nav2_util::declare_parameter_if_not_declared(node_, name_ + ".goal_bias",
                                               rclcpp::ParameterValue(0.1));
  nav2_util::declare_parameter_if_not_declared(node_, name_ + ".goal_tolerance",
                                               rclcpp::ParameterValue(0.25));
  nav2_util::declare_parameter_if_not_declared(node_, name_ + ".max_iterations",
                                               rclcpp::ParameterValue(50000));
  nav2_util::declare_parameter_if_not_declared(node_, name_ + ".use_rrt_star",
                                               rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(node_, name_ + ".rewire_radius",
                                               rclcpp::ParameterValue(0.75));
  nav2_util::declare_parameter_if_not_declared(node_, name_ + ".smooth",
                                               rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(node_, name_ + ".seed",
                                               rclcpp::ParameterValue(42));
  nav2_util::declare_parameter_if_not_declared(node_, name_ + ".lethal_cost",
                                               rclcpp::ParameterValue(253));
  nav2_util::declare_parameter_if_not_declared(node_, name_ + ".allow_unknown",
                                               rclcpp::ParameterValue(false));

  int max_iter = 50000;
  int seed = 42;
  int lethal = 253;
  node_->get_parameter(name_ + ".step_size", params_.step_size);
  node_->get_parameter(name_ + ".goal_bias", params_.goal_bias);
  node_->get_parameter(name_ + ".goal_tolerance", params_.goal_tolerance);
  node_->get_parameter(name_ + ".max_iterations", max_iter);
  node_->get_parameter(name_ + ".use_rrt_star", params_.use_rrt_star);
  node_->get_parameter(name_ + ".neighbor_radius", params_.neighbor_radius);
  node_->get_parameter(name_ + ".smooth", params_.greedy_smoothing);
  node_->get_parameter(name_ + ".seed", seed);
  node_->get_parameter(name_ + ".lethal_cost", lethal);
  node_->get_parameter(name_ + ".allow_unknown", allow_unknown_);
  params_.max_iterations = static_cast<std::size_t>(max_iter);
  params_.seed = static_cast<unsigned int>(seed);
  lethal_cost_ = static_cast<unsigned char>(lethal);

  RCLCPP_INFO(
      logger_,
      "Configured RRT global planner '%s' (frame=%s, rrt_star=%s, step=%.2f).",
      name_.c_str(), global_frame_.c_str(),
      params_.use_rrt_star ? "true" : "false", params_.step_size);
}

void RRTGlobalPlanner::activate() {
  RCLCPP_INFO(logger_, "Activating RRT Global Planner '%s'", name_.c_str());
}

void RRTGlobalPlanner::deactivate() {
  RCLCPP_INFO(logger_, "Deactivating RRT Global Planner '%s'", name_.c_str());
}

void RRTGlobalPlanner::cleanup() {
  RCLCPP_INFO(logger_, "Cleaning up RRT Global Planner '%s'", name_.c_str());
}

nav_msgs::msg::Path
RRTGlobalPlanner::createPlan(const geometry_msgs::msg::PoseStamped &start,
                             const geometry_msgs::msg::PoseStamped &goal) {
  nav_msgs::msg::Path path;
  path.header.frame_id = global_frame_;
  path.header.stamp = node_->now();

  auto costmap = costmap_ros_->getCostmap();
  std::lock_guard lock(*(costmap->getMutex()));

  auto checker = std::make_shared<CostmapCollisionChecker>(
      costmap, lethal_cost_, allow_unknown_);
  rrt_core::RRT rrt(params_, checker);

  const rrt_core::RRTResult result =
      rrt.plan({start.pose.position.x, start.pose.position.y},
               {goal.pose.position.x, goal.pose.position.y});

  if (!result.success) {
    RCLCPP_WARN(logger_,
                "RRT found no path after %zu iterations (tree size: %zu)",
                result.iterations, result.tree_size);
    return path;
  }

  path.poses.reserve(result.path.size());
  for (size_t i = 0; i < result.path.size(); ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = result.path[i].x;
    pose.pose.position.y = result.path[i].y;
    if (i + 1 < result.path.size()) {
      const double yaw = std::atan2(result.path[i + 1].y - result.path[i].y,
                                    result.path[i + 1].x - result.path[i].x);
      pose.pose.orientation = quaternionFromYaw(yaw);
    } else {
      pose.pose.orientation = goal.pose.orientation;
    }
    path.poses.push_back(pose);
  }

  RCLCPP_INFO(logger_, "RRT planned a path: %zu poses (tree: %zu, iters: %zu)",
              path.poses.size(), result.tree_size, result.iterations);
  return path;
}

} // namespace rrt_planner

PLUGINLIB_EXPORT_CLASS(rrt_planner::RRTGlobalPlanner, nav2_core::GlobalPlanner)
