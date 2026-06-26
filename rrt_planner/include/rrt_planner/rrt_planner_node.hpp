#ifndef RRT_PLANNER__RRT_PLANNER_NODE_HPP_
#define RRT_PLANNER__RRT_PLANNER_NODE_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "rrt_core/types.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace rrt_planner {

class RRTPlannerNode : public rclcpp::Node {
public:
  RRTPlannerNode();

private:
  void onGoal(geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void publishPath(const rrt_core::RRTResult &result,
                   const geometry_msgs::msg::PoseStamped &goal);
  void publishTree(const rrt_core::RRTResult &result);

private:
  std::string global_frame_;
  int occupied_threshold_{nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE};
  bool unknown_is_obstacle_{true};
  double inflation_radius_{0.2};

  rrt_core::RRTParams params_;

  rrt_core::Point2D start_point_{0, 0};
  bool has_start_point_{false};

  nav_msgs::msg::OccupancyGrid::SharedPtr map_;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr start_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr tree_pub_;
};

} // namespace rrt_planner

#endif // RRT_PLANNER__RRT_PLANNER_NODE_HPP_
