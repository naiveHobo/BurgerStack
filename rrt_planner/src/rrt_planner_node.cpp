#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rrt_core/grid_collision_checker.hpp"
#include "rrt_planner/rrt_planner_node.hpp"
#include "rrt_planner/utils.hpp"

#include <rrt_core/rrt.hpp>

namespace rrt_planner {

RRTPlannerNode::RRTPlannerNode() : rclcpp::Node("rrt_planner_node") {
  global_frame_ = declare_parameter<std::string>("global_frame", "map");
  occupied_threshold_ = declare_parameter<int>("occupied_threshold", 50);
  unknown_is_obstacle_ = declare_parameter<bool>("unknown_is_obstacle", true);
  inflation_radius_ = declare_parameter<double>("inflation_radius", 0.2);

  params_.step_size = declare_parameter<double>("step_size", 0.3);
  params_.goal_bias = declare_parameter<double>("goal_bias", 0.1);
  params_.goal_tolerance = declare_parameter<double>("goal_tolerance", 0.25);
  params_.max_iterations =
      static_cast<size_t>(declare_parameter<int>("max_iterations", 50000));
  params_.use_rrt_star = declare_parameter<bool>("use_rrt_star", true);
  params_.neighbor_radius = declare_parameter<double>("neighbor_radius", 0.75);
  params_.greedy_smoothing = declare_parameter<bool>("smooth", true);
  params_.seed = static_cast<uint32_t>(declare_parameter<int>("seed", 42));

  map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "map", rclcpp::QoS(1).transient_local().reliable(),
      [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg) { map_ = msg; });

  start_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "initialpose", 10,
      [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        start_point_ = {msg->pose.position.x, msg->pose.position.y};
        has_start_point_ = true;
        RCLCPP_INFO(get_logger(), "Start point set to (%.2lf, %.2lf)",
                    start_point_.x, start_point_.y);
      });

  goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "goal_pose", 10,
      std::bind(&RRTPlannerNode::onGoal, this, std::placeholders::_1));

  path_pub_ = create_publisher<nav_msgs::msg::Path>("rrt_path", 10);
  markers_pub_ =
      create_publisher<visualization_msgs::msg::MarkerArray>("rrt_tree", 10);

  RCLCPP_INFO(get_logger(), "RRT Planner node ready!");
}

void RRTPlannerNode::onGoal(geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  if (!map_) {
    RCLCPP_WARN(get_logger(), "No map available! Ignoring goal.");
    return;
  }

  if (!has_start_point_) {
    RCLCPP_WARN(get_logger(),
                "Start pose not received yet! Defaulting to (0, 0).");
  }

  const auto &info = map_->info;
  auto checker = std::make_shared<rrt_core::GridCollisionChecker>(
      map_->data, info.width, info.height, info.resolution,
      info.origin.position.x, info.origin.position.y, occupied_threshold_,
      !unknown_is_obstacle_, inflation_radius_);

  rrt_core::RRT planner(params_, checker);
  const rrt_core::Point2D goal_point = {msg->pose.position.x,
                                        msg->pose.position.y};
  const rrt_core::RRTResult result = planner.plan(start_point_, goal_point);

  if (result.success) {
    RCLCPP_INFO(get_logger(),
                "RRT path found: %zu waypoints (tree: %zu, iters: %zu)",
                result.path.size(), result.tree_size, result.iterations);
  } else {
    RCLCPP_WARN(get_logger(), "RRT failed (iters: %zu, tree: %zu)",
                result.iterations, result.tree_size);
  }

  publishPath(result, *msg);

  auto markers = createTreeVisualization(result, msg->header);
  markers_pub_->publish(markers);
}

void RRTPlannerNode::publishPath(const rrt_core::RRTResult &result,
                                 const geometry_msgs::msg::PoseStamped &goal) {
  nav_msgs::msg::Path path;
  path.header.frame_id = global_frame_;
  path.header.stamp = get_clock()->now();
  path.poses.reserve(result.path.size());
  for (size_t i = 0; i < result.path.size(); ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = result.path[i].x;
    pose.pose.position.y = result.path[i].y;
    if (i + 1 < result.path.size()) {
      pose.pose.orientation = quaternionFromYaw(
          std::atan2(result.path[i + 1].y - result.path[i].y,
                     result.path[i + 1].x - result.path[i].x));
    } else {
      pose.pose.orientation = goal.pose.orientation;
    }
    path.poses.push_back(pose);
  }
  path_pub_->publish(path);
}

} // namespace rrt_planner
