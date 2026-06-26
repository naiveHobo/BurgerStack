#ifndef RRT_PLANNER__UTILS_HPP_
#define RRT_PLANNER__UTILS_HPP_

#include <string>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rrt_core/types.hpp"
#include "std_msgs/msg/header.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace rrt_planner {

geometry_msgs::msg::Quaternion quaternionFromYaw(double yaw) {
  tf2::Quaternion quaternion;
  quaternion.setRPY(0, 0, yaw);
  return tf2::toMsg(quaternion);
}

visualization_msgs::msg::MarkerArray
createTreeVisualization(const rrt_core::RRTResult &result,
                        const std_msgs::msg::Header &header) {
  visualization_msgs::msg::MarkerArray msg;

  visualization_msgs::msg::Marker edges;
  edges.header = header;
  edges.ns = "tree";
  edges.id = 0;
  edges.type = visualization_msgs::msg::Marker::LINE_LIST;
  edges.action = visualization_msgs::msg::Marker::ADD;
  edges.scale.x = 0.01;
  edges.color.b = 1.0;
  edges.color.a = 0.6;
  edges.pose.orientation.w = 1.0;

  for (size_t i = 0; i < result.tree_nodes.size(); ++i) {
    const int parent = result.tree_parents[i];
    if (parent < 0) {
      continue;
    }
    geometry_msgs::msg::Point a;
    a.x = result.tree_nodes[i].x;
    a.y = result.tree_nodes[i].y;
    geometry_msgs::msg::Point b;
    b.x = result.tree_nodes[static_cast<std::size_t>(parent)].x;
    b.y = result.tree_nodes[static_cast<std::size_t>(parent)].y;
    edges.points.push_back(a);
    edges.points.push_back(b);
  }

  msg.markers.push_back(edges);

  visualization_msgs::msg::Marker route;
  route.header = edges.header;
  route.ns = "path";
  route.id = 1;
  route.type = visualization_msgs::msg::Marker::LINE_STRIP;
  route.action = visualization_msgs::msg::Marker::ADD;
  route.scale.x = 0.02;
  route.color.g = 1.0;
  route.color.a = 1.0;
  route.pose.orientation.w = 1.0;
  for (const auto &p : result.path) {
    geometry_msgs::msg::Point point;
    point.x = p.x;
    point.y = p.y;
    route.points.push_back(point);
  }
  msg.markers.push_back(route);

  return msg;
}

} // namespace rrt_planner

#endif // RRT_PLANNER__UTILS_HPP_
