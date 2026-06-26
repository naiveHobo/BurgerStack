#ifndef RRT_PLANNER__UTILS_HPP_
#define RRT_PLANNER__UTILS_HPP_

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Quaternion.hpp"

namespace rrt_planner {

geometry_msgs::msg::Quaternion quaternionFromYaw(double yaw) {
  tf2::Quaternion quaternion;
  quaternion.setRPY(0, 0, yaw);
  return tf2::toMsg(quaternion);
}

} // namespace rrt_planner

#endif // RRT_PLANNER__UTILS_HPP_
