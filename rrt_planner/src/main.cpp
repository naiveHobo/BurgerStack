#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rrt_planner/rrt_planner_node.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rrt_planner::RRTPlannerNode>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
