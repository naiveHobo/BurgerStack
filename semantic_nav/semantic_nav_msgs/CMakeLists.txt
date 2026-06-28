cmake_minimum_required(VERSION 3.8)
project(semantic_nav_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(action_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SemanticObject.msg"
  "msg/SemanticObjectArray.msg"
  "msg/Detection.msg"
  "msg/DetectionArray.msg"
  "srv/QuerySemanticMap.srv"
  "srv/BuildSemanticMap.srv"
  "action/ExecuteTask.action"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs sensor_msgs action_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
