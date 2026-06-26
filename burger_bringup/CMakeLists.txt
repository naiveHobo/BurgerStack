cmake_minimum_required(VERSION 3.8)
project(burger_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch params rviz maps behavior_trees
  DESTINATION share/${PROJECT_NAME})

ament_package()
