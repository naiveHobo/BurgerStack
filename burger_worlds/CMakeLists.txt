cmake_minimum_required(VERSION 3.8)
project(burger_worlds)

find_package(ament_cmake REQUIRED)

# Vendored Gazebo Classic worlds and their assets. The whole tree is installed to
# the package share dir; sim.launch.py adds <share>/worlds/<name>/models to
# GAZEBO_MODEL_PATH and <share>/worlds/<name> to GAZEBO_RESOURCE_PATH at runtime.
install(DIRECTORY worlds DESTINATION share/${PROJECT_NAME})

ament_package()
