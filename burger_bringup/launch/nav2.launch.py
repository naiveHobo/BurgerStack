"""Nav2 driving the robot with the custom RRT global planner plugin.

Assumes `pixi run sim` is already running. Send goals with RViz's
'Nav2 Goal' / '2D Goal Pose' tool; the planner_server plans them with
rrt_planner/RRTGlobalPlanner and the controller follows the path.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup = get_package_share_directory("burger_bringup")
    nav2_bringup = get_package_share_directory("nav2_bringup")
    use_sim_time = LaunchConfiguration("use_sim_time")

    nav2_params = os.path.join(bringup, "params", "nav2.yaml")
    rviz_config = os.path.join(bringup, "rviz", "planner.rviz")

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup, "launch", "slam.launch.py")),
            launch_arguments={"use_sim_time": use_sim_time}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup, "launch", "navigation_launch.py")),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "params_file": nav2_params,
            }.items(),
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config],
            parameters=[{"use_sim_time": use_sim_time}],
            output="log",
        ),
    ])
