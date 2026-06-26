"""One-command bringup: Gazebo sim + SLAM + Nav2 (RRT planner) + RViz.

Starts Gazebo first, then brings up the navigation stack after a short delay so
the Burger has spawned and is publishing /scan and TF before SLAM and Nav2 come
up (avoids early 'waiting for transform' noise and lifecycle activation races).

nav2.launch.py already pulls in slam.launch.py and RViz, so this file just
composes sim + nav2 and staggers their startup.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    launch_dir = os.path.join(get_package_share_directory("burger_bringup"), "launch")
    world = LaunchConfiguration("world")
    use_sim_time = LaunchConfiguration("use_sim_time")
    nav2_delay = LaunchConfiguration("nav2_delay")

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "sim.launch.py")),
        launch_arguments={"world": world}.items(),
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "nav2.launch.py")),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "world", default_value="world",
            description="TurtleBot3 Gazebo world: 'house' (office-like) or 'world' (arena)."),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument(
            "nav2_delay", default_value="5.0",
            description="Seconds to wait after Gazebo before starting SLAM/Nav2/RViz."),

        sim,
        TimerAction(period=nav2_delay, actions=[nav2]),
    ])
