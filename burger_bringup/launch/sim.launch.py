"""Bring up Gazebo with a TurtleBot3 Burger in an office-like world.

Run this in its own terminal, then `pixi run explore` (or `nav2-rrt`) alongside.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def launch_setup(context, *args, **kwargs):
    world = LaunchConfiguration("world").perform(context)
    tb3_gazebo = get_package_share_directory("turtlebot3_gazebo")
    launch_file = os.path.join(tb3_gazebo, "launch", f"turtlebot3_{world}.launch.py")
    return [
        IncludeLaunchDescription(PythonLaunchDescriptionSource(launch_file)),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "world", default_value="house",
            description="TurtleBot3 Gazebo world: 'house' (office-like) or 'world' (arena)."),
        OpaqueFunction(function=launch_setup),
    ])
