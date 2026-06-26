"""SLAM with slam_toolbox (async online mapping), using our tuned params."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup = get_package_share_directory("burger_bringup")
    slam_toolbox = get_package_share_directory("slam_toolbox")
    use_sim_time = LaunchConfiguration("use_sim_time")
    slam_params = os.path.join(bringup, "params", "slam.yaml")

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(slam_toolbox, "launch", "online_async_launch.py")),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "slam_params_file": slam_params,
            }.items(),
        ),
    ])
