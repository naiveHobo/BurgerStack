"""AMCL localization against a static (ground-truth) occupancy map.

Brings up nav2's map_server + amcl + a localization lifecycle manager, providing /map and
the map->odom transform from a pre-built nav2 occupancy grid (image + yaml) -- e.g. your
self-saved map at burger_bringup/maps/map.yaml or the ground-truth map at maps/map_gt.yaml.
AMCL is the localization-only backend for this stack; slam_toolbox is used only for mapping
and continue (it cannot localize against an external occupancy grid). nav2.launch.py runs
this when `localizer:=amcl`.

amcl is seeded with set_initial_pose at the robot's spawn (x_pose/y_pose, yaw 0) so it
converges without a manual RViz "2D Pose Estimate". The map frame is assumed aligned to the
world frame (true for the vendored office map), so the world spawn pose is also the pose in
the map.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _localization_nodes(context, *args, **kwargs):
    nav2_params = os.path.join(
        get_package_share_directory("burger_bringup"), "params", "nav2.yaml")
    use_sim_time = LaunchConfiguration("use_sim_time")
    map_yaml = LaunchConfiguration("map_yaml").perform(context)
    x_pose = float(LaunchConfiguration("x_pose").perform(context))
    y_pose = float(LaunchConfiguration("y_pose").perform(context))

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[nav2_params, {
            "use_sim_time": use_sim_time,
            "yaml_filename": map_yaml,
        }],
    )

    amcl = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        parameters=[nav2_params, {
            "use_sim_time": use_sim_time,
            # Seed the filter at the robot's spawn so it converges without a manual
            # RViz pose estimate (map frame is aligned to the world frame).
            "set_initial_pose": True,
            "initial_pose.x": x_pose,
            "initial_pose.y": y_pose,
            "initial_pose.z": 0.0,
            "initial_pose.yaw": 0.0,
        }],
    )

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "autostart": True,
            "node_names": ["map_server", "amcl"],
        }],
    )

    return [map_server, amcl, lifecycle_manager]


def generate_launch_description():
    default_map_yaml = os.path.join(
        get_package_share_directory("burger_bringup"), "maps", "map_gt.yaml")

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument(
            "map_yaml", default_value=default_map_yaml,
            description="Path to the nav2 occupancy-grid map yaml for AMCL localization."),
        DeclareLaunchArgument(
            "x_pose", default_value="-6.0",
            description="AMCL initial pose X (robot spawn). Keep equal to sim spawn."),
        DeclareLaunchArgument(
            "y_pose", default_value="8.0",
            description="AMCL initial pose Y (robot spawn). Keep equal to sim spawn."),
        OpaqueFunction(function=_localization_nodes),
    ])
