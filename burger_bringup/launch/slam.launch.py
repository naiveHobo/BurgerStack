"""SLAM with slam_toolbox, using our tuned params.

Two modes are selected with the `slam_mode` argument:
  * mapping  - fresh online async mapping (default; behaviorally identical to before).
  * continue - deserialize a saved pose-graph (`map`) and keep mapping on top of it.

(Localization-only is handled by AMCL, not slam_toolbox -- see localization_amcl.launch.py,
selected via nav2.launch.py's `localizer:=amcl`. slam_toolbox is used only for building or
extending a map, which AMCL cannot do.)

`map` is the serialized pose-graph base name (no extension) and defaults to
burger_bringup/maps/map; it is only used by the continue mode.

For continue we also pass `map_start_pose` so slam_toolbox knows where the robot is in the
loaded map at startup. This MUST equal the robot's actual spawn pose: our maps are anchored
to the world/odom frame (the robot maps starting at its spawn, e.g. (-6, 8) for the
office), so leaving it at the origin would place the robot metres away from the loaded map.
We derive it from the same `x_pose`/`y_pose` used to spawn the robot in sim.launch.py so the
two can never drift.

We launch the slam_toolbox node directly here (instead of delegating to slam_toolbox's
online_async_launch.py) so the saved-map path and start pose can be injected as parameter
overrides. The mapping path uses the same node and parameters as the upstream launch file,
so the existing `bringup`/`office` flows are unchanged.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _slam_node(context, *args, **kwargs):
    bringup = get_package_share_directory("burger_bringup")
    slam_params = os.path.join(bringup, "params", "slam.yaml")
    use_sim_time = LaunchConfiguration("use_sim_time")
    slam_mode = LaunchConfiguration("slam_mode").perform(context)
    map_name = LaunchConfiguration("map").perform(context)

    # Robot spawn pose in the map/world frame, used to relocalize into a loaded map.
    # Must match how sim.launch.py spawns the robot (x_pose/y_pose, yaw 0).
    x_pose = float(LaunchConfiguration("x_pose").perform(context))
    y_pose = float(LaunchConfiguration("y_pose").perform(context))
    map_start_pose = [x_pose, y_pose, 0.0]

    if slam_mode == "mapping":
        node_params = [slam_params, {"use_sim_time": use_sim_time}]
    elif slam_mode == "continue":
        node_params = [slam_params, {
            "use_sim_time": use_sim_time,
            "map_file_name": map_name,
            "map_start_pose": map_start_pose,
        }]
    else:
        raise RuntimeError(
            f"Invalid slam_mode '{slam_mode}'. Expected one of: mapping | continue. "
            "(Localization-only uses AMCL via localizer:=amcl, not slam_toolbox.)")

    executable = "async_slam_toolbox_node"

    return [Node(
        package="slam_toolbox",
        executable=executable,
        name="slam_toolbox",
        output="screen",
        parameters=node_params,
    )]


def generate_launch_description():
    default_map = os.path.join(
        get_package_share_directory("burger_bringup"), "maps", "map")

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument(
            "slam_mode", default_value="mapping",
            description="mapping (fresh) | continue (resume saved map)."),
        DeclareLaunchArgument(
            "map", default_value=default_map,
            description="Serialized pose-graph base name (no extension); used by the "
                        "continue mode."),
        DeclareLaunchArgument(
            "x_pose", default_value="-6.0",
            description="Robot spawn X; map_start_pose for continue. "
                        "Must match sim.launch.py's spawn."),
        DeclareLaunchArgument(
            "y_pose", default_value="8.0",
            description="Robot spawn Y; map_start_pose for continue. "
                        "Must match sim.launch.py's spawn."),
        OpaqueFunction(function=_slam_node),
    ])
