"""Phase 1 — mapping + exploration with live semantic mapping.

Composes burger_bringup's full stack (office world + RGB-D burger + SLAM + Nav2 +
frontier explorer) and adds the semantic layer: perception_node (RGB-D -> map-frame
detections), mapping_node (fuse -> live semantic map), and map_finalizer (the
handoff). A managed map_saver_server provides the occupancy-map SaveMap service the
finalizer calls.

When exploration is done, finalize the maps either automatically
(`auto_finalize:=true`, fires on the explorer's /exploration_complete) or manually
(`ros2 launch semantic_nav_bringup save_maps.launch.py`). Both write
~/.semantic_nav/{occupancy_map.yaml,.pgm, semantic_map.json}.

    ros2 launch semantic_nav_bringup semantic_mapping.launch.py world:=small_house explore:=true
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, NotSubstitution, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_dir = get_package_share_directory("burger_bringup")
    default_params = PathJoinSubstitution(
        [FindPackageShare("semantic_nav_bringup"), "params", "semantic_nav_mock.yaml"])
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("semantic_nav_bringup"), "rviz", "semantic_nav.rviz"])

    world = LaunchConfiguration("world")
    explore = LaunchConfiguration("explore")
    auto_finalize = LaunchConfiguration("auto_finalize")
    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    rviz = LaunchConfiguration("rviz")

    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "bringup.launch.py")),
        launch_arguments={
            "world": world,
            "use_sim_time": use_sim_time,
            "localizer": "slam_toolbox",
            "slam_mode": "mapping",
            "explore": explore,
            # rviz:=true -> show the semantic view as the single window (suppress bringup's
            # planner.rviz, which the semantic config is a superset of).
            "use_rviz": NotSubstitution(rviz),
        }.items(),
    )

    perception = Node(
        package="semantic_perception", executable="perception_node",
        parameters=[params_file], output="screen")
    mapping = Node(
        package="semantic_mapping", executable="mapping_node",
        parameters=[params_file], output="screen")
    finalizer = Node(
        package="semantic_mapping", executable="map_finalizer_node",
        parameters=[params_file, {"auto_finalize": ParameterValue(auto_finalize, value_type=bool)}],
        output="screen")

    # Managed map_saver_server -> provides /map_saver/save_map (mirrors the
    # localization_amcl lifecycle-manager pattern).
    map_saver = Node(
        package="nav2_map_server", executable="map_saver_server", name="map_saver",
        parameters=[{"use_sim_time": use_sim_time, "save_map_timeout": 5.0,
                     "free_thresh_default": 0.25, "occupied_thresh_default": 0.65}],
        output="screen")
    map_saver_lifecycle = Node(
        package="nav2_lifecycle_manager", executable="lifecycle_manager",
        name="lifecycle_manager_map_saver",
        parameters=[{"use_sim_time": use_sim_time, "autostart": True,
                     "node_names": ["map_saver"]}],
        output="screen")

    rviz_node = Node(
        package="rviz2", executable="rviz2", name="rviz2_semantic",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(rviz), output="log")

    return LaunchDescription([
        DeclareLaunchArgument("world", default_value="small_house"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument(
            "explore", default_value="true",
            description="Run the frontier explorer to autonomously build the map."),
        DeclareLaunchArgument(
            "auto_finalize", default_value="false",
            description="Auto-save both maps when the explorer publishes /exploration_complete."),
        DeclareLaunchArgument("params_file", default_value=default_params),
        DeclareLaunchArgument(
            "rviz", default_value="false",
            description="Show the semantic RViz view (map + detection markers + camera) as the "
                        "single window, replacing bringup's planner.rviz. Default false keeps "
                        "the planner.rviz view."),
        bringup,
        perception,
        mapping,
        finalizer,
        map_saver,
        map_saver_lifecycle,
        rviz_node,
    ])
