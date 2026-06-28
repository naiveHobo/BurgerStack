"""Phase 2 — localization + agentic contextual navigation.

Composes burger_bringup's stack in AMCL mode against the saved occupancy map, and
adds the Phase-2 semantic layer: map_server_node (serves the saved semantic map +
QuerySemanticMap) and execute_task_node (turns natural-language commands into Nav2
goals over that map). Send commands with:

    ros2 launch semantic_nav_bringup semantic_navigation.launch.py world:=small_house
    ros2 action send_goal /execute_task_node/execute_task \\
        semantic_nav_msgs/action/ExecuteTask "{command: 'go to the chair'}" --feedback
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, NotSubstitution, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_dir = get_package_share_directory("burger_bringup")
    default_params = PathJoinSubstitution(
        [FindPackageShare("semantic_nav_bringup"), "params", "semantic_nav.yaml"])
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("semantic_nav_bringup"), "rviz", "semantic_nav.rviz"])
    default_occ = os.path.expanduser("~/.semantic_nav/occupancy_map.yaml")
    default_sem = os.path.expanduser("~/.semantic_nav/semantic_map.json")

    world = LaunchConfiguration("world")
    occupancy_map_yaml = LaunchConfiguration("occupancy_map_yaml")
    semantic_map_path = LaunchConfiguration("semantic_map_path")
    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    rviz = LaunchConfiguration("rviz")

    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "bringup.launch.py")),
        launch_arguments={
            "world": world,
            "use_sim_time": use_sim_time,
            "localizer": "amcl",
            "map_yaml": occupancy_map_yaml,
            "explore": "false",
            # rviz:=true -> show the semantic view as the single window (suppress bringup's
            # planner.rviz, which the semantic config is a superset of).
            "use_rviz": NotSubstitution(rviz),
        }.items(),
    )

    map_server = Node(
        package="semantic_mapping", executable="map_server_node",
        parameters=[params_file, {"map_path": semantic_map_path}],
        output="screen")
    execute_task = Node(
        package="semantic_reasoning", executable="execute_task_node",
        parameters=[params_file], output="screen")

    rviz_node = Node(
        package="rviz2", executable="rviz2", name="rviz2_semantic",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(rviz), output="log")

    return LaunchDescription([
        DeclareLaunchArgument("world", default_value="small_house"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument(
            "occupancy_map_yaml", default_value=default_occ,
            description="Occupancy-grid map yaml saved in Phase 1 (AMCL loads it)."),
        DeclareLaunchArgument(
            "semantic_map_path", default_value=default_sem,
            description="Semantic map JSON saved in Phase 1 (map_server_node loads it)."),
        DeclareLaunchArgument("params_file", default_value=default_params),
        DeclareLaunchArgument(
            "rviz", default_value="false",
            description="Show the semantic RViz view (map + detection markers + camera) as the "
                        "single window, replacing bringup's planner.rviz. Default false keeps "
                        "the planner.rviz view."),
        bringup,
        map_server,
        execute_task,
        rviz_node,
    ])
