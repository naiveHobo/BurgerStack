"""Standalone RRT planner demo on a saved occupancy map (no Gazebo).

Serves a pre-built nav2 occupancy map (default: the self-saved
burger_bringup/maps/map.yaml) on /map, runs the standalone rrt_planner_node, and
opens RViz with the planner config. This is the "occupancy grid + start + goal ->
nav_msgs/Path" demo from Section 3 of the assignment, isolated from the full
navigation stack.

Set the goal with RViz's '2D Goal Pose' tool (publishes geometry_msgs/PoseStamped
on /goal_pose, which the node consumes directly). Set the start by publishing a
geometry_msgs/PoseStamped on /initialpose, e.g.

    ros2 topic pub --once /initialpose geometry_msgs/msg/PoseStamped \
        '{header: {frame_id: map}, pose: {position: {x: 0.0, y: 10.0}}}'

(RViz's '2D Pose Estimate' publishes a PoseWithCovarianceStamped, a different type,
so the start is set by topic; if none is given the node defaults the start to the
origin). The node publishes the search tree on /rrt_tree and the path on /rrt_path,
both shown by planner.rviz.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup = get_package_share_directory("burger_bringup")
    default_map_yaml = os.path.join(bringup, "maps", "map.yaml")
    rviz_config = os.path.join(bringup, "rviz", "planner.rviz")

    # No Gazebo here, so there is no /clock; run everything on wall time.
    use_sim_time = LaunchConfiguration("use_sim_time")
    map_yaml = LaunchConfiguration("map_yaml")
    use_rviz = LaunchConfiguration("use_rviz")

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "yaml_filename": map_yaml,
            "topic_name": "map",
            "frame_id": "map",
        }],
    )

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_plan_demo",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "autostart": True,
            "node_names": ["map_server"],
        }],
    )

    rrt_planner_node = Node(
        package="rrt_planner",
        executable="rrt_planner_node",
        name="rrt_planner_node",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(use_rviz),
        output="log",
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument(
            "map_yaml", default_value=default_map_yaml,
            description="Path to the nav2 occupancy-grid map yaml to plan on."),
        DeclareLaunchArgument(
            "use_rviz", default_value="true",
            description="Start RViz with the planner config. Set false to suppress it."),
        map_server,
        lifecycle_manager,
        rrt_planner_node,
        rviz,
    ])
