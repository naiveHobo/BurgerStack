"""Nav2 driving the robot with the custom RRT global planner plugin.

Assumes `pixi run sim` is already running. Send goals with RViz's
'Nav2 Goal' / '2D Goal Pose' tool; the planner_server plans them with
rrt_planner/RRTGlobalPlanner and the controller follows the path.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    bringup = get_package_share_directory("burger_bringup")
    nav2_bringup = get_package_share_directory("nav2_bringup")
    use_sim_time = LaunchConfiguration("use_sim_time")
    slam_mode = LaunchConfiguration("slam_mode")
    map_name = LaunchConfiguration("map")
    x_pose = LaunchConfiguration("x_pose")
    y_pose = LaunchConfiguration("y_pose")
    localizer = LaunchConfiguration("localizer")
    map_yaml = LaunchConfiguration("map_yaml")
    use_rviz = LaunchConfiguration("use_rviz")

    nav2_params = os.path.join(bringup, "params", "nav2.yaml")
    rviz_config = os.path.join(bringup, "rviz", "planner.rviz")
    default_map = os.path.join(bringup, "maps", "map")
    default_map_yaml = os.path.join(bringup, "maps", "map_gt.yaml")

    # The localization backend is either slam_toolbox (default; also does mapping/continue)
    # or AMCL + map_server against a static occupancy map. They are mutually exclusive.
    use_amcl = PythonExpression(["'", localizer, "' == 'amcl'"])

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument(
            "slam_mode", default_value="mapping",
            description="slam_toolbox mode: mapping (fresh) | continue (resume saved map). "
                        "Ignored when localizer:=amcl."),
        DeclareLaunchArgument(
            "map", default_value=default_map,
            description="Serialized pose-graph base name; used by continue/localization."),
        DeclareLaunchArgument(
            "x_pose", default_value="-6.0",
            description="Robot spawn X; slam map_start_pose / AMCL initial pose."),
        DeclareLaunchArgument(
            "y_pose", default_value="8.0",
            description="Robot spawn Y; slam map_start_pose / AMCL initial pose."),
        DeclareLaunchArgument(
            "localizer", default_value="slam_toolbox",
            description="Backend: slam_toolbox (mapping/continue) | amcl (localization-only "
                        "against a static occupancy map). Localization uses AMCL."),
        DeclareLaunchArgument(
            "map_yaml", default_value=default_map_yaml,
            description="Occupancy-grid map yaml for localizer:=amcl (e.g. ground-truth map_gt.yaml)."),
        DeclareLaunchArgument(
            "use_rviz", default_value="true",
            description="Start RViz with the planner config. Set false to suppress it "
                        "(e.g. when a caller opens its own RViz view)."),

        # slam_toolbox backend (mapping / continue / localization) -- unless AMCL is selected.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup, "launch", "slam.launch.py")),
            condition=UnlessCondition(use_amcl),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "slam_mode": slam_mode,
                "map": map_name,
                "x_pose": x_pose,
                "y_pose": y_pose,
            }.items(),
        ),

        # AMCL backend (map_server + amcl against a static occupancy map).
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup, "launch", "localization_amcl.launch.py")),
            condition=IfCondition(use_amcl),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "map_yaml": map_yaml,
                "x_pose": x_pose,
                "y_pose": y_pose,
            }.items(),
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
            condition=IfCondition(use_rviz),
            output="log",
        ),
    ])
