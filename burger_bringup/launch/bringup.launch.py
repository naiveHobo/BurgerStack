"""One-command bringup: Gazebo sim + SLAM + Nav2 (RRT planner) + RViz, with optional
autonomous frontier exploration.

Gazebo starts first; SLAM/Nav2/RViz come up only once the robot is actually
spawned and publishing, detected by waiting for the first message on a readiness
topic (default /scan) rather than guessing with a fixed delay. A /scan message
implies Gazebo is up, the robot spawned, and its sensor + diff-drive plugins are
live (so the odom->base_footprint TF is flowing too). This adapts to machine
speed and heavy worlds (e.g. the office) automatically, and avoids the early
"waiting for transform" churn and lifecycle activation races a too-short delay
would cause -- without the wasted wait of a too-long one.

nav2.launch.py already pulls in slam.launch.py and RViz, so this file just
composes sim + nav2 and sequences their startup on that readiness signal.

When `explore:=true`, the frontier_exploration_ros2 explorer is also started, gated on
the first message on /global_costmap/costmap (published only after the Nav2 lifecycle
activates and the navigate_to_pose action server is up). `slam_mode` selects the map
behavior: mapping (fresh), continue (resume + keep mapping a saved map), or localization
(localize over a saved map, no mapping -- intended without `explore`).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup = get_package_share_directory("burger_bringup")
    launch_dir = os.path.join(bringup, "launch")
    default_map = os.path.join(bringup, "maps", "map")
    default_explorer_params = os.path.join(bringup, "params", "exploration.yaml")

    world = LaunchConfiguration("world")
    use_sim_time = LaunchConfiguration("use_sim_time")
    slam_mode = LaunchConfiguration("slam_mode")
    map_name = LaunchConfiguration("map")
    x_pose = LaunchConfiguration("x_pose")
    y_pose = LaunchConfiguration("y_pose")
    localizer = LaunchConfiguration("localizer")
    map_yaml = LaunchConfiguration("map_yaml")
    use_rviz = LaunchConfiguration("use_rviz")
    ready_topic = LaunchConfiguration("ready_topic")
    ready_timeout = LaunchConfiguration("ready_timeout")
    explore = LaunchConfiguration("explore")
    explorer_params = LaunchConfiguration("explorer_params")
    explorer_ready_action = LaunchConfiguration("explorer_ready_action")
    explorer_ready_timeout = LaunchConfiguration("explorer_ready_timeout")

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "sim.launch.py")),
        launch_arguments={
            "world": world,
            "x_pose": x_pose,
            "y_pose": y_pose,
        }.items(),
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "nav2.launch.py")),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "slam_mode": slam_mode,
            "map": map_name,
            "x_pose": x_pose,
            "y_pose": y_pose,
            "localizer": localizer,
            "map_yaml": map_yaml,
            "use_rviz": use_rviz,
        }.items(),
    )

    # Block until the robot is up and publishing (first message on ready_topic).
    # `--once` prints one message and exits; `timeout` bounds the wait so a failed
    # spawn can't deadlock startup -- on timeout we fall through and start Nav2
    # anyway (its lifecycle/costmaps will then wait/retry on their own).
    wait_for_robot = ExecuteProcess(
        name="wait_for_robot",
        cmd=["bash", "-c", ["timeout ", ready_timeout,
                            " ros2 topic echo --once ", ready_topic,
                            " > /dev/null 2>&1; true"]],
        output="screen",
    )

    start_nav2_when_ready = RegisterEventHandler(
        OnProcessExit(target_action=wait_for_robot, on_exit=[nav2])
    )

    # Optional explorer (explore:=true). Start it only once Nav2 is publishing its global
    # costmap (i.e. lifecycle activated + navigate_to_pose available). Same wait idiom as
    # above; on timeout we start anyway (the explorer has its own QoS waits and a
    # suppression startup grace period).
    explorer = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("frontier_exploration_ros2"),
            "launch", "frontier_explorer.launch.py")),
        launch_arguments={
            "params_file": explorer_params,
            "use_sim_time": use_sim_time,
        }.items(),
    )

    # Poll until Nav2 is fully up -- the navigate_to_pose action server only appears once
    # the lifecycle manager has configured AND activated the servers (controller, planner,
    # costmaps, bt_navigator). We MUST actually wait here: starting the explorer node
    # concurrently with Nav2's lifecycle activation (under the heavy office world) crashes
    # lifecycle_manager_navigation. `ros2 topic echo --once` is NOT usable as a gate -- it
    # returns immediately ("Could not determine the type") when the topic isn't advertised
    # yet, so it never blocks. We poll `ros2 action list` instead, bounded by a timeout so a
    # failed bring-up can't deadlock startup (on timeout we start the explorer anyway; it has
    # its own map/costmap QoS waits and a suppression startup grace period).
    wait_for_nav2 = ExecuteProcess(
        name="wait_for_nav2",
        condition=IfCondition(explore),
        cmd=["bash", "-c", ["t=0; until ros2 action list 2>/dev/null | grep -q ",
                            explorer_ready_action,
                            "; do sleep 2; t=$((t+2)); [ \"$t\" -ge ", explorer_ready_timeout,
                            " ] && break; done; true"]],
        output="screen",
    )

    start_explorer_when_ready = RegisterEventHandler(
        OnProcessExit(target_action=wait_for_nav2, on_exit=[explorer]),
        condition=IfCondition(explore),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "world", default_value="small_house",
            description="Gazebo world: 'small_house' (vendored AWS residential house, default), "
                        "'office' (AWS/OSRF ServiceSim), 'house' or 'world'."),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument(
            "slam_mode", default_value="mapping",
            description="slam_toolbox mode: mapping (fresh) | continue (resume saved map). "
                        "Ignored when localizer:=amcl (localization-only)."),
        DeclareLaunchArgument(
            "map", default_value=default_map,
            description="Serialized pose-graph base name (no extension) for the "
                        "continue/localization slam_mode."),
        DeclareLaunchArgument(
            "x_pose", default_value="-3.5",
            description="Robot spawn X (small_house default; office tasks override to -6.0). "
                        "Used for spawning AND as the slam map_start_pose for "
                        "continue/localization -- keep them equal."),
        DeclareLaunchArgument(
            "y_pose", default_value="-4.5",
            description="Robot spawn Y (small_house default; office tasks override to 8.0). "
                        "Used for spawning AND as the slam map_start_pose for "
                        "continue/localization -- keep them equal."),
        DeclareLaunchArgument(
            "localizer", default_value="slam_toolbox",
            description="Backend: slam_toolbox (mapping/continue) | amcl (localization-only "
                        "against a static occupancy map like map.yaml or map_gt.yaml)."),
        DeclareLaunchArgument(
            "map_yaml", default_value=os.path.join(
                get_package_share_directory("burger_bringup"), "maps", "map_gt.yaml"),
            description="Occupancy-grid map yaml for localizer:=amcl (default ground-truth map_gt.yaml)."),
        DeclareLaunchArgument(
            "use_rviz", default_value="true",
            description="Start the bundled planner RViz (from nav2.launch.py). Set false to run "
                        "headless or when a caller opens its own RViz view."),
        DeclareLaunchArgument(
            "ready_topic", default_value="/scan",
            description="Topic whose first message gates SLAM/Nav2 startup."),
        DeclareLaunchArgument(
            "ready_timeout", default_value="120",
            description="Max seconds to wait for the readiness signal before starting anyway."),
        DeclareLaunchArgument(
            "explore", default_value="false",
            description="Start the frontier_exploration_ros2 explorer alongside the stack."),
        DeclareLaunchArgument(
            "explorer_params", default_value=default_explorer_params,
            description="Frontier explorer parameter file (used when explore:=true)."),
        DeclareLaunchArgument(
            "explorer_ready_action", default_value="/navigate_to_pose",
            description="Action server whose availability gates explorer startup "
                        "(i.e. Nav2 fully activated)."),
        DeclareLaunchArgument(
            "explorer_ready_timeout", default_value="240",
            description="Max seconds to wait for Nav2 before starting the explorer anyway."),

        sim,
        wait_for_robot,
        start_nav2_when_ready,
        wait_for_nav2,
        start_explorer_when_ready,
    ])
