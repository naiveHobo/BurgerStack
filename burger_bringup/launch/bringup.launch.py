"""One-command bringup: Gazebo sim + SLAM + Nav2 (RRT planner) + RViz.

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
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    launch_dir = os.path.join(get_package_share_directory("burger_bringup"), "launch")
    world = LaunchConfiguration("world")
    use_sim_time = LaunchConfiguration("use_sim_time")
    ready_topic = LaunchConfiguration("ready_topic")
    ready_timeout = LaunchConfiguration("ready_timeout")

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "sim.launch.py")),
        launch_arguments={"world": world}.items(),
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "nav2.launch.py")),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
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

    return LaunchDescription([
        DeclareLaunchArgument(
            "world", default_value="world",
            description="Gazebo world: 'office' (AWS/OSRF ServiceSim), 'house' or 'world'."),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument(
            "ready_topic", default_value="/scan",
            description="Topic whose first message gates SLAM/Nav2 startup."),
        DeclareLaunchArgument(
            "ready_timeout", default_value="120",
            description="Max seconds to wait for the readiness signal before starting anyway."),

        sim,
        wait_for_robot,
        start_nav2_when_ready,
    ])
