"""Bring up Gazebo with a TurtleBot3 Burger in a selectable world.

Run this in its own terminal, then `pixi run explore` (or `nav2-rrt`) alongside.

`world:=house` (default) or any upstream TurtleBot3 world (`world`, `dqn_stage1`, ...)
delegates to turtlebot3_gazebo's per-world launch, which bundles Gazebo +
robot_state_publisher + robot spawn.

`world:=office` loads the vendored AWS RoboMaker / OSRF ServiceSim office from the
`burger_worlds` package. Because that world is not an upstream TurtleBot3 world, we
assemble the pieces ourselves (gzserver with service.world + gzclient +
robot_state_publisher + spawn) and spawn a camera-equipped burger
(`TURTLEBOT3_MODEL=burger_cam`) so the semantic-reasoning layer has an RGB feed
(`/camera/image_raw`) in addition to the LIDAR. The office's `model://...` and
`file://media/...` assets are made resolvable by extending GAZEBO_MODEL_PATH /
GAZEBO_RESOURCE_PATH in-process before Gazebo (a child process) starts.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


# Vendored worlds (not part of turtlebot3_gazebo): name -> world file + asset dirs,
# all relative to the owning package's share directory. Any world NOT listed here
# falls back to the upstream turtlebot3_{world}.launch.py.
CUSTOM_WORLDS = {
    "office": {
        "package": "burger_worlds",
        "world": "worlds/office/service.world",
        "models": "worlds/office/models",   # resolves model://<name>
        "resources": "worlds/office",       # resolves file://media/... (parent of media/)
        "robot_model": "burger_cam",        # camera-equipped burger
    },
}


def _prepend_env(var, path):
    current = os.environ.get(var, "")
    os.environ[var] = path + (os.pathsep + current if current else "")


def _custom_world_setup(spec):
    pkg = get_package_share_directory(spec["package"])

    # gzserver is a child of this launch and inherits os.environ at spawn time, so
    # extending these here (before the includes below are processed) is enough.
    _prepend_env("GAZEBO_MODEL_PATH", os.path.join(pkg, spec["models"]))
    _prepend_env("GAZEBO_RESOURCE_PATH", os.path.join(pkg, spec["resources"]))
    # spawn_turtlebot3 / robot_state_publisher read TURTLEBOT3_MODEL from the env when
    # their (lazily-evaluated) launch descriptions are generated, which happens after
    # this function returns -- so setting it here selects the camera variant.
    os.environ["TURTLEBOT3_MODEL"] = spec["robot_model"]

    world_path = os.path.join(pkg, spec["world"])
    pkg_gazebo_ros = get_package_share_directory("gazebo_ros")
    tb3_launch = os.path.join(get_package_share_directory("turtlebot3_gazebo"), "launch")
    use_sim_time = LaunchConfiguration("use_sim_time")
    x_pose = LaunchConfiguration("x_pose")
    y_pose = LaunchConfiguration("y_pose")

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, "launch", "gzserver.launch.py")),
            launch_arguments={"world": world_path}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, "launch", "gzclient.launch.py")),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(tb3_launch, "robot_state_publisher.launch.py")),
            launch_arguments={"use_sim_time": use_sim_time}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(tb3_launch, "spawn_turtlebot3.launch.py")),
            launch_arguments={"x_pose": x_pose, "y_pose": y_pose}.items(),
        ),
    ]


def launch_setup(context, *args, **kwargs):
    world = LaunchConfiguration("world").perform(context)
    spec = CUSTOM_WORLDS.get(world)
    if spec is not None:
        return _custom_world_setup(spec)
    # Upstream TurtleBot3 world (house / world / dqn_stage* / ...).
    tb3_gazebo = get_package_share_directory("turtlebot3_gazebo")
    launch_file = os.path.join(tb3_gazebo, "launch", f"turtlebot3_{world}.launch.py")
    return [IncludeLaunchDescription(PythonLaunchDescriptionSource(launch_file))]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "world", default_value="house",
            description="World: 'office' (vendored AWS/OSRF ServiceSim office, camera "
                        "burger), 'house' (office-like) or 'world' (arena)."),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument(
            "x_pose", default_value="-6.0",
            description="Robot spawn X for custom worlds (e.g. 'office')."),
        DeclareLaunchArgument(
            "y_pose", default_value="8.0",
            description="Robot spawn Y for custom worlds (e.g. 'office')."),
        OpaqueFunction(function=launch_setup),
    ])
