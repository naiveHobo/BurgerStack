"""Bring up Gazebo with a TurtleBot3 Burger in a selectable world.

Run this in its own terminal, then `pixi run explore` (or `nav2-rrt`) alongside.

`world:=small_house` (default) and `world:=office` load vendored worlds from the
`burger_worlds` package (the AWS RoboMaker residential Small House and the AWS RoboMaker /
OSRF ServiceSim office). Because these are not upstream TurtleBot3 worlds, we assemble the
pieces ourselves (gzserver with the .world + gzclient + robot_state_publisher + spawn) and
spawn a camera-equipped RGB-D burger (`TURTLEBOT3_MODEL=burger_depth`) so the semantic
layer has an RGB + depth feed (`/camera/image_raw`, `/camera/depth/...`) in addition to the
LIDAR. Each world's `model://...` and `file://...` assets are made resolvable by extending
GAZEBO_MODEL_PATH / GAZEBO_RESOURCE_PATH in-process before Gazebo (a child process) starts.

Any other `world:=<name>` (`house`, `world`, `dqn_stage1`, ...) delegates to
turtlebot3_gazebo's per-world launch, which bundles Gazebo + robot_state_publisher + robot
spawn (and ignores x_pose/y_pose -- it uses TurtleBot3's own hard-wired spawn point).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# Vendored worlds (not part of turtlebot3_gazebo): name -> world file + asset dirs,
# all relative to the owning package's share directory. Any world NOT listed here
# falls back to the upstream turtlebot3_{world}.launch.py.
#
# Each world has a natural, collision-free spawn pose (documented below). The launch's
# x_pose/y_pose defaults track the DEFAULT world's (small_house) spawn; tasks that select
# a different custom world (office) pass x_pose/y_pose explicitly. See the x_pose/y_pose
# DeclareLaunchArgument defaults below.
#   small_house spawn: (-3.5, -4.5)  -- reused from the world's removed waffle_pi
#   office spawn:      (-6.0,  8.0)  -- reception corridor
CUSTOM_WORLDS = {
    "small_house": {
        "package": "burger_worlds",
        "world": "worlds/small_house/small_house.world",
        "models": "worlds/small_house/models",   # resolves model://aws_robomaker_residential_*
        "resources": "worlds/small_house",        # resolves file://models/... (parent of models/)
        "robot_model": "burger_depth",            # vendored RGB-D burger (burger_bringup)
    },
    "office": {
        "package": "burger_worlds",
        "world": "worlds/office/service.world",
        "models": "worlds/office/models",   # resolves model://<name>
        "resources": "worlds/office",       # resolves file://media/... (parent of media/)
        "robot_model": "burger_depth",      # vendored RGB-D burger (burger_bringup)
    },
}


def _prepend_env(var, path):
    current = os.environ.get(var, "")
    os.environ[var] = path + (os.pathsep + current if current else "")


def _custom_world_setup(spec):
    pkg = get_package_share_directory(spec["package"])
    burger_bringup = get_package_share_directory("burger_bringup")

    # gzserver is a child of this launch and inherits os.environ at spawn time, so
    # extending these here (before the includes below are processed) is enough.
    _prepend_env("GAZEBO_MODEL_PATH", os.path.join(pkg, spec["models"]))
    _prepend_env("GAZEBO_RESOURCE_PATH", os.path.join(pkg, spec["resources"]))
    # Make the vendored RGB-D robot resolvable as model://turtlebot3_burger_depth
    # (its base/wheel/lidar meshes still resolve from turtlebot3_common in the conda
    # models dir, already on GAZEBO_MODEL_PATH via scripts/pixi_activate.sh).
    _prepend_env("GAZEBO_MODEL_PATH", os.path.join(burger_bringup, "models"))
    os.environ["TURTLEBOT3_MODEL"] = spec["robot_model"]

    world_path = os.path.join(pkg, spec["world"])
    pkg_gazebo_ros = get_package_share_directory("gazebo_ros")
    use_sim_time = LaunchConfiguration("use_sim_time")
    x_pose = LaunchConfiguration("x_pose")
    y_pose = LaunchConfiguration("y_pose")

    # We assemble robot_state_publisher + spawn ourselves rather than reusing
    # turtlebot3_gazebo's launch files: those are hard-wired to load the URDF/SDF from
    # the conda turtlebot3_gazebo share dir by TURTLEBOT3_MODEL name, and our RGB-D
    # variant lives in burger_bringup. Loading the matching URDF here is what puts
    # camera_link -> camera_rgb_frame -> camera_rgb_optical_frame into the TF tree,
    # which the perception layer needs to deproject depth pixels into the map frame.
    model_name = spec["robot_model"]
    sdf_path = os.path.join(burger_bringup, "models", f"turtlebot3_{model_name}", "model.sdf")
    urdf_path = os.path.join(burger_bringup, "urdf", f"turtlebot3_{model_name}.urdf")
    with open(urdf_path, "r") as f:
        robot_desc = f.read()

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
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time, "robot_description": robot_desc}],
        ),
        Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            arguments=[
                "-entity", model_name,
                "-file", sdf_path,
                "-x", x_pose, "-y", y_pose, "-z", "0.01",
            ],
            output="screen",
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
            "world", default_value="small_house",
            description="World: 'small_house' (vendored AWS residential house, default) or "
                        "'office' (vendored AWS/OSRF ServiceSim) -- both camera/RGB-D burger; "
                        "or an upstream TurtleBot3 world ('house', 'world', ...)."),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument(
            "x_pose", default_value="-3.5",
            description="Robot spawn X for custom worlds. Default is small_house's spawn; "
                        "office tasks override to -6.0."),
        DeclareLaunchArgument(
            "y_pose", default_value="-4.5",
            description="Robot spawn Y for custom worlds. Default is small_house's spawn; "
                        "office tasks override to 8.0."),
        OpaqueFunction(function=launch_setup),
    ])
