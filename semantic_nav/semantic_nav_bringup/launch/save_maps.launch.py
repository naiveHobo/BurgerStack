"""Manual handoff: save the occupancy map + persist the enriched semantic map.

Fires the map_finalizer's ~/finalize trigger, which calls the map_saver SaveMap
service and mapping_node's BuildSemanticMap service. Run this against a live Phase-1
stack (semantic_mapping.launch.py) when exploration is far enough along:

    ros2 launch semantic_nav_bringup save_maps.launch.py

Writes ~/.semantic_nav/{occupancy_map.yaml,.pgm} and ~/.semantic_nav/semantic_map.json
(paths configurable on the map_finalizer node). This is the manual equivalent of
launching Phase 1 with auto_finalize:=true.
"""
from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            name="finalize_maps",
            cmd=["ros2", "service", "call", "/map_finalizer/finalize",
                 "std_srvs/srv/Trigger", "{}"],
            output="screen",
        ),
    ])
