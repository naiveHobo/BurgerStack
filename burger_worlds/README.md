# burger_worlds

Vendored Gazebo **Classic** simulation worlds for the TurtleBot3 Burger stack.

## Worlds

### `office` — `worlds/office/service.world`
The AWS RoboMaker / **OSRF ServiceSim** office: a multi-room office with corridors,
cubicles, meeting rooms, a cafe/refreshment area and bathrooms, furnished with desks,
chairs, couches, appliances and computers, plus human `actor` models. A reference
occupancy map ships in `worlds/office/map/`.

Run it through `burger_bringup`:

```bash
ros2 launch burger_bringup sim.launch.py world:=office          # Gazebo + burger (with camera)
ros2 launch burger_bringup bringup.launch.py world:=office      # + SLAM + Nav2 + RViz
# or: pixi run office
```

The office launch spawns `TURTLEBOT3_MODEL=burger_cam` so an RGB camera
(`/camera/image_raw`, `/camera/camera_info`) is available for semantic-reasoning work,
in addition to the 2D LIDAR (`/scan`).

## Provenance & license

The office world and models originate from the AWS RoboMaker ServiceSim
(`github.com/osrf/servicesim`), **Apache-2.0**. The assets were assembled from
`github.com/yojuna/robot_worlds` (`worlds/office/`).

Two repairs were applied to the `robot_worlds` copy when vendoring:

1. **Backfilled 4 missing model directories** — `cubicle_wall`, `door`,
   `cubicle_corner`, `cubicle_island` — from the upstream `osrf/servicesim`
   (`servicesim_competition/models/`). The `robot_worlds` copy referenced these
   (~230 placements: cubicle partitions and doors) but omitted their meshes, so the
   world would otherwise load with the entire cubicle layout and all doors missing
   and ~134 "Unable to find model/mesh" errors. With the backfill, every `model://`
   reference resolves cleanly.
2. **Removed an embedded `turtlebot3_waffle_pi`** that the upstream world spawned at
   `(0, 20)`. It would have collided with the burger we spawn and published duplicate
   `/scan` `/odom` `/cmd_vel`, breaking SLAM/Nav2.

The redundant `office_part1.zip` / `office_part2.zip` archives from the source (their
contents were already present as extracted files) are **not** vendored.
