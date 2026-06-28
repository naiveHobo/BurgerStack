# burger_worlds

Vendored Gazebo **Classic** simulation worlds for the TurtleBot3 Burger stack.

Both worlds load through `burger_bringup` (they are not upstream TurtleBot3 worlds, so
`sim.launch.py` assembles gzserver/gzclient + robot_state_publisher + spawn itself) and
spawn a camera-equipped RGB-D burger (`TURTLEBOT3_MODEL=burger_depth`), exposing
`/camera/image_raw`, `/camera/camera_info` and depth (`/camera/depth/...`) for the
semantic-reasoning layer in addition to the 2D LIDAR (`/scan`).

## Worlds

### `small_house` — `worlds/small_house/small_house.world`  (default)
The AWS RoboMaker **residential Small House**: a furnished single-storey home — living
room, bedroom, kitchen and balcony — with sofas, a bed, a TV, refrigerator, kitchen units,
chairs, tables, lamps/chandeliers and decor (~64 referenced `aws_robomaker_residential_*`
models; 68 model dirs including nested includes). A reference occupancy map ships in
`worlds/small_house/map/`.

```bash
pixi run sim-small-house                          # Gazebo + RGB-D burger only
pixi run explore                                  # + SLAM + Nav2 + RViz + frontier explorer
ros2 launch burger_bringup bringup.launch.py      # full stack (small_house is the default)
```

`small_house` is the **default** world for `burger_bringup`'s `sim.launch.py` /
`bringup.launch.py` and for the generic `pixi` full-stack tasks (`bringup`, `explore`,
`localize`, `ai-mapping`, `ai-navigation`, ...). The burger spawns at `(-3.5, -4.5)`.

### `office` — `worlds/office/service.world`
The AWS RoboMaker / **OSRF ServiceSim** office: a multi-room office with corridors,
cubicles, meeting rooms, a cafe/refreshment area and bathrooms, furnished with desks,
chairs, couches, appliances and computers, plus human `actor` models. A reference
occupancy map ships in `worlds/office/map/`.

```bash
pixi run sim-office                               # Gazebo + RGB-D burger only
ros2 launch burger_bringup bringup.launch.py world:=office x_pose:=-6.0 y_pose:=8.0   # full stack
```

The office is no longer the default, so select it with `world:=office` **and** its spawn
pose (`x_pose:=-6.0 y_pose:=8.0`) — `pixi run sim-office` already does both.

## Provenance & license

Both worlds and their models originate from AWS RoboMaker and were assembled from
`github.com/yojuna/robot_worlds`:

- **office** — AWS RoboMaker / OSRF ServiceSim (`github.com/osrf/servicesim`), Apache-2.0,
  from `robot_worlds/worlds/office/`.
- **small_house** — the AWS RoboMaker Small House world sample
  (`github.com/aws-robotics/aws-robomaker-small-house-world`, MIT), from
  `robot_worlds/worlds/small_house/`. See that repository for authoritative license terms.

### office repairs

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

### small_house repairs & notes

1. **Removed an embedded `turtlebot3_waffle_pi`** the upstream world spawned at
   `(-3.5, -4.5)`; it would collide with the burger we spawn and publish duplicate
   `/scan` `/odom` `/cmd_vel`. Its (author-validated, on-floor) pose is reused as the
   burger spawn point.

Unlike the office, the small_house needed **no model backfill** — all 64 referenced
`aws_robomaker_residential_*` models are present, and the world is fully `model://`-based.
Its model SDFs reference meshes with relative `file://models/...` URIs, resolved via
`GAZEBO_RESOURCE_PATH` (set to `worlds/small_house`), so there is **no `media/` tree**.

Excluded as cruft: the redundant `small_house.zip`, the `photos/` directory, the
`small_house.jpg` screenshot and macOS `.DS_Store` files.

**Known cosmetic limitation:** 5 of the 68 models (`Sofa_01`, `Refrigerator_01`,
`AirconditionerA_01`, `Chandelier_01`, `CookingBench_01`) have a texture path baked into
their COLLADA `.dae` by the original authors that points to a non-existent Windows path
(`G:\...` / `C:\...`). Gazebo logs an "unable to find texture" warning and renders those
few meshes with a default material; geometry, collisions and all other models are
unaffected. The local PNG textures ship alongside, so this is fixable later by editing
those `.dae` `<init_from>` references — the meshes are left as-is to stay faithful to upstream.
