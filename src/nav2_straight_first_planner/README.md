# StraightFirst Planner (Nav2 Jazzy prototype)

**Experimental. Robot-tested: NO.** This package is not yet compiled on a ROS2 Jazzy host. Use one robot in a clear test area, E-stop within reach.

- Global planner plugin. For each start/goal pair, emits a straight path with samples spaced 5cm, if a conservative radius check against Global Costmap passes.
- If blocked (lethal/inscribed/unknown/outside map), uses stock **SmacPlanner2D (A*)** by loading the existing plugin and delegating to it.
- Keeps RPP controller, collision monitor, BT segment control and 1Hz replanning unchanged.
- Existing `ComputePathThroughPoses` calls a global planner for each adjacent pose pair; B uses subsets. The first path point on an incoming edge is the *live robot pose*, not the prior graph node. This means perfect graph-edge alignment **is not guaranteed**, particularly after an avoidance detour.
- **No hystersis** on falling back to A*: if costmap changes each second, direct/A* plans can flip. Assess on robot first and add fallback hysteresis if needed.
- Collision checking uses a **circular raster approximation**, radius 0.12m, regardless of robot's configured polygon footprint; adequate only after matching against your Burger/costmap. Unlike a full footprint checker, diagonal discrete cell coverage / between-cell clearance is approximate. Never treat it as safety-certified. Collision Monitor and RPP collision detection remain essential.
- Fallback instantiates SmacPlanner2D internally, under parameter name `Smac2D`. **Do not simultaneously configure `Smac2D` as a separate planner plugin in `planner_plugins`**: that would configure two plugins under the same ROS parameter namespace; remove `Smac2D` from the list while preserving its YAML parameter block.

## Build

Copy the `nav2_straight_first_planner` directory to `~/logitle_ws/src/` on the robot (keep your existing `nav2_graph_waypoint_bt`).

```bash
cd ~/logitle_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select nav2_straight_first_planner --symlink-install
source install/setup.bash
ros2 plugin list | grep -A3 nav2_straight_first_planner
```

## Configure (backup burger.yaml first)

In `planner_server.ros__parameters`:

```yaml
planner_plugins: ["Navfn", "ThetaStar", "StraightFirst"]
# Keep existing Smac2D: block even though it is not listed above.
StraightFirst:
  plugin: "nav2_straight_first_planner::StraightFirstPlanner"
  interpolation_resolution: 0.05
  safety_radius: 0.12
  max_cell_cost: 252
  allow_unknown: false
```

In your `graph_turn_aware.xml`, update **only** PlannerSelector's default:

```xml
<PlannerSelector selected_planner="{selected_planner}"
                 default_planner="StraightFirst" topic_name="planner_selector"/>
```

Keep `<ComputePathThroughPoses planner_id="{selected_planner}" ... />` unchanged. If `/planner_selector` publishes `ThetaStar`, it can override the default; ensure selector chooses StraightFirst for the test.

Restart Nav2 so the XML is reread (`always_reload_bt_xml: false`). There is no need to alter FMS, AMCL, RPP, BT plugin C++, or recovery behavior.

## Verify

```bash
ros2 lifecycle get /planner_server
ros2 param get /planner_server planner_plugins
ros2 param get /planner_server StraightFirst.interpolation_resolution
ros2 param get /bt_navigator default_nav_through_poses_bt_xml
ros2 topic echo /plan --once
```

Check visually that /plan is aligned to waypoint line and that obstacle forces A* fallback. Observe /plan changes and /cmd_vel at each transition. Do not introduce obstacles close to an actively moving robot. If errors, revert planner_plugins list and BT selector default to ThetaStar, restart Nav2.

## Important limits

- No route graph-edge ID or GeoJSON access: acts on received pose pairs only.
- With B plugin you still get segment stopping for bends.
- Allows A* bypass of non-FMS corridors if traversable; for strict one-way routes use traffic-manager routing instead.
- `max_cell_cost=252` means `253/254` blocked and unknown 255 blocked by `allow_unknown:false`. Some inflation costs (<253) considered traversable. Raise safety radius if required by measured clearance.
- The binary build and hardware integration must be tested in your ROS2 Jazzy workspace; package XML and static source syntax are the only checks done here.
