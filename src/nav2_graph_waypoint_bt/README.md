# Nav2 Graph Waypoint BT (Jazzy / Nav2 1.3.13)

**Experimental code; robot hardware validation required.** Two BT nodes: `PruneGraphGoals` (A), `TurnSegmentControl` (B).
Original FMS NavigateThroughPoses Action, ThetaStar, RPP, Costmap, collision monitor, 1Hz replanning retained.

## Preflight (on TurtleBot3 via SSH)

Before touching the launch config, inspect:

```bash
source /opt/ros/jazzy/setup.bash
ros2 pkg xml nav2_controller | grep -m1 '<version>'  # expect 1.3.13
ros2 pkg prefix behaviortree_cpp
ros2 pkg prefix nav2_util
```

## Build on the robot

Copy the ZIP to robot's home, then:

```bash
cd ~/logitle_ws/src
unzip -o ~/nav2_graph_waypoint_bt.zip
cd ~/logitle_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select nav2_graph_waypoint_bt --symlink-install --event-handlers console_direct+
source ~/logitle_ws/install/setup.bash
ros2 pkg prefix nav2_graph_waypoint_bt
ros2 run nav2_graph_waypoint_bt plugin_smoke_test "$(ros2 pkg prefix nav2_graph_waypoint_bt)/lib/libnav2_graph_waypoint_bt.so"
python3 ~/logitle_ws/src/nav2_graph_waypoint_bt/scripts/check_static.py
```

No other workspace packages are modified. If `nav2_graph_waypoint_bt` is missing from `ros2 pkg prefix`, verify the source commands and build logs.

## Backup before application

**Use the actual config YAML and BT filename used by your launch, not a guessed file.**

```bash
cp /ACTUAL/PATH/burger.yaml /ACTUAL/PATH/burger.yaml.bak
cp /ACTUAL/PATH/fms_navigate_through_poses.xml /ACTUAL/PATH/fms_navigate_through_poses.xml.bak
```

Add under `bt_navigator.ros__parameters` in the actual YAML:

```yaml
plugin_lib_names: ["nav2_graph_waypoint_bt"]
default_nav_through_poses_bt_xml: "$(find-pkg-share nav2_graph_waypoint_bt)/behavior_trees/graph_pass_only.xml"
```

A-only is safer to start. After A-only straight / corner / cancel / obstacle tests are complete, switch to:

```yaml
default_nav_through_poses_bt_xml: "$(find-pkg-share nav2_graph_waypoint_bt)/behavior_trees/graph_turn_aware.xml"
```

For B mode: ensure `planner_server.ros__parameters.ThetaStar.use_final_approach_orientation: false` (provided target yaw is preserved). Existing RPP `use_rotate_to_heading: true`, final goal checker XY/Yaw tolerances should stay configured.

IMPORTANT: Launch must also **source ~/logitle_ws/install/setup.bash** so `bt_navigator` finds the shared library. Stop any active navigation tasks before restarting Nav2 for new YAML or BT.

## Monitoring

```bash
ros2 node list | grep bt_navigator
ros2 topic hz /plan
ros2 topic echo /is_rotating_to_heading
```

Look at BT Navigator console logs for `PruneGraphGoals: passed` and `TurnSegmentControl: start segment`.
On multi-robot setups, topics may be namespaced or remapped; verify using `ros2 topic list`.

## Limitations / safety

- No physical robot or ROS 2 Jazzy runtime is available in the author's environment. ROS C++ compilation, BT loading, recovery, and physical tests **must be verified on target**.
- A tracks approach along straight segments from robot/previous waypoint. It does not guarantee geometrically exact edge following; ThetaStar may deviate due to dynamic obstacle costs.
- B recognizes turns only among supplied PoseStamped goals. It does **not** know GeoJSON edges not sent as goals. The first goal can be classified as a turn using current robot pose toward that goal when TF is available, but this is only a geometric approximation of the incoming graph edge.
- B splits the **BT internal planner/controller segments**, not FMS Action requests. At turns, the final segment target orientation is set toward the next edge and RPP should rotate when near the segment endpoint. The nearest approach accuracy is bounded by AMCL, controller tolerances, obstacles and geometry.
- B returns success when the child `FollowPath` returns success; it does not separately verify stopped odometry speed.
- A's passage tolerance uses estimated robot pose, so do not treat `0.04m` as a guaranteed physical-accuracy specification.
- In a very short turn segment or if the turn node is in occupied Keepout/inflation cells, planning or final alignment can fail. Never suppress collision monitor/Keepout to bypass this.
- Keep original BT and YAML backups, test with wheels raised first if possible, then open floor at 0.05-0.10m/s with manual e-stop ready.
- Note: B updates the top-level remaining goals upon **segment** completion. Within a segment, feedback `number_of_poses_remaining` may over-report until the segment finishes; Action end success/failure is unchanged.

## Rollback

Revert the two YAML settings (`plugin_lib_names` and `default_nav_through_poses_bt_xml`) to their original values (or restore original YAML backup), then restart Nav2. Files in this package may remain installed; no existing Nav2 library was modified.
