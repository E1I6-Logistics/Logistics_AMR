# Logitle precision docking

`precision_dock` treats `target_pose` as the dock pose in the `map` frame.
The pose yaw points from the dock toward its approach waypoint. The server:

1. computes a map-frame waypoint `staging_distance` in front of the dock;
2. drives to that waypoint and aligns with the dock axis using fresh map TF;
3. validates the measured 145 mm charger face and 100 mm, 22.5 degree wings
   with gated ICP;
4. reverses under ICP control, then uses a short odometry-only final approach;
5. stops and performs a bounded forward retreat/reacquisition if ICP is lost.

The server fails closed on stale map TF, stale/missing scans, scan-frame TF
failure, invalid action poses, ICP quality/jump rejection, blocked recovery, or
timeout. Recovery is limited to two attempts by default.

Set `docking_dry_run:=true` in `logitle_robot.launch.py` to validate the goal,
TF, scan transform, charger geometry, and ICP gates while forcing every
velocity command to zero. Run dry-run while the charger is visible from the
robot's current pose.

Confirmed charger geometry defaults and launch overrides are:

- `charger_width: 0.145`
- `wing_length: 0.10`
- `wing_angle_deg: 22.5`
