# Map-based Precision Docking + Recovery

Target branch: `jhleedev00/merge` (never edit `dev`).

The robot-specific `initial_points` in `logitle_robot.launch.py` are the **fully docked robot Map poses**. The dock server uses these via launch parameters to generate the staging point 0.35m in front; the ICP V-model origin is assumed 0.10m behind the registered final base pose. **Both distances require physical measurement.**

Measured V-dock geometry: charger_width=0.145m, wing_length=0.10m, wing_angle_deg=22.5.

The `PrecisionDock.target_pose` field is an Action trigger, *not* the dock pose. Use:

```bash
ros2 launch logitle_bringup logitle_robot.launch.py use_nav2:=true docking_dry_run:=true
ros2 action send_goal /precision_dock turtlebot3_my_msg/action/PrecisionDock "{target_pose: {header: {frame_id: 'map'}, pose: {orientation: {w: 1.0}}}}" --feedback
```

Default dry-run forbids movement and deliberately aborts the Action after computing/logging the Map staging and dock poses. It does not validate ICP. For hardware testing, first check AMCL/Map alignment, LiDAR scan TF, physical clearances, and `/cmd_vel` exclusivity. Only then use `docking_dry_run:=false`.

States: INIT → STAGING_TURN → STAGING_DRIVE → STAGING_ALIGN → ICP_SETTLE → ALIGN_HEADING → STRAIGHT_REVERSE → FINAL_REVERSE → COMPLETED. Failures detected during reverse trigger guarded RECOVERY_STOP → RECOVERY_EXIT → STAGING_ALIGN (max 2 retries). It never turns in place inside the dock and aborts if exit path is unknown or blocked. ICP uses a Map spatial gate and requires the dock center and both wings, and three stable detections.

Optional `contact_topic` accepts `std_msgs/Bool`, True means contact. By default it is unset. With LiDAR and odometry alone, *actual collision-free docking cannot be proven*. ROS2/real robot behavior is not validated by unit tests.
