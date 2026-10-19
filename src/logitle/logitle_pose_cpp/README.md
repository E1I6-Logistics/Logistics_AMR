# Logitle TF pose publisher: Python vs C++

This package is an isolated C++17 / `rclcpp` port of the existing Python
`logitle_pose` publisher. The Python implementation and production launch files
are unchanged.

## Verified source contract (dev baseline, 2026-10-19)

- Python source: `src/logitle/logitle_pose/logitle_pose/logitle_pose_publisher.py`
- Production launch: `src/logitle/logitle_bringup/launch/logitle_robot.launch.py`
- TF: `map <- base_footprint` (latest available)
- Output: relative topic `logitle_pose`, `geometry_msgs/msg/PoseStamped`
- Frequency: 5 Hz from launch parameters (its 20 Hz comment is outdated)
- Publisher QoS: KeepLast(10), reliable, volatile
- Header timestamp: original transform stamp; frame ID: `map`
- Missing transform: silently skip publication
- Stale TF: not explicitly filtered in either baseline implementation

## Build

Use Ubuntu 24.04 / ROS 2 Jazzy and an existing workspace checkout containing
the `src/` directory. Run commands from the workspace root:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select logitle_pose_cpp --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

The instructions above are proposed, not a report of a successful build.

## Isolated side-by-side run

Use an isolated ROS_DOMAIN_ID and compatible live /tf, /tf_static, /clock inputs.
Do not run both publishers on the same output topic in a production domain.
In a new shell (with the ROS environment sourced), run:

```bash
ros2 run logitle_pose logitle_pose_publisher --ros-args \
  -r __node:=logitle_pose_publisher_py_bench \
  -p topic_name:=/bench/pose_py

ros2 run logitle_pose_cpp logitle_pose_publisher_cpp --ros-args \
  -p topic_name:=/bench/pose_cpp
```

The commands above are separate long-running processes; start one in each terminal.
Set `-p use_sim_time:=true` on **both** when replaying a bag with a /clock
publisher. The C++ node uses the node ROS clock, not a wall timer.

Check node graph and output:

```bash
ros2 topic info -v /bench/pose_py
ros2 topic info -v /bench/pose_cpp
ros2 topic hz /bench/pose_py
ros2 topic hz /bench/pose_cpp
ros2 topic echo --once /bench/pose_py
ros2 topic echo --once /bench/pose_cpp
```

## Functional parity checklist

1. Use identical TF input data and the same ROS clock.
2. Check both topics' types, QoS, frame_id and stamp semantics.
3. Compare position and quaternion values **at the same original TF stamp**;
   do not pair samples merely by receipt order.
4. Verify that both skip messages before the first usable TF transform.
5. Freeze the TF feed and document whether both continue emitting the last
   available transform. (Baseline does not reject stale transforms.)
6. Stop/restart both processes and repeat.
7. If using bag replay, repeat runs separately as well as concurrently to
   check interference and prevent one process affecting the other's metrics.

## Performance benchmark design (not yet executed)

- Same computer, middleware, ROS_DOMAIN_ID, ROS parameters, TF input and
  process priority; warm-up before measuring.
- Measure per-process CPU and RSS with `pidstat -p <pid> -r -u 1` (or psutil).
- Record received message intervals and TF stamp age separately. A source
  transform can be emitted at 5 Hz despite being updated more slowly.
- For actual node-internal TF lookup latency, instrument both nodes with
  a monotonic/steady clock; do not infer it from subscriber timestamps.
- Compare repeated trials, median, p95/p99, standard deviation, failures,
  CSV samples and environment details.
- Consider separate cold start and steady state metrics; process startup
  and Python/C++ runtime overhead are different concepts.
- No throughput, CPU, memory or speedup results have yet been measured.

## Notes

- This baseline intentionally retains the Python behavior for unavailable
  and stale TF. New safety behavior must be a separate follow-up experiment.
- The C++ node checks that publish_rate_hz is positive and finite. Invalid
  configuration is not part of the equivalence benchmark.
- Compilation and hardware validation must be completed on ROS2 Jazzy; there
  is no claim that a build or test has already passed.
