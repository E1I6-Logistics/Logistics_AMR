"""ROS 2 capture, action, TF, and parameter helpers."""

import math
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistStamped
from nav2_msgs.action import NavigateToPose
from rcl_interfaces.msg import ParameterType
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from .metrics import quaternion_to_yaw, yaw_to_quaternion


STATUS_NAMES = {
    GoalStatus.STATUS_UNKNOWN: "UNKNOWN",
    GoalStatus.STATUS_ACCEPTED: "ACCEPTED",
    GoalStatus.STATUS_EXECUTING: "EXECUTING",
    GoalStatus.STATUS_CANCELING: "CANCELING",
    GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
    GoalStatus.STATUS_CANCELED: "CANCELED",
    GoalStatus.STATUS_ABORTED: "ABORTED",
}


def stamp_seconds(stamp):
    """Convert a ROS builtin time message to floating-point seconds."""
    return float(stamp.sec) + float(stamp.nanosec) / 1_000_000_000.0


def parameter_value_to_python(value):
    """Convert rcl_interfaces/ParameterValue to a Python value."""
    mapping = {
        ParameterType.PARAMETER_BOOL: value.bool_value,
        ParameterType.PARAMETER_INTEGER: value.integer_value,
        ParameterType.PARAMETER_DOUBLE: value.double_value,
        ParameterType.PARAMETER_STRING: value.string_value,
        ParameterType.PARAMETER_BYTE_ARRAY: list(value.byte_array_value),
        ParameterType.PARAMETER_BOOL_ARRAY: list(value.bool_array_value),
        ParameterType.PARAMETER_INTEGER_ARRAY: list(value.integer_array_value),
        ParameterType.PARAMETER_DOUBLE_ARRAY: list(value.double_array_value),
        ParameterType.PARAMETER_STRING_ARRAY: list(value.string_array_value),
    }
    if value.type == ParameterType.PARAMETER_NOT_SET:
        return None
    if value.type not in mapping:
        raise TypeError(f"Unsupported ROS parameter type: {value.type}")
    return mapping[value.type]


class ParameterManager:
    """Read, apply, verify, and restore parameters on lifecycle nodes."""

    def __init__(self, node):
        """Initialize parameter clients owned by an experiment node."""
        self.node = node
        self.clients = {}
        self.original = {}

    def _client(self, node_name):
        normalized = node_name if node_name.startswith("/") else f"/{node_name}"
        if normalized not in self.clients:
            client = AsyncParameterClient(self.node, normalized)
            if not client.wait_for_services(timeout_sec=10.0):
                raise RuntimeError(f"Parameter service unavailable: {normalized}")
            self.clients[normalized] = client
        return normalized, self.clients[normalized]

    def get(self, node_name, names):
        """Read named parameters and return a name-to-value mapping."""
        _, client = self._client(node_name)
        future = client.get_parameters(list(names))
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)
        response = future.result()
        if response is None or len(response.values) != len(names):
            raise RuntimeError(f"Failed to read parameters from {node_name}")
        return {
            name: parameter_value_to_python(value)
            for name, value in zip(names, response.values)
        }

    def backup(self, parameter_plan):
        """Back up every parameter that any condition may modify."""
        for node_name, names in parameter_plan.items():
            normalized, _ = self._client(node_name)
            values = self.get(normalized, sorted(names))
            self.original[normalized] = values
        return self.original

    def apply(self, node_name, values):
        """Set a parameter mapping and verify the resulting values."""
        normalized, client = self._client(node_name)
        parameters = [Parameter(name=name, value=value) for name, value in values.items()]
        future = client.set_parameters(parameters)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)
        response = future.result()
        if response is None or len(response.results) != len(parameters):
            raise RuntimeError(f"Failed to set parameters on {normalized}")
        failures = [
            result.reason or "rejected"
            for result in response.results
            if not result.successful
        ]
        if failures:
            raise RuntimeError(f"Parameter update rejected on {normalized}: {failures}")
        actual = self.get(normalized, values.keys())
        for name, expected in values.items():
            observed = actual[name]
            if isinstance(expected, float):
                if observed is None or not math.isclose(
                    float(observed), expected, rel_tol=1e-9, abs_tol=1e-9
                ):
                    raise RuntimeError(
                        f"Parameter verification failed: {normalized}.{name} "
                        f"expected={expected}, observed={observed}"
                    )
            elif observed != expected:
                raise RuntimeError(
                    f"Parameter verification failed: {normalized}.{name} "
                    f"expected={expected}, observed={observed}"
                )
        return actual

    def apply_condition(self, parameters):
        """Apply all node mappings in one configured condition."""
        observed = {}
        for node_name, values in parameters.items():
            normalized = node_name if node_name.startswith("/") else f"/{node_name}"
            observed[normalized] = self.apply(normalized, values)
        return observed

    def restore(self):
        """Restore every backed-up value, collecting errors for the caller."""
        errors = []
        for node_name, values in self.original.items():
            try:
                self.apply(node_name, values)
            except Exception as error:  # best-effort safety cleanup
                errors.append(f"{node_name}: {error}")
        return errors


class ExperimentNode(Node):
    """Capture AMCL, TF, velocity commands, and NavigateToPose results."""

    def __init__(self, robot_config):
        """Create subscriptions, publishers, action client, and TF buffer."""
        super().__init__("logitle_experiment_runner")
        self.robot_config = robot_config
        topics = robot_config.get("topics", {})
        actions = robot_config.get("actions", {})
        frames = robot_config.get("frames", {})

        self.map_frame = frames.get("map", "map")
        self.base_frame = frames.get("base", "base_footprint")
        self.amcl_topic = topics.get("amcl_pose", "/amcl_pose")
        self.initialpose_topic = topics.get("initialpose", "/initialpose")
        self.cmd_vel_topic = topics.get("cmd_vel", "/cmd_vel")
        self.navigate_action = actions.get("navigate_to_pose", "/navigate_to_pose")

        reliable_volatile = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.latest_amcl = None
        self.amcl_count = 0
        self.cmd_vel_max_x = 0.0
        self.trace_store = None
        self.trace_context = None

        self.create_subscription(
            PoseWithCovarianceStamped,
            self.amcl_topic,
            self._amcl_callback,
            reliable_volatile,
        )
        self.create_subscription(
            TwistStamped,
            self.cmd_vel_topic,
            self._cmd_vel_callback,
            reliable_volatile,
        )
        self.initialpose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            self.initialpose_topic,
            reliable_volatile,
        )
        self.navigator = ActionClient(self, NavigateToPose, self.navigate_action)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.parameters = ParameterManager(self)

    def _amcl_callback(self, message):
        self.latest_amcl = message
        self.amcl_count += 1
        if self.trace_store is not None:
            sample = self.amcl_sample(message)
            self.trace_store.append_trace(
                {
                    **self.trace_context,
                    "trial_elapsed_sec": time.monotonic()
                    - self.trace_context["monotonic_start"],
                    "source": "amcl",
                    "source_stamp_sec": sample["stamp_sec"],
                    "amcl_x_m": sample["x_m"],
                    "amcl_y_m": sample["y_m"],
                    "amcl_yaw_rad": sample["yaw_rad"],
                    "amcl_cov_x": sample["cov_x"],
                    "amcl_cov_y": sample["cov_y"],
                    "amcl_cov_yaw": sample["cov_yaw"],
                }
            )

    def _cmd_vel_callback(self, message):
        linear_x = float(message.twist.linear.x)
        self.cmd_vel_max_x = max(self.cmd_vel_max_x, abs(linear_x))
        if self.trace_store is not None:
            self.trace_store.append_trace(
                {
                    **self.trace_context,
                    "trial_elapsed_sec": time.monotonic()
                    - self.trace_context["monotonic_start"],
                    "source": "cmd_vel",
                    "source_stamp_sec": stamp_seconds(message.header.stamp),
                    "cmd_vel_x_mps": linear_x,
                    "cmd_vel_yaw_rps": float(message.twist.angular.z),
                }
            )

    def begin_trace(self, store, run_id, sequence_index):
        """Start optional callback-level trace logging for one trial."""
        self.reset_motion_metrics()
        self.trace_store = store
        self.trace_context = {
            "schema_version": 1,
            "run_id": run_id,
            "sequence_index": sequence_index,
            "monotonic_start": time.monotonic(),
        }

    def end_trace(self):
        """Stop trace logging."""
        self.trace_store = None
        self.trace_context = None

    def reset_motion_metrics(self):
        """Reset per-trial velocity observations."""
        self.cmd_vel_max_x = 0.0

    def spin_for(self, seconds):
        """Process callbacks for a fixed wall-clock duration."""
        deadline = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=min(0.1, deadline - time.monotonic()))

    def wait_for_amcl_publisher(self, timeout_sec=10.0):
        """Wait for an AMCL publisher without requiring an initialized pose."""
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            if self.count_publishers(self.amcl_topic) > 0:
                return
            rclpy.spin_once(self, timeout_sec=0.1)
        raise TimeoutError(f"No AMCL publisher on {self.amcl_topic}")

    def wait_for_initialpose_subscriber(self, timeout_sec=10.0):
        """Wait for AMCL to subscribe to the initial-pose topic."""
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            if self.initialpose_pub.get_subscription_count() > 0:
                return
            rclpy.spin_once(self, timeout_sec=0.1)
        raise TimeoutError(f"No subscriber on {self.initialpose_topic}")

    def wait_for_amcl(self, timeout_sec=10.0, after_count=None):
        """Wait for the first or a newly received AMCL pose."""
        threshold = -1 if after_count is None else after_count
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            if self.latest_amcl is not None and self.amcl_count > threshold:
                return self.latest_amcl
            rclpy.spin_once(self, timeout_sec=0.1)
        raise TimeoutError(f"No new AMCL pose on {self.amcl_topic}")

    @staticmethod
    def amcl_sample(message):
        """Convert a PoseWithCovarianceStamped into a flat dictionary."""
        pose = message.pose.pose
        covariance = message.pose.covariance
        return {
            "x_m": float(pose.position.x),
            "y_m": float(pose.position.y),
            "yaw_rad": quaternion_to_yaw(pose.orientation),
            "cov_x": float(covariance[0]),
            "cov_y": float(covariance[7]),
            "cov_yaw": float(covariance[35]),
            "stamp_sec": stamp_seconds(message.header.stamp),
        }

    def tf_sample(self, timeout_sec=2.0):
        """Read the latest map-to-base transform."""
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                Time(),
                timeout=Duration(seconds=timeout_sec),
            )
        except TransformException as error:
            raise RuntimeError(
                f"TF unavailable: {self.map_frame} -> {self.base_frame}: {error}"
            ) from error
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        return {
            "x_m": float(translation.x),
            "y_m": float(translation.y),
            "yaw_rad": quaternion_to_yaw(rotation),
            "stamp_sec": stamp_seconds(transform.header.stamp),
        }

    def publish_initial_pose(self, pose, position_stddev_m, yaw_stddev_deg):
        """Publish a numeric initial pose with explicit covariance."""
        self.wait_for_initialpose_subscriber(timeout_sec=5.0)

        message = PoseWithCovarianceStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.map_frame
        message.pose.pose.position.x = float(pose["x"])
        message.pose.pose.position.y = float(pose["y"])
        qx, qy, qz, qw = yaw_to_quaternion(float(pose["yaw"]))
        message.pose.pose.orientation.x = qx
        message.pose.pose.orientation.y = qy
        message.pose.pose.orientation.z = qz
        message.pose.pose.orientation.w = qw
        covariance = [0.0] * 36
        covariance[0] = float(position_stddev_m) ** 2
        covariance[7] = float(position_stddev_m) ** 2
        covariance[35] = math.radians(float(yaw_stddev_deg)) ** 2
        message.pose.covariance = covariance

        previous_count = self.amcl_count
        self.initialpose_pub.publish(message)
        return self.wait_for_amcl(timeout_sec=10.0, after_count=previous_count)

    def wait_for_navigation(self, timeout_sec=15.0):
        """Wait for NavigateToPose action availability."""
        if not self.navigator.wait_for_server(timeout_sec=timeout_sec):
            raise RuntimeError(f"Action server unavailable: {self.navigate_action}")

    def navigate(self, goal_pose):
        """Send one NavigateToPose goal and wait for its terminal result."""
        self.wait_for_navigation()
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self.map_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(goal_pose["x"])
        goal.pose.pose.position.y = float(goal_pose["y"])
        qx, qy, qz, qw = yaw_to_quaternion(float(goal_pose["yaw"]))
        goal.pose.pose.orientation.x = qx
        goal.pose.pose.orientation.y = qy
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        started = time.monotonic()
        send_future = self.navigator.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            return {"status": "REJECTED", "time_sec": time.monotonic() - started}

        result_future = goal_handle.get_result_async()
        try:
            while rclpy.ok() and not result_future.done():
                rclpy.spin_once(self, timeout_sec=0.1)
        except KeyboardInterrupt:
            cancel_future = goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=5.0)
            raise

        wrapped = result_future.result()
        status = "UNKNOWN" if wrapped is None else STATUS_NAMES.get(
            wrapped.status, str(wrapped.status)
        )
        return {"status": status, "time_sec": time.monotonic() - started}
