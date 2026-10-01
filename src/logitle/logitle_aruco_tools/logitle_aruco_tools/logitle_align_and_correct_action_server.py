#!/usr/bin/env python3
"""Action server that aligns to an ArUco marker, then corrects pose."""

import argparse
import math
import threading
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from logitle_aruco_msgs.action import AlignAndCorrectWithAruco, CorrectPoseWithAruco


RESULT_SUCCESS = AlignAndCorrectWithAruco.Result.RESULT_SUCCESS
RESULT_TIMEOUT = AlignAndCorrectWithAruco.Result.RESULT_TIMEOUT
RESULT_CANCELED = AlignAndCorrectWithAruco.Result.RESULT_CANCELED
RESULT_MARKER_LOST = AlignAndCorrectWithAruco.Result.RESULT_MARKER_LOST
RESULT_ALIGN_FAILED = AlignAndCorrectWithAruco.Result.RESULT_ALIGN_FAILED
RESULT_CORRECTION_FAILED = AlignAndCorrectWithAruco.Result.RESULT_CORRECTION_FAILED
RESULT_INVALID_GOAL = AlignAndCorrectWithAruco.Result.RESULT_INVALID_GOAL


def clamp(value, low, high):
    return max(low, min(high, value))


def positive_or_default(value, default):
    return float(value) if float(value) > 0.0 else float(default)


def int_or_default(value, default):
    return int(value) if int(value) > 0 else int(default)


def bool_arg(value):
    if isinstance(value, bool):
        return value
    text = str(value).strip().lower()
    if text in ("1", "true", "yes", "on"):
        return True
    if text in ("0", "false", "no", "off"):
        return False
    raise argparse.ArgumentTypeError(f"expected a boolean value, got {value!r}")


class AlignAndCorrectActionServer(Node):
    def __init__(self, args):
        super().__init__("align_and_correct_with_aruco_action_server")
        self.args = args
        self.callback_group = ReentrantCallbackGroup()
        self.goal_lock = threading.Lock()
        self.goal_active = False
        self.pose_lock = threading.Lock()
        self.latest_pose = None
        self.pose_sub = None

        cmd_type = TwistStamped if args.cmd_vel_stamped else Twist
        self.cmd_pub = self.create_publisher(cmd_type, args.cmd_vel_topic, 10)
        self.correct_client = ActionClient(
            self,
            CorrectPoseWithAruco,
            args.correct_action_name,
            callback_group=self.callback_group,
        )
        self.action_server = ActionServer(
            self,
            AlignAndCorrectWithAruco,
            args.action_name,
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.callback_group,
        )

        self.get_logger().info(
            f"Serving {args.action_name}; correct_action={args.correct_action_name}; "
            f"cmd_vel={args.cmd_vel_topic}; cmd_type={cmd_type.__name__}"
        )

    def goal_callback(self, goal_request):
        if int(goal_request.marker_id) <= 0 and self.args.marker_id <= 0:
            self.get_logger().warn("Rejecting goal: marker_id must be positive")
            return GoalResponse.REJECT
        with self.goal_lock:
            if self.goal_active:
                self.get_logger().warn("Rejecting goal: another align/correct goal is already active")
                return GoalResponse.REJECT
            self.goal_active = True
        return GoalResponse.ACCEPT

    def cancel_callback(self, _goal_handle):
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        goal = goal_handle.request
        marker_id = int(goal.marker_id) if goal.marker_id > 0 else self.args.marker_id
        params = self.params_from_goal(goal)

        if not self.validate_params(params):
            result = self.make_result(False, RESULT_INVALID_GOAL, "Invalid align/correct goal values.")
            goal_handle.abort()
            self.release_goal()
            return result

        topic = self.pose_topic_for_marker(marker_id)
        self.start_pose_input(topic)
        self.get_logger().info(
            f"Accepted align/correct goal: marker_id={marker_id}; target_z={params['target_z']:.3f}m; "
            f"align_timeout={params['align_timeout_sec']:.1f}s; apply_correction={goal.apply_correction}"
        )

        try:
            align_result = self.run_alignment(goal_handle, params)
            if goal_handle.is_cancel_requested:
                result = self.make_result(False, RESULT_CANCELED, "Align/correct canceled.", **align_result)
                goal_handle.canceled()
                return result

            if not align_result["align_success"]:
                code = RESULT_MARKER_LOST if align_result["marker_lost"] else RESULT_ALIGN_FAILED
                result = self.make_result(False, code, align_result["align_message"], **align_result)
                goal_handle.abort()
                return result

            if not goal.apply_correction:
                result = self.make_result(
                    True,
                    RESULT_SUCCESS,
                    "ArUco alignment succeeded; pose correction was skipped.",
                    **align_result,
                )
                goal_handle.succeed()
                return result

            correction = self.run_pose_correction(goal_handle, goal, marker_id, params["correct_timeout_sec"])
            merged = dict(align_result)
            merged.update(correction)
            if not correction["correction_success"]:
                result = self.make_result(
                    False,
                    RESULT_CORRECTION_FAILED,
                    "Alignment succeeded, but pose correction failed.",
                    **merged,
                )
                goal_handle.abort()
                return result

            result = self.make_result(
                True,
                RESULT_SUCCESS,
                "Alignment and pose correction succeeded.",
                **merged,
            )
            goal_handle.succeed()
            return result
        finally:
            self.stop_robot()
            self.stop_pose_input()
            self.release_goal()

    def params_from_goal(self, goal):
        return {
            "timeout_sec": positive_or_default(goal.timeout_sec, self.args.timeout_sec),
            "align_timeout_sec": positive_or_default(goal.align_timeout_sec, self.args.align_timeout_sec),
            "correct_timeout_sec": positive_or_default(goal.correct_timeout_sec, self.args.correct_timeout_sec),
            "target_z": positive_or_default(goal.target_z, self.args.target_z),
            "x_tolerance": positive_or_default(goal.x_tolerance, self.args.x_tolerance),
            "z_tolerance": positive_or_default(goal.z_tolerance, self.args.z_tolerance),
            "z_min_stop": positive_or_default(goal.z_min_stop, self.args.z_min_stop),
            "stable_sec": positive_or_default(goal.stable_sec, self.args.stable_sec),
            "max_linear": positive_or_default(goal.max_linear, self.args.max_linear),
            "max_angular": positive_or_default(goal.max_angular, self.args.max_angular),
            "kx": positive_or_default(goal.kx, self.args.kx),
            "kz": positive_or_default(goal.kz, self.args.kz),
            "required_samples": int_or_default(goal.required_samples, self.args.required_samples),
        }

    def validate_params(self, params):
        for value in params.values():
            if not math.isfinite(float(value)):
                return False
        return (
            params["z_min_stop"] < params["target_z"]
            and params["x_tolerance"] > 0.0
            and params["z_tolerance"] > 0.0
            and params["max_linear"] > 0.0
            and params["max_angular"] > 0.0
        )

    def run_alignment(self, goal_handle, params):
        started = time.time()
        aligned_since = None
        last_marker_seen = False
        last_feedback = 0.0
        final_x = 0.0
        final_z = 0.0
        final_x_error = 0.0
        final_z_error = 0.0

        while rclpy.ok():
            now = time.time()
            if goal_handle.is_cancel_requested:
                return self.align_result(False, False, final_x, final_z, final_x_error, final_z_error, "Canceled.")

            if now - started > min(params["timeout_sec"], params["align_timeout_sec"]):
                self.stop_robot()
                message = "ArUco alignment timed out."
                if not last_marker_seen:
                    message = "ArUco alignment timed out waiting for marker."
                return self.align_result(False, not last_marker_seen, final_x, final_z, final_x_error, final_z_error, message)

            pose = self.get_latest_pose()
            marker_visible = pose is not None and (now - pose["received_at"]) <= self.args.pose_timeout
            if not marker_visible:
                aligned_since = None
                self.stop_robot()
                if now - last_feedback >= self.args.feedback_period:
                    self.publish_feedback(goal_handle, "waiting_for_marker", False, final_x, final_z, final_x_error, final_z_error, 0.0, 0.0, 0.0)
                    last_feedback = now
                time.sleep(self.args.control_period)
                continue

            last_marker_seen = True
            msg = pose["msg"]
            x = float(msg.pose.position.x)
            z = float(msg.pose.position.z)
            linear_x, angular_z, state, x_error, z_error = self.compute_cmd(x, z, params)
            final_x = x
            final_z = z
            final_x_error = x_error
            final_z_error = z_error

            if state == "too_close_stop":
                self.stop_robot()
                return self.align_result(False, False, x, z, x_error, z_error, "Marker is too close; stopped before alignment.")

            if state == "aligned":
                self.stop_robot()
                if aligned_since is None:
                    aligned_since = now
            else:
                aligned_since = None
                self.publish_cmd(linear_x, angular_z)

            aligned_duration = 0.0 if aligned_since is None else now - aligned_since
            if now - last_feedback >= self.args.feedback_period:
                self.publish_feedback(goal_handle, state, True, x, z, x_error, z_error, linear_x, angular_z, aligned_duration)
                last_feedback = now

            if aligned_since is not None and aligned_duration >= params["stable_sec"]:
                self.stop_robot()
                return self.align_result(True, False, x, z, x_error, z_error, "ArUco alignment succeeded.")

            time.sleep(self.args.control_period)

        return self.align_result(False, False, final_x, final_z, final_x_error, final_z_error, "ROS shutdown during alignment.")

    def compute_cmd(self, x, z, params):
        if z < params["z_min_stop"]:
            return 0.0, 0.0, "too_close_stop", x, z - params["target_z"]

        x_error = 0.0 if abs(x) <= params["x_tolerance"] else x
        z_error = 0.0 if abs(z - params["target_z"]) <= params["z_tolerance"] else z - params["target_z"]
        if x_error == 0.0 and z_error == 0.0:
            return 0.0, 0.0, "aligned", x_error, z_error

        angular_z = clamp(-params["kx"] * x_error, -params["max_angular"], params["max_angular"])
        linear_x = clamp(params["kz"] * z_error, -params["max_linear"], params["max_linear"])
        return linear_x, angular_z, "aligning", x_error, z_error

    def run_pose_correction(self, goal_handle, align_goal, marker_id, timeout_sec):
        if not self.correct_client.wait_for_server(timeout_sec=timeout_sec):
            return {
                "correction_success": False,
                "correction_result_code": CorrectPoseWithAruco.Result.RESULT_TIMEOUT,
                "correction_message": f"Correction action server {self.args.correct_action_name} is not available.",
            }

        correct_goal = CorrectPoseWithAruco.Goal()
        correct_goal.marker_id = int(marker_id)
        correct_goal.publish_tf = True
        correct_goal.timeout_sec = float(timeout_sec)
        correct_goal.required_samples = int_or_default(align_goal.required_samples, self.args.required_samples)
        correct_goal.check_yaw = bool(align_goal.check_yaw)
        correct_goal.expected_base_yaw_deg = float(align_goal.expected_base_yaw_deg)
        correct_goal.yaw_tolerance_deg = positive_or_default(align_goal.yaw_tolerance_deg, self.args.yaw_tolerance_deg)

        send_future = self.correct_client.send_goal_async(
            correct_goal,
            feedback_callback=lambda feedback: self.publish_correction_feedback(goal_handle, feedback.feedback),
        )
        goal_response = self.wait_for_future(send_future, timeout_sec)
        if goal_response is None or not goal_response.accepted:
            return {
                "correction_success": False,
                "correction_result_code": CorrectPoseWithAruco.Result.RESULT_INVALID_GOAL,
                "correction_message": "Correction goal was rejected.",
            }

        result_future = goal_response.get_result_async()
        result_response = self.wait_for_future(result_future, timeout_sec)
        if result_response is None:
            return {
                "correction_success": False,
                "correction_result_code": CorrectPoseWithAruco.Result.RESULT_TIMEOUT,
                "correction_message": "Correction action timed out.",
            }

        result = result_response.result
        return {
            "correction_success": bool(result.success),
            "correction_result_code": int(result.result_code),
            "correction_message": str(result.message),
            "map_base_x": float(result.map_base_x),
            "map_base_y": float(result.map_base_y),
            "map_base_yaw_deg": float(result.map_base_yaw_deg),
            "map_base_xy_std": float(result.map_base_xy_std),
            "map_base_yaw_std_deg": float(result.map_base_yaw_std_deg),
        }

    def wait_for_future(self, future, timeout_sec):
        done = threading.Event()
        future.add_done_callback(lambda _future: done.set())
        if not done.wait(timeout=max(0.1, float(timeout_sec))):
            return None
        return future.result()

    def publish_correction_feedback(self, goal_handle, feedback):
        out = AlignAndCorrectWithAruco.Feedback()
        out.state = "correcting_pose"
        out.marker_visible = bool(feedback.marker_visible)
        out.accepted_samples = int(feedback.accepted_samples)
        out.rejected_samples = int(feedback.rejected_samples)
        out.correction_state = str(feedback.state)
        goal_handle.publish_feedback(out)

    def publish_feedback(
        self,
        goal_handle,
        state,
        marker_visible,
        marker_x,
        marker_z,
        x_error,
        z_error,
        linear_x,
        angular_z,
        aligned_duration,
    ):
        feedback = AlignAndCorrectWithAruco.Feedback()
        feedback.state = state
        feedback.marker_visible = bool(marker_visible)
        feedback.latest_marker_x = float(marker_x)
        feedback.latest_marker_z = float(marker_z)
        feedback.latest_x_error = float(x_error)
        feedback.latest_z_error = float(z_error)
        feedback.command_linear_x = float(linear_x)
        feedback.command_angular_z = float(angular_z)
        feedback.aligned_duration_sec = float(aligned_duration)
        feedback.correction_state = ""
        goal_handle.publish_feedback(feedback)

    def align_result(self, success, marker_lost, marker_x, marker_z, x_error, z_error, message):
        return {
            "align_success": bool(success),
            "marker_lost": bool(marker_lost),
            "final_marker_x": float(marker_x),
            "final_marker_z": float(marker_z),
            "final_x_error": float(x_error),
            "final_z_error": float(z_error),
            "align_message": str(message),
        }

    def make_result(
        self,
        success,
        result_code,
        message,
        align_success=False,
        correction_success=False,
        final_marker_x=0.0,
        final_marker_z=0.0,
        final_x_error=0.0,
        final_z_error=0.0,
        correction_result_code=0,
        correction_message="",
        map_base_x=0.0,
        map_base_y=0.0,
        map_base_yaw_deg=0.0,
        map_base_xy_std=0.0,
        map_base_yaw_std_deg=0.0,
        marker_lost=False,
        align_message="",
    ):
        del marker_lost, align_message
        result = AlignAndCorrectWithAruco.Result()
        result.success = bool(success)
        result.result_code = int(result_code)
        result.message = str(message)
        result.align_success = bool(align_success)
        result.correction_success = bool(correction_success)
        result.final_marker_x = float(final_marker_x)
        result.final_marker_z = float(final_marker_z)
        result.final_x_error = float(final_x_error)
        result.final_z_error = float(final_z_error)
        result.correction_result_code = int(correction_result_code)
        result.correction_message = str(correction_message)
        result.map_base_x = float(map_base_x)
        result.map_base_y = float(map_base_y)
        result.map_base_yaw_deg = float(map_base_yaw_deg)
        result.map_base_xy_std = float(map_base_xy_std)
        result.map_base_yaw_std_deg = float(map_base_yaw_std_deg)
        return result

    def start_pose_input(self, topic):
        with self.pose_lock:
            self.latest_pose = None
        self.pose_sub = self.create_subscription(
            PoseStamped,
            topic,
            self.pose_cb,
            10,
            callback_group=self.callback_group,
        )
        self.get_logger().info(f"Subscribed to {topic} for pickup alignment")

    def stop_pose_input(self):
        if self.pose_sub is not None:
            self.destroy_subscription(self.pose_sub)
            self.pose_sub = None
        with self.pose_lock:
            self.latest_pose = None

    def pose_cb(self, msg):
        with self.pose_lock:
            self.latest_pose = {
                "received_at": time.time(),
                "msg": msg,
            }

    def get_latest_pose(self):
        with self.pose_lock:
            return None if self.latest_pose is None else dict(self.latest_pose)

    def pose_topic_for_marker(self, marker_id):
        try:
            return self.args.pose_topic_template.format(marker_id=marker_id)
        except (KeyError, IndexError, ValueError):
            return self.args.pose_topic_template

    def publish_cmd(self, linear_x, angular_z):
        if self.args.cmd_vel_stamped:
            cmd = TwistStamped()
            cmd.header.stamp = self.get_clock().now().to_msg()
            cmd.header.frame_id = self.args.cmd_frame_id
            cmd.twist.linear.x = float(linear_x)
            cmd.twist.angular.z = float(angular_z)
        else:
            cmd = Twist()
            cmd.linear.x = float(linear_x)
            cmd.angular.z = float(angular_z)
        self.cmd_pub.publish(cmd)

    def stop_robot(self):
        for _ in range(3):
            self.publish_cmd(0.0, 0.0)

    def release_goal(self):
        with self.goal_lock:
            self.goal_active = False

    def destroy_node(self):
        self.stop_robot()
        self.stop_pose_input()
        self.action_server.destroy()
        super().destroy_node()


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--action-name", default="/aruco_align_and_correct")
    parser.add_argument("--correct-action-name", default="/aruco_correct_pose")
    parser.add_argument("--pose-topic-template", default="/aruco/id{marker_id}/pose_camera")
    parser.add_argument("--marker-id", type=int, default=24)

    parser.add_argument("--cmd-vel-topic", default="/cmd_vel")
    parser.add_argument("--cmd-vel-stamped", type=bool_arg, default=True)
    parser.add_argument("--cmd-frame-id", default="base_link")

    parser.add_argument("--target-z", type=float, default=0.32)
    parser.add_argument("--x-tolerance", type=float, default=0.025)
    parser.add_argument("--z-tolerance", type=float, default=0.035)
    parser.add_argument("--z-min-stop", type=float, default=0.25)
    parser.add_argument("--stable-sec", type=float, default=0.5)
    parser.add_argument("--max-linear", type=float, default=0.015)
    parser.add_argument("--max-angular", type=float, default=0.08)
    parser.add_argument("--kx", type=float, default=2.0)
    parser.add_argument("--kz", type=float, default=0.4)

    parser.add_argument("--timeout-sec", type=float, default=20.0)
    parser.add_argument("--align-timeout-sec", type=float, default=10.0)
    parser.add_argument("--correct-timeout-sec", type=float, default=10.0)
    parser.add_argument("--required-samples", type=int, default=15)
    parser.add_argument("--yaw-tolerance-deg", type=float, default=4.0)

    parser.add_argument("--pose-timeout", type=float, default=0.3)
    parser.add_argument("--feedback-period", type=float, default=0.2)
    parser.add_argument("--control-period", type=float, default=0.03)
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = AlignAndCorrectActionServer(args)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
