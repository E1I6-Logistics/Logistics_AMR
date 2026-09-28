#!/usr/bin/env python3

import argparse
import csv
import math
import time
from datetime import datetime
from pathlib import Path

import yaml

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import (
    QoSProfile,
    QoSHistoryPolicy,
    QoSReliabilityPolicy,
    QoSDurabilityPolicy,
)

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from rcl_interfaces.msg import ParameterType

from .metrics import (
    heading_error_rad,
    position_error,
    quaternion_to_yaw,
    yaw_to_quaternion,
)


# ============================================================
# Utility
# ============================================================


def load_yaml(path):
    path = Path(path).expanduser()

    with path.open("r", encoding="utf-8") as file:
        return yaml.safe_load(file)


def required_nonnegative_float(prompt):
    while True:
        raw = input(prompt).strip()

        try:
            value = float(raw)

            if value < 0.0:
                raise ValueError

            return value

        except ValueError:
            print("0 이상의 숫자를 입력해주세요.")


def optional_float(prompt):
    while True:
        raw = input(prompt).strip()

        if raw == "":
            return None

        try:
            return float(raw)

        except ValueError:
            print(
                "숫자를 입력하거나 "
                "생략하려면 ENTER를 누르세요."
            )


# ============================================================
# Experiment Node
# ============================================================


class GoalPoseAccuracyExperiment(Node):

    PARAM_XY = "goal_checker.xy_goal_tolerance"
    PARAM_YAW = "goal_checker.yaw_goal_tolerance"

    def __init__(
        self,
        robot_config,
        scenario_config,
        experiment_config,
        output_dir,
        trials_override=None,
        only_condition=None,
    ):
        super().__init__(
            "goal_pose_accuracy_experiment"
        )

        # ----------------------------------------------------
        # Config
        # ----------------------------------------------------

        self.robot_config = robot_config
        self.scenario = scenario_config
        self.experiment = experiment_config

        self.robot = robot_config["robot"]

        self.trials_override = trials_override
        self.only_condition = only_condition

        self.experiment_cfg = (
            self.experiment["experiment"]
        )

        self.goal_checker_cfg = (
            self.experiment["goal_checker"]
        )

        self.execution_cfg = (
            self.experiment.get(
                "execution",
                {},
            )
        )

        self.return_cfg = (
            self.experiment.get(
                "return_to_start",
                {},
            )
        )

        self.initial_localization_cfg = (
            self.experiment.get(
                "initial_localization",
                {},
            )
        )

        # ----------------------------------------------------
        # Topics
        # ----------------------------------------------------

        topics = robot_config["topics"]

        self.amcl_topic = topics.get(
            "amcl_pose",
            "/amcl_pose",
        )

        self.initialpose_topic = topics.get(
            "initialpose",
            "/initialpose",
        )

        # ----------------------------------------------------
        # AMCL state
        # ----------------------------------------------------

        self.latest_amcl_pose = None
        self.amcl_message_count = 0

        # Nav2 AMCL의 amcl_pose는 latched / transient-local
        # 방식으로 사용될 수 있으므로 late subscriber도
        # 마지막 pose를 받을 수 있게 맞춘다.
        amcl_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            self.amcl_topic,
            self._amcl_callback,
            amcl_qos,
        )

        # /initialpose
        self.initialpose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            self.initialpose_topic,
            10,
        )

        # ----------------------------------------------------
        # Nav2 NavigateToPose
        # ----------------------------------------------------

        self.navigator = ActionClient(
            self,
            NavigateToPose,
            "/navigate_to_pose",
        )

        # ----------------------------------------------------
        # controller_server parameters
        # ----------------------------------------------------

        self.controller_params = (
            AsyncParameterClient(
                self,
                "/controller_server",
            )
        )

        self.original_xy_tolerance = None
        self.original_yaw_tolerance = None

        # ----------------------------------------------------
        # CSV
        # ----------------------------------------------------

        self.output_dir = Path(
            output_dir
        ).expanduser()

        self.output_dir.mkdir(
            parents=True,
            exist_ok=True,
        )

        self.csv_file = None
        self.csv_writer = None
        self.csv_path = None

    # ========================================================
    # AMCL
    # ========================================================

    def _amcl_callback(self, msg):
        self.latest_amcl_pose = msg
        self.amcl_message_count += 1

    def spin_for(self, duration_sec):
        deadline = (
            time.monotonic()
            + duration_sec
        )

        while (
            rclpy.ok()
            and time.monotonic() < deadline
        ):
            remaining = (
                deadline
                - time.monotonic()
            )

            rclpy.spin_once(
                self,
                timeout_sec=min(
                    0.1,
                    max(0.0, remaining),
                ),
            )

    def wait_for_amcl_pose(
        self,
        timeout_sec=5.0,
    ):
        """
        AMCL pose가 한 번도 들어오지 않은 경우
        최초 pose를 기다린다.
        """

        deadline = (
            time.monotonic()
            + timeout_sec
        )

        while rclpy.ok():

            if self.latest_amcl_pose is not None:
                return self.latest_amcl_pose

            if time.monotonic() >= deadline:
                raise RuntimeError(
                    f"No {self.amcl_topic} received. "
                    "Check AMCL / localization / QoS."
                )

            rclpy.spin_once(
                self,
                timeout_sec=0.1,
            )

        raise RuntimeError(
            "ROS shutdown while waiting "
            "for AMCL pose."
        )

    def get_current_amcl_pose(
        self,
        refresh_sec=0.5,
        timeout_sec=5.0,
    ):
        """
        최신 AMCL pose 사용.

        1. 아직 한 번도 받은 적 없으면 기다린다.
        2. 이미 pose가 있으면 잠시 callback을 처리한다.
        3. 새 pose가 오면 최신 pose 사용.
        4. 새 pose가 안 와도 기존 최신 pose 사용.
        """

        if self.latest_amcl_pose is None:
            self.wait_for_amcl_pose(
                timeout_sec=timeout_sec
            )

        previous_count = (
            self.amcl_message_count
        )

        deadline = (
            time.monotonic()
            + refresh_sec
        )

        while (
            rclpy.ok()
            and time.monotonic() < deadline
        ):
            rclpy.spin_once(
                self,
                timeout_sec=0.1,
            )

            if (
                self.amcl_message_count
                > previous_count
            ):
                break

        if self.latest_amcl_pose is None:
            raise RuntimeError(
                "AMCL pose unavailable."
            )

        return self.latest_amcl_pose

    # ========================================================
    # Pose / Metrics
    # ========================================================

    @staticmethod
    def extract_pose_2d(msg):

        pose = msg.pose.pose

        return (
            pose.position.x,
            pose.position.y,
            quaternion_to_yaw(
                pose.orientation
            ),
        )

    def calculate_pose_metrics(
        self,
        expected_pose,
        amcl_msg=None,
    ):
        """
        expected pose와 AMCL pose 간
        position / heading error 계산.
        """

        if amcl_msg is None:
            amcl_msg = (
                self.get_current_amcl_pose()
            )

        actual_x, actual_y, actual_yaw = (
            self.extract_pose_2d(
                amcl_msg
            )
        )

        expected_x = float(
            expected_pose["x"]
        )

        expected_y = float(
            expected_pose["y"]
        )

        expected_yaw = float(
            expected_pose["yaw"]
        )

        pos_error_m = position_error(
            expected_x,
            expected_y,
            actual_x,
            actual_y,
        )

        heading_error_deg = math.degrees(
            heading_error_rad(
                expected_yaw,
                actual_yaw,
            )
        )

        return {
            "expected_x": expected_x,
            "expected_y": expected_y,
            "expected_yaw": expected_yaw,

            "amcl_x": actual_x,
            "amcl_y": actual_y,
            "amcl_yaw": actual_yaw,

            "position_error_m":
                pos_error_m,

            "position_error_cm":
                pos_error_m * 100.0,

            "heading_error_deg":
                heading_error_deg,

            "heading_error_abs_deg":
                abs(heading_error_deg),
        }

    # ========================================================
    # START Pose Check
    # ========================================================

    def check_start_pose(self):

        metrics = (
            self.calculate_pose_metrics(
                self.scenario["start"]
            )
        )

        check_cfg = (
            self.scenario.get(
                "start_check",
                {},
            )
        )

        position_limit_m = float(
            check_cfg.get(
                "position_tolerance_m",
                0.03,
            )
        )

        yaw_limit_deg = float(
            check_cfg.get(
                "yaw_tolerance_deg",
                3.0,
            )
        )

        passed = (
            metrics["position_error_m"]
            <= position_limit_m
            and
            metrics["heading_error_abs_deg"]
            <= yaw_limit_deg
        )

        metrics["passed"] = passed

        print()
        print("START Pose Check")
        print("-" * 46)

        print(
            "Expected : "
            f'x={metrics["expected_x"]:.4f}, '
            f'y={metrics["expected_y"]:.4f}, '
            f'yaw={metrics["expected_yaw"]:.4f}'
        )

        print(
            "AMCL     : "
            f'x={metrics["amcl_x"]:.4f}, '
            f'y={metrics["amcl_y"]:.4f}, '
            f'yaw={metrics["amcl_yaw"]:.4f}'
        )

        print()

        print(
            "Position difference : "
            f'{metrics["position_error_cm"]:.2f} cm'
        )

        print(
            "Heading difference  : "
            f'{metrics["heading_error_abs_deg"]:.2f} deg'
        )

        print(
            "Allowed position    : "
            f"{position_limit_m * 100.0:.2f} cm"
        )

        print(
            "Allowed heading     : "
            f"{yaw_limit_deg:.2f} deg"
        )

        print()

        if passed:
            print("✅ START pose OK")
        else:
            print("⚠ START pose mismatch")

        return metrics

    # ========================================================
    # Initial Localization
    # ========================================================

    def publish_initial_pose(
        self,
        pose_cfg,
    ):
        """
        RViz 2D Pose Estimate와 같은 역할을
        정확한 숫자 기반으로 수행한다.
        """

        print()
        print(
            "Publishing START pose "
            "to /initialpose..."
        )

        # ----------------------------------------------------
        # AMCL이 /initialpose를 구독 중인지 확인
        # ----------------------------------------------------

        deadline = (
            time.monotonic()
            + 3.0
        )

        while (
            rclpy.ok()
            and
            self.initialpose_pub
            .get_subscription_count()
            == 0
            and
            time.monotonic() < deadline
        ):
            rclpy.spin_once(
                self,
                timeout_sec=0.1,
            )

        if (
            self.initialpose_pub
            .get_subscription_count()
            == 0
        ):
            raise RuntimeError(
                "No subscriber detected on "
                f"{self.initialpose_topic}. "
                "Check AMCL."
            )

        msg = (
            PoseWithCovarianceStamped()
        )

        msg.header.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        msg.header.frame_id = "map"

        msg.pose.pose.position.x = float(
            pose_cfg["x"]
        )

        msg.pose.pose.position.y = float(
            pose_cfg["y"]
        )

        msg.pose.pose.position.z = 0.0

        quaternion = yaw_to_quaternion(
            float(
                pose_cfg["yaw"]
            )
        )

        msg.pose.pose.orientation.x = (
            quaternion["x"]
        )

        msg.pose.pose.orientation.y = (
            quaternion["y"]
        )

        msg.pose.pose.orientation.z = (
            quaternion["z"]
        )

        msg.pose.pose.orientation.w = (
            quaternion["w"]
        )

        # ----------------------------------------------------
        # Initial pose covariance
        # ----------------------------------------------------

        position_stddev_m = float(
            self.initial_localization_cfg.get(
                "position_stddev_m",
                0.05,
            )
        )

        yaw_stddev_deg = float(
            self.initial_localization_cfg.get(
                "yaw_stddev_deg",
                5.0,
            )
        )

        yaw_stddev_rad = math.radians(
            yaw_stddev_deg
        )

        covariance = [0.0] * 36

        # x
        covariance[0] = (
            position_stddev_m ** 2
        )

        # y
        covariance[7] = (
            position_stddev_m ** 2
        )

        # yaw
        covariance[35] = (
            yaw_stddev_rad ** 2
        )

        msg.pose.covariance = covariance

        previous_count = (
            self.amcl_message_count
        )

        self.initialpose_pub.publish(msg)

        # AMCL이 initialpose를 처리할 시간을 준다.
        settle_sec = float(
            self.initial_localization_cfg.get(
                "settle_sec",
                1.0,
            )
        )

        self.spin_for(
            settle_sec
        )

        # ----------------------------------------------------
        # AMCL response 확인
        # ----------------------------------------------------

        deadline = (
            time.monotonic()
            + 3.0
        )

        while (
            rclpy.ok()
            and time.monotonic() < deadline
        ):

            rclpy.spin_once(
                self,
                timeout_sec=0.1,
            )

            if (
                self.amcl_message_count
                > previous_count
            ):
                print(
                    "✅ AMCL responded after "
                    "/initialpose"
                )
                return

        # Latched pose가 이미 있다면 fatal로 보지 않는다.
        if self.latest_amcl_pose is not None:
            print(
                "⚠ No new AMCL message observed, "
                "but an AMCL pose is available."
            )
            return

        raise RuntimeError(
            "AMCL did not provide a pose "
            "after /initialpose."
        )

    def initial_localization_setup(self):

        print()
        print("=" * 52)
        print("Initial Localization")
        print("=" * 52)

        print()
        print(
            "로봇의 base_footprint 중심을 "
            "물리 START +에 맞추고,"
        )

        print(
            "로봇 방향도 START 방향에 "
            "맞춰주세요."
        )

        print()

        print(
            "[i] 저장된 START pose로 "
            "AMCL 초기화"
        )

        print(
            "[s] 현재 localization 그대로 사용"
        )

        print(
            "[q] 실험 종료"
        )

        while True:

            choice = input(
                "선택 [i/s/q] > "
            ).strip().lower()

            if choice == "q":
                raise KeyboardInterrupt

            if choice == "s":

                print()
                print(
                    "현재 AMCL localization을 "
                    "사용합니다."
                )

                # 여기에서 AMCL pose가 실제로
                # 구독 가능한지 미리 검증.
                self.wait_for_amcl_pose(
                    timeout_sec=5.0
                )

                return

            if choice == "i":

                input(
                    "로봇이 물리 START +에 "
                    "정확히 맞아 있으면 ENTER > "
                )

                self.publish_initial_pose(
                    self.scenario["start"]
                )

                # initialpose 후 최종적으로
                # AMCL pose가 존재하는지 보장.
                self.wait_for_amcl_pose(
                    timeout_sec=5.0
                )

                return

            print(
                "i, s 또는 q를 입력해주세요."
            )

    # ========================================================
    # controller_server parameters
    # ========================================================

    def wait_for_controller_server(self):

        self.get_logger().info(
            "Waiting for controller_server "
            "parameter service..."
        )

        available = (
            self.controller_params
            .wait_for_services(
                timeout_sec=10.0
            )
        )

        if not available:

            raise RuntimeError(
                "controller_server parameter "
                "service unavailable."
            )

    def get_goal_tolerances(self):

        future = (
            self.controller_params
            .get_parameters([
                self.PARAM_XY,
                self.PARAM_YAW,
            ])
        )

        rclpy.spin_until_future_complete(
            self,
            future,
            timeout_sec=5.0,
        )

        response = future.result()

        if response is None:
            raise RuntimeError(
                "Failed to read goal checker "
                "parameters."
            )

        if len(response.values) != 2:
            raise RuntimeError(
                "Unexpected parameter response."
            )

        for name, value in zip(
            [
                self.PARAM_XY,
                self.PARAM_YAW,
            ],
            response.values,
        ):

            if (
                value.type
                != ParameterType.PARAMETER_DOUBLE
            ):
                raise RuntimeError(
                    f"{name} is not "
                    "a DOUBLE parameter."
                )

        return (
            response.values[0].double_value,
            response.values[1].double_value,
        )

    def set_goal_tolerances(
        self,
        xy_tolerance,
        yaw_tolerance,
    ):

        params = [
            Parameter(
                self.PARAM_XY,
                Parameter.Type.DOUBLE,
                float(xy_tolerance),
            ),

            Parameter(
                self.PARAM_YAW,
                Parameter.Type.DOUBLE,
                float(yaw_tolerance),
            ),
        ]

        future = (
            self.controller_params
            .set_parameters(params)
        )

        rclpy.spin_until_future_complete(
            self,
            future,
            timeout_sec=5.0,
        )

        response = future.result()

        if response is None:
            raise RuntimeError(
                "Failed to set goal checker "
                "parameters."
            )

        if len(response.results) != 2:
            raise RuntimeError(
                "Unexpected parameter "
                "set response."
            )

        for result in response.results:

            if not result.successful:
                raise RuntimeError(
                    "Parameter update failed: "
                    f"{result.reason}"
                )

    def apply_goal_tolerances(
        self,
        xy_tolerance,
        yaw_tolerance,
        label,
    ):

        xy_tolerance = float(
            xy_tolerance
        )

        yaw_tolerance = float(
            yaw_tolerance
        )

        self.set_goal_tolerances(
            xy_tolerance,
            yaw_tolerance,
        )

        if self.execution_cfg.get(
            "verify_parameters",
            True,
        ):

            actual_xy, actual_yaw = (
                self.get_goal_tolerances()
            )

            valid = (
                math.isclose(
                    actual_xy,
                    xy_tolerance,
                    abs_tol=1e-6,
                )
                and
                math.isclose(
                    actual_yaw,
                    yaw_tolerance,
                    abs_tol=1e-6,
                )
            )

            if not valid:
                raise RuntimeError(
                    "Goal checker parameter "
                    "verification failed. "
                    f"Expected "
                    f"XY={xy_tolerance}, "
                    f"Yaw={yaw_tolerance}; "
                    f"Actual "
                    f"XY={actual_xy}, "
                    f"Yaw={actual_yaw}"
                )

        print()
        print(
            f"✅ {label} tolerance applied"
        )

        print(
            f"XY  : {xy_tolerance:.3f} m"
        )

        print(
            f"Yaw : {yaw_tolerance:.3f} rad"
        )

    # ========================================================
    # Nav2
    # ========================================================

    def wait_for_nav2(self):

        self.get_logger().info(
            "Waiting for NavigateToPose "
            "action server..."
        )

        available = (
            self.navigator.wait_for_server(
                timeout_sec=20.0
            )
        )

        if not available:
            raise RuntimeError(
                "NavigateToPose action "
                "server unavailable."
            )

    def build_navigation_goal(
        self,
        pose_cfg,
    ):

        goal = NavigateToPose.Goal()

        goal.pose.header.frame_id = "map"

        goal.pose.header.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        goal.pose.pose.position.x = float(
            pose_cfg["x"]
        )

        goal.pose.pose.position.y = float(
            pose_cfg["y"]
        )

        q = yaw_to_quaternion(
            float(
                pose_cfg["yaw"]
            )
        )

        goal.pose.pose.orientation.x = q["x"]
        goal.pose.pose.orientation.y = q["y"]
        goal.pose.pose.orientation.z = q["z"]
        goal.pose.pose.orientation.w = q["w"]

        return goal

    def navigate_to_pose(
        self,
        pose_cfg,
        label,
    ):

        goal = (
            self.build_navigation_goal(
                pose_cfg
            )
        )

        print()
        print(
            f"{label} navigation started..."
        )

        start_time = (
            time.monotonic()
        )

        send_future = (
            self.navigator
            .send_goal_async(goal)
        )

        rclpy.spin_until_future_complete(
            self,
            send_future,
        )

        goal_handle = (
            send_future.result()
        )

        if (
            goal_handle is None
            or not goal_handle.accepted
        ):

            print(
                f"⚠ {label} goal rejected"
            )

            return {
                "status": "REJECTED",
                "time_sec": 0.0,
            }

        result_future = (
            goal_handle
            .get_result_async()
        )

        rclpy.spin_until_future_complete(
            self,
            result_future,
        )

        elapsed = (
            time.monotonic()
            - start_time
        )

        response = (
            result_future.result()
        )

        if response is None:

            return {
                "status": "UNKNOWN",
                "time_sec": elapsed,
            }

        status_map = {
            GoalStatus.STATUS_SUCCEEDED:
                "SUCCEEDED",

            GoalStatus.STATUS_ABORTED:
                "ABORTED",

            GoalStatus.STATUS_CANCELED:
                "CANCELED",
        }

        status_text = status_map.get(
            response.status,
            f"STATUS_{response.status}",
        )

        print()
        print(
            f"{label} result : "
            f"{status_text}"
        )

        print(
            f"{label} time   : "
            f"{elapsed:.2f} sec"
        )

        return {
            "status": status_text,
            "time_sec": elapsed,
        }

    # ========================================================
    # START Ready / Recovery
    # ========================================================

    def return_tolerances(self):

        return (
            float(
                self.return_cfg.get(
                    "xy_goal_tolerance",
                    0.05,
                )
            ),
            float(
                self.return_cfg.get(
                    "yaw_goal_tolerance",
                    0.05,
                )
            ),
        )

    def ensure_start_ready(
        self,
        allow_nav_retry=True,
    ):

        while True:

            metrics = (
                self.check_start_pose()
            )

            if metrics["passed"]:
                return metrics

            print()
            print(
                "START 조건이 맞지 않습니다."
            )

            if allow_nav_retry:
                print(
                    "[r] Nav2로 START 다시 주행"
                )

            print(
                "[c] 현재 상태 다시 검사"
            )

            print(
                "[i] 로봇을 물리 START에 맞춘 뒤 "
                "AMCL 재초기화"
            )

            print(
                "[q] 실험 종료"
            )

            allowed = (
                {"r", "c", "i", "q"}
                if allow_nav_retry
                else {"c", "i", "q"}
            )

            choice = input(
                "선택 > "
            ).strip().lower()

            if choice not in allowed:
                print(
                    "올바른 옵션을 선택해주세요."
                )
                continue

            if choice == "q":
                raise KeyboardInterrupt

            if choice == "c":
                continue

            if choice == "r":

                return_xy, return_yaw = (
                    self.return_tolerances()
                )

                self.apply_goal_tolerances(
                    return_xy,
                    return_yaw,
                    label="RETURN",
                )

                self.navigate_to_pose(
                    self.scenario["start"],
                    label="RETURN TO START",
                )

                continue

            if choice == "i":

                print()
                print(
                    "주의: 실제 로봇이 START에 "
                    "있을 때만 재초기화합니다."
                )

                input(
                    "로봇을 물리 START +에 "
                    "맞춘 뒤 ENTER > "
                )

                self.publish_initial_pose(
                    self.scenario["start"]
                )

    def return_to_start(self):

        print()
        print("=" * 52)
        print("Return To START")
        print("=" * 52)

        input(
            "주변이 안전하면 START 자동 복귀를 "
            "시작합니다. ENTER > "
        )

        return_xy, return_yaw = (
            self.return_tolerances()
        )

        self.apply_goal_tolerances(
            return_xy,
            return_yaw,
            label="RETURN",
        )

        result = (
            self.navigate_to_pose(
                self.scenario["start"],
                label="RETURN TO START",
            )
        )

        if (
            result["status"]
            != "SUCCEEDED"
        ):
            print()
            print(
                "⚠ START 복귀가 정상 완료되지 "
                "않았습니다."
            )

        metrics = (
            self.ensure_start_ready(
                allow_nav_retry=True
            )
        )

        print()
        print(
            "✅ START 복귀 및 localization "
            "검증 완료"
        )

        return metrics

    # ========================================================
    # CSV
    # ========================================================

    def open_csv(self):

        timestamp = (
            datetime.now()
            .strftime("%Y%m%d_%H%M%S")
        )

        robot_name = (
            self.robot["name"]
        )

        scenario_name = (
            self.scenario[
                "scenario"
            ]["name"]
        )

        self.csv_path = (
            self.output_dir
            / (
                f"{timestamp}_"
                f"{robot_name}_"
                f"{scenario_name}_"
                "xy_tolerance_sweep.csv"
            )
        )

        self.csv_file = (
            self.csv_path.open(
                "w",
                newline="",
                encoding="utf-8",
            )
        )

        fieldnames = [
            "timestamp",

            "condition",
            "xy_goal_tolerance_m",
            "yaw_goal_tolerance_rad",
            "trial",

            "start_expected_x_m",
            "start_expected_y_m",
            "start_expected_yaw_rad",

            "start_amcl_x_m",
            "start_amcl_y_m",
            "start_amcl_yaw_rad",

            "start_position_error_cm",
            "start_heading_error_deg",

            "nav_result",
            "nav_time_sec",

            "goal_x_m",
            "goal_y_m",
            "goal_yaw_rad",

            "final_amcl_x_m",
            "final_amcl_y_m",
            "final_amcl_yaw_rad",

            "estimated_position_error_cm",
            "estimated_heading_error_deg",

            "actual_position_error_cm",
            "actual_heading_error_deg",
        ]

        self.csv_writer = (
            csv.DictWriter(
                self.csv_file,
                fieldnames=fieldnames,
            )
        )

        self.csv_writer.writeheader()
        self.csv_file.flush()

        print()
        print(
            f"CSV: {self.csv_path}"
        )

    def save_trial(
        self,
        condition,
        xy_tolerance,
        yaw_tolerance,
        trial,
        start_metrics,
        nav_result,
        goal_metrics,
        actual_position_error_cm,
        actual_heading_error_deg,
    ):

        goal = (
            self.scenario["goal"]
        )

        self.csv_writer.writerow({

            "timestamp":
                datetime.now().isoformat(
                    timespec="seconds"
                ),

            "condition":
                condition,

            "xy_goal_tolerance_m":
                xy_tolerance,

            "yaw_goal_tolerance_rad":
                yaw_tolerance,

            "trial":
                trial,

            "start_expected_x_m":
                start_metrics["expected_x"],

            "start_expected_y_m":
                start_metrics["expected_y"],

            "start_expected_yaw_rad":
                start_metrics["expected_yaw"],

            "start_amcl_x_m":
                start_metrics["amcl_x"],

            "start_amcl_y_m":
                start_metrics["amcl_y"],

            "start_amcl_yaw_rad":
                start_metrics["amcl_yaw"],

            "start_position_error_cm":
                start_metrics[
                    "position_error_cm"
                ],

            "start_heading_error_deg":
                start_metrics[
                    "heading_error_deg"
                ],

            "nav_result":
                nav_result["status"],

            "nav_time_sec":
                f'{nav_result["time_sec"]:.3f}',

            "goal_x_m":
                goal["x"],

            "goal_y_m":
                goal["y"],

            "goal_yaw_rad":
                goal["yaw"],

            "final_amcl_x_m":
                goal_metrics["amcl_x"],

            "final_amcl_y_m":
                goal_metrics["amcl_y"],

            "final_amcl_yaw_rad":
                goal_metrics["amcl_yaw"],

            "estimated_position_error_cm":
                goal_metrics[
                    "position_error_cm"
                ],

            "estimated_heading_error_deg":
                goal_metrics[
                    "heading_error_deg"
                ],

            "actual_position_error_cm":
                actual_position_error_cm,

            "actual_heading_error_deg":
                (
                    ""
                    if actual_heading_error_deg
                    is None
                    else actual_heading_error_deg
                ),
        })

        # Trial 결과는 복귀 전에 즉시 저장
        self.csv_file.flush()

    # ========================================================
    # UI
    # ========================================================

    def print_trial_header(
        self,
        condition_index,
        condition_count,
        label,
        xy_tolerance,
        yaw_tolerance,
        trial,
        trial_count,
    ):

        scenario = (
            self.scenario["scenario"]
        )

        print()
        print("=" * 52)
        print(
            "XY Goal Tolerance Experiment"
        )
        print()

        print(
            f'Robot    : '
            f'{self.robot["name"]}'
        )

        print(
            f'Scenario : '
            f'{scenario["name"]}'
        )

        print(
            f'Distance : '
            f'{float(scenario["distance_m"]):.2f} m'
        )

        print()

        print(
            f"Condition "
            f"{condition_index} / "
            f"{condition_count}"
        )

        print(
            f"Label         : {label}"
        )

        print(
            f"XY tolerance  : "
            f"{xy_tolerance:.3f} m"
        )

        print(
            f"Yaw tolerance : "
            f"{yaw_tolerance:.3f} rad"
        )

        print()

        print(
            f"Trial {trial} / {trial_count}"
        )

        print("=" * 52)

    # ========================================================
    # Config helpers
    # ========================================================

    def get_trials(self):

        if self.trials_override is not None:

            trials = int(
                self.trials_override
            )

        else:

            trials = int(
                self.experiment_cfg[
                    "trials_per_condition"
                ]
            )

        if trials <= 0:
            raise ValueError(
                "Trial count must be > 0."
            )

        return trials

    def get_conditions(self):

        conditions = list(
            self.goal_checker_cfg[
                "xy_goal_tolerances"
            ]
        )

        if self.only_condition is not None:

            conditions = [
                condition
                for condition in conditions
                if (
                    condition["label"]
                    == self.only_condition
                )
            ]

            if not conditions:
                raise ValueError(
                    "Condition not found: "
                    f"{self.only_condition}"
                )

        if not conditions:
            raise ValueError(
                "No XY tolerance conditions."
            )

        return conditions

    # ========================================================
    # Main experiment
    # ========================================================

    def run(self):

        # ----------------------------------------------------
        # 1. System readiness
        # ----------------------------------------------------

        self.wait_for_controller_server()
        self.wait_for_nav2()

        # ----------------------------------------------------
        # 2. Original Nav2 parameter backup
        # ----------------------------------------------------

        (
            self.original_xy_tolerance,
            self.original_yaw_tolerance,
        ) = (
            self.get_goal_tolerances()
        )

        print()
        print(
            "Original Nav2 parameters"
        )

        print(
            "XY  = "
            f"{self.original_xy_tolerance:.3f} m"
        )

        print(
            "Yaw = "
            f"{self.original_yaw_tolerance:.3f} rad"
        )

        trials = (
            self.get_trials()
        )

        conditions = (
            self.get_conditions()
        )

        yaw_tolerance = float(
            self.goal_checker_cfg[
                "yaw_goal_tolerance"
            ]
        )

        auto_return = bool(
            self.return_cfg.get(
                "enabled",
                True,
            )
        )

        return_after_final = bool(
            self.return_cfg.get(
                "return_after_final_trial",
                False,
            )
        )

        # ----------------------------------------------------
        # 3. CSV
        # ----------------------------------------------------

        self.open_csv()

        try:

            # ------------------------------------------------
            # 4. Initial localization
            # ------------------------------------------------

            self.initial_localization_setup()

            # ------------------------------------------------
            # 5. Experiment conditions
            # ------------------------------------------------

            for (
                condition_index,
                condition,
            ) in enumerate(
                conditions,
                start=1,
            ):

                label = (
                    condition["label"]
                )

                xy_tolerance = float(
                    condition["value"]
                )

                # --------------------------------------------
                # Trial loop
                # --------------------------------------------

                for trial in range(
                    1,
                    trials + 1,
                ):

                    self.print_trial_header(
                        condition_index,
                        len(conditions),
                        label,
                        xy_tolerance,
                        yaw_tolerance,
                        trial,
                        trials,
                    )

                    # ========================================
                    # START validation
                    # ========================================

                    print()
                    print(
                        "물리 START +와 "
                        "base_footprint 정렬을 "
                        "확인해주세요."
                    )

                    input(
                        "START 검증하려면 ENTER > "
                    )

                    start_metrics = (
                        self.ensure_start_ready(
                            allow_nav_retry=True
                        )
                    )

                    # ========================================
                    # Experiment tolerance
                    # ========================================

                    self.apply_goal_tolerances(
                        xy_tolerance,
                        yaw_tolerance,
                        label="EXPERIMENT",
                    )

                    # ========================================
                    # GOAL navigation
                    # ========================================

                    print()

                    input(
                        "GOAL 주행을 시작하려면 "
                        "ENTER > "
                    )

                    nav_result = (
                        self.navigate_to_pose(
                            self.scenario["goal"],
                            label="GOAL",
                        )
                    )

                    # ========================================
                    # Final AMCL metrics
                    # ========================================

                    goal_metrics = (
                        self.calculate_pose_metrics(
                            self.scenario["goal"]
                        )
                    )

                    print()

                    print(
                        "AMCL estimated "
                        "position error : "
                        f'{goal_metrics["position_error_cm"]:.2f} cm'
                    )

                    print(
                        "AMCL estimated "
                        "heading error  : "
                        f'{goal_metrics["heading_error_deg"]:.2f} deg'
                    )

                    # ========================================
                    # Physical Ground Truth
                    # ========================================

                    print()
                    print(
                        "GOAL + 중심 ↔ "
                        "실제 base_footprint 중심의 "
                        "직선거리를 측정하세요."
                    )

                    actual_position_error_cm = (
                        required_nonnegative_float(
                            "실측 직선거리 [cm] > "
                        )
                    )

                    actual_heading_error_deg = (
                        optional_float(
                            "실측 heading error [deg] "
                            "(생략=ENTER) > "
                        )
                    )

                    # ========================================
                    # Save result
                    # ========================================

                    self.save_trial(
                        condition=label,
                        xy_tolerance=xy_tolerance,
                        yaw_tolerance=yaw_tolerance,
                        trial=trial,
                        start_metrics=start_metrics,
                        nav_result=nav_result,
                        goal_metrics=goal_metrics,
                        actual_position_error_cm=(
                            actual_position_error_cm
                        ),
                        actual_heading_error_deg=(
                            actual_heading_error_deg
                        ),
                    )

                    print()
                    print(
                        f"✅ Trial {trial} saved"
                    )

                    print(
                        "Physical position error : "
                        f"{actual_position_error_cm:.2f} cm"
                    )

                    # ========================================
                    # START Auto Return
                    # ========================================

                    is_final_trial = (
                        condition_index
                        == len(conditions)
                        and
                        trial == trials
                    )

                    should_return = (
                        auto_return
                        and
                        (
                            not is_final_trial
                            or return_after_final
                        )
                    )

                    if should_return:
                        self.return_to_start()

            # ------------------------------------------------
            # Complete
            # ------------------------------------------------

            print()
            print("=" * 52)

            print(
                "✅ XY Goal Tolerance "
                "Experiment Complete"
            )

            print(
                f"CSV: {self.csv_path}"
            )

            print("=" * 52)

        finally:

            # ------------------------------------------------
            # Restore original Nav2 parameters
            # ------------------------------------------------

            if (
                self.execution_cfg.get(
                    "restore_original_parameters",
                    True,
                )
                and
                self.original_xy_tolerance
                is not None
                and
                self.original_yaw_tolerance
                is not None
            ):

                print()
                print(
                    "Restoring original "
                    "Nav2 parameters..."
                )

                try:

                    self.set_goal_tolerances(
                        self.original_xy_tolerance,
                        self.original_yaw_tolerance,
                    )

                    print(
                        "✅ Original parameters restored"
                    )

                    print(
                        "XY  = "
                        f"{self.original_xy_tolerance:.3f} m"
                    )

                    print(
                        "Yaw = "
                        f"{self.original_yaw_tolerance:.3f} rad"
                    )

                except Exception as error:

                    print(
                        "⚠ Failed to restore "
                        "Nav2 parameters: "
                        f"{error}"
                    )

            # ------------------------------------------------
            # Close CSV
            # ------------------------------------------------

            if self.csv_file is not None:
                self.csv_file.flush()
                self.csv_file.close()


# ============================================================
# CLI
# ============================================================


def parse_args():

    parser = argparse.ArgumentParser(
        description=(
            "Goal Pose Accuracy "
            "XY tolerance sweep experiment"
        )
    )

    parser.add_argument(
        "--robot-config",
        default=(
            "~/dev/Logistics_AMR/"
            "src/logistics_amr_experiments/"
            "config/robots/"
            "turtle2.yaml"
        ),
    )

    parser.add_argument(
        "--scenario-config",
        default=(
            "~/dev/Logistics_AMR/"
            "src/logistics_amr_experiments/"
            "config/scenarios/"
            "straight.yaml"
        ),
    )

    parser.add_argument(
        "--experiment-config",
        default=(
            "~/dev/Logistics_AMR/"
            "src/logistics_amr_experiments/"
            "config/experiments/"
            "xy_tolerance_sweep.yaml"
        ),
    )

    parser.add_argument(
        "--output-dir",
        default=(
            "~/dev/Logistics_AMR/"
            "experiment_data/"
            "goal_pose_accuracy"
        ),
    )

    parser.add_argument(
        "--trials",
        type=int,
        default=None,
        help=(
            "Override trials_per_condition. "
            "Example: --trials 2"
        ),
    )

    parser.add_argument(
        "--only-condition",
        default=None,
        help=(
            "Run only one condition. "
            "Example: baseline_25cm"
        ),
    )

    return parser.parse_args()


# ============================================================
# Main
# ============================================================


def main():

    args = parse_args()

    robot_config = load_yaml(
        args.robot_config
    )

    scenario_config = load_yaml(
        args.scenario_config
    )

    experiment_config = load_yaml(
        args.experiment_config
    )

    rclpy.init()

    node = GoalPoseAccuracyExperiment(
        robot_config=robot_config,
        scenario_config=scenario_config,
        experiment_config=experiment_config,
        output_dir=args.output_dir,
        trials_override=args.trials,
        only_condition=args.only_condition,
    )

    try:

        node.run()

    except KeyboardInterrupt:

        print()
        print(
            "Experiment interrupted "
            "by operator."
        )

    except Exception as error:

        print()
        print("Experiment error:")
        print(error)

    finally:

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()