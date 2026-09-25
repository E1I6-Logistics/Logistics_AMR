"""Interactive staging pose registration workflow."""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from staging_pose_manager.motion import MotionError, OdomMotionController
from staging_pose_manager.paths import records_dir
from staging_pose_manager.pose_statistics import calculate_statistics
from staging_pose_manager.preflight import (
    check_localization,
    print_report,
    run_checks,
)
from staging_pose_manager.session_manager import SessionManager, validate_station_id
from tf2_ros import Buffer, TransformException, TransformListener


TARGET_FRAME = "map"
SOURCE_FRAME = "base_footprint"
RECORD_DIR = records_dir()


class UserQuit(Exception):
    """Signal an operator-requested, resumable exit."""


class SamplingError(RuntimeError):
    """Signal that a complete measurement could not be collected."""


def quaternion_to_yaw(x, y, z, w):
    """Convert a quaternion to planar yaw."""
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


class StagingPoseRecorder(Node):
    """ROS node that reads the localized robot pose from TF."""

    def __init__(self):
        super().__init__("staging_pose_recorder")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def get_pose(self):
        """Read map to base_footprint without retaining failed samples."""
        transform = self.tf_buffer.lookup_transform(
            TARGET_FRAME,
            SOURCE_FRAME,
            Time(),
        )
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        stamp = transform.header.stamp
        return {
            "stamp": f"{stamp.sec}.{stamp.nanosec:09d}",
            "x": translation.x,
            "y": translation.y,
            "yaw": quaternion_to_yaw(
                rotation.x, rotation.y, rotation.z, rotation.w
            ),
        }


def collect_samples(node, sample_count, interval):
    """Collect an all-or-nothing set of TF samples."""
    samples = []
    last_success = time.monotonic()
    next_sample = time.monotonic()
    missing_timeout = max(5.0, sample_count * interval * 4.0)
    try:
        while rclpy.ok() and len(samples) < sample_count:
            remaining = next_sample - time.monotonic()
            rclpy.spin_once(node, timeout_sec=max(0.0, min(0.02, remaining)))
            if time.monotonic() < next_sample:
                continue
            try:
                sample = node.get_pose()
            except TransformException:
                if time.monotonic() - last_success > missing_timeout:
                    raise SamplingError(
                        "map -> base_footprint TF가 너무 오래 없습니다."
                    )
                next_sample = time.monotonic() + min(interval, 0.05)
                continue
            samples.append(sample)
            last_success = time.monotonic()
            next_sample = last_success + interval
            print(f"\rSamples: {len(samples):>3} / {sample_count}", end="", flush=True)
    except KeyboardInterrupt:
        print("\n샘플링이 중단되었습니다. partial samples는 폐기합니다.")
        raise

    print()
    if len(samples) != sample_count:
        raise SamplingError("ROS 종료로 샘플링이 완료되지 않았습니다.")
    return samples, calculate_statistics(samples)


def run_preflight(node):
    """Repeat preflight until required conditions pass or the user quits."""
    while True:
        banner("PREFLIGHT")
        print("실행 순서: Bringup -> Nav2 -> Teleop -> register_pose")
        results = run_checks(node, node.tf_buffer)
        ready = print_report(results)
        if ready:
            print("\nSYSTEM STATUS: READY")
            return
        print("\n부족한 노드/토픽/TF를 시작한 뒤 재검사하세요.")
        if prompt_choice("[r] retry  [q] quit: ", {"r", "q"}) == "q":
            raise UserQuit


def localization_gate(node, resumed=False):
    """Require TF availability and operator confirmation of scan alignment."""
    banner("RESUME LOCALIZATION CHECK" if resumed else "INITIAL LOCALIZATION")
    if resumed:
        answer = prompt_choice(
            "중단 중 로봇을 들거나 수동으로 옮겼습니까? [y/n/q]: ",
            {"y", "n", "q"},
        )
        if answer == "q":
            raise UserQuit
        if answer == "y":
            print("RViz의 2D Pose Estimate를 반드시 다시 수행하세요.")

    while True:
        print("\n1. 바닥 center/heading 마킹에 로봇을 정렬합니다.")
        print("2. 필요하면 RViz에서 2D Pose Estimate를 설정합니다.")
        print("3. LiDAR scan과 map이 정확히 겹치는지 확인합니다.")
        input("완료 후 Enter를 눌러 TF를 확인하세요...")
        for _ in range(10):
            rclpy.spin_once(node, timeout_sec=0.1)
        if not check_localization(node.tf_buffer):
            print("[FAIL] map -> base_footprint TF가 없습니다.")
            if prompt_choice("[r] retry  [q] quit: ", {"r", "q"}) == "q":
                raise UserQuit
            continue
        answer = prompt_choice(
            "[PASS] TF 확인. LiDAR-map 정합도 정상입니까? [y/r/q]: ",
            {"y", "r", "q"},
        )
        if answer == "y":
            return
        if answer == "q":
            raise UserQuit


def select_session():
    """Offer the latest incomplete session before creating a new one."""
    incomplete = SessionManager.find_incomplete(RECORD_DIR)
    if incomplete:
        latest = incomplete[0]
        state = latest.state
        banner("INCOMPLETE SESSION DETECTED")
        print(f"Station: {state['station_id']}")
        print(f"Session: {state['session_id']}")
        print(f"Completed: {len(latest.completed_trials)} / {state['trial_count']}")
        print(f"Baseline: {'COMPLETED' if state.get('baseline') else 'NOT COMPLETED'}")
        current = state.get("current_trial")
        if current:
            print(
                f"Interrupted Trial {current['trial']}: "
                f"{describe_perturbation(current['perturbation'])}"
            )
        choice = prompt_choice(
            "[r] resume latest  [n] new session  [q] quit: ",
            {"r", "n", "q"},
        )
        if choice == "r":
            return latest, True
        if choice == "q":
            raise UserQuit
    return create_session(), False


def create_session(station_id=None):
    """Prompt for and create a fresh session."""
    banner("NEW REGISTRATION SESSION")
    while station_id is None:
        try:
            station_id = validate_station_id(input("Station ID: "))
        except ValueError as exc:
            print(exc)
            station_id = None
    trial_count = prompt_number("Trial count [5]: ", 5, int)
    sample_count = prompt_number("Samples per measurement [30]: ", 30, int)
    interval = prompt_number("Sample interval seconds [0.05]: ", 0.05, float)
    return SessionManager.create(
        RECORD_DIR, station_id, trial_count, sample_count, interval
    )


def run_baseline(node, session):
    """Collect and save a complete reference-only baseline."""
    if session.state.get("baseline"):
        return
    while True:
        banner("BASELINE MEASUREMENT")
        print("Baseline은 raw samples로 저장되지만 최종 통계에서는 제외됩니다.")
        choice = input("정렬과 정지를 확인한 뒤 Enter ([q] quit): ").strip().lower()
        if choice == "q":
            raise UserQuit
        print("map -> base_footprint sampling...")
        samples, result = collect_samples(
            node,
            session.state["sample_count"],
            session.state["sample_interval"],
        )
        session.save_baseline(samples, result)
        print_result(result)
        choice = prompt_choice("[c] continue  [r] repeat baseline  [q] quit: ", {"c", "r", "q"})
        if choice == "c":
            return
        if choice == "q":
            raise UserQuit


def run_trials(node, session, motion):
    """Run every configured trial, revisiting deferred trials at the end."""
    while not session.is_complete:
        trial = session.next_trial_number()
        previous = None
        current = session.state.get("current_trial")
        if current and int(current["trial"]) == trial:
            previous = current["perturbation"]

        banner(f"TRIAL {trial} / {session.state['trial_count']}")
        perturbation = choose_perturbation(previous)
        if perturbation is None:
            session.defer_trial(trial)
            print(f"Trial {trial}을 뒤로 미뤘습니다.")
            continue

        session.start_trial(trial, perturbation)
        while True:
            action = prepare_and_align(motion, perturbation)
            if action == "edit":
                perturbation = choose_perturbation(None)
                if perturbation is None:
                    session.defer_trial(trial)
                    break
                session.start_trial(trial, perturbation)
                continue
            print("map -> base_footprint sampling...")
            samples, result = collect_samples(
                node,
                session.state["sample_count"],
                session.state["sample_interval"],
            )
            session.complete_trial(trial, perturbation, samples, result)
            print_result(result)
            print(
                f"Trial {trial} saved immediately. "
                f"Progress: {len(session.completed_trials)} / {session.state['trial_count']}"
            )
            break


def choose_perturbation(previous):
    """Choose an automatic or manual perturbation condition."""
    if previous:
        print(f"Previous attempt: {describe_perturbation(previous)}")
        choice = prompt_choice(
            "[r] retry previous  [e] edit  [q] quit: ", {"r", "e", "q"}
        )
        if choice == "r":
            return previous
        if choice == "q":
            raise UserQuit

    print("[f] forward  [b] backward  [cw] clockwise  [ccw] counter-clockwise")
    print("[i] incoming(manual)  [c] custom(manual)  [s] defer  [q] quit")
    choice = prompt_choice("Select: ", {"f", "b", "cw", "ccw", "i", "c", "s", "q"})
    if choice == "q":
        raise UserQuit
    if choice == "s":
        return None
    if choice in {"f", "b"}:
        distance = prompt_number("Distance [m] [0.50]: ", 0.50, float)
        return {
            "type": "forward" if choice == "f" else "backward",
            "target_value": distance,
            "target_unit": "m",
            "description": "",
        }
    if choice in {"cw", "ccw"}:
        angle = prompt_number("Angle [deg] [60]: ", 60.0, float)
        return {
            "type": "clockwise" if choice == "cw" else "counter_clockwise",
            "target_value": angle,
            "target_unit": "deg",
            "description": "",
        }
    if choice == "i":
        return {
            "type": "incoming",
            "description": input("Incoming approach label (optional): ").strip(),
        }
    description = input("Custom movement description: ").strip()
    if not description:
        description = "custom"
    return {"type": "custom", "description": description}


def prepare_and_align(motion, perturbation):
    """Execute a perturbation and wait for manual physical-mark alignment."""
    kind = perturbation["type"]
    banner(describe_perturbation(perturbation))
    automatic = kind in {"forward", "backward", "clockwise", "counter_clockwise"}
    if automatic:
        print("자동 이동 중 Teleop 조작부에서 손을 떼고 주변 안전을 확인하세요.")
        choice = input("Enter=execute, [e] edit, [q] quit: ").strip().lower()
        if choice == "e":
            return "edit"
        if choice == "q":
            raise UserQuit
        execute_automatic(motion, perturbation)
    else:
        print("Teleop으로 지정한 이동/접근을 수동 수행하세요.")

    while True:
        print("\nTeleop으로 바닥 center + heading 마킹에 최종 정렬하세요.")
        print("절대 odom 좌표는 최종 원위치 판정에 사용하지 않습니다.")
        options = "Enter=sample, [e] edit, [q] quit"
        if automatic:
            options += ", [r] repeat perturbation"
        choice = input(options + ": ").strip().lower()
        if choice == "":
            return "sample"
        if choice == "e":
            return "edit"
        if choice == "q":
            raise UserQuit
        if choice == "r" and automatic:
            execute_automatic(motion, perturbation)
            continue
        print("올바른 값을 입력하세요.")


def execute_automatic(motion, perturbation):
    """Execute one odom-relative out-and-back motion."""
    kind = perturbation["type"]
    value = float(perturbation["target_value"])
    print(
        f"왕복 각 구간 사이에 {motion.settle_time:.1f}초간 "
        "0속도를 유지합니다."
    )
    try:
        if kind in {"forward", "backward"}:
            sign = 1.0 if kind == "forward" else -1.0
            motion.perturb_translation(value, sign)
        else:
            sign = -1.0 if kind == "clockwise" else 1.0
            motion.perturb_rotation(math.radians(value), sign)
    except (KeyboardInterrupt, MotionError):
        motion.stop()
        raise
    print("자동 왕복 perturbation 완료. 정확한 복귀는 보장하지 않습니다.")


def print_summary(session):
    """Finalize CSV output and print final trial-only statistics."""
    result = session.finalize()
    banner("REGISTRATION SUMMARY")
    print(f"Station: {session.state['station_id']}")
    print(f"Session: {session.state['session_id']}")
    print_result(result, sample_jitter=False)
    print("Baseline은 저장됐지만 위 최종 통계에서 제외됐습니다.")
    print(f"Saved: {session.session_dir}")


def print_result(result, sample_jitter=True):
    """Print pose mean and standard deviation."""
    mean = result["mean"]
    std = result["std"]
    print(f"mean x/y/yaw: {mean['x']:.6f} m, {mean['y']:.6f} m, "
          f"{mean['yaw']:.6f} rad ({math.degrees(mean['yaw']):.3f} deg)")
    label = "sample jitter" if sample_jitter else "trial repeatability"
    print(f"{label}: x={std['x'] * 1000:.3f} mm, y={std['y'] * 1000:.3f} mm, "
          f"yaw={math.degrees(std['yaw']):.4f} deg")


def describe_perturbation(perturbation):
    """Return a concise operator-facing condition description."""
    value = perturbation.get("target_value")
    if value != "" and value is not None:
        return f"{perturbation['type']} {value:g} {perturbation.get('target_unit', '')}"
    description = perturbation.get("description", "")
    return f"{perturbation['type']} {description}".strip()


def prompt_choice(prompt, choices):
    """Read a normalized choice from a fixed set."""
    while True:
        value = input(prompt).strip().lower()
        if value in choices:
            return value
        print(f"Choose one of: {', '.join(sorted(choices))}")


def prompt_number(prompt, default, converter):
    """Read a positive number with a default."""
    while True:
        raw = input(prompt).strip()
        try:
            value = converter(raw) if raw else default
            if value <= 0:
                raise ValueError
            return value
        except ValueError:
            print("0보다 큰 숫자를 입력하세요.")


def banner(title):
    """Print a simple CLI section banner."""
    print("\n" + "=" * 64)
    print(title)
    print("=" * 64)


def next_action(session):
    """Choose another station, repeat this station, or quit."""
    choice = prompt_choice(
        "\n[n] next staging  [r] repeat this station  [q] quit: ",
        {"n", "r", "q"},
    )
    if choice == "q":
        raise UserQuit
    if choice == "r":
        return create_session(session.state["station_id"]), False
    banner("MOVE TO NEXT STAGING")
    print("Bringup/Nav2/Teleop/AMCL을 유지하고 Teleop으로 이동하세요.")
    print("로봇을 들었거나 localization이 이상할 때만 2D Pose Estimate를 다시 하세요.")
    if input("다음 위치 도착 후 Enter ([q] quit): ").strip().lower() == "q":
        raise UserQuit
    return create_session(), False


def main(args=None):
    """Run the complete interactive registration application."""
    rclpy.init(args=args)
    node = StagingPoseRecorder()
    active_session = None
    motion = None
    try:
        banner("STAGING POSE REGISTRATION")
        run_preflight(node)
        motion = OdomMotionController(node)
        active_session, resumed = select_session()
        while True:
            localization_gate(node, resumed=resumed)
            run_baseline(node, active_session)
            run_trials(node, active_session, motion)
            print_summary(active_session)
            active_session, resumed = next_action(active_session)
    except UserQuit:
        if active_session and not active_session.is_complete:
            print("\n완료된 Baseline/Trial은 저장됐습니다. 다음 실행에서 resume할 수 있습니다.")
            print(f"Session: {active_session.session_dir}")
        print("종료합니다.")
    except KeyboardInterrupt:
        if motion:
            motion.stop()
        print("\n중단 감지. partial samples는 폐기했으며 완료된 데이터만 유지합니다.")
        if active_session:
            print(f"Session: {active_session.session_dir}")
    except (MotionError, SamplingError) as exc:
        if motion:
            motion.stop()
        print(f"\nERROR: {exc}")
        print("현재 측정의 partial samples는 저장하지 않았습니다. 다시 실행해 resume하세요.")
    finally:
        if motion:
            motion.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
