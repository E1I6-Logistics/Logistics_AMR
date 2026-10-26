"""
marker_localization 단위 테스트.

두 종류가 있습니다.

1) 손으로 답을 계산할 수 있는 케이스 (test_hand_*)
   -> 좌표계 '규약'이 맞는지 검증합니다. 부호가 뒤집혀 있으면 여기서 걸립니다.
      시뮬레이터로만 테스트하면 규약이 통째로 틀려도 통과하므로 이 케이스가 핵심입니다.

2) 왕복 테스트 (test_roundtrip_*)
   -> 임의의 자세/기울기 조합에서 수식 합성이 맞는지 검증합니다.

실행:
    python3 test_marker_localization.py
    또는
    pytest test_marker_localization.py -v
"""

import math
import numpy as np

from logitle_aruco_tools.logitle_marker_localization import (
    rodrigues, rotation_to_rvec, rpy_to_matrix, matrix_to_rpy,
    make_T, invert, wall_marker_T, camera_mount_T,
    estimate_base_pose, simulate_observation, map_odom_correction,
    xy_yaw, R_FLU_TO_OPTICAL,
)

PI = math.pi


def assert_pose(T, x, y, yaw_deg, z=0.0, tol=1e-9):
    gx, gy, gyaw = xy_yaw(T)
    gz = float(T[2, 3])
    assert abs(gx - x) < tol, f"x: {gx} != {x}"
    assert abs(gy - y) < tol, f"y: {gy} != {y}"
    assert abs(gz - z) < tol, f"z: {gz} != {z}"
    dyaw = (math.degrees(gyaw) - yaw_deg + 180.0) % 360.0 - 180.0
    assert abs(dyaw) < 1e-6, f"yaw: {math.degrees(gyaw)} != {yaw_deg}"


# ==========================================================================
# 1. 기본 수학
# ==========================================================================

def test_rodrigues_matches_opencv():
    """cv2 가 있으면 Rodrigues 구현이 OpenCV 와 일치하는지 확인."""
    try:
        import cv2
    except ImportError:
        return  # OpenCV 없으면 건너뜀
    rng = np.random.default_rng(0)
    for _ in range(50):
        rvec = rng.normal(size=3) * 2.0
        assert np.allclose(rodrigues(rvec), cv2.Rodrigues(rvec)[0], atol=1e-12)


def test_rodrigues_roundtrip():
    rng = np.random.default_rng(1)
    for _ in range(200):
        axis = rng.normal(size=3)
        axis /= np.linalg.norm(axis)
        theta = rng.uniform(0.0, PI - 1e-6)
        rvec = axis * theta
        assert np.allclose(rodrigues(rotation_to_rvec(rodrigues(rvec))),
                           rodrigues(rvec), atol=1e-9)


def test_rodrigues_near_pi():
    """180도 부근은 일반식이 불안정하므로 별도 검증."""
    for axis in [(1, 0, 0), (0, 1, 0), (0, 0, 1), (1, 1, 0), (1, 1, 1)]:
        a = np.array(axis, dtype=float)
        a /= np.linalg.norm(a)
        R = rodrigues(a * PI)
        assert np.allclose(rodrigues(rotation_to_rvec(R)), R, atol=1e-12)
    # 180도 '직전'이 수치적으로 가장 까다로운 구간
    for eps in [1e-3, 1e-5, 1e-7, 1e-9]:
        a = np.array([0.4, -0.7, 0.6])
        a /= np.linalg.norm(a)
        R = rodrigues(a * (PI - eps))
        assert np.allclose(rodrigues(rotation_to_rvec(R)), R, atol=1e-12), f"eps={eps}"


def test_invert():
    rng = np.random.default_rng(2)
    for _ in range(50):
        T = make_T(rpy_to_matrix(*rng.uniform(-PI, PI, 3)), rng.uniform(-3, 3, 3))
        assert np.allclose(T @ invert(T), np.eye(4), atol=1e-12)
        assert np.allclose(invert(T) @ T, np.eye(4), atol=1e-12)


def test_rpy_roundtrip():
    rng = np.random.default_rng(3)
    for _ in range(200):
        roll = rng.uniform(-PI + 0.1, PI - 0.1)
        pitch = rng.uniform(-PI / 2 + 0.1, PI / 2 - 0.1)
        yaw = rng.uniform(-PI + 0.1, PI - 0.1)
        r2, p2, y2 = matrix_to_rpy(rpy_to_matrix(roll, pitch, yaw))
        assert np.allclose([r2, p2, y2], [roll, pitch, yaw], atol=1e-9)


# ==========================================================================
# 2. 입력 구성 함수의 규약
# ==========================================================================

def test_wall_marker_axes():
    """마커 z축은 벽 바깥, y축은 위를 향해야 한다."""
    T = wall_marker_T(2.0, 0.0, 0.5, yaw_deg=180.0)
    x_ax, y_ax, z_ax = T[:3, 0], T[:3, 1], T[:3, 2]
    assert np.allclose(z_ax, [-1, 0, 0], atol=1e-12), "법선이 -x를 향해야 함"
    assert np.allclose(y_ax, [0, 0, 1], atol=1e-12), "마커 위쪽이 맵 +z"
    assert np.allclose(x_ax, [0, -1, 0], atol=1e-12)
    assert np.allclose(np.cross(x_ax, y_ax), z_ax, atol=1e-12), "오른손 좌표계"
    assert abs(np.linalg.det(T[:3, :3]) - 1.0) < 1e-12


def test_wall_marker_yaw_directions():
    for yaw_deg, normal in [(0, [1, 0, 0]), (90, [0, 1, 0]),
                            (180, [-1, 0, 0]), (-90, [0, -1, 0])]:
        T = wall_marker_T(0, 0, 0, yaw_deg=yaw_deg)
        assert np.allclose(T[:3, 2], normal, atol=1e-12), f"yaw={yaw_deg}"


def test_camera_mount_level():
    """수평 장착 시 광축(z_opt)은 로봇 전방, y_opt는 아래를 향해야 한다."""
    T = camera_mount_T(0.05, 0.0, 0.10)
    assert np.allclose(T[:3, :3], R_FLU_TO_OPTICAL, atol=1e-12)
    assert np.allclose(T[:3, 2], [1, 0, 0], atol=1e-12), "광축 = 전방"
    assert np.allclose(T[:3, 1], [0, 0, -1], atol=1e-12), "영상 아래 = 아래"
    assert np.allclose(T[:3, 0], [0, -1, 0], atol=1e-12), "영상 오른쪽 = 로봇 우측"
    assert np.allclose(T[:3, 3], [0.05, 0.0, 0.10], atol=1e-12)


def test_camera_mount_pitch_down():
    """아래로 90도 숙이면 광축이 바닥을 향해야 한다."""
    T = camera_mount_T(0, 0, 0.1, pitch_deg=90.0)
    assert np.allclose(T[:3, 2], [0, 0, -1], atol=1e-12)


# ==========================================================================
# 3. 손으로 계산한 케이스 -- 규약 검증의 핵심
# ==========================================================================

def test_hand_straight_ahead():
    """
    마커: (2.0, 0.0, 0.5), -x 방향(로봇 쪽)을 향함
    카메라: base_link 원점, 수평
    로봇: 원점, +x 방향

    카메라에서 마커는 2.0m 앞, 0.5m 위.
      z_opt(광축)  = +2.0
      y_opt(아래)  = -0.5   (마커가 위에 있으므로)
      x_opt(오른쪽) = 0
    마커가 카메라를 정면으로 마주보므로 x축 기준 180도 회전.
    """
    T_marker = wall_marker_T(2.0, 0.0, 0.5, yaw_deg=180.0)
    T_cam = camera_mount_T(0.0, 0.0, 0.0)
    rvec = np.array([PI, 0.0, 0.0])
    tvec = np.array([0.0, -0.5, 2.0])

    T_base = estimate_base_pose(T_marker, rvec, tvec, T_cam)
    assert_pose(T_base, x=0.0, y=0.0, yaw_deg=0.0, z=0.0)


def test_hand_lateral_offset():
    """
    같은 마커, 로봇이 (1.0, +0.3) 에 있고 여전히 +x 방향.
    로봇이 왼쪽에 있으므로 마커는 영상 오른쪽 -> x_opt = +0.3
    """
    T_marker = wall_marker_T(2.0, 0.0, 0.5, yaw_deg=180.0)
    T_cam = camera_mount_T(0.0, 0.0, 0.0)
    rvec = np.array([PI, 0.0, 0.0])
    tvec = np.array([0.3, -0.5, 1.0])

    T_base = estimate_base_pose(T_marker, rvec, tvec, T_cam)
    assert_pose(T_base, x=1.0, y=0.3, yaw_deg=0.0, z=0.0)


def test_hand_robot_rotated_90():
    """
    마커: (1.0, 2.0, 0.5), -y 방향을 향함
    로봇: (1.0, 1.0), +y 방향(yaw=90도) -> 마커가 정면 1.0m
    """
    T_marker = wall_marker_T(1.0, 2.0, 0.5, yaw_deg=-90.0)
    T_cam = camera_mount_T(0.0, 0.0, 0.0)
    rvec = np.array([PI, 0.0, 0.0])
    tvec = np.array([0.0, -0.5, 1.0])

    T_base = estimate_base_pose(T_marker, rvec, tvec, T_cam)
    assert_pose(T_base, x=1.0, y=1.0, yaw_deg=90.0, z=0.0)


def test_hand_camera_offset_absorbed():
    """
    카메라가 전방 5cm, 높이 10cm 에 있고 마커도 같은 높이(0.10)일 때,
    카메라가 재는 거리는 1.95m 이지만 로봇은 원점에 있어야 한다.
    """
    T_marker = wall_marker_T(2.0, 0.0, 0.10, yaw_deg=180.0)
    T_cam = camera_mount_T(0.05, 0.0, 0.10)
    rvec = np.array([PI, 0.0, 0.0])
    tvec = np.array([0.0, 0.0, 1.95])

    T_base = estimate_base_pose(T_marker, rvec, tvec, T_cam)
    assert_pose(T_base, x=0.0, y=0.0, yaw_deg=0.0, z=0.0)


def test_hand_wrong_marker_size_scales_range():
    """
    마커 크기를 실제보다 10% 크게 입력하면 tvec 이 10% 크게 나오고,
    그만큼 추정 위치가 뒤로 밀린다. (인쇄 배율 오차의 영향 확인)
    """
    T_marker = wall_marker_T(2.0, 0.0, 0.0, yaw_deg=180.0)
    T_cam = camera_mount_T(0.0, 0.0, 0.0)
    rvec = np.array([PI, 0.0, 0.0])

    T_ok = estimate_base_pose(T_marker, rvec, [0.0, 0.0, 1.0], T_cam)
    T_bad = estimate_base_pose(T_marker, rvec, [0.0, 0.0, 1.1], T_cam)

    assert abs(xy_yaw(T_ok)[0] - 1.0) < 1e-12
    assert abs(xy_yaw(T_bad)[0] - 0.9) < 1e-12  # 10cm 뒤로 밀림


# ==========================================================================
# 4. 왕복 테스트 -- 임의 조합에서 수식 합성 검증
# ==========================================================================

def test_roundtrip_planar():
    """평면 주행 로봇 + 수평 카메라, 임의 자세."""
    rng = np.random.default_rng(10)
    T_marker = wall_marker_T(3.0, 1.0, 0.4, yaw_deg=200.0)
    T_cam = camera_mount_T(0.06, -0.01, 0.12)

    for _ in range(300):
        x, y = rng.uniform(-2, 2, 2)
        yaw = rng.uniform(-PI, PI)
        T_true = make_T(rpy_to_matrix(0, 0, yaw), [x, y, 0.0])

        rvec, tvec = simulate_observation(T_true, T_cam, T_marker)
        T_est = estimate_base_pose(T_marker, rvec, tvec, T_cam)
        assert np.allclose(T_est, T_true, atol=1e-9)


def test_roundtrip_tilted_camera_and_marker():
    """카메라 틸트 + 마커 기울어짐까지 포함한 일반 케이스."""
    rng = np.random.default_rng(11)
    for _ in range(300):
        T_marker = wall_marker_T(
            *rng.uniform(-3, 3, 2), rng.uniform(0.1, 1.5),
            yaw_deg=rng.uniform(-180, 180),
            roll_deg=rng.uniform(-10, 10),
        )
        T_cam = camera_mount_T(
            rng.uniform(-0.1, 0.1), rng.uniform(-0.1, 0.1), rng.uniform(0.05, 0.3),
            pitch_deg=rng.uniform(-20, 20),
            yaw_deg=rng.uniform(-15, 15),
            roll_deg=rng.uniform(-5, 5),
        )
        T_true = make_T(rpy_to_matrix(0, 0, rng.uniform(-PI, PI)),
                        [*rng.uniform(-2, 2, 2), 0.0])

        rvec, tvec = simulate_observation(T_true, T_cam, T_marker)
        T_est = estimate_base_pose(T_marker, rvec, tvec, T_cam)
        assert np.allclose(T_est, T_true, atol=1e-9)


def test_multiple_markers_agree():
    """같은 로봇 자세를 서로 다른 마커로 관측하면 같은 결과가 나와야 한다."""
    T_cam = camera_mount_T(0.05, 0.0, 0.12, pitch_deg=5.0)
    T_true = make_T(rpy_to_matrix(0, 0, math.radians(37.0)), [0.8, -0.4, 0.0])

    markers = [
        wall_marker_T(3.0, 0.0, 0.5, yaw_deg=180.0),
        wall_marker_T(0.0, 2.5, 0.8, yaw_deg=-90.0),
        wall_marker_T(-1.5, -1.0, 0.3, yaw_deg=45.0),
    ]
    for T_m in markers:
        rvec, tvec = simulate_observation(T_true, T_cam, T_m)
        assert np.allclose(estimate_base_pose(T_m, rvec, tvec, T_cam), T_true, atol=1e-9)


def test_map_odom_correction():
    """odom 이 map 과 일치하면 보정은 항등변환이어야 한다."""
    T = make_T(rpy_to_matrix(0, 0, 0.7), [1.2, -0.4, 0.0])
    assert np.allclose(map_odom_correction(T, T), np.eye(4), atol=1e-12)

    # 일반 케이스: T_map_odom @ T_odom_base == T_map_base
    T_map_base = make_T(rpy_to_matrix(0, 0, 0.3), [2.0, 1.0, 0.0])
    T_odom_base = make_T(rpy_to_matrix(0, 0, 0.25), [1.9, 1.1, 0.0])
    T_map_odom = map_odom_correction(T_map_base, T_odom_base)
    assert np.allclose(T_map_odom @ T_odom_base, T_map_base, atol=1e-12)


# ==========================================================================

def _run_all():
    tests = [(n, f) for n, f in sorted(globals().items())
             if n.startswith("test_") and callable(f)]
    failed = 0
    for name, fn in tests:
        try:
            fn()
            print(f"  PASS  {name}")
        except AssertionError as e:
            failed += 1
            print(f"  FAIL  {name}\n        {e}")
        except Exception as e:
            failed += 1
            print(f"  ERROR {name}\n        {type(e).__name__}: {e}")
    print(f"\n{len(tests) - failed}/{len(tests)} passed")
    return failed


if __name__ == "__main__":
    raise SystemExit(1 if _run_all() else 0)
