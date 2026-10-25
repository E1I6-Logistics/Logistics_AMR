import math

from logitle_docking.core.geometry import (
    generate_v_funnel,
    get_yaw_from_quaternion,
    icp_2d,
    normalize_angle,
)
from logitle_docking.core.pid import PID
import numpy as np
from scipy.spatial import cKDTree


class DummyQuaternion:
    """테스트용 더미 쿼터니언 클래스."""

    def __init__(self, x=0.0, y=0.0, z=0.0, w=1.0):
        self.x = float(x)
        self.y = float(y)
        self.z = float(z)
        self.w = float(w)


def test_normalize_angle():
    assert math.isclose(normalize_angle(0.0), 0.0, abs_tol=1e-6)
    assert math.isclose(normalize_angle(2 * math.pi), 0.0, abs_tol=1e-6)
    assert math.isclose(normalize_angle(3 * math.pi), math.pi, abs_tol=1e-6)
    assert math.isclose(normalize_angle(-3 * math.pi), -math.pi, abs_tol=1e-6)

    # Wrap-around boundary
    wrapped = normalize_angle(math.pi + 0.2)
    assert math.isclose(wrapped, -math.pi + 0.2, abs_tol=1e-6)


def test_get_yaw_from_quaternion():
    # Identity: yaw = 0
    q_zero = DummyQuaternion(0.0, 0.0, 0.0, 1.0)
    assert math.isclose(get_yaw_from_quaternion(q_zero), 0.0, abs_tol=1e-6)

    # 90 degrees yaw rotation around z
    yaw_target = math.pi / 2.0
    q_90 = DummyQuaternion(0.0, 0.0, math.sin(yaw_target / 2.0), math.cos(yaw_target / 2.0))
    assert math.isclose(get_yaw_from_quaternion(q_90), yaw_target, abs_tol=1e-6)


def test_pid_controller():
    pid = PID(p=2.0, i=0.5, d=0.1, out_min=-5.0, out_max=5.0)

    # P-step response
    out = pid.update(1.0, dt=0.1)
    # output = 2.0 * 1.0 + 0.5 * 0.1 + 0.1 * (1.0 - 0.0) / 0.1 = 2.0 + 0.05 + 1.0 = 3.05
    assert math.isclose(out, 3.05, abs_tol=1e-2)

    # Clamping test
    out_saturated = pid.update(10.0, dt=0.1)
    assert out_saturated == 5.0

    # Reset test
    pid.reset()
    assert pid.prev_err == 0.0
    assert pid.integral == 0.0


def test_v_funnel_generation():
    pts = generate_v_funnel(
        charger_width=0.145,
        wing_length=0.10,
        wing_angle_rad=math.radians(22.5),
    )
    assert len(pts) > 20
    assert pts.shape[1] == 2

    # Verify Y-axis symmetry: for each (x, y), (x, -y) should exist
    for pt in pts:
        x, y = pt[0], pt[1]
        symmetric_found = any(
            np.isclose(x, other[0], atol=1e-4) and np.isclose(-y, other[1], atol=1e-4)
            for other in pts
        )
        assert symmetric_found


def test_icp_2d_convergence():
    charger_pts = generate_v_funnel(0.145, 0.10, math.radians(22.5))
    tree = cKDTree(charger_pts)

    # Synthesize robot-observed points transformed by small dx, dy, and yaw
    dx, dy, dyaw = 0.03, -0.02, math.radians(2.0)
    c, s = math.cos(dyaw), math.sin(dyaw)
    rot_matrix = np.array([[c, -s], [s, c]], dtype=np.float32)
    source_pts = (charger_pts @ rot_matrix.T) + np.array([dx, dy], dtype=np.float32)

    T, fitness, matched_count = icp_2d(
        source_pts=source_pts,
        target_tree=tree,
        target_pts=charger_pts,
        initial_transform=np.identity(3, dtype=np.float32),
        max_iter=30,
        search_radius=0.15,
    )

    assert matched_count >= 15
    assert fitness > 0.85

    recovered_t = T[:2, 2]
    # Source was transformed by T_true; ICP finds transformation from source to target
    # Thus recovered translation is approximately -rot.T @ [dx, dy]
    assert np.all(np.isfinite(recovered_t))
