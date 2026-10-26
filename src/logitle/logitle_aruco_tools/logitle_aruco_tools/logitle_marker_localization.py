"""
ArUco 마커 기반 로봇 위치추정 - 좌표 변환 모듈

핵심 계산:
    T_map_base = T_map_marker @ inv(T_camopt_marker) @ inv(T_base_camopt)

    T_map_marker    : 벽에 붙인 마커의 맵 좌표      (사람이 실측 -> YAML)
    T_camopt_marker : ArUco 자세 추정 결과 (rvec, tvec)
    T_base_camopt   : 카메라 장착 오프셋             (사람이 실측 -> static TF)

좌표계 규약
-----------
map / base_link (ROS REP-103, FLU)
    x = 전방, y = 좌측, z = 위

camera_optical_frame (REP-104)
    x = 영상 오른쪽, y = 영상 아래, z = 광축(전방)

marker (OpenCV ArUco)
    x = 마커 오른쪽, y = 마커 위, z = 마커 평면에서 바깥(보는 쪽)으로
    -> 벽 마커라면 z축이 벽에서 방을 향합니다.

ROS 의존성 없음 (numpy만 필요). 단위 테스트를 로봇 없이 돌리기 위함입니다.
ROS 연동 시:
    - rvec/tvec 은 aruco_opencv 의 검출 결과에서 가져옵니다.
    - T_base_camopt 는 static TF 대신 여기서 camera_mount_T() 로 만들어도 되고,
      tf2 의 lookup_transform 결과를 make_T() 로 감싸도 됩니다.
    - 최종적으로 map->odom 을 발행할 때는 publish 전에
      T_map_odom = T_map_base @ inv(T_odom_base) 를 계산하세요 (map_odom_correction).
"""

from __future__ import annotations

import math
import numpy as np

__all__ = [
    "rodrigues", "rotation_to_rvec",
    "rpy_to_matrix", "matrix_to_rpy",
    "make_T", "invert", "transform_point",
    "wall_marker_T", "camera_mount_T",
    "estimate_base_pose", "simulate_observation",
    "xy_yaw", "yaw_of", "pose_str",
    "R_FLU_TO_OPTICAL",
]

# base_link(FLU) -> camera_optical_frame(RDF) 회전
# 열벡터 = 광학 좌표축을 FLU 로 표현한 것
#   x_opt(영상 오른쪽) = -y_flu
#   y_opt(영상 아래)   = -z_flu
#   z_opt(광축)        = +x_flu
R_FLU_TO_OPTICAL = np.array([
    [0.0,  0.0, 1.0],
    [-1.0, 0.0, 0.0],
    [0.0, -1.0, 0.0],
])


# --------------------------------------------------------------------------
# 회전 변환
# --------------------------------------------------------------------------

def rodrigues(rvec) -> np.ndarray:
    """회전 벡터 -> 3x3 회전 행렬. cv2.Rodrigues() 와 동일한 결과."""
    r = np.asarray(rvec, dtype=float).reshape(3)
    theta = float(np.linalg.norm(r))
    if theta < 1e-12:
        return np.eye(3)
    k = r / theta
    K = np.array([
        [0.0, -k[2], k[1]],
        [k[2], 0.0, -k[0]],
        [-k[1], k[0], 0.0],
    ])
    return np.eye(3) + math.sin(theta) * K + (1.0 - math.cos(theta)) * (K @ K)


def rotation_to_rvec(R) -> np.ndarray:
    """
    3x3 회전 행렬 -> 회전 벡터 (rodrigues 의 역).

    180도 부근에서 acos/sin 을 쓰는 단순 구현은 정밀도가 무너지므로
    쿼터니언(Shepperd 방식)을 거쳐 계산합니다. 전 구간에서 안정적입니다.
    """
    R = np.asarray(R, dtype=float)
    tr = R[0, 0] + R[1, 1] + R[2, 2]

    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s

    v = np.array([x, y, z], dtype=float)
    n = float(np.linalg.norm(v))
    if n < 1e-15:
        return np.zeros(3)
    theta = 2.0 * math.atan2(n, w)
    if theta > math.pi:          # rvec 크기를 [0, pi] 로 정규화
        theta -= 2.0 * math.pi
    return (v / n) * theta


def rpy_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """ROS 규약 RPY(라디안) -> 회전 행렬.  R = Rz(yaw) @ Ry(pitch) @ Rx(roll)"""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]], dtype=float)
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]], dtype=float)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]], dtype=float)
    return Rz @ Ry @ Rx


def matrix_to_rpy(R) -> tuple[float, float, float]:
    """회전 행렬 -> (roll, pitch, yaw) 라디안."""
    R = np.asarray(R, dtype=float)
    sp = -R[2, 0]
    sp = max(-1.0, min(1.0, sp))
    pitch = math.asin(sp)
    if abs(sp) > 1.0 - 1e-9:  # 짐벌락
        roll = 0.0
        yaw = math.atan2(-R[0, 1], R[1, 1])
    else:
        roll = math.atan2(R[2, 1], R[2, 2])
        yaw = math.atan2(R[1, 0], R[0, 0])
    return roll, pitch, yaw


# --------------------------------------------------------------------------
# 동차변환 (4x4)
# --------------------------------------------------------------------------

def make_T(R, t) -> np.ndarray:
    """회전 행렬과 평행이동 -> 4x4 동차변환."""
    T = np.eye(4)
    T[:3, :3] = np.asarray(R, dtype=float).reshape(3, 3)
    T[:3, 3] = np.asarray(t, dtype=float).reshape(3)
    return T


def invert(T) -> np.ndarray:
    """4x4 동차변환의 역변환.  R^-1 = R^T,  t^-1 = -R^T @ t"""
    T = np.asarray(T, dtype=float)
    R = T[:3, :3]
    t = T[:3, 3]
    out = np.eye(4)
    out[:3, :3] = R.T
    out[:3, 3] = -R.T @ t
    return out


def transform_point(T, p) -> np.ndarray:
    """4x4 변환을 3D 점에 적용."""
    p = np.asarray(p, dtype=float).reshape(3)
    T = np.asarray(T, dtype=float)
    return T[:3, :3] @ p + T[:3, 3]


# --------------------------------------------------------------------------
# 입력값 구성
# --------------------------------------------------------------------------

def wall_marker_T(x: float, y: float, z: float,
                  yaw_deg: float, roll_deg: float = 0.0) -> np.ndarray:
    """
    벽에 수직으로 붙인 마커의 맵 좌표 -> T_map_marker.

    x, y, z : 마커 '중심'의 맵 좌표 [m]
    yaw_deg : 마커가 바라보는 방향(바깥 법선)을 맵 +x축 기준 반시계 각도로.
              예) 마커가 -x 방향을 향하면 180, +y 방향을 향하면 90
    roll_deg: 마커가 기울어져 붙은 경우 법선축 기준 회전 [deg] (보통 0)

    마커 축 배치:
        z_marker = (cos yaw, sin yaw, 0)   벽에서 바깥
        y_marker = (0, 0, 1)               위
        x_marker = y x z                   마커 기준 오른쪽
    """
    yaw = math.radians(yaw_deg)
    z_axis = np.array([math.cos(yaw), math.sin(yaw), 0.0])
    y_axis = np.array([0.0, 0.0, 1.0])
    x_axis = np.cross(y_axis, z_axis)

    R = np.column_stack([x_axis, y_axis, z_axis])

    if roll_deg:
        r = math.radians(roll_deg)
        Rz_local = np.array([
            [math.cos(r), -math.sin(r), 0.0],
            [math.sin(r), math.cos(r), 0.0],
            [0.0, 0.0, 1.0],
        ])
        R = R @ Rz_local

    return make_T(R, [x, y, z])


def camera_mount_T(x: float, y: float, z: float,
                   pitch_deg: float = 0.0,
                   yaw_deg: float = 0.0,
                   roll_deg: float = 0.0) -> np.ndarray:
    """
    카메라 장착 오프셋 -> T_base_camopt (base_link 기준 camera_optical_frame).

    x, y, z   : base_link 원점(두 바퀴 축 중점, 바닥면) 기준 렌즈 중심 위치 [m]
                x=전방, y=좌측, z=위
    pitch_deg : 아래로 숙인 각도가 양수 [deg]
    yaw_deg   : 좌측으로 돌린 각도가 양수 [deg]
    roll_deg  : 광축 기준 기울기 [deg]
    """
    R_body = rpy_to_matrix(
        math.radians(roll_deg),
        math.radians(pitch_deg),
        math.radians(yaw_deg),
    )
    return make_T(R_body @ R_FLU_TO_OPTICAL, [x, y, z])


# --------------------------------------------------------------------------
# 본 계산
# --------------------------------------------------------------------------

def estimate_base_pose(T_map_marker, rvec, tvec, T_base_camopt) -> np.ndarray:
    """
    마커 1개 관측으로부터 로봇(base_link)의 맵 좌표 T_map_base 를 계산.

    rvec, tvec : cv2.aruco 자세 추정 결과 (camera_optical_frame 기준 마커 자세)
    """
    T_camopt_marker = make_T(rodrigues(rvec), tvec)
    return (np.asarray(T_map_marker, dtype=float)
            @ invert(T_camopt_marker)
            @ invert(T_base_camopt))


def simulate_observation(T_map_base, T_base_camopt, T_map_marker):
    """
    estimate_base_pose 의 역방향. 테스트/시뮬레이션용.

    로봇이 T_map_base 에 있을 때 카메라가 관측할 (rvec, tvec) 을 만들어 줍니다.
    """
    T_map_camopt = np.asarray(T_map_base, dtype=float) @ np.asarray(T_base_camopt, dtype=float)
    T_camopt_marker = invert(T_map_camopt) @ np.asarray(T_map_marker, dtype=float)
    rvec = rotation_to_rvec(T_camopt_marker[:3, :3])
    tvec = T_camopt_marker[:3, 3].copy()
    return rvec, tvec


def map_odom_correction(T_map_base, T_odom_base) -> np.ndarray:
    """
    map -> odom 보정 변환. TF 트리를 지키려면 map->base_link 가 아니라
    이 값을 발행해야 합니다 (AMCL 과 동일한 방식).
    """
    return np.asarray(T_map_base, dtype=float) @ invert(T_odom_base)


# --------------------------------------------------------------------------
# 출력 도우미
# --------------------------------------------------------------------------

def yaw_of(T) -> float:
    """4x4 변환에서 yaw [rad] 추출."""
    return matrix_to_rpy(np.asarray(T, dtype=float)[:3, :3])[2]


def xy_yaw(T) -> tuple[float, float, float]:
    """4x4 변환 -> (x, y, yaw[rad]). 평면 주행 로봇의 2D 자세."""
    T = np.asarray(T, dtype=float)
    return float(T[0, 3]), float(T[1, 3]), yaw_of(T)


def pose_str(T) -> str:
    x, y, yaw = xy_yaw(T)
    z = float(np.asarray(T, dtype=float)[2, 3])
    return f"x={x:+.4f}  y={y:+.4f}  z={z:+.4f}  yaw={math.degrees(yaw):+.2f}deg"


# --------------------------------------------------------------------------
# 수동 확인용 데모
# --------------------------------------------------------------------------

def main():
    # 마커: (2.0, 0.0, 0.50) 에 붙어 있고 -x 방향(로봇 쪽)을 향함
    T_map_marker = wall_marker_T(2.0, 0.0, 0.50, yaw_deg=180.0)

    # 카메라: base_link 기준 전방 5cm, 높이 10cm, 수평
    T_base_cam = camera_mount_T(0.05, 0.0, 0.10)

    # 로봇이 (0.7, 0.3) 에서 왼쪽으로 10도 틀어진 상태라고 가정
    T_true = make_T(rpy_to_matrix(0, 0, math.radians(10.0)), [0.7, 0.3, 0.0])

    rvec, tvec = simulate_observation(T_true, T_base_cam, T_map_marker)
    T_est = estimate_base_pose(T_map_marker, rvec, tvec, T_base_cam)

    print("관측값  rvec =", np.round(rvec, 5), " tvec =", np.round(tvec, 5))
    print("참값    ", pose_str(T_true))
    print("추정값  ", pose_str(T_est))
    print("오차    ", f"{np.linalg.norm(T_true[:3, 3] - T_est[:3, 3]):.3e} m")


if __name__ == "__main__":
    main()
