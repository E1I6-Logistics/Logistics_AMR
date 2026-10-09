"""도킹 실패 시 안전한 전진 이탈·정체 감지의 순수 기하 함수."""
import math


def retreat_distance(map_pose, final_pose, staging_offset, extra, minimum, maximum):
    """완전 도킹 Pose의 전방 축에서 staging+extra 지점까지 필요한 전진 거리."""
    x, y, _ = map_pose
    fx, fy, fyaw = final_pose
    axial = (x-fx)*math.cos(fyaw) + (y-fy)*math.sin(fyaw)
    distance = max(minimum, staging_offset + extra - axial)
    return distance if 0 < distance <= maximum else None


def axis_progress(start_x, start_y, start_yaw, x, y, forward=True):
    delta = (x-start_x)*math.cos(start_yaw) + (y-start_y)*math.sin(start_yaw)
    return delta if forward else -delta


def corridor_clear(closest_x, front_offset, margin, travel=0.0):
    """LiDAR 유효 관측이 없다면 closest_x=None: 반드시 불허."""
    return closest_x is not None and closest_x > front_offset + margin + travel


def unsafe_turn_near_dock(map_pose, final_pose, heading_error, near_radius, max_turn_deg):
    # 도크에 가까울 때 몸체 간섭을 일으킬 수 있는 제자리 회전은 실행하지 않는다.
    x, y, _ = map_pose
    fx, fy, _ = final_pose
    return math.hypot(x-fx, y-fy) < near_radius and abs(heading_error) > math.radians(max_turn_deg)


class MotionStallWatchdog:
    """명령 방향의 진행량이 제한시간 동안 기준 거리 미만인지 검사한다.

    휠 슬립으로 Odom이 실제 움직임을 과대평가하면 충돌을 검출하지 못한다.
    범퍼/전류 감시 및 상위 안전 정지와 병행해야 한다.
    """
    def __init__(self, timeout=2.5, progress=0.010, min_speed=0.008):
        self.timeout, self.progress, self.min_speed = timeout, progress, min_speed
        self.reset()

    def reset(self):
        self.anchor = None
        self.sign = 0
        self.since = 0.0

    def observe(self, now, pose, commanded_v):
        if pose is None or abs(commanded_v) < self.min_speed:
            self.reset()
            return False
        sign = 1 if commanded_v > 0 else -1
        if self.anchor is None or self.sign != sign:
            self.anchor, self.since, self.sign = tuple(pose), now, sign
            return False
        progress = axis_progress(*self.anchor, pose[0], pose[1], forward=sign > 0)
        if progress >= self.progress:
            self.anchor, self.since = tuple(pose), now
            return False
        return now - self.since >= self.timeout
