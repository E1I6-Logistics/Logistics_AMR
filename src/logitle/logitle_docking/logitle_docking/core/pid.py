import numpy as np


class PID:
    """Anti-windup 및 출력 제한을 지원하는 1차원 PID 제어기."""

    def __init__(self, p: float, i: float, d: float, out_min: float, out_max: float):
        self.p = float(p)
        self.i = float(i)
        self.d = float(d)
        self.out_min = float(out_min)
        self.out_max = float(out_max)
        self.prev_err = 0.0
        self.integral = 0.0

    def update(self, error: float, dt: float = 0.05) -> float:
        """오차와 시간 간격을 바탕으로 제어 출력을 계산하고 반환."""
        dt = max(dt, 0.01)
        self.integral += error * dt
        derivative = (error - self.prev_err) / dt
        output = (self.p * error) + (self.i * self.integral) + (self.d * derivative)

        # Anti-windup: 포화 시 적분 누적 롤백
        if output > self.out_max or output < self.out_min:
            self.integral -= error * dt

        self.prev_err = error
        return float(np.clip(output, self.out_min, self.out_max))

    def reset(self) -> None:
        """이전 오차 및 적분값 초기화."""
        self.prev_err = 0.0
        self.integral = 0.0
