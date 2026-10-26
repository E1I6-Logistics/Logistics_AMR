"""
마커 맵 로더.

벽에 붙인 마커들의 맵 좌표를 YAML 로 관리하고 T_map_marker 로 변환합니다.
사람이 실측해서 채우는 파일이므로, 값 검증을 로드 시점에 해둡니다.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import numpy as np
import yaml

from logitle_aruco_tools.logitle_marker_localization import wall_marker_T

__all__ = ["MarkerEntry", "load_marker_map"]


@dataclass
class MarkerEntry:
    id: int
    size: float            # 검은 정사각형 외곽 한 변 [m] (인쇄물 실측값)
    T_map_marker: np.ndarray
    yaw_deg: float = 0.0   # 법선 방향 (YAML 에 적은 값 그대로)
    note: str = ""

    @property
    def position(self) -> np.ndarray:
        """마커 중심의 맵 좌표."""
        return self.T_map_marker[:3, 3]

    @property
    def normal(self) -> np.ndarray:
        """마커가 바라보는 방향 (벽에서 바깥)."""
        return self.T_map_marker[:3, 2]


def load_marker_map(path: str) -> dict[int, MarkerEntry]:
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)

    if not isinstance(data, dict) or "markers" not in data:
        raise ValueError(f"{path}: 최상위에 'markers' 키가 필요합니다")

    frame = data.get("frame_id", "map")
    out: dict[int, MarkerEntry] = {}

    for i, m in enumerate(data["markers"]):
        missing = {"id", "x", "y", "z", "yaw_deg", "size"} - set(m)
        if missing:
            raise ValueError(f"{path}: markers[{i}] 에 누락된 키 {sorted(missing)}")

        mid = int(m["id"])
        if mid in out:
            raise ValueError(f"{path}: 마커 ID {mid} 가 중복되었습니다")

        size = float(m["size"])
        x = float(m["x"])
        y = float(m["y"])
        z = float(m["z"])
        yaw_deg = float(m["yaw_deg"])
        roll_deg = float(m.get("roll_deg", 0.0))
        values = {
            "x": x,
            "y": y,
            "z": z,
            "yaw_deg": yaw_deg,
            "roll_deg": roll_deg,
            "size": size,
        }
        bad = [name for name, value in values.items() if not math.isfinite(value)]
        if bad:
            raise ValueError(f"{path}: 마커 {mid} 의 {bad} 값이 유한하지 않습니다")
        if not (0.01 <= size <= 1.0):
            raise ValueError(
                f"{path}: 마커 {mid} 의 size={size} 가 비정상입니다. "
                f"단위는 미터입니다 (150mm -> 0.150)")

        out[mid] = MarkerEntry(
            id=mid,
            size=size,
            T_map_marker=wall_marker_T(
                x, y, z,
                yaw_deg=yaw_deg,
                roll_deg=roll_deg,
            ),
            yaw_deg=yaw_deg,
            note=str(m.get("note", "")),
        )

    if not out:
        raise ValueError(f"{path}: 마커가 하나도 없습니다")

    print(f"[marker_map] {path}: 마커 {len(out)}개 로드 "
          f"(frame_id={frame}, ids={sorted(out)})")
    return out


def main():
    import sys

    path = sys.argv[1] if len(sys.argv) > 1 else "logitle_marker_map.yaml"
    for mid, e in sorted(load_marker_map(path).items()):
        p, n = e.position, e.normal
        print(f"  ID {mid:3d}  size={e.size:.4f}m"
              f"  pos=({p[0]:+.3f}, {p[1]:+.3f}, {p[2]:+.3f})"
              f"  법선={e.yaw_deg:+7.1f}deg ({n[0]:+.2f}, {n[1]:+.2f}, {n[2]:+.2f})"
              f"  {e.note}")


if __name__ == "__main__":
    main()
