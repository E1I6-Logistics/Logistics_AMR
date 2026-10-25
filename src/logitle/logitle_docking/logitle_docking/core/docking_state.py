from enum import Enum


class DockingState(Enum):
    """
    정밀 도킹 시퀀스 상태 머신 단계.

    Two-Phase 제어:
      Phase 1 & 2: Odom 기반 경유지(Staging) 이동
      Phase 3 & 4: Pure ICP 기반 후진 도킹 및 근거리 최종 직선 후진
    """

    INIT = 0              # 도크 위치 추정 및 경유지 계산
    STAGING_TURN = 1      # [Odom] 경유지를 향해 제자리 회전 (전진 조준)
    STAGING_DRIVE = 2     # [Odom] 경유지로 직진 주행 (전진)
    STAGING_ALIGN = 3     # [Odom] 도크 진입축 정렬 (도크를 등지는 후진 자세 회전)
    ICP_SETTLE = 4        # 모션 스큐 방지 정지 대기
    ALIGN_HEADING = 5     # [Pure ICP] 근거리 각도 정밀 정렬
    STRAIGHT_REVERSE = 6  # [Pure ICP] 도크 안으로 정밀 감속 후진 (S-curve 조향)
    FINAL_REVERSE = 7     # [Odom] 근거리에서 ICP를 끄고 저속 직선 후진 + yaw 유지
    COMPLETED = 8         # 도킹 완료
