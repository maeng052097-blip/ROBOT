"""LiDAR 거리 데이터로 전방 안전 상태를 판단하는 공용 로직.

통합 컨트롤러(웹캠_LiDAR_주행제어.py)와 LiDAR 스모크 테스트(tests/test_scan.py)가
'완전히 같은' 판단을 쓰도록 한 곳에 모은다. (임계값 튜닝 시 테스트로 본 결과 =
실제 주행 판단)
"""
from common.config import (
    FORWARD_ANGLE_DEG, SAFETY_ARC_DEG, DANGER_MM, SLOW_MM, EMPTY_ARC_IS_SAFE,
)

# 안전 상태 값
SAFE, SLOW, DANGER = "SAFE", "SLOW", "DANGER"


def forward_min_distance(distance_dict):
    """전방 부채꼴(±SAFETY_ARC_DEG) 안의 최소 거리(mm). 측정값이 없으면 None.

    0도 경계를 넘어가도 되도록 각도차를 0~180 범위로 계산한다.
    """
    candidates = []
    for angle, dist in distance_dict.items():
        diff = abs((angle - FORWARD_ANGLE_DEG + 180) % 360 - 180)
        if diff <= SAFETY_ARC_DEG and dist > 0:
            candidates.append(dist)
    return min(candidates) if candidates else None


def classify_safety(distance_dict):
    """전방 최소거리로 DANGER / SLOW / SAFE 를 판단한다.

    DANGER_MM 미만 -> DANGER, SLOW_MM 미만 -> SLOW, 그 외 -> SAFE.
    전방에 측정값이 없으면 EMPTY_ARC_IS_SAFE 설정을 따른다.
    """
    min_dist = forward_min_distance(distance_dict)
    if min_dist is None:
        return SAFE if EMPTY_ARC_IS_SAFE else DANGER
    if min_dist < DANGER_MM:
        return DANGER
    if min_dist < SLOW_MM:
        return SLOW
    return SAFE
