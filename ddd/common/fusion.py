"""카메라 베어링 + LiDAR 거리 융합.

카메라(YOLO)가 준 물체의 방향(베어링)을 LiDAR 각도로 변환해, 그 방향의
LiDAR 거리(mm)를 얻는다. => "탐지한 물체까지의 거리".

좌표 약속:
  - bearing_deg: 화면 중앙=0, 오른쪽 +, 왼쪽 - (detector.bearing_from_cx 와 동일).
  - LiDAR 각도: FORWARD_ANGLE_DEG 가 정면. CAMERA_LIDAR_SIGN 으로 좌우 방향을 맞춘다.
순수 함수라 하드웨어 없이 단위 테스트 가능.
"""
from common.config import FORWARD_ANGLE_DEG, CAMERA_LIDAR_SIGN, FUSION_TOL_DEG


def bearing_to_lidar_angle(bearing_deg):
    """카메라 베어링(deg) -> LiDAR 각도(deg, 0~359)."""
    return (FORWARD_ANGLE_DEG + CAMERA_LIDAR_SIGN * bearing_deg) % 360.0


def distance_at_angle(distance_dict, target_deg, tol_deg=FUSION_TOL_DEG):
    """target_deg 에 tol_deg 이내로 각도상 가장 가까운 측정 거리(mm). 없으면 None."""
    best_d, best_diff = None, None
    for a, d in distance_dict.items():
        if d <= 0:
            continue
        diff = abs((a - target_deg + 180) % 360 - 180)
        if diff <= tol_deg and (best_diff is None or diff < best_diff):
            best_diff, best_d = diff, d
    return best_d


def object_distance_mm(bearing_deg, distance_dict, tol_deg=FUSION_TOL_DEG):
    """카메라 베어링 방향에 있는 물체까지의 LiDAR 거리(mm). 측정값 없으면 None."""
    return distance_at_angle(distance_dict, bearing_to_lidar_angle(bearing_deg), tol_deg)
