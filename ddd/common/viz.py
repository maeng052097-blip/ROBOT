"""공용 시각화 헬퍼 (거리색 / 외곽선 텍스트 / 링 간격).

여러 visualization 도구가 공유한다. 도구별 특수 로직이 없는 순수 UI 유틸이라
common 에 둔다(과거엔 lidar_probe_view.py 에 있어 도구끼리 cross-import 했음).
cv2 는 함수 안에서 import → 이 모듈 자체는 cv2 없이 import 된다.
"""
from common.config import DANGER_MM, SLOW_MM


def zone_bgr(d):
    """거리(mm) -> 안전 색(BGR): DANGER 빨강 / SLOW 주황 / SAFE 초록."""
    if d < DANGER_MM:
        return (0, 0, 255)
    if d < SLOW_MM:
        return (0, 165, 255)
    return (0, 200, 0)


def _txt(img, s, org, color, scale=0.5, thick=1):
    """검은 외곽선 + 색 글씨(점 위에서도 읽히게)."""
    import cv2
    cv2.putText(img, s, org, cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), thick + 2, cv2.LINE_AA)
    cv2.putText(img, s, org, cv2.FONT_HERSHEY_SIMPLEX, scale, color, thick, cv2.LINE_AA)


def ring_step_mm(rmax):
    """rmax 에 맞춰 레이더 링 간격(mm)을 정한다 (화면에 약 6~12개)."""
    for step in (100, 200, 250, 500, 1000, 2000):
        if rmax / step <= 12:
            return step
    return 2000
