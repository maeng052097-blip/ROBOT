"""LiDAR(YDLIDAR X2) 연결 확인 + 전방거리/안전상태 + 전방각 보정 스모크 테스트.

LiDAR만 연결한 상태에서 실행한다. 실시간으로:
  - 신선도(OK / STALE) · 스캔 점 개수
  - 전방 부채꼴(±SAFETY_ARC_DEG) 최소거리 + 안전상태(SAFE / SLOW / DANGER)
  - 전체에서 가장 가까운 점의 '각도'(FORWARD_ANGLE_DEG 보정용)
를 표시한다.

[전방각 보정] 차체 정면 가까이에 물체를 두고, 표시되는 '최근접 각도'를 보면
그 값이 곧 정면 방향 각도다. common/config.py 의 FORWARD_ANGLE_DEG 에 넣는다.
판단 로직(forward_min_distance / classify_safety)은 통합 컨트롤러와 동일하다.
"""
import sys
import pathlib
import time

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))
from common.config import (
    LIDAR_PORT, LIDAR_BAUDRATE, SAFETY_ARC_DEG, DANGER_MM, SLOW_MM, LIDAR_MAX_AGE,
)
from common.safety import forward_min_distance, classify_safety
from drivers.LidarX2 import LidarX2


def list_ports():
    """사용 가능한 COM 포트 출력(포트를 못 찾을 때 도움)."""
    try:
        import serial.tools.list_ports
        ports = list(serial.tools.list_ports.comports())
    except Exception:
        ports = []
    if ports:
        print("사용 가능한 COM 포트:")
        for p in ports:
            print(f"  {p.device}: {p.description}")
    else:
        print("사용 가능한 COM 포트가 없습니다 (장치 연결을 확인하세요).")


def closest_point(distance_dict):
    """전체에서 가장 가까운 (각도, 거리). 없으면 (None, None)."""
    best_a, best_d = None, None
    for a, d in distance_dict.items():
        if d > 0 and (best_d is None or d < best_d):
            best_a, best_d = a, d
    return best_a, best_d


def main():
    lidar = LidarX2(LIDAR_PORT, LIDAR_BAUDRATE)
    if not lidar.open():
        print(f"LiDAR({LIDAR_PORT}) 연결 실패. config.py 의 LIDAR_PORT 를 확인하세요.\n")
        list_ports()
        return

    print(f"LiDAR 연결: {LIDAR_PORT}")
    print(f"전방 +-{SAFETY_ARC_DEG}deg | DANGER<{DANGER_MM}mm, SLOW<{SLOW_MM}mm | Ctrl+C 종료")
    print("[전방각 보정] 정면 가까이에 물체 -> '최근접' 각도가 FORWARD_ANGLE_DEG\n")
    try:
        while True:
            dd = lidar.getDistanceDict()
            fresh = "OK   " if lidar.is_fresh(LIDAR_MAX_AGE) else "STALE"
            fmin = forward_min_distance(dd)
            state = classify_safety(dd)
            ca, cd = closest_point(dd)
            fwd = "-" if fmin is None else f"{fmin}mm"
            near = "-" if ca is None else f"{ca}deg {cd}mm"
            print(f"  [{fresh}] pts {len(dd):3d} | 전방최소 {fwd:>8} {state:<6} | 최근접 {near:>14}   ", end="\r")
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\n종료")
    finally:
        lidar.close()


if __name__ == "__main__":
    main()
