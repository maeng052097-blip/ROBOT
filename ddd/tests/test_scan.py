"""LiDAR(YDLIDAR X2) 연결 및 전방 거리 확인 스모크 테스트.

LiDAR만 연결한 상태에서 실행한다. 전방 부채꼴 최소거리와 안전상태
(SAFE/SLOW/DANGER)를 실시간 출력하므로, 손을 앞에 대보며
common/config.py 의 DANGER_MM / SLOW_MM 임계값을 조정하는 데 쓴다.

판단 로직(forward_min_distance, classify_safety)은 통합 주행 컨트롤러와
'완전히 동일'한 common.safety 를 그대로 쓴다.
"""
import sys
import pathlib
import time

# repo-root(상위 폴더)를 import 경로에 추가
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))

from common.config import LIDAR_PORT, LIDAR_BAUDRATE, SAFETY_ARC_DEG, DANGER_MM, SLOW_MM
from common.safety import forward_min_distance, classify_safety
from drivers.LidarX2 import LidarX2


def main():
    lidar = LidarX2(LIDAR_PORT, LIDAR_BAUDRATE)
    if not lidar.open():
        print(f"LiDAR({LIDAR_PORT}) 연결 실패. config.py 의 LIDAR_PORT 를 확인하세요.")
        return

    print(f"LiDAR 연결: {LIDAR_PORT}")
    print(f"전방 ±{SAFETY_ARC_DEG}° | DANGER<{DANGER_MM}mm, SLOW<{SLOW_MM}mm | Ctrl+C 종료\n")
    try:
        while True:
            dd = lidar.getDistanceDict()
            m = forward_min_distance(dd)
            state = classify_safety(dd)
            shown = "-" if m is None else f"{m} mm"
            print(f"  스캔점 {len(dd):3d} | 전방최소 {shown:>9} | {state:<6}   ", end="\r")
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\n종료")
    finally:
        lidar.close()


if __name__ == "__main__":
    main()
