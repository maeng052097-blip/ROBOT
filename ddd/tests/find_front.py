"""전방(정면) 각도 찾기 — FORWARD_ANGLE_DEG 보정.

차체 정면(카메라가 보는 방향, 주행 방향)에 물체 하나를 '가까이' 두고 실행하면,
LiDAR 가 본 '가장 가까운 점'의 각도를 보여준다. 그 각도가 곧 정면 = FORWARD_ANGLE_DEG.

실행: python tests/find_front.py   (Ctrl+C 로 종료)
사용: 정면에 물체 하나만 가까이 두고(주변은 비우고), 표시된 각도를
      common/config.py 의 FORWARD_ANGLE_DEG 에 입력한다.
"""
import sys
import pathlib
import time

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))

from common.config import LIDAR_PORT, LIDAR_BAUDRATE, FORWARD_ANGLE_DEG
from drivers.LidarX2 import LidarX2


def main():
    lidar = LidarX2(LIDAR_PORT, LIDAR_BAUDRATE)
    if not lidar.open():
        print(f"LiDAR({LIDAR_PORT}) 연결 실패. config.py 의 LIDAR_PORT 를 확인하세요.")
        return

    print("정면(카메라 방향)에 물체 하나를 '가까이' 두세요. 주변은 비우는 게 좋습니다.")
    print(f"가장 가까운 점의 각도가 정면입니다.  (현재 설정 FORWARD_ANGLE_DEG = {FORWARD_ANGLE_DEG})")
    print("Ctrl+C 로 종료.\n")
    try:
        while True:
            dd = lidar.getDistanceDict()
            best_a, best_d = None, None
            for a, d in dd.items():
                if d > 0 and (best_d is None or d < best_d):
                    best_a, best_d = a, d
            if best_a is None:
                print("  (전방에 측정점 없음 - 물체를 더 가까이 두세요)            ", end="\r")
            else:
                print(f"  가장 가까운 점: {best_a:3d} deg, {best_d/10:.0f} cm   ->   "
                      f"FORWARD_ANGLE_DEG = {best_a}        ", end="\r")
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\n종료")
    finally:
        lidar.close()


if __name__ == "__main__":
    main()
