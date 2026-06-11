"""전방(정면) 각도 찾기 — FORWARD_ANGLE_DEG 보정.

차체 정면(카메라가 보는 방향)에 물체 하나를 '가까이' 두고 실행하면, LiDAR 가 본
'가장 가까운 점'의 raw 각도를 보여준다. 그 raw 각도가 곧 정면 = FORWARD_ANGLE_DEG.

실행: py -3.13 tests/find_front.py --lidar x4 --port COM8     (Ctrl+C 종료)
사용: 정면에 물체 하나만 가까이 두고(주변 비우기), 표시된 raw 각도를
      common/config.py 의 FORWARD_ANGLE_DEG 에 입력한다.
주의: track_and_approach 등 다른 프로그램이 같은 COM 포트를 잡고 있으면 먼저 닫을 것.
"""
import argparse
import sys
import pathlib
import time

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))

from common.config import LIDAR_X4_PORT, LIDAR_X4_BAUDRATE, FORWARD_ANGLE_DEG  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description="정면 각도(FORWARD_ANGLE_DEG) 찾기")
    ap.add_argument("--lidar", default="x4", choices=["x2", "x4"])
    ap.add_argument("--port", default=LIDAR_X4_PORT)
    ap.add_argument("--baud", type=int, default=LIDAR_X4_BAUDRATE)
    args = ap.parse_args()

    from drivers import make_lidar
    lidar = make_lidar(args.lidar, args.port, args.baud)
    if not lidar.open():
        print(f"LiDAR({args.port}) 연결 실패. 포트/모델(--lidar)/점유 여부를 확인하세요.")
        return

    print("정면(카메라 방향)에 물체 하나만 '가까이' 두세요. 주변은 비우는 게 좋습니다.")
    print(f"가장 가까운 점의 raw 각도가 정면입니다. (현재 FORWARD_ANGLE_DEG = {FORWARD_ANGLE_DEG})")
    print("Ctrl+C 로 종료.\n")
    try:
        while True:
            dd = lidar.getDistanceDict()
            best_a, best_d = None, None
            for a, d in dd.items():
                if d > 0 and (best_d is None or d < best_d):
                    best_a, best_d = a, d
            if best_a is None:
                print("  (측정점 없음 - 물체를 더 가까이/주변 비우기)            ", end="\r")
            else:
                print(f"  최근접 raw={best_a:3d}deg  {best_d/10:.0f}cm   ->   "
                      f"FORWARD_ANGLE_DEG = {best_a}        ", end="\r")
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\n종료")
    finally:
        lidar.close()


if __name__ == "__main__":
    main()
