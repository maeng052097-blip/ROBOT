"""visualization/lidar_map_view.py — X2 점유격자(occupancy grid) 실시간 맵 (C-1).

X2(벽 맵핑 주력)의 스캔을 점유격자에 누적해 주변 벽/장애물을 흑백 맵으로 표시.
  점유=검정, 자유=흰색, 미지=회색. 로봇=중앙 빨강 점.

[현재 한계 — 중요/불명 아님, 설계상]
  로봇 포즈(위치·헤딩)는 (0,0,0) 고정 = '정지' 가정. 움직이며 맵을 '확장'하려면
  엔코더 오도메트리로 pose 를 갱신해야 함(다음 단계 C-2). 지금은 정지 상태에서
  주변을 누적·평활한 점유격자를 만든다(노이즈가 줄고 벽이 또렷해짐).

조작: q 종료 | c 맵 초기화
실행: py -3.13 visualization/lidar_map_view.py        (X2=COM12 기본, --lidar/--port 로 변경)
"""
import sys
import time
import pathlib
import argparse

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))

from common.config import (
    LIDAR_X2_PORT, LIDAR_X2_OFFSET_DEG, FORWARD_ANGLE_DEG, MAP_SIZE_M, MAP_RES_M,
)
from common.occupancy_grid import OccupancyGrid
from visualization.lidar_probe_view import _txt

DISP = 680   # 표시 한 변(px) 목표


def render(grid):
    import cv2
    import numpy as np

    n = grid.n
    arr = np.array(grid.log, dtype=np.float32).reshape(n, n)     # [cj, ci]
    prob = 1.0 - 1.0 / (1.0 + np.exp(arr))                       # 점유확률
    val = (255.0 * (1.0 - prob)).astype(np.uint8)               # 점유->검정, 자유->흰, 미지->회
    val = np.flipud(val)                                        # +Y 를 위로
    img = cv2.cvtColor(val, cv2.COLOR_GRAY2BGR)

    scale = max(1, DISP // n)
    if scale > 1:
        img = cv2.resize(img, (n * scale, n * scale), interpolation=cv2.INTER_NEAREST)

    rc = grid.origin * scale + scale // 2
    rr = (n - 1 - grid.origin) * scale + scale // 2
    cv2.circle(img, (rc, rr), max(3, scale * 2), (0, 0, 255), -1)   # 로봇(원점)

    _txt(img, "X2 occupancy map  (pose FIXED = stationary)  q quit  c clear",
         (10, 22), (0, 200, 255), 0.5)
    _txt(img, f"{grid.size_m:.0f}m half-extent / {grid.res*100:.0f}cm cell",
         (10, img.shape[0] - 12), (180, 180, 180), 0.45)
    return img


def main():
    ap = argparse.ArgumentParser(description="X2 점유격자 실시간 맵")
    ap.add_argument("--lidar", default="x2", help="LiDAR 모델(x2/x4)")
    ap.add_argument("--port", default=LIDAR_X2_PORT, help="시리얼 포트")
    ap.add_argument("--baud", type=int, default=None)
    ap.add_argument("--size", type=float, default=float(MAP_SIZE_M), help="맵 반경(m)")
    ap.add_argument("--res", type=float, default=float(MAP_RES_M), help="셀 크기(m)")
    args = ap.parse_args()

    import cv2
    from drivers import make_lidar

    lidar = make_lidar(args.lidar, args.port, args.baud)
    if not lidar.open():
        print(f"LiDAR({args.port}) 열기 실패. 포트/모델(--lidar {args.lidar})을 확인하세요.")
        return
    print(f"맵핑 LiDAR={args.lidar} @ {args.port} (정지 가정). 창에서 q=종료, c=초기화")

    grid = OccupancyGrid(args.size, args.res)
    off = LIDAR_X2_OFFSET_DEG if args.lidar == "x2" else 0.0

    warm = time.time() + 3.0
    while time.time() < warm and not lidar.getMeasures():
        time.sleep(0.1)

    win = "X2 occupancy map"
    cv2.namedWindow(win, cv2.WINDOW_AUTOSIZE)
    try:
        while True:
            dd = lidar.getDistanceDict()
            pts = [((a - FORWARD_ANGLE_DEG + off), d) for a, d in dd.items()]
            grid.integrate_scan((0.0, 0.0, 0.0), pts, max_range_m=args.size)
            cv2.imshow(win, render(grid))

            k = cv2.waitKey(30) & 0xFF
            if k == ord("q"):
                break
            elif k == ord("c"):
                grid.log = [0.0] * (grid.n * grid.n)
            if cv2.getWindowProperty(win, cv2.WND_PROP_VISIBLE) < 1:
                break
    except KeyboardInterrupt:
        pass
    finally:
        lidar.close()
        cv2.destroyAllWindows()
        print("종료")


if __name__ == "__main__":
    main()
