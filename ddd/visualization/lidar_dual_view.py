"""visualization/lidar_dual_view.py — 두 LiDAR(X4 + X2)를 동시에 한 레이더에 표시.

  - X4(기본 COM8, 128000) = 초록,  X2(기본 COM12, 115200) = 마젠타  (색으로 출처 구분)
  - 각 LiDAR 점 개수 / 신선도(OK·STALE) 범례 표시.
  - 두 LiDAR 는 장착 yaw 가 달라 0deg 기준이 다르다.
    --x2-offset(또는 창에서 a/d 키)로 X2 를 회전해, 같은 물체가 두 색에서
    같은 방향에 오도록 '대략 정렬'한다(러프 외부보정 1단계: 회전).
  - 한쪽만 연결돼도 연결된 것만 표시(degraded).

집계/드라이버는 기존 공용 모듈 재사용(make_lidar, ring_step_mm, _txt).
실행: py -3.13 visualization/lidar_dual_view.py
조작: q 종료 | , . 줌 | a / d : X2 회전(-/+1deg) | [ / ] : X4 회전(-/+1deg)
"""
import sys
import time
import math
import pathlib
import argparse

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))

from common.config import (
    LIDAR_X4_PORT, LIDAR_X4_BAUDRATE, LIDAR_X2_PORT, LIDAR_X2_BAUDRATE,
    LIDAR_MAX_AGE, FORWARD_ANGLE_DEG, DANGER_MM, SLOW_MM,
)
from visualization.lidar_probe_view import ring_step_mm, _txt

RMAX_MM = 2000          # 표시 기본 최대거리(mm)=2m (근거리 잘 보이게). 멀리는 . 줌아웃 / --rmax
PANEL = 760

X4_COLOR = (0, 255, 0)      # green
X2_COLOR = (255, 0, 255)    # magenta


def draw(scans, rmax, info):
    import cv2
    import numpy as np

    size = PANEL
    img = np.full((size, size, 3), 22, np.uint8)
    cx = cy = size // 2
    max_r = size // 2 - 48

    # 거리 가이드 링 (자동 세분화) + 안전 링
    cv2.circle(img, (cx, cy), max_r, (70, 70, 70), 1)
    step = ring_step_mm(rmax)
    mm = step
    while mm <= rmax + 1:
        rr = int(mm / rmax * max_r)
        cv2.circle(img, (cx, cy), rr, (55, 55, 55), 1)
        lbl = f"{mm/10:.0f}cm" if mm < 1000 else f"{mm/1000:.1f}m"
        _txt(img, lbl, (cx + 3, cy - rr + 14), (90, 90, 90), 0.4)
        mm += step
    cv2.circle(img, (cx, cy), int(SLOW_MM / rmax * max_r), (0, 140, 210), 1)
    cv2.circle(img, (cx, cy), int(DANGER_MM / rmax * max_r), (0, 0, 210), 1)
    cv2.line(img, (cx, cy), (cx, cy - max_r), (0, 150, 0), 1)
    _txt(img, "front", (cx + 5, cy - max_r + 16), (0, 200, 0), 0.45)

    # 측정점 (LiDAR 별 색 + 각도 오프셋)
    for measures, color, off in scans:
        for a, d in measures:
            if not (0 < d <= rmax):
                continue
            rel = math.radians((a - FORWARD_ANGLE_DEG + off) % 360)
            r = d / rmax * max_r
            x = int(cx + r * math.sin(rel)); y = int(cy - r * math.cos(rel))
            cv2.circle(img, (x, y), 2, color, -1)

    # 범례
    y = 28
    for name, color, pts, fresh, port, off in info:
        _txt(img, f"{name} {port}  pts:{pts}  [{'OK' if fresh else 'STALE'}]  off{off:+.0f}deg",
             (14, y), color, 0.55)
        y += 26
    _txt(img, f"range {rmax/1000:.1f}m", (14, y), (235, 235, 235), 0.5)
    _txt(img, "q quit  , . zoom  a/d X2 rot  [ ] X4 rot", (14, size - 12),
         (160, 160, 160), 0.45)
    return img


def main():
    ap = argparse.ArgumentParser(description="두 LiDAR(X4+X2) 동시 표시")
    ap.add_argument("--x4-port", default=LIDAR_X4_PORT)
    ap.add_argument("--x2-port", default=LIDAR_X2_PORT)
    ap.add_argument("--x4-offset", type=float, default=0.0, help="X4 각도 오프셋(deg)")
    ap.add_argument("--x2-offset", type=float, default=0.0, help="X2 각도 오프셋(deg)")
    ap.add_argument("--rmax", type=float, default=float(RMAX_MM))
    args = ap.parse_args()

    import cv2
    from drivers import make_lidar

    x4 = make_lidar("x4", args.x4_port, LIDAR_X4_BAUDRATE)
    x2 = make_lidar("x2", args.x2_port, LIDAR_X2_BAUDRATE)
    if not x4.open():
        print(f"[경고] X4({args.x4_port}) 열기 실패 -> 표시 제외")
        x4 = None
    if not x2.open():
        print(f"[경고] X2({args.x2_port}) 열기 실패 -> 표시 제외")
        x2 = None
    if x4 is None and x2 is None:
        print("두 LiDAR 모두 열기 실패. 포트(--x4-port/--x2-port)를 확인하세요.")
        return
    print(f"X4={args.x4_port}(green)  X2={args.x2_port}(magenta)  창에서 q=종료")

    rmax = max(500.0, args.rmax)
    off = {"X4": args.x4_offset, "X2": args.x2_offset}

    # 워밍업
    end = time.time() + 3.0
    while time.time() < end and not ((x4 and x4.getMeasures()) or (x2 and x2.getMeasures())):
        time.sleep(0.1)

    win = "dual LiDAR view"
    cv2.namedWindow(win, cv2.WINDOW_AUTOSIZE)
    try:
        while True:
            scans, info = [], []
            if x4 is not None:
                scans.append((x4.getMeasures(), X4_COLOR, off["X4"]))
                info.append(("X4", X4_COLOR, len(x4.getDistanceDict()),
                             x4.is_fresh(LIDAR_MAX_AGE), args.x4_port, off["X4"]))
            if x2 is not None:
                scans.append((x2.getMeasures(), X2_COLOR, off["X2"]))
                info.append(("X2", X2_COLOR, len(x2.getDistanceDict()),
                             x2.is_fresh(LIDAR_MAX_AGE), args.x2_port, off["X2"]))

            img = draw(scans, rmax, info)
            cv2.imshow(win, img)

            k = cv2.waitKey(30) & 0xFF
            if k == ord("q"):
                break
            elif k == ord(","):
                rmax = max(500.0, rmax / 1.5)
            elif k == ord("."):
                rmax = min(12000.0, rmax * 1.5)
            elif k == ord("a"):
                off["X2"] -= 1.0
            elif k == ord("d"):
                off["X2"] += 1.0
            elif k == ord("["):
                off["X4"] -= 1.0
            elif k == ord("]"):
                off["X4"] += 1.0

            if cv2.getWindowProperty(win, cv2.WND_PROP_VISIBLE) < 1:
                break
    except KeyboardInterrupt:
        pass
    finally:
        if x4 is not None:
            x4.close()
        if x2 is not None:
            x2.close()
        cv2.destroyAllWindows()
        print("종료")


if __name__ == "__main__":
    main()
