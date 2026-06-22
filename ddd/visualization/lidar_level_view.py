"""lidar_level_view.py — 라이다 재배치(레벨링/전방각) 전용 시각화. 카메라/모터 없음.

목적: 라이다를 다시 장착/수평 조정할 때 화면만 보고 맞춘다.
보여주는 것:
  - 레이더(로봇 기준, 위=정면). 점은 config.LIDAR_FLIPPED/FORWARD_ANGLE_DEG 반영.
  - 정면 ±arc 최소거리(큰 글씨) + '바닥 호(floor arc)' 기울기 추정.
      바닥 호 원리: 라이다가 아래로 θ 기울면 빔이 (높이 h)/tan(θ) 앞 '바닥'에 닿아
      정면에 호가 생긴다. floor_dist 가 가까울수록 더 기운 것. 목표=호를 멀리/없앰.
      θ ≈ atan(h / floor_dist).  (h = --lidar-height-mm, 기본 40mm)
  - 최근접 점의 raw 각도/거리(마젠타) -> 정면에 물체 하나 두면 그 raw 각도 = FORWARD_ANGLE_DEG.

조작:  [ = 범위축소(줌인) / ] = 범위확대  |  q 또는 ESC = 종료
실행:  py -3.13 visualization/lidar_level_view.py --lidar x4 --port COM8
       (정면에 물체 하나 두고 1.0/1.5/2.0m 로 옮기며 잡히는지 확인. 마운트 한 변에
        얇은 심을 끼워 '바닥 호'를 1.75m 이상으로 밀어내면 ±1.3° 이내로 수평.)
"""
import argparse
import math
import sys
import pathlib

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))

from common.config import LIDAR_X4_PORT, LIDAR_X4_BAUDRATE, FORWARD_ANGLE_DEG  # noqa: E402
from common.fusion import lidar_bearing, min_distance_in_arc                   # noqa: E402


def main():
    ap = argparse.ArgumentParser(description="라이다 레벨링/전방각 시각화 (LiDAR 전용)")
    ap.add_argument("--lidar", default="x4", choices=["x2", "x4"])
    ap.add_argument("--port", default=LIDAR_X4_PORT)
    ap.add_argument("--baud", type=int, default=LIDAR_X4_BAUDRATE)
    ap.add_argument("--rmax", type=float, default=2500.0, help="레이더 표시 최대거리(mm)")
    ap.add_argument("--front-arc", type=float, default=15.0, help="정면 부채꼴 반각(deg)")
    ap.add_argument("--lidar-height-mm", type=float, default=40.0, help="바닥에서 라이다 높이(mm)")
    ap.add_argument("--size", type=int, default=720, help="레이더 창 픽셀")
    args = ap.parse_args()

    import cv2
    import numpy as np
    from drivers import make_lidar

    lidar = make_lidar(args.lidar, args.port, args.baud)
    if not lidar.open():
        print(f"[실패] LiDAR({args.port}) 열기 실패. 포트/모델(--lidar)/점유 확인.")
        return
    print(f"[OK] LiDAR({args.lidar}) {args.port}.  [ ] 범위, q 종료. 정면에 물체 하나 두고 레벨링.")

    SZ = args.size
    cx = cy = SZ // 2
    maxr = SZ // 2 - 16
    rmax = [max(500.0, float(args.rmax))]
    win = "lidar level view"
    cv2.namedWindow(win)

    def txt(img, s, org, color, sc=0.5, th=1):
        cv2.putText(img, s, org, cv2.FONT_HERSHEY_SIMPLEX, sc, color, th, cv2.LINE_AA)

    try:
        while True:
            dd = lidar.getDistanceDict(freshest=True)
            fresh = lidar.is_fresh(0.5)
            rm = rmax[0]
            scale = maxr / rm
            img = np.zeros((SZ, SZ, 3), np.uint8)

            # 거리 링
            ring = 500 if rm > 2000 else (250 if rm > 1000 else 100)
            r = ring
            while r <= rm + 1:
                cv2.circle(img, (cx, cy), int(r * scale), (45, 45, 45), 1)
                txt(img, f"{r/10:.0f}cm" if r < 1000 else f"{r/1000:.1f}m",
                    (cx + 4, cy - int(r * scale) + 14), (80, 80, 80), 0.4)
                r += ring
            # 정면 축(위) + 정면 부채꼴
            cv2.line(img, (cx, cy), (cx, cy - maxr), (0, 110, 0), 1)
            for s in (-1, 1):
                ang = math.radians(s * args.front_arc)
                ex = int(cx + math.sin(ang) * maxr); ey = int(cy - math.cos(ang) * maxr)
                cv2.line(img, (cx, cy), (ex, ey), (0, 70, 0), 1)

            # 점 + 최근접 찾기
            near_a, near_d = None, None
            for a, d in dd.items():
                if d <= 0 or d > rm:
                    continue
                b = math.radians(lidar_bearing(a))
                px = int(cx + d * scale * math.sin(b)); py = int(cy - d * scale * math.cos(b))
                if 0 <= px < SZ and 0 <= py < SZ:
                    cv2.circle(img, (px, py), 2, (180, 180, 180), -1)
                if near_d is None or d < near_d:
                    near_a, near_d = a, d

            # 정면 최소거리 + 기울기 추정(바닥 호)
            fmin = min_distance_in_arc(dd, FORWARD_ANGLE_DEG, args.front_arc) if fresh else None
            txt(img, f"LiDAR {'OK' if fresh else 'STALE'}  range {rm/1000:.1f}m  ([ ] 조절)",
                (10, 24), (0, 230, 230), 0.5)
            if fmin is not None:
                tilt = math.degrees(math.atan(args.lidar_height_mm / fmin))
                col = (0, 200, 0) if fmin >= 1750 else ((0, 200, 255) if fmin >= 1000 else (0, 80, 255))
                txt(img, f"front min: {fmin/10:.0f} cm", (10, SZ - 64), col, 0.8, 2)
                txt(img, f"-> tilt ~ {tilt:.1f} deg (h={args.lidar_height_mm:.0f}mm). 목표: front min >= 175cm(<=1.3deg)",
                    (10, SZ - 38), col, 0.5)
            else:
                txt(img, "front min: (정면 부채꼴에 점 없음 = 평면이 정면을 비움)",
                    (10, SZ - 64), (0, 200, 0), 0.6)

            # 최근접 점(전방각 보정)
            if near_a is not None:
                nb = math.radians(lidar_bearing(near_a))
                npx = int(cx + near_d * scale * math.sin(nb)); npy = int(cy - near_d * scale * math.cos(nb))
                cv2.circle(img, (npx, npy), 6, (255, 0, 255), 2)
                txt(img, f"nearest raw={near_a} bearing={lidar_bearing(near_a):+.0f}deg {near_d/10:.0f}cm"
                    f"  -> 정면 물체면 FORWARD_ANGLE_DEG={near_a}", (10, SZ - 12), (255, 0, 255), 0.45)

            cv2.imshow(win, img)
            k = cv2.waitKey(30) & 0xFF
            if k in (ord('q'), 27):
                break
            elif k == ord('['):
                rmax[0] = max(500.0, rmax[0] / 1.3)
            elif k == ord(']'):
                rmax[0] = min(12000.0, rmax[0] * 1.3)
    except KeyboardInterrupt:
        pass
    finally:
        lidar.close()
        cv2.destroyAllWindows()
        print("종료")


if __name__ == "__main__":
    main()
