"""visualization/color_detect.py — 색상 인식 시스템 (YOLO/사람인식 없음).

카메라에서 '색이 있는 영역(블롭)'을 HSV로 찾아:
  - 그 색의 '테두리'로 두르고  - 무슨 색인지 라벨  - 그 방향 LiDAR 거리(cm)
레이더에는 각 색 물체를 그 색 점으로 그 거리에 표시.

색은 픽셀에서 직접 읽으므로 재학습/가중치 불필요, GPU 없이 빠르다.
규약: cb=(cx/W-0.5)*HFOV / LiDAR각 = FORWARD + sign*cb + cal.
조작: a/d=카메라-LiDAR 보정(cal) | s=좌우부호 | , .=레이더 줌 | q=종료
실행: py -3.13 visualization/color_detect.py   (LiDAR=X4/COM8, 카메라=config)
"""
import sys
import math
import pathlib
import argparse

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))

from common.config import (
    CAMERA_INDEX, CAMERA_HFOV_DEG, CAMERA_LIDAR_SIGN, FORWARD_ANGLE_DEG,
    LIDAR_X4_PORT, DANGER_MM, SLOW_MM,
)
from common.camera import open_camera, crop_center_zoom
from common.fusion import min_distance_in_arc, view_bearing_deg
from common.color import HSV_RANGES, color_mask
from common.lidar_metrics import normalize_deg
from visualization.lidar_probe_view import _txt, ring_step_mm

RMAX_MM = 6000
PANEL = 480
MIN_AREA_FRAC = 0.003     # 프레임 면적의 이 비율보다 작은 색 블롭은 무시(노이즈)

# 테두리/표시에 쓰는 각 색의 대표 BGR
NAME_BGR = {
    "red": (0, 0, 255), "orange": (0, 140, 255), "yellow": (0, 230, 230),
    "green": (0, 200, 0), "cyan": (230, 230, 0), "blue": (255, 0, 0),
    "purple": (200, 0, 160), "pink": (170, 90, 255),
}


def detect_color_objects(frame, sat_min, val_min, min_area):
    """색 블롭 목록 [{name, box=(x1,y1,x2,y2), cx, area}, ...]."""
    import cv2
    import numpy as np
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    kernel = np.ones((5, 5), np.uint8)
    out = []
    for name in HSV_RANGES:
        mask = color_mask(hsv, name, sat_min, val_min, is_hsv=True)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in cnts:
            area = cv2.contourArea(c)
            if area < min_area:
                continue
            x, y, w, h = cv2.boundingRect(c)
            out.append({"name": name, "box": (x, y, x + w, y + h),
                        "cx": x + w / 2.0, "w": w, "area": area})
    return out


def draw_radar(dd, radar_objs, cal, hfov, sign, rmax):
    import cv2
    import numpy as np
    size = PANEL
    img = np.full((size, size, 3), 25, np.uint8)
    cx = cy = size // 2
    max_r = size // 2 - 20
    cv2.circle(img, (cx, cy), max_r, (70, 70, 70), 1)
    step = ring_step_mm(rmax)
    mm = step
    while mm <= rmax + 1:
        rr = int(mm / rmax * max_r)
        cv2.circle(img, (cx, cy), rr, (55, 55, 55), 1)
        _txt(img, f"{mm/10:.0f}cm" if mm < 1000 else f"{mm/1000:.1f}m",
             (cx + 3, cy - rr + 13), (90, 90, 90), 0.38)
        mm += step
    cv2.circle(img, (cx, cy), int(SLOW_MM / rmax * max_r), (0, 140, 210), 1)
    cv2.circle(img, (cx, cy), int(DANGER_MM / rmax * max_r), (0, 0, 210), 1)
    cv2.line(img, (cx, cy), (cx, cy - max_r), (0, 150, 0), 1)
    _txt(img, "front", (cx + 4, cy - max_r + 14), (0, 200, 0), 0.4)
    for cb in (-hfov / 2, hfov / 2):
        sb = normalize_deg(sign * cb + cal)
        rel = math.radians(sb % 360)
        x = int(cx + max_r * math.sin(rel)); y = int(cy - max_r * math.cos(rel))
        cv2.line(img, (cx, cy), (x, y), (255, 255, 0), 1)
    for a, d in dd.items():
        if 0 < d <= rmax:
            rel = math.radians((a - FORWARD_ANGLE_DEG) % 360)
            r = d / rmax * max_r
            x = int(cx + r * math.sin(rel)); y = int(cy - r * math.cos(rel))
            cv2.circle(img, (x, y), 2, (90, 90, 90), -1)
    for la, dist, bgr in radar_objs:
        rel = math.radians((la - FORWARD_ANGLE_DEG) % 360)
        r = min(dist, rmax) / rmax * max_r
        x = int(cx + r * math.sin(rel)); y = int(cy - r * math.cos(rel))
        cv2.circle(img, (x, y), 7, bgr, 2)
    return img


def main():
    ap = argparse.ArgumentParser(description="색상 인식 + LiDAR 거리 (YOLO 없음)")
    ap.add_argument("--lidar", default="x4")
    ap.add_argument("--port", default=LIDAR_X4_PORT)
    ap.add_argument("--baud", type=int, default=None)
    ap.add_argument("--cam-index", type=int, default=CAMERA_INDEX)
    ap.add_argument("--cal", type=float, default=0.0, help="카메라-LiDAR 방위보정(deg)")
    ap.add_argument("--sat", type=int, default=90, help="채도 최소(낮추면 옅은 색도)")
    ap.add_argument("--val", type=int, default=70, help="명도 최소")
    ap.add_argument("--min-area-frac", type=float, default=MIN_AREA_FRAC)
    ap.add_argument("--rmax", type=float, default=float(RMAX_MM))
    args = ap.parse_args()

    import cv2
    import numpy as np
    from drivers import make_lidar

    cap = open_camera(args.cam_index)
    if not cap.isOpened():
        print(f"카메라(index {args.cam_index}) 열기 실패.")
        return
    ok, frame = cap.read()
    if not ok:
        print("카메라 프레임 수신 실패.")
        cap.release()
        return
    lidar = make_lidar(args.lidar, args.port, args.baud)
    lidar_ok = lidar.open()
    if not lidar_ok:
        print(f"[경고] LiDAR({args.port}) 열기 실패 -> 거리 없이 색만 표시.")

    hfov = float(CAMERA_HFOV_DEG)
    cal = [float(args.cal)]
    sign = [float(CAMERA_LIDAR_SIGN)]
    rmax = [max(500.0, float(args.rmax))]
    cam_zoom = [1.0]   # 카메라 디지털 줌(중앙 크롭 배율)

    win = "color detect + distance"
    cv2.namedWindow(win, cv2.WINDOW_AUTOSIZE)
    print("색상 인식 시작. a/d=cal, s=부호, ,.=레이더줌, =/- =카메라줌, q=종료")

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                continue
            dd = lidar.getDistanceDict() if lidar_ok else {}
            z = cam_zoom[0]
            view = crop_center_zoom(frame, z)   # 카메라 디지털 줌(중앙 크롭). 화각=hfov/z
            H, W = view.shape[:2]
            eff_hfov = hfov / z
            min_area = args.min_area_frac * W * H
            objs = detect_color_objects(view, args.sat, args.val, min_area)

            cam = cv2.resize(view, (int(W * PANEL / H), PANEL))
            sx = cam.shape[1] / float(W)
            sy = PANEL / float(H)
            radar_objs = []
            for o in objs:
                x1, y1, x2, y2 = o["box"]
                bgr = NAME_BGR.get(o["name"], (255, 255, 255))
                cb = view_bearing_deg(o["cx"], W, hfov, z)   # 줌 무관 베어링(검증됨)
                la = (FORWARD_ANGLE_DEG + sign[0] * cb + cal[0]) % 360
                half = (o["w"] / float(W)) * eff_hfov / 2.0 + 1.0
                dist = min_distance_in_arc(dd, la, half) if dd else None
                cv2.rectangle(cam, (int(x1 * sx), int(y1 * sy)),
                              (int(x2 * sx), int(y2 * sy)), bgr, 3)
                lbl = o["name"] + (f" {dist/10:.0f}cm" if dist else " -") + f" @{la:.0f}deg"
                _txt(cam, lbl, (int(x1 * sx), max(12, int(y1 * sy) - 6)), bgr, 0.5)
                if dist:
                    radar_objs.append((la, dist, bgr))

            # 카메라 중앙(=정면) 십자선 + 정면 LiDAR 거리 (cal/sign 보정 기준점)
            cxw = cam.shape[1] // 2
            cv2.line(cam, (cxw, 0), (cxw, PANEL), (200, 200, 200), 1)
            la_fwd = (FORWARD_ANGLE_DEG + cal[0]) % 360
            fdist = min_distance_in_arc(dd, la_fwd, 2.0) if dd else None
            _txt(cam, "fwd@center " + (f"{fdist/10:.0f}cm" if fdist else "-"),
                 (cxw + 4, PANEL - 26), (200, 200, 200), 0.5)

            combined = np.hstack([cam, draw_radar(dd, radar_objs, cal[0], eff_hfov, sign[0], rmax[0])])
            lines = [
                f"colors: {len(objs)}  cal{cal[0]:+.0f} sign{sign[0]:+.0f}  range {rmax[0]/1000:.1f}m",
                f"cam zoom {z:.1f}x (eff FOV {eff_hfov:.0f}deg)  | LiDAR " + (args.port if lidar_ok else "OFF"),
            ]
            for i, ln in enumerate(lines):
                _txt(combined, ln, (8, PANEL - 8 - (len(lines) - 1 - i) * 18), (235, 235, 235), 0.45)
            _txt(combined, "a/d cal  s sign  ,. radar-zoom  =/- cam-zoom  q quit",
                 (cam.shape[1] + 8, PANEL - 8), (150, 150, 150), 0.42)

            cv2.imshow(win, combined)
            k = cv2.waitKey(1) & 0xFF
            if k == ord("q"):
                break
            elif k == ord("a"):
                cal[0] -= 1.0
            elif k == ord("d"):
                cal[0] += 1.0
            elif k == ord("s"):
                sign[0] = -sign[0]
            elif k == ord(","):
                rmax[0] = max(500.0, rmax[0] / 1.5)
            elif k == ord("."):
                rmax[0] = min(12000.0, rmax[0] * 1.5)
            elif k in (ord("="), ord("+")):
                cam_zoom[0] = min(5.0, cam_zoom[0] * 1.25)
            elif k == ord("-"):
                cam_zoom[0] = max(1.0, cam_zoom[0] / 1.25)
            if cv2.getWindowProperty(win, cv2.WND_PROP_VISIBLE) < 1:
                break
    except KeyboardInterrupt:
        pass
    finally:
        cap.release()
        if lidar_ok:
            lidar.close()
        cv2.destroyAllWindows()
        print("종료")


if __name__ == "__main__":
    main()
