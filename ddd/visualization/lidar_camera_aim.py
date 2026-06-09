"""visualization/lidar_camera_aim.py — LiDAR 클릭 조준 + 사람 집중감시 + 방위(0~360) 표시.

기능:
  1) 레이더(LiDAR) 좌클릭 -> 카메라 그 방향 조준선 + 그 방향 LiDAR 거리.
  2) 카메라 '사람'(COCO yolov8n) 네모칸 인식 + 각 사람까지 LiDAR 거리.
     거리는 '사람 박스 각도폭 내 최소거리'(전경=사람)로 계산 -> 배경 오염에 강함.
  3) 클릭 방향의 사람을 '집중감시': 마젠타 강조 + 별도 'FOCUS' 창에 확대 표시.
  4) 카메라/LiDAR 방위(0~360deg) 글자 + 레이더에 카메라 FOV 노란 쐐기.

[기하] 카메라를 LiDAR '바로 위'에 올리면 수평 위치가 같아 시차(parallax)가 0 ->
  방위(sign/cal)만 맞으면 거리가 정확하다. 카메라가 더 높은 수직오프셋은 '수평거리'에
  영향을 주지 않는다(LiDAR 평면이 사람의 다리/몸통을 맞은 수평거리 = 그 사람까지 거리).

규약: cb=(cx/W-0.5)*HFOV / LiDAR각 = FORWARD + sign*cb + cal.
조작: 레이더 좌클릭=조준 | 우클릭=해제 | a/d=cal 보정 | s=좌우부호(sign) 뒤집기 | q=종료
실행: py -3.13 visualization/lidar_camera_aim.py   (LiDAR=X4/COM8, 카메라=config)
"""
import sys
import math
import pathlib
import argparse

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))

from common.config import (
    CAMERA_INDEX, CAMERA_HFOV_DEG, CAMERA_LIDAR_SIGN, FORWARD_ANGLE_DEG,
    LIDAR_X4_PORT, DANGER_MM, SLOW_MM, COCO_WEIGHTS_PATH,
)
from common.camera import open_camera
from common.fusion import min_distance_in_arc, view_bearing_deg
from common.color import dominant_color
from common.lidar_metrics import normalize_deg, nearest_point_2d
from common.viz import zone_bgr, _txt, ring_step_mm

RMAX_MM = 6000
PANEL = 480
FOCUS_H = 420        # FOCUS 창 높이(px)
ROI_FRAC = 0.22
SEL_TOL_DEG = 5.0
PERSON_CONF = 0.4
GATE_MM = 700        # 클릭/추적 게이트: 이 거리(mm) 이내의 LiDAR 점에 스냅·추적


# zone_bgr 는 common/viz.py 로 이동(import 됨).


def lidar_angle_of_cb(cb, cal, sign):
    """카메라 bearing(cb, deg) -> LiDAR 각도(0~360)."""
    return (FORWARD_ANGLE_DEG + sign * cb + cal) % 360.0


def draw_radar(dd, track, cal, hfov, sign, rmax):
    import cv2
    import numpy as np
    size = PANEL
    img = np.full((size, size, 3), 25, np.uint8)
    cx = cy = size // 2
    max_r = size // 2 - 20
    cv2.circle(img, (cx, cy), max_r, (70, 70, 70), 1)
    # 거리 가이드 링 (rmax 에 맞춰 자동 세분화) + 라벨
    step = ring_step_mm(rmax)
    mm = step
    while mm <= rmax + 1:
        rr = int(mm / rmax * max_r)
        cv2.circle(img, (cx, cy), rr, (55, 55, 55), 1)
        lbl = f"{mm/10:.0f}cm" if mm < 1000 else f"{mm/1000:.1f}m"
        _txt(img, lbl, (cx + 3, cy - rr + 13), (90, 90, 90), 0.38)
        mm += step
    cv2.circle(img, (cx, cy), int(SLOW_MM / rmax * max_r), (0, 140, 210), 1)
    cv2.circle(img, (cx, cy), int(DANGER_MM / rmax * max_r), (0, 0, 210), 1)
    cv2.line(img, (cx, cy), (cx, cy - max_r), (0, 150, 0), 1)
    _txt(img, "front", (cx + 4, cy - max_r + 14), (0, 200, 0), 0.4)

    for cb in (-hfov / 2, hfov / 2):
        sb = normalize_deg(lidar_angle_of_cb(cb, cal, sign) - FORWARD_ANGLE_DEG)
        rel = math.radians(sb % 360)
        x = int(cx + max_r * math.sin(rel)); y = int(cy - max_r * math.cos(rel))
        cv2.line(img, (cx, cy), (x, y), (255, 255, 0), 1)
    sb_c = normalize_deg(lidar_angle_of_cb(0, cal, sign) - FORWARD_ANGLE_DEG)
    rel = math.radians(sb_c % 360)
    x = int(cx + (max_r - 16) * math.sin(rel)); y = int(cy - (max_r - 16) * math.cos(rel))
    _txt(img, "CAM", (x - 14, y), (255, 255, 0), 0.4)

    for a, d in dd.items():
        if 0 < d <= rmax:
            rel = math.radians((a - FORWARD_ANGLE_DEG) % 360)
            r = d / rmax * max_r
            x = int(cx + r * math.sin(rel)); y = int(cy - r * math.cos(rel))
            cv2.circle(img, (x, y), 2, zone_bgr(d), -1)

    if track is not None:
        ta, td = track
        rel = math.radians((ta - FORWARD_ANGLE_DEG) % 360)
        r = min(td, rmax) / rmax * max_r
        x = int(cx + r * math.sin(rel)); y = int(cy - r * math.cos(rel))
        cv2.circle(img, (x, y), 11, (0, 255, 255), 2)   # 추적 원
        cv2.circle(img, (x, y), 2, (0, 255, 255), -1)
    return img


def main():
    ap = argparse.ArgumentParser(description="LiDAR 클릭 조준 + 사람 집중감시")
    ap.add_argument("--lidar", default="x4")
    ap.add_argument("--port", default=LIDAR_X4_PORT)
    ap.add_argument("--baud", type=int, default=None)
    ap.add_argument("--cam-index", type=int, default=CAMERA_INDEX)
    ap.add_argument("--cal", type=float, default=0.0, help="카메라-LiDAR 방위보정(deg)")
    ap.add_argument("--model", default=str(COCO_WEIGHTS_PATH))
    ap.add_argument("--conf", type=float, default=PERSON_CONF)
    ap.add_argument("--rmax", type=float, default=float(RMAX_MM),
                    help="레이더 표시 최대거리(mm). 창에서 , . 로 줌")
    args = ap.parse_args()

    import cv2
    import numpy as np
    from drivers import make_lidar
    from ultralytics import YOLO

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
    if not lidar.open():
        print(f"LiDAR({args.port}) 열기 실패.")
        cap.release()
        return
    model = YOLO(args.model)

    hfov = float(CAMERA_HFOV_DEG)
    cal = [float(args.cal)]
    sign = [float(CAMERA_LIDAR_SIGN)]
    rmax = [max(500.0, float(args.rmax))]
    state = {"pending": None, "track": None, "cam_w": 0}

    def on_mouse(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN and x >= state["cam_w"]:
            rx = x - state["cam_w"] - PANEL // 2
            ry = y - PANEL // 2
            bearing = math.degrees(math.atan2(rx, -ry)) % 360
            rng = (math.hypot(rx, ry) / max(1, (PANEL // 2 - 20))) * rmax[0]
            state["pending"] = ((FORWARD_ANGLE_DEG + bearing) % 360, rng)  # (lidar각, 거리mm)
        elif event == cv2.EVENT_RBUTTONDOWN:
            state["pending"] = None
            state["track"] = None

    win = "LiDAR aim + person watch"
    focus_win = "FOCUS"
    cv2.namedWindow(win, cv2.WINDOW_AUTOSIZE)
    cv2.namedWindow(focus_win, cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback(win, on_mouse)
    print("레이더 좌클릭=조준 / 우클릭=해제 / a,d=cal / s=부호반전 / q=종료")

    def placeholder():
        img = np.full((240, 320, 3), 40, np.uint8)
        _txt(img, "no focus", (90, 125), (160, 160, 160), 0.7)
        return img

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                continue
            dd = lidar.getDistanceDict()
            H, W = frame.shape[:2]

            # 사람 탐지 + 각 사람의 LiDAR 거리(박스 각도폭 내 '최소거리' = 전경=사람)
            res = model.predict(frame, conf=args.conf, classes=[0], verbose=False)
            persons = []
            for b in res[0].boxes:
                x1, y1, x2, y2 = (int(v) for v in b.xyxy[0])
                pcx = (x1 + x2) / 2.0
                cb_c = view_bearing_deg(pcx, W, hfov, 1.0)   # 핀홀 베어링(공용함수)
                half = ((x2 - x1) / float(W)) * hfov / 2.0 + 1.0
                center = lidar_angle_of_cb(cb_c, cal[0], sign[0])
                dist = min_distance_in_arc(dd, center, half)
                persons.append({"box": (x1, y1, x2, y2), "conf": float(b.conf[0]),
                                "cx": pcx, "dist": dist})

            cam = cv2.resize(frame, (int(W * PANEL / H), PANEL))
            state["cam_w"] = cam.shape[1]
            sx = cam.shape[1] / float(W)
            sy = PANEL / float(H)

            # 클릭/추적: 클릭 지점 근처 LiDAR 점에 '스냅'하고, 이후 그 점을 따라간다(추적).
            if state["pending"] is not None:
                pa, pr = state["pending"]; state["pending"] = None
                ta, td, d2 = nearest_point_2d(dd, pa, pr)
                state["track"] = (ta, td) if (ta is not None and d2 is not None and d2 <= GATE_MM) else None
            elif state["track"] is not None:
                ta, td, d2 = nearest_point_2d(dd, state["track"][0], state["track"][1])
                if ta is not None and d2 is not None and d2 <= GATE_MM:
                    state["track"] = (ta, td)   # 따라가기
            track = state["track"]

            B = None
            xp = None
            aim_dist = None
            if track is not None:
                aim_dist = track[1]
                B = normalize_deg(track[0] - FORWARD_ANGLE_DEG)
                cb_aim = sign[0] * (B - cal[0])
                if abs(cb_aim) <= hfov / 2:
                    xp = int(W * (cb_aim / hfov + 0.5))

            target = None
            if xp is not None and persons:
                target = min(persons, key=lambda p: abs(p["cx"] - xp))
                if abs(target["cx"] - xp) > W * 0.18:
                    target = None

            # 포커스 영역(대상 사람 / 조준 ROI) 크롭 + 그 '대표 색'
            src = None
            roi_bounds = None
            if target is not None:
                x1, y1, x2, y2 = target["box"]
                src = frame[max(0, y1):min(H, y2), max(0, x1):min(W, x2)]
            elif xp is not None:
                rw = max(8, int(W * ROI_FRAC))
                rx0 = max(0, xp - rw // 2); rx1 = min(W, xp + rw // 2)
                src = frame[:, rx0:rx1]
                roi_bounds = (rx0, rx1)
            focus_color = None
            focus_bgr = (0, 255, 255)
            if src is not None and src.size > 0:
                focus_color, focus_bgr = dominant_color(src)

            # 사람 박스: 대상은 '그 물체 색' 테두리(+ this object 라벨), 나머지는 초록
            for p in persons:
                x1, y1, x2, y2 = p["box"]
                col = focus_bgr if p is target else (0, 255, 0)
                th = 4 if p is target else 2
                cv2.rectangle(cam, (int(x1 * sx), int(y1 * sy)),
                              (int(x2 * sx), int(y2 * sy)), col, th)
                lbl = (f"this object: {focus_color}" if p is target
                       else f"person {p['conf']:.2f}" + (f" {p['dist']/10:.0f}cm" if p['dist'] else ""))
                _txt(cam, lbl, (int(x1 * sx), max(12, int(y1 * sy) - 6)), col, 0.5)
            if roi_bounds is not None:   # 사람 없이 추적점만일 때, 그 ROI를 색 테두리로
                cv2.rectangle(cam, (int(roi_bounds[0] * sx), 1),
                              (int(roi_bounds[1] * sx), PANEL - 2), focus_bgr, 3)
                _txt(cam, f"this object: {focus_color}",
                     (int(roi_bounds[0] * sx), 20), focus_bgr, 0.5)
            if xp is not None:
                cv2.line(cam, (int(xp * sx), 0), (int(xp * sx), PANEL), (0, 255, 255), 1)

            # FOCUS 창: 그 부분을 '대표 색 테두리'로 둘러 표시
            focus = None
            if src is not None and src.size > 0:
                fw = max(1, int(FOCUS_H * src.shape[1] / src.shape[0]))
                focus = cv2.resize(src, (fw, FOCUS_H))
                cv2.rectangle(focus, (0, 0), (fw - 1, FOCUS_H - 1), focus_bgr, 12)  # 색 테두리
                _txt(focus, f"this object: {focus_color}", (8, 26), (255, 255, 255), 0.62)
                if target is not None and target["dist"]:
                    _txt(focus, f"{target['dist']/10:.0f}cm", (8, FOCUS_H - 12), (255, 255, 255), 0.55)
            if focus is None:
                focus = placeholder()
            cv2.imshow(focus_win, focus)

            combined = np.hstack([cam, draw_radar(dd, track, cal[0], hfov, sign[0], rmax[0])])

            cam_face = lidar_angle_of_cb(0, cal[0], sign[0])
            cam_l = lidar_angle_of_cb(-hfov / 2, cal[0], sign[0])
            cam_r = lidar_angle_of_cb(hfov / 2, cal[0], sign[0])
            lines = [
                f"LiDAR: 0~360 omni (front {FORWARD_ANGLE_DEG:.0f}deg)  range {rmax[0]/1000:.1f}m",
                f"CAM facing {cam_face:.0f}deg  FOV {cam_l:.0f}~{cam_r:.0f}deg  cal{cal[0]:+.0f} sign{sign[0]:+.0f}",
                f"persons: {len(persons)}",
            ]
            if track is not None:
                lines.append(f"TRACK {track[0]:.0f}deg  {track[1]/10:.0f}cm")
            if target is not None:
                lines.append("WATCH person  " + (f"{target['dist']/10:.0f}cm" if target['dist'] else "-(no LiDAR)"))
            if focus_color is not None:
                lines.append(f"FOCUS color: {focus_color}")
            for i, ln in enumerate(lines):
                _txt(combined, ln, (8, PANEL - 8 - (len(lines) - 1 - i) * 18), (235, 235, 235), 0.45)
            _txt(combined, "L-click track  R-clear  a/d cal  s sign  , . zoom  q quit",
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
            if cv2.getWindowProperty(win, cv2.WND_PROP_VISIBLE) < 1:
                break
    except KeyboardInterrupt:
        pass
    finally:
        cap.release()
        lidar.close()
        cv2.destroyAllWindows()
        print("종료")


if __name__ == "__main__":
    main()
