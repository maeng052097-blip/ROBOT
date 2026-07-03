"""visualization/dual_camera_aim.py — LiDAR 양 옆 2카메라 + 색상 + 시차보정 거리.

좌/우 카메라(LiDAR에서 각각 10cm 옆)로 색 물체를 인식하고, 각 카메라의 '시선(ray)'에
놓인 LiDAR 점을 찾아 거리를 '시차(parallax) 보정'해 계산한다(줌해도 정확).
카메라 영역을 클릭하면 그 색 물체를 FOCUS 창에 확대 + 색 테두리 + 거리.

각 물체 라벨 = "색 거리cm @LiDAR각도", 카메라 중앙 십자선 + 정면거리(보정 기준점)도 표시.

규약: cb=(cx/W-0.5)*(HFOV/zoom);  ray방위 = sign*cb + cal;  거리=distance_along_ray.
조작: 좌/우 카메라 클릭=포커스 | a/d=cal | s=부호 | =/- 카메라줌 | ,.=레이더줌 | q
실행: py -3.13 visualization/dual_camera_aim.py --left 1 --right 2
"""
import sys
import math
import pathlib
import argparse

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))

from common.config import (
    CAMERA_HFOV_DEG, CAMERA_LIDAR_SIGN, FORWARD_ANGLE_DEG,
    LIDAR_X4_PORT, DANGER_MM, SLOW_MM,
    CAM_LEFT_INDEX, CAM_RIGHT_INDEX, CAM_SIDE_OFFSET_MM,
)
from common.camera import open_camera, crop_center_zoom
from common.color import dominant_color
from common.fusion import view_bearing_deg, distance_along_ray
from visualization.lidar_probe_view import _txt, ring_step_mm
from visualization.color_detect import detect_color_objects, NAME_BGR

RMAX_MM = 6000
PANEL = 420
FOCUS_H = 360


def placeholder(text):
    import numpy as np
    img = np.full((PANEL, int(PANEL * 1.3), 3), 35, np.uint8)
    _txt(img, text, (20, PANEL // 2), (150, 150, 150), 0.6)
    return img


def process_cam(frame, dd, off_x, zoom, hfov, sign, cal, sat, val, min_frac, perp):
    """카메라 한 대 처리 -> (패널이미지, [포커스용 결과], [레이더용 (la,dist,bgr)])."""
    import cv2
    view = crop_center_zoom(frame, zoom)   # 카메라 디지털 줌(중앙 크롭)
    H, W = view.shape[:2]
    objs = detect_color_objects(view, sat, val, min_frac * W * H)
    panel = cv2.resize(view, (int(W * PANEL / H), PANEL))
    sxp = panel.shape[1] / float(W); syp = PANEL / float(H)

    results, radar_objs = [], []
    for o in objs:
        x1, y1, x2, y2 = o["box"]
        bgr = NAME_BGR.get(o["name"], (255, 255, 255))
        cb = view_bearing_deg(o["cx"], W, hfov, zoom)
        rb = sign * cb + cal
        dist, la = distance_along_ray(dd, off_x, 0.0, rb, FORWARD_ANGLE_DEG, perp) if dd else (None, None)
        dbox = (int(x1 * sxp), int(y1 * syp), int(x2 * sxp), int(y2 * syp))
        cv2.rectangle(panel, (dbox[0], dbox[1]), (dbox[2], dbox[3]), bgr, 3)
        lbl = o["name"] + (f" {dist/10:.0f}cm" if dist else " -") + (f" @{la:.0f}" if la is not None else "")
        _txt(panel, lbl, (dbox[0], max(12, dbox[1] - 6)), bgr, 0.45)
        results.append({"crop": view[max(0, y1):y2, max(0, x1):x2], "name": o["name"],
                        "dist": dist, "dbox": dbox})
        if dist:
            radar_objs.append((la, dist, bgr))

    cxw = panel.shape[1] // 2
    cv2.line(panel, (cxw, 0), (cxw, PANEL), (200, 200, 200), 1)
    fdist, _ = distance_along_ray(dd, off_x, 0.0, cal, FORWARD_ANGLE_DEG, perp) if dd else (None, None)
    _txt(panel, "fwd " + (f"{fdist/10:.0f}cm" if fdist else "-"), (cxw + 4, PANEL - 10),
         (200, 200, 200), 0.45)
    _txt(panel, f"zoom {zoom:.1f}x", (8, 18), (0, 230, 230), 0.45)
    return panel, results, radar_objs


def draw_radar(dd, radar_objs, rmax):
    import cv2
    import numpy as np
    size = PANEL
    img = np.full((size, size, 3), 25, np.uint8)
    cx = cy = size // 2
    max_r = size // 2 - 18
    cv2.circle(img, (cx, cy), max_r, (70, 70, 70), 1)
    step = ring_step_mm(rmax)
    mm = step
    while mm <= rmax + 1:
        rr = int(mm / rmax * max_r)
        cv2.circle(img, (cx, cy), rr, (55, 55, 55), 1)
        _txt(img, f"{mm/10:.0f}cm" if mm < 1000 else f"{mm/1000:.1f}m",
             (cx + 3, cy - rr + 12), (90, 90, 90), 0.36)
        mm += step
    cv2.circle(img, (cx, cy), int(SLOW_MM / rmax * max_r), (0, 140, 210), 1)
    cv2.circle(img, (cx, cy), int(DANGER_MM / rmax * max_r), (0, 0, 210), 1)
    cv2.line(img, (cx, cy), (cx, cy - max_r), (0, 150, 0), 1)
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
    ap = argparse.ArgumentParser(description="2카메라 + 색상 + 시차보정 거리")
    ap.add_argument("--left", type=int, default=CAM_LEFT_INDEX)
    ap.add_argument("--right", type=int, default=CAM_RIGHT_INDEX)
    ap.add_argument("--lidar", default="x4")
    ap.add_argument("--port", default=LIDAR_X4_PORT)
    ap.add_argument("--baud", type=int, default=None)
    ap.add_argument("--offset", type=float, default=float(CAM_SIDE_OFFSET_MM),
                    help="각 카메라의 LiDAR 가로 오프셋(mm)")
    ap.add_argument("--cal", type=float, default=0.0)
    ap.add_argument("--sat", type=int, default=90)
    ap.add_argument("--val", type=int, default=70)
    ap.add_argument("--min-area-frac", type=float, default=0.003)
    ap.add_argument("--perp", type=float, default=200.0, help="시선-점 수직 허용(mm)")
    ap.add_argument("--rmax", type=float, default=float(RMAX_MM))
    args = ap.parse_args()

    import cv2
    import numpy as np
    from drivers import make_lidar

    capL = open_camera(args.left)
    capR = open_camera(args.right)
    okL0 = capL.isOpened()
    okR0 = capR.isOpened()
    if not okL0 and not okR0:
        print("좌/우 카메라 둘 다 열기 실패. --left/--right 인덱스를 확인하세요.")
        return
    if not okL0:
        print(f"[경고] 좌 카메라(index {args.left}) 실패")
    if not okR0:
        print(f"[경고] 우 카메라(index {args.right}) 실패")

    lidar = make_lidar(args.lidar, args.port, args.baud)
    lidar_ok = lidar.open()
    if not lidar_ok:
        print(f"[경고] LiDAR({args.port}) 실패 -> 색만, 거리 없음")

    hfov = float(CAMERA_HFOV_DEG)
    cal = [float(args.cal)]
    sign = [float(CAMERA_LIDAR_SIGN)]
    rmax = [max(500.0, float(args.rmax))]
    zoom = [1.0]
    state = {"focus": None, "panels": []}   # panels: [(x0, x1, results), ...]

    def on_mouse(event, x, y, flags, param):
        if event != cv2.EVENT_LBUTTONDOWN:
            if event == cv2.EVENT_RBUTTONDOWN:
                state["focus"] = None
            return
        for x0, x1, results in state["panels"]:
            if x0 <= x < x1:
                lx = x - x0
                for r in results:
                    bx0, by0, bx1, by1 = r["dbox"]
                    if bx0 <= lx <= bx1 and by0 <= y <= by1:
                        state["focus"] = r
                        return

    win = "dual camera + color + distance"
    focus_win = "FOCUS"
    cv2.namedWindow(win, cv2.WINDOW_AUTOSIZE)
    cv2.namedWindow(focus_win, cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback(win, on_mouse)
    print("좌/우 카메라 클릭=포커스, a/d=cal, s=부호, =/- 줌, ,.=레이더줌, q=종료")

    try:
        while True:
            dd = lidar.getDistanceDict() if lidar_ok else {}
            z = zoom[0]
            radar_all = []
            panel_imgs = []
            panels_meta = []
            x_cursor = 0
            for cap, ok0, off_x, label in ((capL, okL0, -args.offset, "LEFT"),
                                           (capR, okR0, +args.offset, "RIGHT")):
                ok, frame = (cap.read() if ok0 else (False, None))
                if ok:
                    p, results, robj = process_cam(frame, dd, off_x, z, hfov, sign[0],
                                                   cal[0], args.sat, args.val,
                                                   args.min_area_frac, args.perp)
                    radar_all += robj
                else:
                    p, results = placeholder(label + " cam off"), []
                _txt(p, label, (p.shape[1] - 70, 18), (180, 180, 180), 0.5)
                panels_meta.append((x_cursor, x_cursor + p.shape[1], results))
                panel_imgs.append(p)
                x_cursor += p.shape[1]
            state["panels"] = panels_meta

            radar = draw_radar(dd, radar_all, rmax[0])
            combined = np.hstack(panel_imgs + [radar])
            _txt(combined, f"cal{cal[0]:+.0f} sign{sign[0]:+.0f} zoom{z:.1f}x off{args.offset:.0f}mm "
                 f"range{rmax[0]/1000:.1f}m  LiDAR " + (args.port if lidar_ok else "OFF"),
                 (8, PANEL - 8), (235, 235, 235), 0.45)

            # FOCUS 창
            f = state["focus"]
            if f is not None and f["crop"] is not None and getattr(f["crop"], "size", 0) > 0:
                cname, cbgr = dominant_color(f["crop"])
                fw = max(1, int(FOCUS_H * f["crop"].shape[1] / f["crop"].shape[0]))
                fimg = cv2.resize(f["crop"], (fw, FOCUS_H))
                cv2.rectangle(fimg, (0, 0), (fw - 1, FOCUS_H - 1), cbgr, 12)
                _txt(fimg, f"this object: {cname}", (8, 26), (255, 255, 255), 0.6)
                _txt(fimg, (f"{f['dist']/10:.0f}cm" if f["dist"] else "dist -"),
                     (8, FOCUS_H - 12), (255, 255, 255), 0.55)
                cv2.imshow(focus_win, fimg)
            else:
                cv2.imshow(focus_win, placeholder("click a color object"))

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
            elif k in (ord("="), ord("+")):
                zoom[0] = min(5.0, zoom[0] * 1.25)
            elif k == ord("-"):
                zoom[0] = max(1.0, zoom[0] / 1.25)
            elif k == ord(","):
                rmax[0] = max(500.0, rmax[0] / 1.5)
            elif k == ord("."):
                rmax[0] = min(12000.0, rmax[0] * 1.5)
            if cv2.getWindowProperty(win, cv2.WND_PROP_VISIBLE) < 1:
                break
    except KeyboardInterrupt:
        pass
    finally:
        if okL0:
            capL.release()
        if okR0:
            capR.release()
        if lidar_ok:
            lidar.close()
        cv2.destroyAllWindows()
        print("종료")


if __name__ == "__main__":
    main()
