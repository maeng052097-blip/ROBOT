"""카메라 + LiDAR 통합 시각화 (OpenCV 단일 창).

한 cv2 창에 (좌) 카메라 + YOLO 박스 + 거리, (우) LiDAR 레이더를 함께 표시.
거리 측정:
  - 레이더 영역을 '좌클릭' -> 그 방향에서 가장 가까운 점의 거리(cm)를 표시.
  - 카메라가 물체를 인식하면 -> 박스에 거리(cm).
matplotlib 대신 cv2.imshow 라 창이 안정적으로 뜬다.

필요: 카메라(필수) + (권장)LiDAR.  (Arduino 불필요)
실행: python visualization/realtime_view.py   ('q' 또는 창 닫기 = 종료)
"""
import sys
import pathlib
import math

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))

from common.config import (
    CAMERA_INDEX, LIDAR_PORT, LIDAR_BAUDRATE, LIDAR_MAX_AGE, WEIGHTS_PATH,
    FORWARD_ANGLE_DEG, DANGER_MM, SLOW_MM,
)
from common.camera import open_camera
from common.fusion import object_distance_mm, bearing_to_lidar_angle
from drivers.LidarX2 import LidarX2
from inference.detector import TargetDetector

RMAX_MM = 6000          # 레이더 표시 최대 거리(mm)
SELECT_TOL_DEG = 6      # 클릭 방향에서 이 각도 이내의 점을 그 방향 거리로 본다
PANEL = 600             # 레이더/표시 높이(px)


def zone_bgr(d):
    if d < DANGER_MM:
        return (0, 0, 255)      # red
    if d < SLOW_MM:
        return (0, 165, 255)    # orange
    return (0, 200, 0)          # green


def nearest(distance_dict, target_deg):
    """target_deg 에 각도상 가장 가까운 (angle, distance, diff)."""
    best_a = best_d = best_diff = None
    for a, d in distance_dict.items():
        if d <= 0:
            continue
        diff = abs((a - target_deg + 180) % 360 - 180)
        if best_diff is None or diff < best_diff:
            best_diff, best_a, best_d = diff, a, d
    return best_a, best_d, best_diff


def draw_radar(dd, lidar_alive, target_angle):
    import cv2
    import numpy as np

    size = PANEL
    img = np.full((size, size, 3), 25, dtype=np.uint8)
    cx = cy = size // 2
    max_r = size // 2 - 26

    # 거리 가이드 링
    cv2.circle(img, (cx, cy), max_r, (70, 70, 70), 1)
    cv2.circle(img, (cx, cy), int(SLOW_MM / RMAX_MM * max_r), (0, 140, 210), 1)
    cv2.circle(img, (cx, cy), int(DANGER_MM / RMAX_MM * max_r), (0, 0, 210), 1)
    cv2.line(img, (cx, cy), (cx, cy - max_r), (0, 200, 0), 1)
    cv2.putText(img, "front", (cx + 4, cy - max_r + 16),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 200, 0), 1)

    # 측정점
    for a, d in dd.items():
        if 0 < d <= RMAX_MM:
            rel = math.radians((a - FORWARD_ANGLE_DEG) % 360)
            r = d / RMAX_MM * max_r
            x = int(cx + r * math.sin(rel))
            y = int(cy - r * math.cos(rel))
            cv2.circle(img, (x, y), 2, zone_bgr(d), -1)

    # 클릭 선택 방향
    if target_angle is not None:
        na, nd, ndiff = nearest(dd, target_angle)
        if na is not None and ndiff is not None and ndiff <= SELECT_TOL_DEG:
            rel = math.radians((na - FORWARD_ANGLE_DEG) % 360)
            r = nd / RMAX_MM * max_r
            x = int(cx + r * math.sin(rel))
            y = int(cy - r * math.cos(rel))
            cv2.circle(img, (x, y), 9, (255, 90, 0), 2)
            cv2.putText(img, f"{na}deg {nd/10:.0f}cm", (12, size - 16),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 120, 0), 2)
        else:
            cv2.putText(img, f"{target_angle}deg (no reading)", (12, size - 16),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 120, 0), 2)

    if not lidar_alive:
        cv2.putText(img, "LiDAR not connected", (20, cy),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
    return img


def main():
    import cv2
    import numpy as np

    cap = open_camera(CAMERA_INDEX)
    if not cap.isOpened():
        print(f"카메라(index {CAMERA_INDEX}) 열기 실패.")
        return
    ok, frame = cap.read()
    if not ok:
        print("카메라 프레임 수신 실패.")
        cap.release()
        return

    lidar = LidarX2(LIDAR_PORT, LIDAR_BAUDRATE)
    if not lidar.open():
        print(f"[경고] LiDAR({LIDAR_PORT}) 연결 실패 -> 레이더 '미연결' 표시.")
        lidar = None

    detector = None
    if WEIGHTS_PATH.exists():
        try:
            detector = TargetDetector()
        except Exception as exc:
            print(f"YOLO 로드 실패(영상+레이더만): {exc}")

    state = {"target": None, "cam_w": 0}

    def on_mouse(event, x, y, flags, param):
        if event != cv2.EVENT_LBUTTONDOWN or lidar is None:
            return
        if x < state["cam_w"]:      # 왼쪽 카메라 영역 클릭은 무시
            return
        rx, ry = x - state["cam_w"], y
        cx = cy = PANEL // 2
        rel = math.degrees(math.atan2(rx - cx, -(ry - cy))) % 360
        state["target"] = round((FORWARD_ANGLE_DEG + rel) % 360)

    win = "camera + LiDAR  (left-click radar=measure, q=quit)"
    cv2.namedWindow(win, cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback(win, on_mouse)
    print("준비됨. 창이 떴습니다. 레이더 좌클릭=거리측정, 'q'=종료")

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                continue
            dd = lidar.getDistanceDict() if lidar is not None else {}

            if detector is not None:
                info = detector.detect(frame)
                if info is not None:
                    x1, y1, x2, y2 = (int(v) for v in info["box"])
                    dist = object_distance_mm(info["bearing_deg"], dd) if dd else None
                    lbl = f"{info['label']} " + (f"{dist/10:.0f}cm" if dist else "no-LiDAR")
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(frame, lbl, (x1, max(20, y1 - 8)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

            h, w = frame.shape[:2]
            cam_w = int(w * PANEL / h)
            cam = cv2.resize(frame, (cam_w, PANEL))
            state["cam_w"] = cam_w
            radar = draw_radar(dd, lidar is not None, state["target"])
            combined = np.hstack([cam, radar])

            if lidar is not None:
                fresh = "OK" if lidar.is_fresh(LIDAR_MAX_AGE) else "STALE"
                cv2.putText(combined, f"LiDAR {LIDAR_PORT}  pts:{len(dd)}  [{fresh}]",
                            (cam_w + 8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)

            cv2.imshow(win, combined)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            if cv2.getWindowProperty(win, cv2.WND_PROP_VISIBLE) < 1:
                break
    except KeyboardInterrupt:
        pass
    finally:
        cap.release()
        if lidar is not None:
            lidar.close()
        cv2.destroyAllWindows()
        print("\n종료")


if __name__ == "__main__":
    main()
