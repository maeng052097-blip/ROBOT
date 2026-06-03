"""카메라 + LiDAR 융합 테스트: 탐지한 물체까지의 거리를 화면에 표시.

흐름: 카메라로 물체 탐지 -> 베어링(각도) 계산 -> LiDAR 로 그 방향 거리 조회 -> 오버레이.
필요: 카메라 + LiDAR 연결, best.pt(가중치).
실행: python tests/test_fusion.py   (창에서 q 또는 Ctrl+C 로 종료)

보정:
  - 거리는 맞는데 좌/우가 반대로 잡히면 config 의 CAMERA_LIDAR_SIGN 을 -1 로.
  - 거리가 계속 'no reading'이면: ① 물체가 LiDAR 스캔 높이에 안 걸림(2D 한계)
    ② FUSION_TOL_DEG 를 키우기 ③ CAMERA_HFOV_DEG/FORWARD_ANGLE_DEG 보정.
"""
import sys
import pathlib
import time

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))

from common.config import (
    CAMERA_INDEX, LIDAR_PORT, LIDAR_BAUDRATE, WEIGHTS_PATH,
)
from common.camera import open_camera
from common.fusion import bearing_to_lidar_angle, object_distance_mm
from drivers.LidarX2 import LidarX2
from inference.detector import TargetDetector

SHOW_WINDOW = True


def main():
    import cv2

    lidar = LidarX2(LIDAR_PORT, LIDAR_BAUDRATE)
    if not lidar.open():
        print(f"LiDAR({LIDAR_PORT}) 연결 실패. config.py 의 LIDAR_PORT 를 확인하세요.")
        return
    if not WEIGHTS_PATH.exists():
        print(f"가중치 없음: {WEIGHTS_PATH} (models/weights/best.pt 에 배치 필요)")
        lidar.close()
        return
    try:
        detector = TargetDetector()
    except Exception as exc:
        print(f"YOLO 로드 실패: {exc}")
        lidar.close()
        return
    cap = open_camera(CAMERA_INDEX)
    if not cap.isOpened():
        print(f"카메라(index {CAMERA_INDEX}) 열기 실패.")
        lidar.close()
        return

    print("카메라+LiDAR 융합 테스트 시작. 창에서 q 또는 Ctrl+C 로 종료.")
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                time.sleep(0.05)
                continue

            info = detector.detect(frame)
            if info is not None:
                bd = info["bearing_deg"]
                la = bearing_to_lidar_angle(bd)
                dist = object_distance_mm(bd, lidar.getDistanceDict())
                dist_txt = f"{dist/10:.0f} cm" if dist else "LiDAR: no reading"
                print(f"  {info['label']:11} conf={info['conf']:.2f} "
                      f"bearing={bd:+5.0f}deg (lidar {la:5.0f}deg) -> {dist_txt}      ", end="\r")
                if SHOW_WINDOW:
                    x1, y1, x2, y2 = (int(v) for v in info["box"])
                    short = f"{dist/10:.0f}cm" if dist else "no-LiDAR"
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(frame, f"{info['label']} {short}", (x1, max(20, y1 - 8)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            elif SHOW_WINDOW:
                cv2.putText(frame, "no target", (20, 40),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)

            if SHOW_WINDOW:
                cv2.imshow("fusion test (q to quit)", frame)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
    except KeyboardInterrupt:
        pass
    finally:
        cap.release()
        if SHOW_WINDOW:
            cv2.destroyAllWindows()
        lidar.close()
        print("\n종료")


if __name__ == "__main__":
    main()
