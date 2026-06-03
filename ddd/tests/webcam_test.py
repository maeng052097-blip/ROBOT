"""웹캠 + YOLO 탐지 스모크 테스트.

카메라를 열고, 가중치(best.pt)가 있으면 매 프레임 탐지하여
목표 라벨 / 신뢰도 / 방향(LEFT/CENTER/RIGHT/NONE)을 출력한다.
가중치가 없으면 카메라 프레임 수신만 확인한다.

방향 판단은 통합 컨트롤러와 동일한 inference.detector 를 쓴다.
GUI 창 표시는 SHOW_WINDOW 로 켜고 끈다(헤드리스 환경에선 False).
"""
import sys
import pathlib
import time

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))

from common.camera import open_camera
from common.config import CAMERA_INDEX, WEIGHTS_PATH
from inference.detector import TargetDetector

SHOW_WINDOW = True  # cv2 창으로 방향을 표시. 콘솔 출력만 원하면 False.


def main():
    import cv2

    cap = open_camera(CAMERA_INDEX)
    if not cap.isOpened():
        print(f"카메라(index {CAMERA_INDEX}) 열기 실패. config.py 의 CAMERA_INDEX 를 확인하세요.")
        return

    detector = None
    if WEIGHTS_PATH.exists():
        try:
            detector = TargetDetector()
            print(f"YOLO 로드: {WEIGHTS_PATH}")
        except Exception as exc:
            print(f"YOLO 로드 실패(프레임 수신만 확인): {exc}")
    else:
        print(f"가중치 없음({WEIGHTS_PATH}). 프레임 수신만 확인합니다.")

    print("Ctrl+C 또는 (창에서) q 로 종료\n")
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("프레임 수신 실패")
                time.sleep(0.1)
                continue

            direction = "NONE"
            if detector is not None:
                direction = detector.detect_direction(frame)
                info = detector.last
                if info:
                    label, conf, _ = info
                    print(f"  목표 {label:12} conf={conf:.2f}  방향={direction}     ", end="\r")
                else:
                    print(f"  목표 없음  방향={direction}            ", end="\r")

            if SHOW_WINDOW:
                cv2.putText(frame, direction, (20, 40),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 0), 2)
                cv2.imshow("webcam test (q to quit)", frame)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
    except KeyboardInterrupt:
        print("\n종료")
    finally:
        cap.release()
        if SHOW_WINDOW:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
