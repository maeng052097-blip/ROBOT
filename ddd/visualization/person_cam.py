"""visualization/person_cam.py — 카메라에서 '사람' 실시간 인식(네모칸 + 라벨).

COCO 사전학습 YOLO(yolov8n.pt, person=class 0)로 사람을 탐지해 초록 박스 + "person 0.xx".
재활용 best.pt 에는 person 클래스가 없어서 COCO 가중치를 쓴다.

필요: ultralytics(torch). 가중치=models/yolov8n.pt (없으면 ultralytics 자동 다운로드).
실행: py -3.13 visualization/person_cam.py        (사람만)
      py -3.13 visualization/person_cam.py --classes ""   (COCO 80종 전체)
조작: q 종료
"""
import sys
import pathlib
import argparse

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))

from common.config import CAMERA_INDEX, COCO_WEIGHTS_PATH
from common.camera import open_camera


def main():
    ap = argparse.ArgumentParser(description="카메라 사람 인식(COCO YOLO)")
    ap.add_argument("--model", default=str(COCO_WEIGHTS_PATH), help="YOLO 가중치(COCO)")
    ap.add_argument("--cam-index", type=int, default=CAMERA_INDEX)
    ap.add_argument("--conf", type=float, default=0.4, help="신뢰도 임계값")
    ap.add_argument("--classes", default="0",
                    help="탐지할 COCO 클래스 id(쉼표). 기본 '0'=person. 빈값=''=전체")
    args = ap.parse_args()

    import cv2
    from ultralytics import YOLO

    model = YOLO(args.model)
    names = model.names
    cls_filter = ([int(c) for c in args.classes.split(",") if c.strip() != ""]
                  if args.classes.strip() != "" else None)

    cap = open_camera(args.cam_index)
    if not cap.isOpened():
        print(f"카메라(index {args.cam_index}) 열기 실패.")
        return
    target = "person" if cls_filter == [0] else (
        ",".join(names[c] for c in cls_filter) if cls_filter else "all")
    print(f"모델={args.model} | 대상={target} | conf={args.conf} | q=종료")

    win = "person detection (q to quit)"
    cv2.namedWindow(win, cv2.WINDOW_AUTOSIZE)
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                continue
            res = model.predict(frame, conf=args.conf, classes=cls_filter, verbose=False)
            r = res[0]
            count = 0
            for b in r.boxes:
                cls = int(b.cls[0])
                conf = float(b.conf[0])
                x1, y1, x2, y2 = (int(v) for v in b.xyxy[0])
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                label = f"{names[cls]} {conf:.2f}"
                ytxt = max(18, y1 - 6)
                cv2.putText(frame, label, (x1, ytxt), cv2.FONT_HERSHEY_SIMPLEX,
                            0.6, (0, 0, 0), 3, cv2.LINE_AA)
                cv2.putText(frame, label, (x1, ytxt), cv2.FONT_HERSHEY_SIMPLEX,
                            0.6, (0, 255, 0), 1, cv2.LINE_AA)
                count += 1
            tag = "persons" if cls_filter == [0] else "objects"
            cv2.putText(frame, f"{tag}: {count}", (12, 28), cv2.FONT_HERSHEY_SIMPLEX,
                        0.8, (0, 0, 0), 4, cv2.LINE_AA)
            cv2.putText(frame, f"{tag}: {count}", (12, 28), cv2.FONT_HERSHEY_SIMPLEX,
                        0.8, (0, 255, 0), 2, cv2.LINE_AA)

            cv2.imshow(win, frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
            if cv2.getWindowProperty(win, cv2.WND_PROP_VISIBLE) < 1:
                break
    except KeyboardInterrupt:
        pass
    finally:
        cap.release()
        cv2.destroyAllWindows()
        print("종료")


if __name__ == "__main__":
    main()
