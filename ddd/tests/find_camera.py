"""카메라 인덱스 찾기 — 어떤 인덱스가 Logitech 인지 미리보기로 확인.

노트북 내장캠 + 외장(Logitech)이 함께 있으면 인덱스가 헷갈린다(이 PC엔 0,1 둘 다 존재).
각 인덱스의 화면을 직접 보고 Logitech 인 것을 골라 common/config.py 의 CAMERA_INDEX 에 넣는다.

실행: python tests/find_camera.py
조작: 'n' = 다음 카메라, 'q' = 종료. 화면 좌상단에 현재 인덱스 표시.
"""
import sys
import pathlib

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))

MAX_INDEX = 4  # 0..MAX_INDEX-1 검사


def main():
    import cv2

    print("카메라 스캔 중...")
    available = []
    for i in range(MAX_INDEX):
        cap = cv2.VideoCapture(i)
        ok = cap.isOpened()
        if ok:
            ret, _ = cap.read()
            ok = bool(ret)
        cap.release()
        if ok:
            available.append(i)

    if not available:
        print("카메라를 찾지 못했습니다. 연결을 확인하세요.")
        return
    print("사용 가능한 인덱스:", available)
    print("'n' = 다음, 'q' = 종료. Logitech 화면인 인덱스를 골라 config.CAMERA_INDEX 에 설정하세요.")

    pos = 0
    cap = cv2.VideoCapture(available[pos])
    try:
        while True:
            ret, frame = cap.read()
            if ret:
                idx = available[pos]
                cv2.putText(frame, f"camera index {idx}   (n=next, q=quit)", (15, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                cv2.imshow("find camera", frame)
            key = cv2.waitKey(30) & 0xFF
            if key == ord("q"):
                break
            if key == ord("n"):
                cap.release()
                pos = (pos + 1) % len(available)
                cap = cv2.VideoCapture(available[pos])
                print("-> index", available[pos])
    finally:
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
