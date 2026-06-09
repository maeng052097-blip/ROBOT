"""카메라 열기 헬퍼.

Windows 기본(MSMF) 백엔드는 카메라 여는 데 수 초 걸릴 수 있다.
DSHOW 백엔드를 우선 시도(빠름)하고, 실패 시 기본으로 폴백한다.
또 16:9 해상도로 설정해 HFOV(수평화각) 가정과 맞춘다.

cv2 는 함수 안에서 임포트하므로, 이 모듈 자체는 cv2 없이도 import 된다.
"""
from common.config import CAMERA_WIDTH, CAMERA_HEIGHT


def open_camera(index, width=CAMERA_WIDTH, height=CAMERA_HEIGHT):
    """카메라를 연다. cv2.VideoCapture 반환(isOpened()/read()로 확인).

    여러 조합을 순서대로 시도해, '실제 프레임이 나오는' 첫 조합을 쓴다.
      1) DSHOW + 지정 해상도(빠름)   2) DSHOW + 기본 해상도
      3) 기본 백엔드 + 지정 해상도   4) 기본 백엔드 + 기본 해상도
    (열리기만 하고 프레임을 못 주는 경우 'isOpened=True'여도 다음으로 넘어간다.)
    """
    import cv2

    attempts = [
        (cv2.CAP_DSHOW, True),
        (cv2.CAP_DSHOW, False),
        (None, True),
        (None, False),
    ]
    for backend, set_res in attempts:
        cap = cv2.VideoCapture(index, backend) if backend is not None else cv2.VideoCapture(index)
        if not cap.isOpened():
            cap.release()
            continue
        if set_res:
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        ok, _ = cap.read()
        if ok:
            return cap          # 프레임 확인됨 -> 이 조합 사용
        cap.release()

    return cv2.VideoCapture(index)  # 마지막 폴백(호출측에서 처리)


def crop_center_zoom(frame, zoom):
    """디지털 줌: 중앙을 zoom 배 잘라 '확대된 시야'(view)를 반환. zoom<=1 이면 원본.

    유효 화각은 호출측에서 hfov/zoom 으로 계산한다(여기선 크롭만). 순수 슬라이싱이라
    cv2 불필요. color_detect / dual_camera_aim 이 공유한다.
    """
    if zoom is None or zoom <= 1.0:
        return frame
    h, w = frame.shape[:2]
    cw = int(w / zoom)
    ch = int(h / zoom)
    ox = (w - cw) // 2
    oy = (h - ch) // 2
    return frame[oy:oy + ch, ox:ox + cw]
