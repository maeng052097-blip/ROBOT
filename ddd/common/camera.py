"""카메라 열기 헬퍼.

Windows 기본(MSMF) 백엔드는 카메라 여는 데 수 초 걸릴 수 있다.
DSHOW 백엔드를 우선 시도(빠름)하고, 실패 시 기본으로 폴백한다.
또 16:9 해상도로 설정해 HFOV(수평화각) 가정과 맞춘다.

cv2 는 함수 안에서 임포트하므로, 이 모듈 자체는 cv2 없이도 import 된다.
"""
import threading
import time

from common.config import CAMERA_WIDTH, CAMERA_HEIGHT, CAMERA_FPS


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
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)   # 내부 지연버퍼 최소화(미지원 드라이버는 무시)
        if set_res:
            # StreamCam 의 1080p60 은 MJPEG 전용(YUY2 는 USB 대역폭상 불가).
            # FOURCC 는 해상도보다 '먼저' 설정해야 적용된다(Windows/DSHOW 레시피).
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            cap.set(cv2.CAP_PROP_FPS, CAMERA_FPS)
        ok, _ = cap.read()
        if ok:
            return cap          # 프레임 확인됨 -> 이 조합 사용
        cap.release()

    return cv2.VideoCapture(index)  # 마지막 폴백(호출측에서 처리)


def crop_center_zoom(frame, zoom, anchor_y=0.5):
    """디지털 줌: zoom 배 잘라 '확대된 시야'(view)를 반환. zoom<=1 이면 원본.

    anchor_y = 크롭 창의 세로 위치(0.0=상단 유지, 0.5=중앙, 1.0=하단 유지).
    카메라가 라이다 위에서 전방을 보면 '가까운 물체는 화면 아래'에 있으므로,
    근거리 작업에선 0.6~0.8 로 내려 잡아야 줌인 시 물체가 잘려나가지 않는다.
    ★ 가로는 항상 중앙 고정 — 베어링<->픽셀 매핑(view_x_from_bearing)이 가로 중앙
    크롭을 전제하므로 가로 팬을 추가하면 안 된다(세로는 매핑과 무관해 안전).
    순수 슬라이싱(cv2 불필요). color_detect / dual_camera_aim / track_and_approach 공유.
    """
    if zoom is None or zoom <= 1.0:
        return frame
    h, w = frame.shape[:2]
    cw = int(w / zoom)
    ch = int(h / zoom)
    ox = (w - cw) // 2
    a = 0.0 if anchor_y is None else max(0.0, min(1.0, float(anchor_y)))
    oy = int((h - ch) * a)
    return frame[oy:oy + ch, ox:ox + cw]


def camera_info(cap):
    """실제 적용된 해상도/FOURCC/FPS 문자열(진단). MJPG 미적용·저fps 판별용.

    예: '1920x1080 fourcc=MJPG fps=60'. YUY2 로 떨어졌으면 1080p 가 ~5fps 로 묶인다.
    """
    import cv2
    fcc = int(cap.get(cv2.CAP_PROP_FOURCC)) & 0xFFFFFFFF
    fcc_s = "".join(chr((fcc >> (8 * i)) & 0xFF) for i in range(4)) if fcc else "????"
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = cap.get(cv2.CAP_PROP_FPS)
    return f"{w}x{h} fourcc={fcc_s} fps={fps:.0f}"


class FrameGrabber:
    """배경 스레드가 cap.read() 를 계속 소비하며 '최신 프레임'만 보관(렉 제거).

    cap.read() 는 카메라 fps 에 블로킹되고 내부 버퍼가 지연을 누적한다(OpenCV 의
    알려진 문제 — issue #13145). 스레드가 프레임을 소비하면 메인 루프는 비블로킹으로
    항상 가장 최근 장면을 쓴다. cap 은 read()/release() 만 있으면 됨(테스트는 가짜 cap).
    """

    def __init__(self, cap):
        self.cap = cap
        self.frame = None
        self.n = 0                      # 누적 수신 프레임 수(실측 fps 진단용)
        self.lock = threading.Lock()
        self.running = True
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def _loop(self):
        while self.running:
            try:
                ok, f = self.cap.read()
            except Exception:
                ok, f = False, None
            if ok and f is not None:
                with self.lock:
                    self.frame = f
                    self.n += 1
            else:
                time.sleep(0.005)

    def latest(self):
        """가장 최근 프레임의 복사본(없으면 None). 비블로킹."""
        with self.lock:
            return None if self.frame is None else self.frame.copy()

    def release(self):
        self.running = False
        try:
            self.thread.join(timeout=1.0)
        except Exception:
            pass
        try:
            self.cap.release()
        except Exception:
            pass


# ===== 수동 포커스 (Logitech UVC 규약: 0~250, 5단위, 0=원거리/무한대, 250=최근접) =====
# StreamCam 은 근거리에서 AF 가 헤매는 것이 알려져 있어(리뷰/커뮤니티) 수동 고정이 표준 해법.
# 단, OpenCV 로 포커스 제어가 되는지는 드라이버/백엔드 의존 -> set 반환값 + 읽기백으로 확인하고,
# 실패 시 Logi Tune 의 수동 슬라이더로 1회 고정하는 폴백을 쓴다. (Logi Tune/G HUB 가 켜져
# 있으면 설정을 덮어쓰므로 끄고 테스트할 것. UVC 설정은 재부팅/재연결 시 초기화된다.)
FOCUS_MIN, FOCUS_MAX, FOCUS_STEP = 0, 250, 5


def clamp_focus(value):
    """포커스 값을 UVC 규약(0~250, 5의 배수)으로 정규화."""
    v = int(round(value / float(FOCUS_STEP))) * FOCUS_STEP
    return max(FOCUS_MIN, min(FOCUS_MAX, v))


def set_manual_focus(cap, value):
    """AF 끄고 수동 포커스 적용. (ok, 적용값, 읽기백) 반환.

    드라이버가 set 을 조용히 무시할 수 있으므로 호출측은 ok 와 읽기백을 함께 확인할 것.
    DSHOW 에서 실패하면 MSMF 백엔드로 다시 여는 것이 알려진 우회(BRIO 사례).
    """
    import cv2
    v = clamp_focus(value)
    ok_af = cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
    ok_f = cap.set(cv2.CAP_PROP_FOCUS, v)
    readback = cap.get(cv2.CAP_PROP_FOCUS)
    return bool(ok_af and ok_f), v, readback


def enable_autofocus(cap):
    """오토포커스 복귀. 성공 여부 반환(드라이버 의존)."""
    import cv2
    return bool(cap.set(cv2.CAP_PROP_AUTOFOCUS, 1))
