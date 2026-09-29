"""웨이퍼 검사 카메라 피드(USB 웹캠) — 공유 JPEG 1장 + STALE 감지 + 단계 11 캡처 저장.

재사용: common/camera.open_camera(MJPG, DSHOW 우선) / FrameGrabber(최신 프레임) / camera_info.
  - 인코더 스레드 1개가 FrameGrabber.n 이 바뀔 때만 축소(CAM_JPEG_W) + imencode 1회 -> 모든 클라이언트가 같은 바이트 사용.
  - n 이 CAM_STALE_S 이상 멈추면 STALE, 한 번도 안 들어오면 NO_SIGNAL (끊긴 마지막 프레임을 라이브로 오인 방지).
  - 복구(재연결)는 하지 않는다: STALE 이면 서버 재시작 안내.
cv2/numpy 는 CameraFeed 생성 시에만 import (SIM 기본값 --cam none 은 cv2 불필요).
콘솔 출력 없음(스레드에서 print 금지 - Windows 콘솔 QuickEdit 멈춤 방지).
"""
import threading
import time
from datetime import datetime

from wafer_cell import cell_config as C


class NullFeed:
    """카메라 없음(--cam none). 화면은 브라우저가 그린 '개념도'를 쓴다."""
    mode = "none"

    def __init__(self):
        self.forced_fail = False

    def status(self):
        return {"mode": "none", "status": "FAIL" if self.forced_fail else "NONE", "seq": 0, "info": ""}

    def jpeg(self):
        return 0, None

    def capture(self, slot, wafer_id):
        return None

    def poll_saved(self):
        return []

    def close(self):
        pass


class CameraFeed:
    mode = "live"

    def __init__(self, index, width=None, height=None, fps=None):
        import cv2                                   # noqa: F401  (지연 import)
        from common.camera import open_camera, FrameGrabber, camera_info
        self.cv2 = cv2
        w = width or C.WAFER_CAM_W
        h = height or C.WAFER_CAM_H
        f = fps or C.WAFER_CAM_FPS
        self.cap = open_camera(index, w, h, f)
        self.opened = bool(self.cap is not None and self.cap.isOpened())
        self.info = camera_info(self.cap) if self.opened else "open failed"
        self.grabber = FrameGrabber(self.cap) if self.opened else None
        self.lock = threading.Lock()
        self.seq, self.buf = 0, None
        self.last_n, self.last_change = 0, time.monotonic()
        self.forced_fail = False
        self.requests, self.saved = [], []
        self.running = True
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def _loop(self):
        cv2 = self.cv2
        while self.running:
            time.sleep(1.0 / 15)
            g = self.grabber
            if g is None:
                continue
            n = g.n
            if n == self.last_n:
                continue
            self.last_n, self.last_change = n, time.monotonic()
            frame = g.latest()
            if frame is None:
                continue
            try:
                self._handle_requests(frame, n)
                hh, ww = frame.shape[:2]
                if ww > C.CAM_JPEG_W:
                    frame = cv2.resize(frame, (C.CAM_JPEG_W, int(hh * C.CAM_JPEG_W / ww)),
                                       interpolation=cv2.INTER_AREA)
                ok, enc = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), C.CAM_JPEG_Q])
                if ok:
                    with self.lock:
                        self.seq += 1
                        self.buf = enc.tobytes()
            except Exception:
                pass

    def _handle_requests(self, frame, n):
        """캡처 요청 후 새 프레임이 2장 이상 들어온 시점의 원본 해상도 프레임을 저장."""
        with self.lock:
            due = [r for r in self.requests if n >= r["n0"] + 2]
            self.requests = [r for r in self.requests if n < r["n0"] + 2]
        for r in due:
            img = frame.copy()
            txt = f"SIM PROCESS / REAL CAMERA  slot {r['slot']}  {r['stamp']}"
            self.cv2.putText(img, txt, (12, 32), self.cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 4)
            self.cv2.putText(img, txt, (12, 32), self.cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 1)
            C.CAPTURE_DIR.mkdir(parents=True, exist_ok=True)
            path = C.CAPTURE_DIR / r["name"]
            ok = self.cv2.imwrite(str(path), img)
            with self.lock:
                self.saved.append({"wafer": r["wafer"], "slot": r["slot"], "file": r["name"] if ok else None})

    def status(self):
        if self.forced_fail or not self.opened:
            st = "FAIL"
        elif self.last_n == 0 and (self.grabber is None or self.grabber.n == 0):
            st = "NO_SIGNAL"
        elif time.monotonic() - self.last_change > C.CAM_STALE_S:
            st = "STALE"
        else:
            st = "OK"
        with self.lock:
            seq = self.seq
        return {"mode": "live", "status": st, "seq": seq, "info": self.info}

    def jpeg(self):
        with self.lock:
            return self.seq, self.buf

    def capture(self, slot, wafer_id):
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        name = f"sim_slot{int(slot or 0):02d}_w{int(wafer_id or 0)}_{stamp}.jpg"
        with self.lock:
            self.requests.append({"slot": slot, "wafer": wafer_id, "n0": self.last_n,
                                  "stamp": stamp, "name": name})
        return name

    def poll_saved(self):
        with self.lock:
            out, self.saved = self.saved, []
        return out

    def close(self):
        self.running = False
        try:
            self.thread.join(timeout=1.0)
        except Exception:
            pass
        g = self.grabber
        if g is not None:
            g.running = False
            try:
                g.thread.join(timeout=1.0)
            except Exception:
                pass
            if not g.thread.is_alive():      # read() 가 막혀 있으면 release 하지 않는다(드라이버 멈춤 방지)
                try:
                    g.cap.release()
                except Exception:
                    pass
