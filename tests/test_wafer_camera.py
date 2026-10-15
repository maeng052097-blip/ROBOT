"""tests/test_wafer_camera.py — 검사 카메라 피드(CameraFeed) 단위테스트. 실제 웹캠 대신 가짜 캡처.

  - 프레임이 들어오면 공유 JPEG(seq 증가), 상태 OK
  - 캡처 요청 -> 새 프레임 2장 뒤 원본 해상도 jpg 저장(sim_ 접두사) + poll_saved 로 파일명 반환
  - 카메라가 멈추면(read 실패 지속) CAM_STALE_S 뒤 STALE (멈춘 영상을 라이브로 오인 방지)
cv2 가 없는 환경이면 SKIP (py -3.13 에는 설치돼 있음).

실행: py -3.13 tests/test_wafer_camera.py
"""
import sys
import tempfile
import time
import pathlib

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))


class FakeCap:
    def __init__(self):
        import numpy as np
        self.np, self.i, self.alive = np, 0, True

    def isOpened(self):
        return True

    def read(self):
        time.sleep(0.02)
        if not self.alive:
            return False, None
        self.i += 1
        f = self.np.zeros((720, 1280, 3), self.np.uint8)
        f[:, :, 1] = self.i % 200
        return True, f

    def get(self, prop):
        return 0

    def release(self):
        pass


def main():
    print("test_wafer_camera:")
    try:
        import cv2  # noqa: F401
    except Exception:
        print("  SKIP (cv2 not installed)")
        print("OK (skipped)")
        return
    import common.camera as cam
    from wafer_cell import cell_config as C
    from wafer_cell import camera_feed as F

    fake = FakeCap()
    cam.open_camera = lambda *a, **k: fake
    cam.camera_info = lambda cap: "fake 1280x720"
    tmp = pathlib.Path(tempfile.mkdtemp())
    C.CAPTURE_DIR = tmp
    feed = F.CameraFeed(0)
    try:
        t0 = time.time()
        while feed.status()["seq"] < 3:
            assert time.time() - t0 < 5, "no frames encoded"
            time.sleep(0.05)
        st = feed.status()
        assert st["status"] == "OK" and st["mode"] == "live", st
        seq, buf = feed.jpeg()
        assert buf[:2] == b"\xff\xd8", "jpeg header"
        img = cv2.imdecode(__import__("numpy").frombuffer(buf, "uint8"), 1)
        assert img.shape[1] == C.CAM_JPEG_W, img.shape
        print("  OK shared JPEG, downscaled to", img.shape[1], "px")

        name = feed.capture(3, 7)
        assert name.startswith("sim_slot03_w7_")
        saved = []
        t0 = time.time()
        while not saved:
            assert time.time() - t0 < 5, "capture not saved"
            saved = feed.poll_saved()
            time.sleep(0.05)
        assert saved[0]["file"] == name and (tmp / name).is_file()
        full = cv2.imread(str(tmp / name))
        assert full.shape[:2] == (720, 1280)
        print("  OK capture saved at full resolution:", name)

        fake.alive = False
        t0 = time.time()
        while feed.status()["status"] != "STALE":
            assert time.time() - t0 < C.CAM_STALE_S + 2, "STALE not detected"
            time.sleep(0.05)
        print(f"  OK STALE after {time.time() - t0:.1f}s of no frames")
    finally:
        feed.close()
    print("OK (all passed)")


if __name__ == "__main__":
    main()
