"""tests/test_frame_grabber.py — FrameGrabber(렉 제거 스레드 캡처) 단위테스트.

진짜 카메라 대신 read()/release() 만 가진 '가짜 캡처'로 검증(하드웨어 불필요):
  - 배경 스레드가 프레임을 계속 수신해 n 이 증가한다.
  - latest() 는 가장 최근 프레임(복사본)을 비블로킹으로 준다.
  - release() 가 스레드를 멈추고 cap.release() 까지 호출한다.

실행: py -3.13 tests/test_frame_grabber.py
"""
import sys
import time
import pathlib

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))


class FakeCap:
    """read() 가 호출될 때마다 픽셀값=호출횟수 인 4x4 프레임을 주는 가짜 카메라."""

    def __init__(self):
        import numpy as np
        self.np = np
        self.i = 0
        self.released = False

    def read(self):
        self.i += 1
        time.sleep(0.002)   # 실제 카메라처럼 약간 블로킹
        return True, self.np.full((4, 4, 3), self.i % 250, self.np.uint8)

    def release(self):
        self.released = True


def main():
    from common.camera import FrameGrabber

    print("test_frame_grabber:")
    cap = FakeCap()
    g = FrameGrabber(cap)

    time.sleep(0.15)
    f1 = g.latest()
    assert f1 is not None and f1.shape == (4, 4, 3), "최신 프레임 수신"
    n1 = g.n
    assert n1 > 0
    print(f"  OK background thread receiving (n={n1})")

    time.sleep(0.1)
    assert g.n > n1, "스레드가 계속 수신해 n 증가"
    f2 = g.latest()
    assert int(f2[0, 0, 0]) >= int(f1[0, 0, 0]), "latest 는 더 최신 프레임"
    print("  OK latest() returns newer frame (non-blocking)")

    # latest() 는 복사본: 호출측이 수정해도 내부 프레임 불변
    f2[:] = 0
    f3 = g.latest()
    assert int(f3[0, 0, 0]) != 0 or g.n == 0
    print("  OK latest() returns a copy")

    g.release()
    assert cap.released, "release() 가 cap.release() 호출"
    n_after = g.n
    time.sleep(0.05)
    assert g.n == n_after, "release 후 수신 정지"
    print("  OK release() stops thread and releases cap")
    print("OK (all passed)")


if __name__ == "__main__":
    main()
