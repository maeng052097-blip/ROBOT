"""tests/lidar_raw_probe.py — COM 포트 '원시 바이트' 진단.

목적: LiDAR 창이 STALE(데이터 0)일 때 원인을 데이터로 가린다.
  - 데이터가 오긴 하는가? (bytes>0)
  - YDLIDAR 패킷 헤더(0xAA 0x55)가 보이는가?
  - 어느 baud(115200=X2 / 128000=X4)에서 보이는가?

같은 포트를 두 baud 로 각각 몇 초 읽어 바이트 수·헤더 수·샘플(hex)을 출력한다.

주의: LiDAR 창(lidar_probe_view.py)이 떠 있으면 포트를 점유해 'open 실패'가 난다.
      먼저 그 창을 닫고(q) 이 진단을 실행하세요.

실행: py -3.13 tests/lidar_raw_probe.py --port COM8
"""
import sys
import time
import pathlib
import argparse

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))

from common.config import LIDAR_PORT


def sniff(port, baud, seconds):
    import serial
    try:
        s = serial.Serial(port, baud, timeout=0.5)
    except Exception as e:
        return None, f"open 실패: {e}"
    total = 0
    buf = bytearray()
    sample = b""
    end = time.time() + seconds
    try:
        while time.time() < end:
            d = s.read(512)
            if d:
                total += len(d)
                if len(sample) < 24:
                    sample += d[: 24 - len(sample)]
                buf.extend(d)
                if len(buf) > 20000:        # 헤더 카운트용으로 충분
                    buf = buf[-20000:]
    finally:
        s.close()
    headers = sum(1 for i in range(len(buf) - 1) if buf[i] == 0xAA and buf[i + 1] == 0x55)
    return (total, headers, sample.hex(" ")), None


def main():
    ap = argparse.ArgumentParser(description="COM 포트 원시 바이트 진단")
    ap.add_argument("--port", default=LIDAR_PORT, help="진단할 COM 포트")
    ap.add_argument("--seconds", type=float, default=3.0, help="각 baud 당 읽기 시간(초)")
    args = ap.parse_args()

    print(f"진단 포트: {args.port}  (각 baud {args.seconds}s)\n")
    results = {}
    for baud in (115200, 128000):
        print(f"[baud {baud}] 읽는 중...")
        res, err = sniff(args.port, baud, args.seconds)
        if err:
            print(f"  {err}")
            results[baud] = (0, 0)
        else:
            total, headers, sample = res
            print(f"  bytes={total}  headers(0xAA55)={headers}")
            print(f"  sample={sample}")
            results[baud] = (total, headers)
        print()

    print("=" * 56)
    b1, b2 = results.get(115200, (0, 0)), results.get(128000, (0, 0))
    if b1[1] > 0 and b1[1] >= b2[1]:
        print("판정: 115200 에서 헤더 검출 -> 이 장치는 X2.")
        print(f"  실행: py -3.13 visualization/lidar_probe_view.py --port {args.port}")
    elif b2[1] > 0:
        print("판정: 128000 에서 헤더 검출 -> 이 장치는 X4.")
        print(f"  실행: py -3.13 visualization/lidar_probe_view.py --lidar x4 --port {args.port}")
    elif b1[0] == 0 and b2[0] == 0:
        print("판정: 두 baud 모두 bytes=0 -> 데이터 자체가 안 옴.")
        print("  원인 후보: 모터 미회전/전원 부족, USB 케이블/어댑터, 포트 점유(창 안 닫힘).")
        print("  확인: LiDAR 윗부분이 실제로 도는지(전원), 다른 USB 포트/케이블로 교체.")
    else:
        print("판정: 바이트는 오는데 헤더(0xAA55)가 없음 -> 다른 장치이거나 노이즈.")
        print("  포트 번호가 맞는지(목록 재확인), 또는 baud 가 둘 다 아님.")
    print("=" * 56)


if __name__ == "__main__":
    main()
