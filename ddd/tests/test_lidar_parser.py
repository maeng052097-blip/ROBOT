"""LiDAR 패킷 파서 + 신선도 로직 단위 테스트 (하드웨어 불필요).

LidarX2._parse_packets 가 YDLIDAR X2 패킷 바이트를 올바른 (각도, 거리)로
디코드하는지, is_fresh 가 동작하는지 합성 데이터로 검증한다.
실행: python tests/test_lidar_parser.py
"""
import sys
import pathlib
import time

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))

from drivers.LidarX2 import LidarX2


def make_packet(sample_count, start_deg, end_deg, distances, ptype=0):
    """합성 패킷 생성. angle_raw = deg * 128  (angle = (raw >> 1) / 64)."""
    def raw(deg):
        return int(round(deg * 128)) & 0xFFFF
    sr, er = raw(start_deg), raw(end_deg)
    pkt = bytearray([0xAA, 0x55, ptype & 0xFF, sample_count & 0xFF,
                     sr & 0xFF, (sr >> 8) & 0xFF, er & 0xFF, (er >> 8) & 0xFF,
                     0x00, 0x00])  # 바이트 8,9: 미사용(체크섬 자리)
    for d in distances:
        pkt += bytearray([d & 0xFF, (d >> 8) & 0xFF])
    return pkt


def approx(a, b, tol=0.01):
    return abs(a - b) <= tol


# T1) 기본 2샘플: 0deg/10deg, 1000mm/2000mm
m, rest = LidarX2._parse_packets(bytearray(make_packet(2, 0.0, 10.0, [1000, 2000])))
assert len(m) == 2, m
assert approx(m[0][0], 0.0) and m[0][1] == 1000, m
assert approx(m[1][0], 10.0) and m[1][1] == 2000, m
assert len(rest) == 0, rest
print("T1 기본 패킷 OK:", m)

# T2) 거리 0 은 제외 (가운데 샘플만 유효)
m, rest = LidarX2._parse_packets(bytearray(make_packet(3, 0.0, 20.0, [0, 1500, 0])))
assert len(m) == 1 and m[0][1] == 1500 and approx(m[0][0], 10.0), m
print("T2 거리0 제외 OK:", m)

# T3) 헤더 앞 잡음 무시
pkt = bytearray([0xFF, 0x12, 0x34]) + make_packet(1, 90.0, 90.0, [777])
m, rest = LidarX2._parse_packets(bytearray(pkt))
assert len(m) == 1 and m[0][1] == 777 and approx(m[0][0], 90.0), m
print("T3 잡음 무시 OK:", m)

# T4) 미완성 패킷은 보류(버퍼 보존, 측정 0)
full = make_packet(2, 0.0, 10.0, [1000, 2000])
partial = full[:-2]
m, rest = LidarX2._parse_packets(bytearray(partial))
assert len(m) == 0 and len(rest) == len(partial), (m, len(rest), len(partial))
print("T4 미완성 보류 OK: rest", len(rest), "bytes")

# T5) 이어붙이면 완성되어 파싱
m, rest = LidarX2._parse_packets(bytearray(partial) + full[-2:])
assert len(m) == 2, m
print("T5 이어붙임 완성 OK:", m)

# T6) 360도 경계: start=350, end=10 -> diff=20 (j1 = 370 % 360 = 10)
m, rest = LidarX2._parse_packets(bytearray(make_packet(2, 350.0, 10.0, [500, 600])))
assert approx(m[0][0], 350.0) and approx(m[1][0], 10.0), m
print("T6 360 경계 OK:", m)

# T7) 신선도: 미수신 None/False, last_update 설정 시 True, 오래되면 False
lid = LidarX2("COMX")
assert lid.seconds_since_update() is None and lid.is_fresh() is False
lid.last_update = time.time()
assert lid.is_fresh(0.5) is True
lid.last_update = time.time() - 10
assert lid.is_fresh(0.5) is False
print("T7 신선도 로직 OK")

print("ALL_LIDAR_PARSER_TESTS_OK")
