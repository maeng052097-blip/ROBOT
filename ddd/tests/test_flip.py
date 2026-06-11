"""tests/test_flip.py — 뒤집힌 라이다 좌표 변환 단위테스트(하드웨어 불필요).

lidar_bearing(raw)=로봇베어링, bearing_to_lidar_angle(b)=raw 의 왕복 일치와
뒤집힘 부호(LIDAR_FLIPPED)를 config 와 일관되게 검증한다.

실행: py -3.13 tests/test_flip.py
"""
import sys
import pathlib

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))


def main():
    from common.config import LIDAR_FLIPPED, FORWARD_ANGLE_DEG
    from common.fusion import lidar_dir_sign, lidar_bearing, bearing_to_lidar_angle
    from common.lidar_metrics import angular_diff

    print("test_flip:")

    # dir_sign 은 '라이다 장착(뒤집힘)'만의 부호 (카메라 부호와 분리)
    expected_sign = -1.0 if LIDAR_FLIPPED else 1.0
    assert lidar_dir_sign() == expected_sign, (lidar_dir_sign(), expected_sign)
    print(f"  OK dir_sign = {lidar_dir_sign():+.0f} (LIDAR_FLIPPED={LIDAR_FLIPPED})")

    # 전방각 raw -> 베어링 0
    assert abs(lidar_bearing(FORWARD_ANGLE_DEG)) < 1e-9
    print("  OK forward raw -> bearing 0")

    # 왕복 일치: raw -> bearing -> raw (부호값 무관하게 항상 성립)
    for raw in (0.0, 30.0, 90.0, 170.0, 200.0, 300.0, 359.0):
        b = lidar_bearing(raw)
        raw2 = bearing_to_lidar_angle(b)
        assert angular_diff(raw2, raw) < 1e-6, (raw, b, raw2)
    print("  OK roundtrip raw->bearing->raw")

    # 전방에서 +10도(raw) 떨어진 점의 베어링은 dir_sign*10 (뒤집힘이면 -10 = 좌측)
    assert abs(lidar_bearing(FORWARD_ANGLE_DEG + 10.0) - lidar_dir_sign() * 10.0) < 1e-9
    print("  OK +10deg raw mirrors per dir_sign")

    print("OK (all passed)")


if __name__ == "__main__":
    main()
