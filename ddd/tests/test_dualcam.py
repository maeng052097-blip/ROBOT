"""tests/test_dualcam.py — 2카메라 재설계 핵심 로직 단위테스트(하드웨어 불필요).

검증: ① 클릭 패널 3분할 ② 카메라 side->off_x 부호 ③ 시차 거리/로봇각(양 카메라가
같은 물체를 같은 로봇각으로 복원) ④ 로봇 베어링 = signed_diff(lidar_angle, FORWARD)
이며 lidar_bearing() 재적용이 아님(플립 이중적용 방지).
실행: py -3.13 tests/test_dualcam.py
"""
import sys
import math
import pathlib

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))


def approx(a, b, t=1e-6):
    return abs(a - b) <= t


def main():
    from visualization.track_and_approach import classify_click, side_off_x, side_toe
    from common.config import CAM_SIDE_OFFSET_MM
    from common.fusion import distance_along_ray
    from common.lidar_metrics import normalize_deg

    print("test_dualcam:")

    # ① 클릭 3분할 (camL | camR | radar), 각 패널 폭 CAM_W
    W = 640
    assert classify_click(10, W) == ("L", 10)
    assert classify_click(650, W) == ("R", 10)
    assert classify_click(1300, W) == ("radar", 20)
    assert classify_click(W - 1, W)[0] == "L" and classify_click(W, W)[0] == "R"
    print("  OK classify_click 3-way split")

    # ② side -> off_x 부호 (왼쪽 L=-, 오른쪽 R=+; 색은 라벨일 뿐 계산 무관)
    assert side_off_x("L") == -CAM_SIDE_OFFSET_MM
    assert side_off_x("R") == CAM_SIDE_OFFSET_MM
    assert side_off_x("radar") == 0.0
    assert side_toe("L") == -side_toe("R")  # 대칭 (기본 0)
    print(f"  OK side_off_x signs (L=-{CAM_SIDE_OFFSET_MM}, R=+{CAM_SIDE_OFFSET_MM})")

    # ③ 시차: 정면 1m 물체를 좌/우 카메라가 각자 베어링으로 봐도 같은 (거리, 로봇각0) 복원
    for D in (500.0, 1000.0, 2000.0):
        dd = {0: int(D)}                       # 정면(robot/raw angle 0, FORWARD=0)
        for side in ("L", "R"):
            off = side_off_x(side)
            cb = math.degrees(math.atan2(-off, D))   # 그 카메라가 보는 베어링
            rng, la = distance_along_ray(dd, off, 0.0, cb, 0.0, 200.0)
            assert rng == int(D) and approx(la, 0.0), (side, D, rng, la)
    print("  OK parallax: both cams recover (range, robot_angle 0) for dead-front")

    # 우측 20° 물체 -> 좌카메라가 더 큰 베어링으로 보지만 로봇각 20° 복원
    D = 1000.0
    dd = {20: int(D)}
    off = side_off_x("L")
    cb = math.degrees(math.atan2(D * math.sin(math.radians(20)) - off, D * math.cos(math.radians(20))))
    rng, la = distance_along_ray(dd, off, 0.0, cb, 0.0, 200.0)
    assert rng == int(D) and approx(la, 20.0), (rng, la)
    print("  OK parallax: 20deg-right -> robot_angle 20 from left cam")

    # ④ 로봇 베어링 = normalize_deg(lidar_angle - FORWARD). FORWARD!=0 케이스.
    forward = 30.0
    # 정면(로봇기준 0)인 물체는 raw=forward 에 위치
    dd = {30: 1000}
    off = side_off_x("R")
    cb = math.degrees(math.atan2(-off, 1000.0))
    rng, la = distance_along_ray(dd, off, 0.0, cb, forward, 200.0)
    rb = normalize_deg(la - forward)
    assert approx(rb, 0.0, 1e-6), (la, rb)   # 로봇기준 정면 = 0
    assert la == 30                          # 반환은 raw 각도(30)
    print("  OK robot_bearing = signed_diff(lidar_angle, FORWARD) (no double-flip)")

    print("OK (all passed)")


if __name__ == "__main__":
    main()
