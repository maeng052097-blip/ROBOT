"""tests/test_lidar_freshest.py — getDistanceDict 의 median vs freshest (움직임 마스킹 회귀방지).

누적 median 은 정적 노이즈엔 좋지만, '움직이는' 근접 물체를 stale 배경으로 덮어
마스킹/지연시킨다. 실시간 추적은 freshest=True(각 각도의 최신값)를 써야 한다.
실행: py -3.13 tests/test_lidar_freshest.py   (하드웨어 불필요)
"""
import sys
import pathlib

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))


def main():
    from drivers.LidarX2 import LidarX2

    print("test_lidar_freshest:")
    lidar = LidarX2("COMx")   # 시리얼 안 엶, measures 직접 주입
    # angle 0: 배경 1000 이 누적된 뒤 물체가 360->350 으로 접근 (시간순 append)
    lidar.measures = [(0, 1000), (0, 1000), (0, 1000), (0, 360), (0, 350)]

    med = lidar.getDistanceDict()
    fresh = lidar.getDistanceDict(freshest=True)
    assert med[0] == 1000, f"median 은 누적 배경에 가려 물체를 놓쳐야 함(현상 재현): {med}"
    assert fresh[0] == 350, f"freshest 는 최신(물체)을 반영해야 함: {fresh}"
    print("  OK median masks moving object (1000), freshest reveals it (350)")

    # 빈 입력
    lidar.measures = []
    assert lidar.getDistanceDict() == {} and lidar.getDistanceDict(freshest=True) == {}
    print("  OK empty input")

    # 0/음수 거리 무시
    lidar.measures = [(10, 0), (10, 500)]
    assert lidar.getDistanceDict(freshest=True) == {10: 500}
    print("  OK ignores non-positive distance")

    # X4 팩토리 배선: make_lidar('x4')가 config 의 거리보정(SCALE/OFFSET)을 전달하는지
    # (시리얼 안 엶 — 생성만). 보정값을 lidar_calibrate 로 채우면 자동 적용되는 경로 보장.
    from drivers import make_lidar
    from common.config import LIDAR_X4_DIST_SCALE, LIDAR_X4_DIST_OFFSET_MM
    l4 = make_lidar("x4", "COMx")
    assert l4.dist_scale == LIDAR_X4_DIST_SCALE, (l4.dist_scale, LIDAR_X4_DIST_SCALE)
    assert l4.dist_offset_mm == LIDAR_X4_DIST_OFFSET_MM
    print("  OK make_lidar('x4') wires config dist scale/offset")

    print("OK (all passed)")


if __name__ == "__main__":
    main()
