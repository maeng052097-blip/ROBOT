"""tests/test_occupancy_grid.py — 점유격자 코어 단위테스트 (하드웨어 불필요).

실행: py -3.13 tests/test_occupancy_grid.py  (통과 시 'OK', 실패 시 AssertionError)
"""
import sys
import pathlib

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))

from common.occupancy_grid import OccupancyGrid, bresenham  # noqa: E402


def test_bresenham():
    assert bresenham(0, 0, 3, 0) == [(0, 0), (1, 0), (2, 0), (3, 0)]
    assert bresenham(0, 0, 0, 3) == [(0, 0), (0, 1), (0, 2), (0, 3)]
    assert bresenham(0, 0, 2, 2) == [(0, 0), (1, 1), (2, 2)]
    assert bresenham(0, 0, -2, 0) == [(0, 0), (-1, 0), (-2, 0)]
    print("  OK bresenham")


def test_world_to_cell():
    g = OccupancyGrid(size_m=8.0, res_m=0.05)
    o = g.origin
    assert g.world_to_cell(0.0, 0.0) == (o, o)
    assert g.world_to_cell(0.15, 0.0) == (o + 3, o)      # +X (3셀)
    assert g.world_to_cell(0.0, -0.10) == (o, o - 2)     # -Y (2셀)
    print("  OK world_to_cell")


def test_integrate_marks_occupied_and_free():
    g = OccupancyGrid(size_m=8.0, res_m=0.05)
    g.integrate_scan((0.0, 0.0, 0.0), [(0.0, 2000.0)])   # 정면 2m -> +Y 40셀
    o = g.origin
    eci, ecj = g.world_to_cell(0.0, 2.0)
    assert (eci, ecj) == (o, o + 40), (eci, ecj)
    assert g.prob(eci, ecj) > 0.5, g.prob(eci, ecj)       # 끝점 = 점유
    assert g.prob(o, o + 20) < 0.5, g.prob(o, o + 20)     # 중간 = 자유
    print("  OK integrate occupied/free")


def test_integrate_right_direction():
    g = OccupancyGrid(size_m=8.0, res_m=0.05)
    g.integrate_scan((0.0, 0.0, 0.0), [(90.0, 1000.0)])  # 오른쪽 1m -> +X 20셀
    o = g.origin
    eci, ecj = g.world_to_cell(1.0, 0.0)
    assert (eci, ecj) == (o + 20, o), (eci, ecj)
    assert g.prob(eci, ecj) > 0.5
    print("  OK integrate +90deg -> +X")


def main():
    print("test_occupancy_grid:")
    test_bresenham()
    test_world_to_cell()
    test_integrate_marks_occupied_and_free()
    test_integrate_right_direction()
    print("OK (all passed)")


if __name__ == "__main__":
    main()
