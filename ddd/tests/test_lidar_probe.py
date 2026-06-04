"""tests/test_lidar_probe.py — common.lidar_metrics 순수 분석함수 단위테스트.

하드웨어/serial 불필요. 합성 (angle, distance) 점으로 window 집계·aggregate를 검증.
이 함수들은 텍스트 실측(lidar_range_probe)과 시각 뷰(lidar_probe_view)가 공유한다.
실행: python tests/test_lidar_probe.py   (모두 통과 시 'OK', 실패 시 AssertionError)
"""
import sys
import pathlib

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))  # project root

from common import lidar_metrics as probe  # noqa: E402


def approx(a, b, tol=1e-6):
    return abs(a - b) <= tol


def test_angular_diff_wrap():
    assert probe.angular_diff(359, 0) == 1
    assert probe.angular_diff(1, 359) == 2
    assert probe.angular_diff(10, 350) == 20
    print("  OK angular_diff wrap")


def test_window_filters_and_wrap():
    # bearing 0, ±5deg: 358(=-2),359(=-1),0,1,2 포함 / 90,180 제외
    measures = [(358, 1000), (359, 1010), (0, 990), (1, 1005), (2, 1000),
                (90, 500), (180, 700)]
    pts = probe.window_points(measures, 0, 5)
    assert len(pts) == 5, pts
    rels = sorted(round(r, 1) for r, _ in pts)
    assert rels == [-2.0, -1.0, 0.0, 1.0, 2.0], rels
    print("  OK window_points filter + wrap")


def test_summarize_distances_and_bins():
    measures = [(358, 1000), (359, 1010), (0, 990), (1, 1005), (2, 1000),
                (90, 500)]  # 90deg 는 window 밖
    s = probe.summarize_window(measures, 0, 5)
    assert s["n_raw"] == 5, s
    assert s["n_bins"] == 5, s          # 5개 서로 다른 정수각도
    assert s["d_min"] == 990, s
    assert s["d_max"] == 1010, s
    assert s["d_med"] == 1000, s
    assert s["d_std"] > 0, s
    assert approx(s["ang_span"], 4.0), s
    print("  OK summarize distances/bins/span")


def test_summarize_empty():
    s = probe.summarize_window([(90, 500), (180, 700)], 0, 5)
    assert s["n_raw"] == 0 and s["d_med"] is None and s["ang_span"] is None, s
    print("  OK summarize empty window")


def test_aggregate_hit_rate():
    snaps = [
        {"n_raw": 3, "n_bins": 3, "d_min": 990, "d_med": 1000, "d_max": 1010,
         "d_std": 8.0, "ang_span": 4.0, "ang_density": 1.25, "total_pts": 80,
         "dict_pts": 70, "fwd": 1000, "state": "SLOW"},
        {"n_raw": 0, "n_bins": 0, "d_min": None, "d_med": None, "d_max": None,
         "d_std": None, "ang_span": None, "ang_density": None, "total_pts": 78,
         "dict_pts": 69, "fwd": None, "state": "SAFE"},
        {"n_raw": 2, "n_bins": 2, "d_min": 980, "d_med": 985, "d_max": 990,
         "d_std": 5.0, "ang_span": 2.0, "ang_density": 1.0, "total_pts": 82,
         "dict_pts": 71, "fwd": 980, "state": "SLOW"},
    ]
    agg = probe.aggregate(snaps)
    assert agg["snapshots"] == 3, agg
    assert approx(agg["hit_rate"], 2 / 3), agg          # 3중 2 스냅샷 hit
    assert agg["d_min_mm"] == 980, agg                  # 전체 최소
    assert agg["d_max_mm"] == 1010, agg                 # 전체 최대
    assert approx(agg["fwd_found_rate"], 2 / 3), agg
    assert agg["state_counts"].get("SLOW") == 2, agg
    print("  OK aggregate hit_rate/pooled stats")


def test_theoretical_spacing_monotonic():
    # 거리↑ -> 점간격↑ (단조 증가). 5m가 1m보다 커야 함.
    s1 = probe.theoretical_spacing_mm(1000, 0.75)
    s5 = probe.theoretical_spacing_mm(5000, 0.75)
    assert s5 > s1 > 0, (s1, s5)
    # 1m, 0.75deg -> 약 13.1mm
    assert approx(s1, 1000 * (0.75 * 3.141592653589793 / 180), tol=0.5), s1
    print("  OK theoretical spacing monotonic")


def main():
    print("test_lidar_probe:")
    test_angular_diff_wrap()
    test_window_filters_and_wrap()
    test_summarize_distances_and_bins()
    test_summarize_empty()
    test_aggregate_hit_rate()
    test_theoretical_spacing_monotonic()
    print("OK (all passed)")


if __name__ == "__main__":
    main()
