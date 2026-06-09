"""LiDAR window 집계(거리/각도/탐지) 공용 분석 함수.

텍스트 실측 도구(tests/lidar_range_probe.py)와 시각 뷰(visualization/lidar_probe_view.py)가
'완전히 같은' 집계를 쓰도록 한 곳에 둔다(= 화면으로 본 판단 = 기록된 수치).
하드웨어 불필요한 순수 함수라 단위테스트(tests/test_lidar_probe.py) 대상.
"""
import math
from collections import Counter
from statistics import median, pstdev


def angular_diff(a, b):
    """두 각도(deg)의 최소 차이(0~180). 0도 경계 wrap 처리."""
    return abs((a - b + 180) % 360 - 180)


def window_points(measures, target_deg, window_deg):
    """target_deg ±window_deg 안의 점을 [(상대각도, 거리mm), ...] 로. distance>0만."""
    out = []
    for a, d in measures:
        if d > 0 and angular_diff(a, target_deg) <= window_deg:
            rel = (a - target_deg + 180) % 360 - 180
            out.append((rel, d))
    return out


def summarize_window(measures, target_deg, window_deg):
    """한 스냅샷의 window 통계(측정값만). 점이 없으면 n_raw=0, 나머지 None."""
    pts = window_points(measures, target_deg, window_deg)
    if not pts:
        return {"n_raw": 0, "n_bins": 0, "d_min": None, "d_med": None,
                "d_max": None, "d_std": None, "ang_span": None, "ang_density": None}
    dists = [d for _, d in pts]
    rels = [r for r, _ in pts]
    bins = {int(round(target_deg + r)) % 360 for r in rels}
    span = max(rels) - min(rels)
    return {
        "n_raw": len(pts),
        "n_bins": len(bins),
        "d_min": min(dists),
        "d_med": median(dists),
        "d_max": max(dists),
        "d_std": pstdev(dists) if len(dists) > 1 else 0.0,
        "ang_span": round(span, 2),
        "ang_density": (len(bins) / span) if span > 0 else None,  # bins per deg(측정)
    }


def aggregate(snapshots):
    """스냅샷 리스트를 한 조건의 요약으로 집계."""
    n = len(snapshots)
    if n == 0:
        return {"snapshots": 0, "hit_rate": 0.0}

    def present(key):
        return [s[key] for s in snapshots if s.get(key) is not None]

    hits = [s for s in snapshots if s["n_raw"] > 0]
    d_meds = present("d_med")
    d_mins = present("d_min")
    d_maxs = present("d_max")
    d_stds = present("d_std")
    spans = present("ang_span")
    n_raws = [s["n_raw"] for s in snapshots]
    n_binss = [s["n_bins"] for s in snapshots]
    fwds = present("fwd")
    states = [s["state"] for s in snapshots if s.get("state") is not None]
    dict_pts = present("dict_pts")
    totals = present("total_pts")

    return {
        "snapshots": n,
        "hit_rate": len(hits) / n,
        "n_raw_mean": (sum(n_raws) / n),
        "n_bins_mean": (sum(n_binss) / n),
        "d_min_mm": (min(d_mins) if d_mins else None),
        "d_med_mm": (median(d_meds) if d_meds else None),
        "d_max_mm": (max(d_maxs) if d_maxs else None),
        "d_std_mm": (median(d_stds) if d_stds else None),
        "ang_span_deg": (sum(spans) / len(spans)) if spans else None,
        "dict_pts_mean": (sum(dict_pts) / len(dict_pts)) if dict_pts else None,
        "total_pts_mean": (sum(totals) / len(totals)) if totals else None,
        "fwd_found_rate": (len(fwds) / n),
        "state_counts": dict(Counter(states)),
    }


def theoretical_spacing_mm(distance_mm, res_deg):
    """이론 점간격(mm) = 거리 × tan(분해능). '가정(분해능)' 기반 참고값."""
    return distance_mm * math.tan(math.radians(res_deg))


def normalize_deg(x):
    """각도를 (-180, 180] 범위로 정규화."""
    return ((x + 180) % 360) - 180


def nearest_point(measures, min_mm=120.0, max_mm=None):
    """가장 가까운 (angle, dist). min_mm 미만·0이하 제외, max_mm 초과 제외.

    두 LiDAR 정렬 시 '같은 근접 물체'의 방위를 비교하는 데 쓴다. 없으면 (None, None).
    """
    best_a = best_d = None
    for a, d in measures:
        if d < min_mm:
            continue
        if max_mm is not None and d > max_mm:
            continue
        if best_d is None or d < best_d:
            best_a, best_d = a, d
    return best_a, best_d


def nearest_point_2d(measures, target_angle, target_dist_mm):
    """(target_angle, target_dist_mm) 위치에 2D(극→직교)로 가장 가까운 (angle, dist, 거리mm).

    레이더에서 한 점을 클릭/추적할 때 '방향+거리' 모두 가까운 점을 고른다.
    measures 는 {angle:dist} 또는 [(angle,dist),...]. 없으면 (None, None, None).
    """
    items = measures.items() if hasattr(measures, "items") else measures
    tx = target_dist_mm * math.sin(math.radians(target_angle))
    ty = target_dist_mm * math.cos(math.radians(target_angle))
    best_a = best_d = best_d2 = None
    for a, d in items:
        if d <= 0:
            continue
        px = d * math.sin(math.radians(a))
        py = d * math.cos(math.radians(a))
        d2 = (px - tx) ** 2 + (py - ty) ** 2
        if best_d2 is None or d2 < best_d2:
            best_d2, best_a, best_d = d2, a, d
    if best_a is None:
        return None, None, None
    return best_a, best_d, math.sqrt(best_d2)
