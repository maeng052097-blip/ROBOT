"""tests/lidar_range_probe.py — LiDAR 원거리/소형물체 탐지 한계 '실측' 도구 (Part 1).

목적:
  "큰 물체(사람)가 ~1.2m 부터만 잡힌다"의 원인을 추측이 아니라 '데이터'로 가린다.
  세 가설을 구분한다:
    (A) 반사율/신호  : 반환광 ∝ 반사율/거리². 어둡고 흡수성인 표면은 멀면 신호가 약해 미검출.
    (B) 소프트웨어    : 원시 점은 들어오는데 평활/군집/전방부채꼴 단계에서 버려짐.
    (C) 스캔레이트/기하: 회전당 점 수가 적어 작은 물체에 점이 안 맺힘.

측정 방식(사실 기반):
  - getMeasures() 의 '원시' (angle, distance_mm) 점을 평활 전 그대로 본다.
  - 지정 방향(--bearing, 기본 FORWARD_ANGLE_DEG) ±--window 안의 점만 모아
    개수 / 거리(min·중앙·max·표준편차) / 각도폭 / 각도밀도를 집계(측정값).
  - 동시에 getDistanceDict()+forward_min_distance()(실제 주행 파이프라인)도 보여
    '원시엔 보이는데 파이프라인이 잃는지'(가설 B)를 비교.
  - 한 번 실행 = 한 조건(물체/재질/거리). 결과를 CSV 1행으로 누적 -> 조건 간 비교.

  ※ 같은 집계를 '눈으로' 보려면 visualization/lidar_probe_view.py (OpenCV 레이더) 사용.

구분 실험 설계(권장 절차):
  (A) 같은 거리에서 '재질'만 바꿔 측정:
        --material dark_cloth   vs   white_foam   vs   retro_tape
      흰/재귀반사가 훨씬 멀리(또는 hit_rate 높게) 잡히면 -> 원인 = 반사율(A).
  (B) 물체를 정면(bearing=FORWARD)에 두고: 원시 window엔 점이 있는데(n_raw>0)
      forward_min_distance 가 '없음' 이거나 classify 가 SAFE 면 -> 원인 = SW(B).
  (C) 같은 물체/거리에서 점 개수(n_raw)·각도밀도를 거리별로 비교(기하 곡선).
  (소형 경계) 폭이 다른 물체(30/15/7 cm)를 거리별로 -> n_bins(맞은 각도수) 곡선.

실행 예:
  python tests/lidar_range_probe.py --object person --material dark_cloth --distance-cm 120
  python tests/lidar_range_probe.py --object foam   --material white_foam --distance-cm 300 --window 6 --seconds 10
  python tests/lidar_range_probe.py --bearing 0 --raw       # 원시점까지 덤프

종료: --seconds 후 자동 종료. 요약 출력 + CSV 1행 기록.
"""
import sys
import csv
import time
import pathlib
import argparse

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))

from common.config import (
    PROJECT_ROOT, LIDAR_PORT, LIDAR_BAUDRATE, LIDAR_MAX_AGE,
    FORWARD_ANGLE_DEG, DANGER_MM, SLOW_MM,
)
from common.safety import forward_min_distance, classify_safety
# 순수 분석 함수는 common.lidar_metrics 로 분리(텍스트 실측 + 시각 뷰가 공유).
from common.lidar_metrics import (
    window_points, summarize_window, aggregate, theoretical_spacing_mm,
)

DEFAULT_CSV = PROJECT_ROOT / "lidar_probe_log.csv"
# 이론 점간격 계산에 쓰는 X2 각도분해능(스펙: 0.6~0.96deg). '가정값'임을 명시한다.
RES_MIN_DEG, RES_MAX_DEG = 0.6, 0.96


# ---------- 수집/표시 (하드웨어 필요) ----------

def collect(lidar, target_deg, window_deg, seconds, raw=False, period=0.2):
    snapshots = []
    deadline = time.time() + seconds
    while time.time() < deadline:
        measures = lidar.getMeasures()
        dd = lidar.getDistanceDict()
        s = summarize_window(measures, target_deg, window_deg)
        s["total_pts"] = len(measures)
        s["dict_pts"] = len(dd)
        s["fwd"] = forward_min_distance(dd)
        s["state"] = classify_safety(dd)
        s["fresh"] = lidar.is_fresh(LIDAR_MAX_AGE)
        snapshots.append(s)

        remain = max(0.0, deadline - time.time())
        fresh = "OK  " if s["fresh"] else "STALE"
        if s["n_raw"] > 0:
            line = (f"[{fresh}] t-{remain:4.1f}s win+-{window_deg:g}@{target_deg:g}  "
                    f"raw {s['n_raw']:3d} bins {s['n_bins']:2d}  "
                    f"d {s['d_med']/10:5.1f}cm({s['d_min']/10:.0f}~{s['d_max']/10:.0f} s{s['d_std']/10:.1f})  "
                    f"ang+-{s['ang_span']:.1f}  | dict {s['dict_pts']:3d} "
                    f"fwd {'-' if s['fwd'] is None else str(s['fwd'])+'mm'} {s['state']}      ")
        else:
            line = (f"[{fresh}] t-{remain:4.1f}s win+-{window_deg:g}@{target_deg:g}  "
                    f"raw   0 (window에 점 없음)  | dict {s['dict_pts']:3d} "
                    f"fwd {'-' if s['fwd'] is None else str(s['fwd'])+'mm'} {s['state']}      ")
        print(line, end="\r")

        if raw and s["n_raw"] > 0:
            pts = window_points(measures, target_deg, window_deg)
            pts.sort()
            dump = ", ".join(f"{r:+.1f}deg:{d/10:.0f}cm" for r, d in pts[:24])
            print(f"\n    raw[{len(pts)}]: {dump}")

        time.sleep(period)
    print()
    return snapshots


def print_summary(args, agg):
    print("\n" + "=" * 64)
    print("조건:")
    print(f"  object={args.object}  material={args.material}  "
          f"distance(ground-truth)={args.distance_cm}cm  scan_hz_note={args.hz}")
    print(f"  bearing={args.bearing}deg  window=+-{args.window}deg  seconds={args.seconds}s")
    print("-" * 64)
    print("측정 결과:")
    print(f"  스냅샷 수      : {agg['snapshots']}")
    print(f"  hit_rate       : {agg['hit_rate']*100:.0f}%  "
          f"(window 안에 점이 1개 이상 잡힌 스냅샷 비율)")
    if agg.get("d_med_mm") is not None:
        dm = agg["d_med_mm"]
        print(f"  원시 거리      : 중앙 {dm/10:.1f}cm "
              f"(min {agg['d_min_mm']/10:.1f} ~ max {agg['d_max_mm']/10:.1f}, "
              f"sigma {agg['d_std_mm']/10:.1f}cm)")
        print(f"  원시 점/스냅샷 : n_raw 평균 {agg['n_raw_mean']:.1f}, "
              f"n_bins 평균 {agg['n_bins_mean']:.1f}, "
              f"각도폭 평균 {agg['ang_span_deg']:.1f}deg")
        sp_min = theoretical_spacing_mm(dm, RES_MIN_DEG)
        sp_max = theoretical_spacing_mm(dm, RES_MAX_DEG)
        print(f"  [이론/가정] 점간격@{dm/10:.0f}cm : "
              f"{sp_min:.0f}~{sp_max:.0f}mm (분해능 {RES_MIN_DEG}~{RES_MAX_DEG}deg 가정)")
    else:
        print("  원시 거리      : (window에서 점이 한 번도 안 잡힘 -> 이 거리/재질에서 미검출)")
    print(f"  파이프라인     : dict 점수 평균 "
          f"{('%.0f' % agg['dict_pts_mean']) if agg.get('dict_pts_mean') is not None else '-'}, "
          f"전방최소 발견율 {agg['fwd_found_rate']*100:.0f}%, "
          f"상태 {agg.get('state_counts', {})}")
    print("-" * 64)
    print("해석 힌트:")
    print("  - 같은 거리에서 흰/재귀반사가 어두운것보다 hit_rate↑ & n_raw↑ => 원인=반사율(A)")
    print("  - 원시 n_raw>0 인데 전방최소 발견율 낮음/상태 SAFE => 원인=소프트웨어(B)")
    print("  - 거리↑ 일수록 n_bins↓(점간격↑). 측정 n_bins 곡선이 곧 소형 탐지 경계(C/기하)")
    print("=" * 64)


def append_csv(path, args, agg):
    cols = ["object", "material", "distance_cm", "scan_hz_note",
            "bearing_deg", "window_deg", "seconds", "snapshots",
            "hit_rate", "n_raw_mean", "n_bins_mean",
            "d_min_mm", "d_med_mm", "d_max_mm", "d_std_mm", "ang_span_deg",
            "dict_pts_mean", "fwd_found_rate", "state_counts"]
    row = {
        "object": args.object, "material": args.material,
        "distance_cm": args.distance_cm, "scan_hz_note": args.hz,
        "bearing_deg": args.bearing, "window_deg": args.window,
        "seconds": args.seconds, "snapshots": agg.get("snapshots"),
        "hit_rate": round(agg.get("hit_rate", 0.0), 3),
        "n_raw_mean": _r(agg.get("n_raw_mean")), "n_bins_mean": _r(agg.get("n_bins_mean")),
        "d_min_mm": _r(agg.get("d_min_mm")), "d_med_mm": _r(agg.get("d_med_mm")),
        "d_max_mm": _r(agg.get("d_max_mm")), "d_std_mm": _r(agg.get("d_std_mm")),
        "ang_span_deg": _r(agg.get("ang_span_deg")),
        "dict_pts_mean": _r(agg.get("dict_pts_mean")),
        "fwd_found_rate": round(agg.get("fwd_found_rate", 0.0), 3),
        "state_counts": agg.get("state_counts", {}),
    }
    path = pathlib.Path(path)
    new = not path.exists()
    with open(path, "a", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=cols)
        if new:
            w.writeheader()
        w.writerow(row)
    print(f"\nCSV 기록: {path}  (조건별로 누적해 비교하세요)")


def _r(x, nd=1):
    return None if x is None else round(x, nd)


def main():
    ap = argparse.ArgumentParser(description="YDLIDAR X2 원거리/소형 탐지 한계 실측")
    ap.add_argument("--bearing", type=float, default=float(FORWARD_ANGLE_DEG),
                    help="측정할 방향(deg). 기본=FORWARD_ANGLE_DEG")
    ap.add_argument("--window", type=float, default=5.0, help="방향 ±window(deg)")
    ap.add_argument("--seconds", type=float, default=8.0, help="수집 시간(초)")
    ap.add_argument("--object", default="unknown", help="물체 라벨(예: person, can)")
    ap.add_argument("--material", default="unknown",
                    help="표면 라벨(예: dark_cloth, white_foam, retro_tape)")
    ap.add_argument("--distance-cm", dest="distance_cm", default="NA",
                    help="실측 거리(cm, ground-truth). 줄자로 잰 값을 기록")
    ap.add_argument("--hz", default="NA", help="스캔레이트 메모(드라이버로 변경 불가일 수 있음)")
    ap.add_argument("--csv", default=str(DEFAULT_CSV), help="결과 누적 CSV 경로")
    ap.add_argument("--lidar", choices=["x2", "x4"], default="x2", help="LiDAR 모델(x2/x4)")
    ap.add_argument("--port", default=None, help="시리얼 포트(기본=config LIDAR_PORT). X4 는 다른 COM 일 수 있음")
    ap.add_argument("--baud", type=int, default=None, help="통신속도(기본 x2=115200, x4=128000)")
    ap.add_argument("--raw", action="store_true", help="원시 window 점도 덤프")
    args = ap.parse_args()

    # 드라이버는 여기서만 임포트(serial 의존). 순수 함수는 common 에서 테스트 가능.
    from drivers import make_lidar

    port = args.port or LIDAR_PORT
    lidar = make_lidar(args.lidar, port, args.baud)
    if not lidar.open():
        print(f"LiDAR({port}) 연결 실패. 포트/모델(--lidar {args.lidar})/통신속도(--baud)를 확인하세요.")
        return
    print(f"LiDAR({args.lidar}) 연결: {port} | DANGER<{DANGER_MM} SLOW<{SLOW_MM}mm")
    print(f"측정: {args.object}/{args.material} @ {args.distance_cm}cm, "
          f"bearing {args.bearing}deg +-{args.window}deg, {args.seconds}s\n")

    # 워밍업: 첫 스캔이 들어올 때까지 잠깐 대기
    warm = time.time() + 3.0
    while time.time() < warm and not lidar.getMeasures():
        time.sleep(0.1)

    try:
        snaps = collect(lidar, args.bearing, args.window, args.seconds, raw=args.raw)
    except KeyboardInterrupt:
        print("\n중단됨")
        snaps = []
    finally:
        lidar.close()

    if snaps:
        agg = aggregate(snaps)
        print_summary(args, agg)
        append_csv(args.csv, args, agg)


if __name__ == "__main__":
    main()
