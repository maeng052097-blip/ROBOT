"""visualization/lidar_probe_view.py — LiDAR 실험을 '눈으로' 보고 판단하는 OpenCV 뷰.

반사율 실험(A) 등을 시각적으로:
  - 레이더에 LiDAR 측정점 + 목표 방향 '쐐기(±window)'를 노란선으로 강조.
  - 쐐기 안의 점은 노란색으로, 그 안의 n_raw/n_bins/거리(cm)/각도폭을 실시간 표시.
  - 최근 N초 hit_rate 막대 + 큰 DETECTED / NONE 표시 -> 잡히는지 한눈에.
  - 실제 주행 파이프라인(dict 점수 / 전방최소 / SAFE·SLOW·DANGER)도 같이 표시.
판을 검정 -> 흰 -> 재귀반사로 바꾸며 쐐기 안 점이 생기는지 / 어느 거리까지 잡히는지 비교.

집계는 common.lidar_metrics 를 그대로 사용 -> '화면으로 본 판단 = 텍스트 도구 수치'.
matplotlib 대신 cv2.imshow 라 창이 안정적으로 뜬다.

조작:
  q       종료
  l       지금 화면(최근 hit-window)을 CSV 1행으로 기록(--object/--material/--distance-cm 라벨)
  [ / ]   window 각도 축소 / 확대
  - / =   bearing 1deg 감소 / 증가

필요: LiDAR(필수), 카메라 불필요.
실행: python visualization/lidar_probe_view.py --object board30 --material dark_cloth --distance-cm 200
"""
import sys
import csv
import time
import math
import pathlib
import argparse
from collections import deque

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))

from common.config import (
    LIDAR_PORT, LIDAR_BAUDRATE, LIDAR_MAX_AGE, PROJECT_ROOT,
    FORWARD_ANGLE_DEG, DANGER_MM, SLOW_MM,
)
from common.lidar_metrics import summarize_window, aggregate, theoretical_spacing_mm
from common.safety import forward_min_distance, classify_safety

RMAX_MM = 2000          # 표시 기본 최대거리(mm)=2m (근거리 잘 보이게). 멀리는 . 줌아웃 / --rmax
PANEL = 720             # 창 한 변(px)
HITWIN_S = 6.0          # 최근 hit_rate 계산 구간(초)
DEFAULT_CSV = PROJECT_ROOT / "lidar_probe_log.csv"
RES_MIN_DEG, RES_MAX_DEG = 0.6, 0.96


def zone_bgr(d):
    if d < DANGER_MM:
        return (0, 0, 255)
    if d < SLOW_MM:
        return (0, 165, 255)
    return (0, 200, 0)


def _txt(img, s, org, color, scale=0.5, thick=1):
    """검은 외곽선 + 색 글씨(점 위에서도 읽히게)."""
    import cv2
    cv2.putText(img, s, org, cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), thick + 2, cv2.LINE_AA)
    cv2.putText(img, s, org, cv2.FONT_HERSHEY_SIMPLEX, scale, color, thick, cv2.LINE_AA)


def ring_step_mm(rmax):
    """rmax 에 맞춰 링 간격(mm)을 정한다 (화면에 약 6~12개 링이 나오도록)."""
    for step in (100, 200, 250, 500, 1000, 2000):
        if rmax / step <= 12:
            return step
    return 2000


def draw(measures, bearing, window, s, hitrate, pipe, labels, rmax):
    import cv2
    import numpy as np

    size = PANEL
    img = np.full((size, size, 3), 22, np.uint8)
    cx = cy = size // 2
    max_r = size // 2 - 48

    # 거리 가이드 링 + 라벨 (rmax 에 맞춰 간격 자동 세분화)
    cv2.circle(img, (cx, cy), max_r, (70, 70, 70), 1)
    step = ring_step_mm(rmax)
    mm = step
    while mm <= rmax + 1:
        rr = int(mm / rmax * max_r)
        cv2.circle(img, (cx, cy), rr, (55, 55, 55), 1)
        lbl = f"{mm/10:.0f}cm" if mm < 1000 else f"{mm/1000:.1f}m"
        _txt(img, lbl, (cx + 3, cy - rr + 14), (90, 90, 90), 0.4)
        mm += step
    cv2.circle(img, (cx, cy), int(SLOW_MM / rmax * max_r), (0, 140, 210), 1)
    cv2.circle(img, (cx, cy), int(DANGER_MM / rmax * max_r), (0, 0, 210), 1)
    cv2.line(img, (cx, cy), (cx, cy - max_r), (0, 150, 0), 1)  # 정면(0deg)
    _txt(img, "front", (cx + 5, cy - max_r + 16), (0, 200, 0), 0.45)

    # 목표 방향 쐐기(±window) — 경계 노란선 + 중심선
    for edge in (bearing - window, bearing + window):
        rel = math.radians((edge - FORWARD_ANGLE_DEG) % 360)
        x = int(cx + max_r * math.sin(rel)); y = int(cy - max_r * math.cos(rel))
        cv2.line(img, (cx, cy), (x, y), (0, 215, 215), 1)
    relb = math.radians((bearing - FORWARD_ANGLE_DEG) % 360)
    xb = int(cx + max_r * math.sin(relb)); yb = int(cy - max_r * math.cos(relb))
    cv2.line(img, (cx, cy), (xb, yb), (0, 120, 120), 1)

    # 측정점: 쐐기 안=노랑, 밖=거리색
    for a, d in measures:
        if not (0 < d <= rmax):
            continue
        rel = math.radians((a - FORWARD_ANGLE_DEG) % 360)
        r = d / rmax * max_r
        x = int(cx + r * math.sin(rel)); y = int(cy - r * math.cos(rel))
        if abs((a - bearing + 180) % 360 - 180) <= window:
            cv2.circle(img, (x, y), 3, (0, 255, 255), -1)
        else:
            cv2.circle(img, (x, y), 2, zone_bgr(d), -1)

    # ---- 상단: 큰 DETECTED / NONE ----
    if s["n_raw"] > 0:
        _txt(img, "DETECTED", (16, 34), (0, 255, 0), 0.9, 2)
    else:
        _txt(img, "NONE", (16, 34), (40, 40, 255), 0.9, 2)
    fresh = "OK" if pipe["fresh"] else "STALE"
    _txt(img, f"LiDAR {LIDAR_PORT} [{fresh}]", (size - 210, 26), (220, 220, 220), 0.5)

    # ---- 좌상단: 지표 ----
    y0, dy = 60, 24
    lines = [
        f"bearing {bearing:.0f}deg  window +-{window:.0f}deg  range {rmax/1000:.1f}m",
        f"in-window: n_raw {s['n_raw']}   n_bins {s['n_bins']}",
    ]
    if s["d_med"] is not None:
        lines.append(f"dist {s['d_med']/10:.1f}cm  ({s['d_min']/10:.0f}~{s['d_max']/10:.0f}, "
                     f"sd{s['d_std']/10:.1f})  ang+-{s['ang_span']:.1f}")
        sp = theoretical_spacing_mm(s["d_med"], RES_MAX_DEG)
        lines.append(f"[ref] spacing@dist ~{sp:.0f}mm (res {RES_MAX_DEG}deg)")
    else:
        lines.append("dist -  (쐐기 안에 점 없음)")
    fwd = "-" if pipe["fwd"] is None else f"{pipe['fwd']}mm"
    lines.append(f"[pipeline] dict {pipe['dict_pts']}  fwd {fwd}  {pipe['state']}")
    for i, ln in enumerate(lines):
        _txt(img, ln, (16, y0 + i * dy), (235, 235, 235), 0.5)

    # ---- hit_rate 막대 ----
    by = y0 + len(lines) * dy + 8
    bw, bh = 260, 16
    cv2.rectangle(img, (16, by), (16 + bw, by + bh), (90, 90, 90), 1)
    fillc = (0, 200, 0) if hitrate >= 0.8 else (0, 165, 255) if hitrate >= 0.3 else (40, 40, 255)
    cv2.rectangle(img, (16, by), (16 + int(bw * hitrate), by + bh), fillc, -1)
    _txt(img, f"hit_rate(last {HITWIN_S:.0f}s) {hitrate*100:.0f}%", (16, by + bh + 18),
         (235, 235, 235), 0.5)

    # ---- 하단: 라벨 + 도움말 ----
    _txt(img, f"label: {labels.object}/{labels.material} @ {labels.distance_cm}cm",
         (16, size - 34), (180, 220, 255), 0.5)
    _txt(img, "q quit  l log  [ ] window  - = bearing  , . zoom", (16, size - 12),
         (160, 160, 160), 0.45)
    return img


def append_csv(path, labels, bearing, window, agg):
    cols = ["object", "material", "distance_cm", "scan_hz_note",
            "bearing_deg", "window_deg", "seconds", "snapshots",
            "hit_rate", "n_raw_mean", "n_bins_mean",
            "d_min_mm", "d_med_mm", "d_max_mm", "d_std_mm", "ang_span_deg",
            "dict_pts_mean", "fwd_found_rate", "state_counts"]

    def r(x, nd=1):
        return None if x is None else round(x, nd)

    row = {
        "object": labels.object, "material": labels.material,
        "distance_cm": labels.distance_cm, "scan_hz_note": labels.hz,
        "bearing_deg": round(bearing, 1), "window_deg": round(window, 1),
        "seconds": HITWIN_S, "snapshots": agg.get("snapshots"),
        "hit_rate": round(agg.get("hit_rate", 0.0), 3),
        "n_raw_mean": r(agg.get("n_raw_mean")), "n_bins_mean": r(agg.get("n_bins_mean")),
        "d_min_mm": r(agg.get("d_min_mm")), "d_med_mm": r(agg.get("d_med_mm")),
        "d_max_mm": r(agg.get("d_max_mm")), "d_std_mm": r(agg.get("d_std_mm")),
        "ang_span_deg": r(agg.get("ang_span_deg")),
        "dict_pts_mean": r(agg.get("dict_pts_mean")),
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


def main():
    ap = argparse.ArgumentParser(description="LiDAR 실험 시각 뷰(OpenCV 레이더)")
    ap.add_argument("--bearing", type=float, default=float(FORWARD_ANGLE_DEG))
    ap.add_argument("--window", type=float, default=5.0)
    ap.add_argument("--object", default="board30")
    ap.add_argument("--material", default="unknown")
    ap.add_argument("--distance-cm", dest="distance_cm", default="NA")
    ap.add_argument("--hz", default="NA")
    ap.add_argument("--csv", default=str(DEFAULT_CSV))
    ap.add_argument("--lidar", choices=["x2", "x4"], default="x2", help="LiDAR 모델(x2/x4)")
    ap.add_argument("--port", default=None, help="시리얼 포트(기본=config LIDAR_PORT)")
    ap.add_argument("--baud", type=int, default=None, help="통신속도(기본 x2=115200, x4=128000)")
    ap.add_argument("--rmax", type=float, default=float(RMAX_MM),
                    help="레이더 표시 최대거리(mm). 창에서 , . 로 줌인/줌아웃")
    args = ap.parse_args()

    import cv2
    from drivers import make_lidar

    port = args.port or LIDAR_PORT
    lidar = make_lidar(args.lidar, port, args.baud)
    if not lidar.open():
        print(f"LiDAR({port}) 연결 실패. 포트/모델(--lidar {args.lidar})/통신속도(--baud)를 확인하세요.")
        return
    print(f"LiDAR({args.lidar}) 연결: {port}. 창에서 q=종료, l=CSV기록, [ ]=window, - ==bearing")

    bearing, window = args.bearing, args.window
    rmax = max(500.0, args.rmax)
    history = deque()        # (t, hit?) 최근 HITWIN_S 초
    snaps = deque(maxlen=600)  # (t, snapshot) 최근 기록용

    warm = time.time() + 3.0
    while time.time() < warm and not lidar.getMeasures():
        time.sleep(0.1)

    win = "LiDAR probe view"
    cv2.namedWindow(win, cv2.WINDOW_AUTOSIZE)
    try:
        while True:
            measures = lidar.getMeasures()
            dd = lidar.getDistanceDict()
            s = summarize_window(measures, bearing, window)
            now = time.time()

            history.append((now, s["n_raw"] > 0))
            while history and now - history[0][0] > HITWIN_S:
                history.popleft()
            hitrate = (sum(1 for _, h in history if h) / len(history)) if history else 0.0

            pipe = {"dict_pts": len(dd), "fwd": forward_min_distance(dd),
                    "state": classify_safety(dd), "fresh": lidar.is_fresh(LIDAR_MAX_AGE)}

            rec = dict(s)
            rec["total_pts"] = len(measures); rec["dict_pts"] = pipe["dict_pts"]
            rec["fwd"] = pipe["fwd"]; rec["state"] = pipe["state"]
            snaps.append((now, rec))

            img = draw(measures, bearing, window, s, hitrate, pipe, args, rmax)
            cv2.imshow(win, img)

            k = cv2.waitKey(30) & 0xFF
            if k == ord("q"):
                break
            elif k == ord("["):
                window = max(1.0, window - 1.0)
            elif k == ord("]"):
                window = min(45.0, window + 1.0)
            elif k == ord("-"):
                bearing = (bearing - 1.0) % 360
            elif k in (ord("="), ord("+")):
                bearing = (bearing + 1.0) % 360
            elif k == ord(","):
                rmax = max(500.0, rmax / 1.5)
            elif k == ord("."):
                rmax = min(12000.0, rmax * 1.5)
            elif k == ord("l"):
                recent = [r for t, r in snaps if now - t <= HITWIN_S]
                agg = aggregate(recent)
                append_csv(args.csv, args, bearing, window, agg)
                print(f"[log] {args.material}@{args.distance_cm}cm  "
                      f"hit_rate={agg.get('hit_rate', 0)*100:.0f}%  "
                      f"n_raw_mean={agg.get('n_raw_mean')}  "
                      f"d_med={agg.get('d_med_mm')}mm  -> {args.csv}")

            if cv2.getWindowProperty(win, cv2.WND_PROP_VISIBLE) < 1:
                break
    except KeyboardInterrupt:
        pass
    finally:
        lidar.close()
        cv2.destroyAllWindows()
        print("종료")


if __name__ == "__main__":
    main()
