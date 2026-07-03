"""depth_calibrate.py — 뎁스 보정계수 산출/검증. 하드웨어 불필요(순수 계산).

깊이 오차는 거리^2로 커지므로(스테레오: dZ = Z^2*dd/(fx*B)) 근~원 전 구간을
단일직선으로 못 맞출 수 있다. 이 도구는:
  - 단일직선(Z_true=scale*Z+offset) 적합 + 노드별 잔차(mm 와 %)
  - 단조성 검사(실제값이 측정값과 함께 증가하는지)
  - 홀드아웃 검증(적합에 안 쓴 거리로 실측 검증) — 진짜 정확도 판정
  - 근/원(기본 800mm) 분리 회귀 비교
  - 항상 사용가능한 구간별 LUT 문자열 출력
을 제공하고, 단일직선 vs 구간별(LUT) 중 무엇을 쓸지 추천한다.

입력: '측정mm:실제mm' 또는 '측정mm:실제mm:프레임std' (std 선택). 실제거리는 mm.
실행 예:
  py -3.13 tests/depth_calibrate.py 296:300 493:500 686:700
  py -3.13 tests/depth_calibrate.py 150:152:0.2 200:203:0.3 300:299 ... --holdout 450,1200
  (--holdout 은 '실제거리' 목록; 그 점은 적합에서 빼고 검증에만 씀)
주의: 해상도/프리셋/노출/재장착이 바뀌면 fx·Min-Z·노이즈가 달라져 보정이 무효 -> 재측정.
"""
import sys
import argparse

try:    # 콘솔(cp949)에 없는 문자를 print 해도 앱이 안 죽게
    sys.stdout.reconfigure(errors="replace")
    sys.stderr.reconfigure(errors="replace")
except Exception:
    pass


def fit(pairs):
    """pairs=[(meas, true), ...] -> (scale, offset). 최소제곱 1차. 점<2 또는 x분산0이면 None."""
    n = len(pairs)
    if n < 2:
        return None
    mx = sum(p[0] for p in pairs) / n
    my = sum(p[1] for p in pairs) / n
    sxx = sum((p[0] - mx) ** 2 for p in pairs)
    if sxx == 0:
        return None
    scale = sum((p[0] - mx) * (p[1] - my) for p in pairs) / sxx
    return scale, my - scale * mx


def parse_triplet(tok):
    """'측정:실제' 또는 '측정:실제:std' -> (meas, true, std|None)."""
    parts = tok.split(":")
    m, t = float(parts[0]), float(parts[1])
    s = float(parts[2]) if len(parts) >= 3 else None
    return m, t, s


def report_residuals(label, scale, offset, pts):
    """pts=[(meas,true)] 에 대해 노드별 잔차 mm 와 % 출력, 최대 절대잔차 반환."""
    worst = 0.0
    print(f"{label} (측정 -> 보정 vs 실제):")
    for m, t in sorted(pts):
        c = scale * m + offset
        err = c - t
        pct = (err / t * 100.0) if t else 0.0
        worst = max(worst, abs(err))
        print(f"  meas {m:8.1f} -> corr {c:8.1f}   (true {t:8.1f}, err {err:+7.1f} mm / {pct:+5.2f}%)")
    return worst


def main():
    ap = argparse.ArgumentParser(description="뎁스 보정계수 산출/검증(순수 계산)")
    ap.add_argument("pairs", nargs="+", help="'측정:실제' 또는 '측정:실제:std' (mm) 여러 개")
    ap.add_argument("--holdout", default="", help="적합서 제외할 '실제거리' 목록(mm), 예 '450,1200'. 검증용")
    ap.add_argument("--split", type=float, default=800.0, help="근/원 분리 경계(mm). 기본 800")
    args = ap.parse_args()

    triples = []
    for tok in args.pairs:
        if ":" not in tok:
            print(f"[무시] 형식 아님: {tok}")
            continue
        try:
            triples.append(parse_triplet(tok))
        except ValueError:
            print(f"[무시] 숫자 아님: {tok}")
    if len(triples) < 2:
        print("사용법: py -3.13 tests/depth_calibrate.py 측정:실제[:std] ... (2쌍 이상)")
        return

    holdset = set()
    for x in args.holdout.split(","):
        x = x.strip()
        if x:
            holdset.add(float(x))

    all_pts = [(m, t) for m, t, _ in triples]
    fit_pts = [(m, t) for m, t, _ in triples if t not in holdset]
    hold_pts = [(m, t) for m, t, _ in triples if t in holdset]
    stds = {t: s for m, t, s in triples if s is not None}

    # --- 단조성 검사 (측정 오름차순일 때 실제도 증가해야 함) ---
    srt = sorted(all_pts)
    mono = all(srt[i][1] < srt[i + 1][1] for i in range(len(srt) - 1))
    if not mono:
        print("[경고] 단조 아님: 측정이 커지는데 실제가 감소하는 구간이 있음 -> 측정오류 의심. LUT도 비단조가 됨.")

    r = fit(fit_pts)
    if r is None:
        print("[실패] 적합점이 부족하거나 측정값이 모두 같음(서로 다른 거리 필요).")
        return
    scale, offset = r
    tmax = max(t for _, t in all_pts)

    print(f"[단일직선] Z_true = {scale:.5f} * Z_meas + ({offset:.2f}) mm   (적합점 {len(fit_pts)}개)")
    worst = report_residuals("적합 잔차", scale, offset, fit_pts)
    print(f"적합 최대 잔차: {worst:.1f} mm")

    hold_worst = 0.0
    if hold_pts:
        hold_worst = report_residuals("[홀드아웃 검증]", scale, offset, hold_pts)
        print(f"홀드아웃 최대 잔차: {hold_worst:.1f} mm  (이게 진짜 정확도 지표)")

    # --- 근/원 분리 비교 ---
    near = [(m, t) for m, t in all_pts if t < args.split]
    far = [(m, t) for m, t in all_pts if t >= args.split]
    if len(near) >= 2 and len(far) >= 2:
        rn, rf = fit(near), fit(far)
        print(f"\n[근/원 분리 @ {args.split:.0f}mm]")
        if rn:
            wn = report_residuals(f"  근거리(<{args.split:.0f}) 직선", rn[0], rn[1], near)
            print(f"  근거리 직선: scale={rn[0]:.5f} offset={rn[1]:.2f}  최대잔차 {wn:.1f}mm")
        if rf:
            wf = report_residuals(f"  원거리(>={args.split:.0f}) 직선", rf[0], rf[1], far)
            print(f"  원거리 직선: scale={rf[0]:.5f} offset={rf[1]:.2f}  최대잔차 {wf:.1f}mm")

    # --- 노이즈(std) 반영 ---
    if stds:
        print("\n[노이즈 std(입력값)]")
        for t in sorted(stds):
            print(f"  true {t:.0f}mm: 프레임간 std {stds[t]:.1f}mm  ({stds[t]/t*100:.2f}%)")
        print("  (참고: 보정은 계통오차만 줄임. std=랜덤노이즈는 못 줄임 -> 먼거리는 '탐지·조향'용)")

    # --- LUT(전 구간, 항상 사용가능) ---
    lut = ",".join(f"{m:.0f}:{t:.0f}" for m, t in sorted(all_pts))

    # --- 추천 (밴드별 허용치: 근거리 max(2mm,0.5%Z), 원거리 max(1%Z)) ---
    def band_tol(t):
        return max(2.0, 0.005 * t) if t < args.split else max(0.01 * t, 2.0)

    judged = hold_pts if hold_pts else fit_pts
    kind = "홀드아웃" if hold_pts else "적합"
    exceed = [(t, abs(scale * m + offset - t), band_tol(t))
              for m, t in judged if abs(scale * m + offset - t) > band_tol(t)]
    _ = (worst, hold_worst, tmax)     # (진단용 값 보존)
    print()
    if not exceed:
        print(f"[추천] 단일직선 충분 ({kind} 전 노드가 밴드허용 내). 사용:")
        print(f"  py -3.13 visualization/depth_detect.py --depth-scale {scale:.5f} --depth-offset {offset:.2f}")
    else:
        print(f"[추천] 구간별(LUT) 권장 ({kind}에서 밴드허용 초과 {len(exceed)}개 -> 단일직선 부적합):")
        for t, err, tl in sorted(exceed):
            print(f"    true {t:.0f}mm: 오차 {err:.1f} > 허용 {tl:.1f}mm")
        print("  사용:")
        print(f"  py -3.13 visualization/depth_detect.py --depth-lut \"{lut}\"")
    if not hold_pts:
        print("  [주의] 홀드아웃 미지정 -> 위는 적합점 기준(과적합 가능). "
              "실제 정확도는 --holdout 로 안 쓴 '실제거리'를 검증해야 확정.")
    print(f"\n[참고] 전 구간 LUT 문자열(항상 사용가능): --depth-lut \"{lut}\"")


if __name__ == "__main__":
    main()
