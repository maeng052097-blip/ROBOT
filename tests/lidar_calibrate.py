"""tests/lidar_calibrate.py — LiDAR 거리 보정값(scale, offset) 계산.

(실제cm:측정cm) 쌍들을 주면 최소제곱으로
    true_mm = SCALE * measured_mm + OFFSET_MM
를 구해 config 에 넣을 값과 잔차를 출력한다. 하드웨어 불필요(순수 계산).

측정 방법:
  py -3.13 visualization/lidar_probe_view.py --lidar x2 --port COM12 --distance-cm <줄자값>
  로 각 거리에서 화면의 'dist'(cm)를 읽어 (실제:측정) 쌍을 만든다.

실행 예:
  py -3.13 tests/lidar_calibrate.py --pairs 30:39.5,100:109.8,200:209.6
  (왼쪽=줄자 실제 cm, 오른쪽=화면 측정 cm)
"""
import argparse


def fit(pairs_mm):
    """pairs_mm: [(measured_mm, true_mm), ...] -> (scale, offset_mm)."""
    n = len(pairs_mm)
    if n == 1:
        m, t = pairs_mm[0]
        return 1.0, t - m            # 1점이면 scale=1 가정, offset 만
    sm = sum(m for m, _ in pairs_mm)
    st = sum(t for _, t in pairs_mm)
    smm = sum(m * m for m, _ in pairs_mm)
    smt = sum(m * t for m, t in pairs_mm)
    denom = n * smm - sm * sm
    if denom == 0:
        return 1.0, (st - sm) / n
    scale = (n * smt - sm * st) / denom
    offset = (st - scale * sm) / n
    return scale, offset


def main():
    ap = argparse.ArgumentParser(description="LiDAR 거리 보정값 계산")
    ap.add_argument("--pairs", required=True,
                    help="실제cm:측정cm 쌍(쉼표구분). 예 30:39.5,100:109.8,200:209.6")
    args = ap.parse_args()

    pairs = []
    for tok in args.pairs.split(","):
        t_cm, m_cm = tok.split(":")
        pairs.append((float(m_cm) * 10.0, float(t_cm) * 10.0))  # (measured_mm, true_mm)

    scale, offset = fit(pairs)
    print("입력 (실제cm, 측정cm):")
    for m_mm, t_mm in pairs:
        print(f"  실제 {t_mm/10:6.1f}  측정 {m_mm/10:6.1f}")
    print("-" * 50)
    print("권장 보정값 (common/config.py 에 기입):")
    print(f"  LIDAR_X2_DIST_SCALE = {scale:.4f}")
    print(f"  LIDAR_X2_DIST_OFFSET_MM = {offset:.1f}")
    print("-" * 50)
    print("보정 후 예상 잔차(보정값 - 실제):")
    worst = 0.0
    for m_mm, t_mm in pairs:
        pred = scale * m_mm + offset
        err = pred - t_mm
        worst = max(worst, abs(err))
        print(f"  실제 {t_mm/10:6.1f}cm -> 보정 {pred/10:6.1f}cm  (오차 {err/10:+.2f}cm)")
    print(f"최대 잔차 {worst/10:.2f}cm")
    if abs(scale - 1.0) < 0.01:
        print("해석: scale 약 1.0 -> 사실상 '상수 오프셋'. OFFSET 만으로 충분.")
    else:
        print(f"해석: scale={scale:.3f} -> 거리에 비례하는 성분 존재(스케일 보정 필요).")


if __name__ == "__main__":
    main()
