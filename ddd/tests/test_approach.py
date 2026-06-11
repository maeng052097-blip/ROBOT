"""tests/test_approach.py — 물체 접근 제어 법칙 단위테스트(하드웨어 불필요).

실행: py -3.13 tests/test_approach.py
"""
import sys
import pathlib

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))

from common.approach import approach_command  # noqa: E402

TARGET = 180.0


def main():
    print("test_approach:")

    # 물체 놓침
    assert approach_command(None, 0.0, TARGET) == (0, 0, 0, "LOST")
    print("  OK None range -> LOST (stop)")

    # 너무 가까움(블라인드존) -> 정지
    assert approach_command(100.0, 0.0, TARGET, min_safe_mm=130.0) == (0, 0, 0, "TOO_CLOSE")
    print("  OK below min-safe -> TOO_CLOSE (stop)")

    # 도착: 정면 + 목표 deadband 안
    assert approach_command(185.0, 2.0, TARGET, deadband_mm=25.0, face_tol_deg=8.0) == (0, 0, 0, "ARRIVED")
    print("  OK at target & facing -> ARRIVED (stop)")

    # 멀고 정면 -> 전진
    vx, vy, w, st = approach_command(500.0, 0.0, TARGET)
    assert vx > 0 and w == 0 and vy == 0 and st == "APPROACH", (vx, vy, w, st)
    print("  OK far & facing -> forward")

    # 물체가 오른쪽(+) -> 우회전(+), 아직 정면 아니라 전진 보류
    vx, vy, w, st = approach_command(500.0, 30.0, TARGET, face_tol_deg=8.0)
    assert w > 0 and vx == 0 and st == "APPROACH", (vx, vy, w, st)
    print("  OK object on right -> turn right, no forward yet")

    # 목표보다 가깝지만 사거리 안 -> 후진해서 목표로
    vx, vy, w, st = approach_command(140.0, 0.0, TARGET, min_safe_mm=130.0)
    assert vx < 0 and st == "APPROACH", (vx, vy, w, st)
    print("  OK closer than target (but >min-safe) -> back up")

    # 속도 상한
    vx, _, _, _ = approach_command(9000.0, 0.0, TARGET, vx_max=35)
    assert vx == 35, vx
    _, _, w, _ = approach_command(500.0, 90.0, TARGET, w_max=30)
    assert w == 30, w
    print("  OK speed clamps (vx_max / w_max)")

    # ---- RangeGate: 원거리 배경 스파이크 억제(실측: 2m+ 값 튐) ----
    from common.approach import RangeGate
    g = RangeGate(gate_mm=300, hold_s=0.8)
    assert g.update(2000, 0.0) == (2000, "TRACK")     # 첫 측정 수용
    assert g.update(2050, 0.1) == (2050, "TRACK")     # 게이트 내 갱신(표적 추적)
    assert g.update(5200, 0.2) == (2050, "HOLD")      # 배경 스파이크(+3m 점프) 거부+유지
    assert g.update(None, 0.5) == (2050, "HOLD")      # 빔 미스도 유지
    assert g.update(2100, 0.6) == (2100, "TRACK")     # 표적 복귀 -> 즉시 재추적
    assert g.update(None, 0.7)[1] == "HOLD"
    assert g.update(None, 1.6) == (None, "LOST")      # hold 0.8s 초과 -> 상실
    assert g.update(5200, 1.7) == (5200, "TRACK")     # 상실 후 새 값 수용(재획득)
    print("  OK RangeGate: spike->HOLD, miss->HOLD, expire->LOST, reacquire")

    # 움직이는 표적 추적: 게이트 창이 수용값 기준으로 따라감(접근 시나리오)
    g.reset()
    t = 0.0
    for d in (2000, 1750, 1500, 1280, 1050):          # 스텝 250~280mm < 게이트 300
        r, st = g.update(d, t)
        assert (r, st) == (d, "TRACK"), (d, r, st)
        t += 0.15
    print("  OK RangeGate follows approaching target (window re-centers)")

    print("OK (all passed)")


if __name__ == "__main__":
    main()
