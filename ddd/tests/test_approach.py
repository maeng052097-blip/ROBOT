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

    # ---- cam_approach_command: 카메라 단독 원거리 단계(Q1-b) ----
    from common.approach import cam_approach_command
    # 측정 없음 -> LOST 정지
    assert cam_approach_command(None, 0.0) == (0, 0, 0, "LOST")
    # 60cm 한계 미만인데 아직 이 단계(=LiDAR 미획득) -> CAM_LIMIT 정지
    assert cam_approach_command(550.0, 0.0, min_cam_mm=600.0) == (0, 0, 0, "CAM_LIMIT")
    # 멀고 정면 -> 서행 전진
    vx, vy, w, st = cam_approach_command(2000.0, 0.0, min_cam_mm=600.0, vx_far=22)
    assert (vx, vy, w, st) == (22, 0, 0, "CAM_APPROACH"), (vx, vy, w, st)
    # 옆에 있으면 회전만(정면 정렬 전 전진 금지)
    vx, vy, w, st = cam_approach_command(2000.0, 25.0, face_tol_deg=8.0, w_max=30)
    assert vx == 0 and w == 30 and st == "CAM_APPROACH", (vx, vy, w, st)
    # 좌측 표적 -> 좌회전(-)
    _, _, w, _ = cam_approach_command(2000.0, -25.0, w_max=30)
    assert w == -30, w
    print("  OK cam_approach: LOST/CAM_LIMIT stop, align-then-creep, turn signs")

    # ---- blocking_distance: 접근 중 난입 차단물 감지(표적은 차단물 아님) ----
    from common.fusion import blocking_distance
    # 표적 1m 추적 중 전방 2도에 20cm 난입 -> 차단물 200
    assert blocking_distance({0: 1000, 2: 200}, 1000.0) == 200
    # 표적 점만 있음 -> None (1000 >= 1000-200)
    assert blocking_distance({0: 1000, 3: 950}, 1000.0) is None
    # 도착 직전(표적 18cm): 표적 자체로는 절대 오발 불가(d < -20 불가능)
    assert blocking_distance({0: 180, 1: 175}, 180.0) is None
    # 옆(90도)의 가까운 점은 전방 호 밖 -> 무시
    assert blocking_distance({90: 150, 0: 1000}, 1000.0) is None
    # 표적 거리 미상 -> None
    assert blocking_distance({0: 200}, None) is None
    print("  OK blocking_distance: intruder caught, target/side/arrival never trigger")

    print("OK (all passed)")


if __name__ == "__main__":
    main()
