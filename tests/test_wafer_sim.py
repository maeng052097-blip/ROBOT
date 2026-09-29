"""tests/test_wafer_sim.py — CELL_V9 시뮬레이터 단위테스트(하드웨어 불필요, 스레드 없음).

  - 원점 -> 카세트 준비(TOF) -> 10장 전체: 알람 없이 10장 인계, 사이클 82~106s(가정 속도)
  - dt 0.02 와 0.2 의 사이클 차이 0.1s 이내(잔여 dt 이월)
  - 빈 슬롯(연속 투입 가정 위반) -> WAIT_OPERATOR -> '빈 슬롯' 확정 -> 계속, 불일치 표시
  - S4 오감지(거짓 없음) -> '재시도' 로 복구, 다음 로봇 지연 -> WAIT_S5
  - S3 고장 -> ALARM -> reset/고장 해제/resume 로 복구
  - 단계 10 에서 E-STOP -> 재시작 시 검사 크래들 확인 요구, 확인만 하고 치우지 않으면 단계 7 충돌
  - 벨트 이빨 튐 -> S1 불일치 알람, debug_hold 로 단계 6 선회 중간 정지, 시뮬 예외는 tick 에서 raise

실행: py -3.13 tests/test_wafer_sim.py
"""
import sys
import pathlib

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))


def run_until(sim, cond, dt=0.1, limit=200000):
    for _ in range(limit):
        if cond(sim):
            return True
        sim.tick(dt)
    return cond(sim)


def ready_sim(slots=None, seed=1):
    from wafer_cell.sim import CellSim
    s = CellSim(seed=seed)
    if slots is not None:
        s.command("load_map", slots=slots)
    s.command("home")
    assert run_until(s, lambda x: x.state != "HOMING", dt=0.02)
    assert s.state == "READY", (s.state, s.alarm)
    return s


def main():
    from wafer_cell.sim import CellSim, CommandError

    print("test_wafer_sim:")
    s = CellSim()
    try:
        s.command("start")
        raise AssertionError("start before home must fail")
    except CommandError as e:
        assert e.code == "bad_state"
    s = ready_sim()
    assert s.homed == {"theta": True, "z": True, "r": True}
    assert (s.theta, s.zc, s.rx) == (0.0, 95.0, 0.0)
    r = s.command("cassette_ready")
    assert r["n_est"] == 10
    s.command("start")
    assert run_until(s, lambda x: x.state not in ("RUNNING", "WAIT_S5"))
    assert s.state == "CASSETTE_DONE" and s.alarm is None, (s.state, s.alarm)
    assert sum(1 for w in s.wafers if w["loc"] == "handed_off") == 10
    assert len(s.cycles) == 10 and 82 < s.cycles[0] < 86 and 102 < s.cycles[-1] < 106, s.cycles
    snap = s.snapshot()
    assert snap["counts"]["done"] == 10 and snap["state"] == "CASSETTE_DONE"
    print("  OK full cassette: 10 handed off, cycles", s.cycles[0], "..", s.cycles[-1])

    res = []
    for dt in (0.02, 0.2):
        x = ready_sim([True, True] + [False] * 8)
        x.command("cassette_ready")
        x.command("start")
        run_until(x, lambda y: y.state == "CASSETTE_DONE", dt=dt)
        res.append(x.cycles)
    assert all(abs(a - b) < 0.1 for a, b in zip(*res)), res
    print("  OK dt invariance", res)

    x = ready_sim([True, True, False, True] + [False] * 6)
    assert x.command("cassette_ready")["n_est"] == 4
    x.command("start")
    assert run_until(x, lambda y: y.state == "WAIT_OPERATOR")
    assert x.prompt["slot"] == 3 and x.prompt["options"] == ["empty", "retry"]
    x.command("confirm", choice="empty")
    assert run_until(x, lambda y: y.state == "CASSETTE_DONE")
    snap = x.snapshot()
    assert snap["cassette"]["mismatch"] is True and snap["counts"]["done"] == 3
    assert [sv["status"] for sv in snap["slots"]][:4] == ["done", "done", "empty", "done"]
    print("  OK empty slot below ToF top -> operator confirm, mismatch flagged")

    x = ready_sim([True, True] + [False] * 8)
    x.command("cassette_ready")
    x.command("fault", name="s4_false_none")
    x.command("start")
    assert run_until(x, lambda y: y.state == "WAIT_OPERATOR")
    x.command("confirm", choice="retry")
    x.command("fault", name="next_robot_delay")
    assert run_until(x, lambda y: y.state == "WAIT_S5")
    x.command("fault", name="next_robot_delay", on=False)
    assert run_until(x, lambda y: y.state == "CASSETTE_DONE")
    assert x.alarm is None and x.snapshot()["counts"]["done"] == 2
    print("  OK S4 false-none -> retry recovers; next-robot delay -> WAIT_S5")

    x = ready_sim([True] + [False] * 9)
    x.command("cassette_ready")
    x.command("fault", name="s3_fail")
    x.command("start")
    assert run_until(x, lambda y: y.state == "ALARM")
    assert x.alarm["code"] == "S3_NOT_CONFIRMED" and x.alarm["step"] == "4", x.alarm
    x.command("reset")
    assert x.state == "PAUSED"
    x.command("fault", name="s3_fail", on=False)
    x.command("resume")
    assert run_until(x, lambda y: y.state == "CASSETTE_DONE")
    print("  OK S3 fault -> ALARM at step 4 -> reset/resume recovers")

    x = ready_sim([True, True] + [False] * 8)
    x.command("cassette_ready")
    x.command("start")
    assert run_until(x, lambda y: y.snapshot()["step"] and y.snapshot()["step"]["key"] == "10")
    x.command("estop")
    assert x.state == "ESTOP" and not x.homed["z"]
    x.command("reset")
    x.command("home")
    assert run_until(x, lambda y: y.state != "HOMING", dt=0.02) and x.state == "READY"
    try:
        x.command("start")
        raise AssertionError("must require C90 confirm")
    except CommandError as e:
        assert e.code == "need_c90_confirm"
    x.command("confirm", choice="c90_clear")          # 확인만 하고 치우지 않음(운전자 실수)
    x.command("start")
    assert run_until(x, lambda y: y.state == "ALARM")
    assert x.alarm["code"] == "COLLISION" and x.alarm["step"] == "7", x.alarm
    x.command("reset")
    x.command("remove_wafer", loc="C90")
    try:
        x.command("home")
        x.command("start")
        raise AssertionError("blade wafer must block start")
    except CommandError as e:
        assert e.code in ("blade_wafer", "bad_state"), e.code
    run_until(x, lambda y: y.state != "HOMING", dt=0.02)
    x.command("remove_wafer", loc="blade")          # 충돌 때 들고 있던 슬롯 2 웨이퍼
    x.command("home")
    assert run_until(x, lambda y: y.state != "HOMING", dt=0.02)
    try:
        x.command("start")                           # 두 장 모두 수동 제거 -> 남은 슬롯 없음
        raise AssertionError("nothing left")
    except CommandError as e:
        assert e.code == "nothing", e.code
    print("  OK E-STOP at step 10 -> C90 confirm required; confirm without clearing -> collision at 7")

    x = ready_sim([True] + [False] * 9)
    x.command("cassette_ready")
    x.command("fault", name="belt_skip")
    x.command("start")
    assert run_until(x, lambda y: y.state == "ALARM")
    assert x.alarm["code"] == "S1_MISMATCH" and x.alarm["step"] == "21"
    print("  OK belt skip -> S1 mismatch at step 21")

    x = ready_sim()
    x.command("cassette_ready")
    x.command("debug_hold", step="6", frac=0.5)
    x.command("start")
    assert run_until(x, lambda y: y.state == "PAUSED")
    assert abs(x.theta - 45.0) < 0.01 and x.snapshot()["step"]["key"] == "6", x.theta
    print("  OK debug_hold pauses mid-rotation (theta 45)")

    x = ready_sim()
    x.command("fault", name="sim_exception")
    try:
        x.tick(0.02)
        raise AssertionError("expected RuntimeError")
    except RuntimeError:
        pass
    print("  OK injected sim exception raises from tick")
    print("OK (all passed)")


if __name__ == "__main__":
    main()
