"""tests/test_wafer_sim.py — R9 시뮬레이터 단위테스트(하드웨어 불필요, 스레드 없음).

  - 비디오 흐름 재현: 웨이퍼 1 OK 슬롯5, 2 NG 슬롯5, 3·4·5 OK 4·3·2, 투입 교체, 6 OK 1 -> OK 가득 -> 대차 호출 -> 앞 왼 받침
    이어서 4 번째 카세트까지 -> 대차 출발, 검사한 웨이퍼 21 (비디오 마지막 화면과 같음)
  - dt 0.02 와 0.2 의 첫 사이클 차이 0.1s 이내(잔여 dt 이월)
  - 작업자 자동 OFF: 투입 비면 WAIT_INPUT 유지 -> operator 버튼으로 재개
  - NG 가득 -> WAIT_NG -> 빈 NG 카세트 -> 계속
  - 빈 투입 슬롯 -> WAIT_OPERATOR -> '빈 슬롯' 확정 -> 다음 슬롯, S4 오감지 -> '재시도' 로 복구
  - S3 고장 -> ALARM -> reset/해제/resume 로 복구, 벨트 이빨 튐 -> 0° 도착 시 S1 불일치
  - 검사 중 E-STOP -> 재시작 시 검사 크래들 확인 요구, debug_hold, 시뮬 예외는 tick 에서 raise

실행: py -3.13 tests/test_wafer_sim.py
"""
import sys
import pathlib

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))


def run_until(sim, cond, dt=0.1, limit=400000):
    for _ in range(limit):
        if cond(sim):
            return True
        sim.tick(dt)
    return cond(sim)


def ready_sim(seed=1, ng_rate=0.0):
    from wafer_cell.sim import CellSim
    s = CellSim(seed=seed)
    s.ng_rate = ng_rate
    s.command("home")
    assert run_until(s, lambda x: x.state != "HOMING", dt=0.02)
    assert s.state == "READY", (s.state, s.alarm)
    assert abs(s.theta - 120) < 1e-6 and abs(s.zc - 105) < 1e-6 and s.rx == 0
    return s


def where(s, no):
    w = next(w for w in s.wafers if w["no"] == no)
    if w["loc"] != "cass":
        return w["loc"], w["slot"], w["judgment"]
    c = s.cassettes[w["cass"]]
    return ("cart" if c["where"].startswith("cart:") else c["kind"]), w["slot"], w["judgment"]


def main():
    from wafer_cell.sim import CellSim, CommandError

    print("test_wafer_sim:")
    s = ready_sim()
    s.command("start")
    assert run_until(s, lambda x: x.inspected >= 1)
    s.command("next_ng", on=True)                                   # 비디오: 웨이퍼 2 NG
    assert run_until(s, lambda x: x.cart["seats"]["FL"] is not None and x.state == "WAIT_OK"), (s.state, s.alarm)
    got = [where(s, n) for n in range(1, 7)]
    assert got == [("cart", 5, "OK"), ("ng", 5, "NG"), ("cart", 4, "OK"), ("cart", 3, "OK"),
                   ("cart", 2, "OK"), ("cart", 1, "OK")], got
    cid = s.cart["seats"]["FL"]
    assert sorted(w["no"] for w in s.wafers if w["loc"] == "cass" and w["cass"] == cid) == [1, 3, 4, 5, 6]
    assert s.dock["ok"] is None and s.dock_count("ng") == 1 and s.inspected == 6
    codes = [e["code"] for e in s.events]
    assert codes.index("WAIT_INPUT") < codes.index("CART_CALL") < codes.index("CASS_PICK") < codes.index("CASS_PLACE")
    print("  OK video flow: wafers 1..6 -> OK 5,4,3,2,1 / NG 5, input swap, cart called, cassette on front-left seat")

    assert run_until(s, lambda x: x.cart_departed >= 1, limit=2000000), (s.state, s.alarm)
    assert s.inspected == 21 and s.dock_count("ng") == 1 and s.alarm is None
    caps = [e["msg"] for e in s.events if e["code"] == "CART"]
    assert any("180°" in c for c in caps) and any("출발" in c for c in caps)
    print("  OK 4 cassettes loaded (front L/R, turn 180, rear L/R) -> cart leaves, 21 wafers inspected (video end)")

    cyc = []
    for dt in (0.02, 0.2):
        x = ready_sim()
        x.command("start")
        assert run_until(x, lambda y: len(y.cycles) >= 1, dt=dt)
        cyc.append(x.cycles[0])
    assert abs(cyc[0] - cyc[1]) < 0.1, cyc
    assert 60 < cyc[0] < 90, cyc
    print(f"  OK first cycle {cyc[0]:.1f}s, dt 0.02 vs 0.2 differ {abs(cyc[0] - cyc[1]):.3f}s")

    x = ready_sim()
    x.command("auto_operator", on=False)
    x.command("start")                       # 전부 OK -> 웨이퍼 5 에서 OK 가득 -> 대차 -> 빈 OK 도크
    assert run_until(x, lambda y: y.state == "WAIT_OK")
    t0 = x.t
    run_until(x, lambda y: y.t > t0 + 60)
    assert x.state == "WAIT_OK" and x.op_pending == {"ok": None}
    x.command("operator", task="ok")
    try:
        x.command("operator", task="ok")
        raise AssertionError("OK dock is occupied")
    except CommandError as e:
        assert e.code == "bad_state"
    assert run_until(x, lambda y: y.state == "WAIT_INPUT")
    t0 = x.t
    run_until(x, lambda y: y.t > t0 + 60)
    assert x.state == "WAIT_INPUT" and x.op_pending == {"input": None}
    x.command("operator", task="input")
    assert run_until(x, lambda y: y.state == "RUNNING" and y.slot == 1)
    print("  OK auto operator off: WAIT_OK / WAIT_INPUT hold until the operator buttons")

    x = ready_sim(ng_rate=1.0)
    x.command("auto_operator", on=False)
    x.command("start")
    assert run_until(x, lambda y: y.state == "WAIT_NG"), x.state
    assert x.dock_count("ng") == 5 and x.inspected == 5
    x.command("operator", task="ng")
    assert run_until(x, lambda y: y.state in ("WAIT_INPUT", "RUNNING"))
    assert sum(1 for w in x.wafers if w["loc"] == "ng_out") == 5
    print("  OK NG full -> WAIT_NG -> empty NG cassette -> continue")

    x = CellSim()
    x.command("load_input", slots=[True, False, True, True, True])
    x.command("home")
    run_until(x, lambda y: y.state == "READY", dt=0.02)
    x.command("start")
    assert run_until(x, lambda y: y.state == "WAIT_OPERATOR")
    assert x.prompt["slot"] == 2
    x.command("confirm", choice="empty")
    assert run_until(x, lambda y: y.slot == 3)
    assert 2 in x.input_empty
    x.command("fault", name="s4_false_none")
    assert run_until(x, lambda y: y.state == "WAIT_OPERATOR")
    x.command("confirm", choice="retry")
    assert run_until(x, lambda y: y.inspected >= 3)
    assert x.alarm is None
    print("  OK empty input slot -> operator 'empty', S4 false-none -> 'retry' recovers")

    x = ready_sim()
    x.command("fault", name="s3_fail")
    x.command("start")
    assert run_until(x, lambda y: y.state == "ALARM")
    assert x.alarm["code"] == "S3_NOT_CONFIRMED", x.alarm
    x.command("reset")
    assert x.state == "PAUSED"
    x.command("fault", name="s3_fail", on=False)
    x.command("resume")
    assert run_until(x, lambda y: y.inspected >= 1) and x.alarm is None
    print("  OK S3 fault -> ALARM -> reset/clear/resume recovers")

    x = ready_sim()
    x.command("start")
    assert run_until(x, lambda y: y.inspected >= 1)
    x.command("fault", name="belt_skip")
    assert run_until(x, lambda y: y.state == "ALARM")
    assert x.alarm["code"] == "S1_MISMATCH" and x.alarm["step"] == "8", x.alarm
    print("  OK belt skip -> S1 mismatch on arrival at OK 0 deg")

    x = ready_sim()
    x.command("start")
    assert run_until(x, lambda y: y._wafer_by("inspect") is not None)
    x.command("estop")
    assert x.state == "ESTOP" and not x.homed["z"]
    x.command("reset")
    x.command("home")
    assert run_until(x, lambda y: y.state != "HOMING", dt=0.02) and x.state == "READY", (x.state, x.alarm)
    try:
        x.command("start")
        raise AssertionError("must require inspection cradle confirm")
    except CommandError as e:
        assert e.code == "need_insp_confirm"
    x.command("remove_wafer", loc="inspect")
    x.command("confirm", choice="insp_clear")
    x.command("start")
    assert run_until(x, lambda y: y.inspected >= 2) and x.alarm is None
    print("  OK E-STOP during inspection -> inspection cradle confirm required on restart")

    x = ready_sim()
    x.command("start")
    assert run_until(x, lambda y: y.cart["seats"]["FL"] is None and y.state == "WAIT_CART")
    x.command("debug_hold", step="T8", frac=0.5)
    assert run_until(x, lambda y: y.state == "PAUSED")
    assert -60 < x.theta < -30 and x.occ().fork_cassette, x.theta
    print(f"  OK debug_hold pauses mid T8 (theta {x.theta:.1f}, cassette on fork)")

    x = ready_sim()
    x.command("debug_hold", step="6", frac=0.5)                     # 촬영(움직임 없음)도 멈춰야 함
    x.command("start")
    assert run_until(x, lambda y: y.state == "PAUSED", dt=0.02)
    assert x.program[x.pc].key == "6" and x._wafer_by("inspect") is not None and x.cur["kind"] == "capture"
    print("  OK debug_hold also pauses mid capture (step 6, wafer on inspection cradle)")

    x = ready_sim()
    x.command("fault", name="sim_exception")
    try:
        x.tick(0.02)
        raise AssertionError("expected exception")
    except RuntimeError:
        pass
    snap = x.snapshot()
    for k in ("docks", "cart", "fork", "inspect", "operator", "headline", "program", "counts"):
        assert k in snap, k
    print("  OK injected sim exception raises from tick, snapshot fields")
    print("OK (all passed)")


if __name__ == "__main__":
    main()
