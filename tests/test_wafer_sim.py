"""tests/test_wafer_sim.py — R22 시뮬레이터 단위테스트(하드웨어 불필요, 스레드 없음).

  - 원점: 자리 사이(θ150 · Z140 · R30)에서 R 후진 -> Z105 -> θ 를 - 쪽으로 찾아 120 -> READY,
          θ10 에서는 θ0 을 찾고 대기 120 으로, θ180 센서가 죽었고 268° 에서 시작하면 95° 안에 없음 -> HOME_NO_STATION
  - 대차 1 대(카세트 4): 대차 -> ① -> 슬롯마다 검사 · UV · 원래 슬롯 -> 대차 -> 다음 받침(옆걸음 · 180° 회전) -> 출발,
    웨이퍼 20 장이 모두 원래 슬롯으로, 알람 0, 새 대차 호출
  - 불량: '다음 웨이퍼 NG' -> ③ 위 슬롯부터(5, 4), ① 슬롯 코드 N / 빈 슬롯 -> E1(알람 없이 다음 슬롯)
  - ③ 가득 -> WAIT_NG -> 작업자(자동/수동) 교체 -> TOF-NG 없음->있음 으로 포인터 0 -> 계속
  - 고장 주입마다 해당 알람: θ 센서 2 개 LOW · θ 센서 죽음 · 벨트 이빨 튐 · Z/R Sharp · S4 오감지 · TOF-IN · UV LED · 과열 ·
    보정 창 밖 · 판정 없음 · 카메라
  - S4 가 P3 에서 거짓 '없음' -> 빈 슬롯으로 믿고 웨이퍼를 든 채 다음 슬롯으로 -> 기하 판정이 충돌을 잡음(위험 시연)
  - 알람 해제 -> 재개로 이어감, 치명 알람 뒤 셀 안 웨이퍼 -> start 거부(cell_not_clear) -> 치우면 시작
  - UV 중 E-STOP -> 램프 끔, PC 하트비트 끊김(link_lost) -> UV 즉시 끔 + 축 동작 끝낸 뒤 PAUSED(LINK) -> 재개하면 처음부터 조사
  - 외부 판정(external): 촬영 요청 -> 다른 id 무시 -> 맞는 id 로 진행, 5 s 무응답 -> JUDGE_TIMEOUT, δr -4 -> CORR_RANGE
  - dt 0.02 / 0.2 사이클 차이 0.1 s 이내, debug_hold, 시뮬 예외는 tick 에서 raise

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


def ready_sim(seed=1, ng_rate=0.0, **kw):
    from wafer_cell.sim import CellSim
    s = CellSim(seed=seed, **kw)
    s.ng_rate = ng_rate
    s.command("home")
    assert run_until(s, lambda x: x.state != "HOMING", dt=0.05)
    assert s.state == "READY", (s.state, s.alarm)
    assert abs(s.theta - 120) < 1e-6 and abs(s.zc - 105) < 1e-6 and s.rx <= 2
    return s


def started(**kw):
    s = ready_sim(**kw)
    s.command("start")
    return s


def codes(s):
    return [e["code"] for e in s.events]


def alarm_after(fault, cond_before=None, ng=False, judge_mode="internal", limit=200000):
    s = started(judge_mode=judge_mode)
    if cond_before:
        assert run_until(s, cond_before), (fault, s.state, s.alarm)
    if ng:
        s.ng_next = True
    s.command("fault", name=fault, on=True)
    assert run_until(s, lambda x: x.state == "ALARM", limit=limit), (fault, s.state)
    return s


def main():
    from wafer_cell.sim import CellSim, CommandError
    from wafer_cell import cell_config as C

    print("test_wafer_sim:")
    s = CellSim()
    assert s.state == "IDLE" and not any(s.homed.values()) and s.theta == 150
    s.command("home")
    assert run_until(s, lambda x: x.state != "HOMING", dt=0.05) and s.state == "READY"
    assert [e["d"] for e in s.events if e["code"] == "TH_FOUND"] == ["inspect"]
    s = CellSim()
    s.theta = 10.0
    s.command("home")
    assert run_until(s, lambda x: x.state != "HOMING", dt=0.05) and s.state == "READY" and s.theta == 120
    assert [e["d"] for e in s.events if e["code"] == "TH_FOUND"] == ["uv"]
    s = CellSim()
    s.theta = 268.0
    s.command("fault", name="th_dead", on=True)
    s.command("home")
    assert run_until(s, lambda x: x.state != "HOMING", dt=0.05)
    assert s.alarm and s.alarm["code"] == "HOME_NO_STATION", s.alarm
    s = CellSim()
    s.command("fault", name="th_double", on=True)
    s.command("home")
    assert run_until(s, lambda x: x.state != "HOMING", dt=0.05) and s.alarm["code"] == "TH_SENSOR_FAULT", s.alarm
    print("  OK homing: R -> Z105 -> theta search (150->120, 10->0->120), no station in 95 deg, two LOW = fault")

    s = started()
    assert run_until(s, lambda x: x.carts_done >= 1 or x.state == "ALARM", limit=2000000), (s.state, s.alarm)
    assert s.alarm is None and s.counts == {"inspected": 20, "ok": 20, "ng": 0, "empty": 0}, (s.counts, s.alarm)
    done = [w for w in s.wafers if w["t_done"] is not None]
    assert len(done) == 20 and all(w["dest"] == "K" and w["dest_slot"] == w["src_slot"] for w in done)
    assert all(w["uv_s"] >= C.UV_S - 1e-6 and w["judgment"] == "OK" for w in done)
    cart_codes = [e["d"] for e in s.events if e["code"] == "CART"]
    assert "ROTATE" in cart_codes and "LEAVE" in cart_codes and "LEFT" in cart_codes, cart_codes
    assert any(c.startswith("LATERAL") for c in cart_codes)
    first = {}
    for w in sorted(done, key=lambda w: w["t_done"]):
        first.setdefault(s.cassettes[w["cass"]]["seat"], w["t_done"])
    assert sorted(first, key=first.get) == ["FL", "FR", "RL", "RR"], first
    assert run_until(s, lambda x: x.cart["state"] == "docked" and x.state == "RUNNING", limit=200000)
    print(f"  OK one cart (4 cassettes): 20 wafers OK -> UV {C.UV_S:g}s -> original slot, seats FL FR RL RR, cart leaves, new cart docks")

    s = started()
    s.next_input_map = [True, False, True, True, True]
    s.cart = s._new_cart("docked", aligned="FL")
    assert run_until(s, lambda x: x.counts["inspected"] >= 1)
    s.ng_next = True
    assert run_until(s, lambda x: x.counts["ng"] >= 1)
    s.ng_next = True
    assert run_until(s, lambda x: x.counts["ng"] >= 2)
    assert run_until(s, lambda x: x.dock["input"] is None and x.seat_stage("FL") == "D" or x.state == "ALARM")
    assert s.alarm is None
    fin = {w["src_slot"]: (w["dest"], w["dest_slot"]) for w in s.wafers if w["t_done"] is not None and w["src_cass"] == s.cart["seats"]["FL"]}
    assert fin == {1: ("K", 1), 3: ("N", 5), 4: ("N", 4), 5: ("K", 5)}, fin
    assert s.counts["empty"] == 1 and "EMPTY_SLOT" in codes(s) and s.ng_ptr == 2
    print("  OK NG -> NG cassette top-first (5, 4), OK back to own slot, empty slot 2 skipped without alarm")

    s = started(ng_rate=1.0)
    assert run_until(s, lambda x: x.state == "WAIT_NG", limit=800000), (s.state, s.alarm)
    assert s.ng_ptr == 5 and "NG_FULL" in codes(s)
    assert run_until(s, lambda x: x.state == "RUNNING" and x.ng_ptr == 0)
    assert codes(s).index("NG_OUT") < codes(s).index("NG_SWAP")
    assert run_until(s, lambda x: x.counts["ng"] >= 6)
    s.command("auto_operator", on=False)
    assert run_until(s, lambda x: x.state == "WAIT_NG", limit=800000)
    for _ in range(300):
        s.tick(0.1)
    assert s.state == "WAIT_NG"
    s.command("operator", task="ng")
    assert run_until(s, lambda x: x.state == "RUNNING" and x.ng_ptr == 0)
    s.speed = 50.0                                                  # 빼고 놓기가 한 tick 안에 끝나도 포인터가 0 으로
    assert run_until(s, lambda x: x.state == "WAIT_NG", limit=800000)
    s.command("operator", task="ng")
    s.tick(0.2)
    assert s.ng_ptr == 0 and s.state == "RUNNING", (s.ng_ptr, s.state)
    print("  OK NG full -> WAIT_NG -> operator swap (auto / button) -> TOF-NG absent->present resets pointer")

    cases = [
        ("belt_skip", "TH_MISMATCH", None),
        ("th_dead", "TH_MISMATCH", None),
        ("th_double", "TH_SENSOR_FAULT", None),
        ("z_sharp_drift", "Z_VERIFY", None),
        ("r_sharp_fail", None, None),
        ("tof_in_none", "CASS_MISSING", None),
        ("s4_false_present", "S4_REMAINS", lambda x: x.slot is not None),
        ("uv_led_fail", "UV_FAULT", None),
        ("uv_overtemp", "UV_LOST", None),
        ("corr_out", "CORR_RANGE", None),
        ("judge_drop", "JUDGE_TIMEOUT", None),
        ("camera_fail", "CAMERA", None),
    ]
    for fault, want, before in cases:
        s = alarm_after(fault, before)
        got = s.alarm["code"]
        if want is None:
            assert got in ("R_NOT_RETRACTED", "R_VERIFY", "INTERLOCK"), (fault, got)
        else:
            assert got == want, (fault, got, s.alarm)
        assert s.alarm["msg"] and s.alarm["code"] in [e["code"] for e in s.events if e["level"] == "alarm"]
    print("  OK fault injection -> alarm: " + ", ".join(f"{f}->{w or 'R_*'}" for f, w, _ in cases))

    s = alarm_after("uv_overtemp")
    assert not s.uv["lamp"] and not s.sensors()["A_UV"] and "UV_ABORT" in codes(s)
    s.command("fault", name="uv_overtemp", on=False)
    s.command("reset")
    assert s.state == "PAUSED"
    s.command("resume")
    assert run_until(s, lambda x: x.counts["ok"] >= 1 or x.state == "ALARM") and s.alarm is None
    assert codes(s).count("UV_ON") >= 2
    print("  OK non-fatal alarm (UV lost) -> reset -> PAUSED -> resume -> UV again from the start -> wafer done")

    s = started()
    assert run_until(s, lambda x: x._step_key() == "P3" and x.slot == 1)
    s.command("fault", name="s4_false_none", on=True)
    assert run_until(s, lambda x: x.state == "ALARM", limit=200000)
    assert "EMPTY_SLOT" in codes(s) and s.alarm["code"] == "COLLISION" and s._wafer_at("blade") is not None, s.alarm
    print(f"  OK S4 false 'none' at P3 -> treated as empty slot -> wafer still on fork -> COLLISION ({s.alarm['d']})")
    s.command("reset")
    assert s.state == "IDLE"
    s.command("home")
    assert run_until(s, lambda x: x.state != "HOMING", dt=0.05) and s.state == "READY"
    try:
        s.command("start")
        raise AssertionError("start with wafer on fork")
    except CommandError as e:
        assert e.code == "cell_not_clear"
    s.command("remove_wafer", loc="blade")
    s.command("start")
    assert run_until(s, lambda x: x.counts["ok"] >= 1 or x.state == "ALARM") and s.alarm is None
    print("  OK fatal alarm -> IDLE -> home -> start refused (cell_not_clear) -> remove wafer -> runs")

    s = started()
    s.command("debug_hold", step="U6", frac=0.5)
    assert run_until(s, lambda x: x.state == "PAUSED")
    assert s.pause_reason == "HOLD" and s.cur and s.cur["kind"] == "uv"
    assert s.uv["lamp"] and s.sensors()["A_UV"]
    s.command("resume")                                            # 단계 고정 해제 = 그 자리에서 이어감(UV 그대로)
    assert s.uv["lamp"] and s.cur and s.cur["kind"] == "uv" and s.snapshot()["uv"]["left_s"] is not None
    s.command("estop")
    assert s.state == "ESTOP" and not s.uv["lamp"] and not s.sensors()["D9"] and not any(s.homed.values())
    s.command("reset")
    assert s.state == "IDLE" and s.sensors()["D9"]
    print("  OK E-STOP during UV -> lamp off, D9 open, homing lost; reset -> IDLE")

    s = started()
    assert run_until(s, lambda x: x._step_key() == "U6" and x.uv["lamp"])
    s.link_lost()
    assert not s.uv["lamp"] and s.state in ("RUNNING", "PAUSED")
    s.tick(0.05)
    assert s.state == "PAUSED" and s.pause_reason == "LINK", (s.state, s.pause_reason)
    s.command("resume")
    assert run_until(s, lambda x: x._step_key() == "U7" or x.state == "ALARM") and s.alarm is None
    assert codes(s).count("UV_ON") == 2 and codes(s).count("UV_OFF") == 1
    s = started()
    assert run_until(s, lambda x: x.cur and x.cur["kind"] == "z" and abs(x.cur["to"] - x.zc) > 20)
    to = s.cur["to"]
    s.link_lost()
    assert run_until(s, lambda x: x.state == "PAUSED", dt=0.05)
    assert abs(s.zc - to) < 1e-6 and s.pause_reason == "LINK"
    print("  OK link lost: UV off at once, current axis move finishes, PAUSED(LINK); resume -> UV restarts")

    s = started(judge_mode="external")
    assert run_until(s, lambda x: bool(x.pending_captures))
    req = s.take_capture_requests()[0]
    assert req["n"] == 1 and req["w"] == 1
    assert not s.submit_judgment(req["id"] + 1, "OK")
    for _ in range(10):
        s.tick(0.1)
    assert s._step_key() == "I3"
    assert s.submit_judgment(req["id"], "OK", -1.0, 0.2)
    assert run_until(s, lambda x: x._step_key() == "I4")
    assert run_until(s, lambda x: x.cur and x.cur["kind"] == "r" and x._step_key() == "I4")
    assert abs(s.cur["to"] - (C.STATION_RX["inspect"] - 1.0)) < 1e-9 and abs(s.theta - 120.2) < 1e-6
    assert run_until(s, lambda x: bool(x.pending_captures))
    s.take_capture_requests()
    t0 = s.t
    assert run_until(s, lambda x: x.state == "ALARM")
    assert s.alarm["code"] == "JUDGE_TIMEOUT" and abs((s.t - t0) - C.JUDGE_TIMEOUT_S) < 0.2
    s.command("reset")
    s.command("resume")
    assert run_until(s, lambda x: bool(x.pending_captures))
    s.submit_judgment(s.take_capture_requests()[0]["id"], "OK", -4.0, 0.0)
    assert run_until(s, lambda x: x.state == "ALARM") and s.alarm["code"] == "CORR_RANGE"
    print("  OK external judge: wrong id ignored, correction applied (RX 51-1, theta 120.2), timeout 5 s, out-of-window")

    cyc = []
    for dt in (0.02, 0.2):
        x = started()
        assert run_until(x, lambda y: len(y.cycles) >= 1, dt=dt)
        cyc.append(x.cycles[0])
    assert abs(cyc[0] - cyc[1]) < 0.1, cyc
    assert 60 < cyc[0] < 200, cyc
    print(f"  OK first wafer cycle {cyc[0]:.1f}s (dt 0.02) vs {cyc[1]:.1f}s (dt 0.2), assumed speeds")

    s = started()
    s.command("debug_hold", step="I3", frac=0.5)
    assert run_until(s, lambda x: x.state == "PAUSED") and s.pause_reason == "HOLD" and s._wafer_at("inspect")
    snap = s.snapshot()
    for k in ("pose", "meas", "sensors", "docks", "uv", "cart", "interlock", "counts", "link", "headline", "step"):
        assert k in snap, k
    assert snap["step"]["key"] == "I3" and snap["inspect"]["wafer"] and snap["docks"]["input"]["slots"][0]["code"] == "O"
    assert set(snap["sensors"]) >= set(C.TH_SENSORS) | {"S4", "TOF_IN", "TOF_NG", "A_UV", "D9", "ESTOP"}
    s.command("fault", name="sim_exception", on=True)
    try:
        s.tick(0.1)
        raise AssertionError("no exception")
    except RuntimeError:
        pass
    try:
        CellSim(source="LIVE").command("fault", name="belt_skip")
        raise AssertionError("sim-only accepted in LIVE")
    except CommandError as e:
        assert e.code == "sim_only"
    print("  OK debug_hold (capture), snapshot shape, sim exception raises, sim-only refused for LIVE source")
    print("OK (all passed)")


if __name__ == "__main__":
    main()
