"""tests/test_wafer_sync.py — '화면 = 장비 상태' 동기화 보증(가짜 장비 + PC 수신부, 가짜 시계, 하드웨어 불필요).

가짜 셀 Mega 가 ST 를 보낼 때마다 그 순간의 장비 스냅샷을 ST 순번(q)으로 기록하고(trace),
PC 쪽 LiveSource 가 만든 화면 스냅샷을 '받은 ST 의 q' 장비 스냅샷과 매 단계 비교한다.
  비교: state · pause_reason · 단계(키 · 동작 · 제목) · 슬롯 · 웨이퍼 번호 · 원점 · pose(프로토콜 반올림 허용) · Sharp ·
        센서 12 개 · ① 코드 · ③ 개수 · 포크/크래들/UV 받침 · 판정 · UV 램프/남은 초 · 대차(상태 · 받침 · 정렬, 도킹 중 자세) ·
        카운트 · 알람 코드/문구 · 선회 허용 · 제목 한 줄(headline)
  시나리오: 연결 -> 원점 -> 시작 -> 정상 · 불량 · 빈 슬롯 -> UV 중 하트비트 끊김 -> 재개 -> 판정 없음 알람 -> 해제 -> 재개
           -> 카세트 끝 -> 대차로 -> 대차 이동
결과: 불일치 0 이어야 한다. (실제 Mega 펌웨어가 같은 값을 보내는지는 펌웨어가 생긴 뒤 다시 확인 [불명])

실행: py -3.13 tests/test_wafer_sync.py
"""
import sys
import pathlib

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))


class Clock:
    def __init__(self):
        self.t = 5000.0

    def __call__(self):
        return self.t


def near(a, b, tol):
    if a is None or b is None:
        return a is b
    return abs(a - b) <= tol


def compare(dev, pc):
    """장비 스냅샷 dev 와 화면 스냅샷 pc 의 차이 목록."""
    out = []

    def eq(name, a, b):
        if a != b:
            out.append(f"{name}: device={a!r} screen={b!r}")

    def nr(name, a, b, tol):
        if not near(a, b, tol):
            out.append(f"{name}: device={a!r} screen={b!r}")

    for k in ("state", "pause_reason", "slot", "ng_slot", "wafer_no", "homed", "stage", "th_low", "headline"):
        eq(k, dev[k], pc[k])
    ds, ps = dev["step"], pc["step"]
    eq("step", ds and (ds["key"], ds["action"], ds["title"], ds["group"]), ps and (ps["key"], ps["action"], ps["title"], ps["group"]))
    eq("homed_axes", dev["homed_axes"], pc["homed_axes"])
    nr("theta", dev["pose"]["theta"], pc["pose"]["theta"], 0.006)
    nr("zc", dev["pose"]["zc"], pc["pose"]["zc"], 0.06)
    nr("rx", dev["pose"]["rx"], pc["pose"]["rx"], 0.06)
    for k in ("zd", "rd", "zc_s", "rx_s"):
        nr("meas." + k, dev["meas"][k], pc["meas"][k], 0.11)
    eq("sensors", dev["sensors"], pc["sensors"])
    eq("docks.input", (dev["docks"]["input"]["present"], [x["code"] for x in dev["docks"]["input"]["slots"]]),
       (pc["docks"]["input"]["present"], [x["code"] for x in pc["docks"]["input"]["slots"]]))
    eq("docks.ng", (dev["docks"]["ng"]["present"], dev["docks"]["ng"]["count"], [x["code"] for x in dev["docks"]["ng"]["slots"]]),
       (pc["docks"]["ng"]["present"], pc["docks"]["ng"]["count"], [x["code"] for x in pc["docks"]["ng"]["slots"]]))
    for k in ("fork", "inspect", "uvh"):
        eq(k, dev[k], pc[k])
    eq("uv.lamp", (dev["uv"]["lamp"], dev["uv"]["a_uv"]), (pc["uv"]["lamp"], pc["uv"]["a_uv"]))
    nr("uv.left_s", dev["uv"]["left_s"], pc["uv"]["left_s"], 0.11)
    dc, pcart = dev["cart"], pc["cart"]
    eq("cart", (dc["state"], dc["seats"], dc["aligned"], dc["departed"], dc["caption"]),
       (pcart["state"], pcart["seats"], pcart["aligned"], pcart["departed"], pcart["caption"]))
    if dc["state"] == "docked":
        eq("cart.pose", (dc["rot"], dc["lat"], dc["dist"]), (pcart["rot"], pcart["lat"], pcart["dist"]))
    dj, pj = dev["judgment"] or {}, pc["judgment"] or {}
    eq("judgment", (dj.get("no"), dj.get("result")), (pj.get("no"), pj.get("result")))
    for k in ("inspected", "ok", "ng", "empty", "ng_dock", "carts"):
        eq("counts." + k, dev["counts"][k], pc["counts"][k])
    nr("counts.cycle_last", dev["counts"]["cycle_last"], pc["counts"]["cycle_last"], 0.011)
    da, pa = dev["alarm"], pc["alarm"]
    eq("alarm", da and (da["code"], da["msg"], da["fatal"]), pa and (pa["code"], pa["msg"], pa["fatal"]))
    eq("interlock", dev["interlock"]["rotate_ok"], pc["interlock"]["rotate_ok"])
    return out


def main():
    from wafer_cell.fake_mega import FakeMega
    from wafer_cell import live as L

    print("test_wafer_sync:")
    clk = Clock()
    fake = FakeMega(seed=3, speed=5.0, clock=clk)
    tr = L.FakeTransport(fake)
    src = L.LiveSource(tr, clock=clk)
    state = {"answer": True, "verdict": "OK", "hb": True, "frames": 0, "bad": [], "seen": set()}

    def step(n=1):
        for _ in range(n):
            clk.t += 0.05
            if not state["hb"]:
                src.last_hb = clk.t
            src.tick()
            for req in src.take_capture_requests():
                if state["answer"]:
                    src.submit_judgment(req["id"], state["verdict"], 0.0, 0.0)
            if src.st is None or fake.trace is None:
                continue
            dev = fake.trace.get(src.st["q"])
            if dev is None:
                continue
            pc = src.snapshot()
            state["frames"] += 1
            state["seen"].add(pc["state"])
            if pc["step"]:
                state["seen"].add(pc["step"]["key"])
            for d in compare(dev, pc):
                if len(state["bad"]) < 30:
                    state["bad"].append(f"q={src.st['q']} {pc['state']} {pc['step'] and pc['step']['key']}: {d}")

    def until(cond, n=60000):
        for _ in range(n):
            if cond(src.snapshot()):
                return True
            step()
        return cond(src.snapshot())

    def cmd(c):
        kind, cid = src.command(c)
        step(2)
        r = src.wait_ack(cid, 0.01)
        assert r and r["ok"], (c, r)

    src.tick()                                  # 열기 = 장비 리셋
    fake.trace = {}
    cid = fake.sim.cart["seats"]["FL"]
    for w in fake.sim.wafers:                   # 첫 카세트 슬롯 3 을 비움(빈 슬롯 시나리오)
        if w["cass"] == cid and w["src_slot"] == 3:
            w["loc"] = "removed"
    assert until(lambda s: src.status == "OK")
    step(10)
    cmd("home")
    assert until(lambda s: s["state"] == "READY")
    cmd("start")
    assert until(lambda s: s["counts"]["ok"] >= 1)
    state["verdict"] = "NG"
    assert until(lambda s: s["counts"]["ng"] >= 1)
    state["verdict"] = "OK"
    assert until(lambda s: s["counts"]["empty"] >= 1)
    assert until(lambda s: s["step"] and s["step"]["key"] == "U6" and s["uv"]["lamp"])
    state["hb"] = False
    assert until(lambda s: s["state"] == "PAUSED" and s["pause_reason"] == "LINK", n=400)
    state["hb"] = True
    step(20)
    cmd("resume")
    state["answer"] = False
    assert until(lambda s: s["state"] == "ALARM")
    assert src.snapshot()["alarm"]["code"] == "JUDGE_TIMEOUT"
    state["answer"] = True
    cmd("reset")
    cmd("resume")
    assert until(lambda s: s["state"] == "WAIT_CART" and s["cart"]["state"] == "moving", n=200000)
    assert until(lambda s: s["cart"]["state"] == "docked" and s["state"] == "RUNNING", n=200000)
    step(100)
    for x in ("IDLE", "HOMING", "READY", "RUNNING", "PAUSED", "ALARM", "WAIT_CART", "C1", "P3", "E1", "I3", "N2",
              "U6", "R4", "L5"):
        assert x in state["seen"], (x, sorted(state["seen"]))
    assert state["frames"] > 2000, state["frames"]
    assert not state["bad"], "\n".join(state["bad"])
    print(f"  OK {state['frames']} ST frames compared (device snapshot vs screen snapshot): 0 mismatches")
    print("  OK covered: home, OK -> UV -> original slot, NG -> NG cassette, empty slot, HB lost during UV, "
          "JUDGE_TIMEOUT -> reset -> resume, cassette back to cart, cart moving to next seat")
    print("OK (all passed)")


if __name__ == "__main__":
    main()
