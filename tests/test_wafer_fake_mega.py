"""tests/test_wafer_fake_mega.py — 가짜 셀 Mega(fake_mega.py) + PC 수신부(LiveSource) 끝까지(가짜 시계, 하드웨어 불필요).

  - 연결(포트 열기) = 리셋: HEL -> IDLE · 원점 미확정 · UV 꺼짐, 스스로 움직이지 않음
  - 명령: IDLE 에서 start -> NAK bad_state, 기대 상태(x) 다름 -> NAK, 같은 id 두 번 -> 한 번만 실행, home -> ACK -> READY
  - 시작 -> 촬영 요청(CAP) -> PC 판정(JDG OK) -> UV -> ① 원래 슬롯(SL in 'K'), JDG NG -> ③ (SL ng 1, in 'N')
  - 모든 출력 줄이 SPEC 을 지키고 200 B 이하
  - UV 조사 중 하트비트 끊김 -> UV 즉시 끔, PAUSED(LINK); 하트비트가 돌아와도 스스로 재개 안 함 -> resume
  - CAP 에 답하지 않으면 5 s 뒤 JUDGE_TIMEOUT

실행: py -3.13 tests/test_wafer_fake_mega.py
"""
import sys
import pathlib

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))


class Clock:
    def __init__(self):
        self.t = 1000.0

    def __call__(self):
        return self.t


class Rig:
    """가짜 장비 + PC 수신부를 같은 가짜 시계로 돌린다. answer=True 면 CAP 에 판정을 보낸다."""

    def __init__(self, speed=20.0):
        from wafer_cell.fake_mega import FakeMega
        from wafer_cell import live as L
        from wafer_cell import protocol as P
        self.P, self.clk = P, Clock()
        self.fake = FakeMega(seed=1, speed=speed, clock=self.clk)
        self.tr = L.FakeTransport(self.fake)
        self.src = L.LiveSource(self.tr, clock=self.clk)
        self.lines, self.answer, self.verdict, self.hb = [], True, "OK", True
        orig_take = self.fake.take_output

        def take():
            out = orig_take()
            self.lines += [x for x in out.split(b"\n") if x]
            return out
        self.fake.take_output = take

    def step(self, n=1, dt=0.05):
        for _ in range(n):
            self.clk.t += dt
            if not self.hb:
                self.src.last_hb = self.clk.t              # PC 가 하트비트를 보내지 않는 척
            self.src.tick()
            for req in self.src.take_capture_requests():
                if self.answer:
                    self.src.submit_judgment(req["id"], self.verdict, 0.0, 0.0)

    def until(self, cond, n=40000, dt=0.05):
        for _ in range(n):
            if cond(self.src.snapshot()):
                return True
            self.step(1, dt)
        return cond(self.src.snapshot())

    def cmd(self, c):
        kind, cid = self.src.command(c)
        self.step(2)
        return self.src.wait_ack(cid, 0.01)


def main():
    from wafer_cell import protocol as P
    from wafer_cell.sim import CommandError

    print("test_wafer_fake_mega:")
    r = Rig()
    r.step(5)
    s = r.src.snapshot()
    assert r.src.status == "OK" and s["state"] == "IDLE" and not s["homed"] and not s["uv"]["lamp"], (r.src.status, s["state"])
    assert r.lines[0].startswith(b"$HEL") and s["link"]["fw"] == "fake-sim"
    t0 = s["pose"]
    r.step(40)
    assert r.src.snapshot()["pose"] == t0                    # 스스로 안 움직임
    print("  OK connect = reset: HEL, IDLE, not homed, UV off, no motion on its own")

    a = r.cmd("start")
    assert a and not a["ok"] and a["e"] == "bad_state", a
    kind, cid = r.src.command("home")
    r.src.st["st"] = "READY"                                  # PC 화면이 틀린 상태를 믿는다고 가정
    kind2, cid2 = r.src.command("home")
    r.step(2)
    a2 = r.src.wait_ack(cid2, 0.01)
    a1 = r.src.wait_ack(cid, 0.01)
    assert a1["ok"] and not a2["ok"] and a2["e"] == "bad_state", (a1, a2)
    homes = [e for e in r.fake.sim.events if e["code"] == "HOME"]
    r.tr.write(P.encode("CMD", {"id": cid, "c": "home"}))     # 같은 id 다시 -> 실행 안 함
    r.step(2)
    assert len([e for e in r.fake.sim.events if e["code"] == "HOME"]) == len(homes) == 1
    assert r.until(lambda x: x["state"] == "READY")
    s = r.src.snapshot()
    assert s["homed"] and s["pose"]["theta"] == 120.0 and s["sensors"]["TH120"]
    print("  OK commands: NAK bad_state, stale expected-state refused, duplicate id not re-run, home -> READY")

    a = r.cmd("start")
    assert a and a["ok"] and a["st"] == "RUNNING", a
    assert r.until(lambda x: x["counts"]["ok"] >= 1 or x["state"] == "ALARM")
    s = r.src.snapshot()
    assert s["alarm"] is None and s["docks"]["input"]["slots"][0]["code"] == "K", s["docks"]["input"]
    assert s["counts"]["inspected"] == 1 and any(e["code"] == "UV_ON" for e in r.src.events)
    r.verdict = "NG"
    assert r.until(lambda x: x["counts"]["ng"] >= 1 or x["state"] == "ALARM")
    s = r.src.snapshot()
    assert s["alarm"] is None and s["counts"]["ng_dock"] == 1 and s["docks"]["input"]["slots"][1]["code"] == "N"
    assert s["docks"]["ng"]["slots"][4]["code"] == "N"
    r.verdict = "OK"
    bad = [ln for ln in r.lines if len(ln) > P.MAX_MEGA_LINE or P.validate(P.parse(ln))]
    assert not bad, bad[:3]
    types = {P.parse(ln).type for ln in r.lines}
    assert types >= {"HEL", "ST", "SL", "EV", "CAP", "ACK"}, types
    print(f"  OK run: CAP -> JDG OK -> UV -> slot 1 'K'; JDG NG -> NG slot 5; {len(r.lines)} lines all valid <= 200 B")

    assert r.until(lambda x: x["step"] and x["step"]["key"] == "U6" and x["uv"]["lamp"])
    r.hb = False
    assert r.until(lambda x: not x["uv"]["lamp"] and x["state"] == "PAUSED", n=200)
    s = r.src.snapshot()
    assert s["pause_reason"] == "LINK" and "PC 연결" in s["headline"]
    r.hb = True
    r.step(60)
    assert r.src.snapshot()["state"] == "PAUSED"
    a = r.cmd("resume")
    assert a["ok"]
    assert r.until(lambda x: x["uv"]["lamp"])
    print("  OK heartbeat lost during UV: lamp off at once, PAUSED(LINK), stays paused when HB returns, resume")

    r.answer = False
    assert r.until(lambda x: x["state"] == "ALARM", n=80000)
    s = r.src.snapshot()
    assert s["alarm"]["code"] == "JUDGE_TIMEOUT" and "판정" in s["alarm"]["msg"]
    r.answer = True
    a = r.cmd("reset")
    assert a["ok"] and r.src.snapshot()["state"] == "PAUSED"
    a = r.cmd("resume")
    assert a["ok"]
    assert r.until(lambda x: x["step"] and x["step"]["key"] == "I4")
    print("  OK no JDG -> JUDGE_TIMEOUT -> reset -> resume -> new CAP answered")

    r.fake.frozen = True
    r.step(30)
    assert r.src.status == "STALE"
    try:
        r.src.command("pause")
        raise AssertionError("command while STALE")
    except CommandError as e:
        assert e.code == "link_down"
    print("  OK device frozen -> STALE, commands refused")
    print("OK (all passed)")


if __name__ == "__main__":
    main()
