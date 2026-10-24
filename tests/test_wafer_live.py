"""tests/test_wafer_live.py — 실장비(LIVE) 수신부 단위테스트(하드웨어 · pyserial 불필요, 가짜 시계).

  - live.py 를 import 해도 pyserial 을 불러오지 않음, pyserial 없으면 NO_PYSERIAL
  - 연결 상태: DOWN -> WAIT_HELLO -> (HEL) STALE -> (ST) OK -> 1 s 끊김 STALE, 프로토콜 버전 다름 = VERSION
  - 하트비트 0.5 s 마다, HEL 못 받으면 2 s 뒤 CMD hello
  - 명령: 연결 OK 가 아니면 link_down(서버 503), CMD 에 지금 상태(x) 실음, ACK ok / NAK / 시간 초과(None)
  - 재부팅 감지(HEL 부팅 id 바뀜, ms 줄어듦), ST 순번 빠짐 셈, 깨진 줄 셈
  - 스냅샷 키가 시뮬 스냅샷과 같음(같은 화면 코드가 그대로 돌게), 알람 문구는 EV 세부(d)로
  - CAP -> 촬영 요청, JDG 줄(<= 60 B), WDONE + CYCLE -> 끝난 웨이퍼 기록

실행: py -3.13 tests/test_wafer_live.py
"""
import sys
import pathlib

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))


class Clock:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t


def st_line(P, **kw):
    f = {"q": 1, "ms": 1000, "st": "IDLE", "stp": "-", "ai": 0, "n": 0, "th": 150.0, "zc": 140.0, "rx": 30.0,
         "hm": 0, "ths": "00000", "zd": 209.1, "rd": 134.0, "s4": 0, "ti": 0, "tn": 1, "d9": 1, "uv": 0, "au": 0,
         "al": "-"}
    f.update(kw)
    return P.encode("ST", f)


def sl_line(P, **kw):
    f = {"in": "-----", "ng": 0, "cs": "FFFF", "ca": "DOCKED", "cal": "FL", "fw": 0, "fc": 0, "cr": 0, "uh": 0,
         "ins": 0, "okc": 0, "ngc": 0, "emp": 0, "dep": 0}
    f.update(kw)
    return P.encode("SL", f)


def main():
    from wafer_cell import protocol as P
    from wafer_cell import live as L
    from wafer_cell.sim import CellSim, CommandError

    print("test_wafer_live:")
    assert "serial" not in sys.modules, "live.py must not import pyserial at module load"
    saved = sys.modules.get("serial")
    sys.modules["serial"] = None                      # import serial -> ImportError
    try:
        src = L.LiveSource(L.SerialTransport("COM99"))
        src.tick()
        assert src.status == "NO_PYSERIAL", src.status
    finally:
        if saved is None:
            del sys.modules["serial"]
        else:
            sys.modules["serial"] = saved
    print("  OK pyserial imported lazily; missing pyserial -> NO_PYSERIAL")

    clk = Clock()
    tr = L.QueueTransport()
    tr.fail_open = "FileNotFoundError"
    src = L.LiveSource(tr, clock=clk)
    src.tick()
    assert src.status == "DOWN"
    tr.fail_open = None
    clk.t += 0.5
    src.tick()
    assert src.status == "DOWN"                        # 1 s 마다만 다시 연다
    clk.t += 0.6
    src.tick()
    assert src.status == "WAIT_HELLO" and tr.is_open()
    try:
        src.command("home")
        raise AssertionError("command while WAIT_HELLO")
    except CommandError as e:
        assert e.code == "link_down"
    hb0 = sum(1 for x in tr.sent if x.startswith(b"$HB"))
    for _ in range(10):
        clk.t += 0.25
        src.tick()
    hbs = sum(1 for x in tr.sent if x.startswith(b"$HB")) - hb0
    assert 4 <= hbs <= 6, hbs
    assert any(x.startswith(b"$CMD") and b"c=hello" in x for x in tr.sent), "no hello request"
    tr.inbox.append(P.encode("HEL", {"p": 1, "b": "AAA1", "fw": "t"}))
    src.tick()
    assert src.status == "STALE"
    tr.inbox.append(st_line(P) + sl_line(P))
    src.tick()
    assert src.status == "OK", src.status
    clk.t += 1.2
    src.tick()
    assert src.status == "STALE"
    tr.inbox.append(st_line(P, q=2, ms=2200))
    src.tick()
    assert src.status == "OK"
    print("  OK link states DOWN -> WAIT_HELLO -> STALE -> OK -> STALE -> OK, HB every 0.5 s, hello re-request")

    tr.sent.clear()
    kind, cid = src.command("home")
    assert kind == "pending"
    line = P.parse(tr.sent[-1])
    assert line.type == "CMD" and line.fields["c"] == "home" and line.fields["x"] == "IDLE"
    assert src.wait_ack(cid, 0.01) is None                # 아직 ACK 없음 -> 시간 초과
    kind, cid = src.command("home")
    tr.inbox.append(P.encode("ACK", {"id": cid, "ok": 1, "e": "-", "st": "HOMING"}))
    src.tick()
    r = src.wait_ack(cid, 0.1)
    assert r and r["ok"] and r["st"] == "HOMING"
    kind, cid = src.command("start")
    tr.inbox.append(P.encode("ACK", {"id": cid, "ok": 0, "e": "not_homed", "st": "IDLE"}))
    src.tick()
    r = src.wait_ack(cid, 0.1)
    assert r and not r["ok"] and r["e"] == "not_homed" and "원점" in r["msg"]
    for bad in ("fault", "speed", "hello", "estop"):
        try:
            src.command(bad)
            raise AssertionError(bad)
        except CommandError as e:
            assert e.code == "sim_only"
    print("  OK commands: CMD carries expected state, ACK ok / NAK with Korean reason / timeout; only 6 LIVE commands")

    tr.inbox.append(b"garbage\n$ST,q=3*00\n" + st_line(P, q=5, ms=2300))
    src.tick()
    assert src.counters["st_gap"] == 2 and src.counters["bad_line"] >= 1
    tr.inbox.append(st_line(P, q=6, ms=100))
    src.tick()
    assert src.counters["reboot"] == 1
    tr.inbox.append(P.encode("HEL", {"p": 1, "b": "BBB2"}) + st_line(P, q=1, ms=50))
    src.tick()
    assert src.counters["reboot"] == 2 and any(e["code"] == "REBOOT" for e in src.events)
    tr.inbox.append(P.encode("HEL", {"p": 2, "b": "BBB2"}))
    src.tick()
    assert src.status == "VERSION"
    tr.inbox.append(P.encode("HEL", {"p": 1, "b": "BBB2"}) + st_line(P, q=2, ms=60))
    src.tick()
    assert src.status == "OK"
    print("  OK reboot detection (boot id / ms), ST gaps counted, garbage counted, VERSION mismatch")

    tr.inbox.append(st_line(P, q=3, ms=5000, st="RUNNING", stp="U6", ai=1, n=2, w=7, th=60.0, zc=105.0, rx=-2.0,
                            hm=7, ths="01000", zd=244.1, rd=102.0, uv=1, au=1, ul=6.5)
                    + sl_line(P, **{"in": "KOUUU"}, fw=0, uh=1, ins=7, okc=6, ju="OK", jw=7, jdr=-0.5, jdt=0.1,
                              ca="DOCKED", cal="FR", cs="DWFF"))
    src.tick()
    snap = src.snapshot()
    ref = CellSim().snapshot()
    assert set(snap) == set(ref), set(snap) ^ set(ref)
    for k in ("docks", "cart", "uv", "counts", "fork", "interlock", "sensors"):
        assert set(snap[k]) >= set(ref[k]) - {"tof"}, (k, set(ref[k]) - set(snap[k]))
    assert snap["step"]["key"] == "U6" and snap["step"]["stage"] == "uv" and "UV" in snap["headline"]
    assert snap["uv"]["lamp"] and snap["uv"]["left_s"] == 6.5 and snap["sensors"]["TH60"] and snap["homed"]
    assert snap["docks"]["input"]["slots"][1]["code"] == "O" and snap["cart"]["seats"]["FR"] == "W"
    assert snap["cart"]["lat"] == -200.0 and snap["wafer_no"] == 7 and snap["meas"]["rx_s"] == -2.0
    assert snap["interlock"]["rotate_ok"] and snap["expect"] is None and snap["link"]["status"] == "OK"
    print("  OK snapshot keys == SIM snapshot keys; step/UV/slots/cart/sharp derived from ST + SL")

    tr.inbox.append(P.encode("EV", {"q": 9, "lv": "A", "c": "UV_INTERLOCK", "d": "TH60+D9", "stp": "U6", "n": 2})
                    + st_line(P, q=4, ms=5100, st="ALARM", stp="U6", al="UV_INTERLOCK"))
    src.tick()
    a = src.snapshot()["alarm"]
    assert a["code"] == "UV_INTERLOCK" and "θ60 (UV 자리)" in a["msg"] and "D9" in a["msg"] and a["step"] == "U6"
    tr.inbox.append(P.encode("CAP", {"id": 41, "n": 3, "w": 8}))
    src.tick()
    caps = src.take_capture_requests()
    assert caps == [{"id": 41, "n": 3, "w": 8, "wafer": None, "t": 5.1}], caps
    tr.sent.clear()
    src.submit_judgment(41, "NG", -1.25, 0.333)
    j = P.parse(tr.sent[-1])
    assert j.type == "JDG" and j.fields == {"id": "41", "r": "NG", "dr": "-1.2", "dt": "0.33"} and len(tr.sent[-1]) <= 61
    tr.inbox.append(P.encode("EV", {"q": 10, "lv": "I", "c": "WDONE", "d": "N", "n": 3, "k": 5, "w": 8})
                    + P.encode("EV", {"q": 11, "lv": "I", "c": "CYCLE", "d": "88.5", "w": 8}))
    src.tick()
    fin = src.finished_wafers()
    assert len(fin) == 1 and fin[0]["no"] == 8 and fin[0]["dest"] == "N" and fin[0]["dest_slot"] == 5 \
        and fin[0]["cycle_s"] == 88.5 and fin[0]["judgment"] == "NG"
    assert any("③" in e["msg"] for e in src.events if e["code"] == "WDONE")
    print("  OK alarm text from EV detail, CAP -> request, JDG line <= 60 B, WDONE + CYCLE -> finished record")

    tr.close()
    src.tick()
    assert src.status in ("DOWN", "WAIT_HELLO")
    print("OK (all passed)")


if __name__ == "__main__":
    main()
