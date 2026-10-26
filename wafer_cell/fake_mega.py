"""fake_mega.py - 가짜 셀 Mega. CellSim(판정 = 외부)을 감싸 PROTOCOL.md 줄로만 대화한다.

하드웨어 없이 LIVE 경로 전체(서버 연결 · 명령 ACK · 촬영 요청/판정 · 하트비트 끊김)를 시험한다.
  - 서버 안에서:     py -3.13 wafer_cell/server.py --source live --serial fake
  - 따로(TCP):       py -3.13 wafer_cell/fake_mega.py --tcp 8770     -> server.py --source live --serial tcp:127.0.0.1:8770
  - 따로(가상 COM):  py -3.13 wafer_cell/fake_mega.py --port COM20   (com0com 같은 가상 쌍이 있을 때)
진짜 Mega 처럼: 연결(포트 열기)마다 리셋 -> HEL -> IDLE · 원점 미확정 · UV 꺼짐, 스스로 움직이지 않음.
최근 명령 id 8 개를 기억해 같은 명령을 두 번 실행하지 않고, 하트비트가 1.5 s 끊기면 PAUSED(LINK) + UV 끔.
콘솔 출력은 ASCII 만.
"""
import argparse
import re
import secrets
import socket
import sys
import time
import pathlib
from collections import OrderedDict

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))

from wafer_cell import cell_config as C          # noqa: E402
from wafer_cell import protocol as P             # noqa: E402
from wafer_cell.sim import CellSim, CommandError, LIVE_CMDS   # noqa: E402

SAFE = re.compile(r"[^A-Za-z0-9_.+\-/:#]")
LV = {"info": "I", "warn": "W", "alarm": "A"}
CART_ST = {"away": "AWAY", "moving": "MOVING", "docked": "DOCKED", "leaving": "LEAVING"}


def _word(s, n=40):
    return SAFE.sub("_", str(s))[:n] if s not in (None, "") else None


class FakeMega:
    def __init__(self, seed=1, speed=1.0, clock=time.monotonic, boot_id=None):
        self.seed, self.speed, self.clock = seed, float(speed), clock
        self.reset(boot_id)

    def reset(self, boot_id=None):
        """전원 투입 / 포트 열기(DTR 리셋)와 같다."""
        self.sim = CellSim(seed=self.seed, source="SIM", judge_mode="external")
        self.sim.speed = self.speed
        self.boot = boot_id or secrets.token_hex(3).upper()
        self.t0 = self.last_step = self.clock()
        self.dec = P.Decoder(max_len=P.MAX_PC_LINE)
        self.out = []
        self.acks = OrderedDict()
        self.last_hb = None
        self.lost = False
        self.st_q = 0
        self.ev_sent = 0
        self.last_st = -1.0
        self.last_sl, self.last_sl_t = None, -10.0
        self.frozen = False                  # 시험용: 장비가 멈춘 척(아무것도 안 보냄)
        self.trace = getattr(self, "trace", None)   # 시험용: {} 를 넣으면 ST 순번별 스냅샷을 모음
        self._send("HEL", {"p": P.PROTO_VERSION, "b": self.boot, "fw": "fake-sim"})

    # ------------------------------------------------------------ 입출력
    def _send(self, typ, fields):
        try:
            self.out.append(P.encode(typ, {k: v for k, v in fields.items() if v is not None}))
        except P.ProtoError:
            pass

    def take_output(self):
        out, self.out = b"".join(self.out), []
        return out

    def receive(self, data):
        if self.frozen:
            return
        for m in self.dec.feed(data):
            if P.validate(m):
                continue
            self._handle(P.typed(m), m.type)

    def _handle(self, f, typ):
        if typ == "HB":
            self.last_hb, self.lost = self.clock(), False
        elif typ == "JDG":
            self.sim.submit_judgment(f["id"], f["r"], f.get("dr") or 0.0, f.get("dt") or 0.0)
        elif typ == "CMD":
            cid, c = f["id"], f["c"]
            if cid in self.acks:                       # 같은 id 다시 옴 -> 실행하지 않고 같은 답
                ok, e = self.acks[cid]
                self._send("ACK", {"id": cid, "ok": ok, "e": e, "st": self.sim.state})
                return
            ok, e = True, "-"
            if c == "hello":
                self._send("HEL", {"p": P.PROTO_VERSION, "b": self.boot, "fw": "fake-sim"})
            elif c not in LIVE_CMDS:
                ok, e = False, "unknown"
            elif f.get("x") and f["x"] != self.sim.state:
                ok, e = False, "bad_state"
            else:
                try:
                    self.sim.command(c)
                except CommandError as err:
                    ok, e = False, err.code if err.code in P.ACK_ERRORS else "bad_state"
            self.acks[cid] = (ok, e)
            while len(self.acks) > 8:
                self.acks.popitem(last=False)
            self._send("ACK", {"id": cid, "ok": ok, "e": e, "st": self.sim.state})

    # ------------------------------------------------------------ 진행
    def step(self):
        now = self.clock()
        dt, self.last_step = min(now - self.last_step, 0.25), now
        if self.frozen:
            return
        if self.last_hb is not None and not self.lost and now - self.last_hb > C.LINK_LOSS_S:
            self.lost = True
            self.sim.link_lost()
        try:
            self.sim.tick(dt)
        except Exception as e:
            self.sim.raise_internal("SIM_EXCEPTION", type(e).__name__)
        for req in self.sim.take_capture_requests():
            self._send("CAP", {"id": req["id"], "n": req["n"] or 0, "w": req["w"] or 0})
        for e in self.sim.events_since(self.ev_sent):
            self.ev_sent = e["seq"]
            self._send("EV", {"q": e["seq"] % 65536, "lv": LV.get(e["level"], "I"), "c": _word(e["code"], 20),
                              "d": _word(e.get("d")), "stp": e.get("step"), "n": e.get("n"), "k": e.get("k"),
                              "w": e.get("w"), "s": e.get("seat")})
        if now - self.last_st >= 1.0 / C.ST_HZ - 1e-6:
            self.last_st = now
            snap = self.sim.snapshot()                    # ST · SL 은 같은 순간의 스냅샷 하나에서 만든다
            self._send_st(now, snap)
            if self.trace is not None:                    # 시험용: 보낸 ST 순번 -> 그 순간 장비 상태
                self.trace[self.st_q] = snap
            sl = self._sl_fields(snap)
            if sl != self.last_sl or now - self.last_sl_t >= C.SL_PERIOD_S:
                self.last_sl, self.last_sl_t = sl, now
                self._send("SL", sl)

    def _send_st(self, now, snap):
        st, m, sens, uv = snap["step"], snap["meas"], snap["sensors"], snap["uv"]
        self.st_q = (self.st_q + 1) % 65536
        self._send("ST", {
            "q": self.st_q, "ms": int((now - self.t0) * 1000) % 4294967296, "st": snap["state"],
            "stp": st["key"] if st else "-", "ai": st["action"] if st else 0, "n": snap["slot"] or 0,
            "k": snap["ng_slot"] or 0, "w": snap["wafer_no"] or 0,
            "th": round(snap["pose"]["theta"], 2), "zc": round(snap["pose"]["zc"], 1), "rx": round(snap["pose"]["rx"], 1),
            "hm": P.homed_bits(snap["homed_axes"]), "ths": P.mask5(sens[k] for k in C.TH_SENSORS),
            "zd": "-" if m["zd"] is None else round(m["zd"], 1), "rd": "-" if m["rd"] is None else round(m["rd"], 1),
            "s4": sens["S4"], "ti": sens["TOF_IN"], "tn": sens["TOF_NG"], "d9": sens["D9"], "uv": uv["lamp"],
            "au": uv["a_uv"], "ul": "-" if uv["left_s"] is None else round(uv["left_s"], 1),
            "pr": snap["pause_reason"] or "-", "al": snap["alarm"]["code"] if snap["alarm"] else "-"})

    def _sl_fields(self, snap):
        cart, c, j = snap["cart"], snap["counts"], snap["judgment"] or {}
        mv = self.sim.cart["moves"][0]["code"] if self.sim.cart["moves"] else None
        return {
            "in": "".join(x["code"] for x in snap["docks"]["input"]["slots"]),
            "ng": "-" if snap["docks"]["ng"]["count"] is None else snap["docks"]["ng"]["count"],
            "cs": "".join(cart["seats"][x] or "-" for x in C.CART_SEATS), "ca": CART_ST[cart["state"]],
            "cal": cart["aligned"] or "-", "cmv": _word(mv) if mv else "-",
            "fw": snap["fork"]["wafer"], "fc": snap["fork"]["cassette"], "cr": snap["inspect"]["wafer"],
            "uh": snap["uvh"]["wafer"], "ins": c["inspected"], "okc": c["ok"], "ngc": c["ng"], "emp": c["empty"],
            "cyc": "-" if c["cycle_last"] is None else c["cycle_last"], "dep": c["carts"],
            "ju": j.get("result") or "-", "jw": j.get("no") or 0,
            "jdr": j.get("dr") if j else "-", "jdt": j.get("dth") if j else "-"}


def _serve_tcp(port, speed, seed):
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("127.0.0.1", port))
    srv.listen(1)
    print(f"fake Mega (TCP) on 127.0.0.1:{port}  speed x{speed}  Ctrl+C to quit")
    while True:
        conn, addr = srv.accept()
        print(f"connected {addr[0]}:{addr[1]} -> reset (like Mega DTR auto-reset)")
        fake = FakeMega(seed=seed, speed=speed)
        conn.setblocking(False)
        try:
            while True:
                try:
                    data = conn.recv(4096)
                    if not data:
                        break
                    fake.receive(data)
                except BlockingIOError:
                    pass
                fake.step()
                out = fake.take_output()
                if out:
                    conn.sendall(out)
                time.sleep(0.01)
        except (ConnectionError, OSError):
            pass
        finally:
            conn.close()
            print("disconnected")


def _serve_serial(port, baud, speed, seed):
    import serial
    ser = serial.Serial(port, baud, timeout=0)
    print(f"fake Mega (serial) on {port} @ {baud}  speed x{speed}  Ctrl+C to quit")
    fake = FakeMega(seed=seed, speed=speed)
    while True:
        data = ser.read(4096)
        if data:
            fake.receive(data)
        fake.step()
        out = fake.take_output()
        if out:
            ser.write(out)
        time.sleep(0.01)


def main():
    try:
        sys.stdout.reconfigure(errors="replace")
    except Exception:
        pass
    ap = argparse.ArgumentParser(description="fake R22 cell Mega (CellSim behind the line protocol)")
    ap.add_argument("--tcp", type=int, default=None, help=f"listen on 127.0.0.1:PORT (e.g. {C.FAKE_TCP_PORT})")
    ap.add_argument("--port", default=None, help="serial port (virtual COM pair)")
    ap.add_argument("--baud", type=int, default=C.SERIAL_BAUD)
    ap.add_argument("--speed", type=float, default=1.0)
    ap.add_argument("--seed", type=int, default=1)
    a = ap.parse_args()
    try:
        if a.port:
            _serve_serial(a.port, a.baud, a.speed, a.seed)
        else:
            _serve_tcp(a.tcp or C.FAKE_TCP_PORT, a.speed, a.seed)
    except KeyboardInterrupt:
        print("bye")


if __name__ == "__main__":
    main()
