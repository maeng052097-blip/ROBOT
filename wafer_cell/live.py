"""live.py - 실장비(LIVE) 데이터 소스: 셀 Mega 줄 프로토콜(PROTOCOL.md)을 받아 시뮬과 같은 모양의 스냅샷으로 바꾼다.

전송:
  SerialTransport  pyserial 은 열 때만 import(없으면 NO_PYSERIAL), 끊기면 1 s 마다 다시 연다(명령은 다시 보내지 않음)
  TcpTransport     tcp:HOST:PORT (fake_mega.py --tcp)
  FakeTransport    같은 프로세스의 FakeMega (server.py --serial fake)
  QueueTransport   단위테스트용
연결 상태(link.status): DOWN 연결 안 됨 · NO_PYSERIAL · WAIT_HELLO 응답 대기 · VERSION 프로토콜 불일치 · STALE 데이터 멈춤 · OK
Mega 가 공정의 권한자다. PC 는 HB(0.5 s) · CMD(원점/시작/일시정지/재개/정지/알람 해제) · JDG(판정)만 보낸다.
콘솔 출력 없음.
"""
import socket
import threading
import time

from wafer_cell import cell_config as C
from wafer_cell import protocol as P
from wafer_cell import sequence as Q
from wafer_cell import texts as X
from wafer_cell.sim import CommandError

CART_ST = {"AWAY": "away", "MOVING": "moving", "DOCKED": "docked", "LEAVING": "leaving"}
LV = {"I": "info", "W": "warn", "A": "alarm"}
ACK_KR = {"bad_state": "지금 장비 상태에서는 할 수 없음", "estop": "E-STOP 해제 필요", "not_homed": "원점 복귀 먼저",
          "cell_not_clear": "셀 안에 웨이퍼/카세트가 남음 (손으로 치운 뒤)", "unknown": "장비가 모르는 명령",
          "bad_arg": "인자 오류", "busy": "장비 바쁨"}


class TransportError(Exception):
    pass


class SerialTransport:
    def __init__(self, port, baud=C.SERIAL_BAUD):
        self.name, self.baud, self.ser = str(port), int(baud), None

    def open(self):
        try:
            import serial                                   # 열 때만(설치 안 된 PC 에서도 SIM 은 돌게)
        except ImportError:
            raise TransportError("NO_PYSERIAL")
        try:
            self.ser = serial.Serial(self.name, self.baud, timeout=0, write_timeout=0.2)
        except Exception as e:
            self.ser = None
            raise TransportError(type(e).__name__)

    def is_open(self):
        return self.ser is not None

    def read(self):
        try:
            return self.ser.read(4096) or b""
        except Exception:
            self.close()
            raise TransportError("read")

    def write(self, data):
        try:
            self.ser.write(data)
        except Exception:
            self.close()
            raise TransportError("write")

    def close(self):
        try:
            if self.ser:
                self.ser.close()
        finally:
            self.ser = None


class TcpTransport:
    def __init__(self, host, port):
        self.host, self.port, self.sock = host, int(port), None
        self.name = f"tcp:{host}:{port}"

    def open(self):
        try:
            s = socket.create_connection((self.host, self.port), timeout=0.5)
            s.setblocking(False)
            self.sock = s
        except OSError as e:
            self.sock = None
            raise TransportError(type(e).__name__)

    def is_open(self):
        return self.sock is not None

    def read(self):
        try:
            data = self.sock.recv(65536)
            if not data:
                self.close()
                raise TransportError("closed")
            return data
        except BlockingIOError:
            return b""
        except OSError:
            self.close()
            raise TransportError("read")

    def write(self, data):
        try:
            self.sock.sendall(data)
        except OSError:
            self.close()
            raise TransportError("write")

    def close(self):
        if self.sock:
            try:
                self.sock.close()
            except OSError:
                pass
        self.sock = None


class FakeTransport:
    """같은 프로세스의 FakeMega. 열 때마다 리셋(진짜 Mega 의 DTR 자동 리셋처럼)."""

    def __init__(self, fake):
        self.fake, self.name, self._open = fake, "fake", False

    def open(self):
        self.fake.reset()
        self._open = True

    def is_open(self):
        return self._open

    def read(self):
        self.fake.step()
        return self.fake.take_output()

    def write(self, data):
        self.fake.receive(data)

    def close(self):
        self._open = False


class QueueTransport:
    """단위테스트: inbox 에 넣은 바이트를 읽고, 쓴 바이트는 sent 에 쌓는다."""

    def __init__(self, name="queue"):
        self.name, self.inbox, self.sent, self._open, self.fail_open = name, [], [], False, None

    def open(self):
        if self.fail_open:
            raise TransportError(self.fail_open)
        self._open = True

    def is_open(self):
        return self._open

    def read(self):
        out, self.inbox = b"".join(self.inbox), []
        return out

    def write(self, data):
        self.sent.append(data)

    def close(self):
        self._open = False


def make_transport(spec, baud=C.SERIAL_BAUD, seed=1, speed=1.0):
    s = str(spec or "")
    if s.lower() == "fake":
        from wafer_cell.fake_mega import FakeMega
        return FakeTransport(FakeMega(seed=seed, speed=speed))
    if s.lower().startswith("tcp:"):
        _, host, port = s.split(":", 2)
        return TcpTransport(host, int(port))
    if not s:
        raise ValueError("--serial required for --source live (fake | tcp:HOST:PORT | COMx)")
    return SerialTransport(s, baud)


class LiveSource:
    kind = "LIVE"

    def __init__(self, transport, clock=time.monotonic):
        self.tr, self.clock = transport, clock
        self.faults = set()
        self.dec = P.Decoder(max_len=P.MAX_MEGA_LINE)
        self.status, self.err = "DOWN", None
        self.hello, self.st, self.sl = None, None, None
        self.st_at = self.opened_at = self.last_open_try = None
        self.last_hb = self.last_hello_req = 0.0
        self.hb_n = self.cmd_id = 0
        self.frames = 0
        self.events, self.seq = [], 0
        self.alarm_ev = {}
        self.acks = {}
        self.caps, self.finished, self._pending_done = [], [], None
        self.cycles = []
        self.counters = {"bad_line": 0, "bad_field": 0, "st_gap": 0, "reboot": 0, "reconnect": 0}
        self._lock = threading.Lock()
        self._log("info", "BOOT")

    # ------------------------------------------------------------ 로그
    def _log(self, level, code, d="", t=None, **params):
        self.seq += 1
        e = {"seq": self.seq, "t": round(t if t is not None else self._dev_t(), 2), "level": level, "code": code,
             "d": str(d or "")}
        e.update({k: v for k, v in params.items() if v is not None})
        e["msg"] = X.event_text(code, e["d"], **{k: v for k, v in params.items() if v is not None})
        self.events.append(e)
        if len(self.events) > 400:
            del self.events[:len(self.events) - 400]
        return e

    def _dev_t(self):
        return (self.st or {}).get("ms", 0) / 1000.0

    def events_since(self, seq):
        return [e for e in self.events if e["seq"] > seq]

    # ------------------------------------------------------------ 연결
    @property
    def state(self):
        return (self.st or {}).get("st") or "IDLE"

    def _try_open(self, now):
        if self.last_open_try is not None and now - self.last_open_try < C.RECONNECT_S:
            return
        self.last_open_try = now
        try:
            self.tr.open()
        except TransportError as e:
            self.status, self.err = ("NO_PYSERIAL" if str(e) == "NO_PYSERIAL" else "DOWN"), str(e)
            return
        self.opened_at, self.hello, self.st, self.sl, self.st_at = now, None, None, None, None
        self.dec = P.Decoder(max_len=P.MAX_MEGA_LINE)
        self.status, self.err = "WAIT_HELLO", None
        self.counters["reconnect"] += 1
        self._log("info", "LINK_UP", self.tr.name)

    def _write(self, data):
        try:
            self.tr.write(data)
            return True
        except TransportError as e:
            self._lost(str(e))
            return False

    def _lost(self, why):
        self.status, self.err = "DOWN", why
        self._log("warn", "LINK_DOWN", why)
        with self._lock:
            for a in self.acks.values():
                if a["res"] is None:
                    a["res"] = {"ok": False, "e": "link_down"}
                    a["ev"].set()

    def tick(self, dt=None):
        now = self.clock()
        if not self.tr.is_open():
            self._try_open(now)
            if not self.tr.is_open():
                return
        try:
            data = self.tr.read()
        except TransportError as e:
            self._lost(str(e))
            return
        if data:
            for m in self.dec.feed(data):
                self._on(m, now)
        self.counters["bad_line"] = sum(self.dec.errors.values())
        if now - self.last_hb >= C.HB_PERIOD_S:
            self.last_hb = now
            self.hb_n = (self.hb_n + 1) % 65536
            self._write(P.encode("HB", {"t": self.hb_n}))
        if self.hello is None:
            self.status = "WAIT_HELLO"
            if self.opened_at is not None and now - self.opened_at > 2.0 and now - self.last_hello_req > 2.0:
                self.last_hello_req = now                  # HEL 을 놓쳤으면 2 s 마다 다시 묻는다
                self.cmd_id = self.cmd_id % 65535 + 1
                self._write(P.encode("CMD", {"id": self.cmd_id, "c": "hello"}))
        elif int(self.hello.get("p") or 0) != P.PROTO_VERSION:
            self.status = "VERSION"
        elif self.st_at is None or now - self.st_at > C.STALE_S:
            self.status = "STALE"
        else:
            self.status = "OK"

    def _on(self, m, now):
        if P.validate(m):
            self.counters["bad_field"] += 1
            return
        f = P.typed(m)
        if m.type == "HEL":
            if self.hello is not None and f.get("b") != self.hello.get("b"):
                self.counters["reboot"] += 1
                self.st, self.sl = None, None
                self._log("warn", "REBOOT", f.get("b"))
            self.hello = f
        elif m.type == "ST":
            if self.st is not None:
                if f["ms"] < self.st["ms"] and self.hello is not None:
                    self.counters["reboot"] += 1
                    self._log("warn", "REBOOT", "ms")
                gap = (f["q"] - self.st["q"]) % 65536
                if gap > 1:
                    self.counters["st_gap"] += gap - 1
            self.st, self.st_at = f, now
            self.frames += 1
        elif m.type == "SL":
            self.sl = f
        elif m.type == "EV":
            self._on_ev(f)
        elif m.type == "CAP":
            self.caps.append({"id": f["id"], "n": f["n"] or None, "w": f["w"] or None, "wafer": None,
                              "t": round(self._dev_t(), 2)})
        elif m.type == "ACK":
            with self._lock:
                a = self.acks.get(f["id"])
                if a and a["res"] is None:
                    a["res"] = {"ok": f["ok"], "e": f.get("e"), "st": f.get("st")}
                    a["ev"].set()

    def _on_ev(self, f):
        code = f["c"]
        e = self._log(LV.get(f["lv"], "info"), code, f.get("d") or "", step=f.get("stp"), n=f.get("n") or None,
                      k=f.get("k") or None, w=f.get("w") or None, seat=f.get("s"))
        if f["lv"] == "A":
            self.alarm_ev[code] = e
        if code == "WDONE":
            self._flush_done()
            j = (self.sl or {})
            self._pending_done = {"no": f.get("w"), "src_slot": f.get("n"), "dest": f.get("d"),
                                  "judgment": "NG" if f.get("d") == "N" else "OK",
                                  "dest_slot": f.get("k") if f.get("d") == "N" else f.get("n"),
                                  "dr": j.get("jdr") if j.get("jw") == f.get("w") else None,
                                  "dth": j.get("jdt") if j.get("jw") == f.get("w") else None,
                                  "uv_s": None, "t_pick": None, "t_inspect": None, "t_done": e["t"], "cycle_s": None}
        elif code == "CYCLE":
            try:
                c = float(f.get("d"))
            except (TypeError, ValueError):
                c = None
            if c is not None:
                self.cycles.append(c)
            if self._pending_done and self._pending_done["no"] == f.get("w"):
                self._pending_done["cycle_s"] = c
            self._flush_done()

    def _flush_done(self):
        if self._pending_done:
            self.finished.append(self._pending_done)
            self._pending_done = None

    # ------------------------------------------------------------ 명령 · 판정
    def command(self, cmd, args=None):
        if cmd not in P.LIVE_CMDS or cmd == "hello":
            raise CommandError("sim_only", "실장비에는 원점/시작/일시정지/재개/정지/알람 해제만 보냄")
        if self.status != "OK":
            raise CommandError("link_down", f"장비 연결 {self.status}")
        self.cmd_id = self.cmd_id % 65535 + 1
        cid = self.cmd_id
        with self._lock:
            self.acks[cid] = {"ev": threading.Event(), "res": None, "t": self.clock()}
            for k in [k for k, v in self.acks.items() if self.clock() - v["t"] > 30]:
                self.acks.pop(k, None)
        if not self._write(P.encode("CMD", {"id": cid, "c": cmd, "x": self.state})):
            raise CommandError("link_down", "장비 연결 끊김")
        return ("pending", cid)

    def wait_ack(self, cid, timeout=C.ACK_TIMEOUT_S):
        with self._lock:
            a = self.acks.get(cid)
        if a is None:
            return None
        if not a["ev"].wait(timeout):
            return None
        with self._lock:
            self.acks.pop(cid, None)
        r = dict(a["res"])
        r["msg"] = ACK_KR.get(r.get("e"), r.get("e") or "")
        return r

    def take_capture_requests(self):
        out, self.caps = self.caps, []
        return out

    def submit_judgment(self, cid, result, dr=0.0, dth=0.0):
        return self._write(P.encode("JDG", {"id": int(cid), "r": result, "dr": round(float(dr), 1),
                                            "dt": round(float(dth), 2)}))

    def finished_wafers(self):
        out, self.finished = self.finished, []
        return out

    def raise_internal(self, code, d=""):
        self._log("alarm", code, d)

    # ------------------------------------------------------------ 스냅샷 (시뮬과 같은 모양)
    def snapshot(self):
        st, sl = self.st or {}, self.sl or {}
        now = self.clock()
        state = st.get("st") or "IDLE"
        stp = st.get("stp")
        n, k, w = st.get("n") or None, st.get("k") or None, st.get("w") or None
        seat = sl.get("cal")
        ai = st.get("ai") or 0
        step = None
        act = None
        stage = None
        if stp:
            d = Q.describe(stp, n, k, seat)
            if d:
                stage = Q.STAGE_OF_GROUP.get(d[0])
                step = {"key": stp, "group": d[0], "title": d[1], "detail": d[2], "stage": stage,
                        "action": ai, "actions": max(1, Q.action_count(stp))}
                act = Q.action_at(stp, ai, n, k, seat)
        if state == "WAIT_CART":
            stage = "cart"
        elif state == "WAIT_NG":
            stage = "ng"
        pose = {"theta": st.get("th") or 0.0, "zc": st.get("zc") or 0.0, "rx": st.get("rx") or 0.0}
        zd, rd = st.get("zd"), st.get("rd")
        meas = {"zd": zd, "rd": rd, "zc_s": None if zd is None else round(C.zc_from_sharp(zd), 1),
                "rx_s": None if rd is None else round(C.rx_from_sharp(rd), 1), "tof_in": None, "tof_ng": None}
        target = exp = None
        if act is not None and state in ("HOMING", "RUNNING"):
            if act.kind in ("z", "r", "rot") and not act.corr:
                target = {"axis": {"z": "zc", "r": "rx", "rot": "theta"}[act.kind], "to": act.value}
            elif act.kind == "expect":
                exp = {"sensor": act.sensor, "value": act.expect}
            elif act.kind == "check_pick":
                exp = {"sensor": "S4", "value": True}
            elif act.kind == "expect_th":
                exp = {"sensor": C.TH_SENSORS[C.STATION_ORDER.index(act.sensor)], "value": True}
        mask = st.get("ths") or [False] * 5
        d9 = bool(st.get("d9", True))
        sens = dict(zip(C.TH_SENSORS, mask))
        sens.update({"S4": bool(st.get("s4")), "TOF_IN": bool(st.get("ti")), "TOF_NG": bool(st.get("tn")),
                     "A_UV": bool(st.get("au")), "D9": d9, "ESTOP": not d9 if self.st else False})
        codes = sl.get("in") or "-----"
        in_present = codes != "-" * C.SLOT_COUNT
        ngc = sl.get("ng")
        ng_present = bool(st.get("tn")) and ngc is not None
        k0 = C.SLOT_COUNT - (ngc or 0)
        ju, jw = sl.get("ju"), sl.get("jw") or None
        jfor = (lambda on: (ju if on and jw and jw == w else None))
        homed_axes = P.homed_from_bits(st.get("hm"))
        alarm = None
        al = st.get("al")
        if al and state in ("ALARM", "ESTOP"):
            ev = self.alarm_ev.get(al) or {}
            alarm = {"code": al, "d": ev.get("d", ""), "msg": X.alarm_text(al, ev.get("d", "")),
                     "step": ev.get("step"), "slot": ev.get("n"), "fatal": X.alarm_fatal(al), "t": ev.get("t")}
        cal = sl.get("cal")
        cstate = CART_ST.get(sl.get("ca"), "away") if sl else "away"
        mode = act.mode if act is not None and act.kind == "rot" else "swing"
        to = act.value if act is not None and act.kind == "rot" else pose["theta"]
        rx_ok = meas["rx_s"] is not None and meas["rx_s"] <= C.RX_RET_OK
        ok, why = Q.can_rotate(pose["zc"], rx_ok, pose["theta"], to, mode, bool(sl.get("fw")), bool(sl.get("fc")))
        cmv = sl.get("cmv")
        snap = {
            "tick": self.frames, "t": round(self._dev_t(), 3), "source": "LIVE", "state": state,
            "pause_reason": st.get("pr") if state == "PAUSED" else None, "speed": 1.0,
            "pose": pose, "meas": meas, "target": target,
            "homed": all(homed_axes.values()), "homed_axes": homed_axes,
            "step": step, "stage": stage, "phase": None,
            "slot": n, "ng_slot": k, "wafer_no": w,
            "sensors": sens, "th_low": sum(1 for x in mask if x), "expect": exp,
            "docks": {"input": {"present": in_present, "seat": cal if in_present else None,
                                "slots": [{"n": i + 1, "code": codes[i]} for i in range(C.SLOT_COUNT)]},
                      "ng": {"present": ng_present, "count": ngc if ng_present else None,
                             "slots": [{"n": i, "code": ("N" if ng_present and i > k0 else "-")}
                                       for i in range(1, C.SLOT_COUNT + 1)]}},
            "fork": {"wafer": bool(sl.get("fw")), "judgment": jfor(sl.get("fw")), "cassette": bool(sl.get("fc"))},
            "inspect": {"wafer": bool(sl.get("cr")), "judgment": jfor(sl.get("cr"))},
            "uvh": {"wafer": bool(sl.get("uh"))},
            "uv": {"lamp": bool(st.get("uv")), "a_uv": bool(st.get("au")), "total_s": C.UV_S,
                   "left_s": st.get("ul"), "handed": None, "missing": []},
            "cart": {"state": cstate,
                     "seats": {s: (None if c == "-" else c) for s, c in zip(C.CART_SEATS, sl.get("cs") or "----")},
                     "aligned": cal if cstate == "docked" else None,
                     "caption": X.cart_text(cmv) if cmv else None,
                     "rot": 180.0 if cal and cal[0] == "R" else 0.0,
                     "lat": -C.CART_LATERAL if cal and cal[1] == "R" else 0.0,
                     "dist": 0.0 if cstate == "docked" else (900.0 if cstate == "away" else C.CART_RETREAT),
                     "departed": sl.get("dep") or 0},
            "judgment": ({"no": jw, "result": ju, "dr": sl.get("jdr"), "dth": sl.get("jdt"), "t": None} if ju else None),
            "ng_next": False, "ng_rate": 0.0,
            "alarm": alarm,
            "interlock": {"rotate_ok": ok, "why": why, "mode": mode},
            "counts": {"inspected": sl.get("ins") or 0, "ok": sl.get("okc") or 0, "ng": sl.get("ngc") or 0,
                       "empty": sl.get("emp") or 0, "ng_dock": ngc if ng_present else None,
                       "cycle_last": sl.get("cyc"),
                       "cycle_avg": round(sum(self.cycles) / len(self.cycles), 1) if self.cycles else None,
                       "carts": sl.get("dep") or 0},
            "faults": [],
            "operator": {"auto": False, "pending": []},
            "link": {"status": self.status, "port": self.tr.name, "err": self.err,
                     "age_s": None if self.st_at is None else round(now - self.st_at, 2),
                     "boot": (self.hello or {}).get("b"), "proto": (self.hello or {}).get("p"),
                     "fw": (self.hello or {}).get("fw"), "counters": dict(self.counters)},
            "log_seq": self.seq,
        }
        snap["headline"], snap["subline"] = X.headline(snap)
        return snap
