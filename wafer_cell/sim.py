"""R22 웨이퍼 검사셀 시뮬레이터(순수 로직, 스레드 없음) - 실장비 Mega 공정의 모형.

CellSim.tick(dt) 로 시간을 진행하고 command(name, **args) 로 조작한다.
  - 축은 등속 이동(가정 속도). 가속 무시 -> 사이클은 하한값.
  - 지령값(pose)과 측정값(meas: Z/R Sharp, θ 자리 센서 5 개)을 나눈다. 고장 주입은 측정값을 바꾼다.
  - 웨이퍼/카세트 이동은 포크 면이 높이를 교차하는 순간 적용, 충돌은 이동 시작 전에 구간 전체로 판정(geometry.py).
  - 공정 흐름 = 사용자 설명 + R22 문서(sequence.py):
      대차 받침 카세트 -> ① -> 슬롯 1..5: 집기 -> ② 검사(촬영 · 판정 · δ 보정 다시 집기)
        -> NG: ③ (위 슬롯부터) / OK: UV 탑 넘김 -> UV -> 다시 집기 -> ① 원래 슬롯
      -> 5 장 끝 -> ① 카세트를 원래 받침에 -> 대차 다음 받침 / 다 하면 출발 -> 새 대차.
  - 판정: judge_mode='internal' 이면 시뮬 안에서(OK, NG 비율, '다음 웨이퍼 NG'),
          'external' 이면 촬영 요청(pending_captures)을 내고 submit_judgment() 를 기다린다(PC 판정 = 실장비와 같은 길).
  - 기계가 아는 것(믿음)과 시뮬 진실을 나눈다: ① 슬롯 코드(U/O/K/N/E), ③ 포인터는 기계 믿음.
문자열(한글)은 texts.py 가 만든다. 이 모듈은 print 하지 않는다.
"""
import random

from wafer_cell import cell_config as C
from wafer_cell import geometry as G
from wafer_cell import sequence as Q
from wafer_cell import texts as X
from wafer_cell import tof as T

STATES = ("IDLE", "HOMING", "READY", "RUNNING", "WAIT_CART", "WAIT_NG", "PAUSED", "ALARM", "ESTOP")
RUN_STATES = ("HOMING", "RUNNING")
WAIT_STATES = ("WAIT_CART", "WAIT_NG")
LIVE_CMDS = ("home", "start", "pause", "resume", "stop", "reset")
SIM_ONLY = ("speed", "fault", "estop", "operator", "auto_operator", "cart_map", "input_map", "remove_wafer",
            "debug_hold", "next_ng", "ng_rate")
FAULTS = ("th_double", "th_dead", "belt_skip", "z_sharp_drift", "r_sharp_fail", "s4_false_none", "s4_false_present",
          "tof_in_none", "uv_led_fail", "uv_overtemp", "corr_out", "judge_drop", "camera_fail", "cart_delay",
          "sim_exception")
ONE_SHOT = ("belt_skip", "s4_false_none", "s4_false_present", "sim_exception")
JUDGE_FAULTS = ("corr_out", "judge_drop", "camera_fail")      # PC(판정) 쪽 고장
AXIS_KINDS = ("z", "r", "rot", "home_th")
SLOT_CODES = "UOKNE"       # U 미처리(있다고 봄) · O 꺼냄 · K 정상 원위치 · N 불량(③) · E 빈 슬롯(S4 없음)


class CommandError(Exception):
    def __init__(self, code, msg=""):
        super().__init__(code)
        self.code, self.msg = code, msg or code


class CellSim:
    def __init__(self, seed=1, source="SIM", judge_mode="internal"):
        self.rng = random.Random(seed)                 # 판정(internal) 난수열
        self.noise = random.Random(seed + 7)           # 센서 잡음(공정 흐름과 분리)
        self.source, self.judge_mode = source, judge_mode
        self.t, self.tick_n, self.speed = 0.0, 0, 1.0
        self.theta, self.zc, self.rx = 150.0, 140.0, 30.0    # 전원 투입 자세(자리 사이 · 중간 높이 · 조금 나옴)
        self.theta_err = 0.0                           # 실제 θ - 지령 θ (벨트 이빨 튐)
        self.homed = {"theta": False, "z": False, "r": False}
        self.state, self.prev_state, self.pause_reason, self.pause_req = "IDLE", None, None, None
        self.wafers, self.cassettes, self._wid, self._cid = [], {}, 0, 0
        self.dock = {"input": None, "ng": None}
        self.slot_state = {}                           # ① 카세트 슬롯 코드(기계 믿음)
        self.ng_ptr = 0                                # ③ 카운터(새 카세트 = 0, 위 슬롯부터)
        self.next_input_map = [True] * C.SLOT_COUNT
        self.next_cart_map = list(C.CART_DEFAULT_MAP)
        self.program, self.pc, self.ai, self.phase, self.cur = [], 0, 0, None, None
        self.slot = self.ng_slot = self.wafer = None
        self.wafer_count = 0
        self.judg = None                               # 지금 웨이퍼 판정 {id, result, dr, dth}
        self.last_judgment = None
        self.cap_id = 0
        self.pending_captures = []
        self.ng_rate, self.ng_next = C.NG_RATE, False
        self.auto_operator, self.op_pending, self.op_place_at = True, {}, None
        self.uv = {"lamp": False, "handed": False, "handed_pending": False, "tripped": False}
        self.alarm, self.faults, self.events, self.seq, self.hold = None, set(), [], 0, None
        self.homing_t0, self.estop_in = None, False
        self.finished, self.cycles, self.cycle_t0 = [], [], None
        self.counts = {"inspected": 0, "ok": 0, "ng": 0, "empty": 0}
        self.carts_done = 0
        self.cart = self._new_cart("docked", aligned="FL")
        self._place_ng()
        self._tof_ng_prev = True
        self._log("info", "BOOT")

    # ------------------------------------------------------------ 카세트 · 웨이퍼
    def _new_cassette(self, kind, where, seat=None, fill=False):
        self._cid += 1
        cid = self._cid
        self.cassettes[cid] = {"id": cid, "kind": kind, "where": where, "seat": seat, "stage": "F"}
        if fill:
            for i, present in enumerate(self.next_input_map[:C.SLOT_COUNT]):
                if present:
                    self._wid += 1
                    self.wafers.append({"id": self._wid, "no": None, "cass": cid, "src_cass": cid, "src_slot": i + 1, "slot": i + 1,
                                        "loc": "cass", "judgment": None, "dr": None, "dth": None, "uv_s": 0.0,
                                        "dest": None, "dest_slot": None, "t_pick": None, "t_inspect": None,
                                        "t_done": None, "cycle_s": None})
        return cid

    def _place_ng(self):
        cid = self._new_cassette("ng", "ng")
        self.dock["ng"] = cid
        return cid

    def _wafers_in(self, cid):
        return [w for w in self.wafers if w["loc"] == "cass" and w["cass"] == cid]

    def _wafer_at(self, loc):
        return next((w for w in self.wafers if w["loc"] == loc), None)

    def _wafer(self, wid):
        return next((w for w in self.wafers if w["id"] == wid), None)

    def _fork_cassette(self):
        return next((c for c in self.cassettes.values() if c["where"] == "fork"), None)

    # ------------------------------------------------------------ 대차
    def _new_cart(self, state, aligned=None):
        cart = {"state": state, "seats": {s: None for s in C.CART_SEATS}, "rot": 0.0, "lat": 0.0,
                "dist": 900.0 if state == "away" else 0.0, "moves": []}
        if state != "away":
            self._fill_cart(cart)
        if aligned:
            cart["rot"] = 0.0 if aligned[0] == "F" else 180.0
            cart["lat"] = 0.0 if aligned[1] == "L" else -C.CART_LATERAL
        return cart

    def _fill_cart(self, cart):
        for s, on in zip(C.CART_SEATS, self.next_cart_map):
            if on:
                cart["seats"][s] = self._new_cassette("in", "cart:" + s, seat=s, fill=True)

    def _cart_aligned_seat(self):
        cart = self.cart
        if cart["state"] != "docked" or abs(cart["dist"]) > 0.5:
            return None
        return ("F" if abs(cart["rot"]) < 0.5 else "R") + ("L" if abs(cart["lat"]) < 0.5 else "R")

    def seat_stage(self, s):
        cid = self.cart["seats"].get(s)
        if cid is not None:
            return self.cassettes[cid]["stage"]
        if any(c["seat"] == s and c["stage"] == "W" and c["where"] not in ("gone",) and c["kind"] == "in"
               and self.cart["state"] != "away" for c in self.cassettes.values()):
            return "W"
        return None

    def _cart_plan(self, seat):
        """seat 받침이 로봇 앞에 오도록 이동 구간 목록(seat None = 새 대차 호출)."""
        cart = self.cart
        mv = []
        slow = 3.0 if "cart_delay" in self.faults else 1.0
        if cart["state"] == "away":
            self.cart = cart = self._new_cart("moving")
            seat = next((s for s in C.CART_SEATS if cart["seats"][s] is not None), "FL")
            rot, lat = (0.0 if seat[0] == "F" else 180.0), (0.0 if seat[1] == "L" else -C.CART_LATERAL)
            cart["dist"], cart["rot"], cart["lat"] = 900.0, rot, lat
            mv.append(("CALL", C.CART_CALL_S * slow, {"dist": C.CART_RETREAT}))
            mv.append(("ARRIVE:" + seat, C.CART_DOCK_S, {"dist": 0.0}))
        else:
            rot, lat = (0.0 if seat[0] == "F" else 180.0), (0.0 if seat[1] == "L" else -C.CART_LATERAL)
            cart["state"] = "moving"
            mv.append(("RETREAT", C.CART_RETREAT_S * slow, {"dist": C.CART_RETREAT}))
            if abs(cart["rot"] - rot) > 0.5:
                if abs(cart["lat"]) > 0.5:
                    mv.append(("LATERAL:+%g" % C.CART_LATERAL, C.CART_LATERAL_S, {"lat": 0.0}))
                mv.append(("ROTATE", C.CART_ROTATE_S, {"rot": rot}))
                if abs(lat) > 0.5:
                    mv.append(("LATERAL:%+g" % lat, C.CART_LATERAL_S, {"lat": lat}))
            elif abs(cart["lat"] - lat) > 0.5:
                mv.append(("LATERAL:%+g" % (lat - cart["lat"]), C.CART_LATERAL_S, {"lat": lat}))
            mv.append(("DOCK:" + seat, C.CART_DOCK_S, {"dist": 0.0}))
        cart["moves"] = [{"code": c, "dur": d, "left": d, "to": to, "from": None} for c, d, to in mv]
        self._log("info", "CART", cart["moves"][0]["code"])

    def _cart_leave(self):
        cart = self.cart
        cart["state"] = "leaving"
        cart["moves"] = [{"code": "LEAVE", "dur": C.CART_LEAVE_S, "left": C.CART_LEAVE_S, "to": {"dist": 900.0},
                          "from": None}]
        self._log("info", "CART", "LEAVE")

    def _cart_env(self, dt):
        cart = self.cart
        while dt > 1e-9 and cart["moves"]:
            m = cart["moves"][0]
            if m["from"] is None:
                m["from"] = {k: cart[k] for k in m["to"]}
            use = min(dt, m["left"])
            m["left"] -= use
            dt -= use
            f = 1.0 - (m["left"] / m["dur"] if m["dur"] > 0 else 0.0)
            for k, v in m["to"].items():
                cart[k] = m["from"][k] + (v - m["from"][k]) * f
            if m["left"] <= 1e-9:
                cart["moves"].pop(0)
                if cart["moves"]:
                    self._log("info", "CART", cart["moves"][0]["code"])
        if not cart["moves"]:
            if cart["state"] == "moving":
                cart["state"] = "docked"
                self._log("info", "CART", "DOCKED:" + (self._cart_aligned_seat() or ""))
            elif cart["state"] == "leaving":
                for s, cid in cart["seats"].items():
                    if cid is not None:
                        self.cassettes[cid]["where"] = "gone"
                        for w in self._wafers_in(cid):
                            w["loc"] = "shipped"
                self.carts_done += 1
                self.cart = self._new_cart("away")
                self._log("info", "CART", "LEFT")

    # ------------------------------------------------------------ 작업자(시뮬) - ③ 교체만
    def _request_op(self, task):
        if task in self.op_pending:
            return
        self.op_pending[task] = (self.t + C.OPERATOR_S - 1.0) if self.auto_operator else None

    def _do_op(self, task):
        if task != "ng":
            raise CommandError("bad_arg", "task = ng")
        old = self.dock["ng"]
        if old is not None:
            for w in self._wafers_in(old):
                w["loc"] = "ng_out"
            self.cassettes[old]["where"] = "gone"
            self.dock["ng"] = None
            if self._tof_ng_prev:                      # 빼는 순간을 기록(빼고 놓기가 한 주기 안에 끝나도 '없음'을 놓치지 않게)
                self._tof_ng_prev = False
                self._log("warn", "NG_OUT")
        self.op_pending.pop(task, None)
        self.op_place_at = self.t + 1.0
        self._log("info", "OPERATOR", "NG_SWAP")

    def _op_env(self):
        for task, due in list(self.op_pending.items()):
            if due is not None and self.t >= due:
                self._do_op(task)
        if self.op_place_at is not None and self.t >= self.op_place_at and self.dock["ng"] is None:
            self.op_place_at = None
            self._place_ng()

    # ------------------------------------------------------------ 센서
    def occ(self):
        o = G.Occupancy()
        for kind in C.CASSETTES:
            cid = self.dock[kind]
            if cid is not None:
                o.cass[kind] = {w["slot"]: True for w in self._wafers_in(cid)}
        o.blade = self._wafer_at("blade") is not None
        o.inspect = self._wafer_at("inspect") is not None
        o.uvh = self._wafer_at("uvh") is not None
        o.fork_cassette = self._fork_cassette() is not None
        a = self._cart_aligned_seat()
        if a:
            o.cart_seat = "full" if self.cart["seats"][a] is not None else "free"
        return o

    def th_mask(self):
        m = G.th_mask(self.theta + self.theta_err)
        if "th_double" in self.faults:
            m[0] = True                                    # θ0 센서가 늘 LOW (붙음)
        if "th_dead" in self.faults:
            m[3] = False                                   # θ180 센서 죽음
        return m

    def meas(self):
        n = self.noise.uniform
        zd = C.z_sharp_d(self.zc + (6.0 if "z_sharp_drift" in self.faults else 0.0)) + n(-C.SHARP_NOISE, C.SHARP_NOISE)
        rd = 160.0 if "r_sharp_fail" in self.faults else C.r_sharp_d(self.rx) + n(-C.SHARP_NOISE, C.SHARP_NOISE)
        zd = zd if C.SHARP_MIN_MM <= zd <= C.SHARP_MAX_MM else None
        rd = rd if C.SHARP_MIN_MM <= rd <= C.SHARP_MAX_MM else None
        tin = (C.TOF_CASS_D + n(-C.TOF_NOISE, C.TOF_NOISE)) if (self.dock["input"] is not None
                                                               and "tof_in_none" not in self.faults) else C.TOF_OUT_OF_RANGE
        tng = (C.TOF_CASS_D + n(-C.TOF_NOISE, C.TOF_NOISE)) if self.dock["ng"] is not None else C.TOF_OUT_OF_RANGE
        return {"zd": zd, "rd": rd, "zc_s": None if zd is None else C.zc_from_sharp(zd),
                "rx_s": None if rd is None else C.rx_from_sharp(rd), "tof_in": tin, "tof_ng": tng}

    def sensors(self, m=None):
        m = m or self.meas()
        o = self.occ()
        s4 = G.s4_on(self.theta + self.theta_err, self.zc, self.rx, o)
        if s4 and "s4_false_none" in self.faults:
            s4 = False
        if not s4 and "s4_false_present" in self.faults:
            s4 = True
        out = dict(zip(C.TH_SENSORS, self.th_mask()))
        out.update({"S4": s4, "TOF_IN": T.presence(m["tof_in"]) == "present",
                    "TOF_NG": T.presence(m["tof_ng"]) == "present",
                    "A_UV": self.uv["lamp"] and "uv_led_fail" not in self.faults and not self.uv["tripped"],
                    "D9": not self.estop_in, "ESTOP": self.estop_in})
        return out

    def _rx_ret_ok(self, m=None):
        m = m or self.meas()
        return m["rx_s"] is not None and m["rx_s"] <= C.RX_RET_OK

    def _log(self, level, code, d="", **params):
        self.seq += 1
        e = {"seq": self.seq, "t": round(self.t, 2), "level": level, "code": code, "d": str(d)}
        e.update({k: v for k, v in params.items() if v is not None})
        e["msg"] = X.event_text(code, e["d"], **{k: v for k, v in params.items() if v is not None})
        self.events.append(e)
        if len(self.events) > 400:
            del self.events[:len(self.events) - 400]
        return e

    def events_since(self, seq):
        return [e for e in self.events if e["seq"] > seq]

    # ------------------------------------------------------------ 알람
    def _step_key(self):
        return self.program[self.pc].key if self.pc < len(self.program) else None

    def _raise(self, code, d=""):
        fatal = X.alarm_fatal(code)
        step = self._step_key()
        self.alarm = {"code": code, "d": str(d), "msg": X.alarm_text(code, d), "step": step, "slot": self.slot,
                      "fatal": fatal, "t": round(self.t, 2)}
        self.state = "ALARM"
        self._lamp_off(abort=True)
        self._log("alarm", code, d, step=step, n=self.slot)
        for f in ONE_SHOT:
            self.faults.discard(f)
        if fatal:
            self._abort(lose_home=True)
        else:
            self.cur = None                    # 해제 후 재개하면 지금 동작을 처음부터
        return self.alarm

    def raise_internal(self, code, d=""):
        return self._raise(code, d)

    def _abort(self, lose_home):
        self._lamp_off(abort=True)
        self.program, self.pc, self.ai, self.cur, self.phase = [], 0, 0, None, None
        self.hold, self.pause_req = None, None
        if lose_home:
            self.homed = {k: False for k in self.homed}

    def _lamp_off(self, abort=False):
        if self.uv["lamp"]:
            self.uv["lamp"] = False
            self._log("warn" if abort else "info", "UV_ABORT" if abort else "UV_OFF")
            if self.cur and self.cur["kind"] == "uv":
                self.cur = None

    def link_lost(self):
        """실장비 규칙: PC 하트비트가 끊기면 지금 축 동작만 끝내고 PAUSED(LINK), UV 는 바로 끔."""
        if self.state in RUN_STATES + WAIT_STATES and self.pause_req is None:
            self._lamp_off(abort=True)
            self.pause_req = "LINK"
            self._log("warn", "LINK_LOST")

    # ------------------------------------------------------------ 명령
    def command(self, cmd, **a):
        if cmd in SIM_ONLY and self.source != "SIM":
            raise CommandError("sim_only", "시뮬레이션 전용 명령")
        fn = getattr(self, "_cmd_" + str(cmd), None)
        if fn is None:
            raise CommandError("unknown", f"알 수 없는 명령 {cmd}")
        try:
            return fn(**a) or {}
        except TypeError as e:
            raise CommandError("bad_arg", f"인자 오류: {e}")

    def _need(self, *states):
        if self.state not in states:
            raise CommandError("bad_state", f"지금 상태({self.state})에서는 할 수 없음")

    def cell_clear_problems(self):
        out = [loc for loc in ("blade", "inspect", "uvh") if self._wafer_at(loc)]
        if self._fork_cassette():
            out.append("cassette")
        return out

    def _cmd_home(self):
        self._need("IDLE", "READY")
        if self.estop_in:
            raise CommandError("estop", "E-STOP 해제 필요")
        self._load(Q.homing_steps(), "home")
        self.state, self.homing_t0, self.slot = "HOMING", self.t, None
        self._log("info", "HOME")

    def _cmd_start(self):
        self._need("READY")
        if not all(self.homed.values()):
            raise CommandError("not_homed", "원점 복귀 먼저")
        bad = self.cell_clear_problems()
        if bad:
            raise CommandError("cell_not_clear", "셀 안에 웨이퍼/카세트가 남음: " + ", ".join(bad))
        self.state = "RUNNING"
        self._log("info", "START")
        self._schedule()

    def _cmd_pause(self):
        self._need(*RUN_STATES, *WAIT_STATES)
        self._lamp_off(abort=True)
        if self.cur and self.cur["kind"] in ("capture", "uv"):
            self.cur = None
        self.prev_state, self.state, self.pause_reason = self.state, "PAUSED", "USER"
        self._log("info", "PAUSE")

    def _cmd_resume(self):
        self._need("PAUSED")
        if self.estop_in:
            raise CommandError("estop", "E-STOP 해제 필요")
        hold = self.pause_reason == "HOLD"                # 시뮬 단계 고정은 멈춘 자리에서 그대로 이어감
        self.state, self.pause_reason, self.pause_req = self.prev_state or "RUNNING", None, None
        if self.cur and self.cur["kind"] in ("expect", "expect_th", "ret"):
            self.cur["t0"] = self.t
        if self.cur and self.cur["kind"] in ("capture", "uv") and not hold:
            self._lamp_off(abort=True)
            self.cur = None
        self._log("info", "RESUME")

    def _cmd_stop(self):
        self._need(*RUN_STATES, *WAIT_STATES, "PAUSED")
        self._abort(lose_home=True)
        self.state, self.slot, self.pause_reason = "IDLE", None, None
        self._log("warn", "STOP")

    def _cmd_estop(self, on=True):
        if not on:
            raise CommandError("bad_arg", "E-STOP 해제는 reset")
        self.estop_in = True
        self._abort(lose_home=True)
        self.state, self.pause_reason = "ESTOP", None
        self.alarm = {"code": "ESTOP", "d": "", "msg": X.alarm_text("ESTOP"), "step": None, "slot": self.slot,
                      "fatal": True, "t": round(self.t, 2)}
        self._log("alarm", "ESTOP")

    def _cmd_reset(self):
        self._need("ALARM", "ESTOP")
        if self.state == "ESTOP":
            self.estop_in = False
            self.state, self.alarm = "IDLE", None
            self._log("info", "RESET")
            return
        fatal = self.alarm and self.alarm.get("fatal")
        self.alarm = None
        if fatal or not self.program:
            self.state = "READY" if all(self.homed.values()) else "IDLE"
        else:
            self.prev_state, self.state, self.pause_reason = "RUNNING", "PAUSED", "USER"
        self._log("info", "RESET")

    def _cmd_operator(self, task="ng"):
        self._do_op(task)

    def _cmd_auto_operator(self, on=True):
        self.auto_operator = bool(on)
        for task in self.op_pending:
            self.op_pending[task] = (self.t + C.OPERATOR_S - 1.0) if self.auto_operator else None

    def _cmd_next_ng(self, on=True):
        self.ng_next = bool(on)

    def _cmd_ng_rate(self, value):
        v = float(value)
        if not 0.0 <= v <= 1.0:
            raise CommandError("bad_arg", "NG 비율 0~1")
        self.ng_rate = v

    def _cmd_speed(self, value):
        v = float(value)
        if not 0.1 <= v <= 50:
            raise CommandError("bad_arg", "배속 0.1~50")
        self.speed = v

    def _cmd_input_map(self, slots):
        """(시뮬) 다음에 오는 대차 카세트의 웨이퍼 배치(슬롯 1..5)."""
        self.next_input_map = [bool(x) for x in slots][:C.SLOT_COUNT]

    def _cmd_cart_map(self, seats):
        """(시뮬) 다음에 오는 대차의 카세트 자리(앞 왼 · 앞 오른 · 뒤 왼 · 뒤 오른)."""
        m = [bool(x) for x in seats][:len(C.CART_SEATS)]
        if not any(m):
            raise CommandError("bad_arg", "카세트 1 개 이상")
        self.next_cart_map = m

    def _cmd_fault(self, name, on=True):
        if name not in FAULTS:
            raise CommandError("bad_arg", f"알 수 없는 고장 {name}")
        (self.faults.add if on else self.faults.discard)(name)
        self._log("warn" if on else "info", "FAULT", f"{name} {'ON' if on else 'OFF'}")

    def _cmd_remove_wafer(self, loc):
        self._need("IDLE", "READY", "ALARM")
        if loc == "cassette":
            c = self._fork_cassette()
            if c is None:
                raise CommandError("bad_arg", "포크에 카세트 없음")
            c["where"] = "cart:" + c["seat"]
            c["stage"] = "F" if any(v == "U" for v in self.slot_state.values()) else "D"
            self.cart["seats"][c["seat"]] = c["id"]
            self._log("info", "REMOVE", "cassette")
            return
        w = self._wafer_at(loc)
        if w is None:
            raise CommandError("bad_arg", f"{loc} 에 웨이퍼 없음")
        w["loc"] = "removed"
        if self.slot_state.get(w["src_slot"]) == "O" and self.dock["input"] == w["cass"]:
            self.slot_state[w["src_slot"]] = "E"
        self._log("info", "REMOVE", {"blade": "fork", "uvh": "uv"}.get(loc, loc))

    def _cmd_debug_hold(self, step, frac=0.0, slot=None):
        self.hold = {"step": str(step), "frac": float(frac), "slot": slot}

    # ------------------------------------------------------------ 스케줄
    def _load(self, steps, phase):
        self.program, self.pc, self.ai, self.cur, self.phase = list(steps), 0, 0, None, phase

    def _at_standby(self):
        return (abs(self.theta - C.THETA_STANDBY) < 0.5 and abs(self.zc - C.ZC_SWING) < 0.3
                and self.rx <= C.RX_RET_OK)

    def _wait(self, state, task=None):
        if task:
            self._request_op(task)
        if not self._at_standby():
            self._load(Q.standby_steps(), "standby")
            self.state = "RUNNING"
            return
        if self.state != state and state == "WAIT_NG":
            self._log("warn", "NG_FULL")
        self.state = state

    def _schedule(self):
        """프로그램이 끝났거나 대기 중일 때 다음 일을 고른다."""
        inp = self.dock["input"]
        if inp is not None:
            pend = [n for n in range(1, C.SLOT_COUNT + 1) if self.slot_state.get(n) == "U"]
            if not pend:
                seat = self.cassettes[inp]["seat"]
                if self._cart_aligned_seat() == seat and self.cart["seats"][seat] is None:
                    self._load(Q.cart_out_steps(seat), "cart_out")
                    self.state = "RUNNING"
                    self._log("info", "CASS_DONE", seat=seat)
                    return
                self._wait("WAIT_CART")
                return
            if self.dock["ng"] is None or self.ng_ptr >= C.SLOT_COUNT:
                self._wait("WAIT_NG", "ng")
                return
            n = pend[0]
            self.slot, self.ng_slot, self.wafer, self.judg = n, None, None, None
            self._load(Q.input_steps(n) + Q.inspect_steps(n), "wafer")
            self.state = "RUNNING"
            self.cycle_t0 = self.t
            return
        cart = self.cart
        if cart["state"] == "docked":
            a = self._cart_aligned_seat()
            if a and self.seat_stage(a) == "F":
                self._load(Q.cart_in_steps(a), "cart_in")
                self.state = "RUNNING"
                return
            nxt = next((s for s in C.CART_SEATS if self.seat_stage(s) == "F"), None)
            if nxt:
                self._cart_plan(nxt)
            else:
                self._cart_leave()
        elif cart["state"] == "away":
            self._cart_plan(None)
        self._wait("WAIT_CART")

    def _program_done(self):
        phase = self.phase
        if self.state == "HOMING":
            self._load([], None)
            self.homing_t0 = None
            self.state = "READY"
            self._log("info", "HOMED")
            return
        if phase == "wafer":
            w = self._wafer(self.wafer)
            if w and w["judgment"] == "NG":
                self.ng_slot = C.SLOT_COUNT - self.ng_ptr
                self._load(Q.ng_steps(self.ng_slot), "ng")
            else:
                self._load(Q.uv_steps() + Q.return_steps(self.slot), "okret")
            return
        if phase in ("ng", "okret"):
            self._finish_wafer(phase)
        if phase == "empty":
            self.slot = None
        self._load([], None)
        self._schedule()

    def _finish_wafer(self, phase):
        w = self._wafer(self.wafer)
        dt = round(self.t - (self.cycle_t0 or self.t), 2)
        self.cycles.append(dt)
        if w:
            w["t_done"], w["cycle_s"] = round(self.t, 2), dt
            w["dest"] = "N" if phase == "ng" else "K"
            w["dest_slot"] = self.ng_slot if phase == "ng" else self.slot
            self.finished.append(dict(w))
        if phase == "ng":
            self.ng_ptr += 1
            self.slot_state[self.slot] = "N"
            self.counts["ng"] += 1
        else:
            self.slot_state[self.slot] = "K"
            self.counts["ok"] += 1
        no = w["no"] if w else None
        self._log("info", "WDONE", "N" if phase == "ng" else "K", w=no, n=self.slot, k=self.ng_slot)
        self._log("info", "CYCLE", f"{dt:.1f}", w=no)
        self.wafer, self.slot, self.ng_slot, self.judg = None, None, None, None

    def finished_wafers(self):
        out, self.finished = self.finished, []
        return out

    # ------------------------------------------------------------ 진행
    def tick(self, dt):
        self.tick_n += 1
        if "sim_exception" in self.faults:
            self.faults.discard("sim_exception")
            raise RuntimeError("injected sim exception")
        remaining = max(0.0, float(dt)) * self.speed
        guard = 0
        while remaining > 1e-9 and guard < 4000:
            guard += 1
            if self.state not in RUN_STATES:
                self._env(remaining)
                remaining = 0.0
                break
            used = self._run(remaining)
            if used > 0:
                self._env(used)
                remaining -= used
        if remaining > 1e-9:
            self._env(remaining)

    def _env(self, dt):
        self.t += dt
        self._cart_env(dt)
        self._op_env()
        tn = self.sensors()["TOF_NG"]
        if tn and not self._tof_ng_prev:
            self.ng_ptr = 0
            self._log("info", "NG_SWAP")
        elif not tn and self._tof_ng_prev:
            self._log("warn", "NG_OUT")
        self._tof_ng_prev = tn
        if self.state == "HOMING" and self.homing_t0 is not None and self.t - self.homing_t0 > C.HOMING_TIMEOUT_S:
            self._raise("HOMING_TIMEOUT")
        if self.state in WAIT_STATES:
            if self.pause_req:
                self.prev_state, self.state, self.pause_reason, self.pause_req = self.state, "PAUSED", self.pause_req, None
            else:
                self._schedule()

    def _run(self, budget):
        if self.pause_req and (self.cur is None or self.cur["kind"] not in AXIS_KINDS):
            self.cur = None
            self.prev_state, self.state, self.pause_reason, self.pause_req = self.state, "PAUSED", self.pause_req, None
            return 0.0
        if self.pc >= len(self.program):
            self._program_done()
            return 0.0
        step = self.program[self.pc]
        if self.ai >= len(step.actions):
            self.pc, self.ai = self.pc + 1, 0
            return 0.0
        act = step.actions[self.ai]
        if self.cur is None:
            if (self.hold and self.ai == 0 and self.hold["step"] == step.key and self.hold["frac"] <= 0
                    and self._hold_slot_ok()):
                self.hold = None
                self.prev_state, self.state, self.pause_reason = self.state, "PAUSED", "HOLD"
                return 0.0
            if self.ai == 0:
                self._log("info", "STEP", step=step.key, n=self.slot, k=self.ng_slot,
                          seat=self._step_seat(step), w=self._wafer_no())
            if not self._begin(act, step):
                return 0.0
            if self.cur is None:
                self.ai += 1
                return 0.0
        return self._progress(budget, step)

    def _step_seat(self, step):
        if step.group == "cart_in":
            return self._cart_aligned_seat()
        if step.group == "cart_out":
            inp = self.dock["input"]
            fc = self._fork_cassette()
            c = self.cassettes.get(inp) if inp else fc
            return c["seat"] if c else None
        return None

    def _wafer_no(self):
        w = self._wafer(self.wafer)
        return w["no"] if w else None

    def _hold_slot_ok(self):
        return self.hold.get("slot") in (None, self.slot)

    def _target(self, act):
        j = self.judg or {}
        if act.kind == "r":
            return act.value + (j.get("dr", 0.0) if act.corr else 0.0)
        if act.kind == "rot":
            return act.value + (j.get("dth", 0.0) if act.corr else 0.0)
        return act.value

    def _begin(self, act, step):
        o = self.occ()
        k = act.kind
        if k == "z":
            r = G.z_move(self.theta, self.rx, self.zc, act.value, o)
            if r["collision"]:
                self._raise("COLLISION", "/".join(str(x) for x in r["collision"]))
                return False
            self.cur = {"kind": "z", "from": self.zc, "to": act.value, "events": [list(e) for e in r["events"]]}
        elif k == "r":
            to = self._target(act)
            col = G.r_move(self.theta, self.zc, self.rx, to, o)
            if col:
                self._raise("COLLISION", "/".join(str(x) for x in col))
                return False
            self.cur = {"kind": "r", "from": self.rx, "to": to}
        elif k == "rot":
            to = self._target(act)
            ok, why = Q.can_rotate(self.zc, self._rx_ret_ok(), self.theta, to, act.mode, o.blade, o.fork_cassette)
            if not ok:
                self._raise("INTERLOCK", "+".join(why))
                return False
            self.cur = {"kind": "rot", "from": self.theta, "to": to}
        elif k == "home_z":
            if self.meas()["zd"] is None:
                self._raise("Z_VERIFY", "sharp")
                return False
            self.homed["z"] = True
        elif k == "home_th":
            if not self._rx_ret_ok() or abs(self.zc - C.ZC_SWING) > 0.25 or o.fork_cassette:
                self._raise("INTERLOCK", "R_NOT_RET" if not self._rx_ret_ok() else "Z_NOT_SWING")
                return False
            self.cur = {"kind": "home_th", "from": self.theta, "moved": 0.0}
        elif k in ("vz", "vr"):
            m = self.meas()
            got = m["zc_s"] if k == "vz" else m["rx_s"]
            want = self.zc if k == "vz" else self.rx
            if got is None or abs(got - want) > C.SHARP_TOL:
                self._raise("Z_VERIFY" if k == "vz" else "R_VERIFY",
                            f"{'-' if got is None else round(got, 1)}/{round(want, 1)}")
                return False
        elif k in ("expect", "expect_th", "ret"):
            r = self._check(act)
            if r == "fault":
                return False
            if r:
                self._on_expect_ok(act, step)
                return True
            self.cur = {"kind": k, "t0": self.t}
        elif k == "check_pick":
            s4 = self.sensors()["S4"]
            self.faults.discard("s4_false_none")
            n = self.slot
            if not s4:
                self.slot_state[n] = "E"
                self.counts["empty"] += 1
                self._log("warn", "EMPTY_SLOT", n=n)
                self._load(Q.empty_steps(n), "empty")
                return False
            self.slot_state[n] = "O"
            self.wafer_count += 1
            w = self._wafer_at("blade")
            if w:
                w["no"] = self.wafer_count
                self.wafer = w["id"]
        elif k == "capture":
            w = self._wafer_at("inspect")
            self.cap_id += 1
            self.judg = None
            self.cur = {"kind": "capture", "t0": self.t, "id": self.cap_id}
            no = w["no"] if w else self._wafer_no()
            self._log("info", "CAP", w=no, n=self.slot)
            if self.judge_mode == "internal":
                if "judge_drop" not in self.faults:
                    self.judg = self._judge_internal(self.cap_id)
            else:
                self.pending_captures.append({"id": self.cap_id, "n": self.slot, "w": no,
                                              "wafer": w["id"] if w else None, "t": round(self.t, 2)})
        elif k == "uv":
            miss = self.uv_missing()
            if miss:
                self._raise("UV_INTERLOCK", "+".join(miss))
                return False
            self.uv["lamp"], self.uv["tripped"] = True, False
            self.cur = {"kind": "uv", "el": 0.0, "auv_ok": False}
            self._log("info", "UV_ON", w=self._wafer_no())
        return True

    def uv_missing(self, m=None):
        m = m or self.meas()
        w = self._wafer(self.wafer)
        return Q.uv_interlock(self.sensors(m)["TH0"], self.theta, self._rx_ret_ok(m), self.zc, self.uv["handed"],
                              bool(w and w["judgment"] == "OK"), not self.estop_in)

    def _judge_internal(self, cid):
        if "camera_fail" in self.faults:
            return {"id": cid, "result": "ERR", "dr": 0.0, "dth": 0.0}
        ng = self.ng_next or self.rng.random() < self.ng_rate
        self.ng_next = False
        return {"id": cid, "result": "NG" if ng else "OK", "dr": -5.0 if "corr_out" in self.faults else 0.0,
                "dth": 0.0}

    def submit_judgment(self, cid, result, dr=0.0, dth=0.0):
        """PC 판정 도착(external). 지금 기다리는 촬영 id 와 같을 때만 받는다."""
        c = self.cur
        if not c or c["kind"] != "capture" or c["id"] != int(cid) or self.judg is not None:
            return False
        self.judg = {"id": int(cid), "result": str(result), "dr": float(dr), "dth": float(dth)}
        return True

    def take_capture_requests(self):
        out, self.pending_captures = self.pending_captures, []
        return out

    def _check(self, act):
        """True / False(아직) / 'fault'(즉시 알람)."""
        s = self.sensors()
        if act.kind == "expect_th":
            low = [k for k, v in zip(C.STATION_ORDER, self.th_mask()) if v]
            if len(low) >= 2:
                self._raise("TH_SENSOR_FAULT", "+".join(low))
                return "fault"
            return low == [act.sensor]
        if act.kind == "ret":
            return self._rx_ret_ok()
        v = s[act.sensor]
        return v == act.expect

    def _on_expect_ok(self, act, step):
        if act.kind == "ret" and step.group == "home":
            self.homed["r"] = True
        if act.kind == "expect" and act.sensor == "S4":
            self.faults.discard("s4_false_present")
            if not act.expect and self.uv["handed_pending"]:
                self.uv["handed"], self.uv["handed_pending"] = True, False

    def _move_toward(self, cur, to, speed, budget):
        dist = abs(to - cur)
        tneed = dist / speed if speed > 0 else 0.0
        if tneed <= budget:
            return to, tneed, True
        step = speed * budget
        return cur + (step if to > cur else -step), budget, False

    def _progress(self, budget, step):
        c = self.cur
        k = c["kind"]
        limit = budget
        timed = {a.kind for a in step.actions} & {"capture", "uv"}        # 촬영/UV 단계는 그 동작에서 멈춤
        if self.hold and self.hold["step"] == step.key and self._hold_slot_ok() \
                and k in ("z", "r", "rot", "capture", "uv") and (not timed or k in timed):
            fb = self._hold_budget(c)
            if fb is not None:
                if fb <= 1e-9:
                    self.hold = None
                    self.prev_state, self.state, self.pause_reason = self.state, "PAUSED", "HOLD"
                    return 0.0
                limit = min(limit, fb)
        done = False
        if k == "z":
            new, used, done = self._move_toward(self.zc, c["to"], C.Z_SPEED, limit)
            lo, hi = sorted((self.zc, new))
            for ev in list(c["events"]):
                if lo - 1e-9 <= ev[3] <= hi + 1e-9:
                    self._apply_transfer(ev)
                    c["events"].remove(ev)
            self.zc = new
        elif k == "r":
            new, used, done = self._move_toward(self.rx, c["to"], C.R_SPEED, limit)
            self.rx = new
        elif k == "rot":
            new, used, done = self._move_toward(self.theta, c["to"], C.THETA_SPEED, limit)
            self.theta = new
            if done:
                self.theta = C.in_frame(self.theta) if not (C.THETA_FRAME[0] <= self.theta < C.THETA_FRAME[1]) else self.theta
                if "belt_skip" in self.faults:
                    self.faults.discard("belt_skip")
                    self.theta_err += 3.0
        elif k == "home_th":
            return self._progress_home_th(c, limit)
        elif k == "capture":
            return self._progress_capture(c, limit)
        elif k == "uv":
            return self._progress_uv(c, limit)
        else:                                   # expect / expect_th / ret : 시간 경과 대기
            act = step.actions[self.ai]
            r = self._check(act)
            if r == "fault":
                return 0.0
            if r:
                self._on_expect_ok(act, step)
                used, done = 0.0, True
            else:
                used = limit
                if self.t + used - c["t0"] >= C.EXPECT_TIMEOUT_S:
                    used = max(0.0, c["t0"] + C.EXPECT_TIMEOUT_S - self.t)
                    self._expect_fail(act, step)
                    return used
        if done:
            self.cur = None
            self.ai += 1
        return used

    def _progress_home_th(self, c, limit):
        used = 0.0
        while used < limit - 1e-12:
            ds = min(0.5, C.THETA_HOME_SPEED * (limit - used), C.THETA_HOME_RANGE - c["moved"])
            if ds <= 1e-12:
                break
            self.theta -= ds
            c["moved"] += ds
            used += ds / C.THETA_HOME_SPEED
            low = [k for k, v in zip(C.STATION_ORDER, self.th_mask()) if v]
            if len(low) >= 2:
                self._raise("TH_SENSOR_FAULT", "+".join(low))
                return used
            if len(low) == 1:
                self.theta, self.theta_err = C.STATIONS[low[0]], 0.0
                self.homed["theta"] = True
                self._log("info", "TH_FOUND", low[0])
                self.cur = None
                self.ai += 1
                return used
        if c["moved"] >= C.THETA_HOME_RANGE - 1e-9:
            self.theta = C.in_frame(self.theta)
            self._raise("HOME_NO_STATION")
        return used

    def _progress_capture(self, c, limit):
        el = self.t - c["t0"]
        if self.judg is not None:
            need = max(0.0, C.CAPTURE_S - el)
            used = min(limit, need)
            if used + 1e-9 >= need:
                self._apply_judgment()
            return used
        need = C.JUDGE_TIMEOUT_S - el
        if need <= 1e-9:
            self._raise("JUDGE_TIMEOUT")
            return 0.0
        return min(limit, need)

    def _apply_judgment(self):
        j = self.judg
        w = self._wafer_at("inspect")
        self.counts["inspected"] += 1
        no = w["no"] if w else None
        self.last_judgment = {"no": no, "result": j["result"], "dr": j["dr"], "dth": j["dth"], "t": round(self.t, 2)}
        if j["result"] not in ("OK", "NG"):
            self._raise("CAMERA")
            return
        if w:                                          # 분류는 알았음(보정 창 밖이어도 웨이퍼 색은 판정대로)
            w["judgment"], w["dr"], w["dth"], w["t_inspect"] = j["result"], j["dr"], j["dth"], round(self.t, 2)
        if not Q.corr_ok(j["dr"], j["dth"]):
            self._raise("CORR_RANGE", f"{j['dr']:+.1f}/{j['dth']:+.2f}")
            return
        self._log("warn" if j["result"] == "NG" else "info", "JUDGE", j["result"], w=no, n=self.slot)
        self.cur = None
        self.ai += 1

    def _progress_uv(self, c, limit):
        el = c["el"]
        nxt = C.UV_S - el
        if not c["auv_ok"]:
            nxt = min(nxt, C.UV_AUV_S - el)
        if "uv_overtemp" in self.faults and el < 3.0:
            nxt = min(nxt, 3.0 - el)
        used = min(limit, max(0.0, nxt))
        c["el"] = el = el + used
        if not c["auv_ok"] and el >= C.UV_AUV_S - 1e-9:
            if not self.sensors()["A_UV"]:
                self._raise("UV_FAULT")
                return used
            c["auv_ok"] = True
        if "uv_overtemp" in self.faults and el >= 3.0 - 1e-9:
            self.uv["tripped"] = True
            self._raise("UV_LOST", f"{el:.1f}s")
            return used
        if el >= C.UV_S - 1e-9:
            self.uv["lamp"] = False
            w = self._wafer(self.wafer)
            if w:
                w["uv_s"] = round(el, 2)
            self._log("info", "UV_OFF", w=self._wafer_no())
            self.cur = None
            self.ai += 1
        return used

    def _hold_budget(self, c):
        f = self.hold["frac"]
        if f <= 0:
            return None
        kind = c["kind"]
        if kind == "capture":
            total, speed, done_amt = C.CAPTURE_S, 1.0, self.t - c["t0"]
        elif kind == "uv":
            total, speed, done_amt = C.UV_S, 1.0, c["el"]
        elif kind == "z":
            total, speed, done_amt = abs(c["to"] - c["from"]), C.Z_SPEED, abs(self.zc - c["from"])
        elif kind == "r":
            total, speed, done_amt = abs(c["to"] - c["from"]), C.R_SPEED, abs(self.rx - c["from"])
        else:
            total, speed, done_amt = abs(c["to"] - c["from"]), C.THETA_SPEED, abs(self.theta - c["from"])
        if total <= 0:
            return None
        return max(0.0, (f * total - done_amt) / speed)

    def _expect_fail(self, act, step):
        if act.kind == "expect_th":
            self._raise("TH_MISMATCH", act.sensor)
        elif act.kind == "ret":
            self._raise("R_NOT_RETRACTED")
        elif act.sensor == "S4":
            self._raise("S4_REMAINS" if not act.expect else "S4_MISSING")
        elif act.sensor == "TOF_IN":
            self._raise("CASS_MISSING" if act.expect else "DOCK_OCCUPIED", "input")
        elif act.sensor == "TOF_NG":
            self._raise("CASS_MISSING", "ng")
        else:
            self._raise("INTERLOCK")

    def _apply_transfer(self, ev):
        kind, where, slot, _ = ev
        if kind == "cass_pick":
            if where == "cart":
                seat = self._cart_aligned_seat()
                cid = self.cart["seats"][seat]
                self.cart["seats"][seat] = None
                self.cassettes[cid]["where"], self.cassettes[cid]["stage"] = "fork", "W"
            else:
                cid = self.dock["input"]
                self.dock["input"] = None
                self.cassettes[cid]["where"] = "fork"
            self._log("info", "CASS_PICK", where)
            return
        if kind == "cass_place":
            c = self._fork_cassette()
            if where == "cart":
                seat = self._cart_aligned_seat()
                c["where"], c["stage"] = "cart:" + seat, "D"
                self.cart["seats"][seat] = c["id"]
                self.slot_state = {}
            else:
                c["where"] = "input"
                self.dock["input"] = c["id"]
                self.slot_state = {n: "U" for n in range(1, C.SLOT_COUNT + 1)}
            self._log("info", "CASS_PLACE", where)
            return
        if kind == "pick":
            if where in ("input", "ng"):
                w = next((w for w in self._wafers_in(self.dock[where]) if w["slot"] == slot), None)
            else:
                w = self._wafer_at({"inspect": "inspect", "uv": "uvh"}[where])
            if w:
                w["loc"] = "blade"
                if where == "input":
                    w["t_pick"] = round(self.t, 2)
                if where == "uv":
                    self.uv["handed"] = False
            self._log("info", "PICK", where, n=slot if where in ("input", "ng") else None)
            return
        w = self._wafer_at("blade")
        if w:
            if where in ("input", "ng"):
                w["loc"], w["cass"], w["slot"] = "cass", self.dock[where], slot
            elif where == "uv":
                w["loc"] = "uvh"
                self.uv["handed_pending"] = True
            else:
                w["loc"] = "inspect"
        self._log("info", "PLACE", where, n=slot if where in ("input", "ng") else None)

    # ------------------------------------------------------------ 스냅샷
    def interlock(self, m=None):
        step = self.program[self.pc] if self.pc < len(self.program) else None
        mode, to = "swing", self.theta
        if step and self.ai < len(step.actions) and step.actions[self.ai].kind == "rot":
            a = step.actions[self.ai]
            mode, to = a.mode, self._target(a)
        o = self.occ()
        ok, why = Q.can_rotate(self.zc, self._rx_ret_ok(m), self.theta, to, mode, o.blade, o.fork_cassette)
        return {"rotate_ok": ok, "why": why, "mode": mode}

    def _input_view(self):
        cid = self.dock["input"]
        present = cid is not None
        slots = [{"n": n, "code": (self.slot_state.get(n, "U") if present else "-")} for n in range(1, C.SLOT_COUNT + 1)]
        seat = self.cassettes[cid]["seat"] if present else None
        return {"present": present, "slots": slots, "seat": seat}

    def _ng_view(self):
        present = self.dock["ng"] is not None
        k0 = C.SLOT_COUNT - self.ng_ptr
        slots = [{"n": n, "code": ("N" if present and n > k0 else "-")} for n in range(1, C.SLOT_COUNT + 1)]
        return {"present": present, "count": self.ng_ptr if present else None, "slots": slots}

    def snapshot(self):
        step = self.program[self.pc] if self.pc < len(self.program) else None
        cur = self.cur or {}
        target = None
        if cur.get("kind") in ("z", "r", "rot"):
            target = {"axis": {"z": "zc", "r": "rx", "rot": "theta"}[cur["kind"]], "to": round(cur["to"], 3)}
        exp = None
        if step and self.ai < len(step.actions):
            a = step.actions[self.ai]
            if a.kind == "expect":
                exp = {"sensor": a.sensor, "value": a.expect}
            elif a.kind == "check_pick":
                exp = {"sensor": "S4", "value": True}
            elif a.kind == "expect_th":
                exp = {"sensor": C.TH_SENSORS[C.STATION_ORDER.index(a.sensor)], "value": True}
        m = self.meas()
        sens = self.sensors(m)
        stage = Q.STAGE_OF_GROUP.get(step.group) if step else None
        if self.state == "WAIT_CART":
            stage = "cart"
        elif self.state == "WAIT_NG":
            stage = "ng"
        seat = self._step_seat(step) if step else None
        sv = None
        if step:
            title, detail = X.step_title(step.key, self.slot, self.ng_slot, seat)
            sv = {"key": step.key, "group": step.group, "title": title, "detail": detail, "stage": stage,
                  "action": self.ai, "actions": len(step.actions)}
        blade, insp, uvh = self._wafer_at("blade"), self._wafer_at("inspect"), self._wafer_at("uvh")
        cart = self.cart
        moving = cart["moves"][0]["code"] if cart["moves"] else None
        ucur = self.cur if (self.cur and self.cur["kind"] == "uv") else None
        miss = self.uv_missing(m) if step and step.key == "U6" else []
        snap = {
            "tick": self.tick_n, "t": round(self.t, 3), "t_src": None, "source": self.source, "state": self.state,
            "pause_reason": self.pause_reason if self.state == "PAUSED" else None, "speed": self.speed,
            "pose": {"theta": round(self.theta, 3), "zc": round(self.zc, 3), "rx": round(self.rx, 3)},
            "meas": {k: (None if v is None else round(v, 1)) for k, v in m.items()},
            "target": target,
            "homed": all(self.homed.values()), "homed_axes": dict(self.homed),
            "step": sv, "stage": stage, "phase": self.phase,
            "slot": self.slot, "ng_slot": self.ng_slot, "wafer_no": self._wafer_no(),
            "sensors": sens, "th_low": sum(1 for k in C.TH_SENSORS if sens[k]), "expect": exp,
            "docks": {"input": self._input_view(), "ng": self._ng_view()},
            "fork": {"wafer": blade is not None, "judgment": blade["judgment"] if blade else None,
                     "cassette": self._fork_cassette() is not None},
            "inspect": {"wafer": insp is not None, "judgment": insp["judgment"] if insp else None},
            "uvh": {"wafer": uvh is not None},
            "uv": {"lamp": self.uv["lamp"], "a_uv": sens["A_UV"], "total_s": C.UV_S,
                   "left_s": round(C.UV_S - ucur["el"], 1) if ucur else None, "handed": self.uv["handed"],
                   "missing": miss},
            "cart": {"state": cart["state"], "seats": {s: self.seat_stage(s) for s in C.CART_SEATS},
                     "aligned": self._cart_aligned_seat(), "caption": X.cart_text(moving) if moving else None,
                     "rot": round(cart["rot"], 2), "lat": round(cart["lat"], 2), "dist": round(cart["dist"], 2),
                     "departed": self.carts_done, "pos_known": True},
            "judgment": self.last_judgment, "ng_next": self.ng_next, "ng_rate": self.ng_rate,
            "alarm": self.alarm,
            "interlock": self.interlock(m),
            "counts": {"inspected": self.counts["inspected"], "ok": self.counts["ok"], "ng": self.counts["ng"],
                       "empty": self.counts["empty"],
                       "ng_dock": self.ng_ptr if self.dock["ng"] is not None else None,
                       "cycle_last": self.cycles[-1] if self.cycles else None,
                       "cycle_avg": round(sum(self.cycles) / len(self.cycles), 1) if self.cycles else None,
                       "carts": self.carts_done},
            "faults": sorted(self.faults),
            "operator": {"auto": self.auto_operator,
                         "pending": [{"task": t, "due_in": (round(max(0.0, d - self.t), 1) if d is not None else None)}
                                     for t, d in self.op_pending.items()]},
            "link": {"status": "SIM"},
            "log_seq": self.seq,
        }
        snap["headline"], snap["subline"] = X.headline(snap)
        return snap
