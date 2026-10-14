"""R9 웨이퍼 검사셀 시뮬레이터(순수 로직, 스레드 없음).

CellSim.tick(dt) 로 시간을 진행하고 command(name, **args) 로 조작한다.
  - 축은 등속 이동(가정 속도). 가속 무시 -> 사이클은 하한값.
  - 센서는 기하(geometry.py)에서 파생: S1 θ≈0, S2 Z≤95.5, S3 R≤0.5, S4 포크 웨이퍼.
  - 웨이퍼/카세트 이동은 포크 면이 높이를 교차하는 순간 적용, 충돌은 이동 시작 전에 구간 전체로 판정.
  - 공정 흐름 = 비디오 'R9 웨이퍼 검사 셀':
      투입(아래 슬롯부터) -> 검사 촬영 · 판정 -> OK/NG(위 슬롯부터) -> OK 5장 가득 -> 대차 호출 -> T1~T13 적재
      투입 카세트가 비면 작업자 교체, OK 도크가 비면 빈 OK 카세트, NG 가 가득 차면 빈 NG 카세트(사용자 결정).
  - 작업자 일은 '작업자 자동'(기본 ON)이면 OPERATOR_S 뒤 자동, 끄면 operator 명령(버튼)으로만 진행.
  - 대차(메카넘)는 시간 구간으로 움직이는 모형: 호출 -> 도착 -> PL 도킹, 물러남 -120, 옆걸음 ±200, 제자리 180° 회전, 출발.
문자열(한글)은 화면용. 이 모듈은 print 하지 않는다.
"""
import random

from wafer_cell import cell_config as C
from wafer_cell import geometry as G
from wafer_cell import sequence as Q
from wafer_cell import tof as T

RUN_STATES = ("HOMING", "RUNNING")
WAIT_STATES = ("WAIT_INPUT", "WAIT_OK", "WAIT_NG", "WAIT_CART")
SIM_ONLY = ("speed", "load_input", "fault", "remove_wafer", "debug_hold", "auto_operator", "next_ng", "ng_rate")
FAULTS = ("s4_false_none", "s4_false_present", "s3_fail", "belt_skip", "camera_fail", "sim_exception", "cart_delay",
          "tof_invalid")
ONE_SHOT = ("s4_false_none", "s4_false_present", "belt_skip", "sim_exception")

ALARM_TEXT = {
    "HOMING_TIMEOUT": "원점 복귀 시간 초과",
    "S3_NOT_CONFIRMED": "S3 후진 미확인 (선회/하강 금지)",
    "S4_REMAINS": "내려놓은 뒤 S4 잔존 (웨이퍼가 포크에 남음?)",
    "S4_MISSING": "다시 들기 후 S4 없음",
    "S1_MISMATCH": "θ 원점 S1 불일치 (벨트 이빨 튐 의심)",
    "S2_NOT_FOUND": "Z 원점(S2) 미확인",
    "INTERLOCK": "인터록 위반",
    "COLLISION": "충돌 (시뮬 기하 판정)",
    "ESTOP": "비상정지",
    "CAMERA": "카메라 실패",
    "SIM_EXCEPTION": "시뮬 내부 오류",
}
FATAL = ("HOMING_TIMEOUT", "COLLISION", "SIM_EXCEPTION", "S2_NOT_FOUND")
OP_TEXT = {
    "input": "작업자: 빈 투입 카세트를 빼고 새 투입 카세트 (5 장) 를 놓음",
    "ok": "작업자: 빈 OK 카세트를 OK 도크 (0°) 에 놓음",
    "ng": "작업자: 가득 찬 NG 카세트를 빼고 빈 NG 카세트를 놓음",
}
OP_BUTTON = {"input": "투입 카세트 교체", "ok": "빈 OK 카세트 놓음", "ng": "빈 NG 카세트 놓음"}


class CommandError(Exception):
    def __init__(self, code, msg):
        super().__init__(code)
        self.code, self.msg = code, msg


class CellSim:
    def __init__(self, seed=1, source="SIM"):
        self.rng = random.Random(seed)
        self.source = source
        self.t = 0.0
        self.tick_n = 0
        self.speed = 1.0
        self.theta, self.zc, self.rx = C.THETA_STANDBY, C.ZC_SWING, C.RX_RET   # 전원 투입 시 실제 자세(시뮬 진실)
        self.homed = {"theta": False, "z": False, "r": False}
        self.state = "IDLE"
        self.prev_state = None
        self.wafers = []                 # {id, no, src_slot, loc, cass, slot, judgment, t_pick, t_inspect, t_sort, capture, cycle_s}
        self.cassettes = {}              # id -> {id, kind, where}   where: 'input'|'ok'|'ng' 도크, 'fork', 'cart:FL', 'gone'
        self._wid = self._cid = 0
        self.dock = {"input": None, "ng": None, "ok": None}   # 도크 -> 카세트 id
        self.input_tried = set()         # 이 투입 카세트에서 시도한 슬롯
        self.input_empty = set()         # 운전자가 빈 슬롯으로 확정
        self.next_input_map = [True] * C.SLOT_COUNT
        self.program, self.pc, self.ai = [], 0, 0
        self.phase = None                # 'home'|'wafer'|'sort'|'empty'|'cart'|'standby'
        self.cur = None
        self.wafer = None                # 진행 중 웨이퍼 id
        self.slot = None                 # 진행 중 투입 슬롯
        self.inspected = 0
        self.ng_rate = C.NG_RATE
        self.ng_next = False
        self.auto_operator = True
        self.op_pending = {}             # task -> due time(자동) 또는 None(수동 대기)
        self.cart = self._new_cart(state="away")
        self.cart_departed = 0
        self.alarm = None
        self.prompt = None
        self.faults = set()
        self.events = []
        self.seq = 0
        self.hold = None
        self.homing_t0 = None
        self.insp_verified = True
        self.estop_in = False
        self.pending_captures = []       # 서버가 가져가 실제 카메라 저장
        self.captures = []
        self.cycles = []
        self.cycle_t0 = None
        self.last_judgment = None
        self._sort_dest = None
        self._cart_seat = None
        self.tof_rng = random.Random(seed + 7)   # 판정 난수열(rng)과 분리 -> TOF 잡음이 공정 흐름을 바꾸지 않음
        self.tof = {k: {"d": None, "kind": "none", "slot": None, "count_est": None, "blocked": False}
                    for k in C.CASSETTES}
        self._place_cassette("input", fill=True, log=False)
        self._place_cassette("ng", log=False)
        self._place_cassette("ok", log=False)
        self._tof_update()
        self._log("info", "BOOT", "시뮬레이터 시작 - 원점 복귀가 필요합니다")

    # ------------------------------------------------------------ 카세트 · 웨이퍼
    def _place_cassette(self, kind, fill=False, log=True):
        self._cid += 1
        cid = self._cid
        self.cassettes[cid] = {"id": cid, "kind": kind, "where": kind}
        self.dock[kind] = cid
        if kind == "input":
            self.input_tried, self.input_empty = set(), set()
            if fill:
                for i, present in enumerate(self.next_input_map[:C.SLOT_COUNT]):
                    if present:
                        self._wid += 1
                        self.wafers.append({"id": self._wid, "no": None, "src_slot": i + 1, "loc": "cass",
                                            "cass": cid, "slot": i + 1, "judgment": None, "t_pick": None,
                                            "t_inspect": None, "t_sort": None, "capture": None, "cycle_s": None})
        if log:
            self._log("info", "OPERATOR", OP_TEXT[kind])
        return cid

    def _wafers_in(self, cid):
        return [w for w in self.wafers if w["loc"] == "cass" and w["cass"] == cid]

    def dock_slots(self, kind):
        cid = self.dock[kind]
        return {} if cid is None else {w["slot"]: w for w in self._wafers_in(cid)}

    def dock_count(self, kind):
        return len(self.dock_slots(kind))

    def _wafer_by(self, loc):
        return next((w for w in self.wafers if w["loc"] == loc), None)

    # ------------------------------------------------------------ 하단 TOF
    def _tof_true_d(self, kind):
        """시뮬 진실에서 계산한 거리(잡음 · 고장 반영 전)."""
        if self.dock[kind] is None:
            return float(C.TOF_OUT_OF_RANGE)
        here = self.dock_slots(kind)
        if not here:
            return C.tof_empty_d()
        return C.tof_expected(min(here))

    def _tof_measure(self, kind):
        d = self._tof_true_d(kind)
        if d >= C.TOF_OUT_OF_RANGE:
            return d
        if "tof_invalid" in self.faults:
            d += C.SLOT_PITCH / 2.0                      # 슬롯 사이 값 -> 판정 불가
        return d + self.tof_rng.uniform(-C.TOF_NOISE, C.TOF_NOISE)

    def _tof_update(self):
        carrying = self._wafer_by("blade") is not None or any(c["where"] == "fork" for c in self.cassettes.values())
        for kind in C.CASSETTES:
            cur = self.tof[kind]
            if T.blocked(self.theta, self.rx, kind, carrying):
                cur["blocked"] = True                     # 포크가 빔을 가림 -> 마지막 값 유지
                continue
            d = self._tof_measure(kind)
            self.tof[kind] = dict(T.classify(d), d=d, blocked=False)

    def occ(self):
        o = G.Occupancy()
        for kind in C.CASSETTES:
            if self.dock[kind] is not None:
                o.cass[kind] = {k: True for k in self.dock_slots(kind)}
        o.blade = self._wafer_by("blade") is not None
        o.inspect = self._wafer_by("inspect") is not None
        o.fork_cassette = any(c["where"] == "fork" for c in self.cassettes.values())
        o.cart_seat_free = self._cart_ready_seat() is not None
        return o

    def sensors(self):
        o = self.occ()
        return {
            "S1": abs(G.wrap_deg(self.theta)) <= 0.5,
            "S2": self.zc <= C.ZC_HOME + 0.5,
            "S3": self.rx <= G.RX_TOL and "s3_fail" not in self.faults,
            "S4": G.s4_on(self.theta, self.zc, self.rx, o),
            "ESTOP": self.estop_in,
        }

    def _read(self, sensor):
        """Expect 용 판독(고장주입 반영)."""
        v = self.sensors()[sensor]
        if sensor == "S4" and v and "s4_false_none" in self.faults:
            return False
        if sensor == "S4" and not v and "s4_false_present" in self.faults:
            return True
        if sensor == "S1" and "belt_skip" in self.faults:
            return False
        return v

    def _log(self, level, code, msg, **params):
        self.seq += 1
        self.events.append({"seq": self.seq, "t": round(self.t, 2), "level": level,
                            "code": code, "msg": msg, **params})
        if len(self.events) > 300:
            del self.events[:len(self.events) - 300]

    def events_since(self, seq):
        return [e for e in self.events if e["seq"] > seq]

    # ------------------------------------------------------------ 대차
    def _new_cart(self, state):
        # rot 0 = 앞 받침이 셀 쪽, 180 = 뒤 받침이 셀 쪽. lat 0 = 왼 받침이 로봇 앞, -200 = 오른 받침이 로봇 앞.
        return {"state": state, "seats": {s: None for s in C.CART_SEATS}, "rot": 0.0, "lat": 0.0,
                "dist": 900.0 if state == "away" else 0.0, "moves": [], "caption": None, "nth": 0}

    def _cart_aligned_seat(self):
        cart = self.cart
        if cart["state"] != "docked" or abs(cart["dist"]) > 0.5:
            return None
        side = "F" if abs(cart["rot"]) < 0.5 else "R"
        lr = "L" if abs(cart["lat"]) < 0.5 else "R"
        return side + lr

    def _cart_ready_seat(self):
        s = self._cart_aligned_seat()
        return s if s and self.cart["seats"][s] is None else None

    def _cart_next_seat(self):
        return next((s for s in C.CART_SEATS if self.cart["seats"][s] is None), None)

    def _cart_plan(self):
        """다음 빈 받침이 로봇 앞에 오도록 이동 구간 목록을 만든다(비디오 자막 순서)."""
        cart = self.cart
        seat = self._cart_next_seat()
        kr = C.CART_SEAT_KR[seat]
        rot = 0.0 if seat[0] == "F" else 180.0
        lat = 0.0 if seat[1] == "L" else -C.CART_LATERAL
        mv = []
        slow = 3.0 if "cart_delay" in self.faults else 1.0
        if cart["state"] == "away":
            cart.update(self._new_cart("moving"))
            cart["dist"], cart["rot"], cart["lat"] = 900.0, rot, lat
            mv.append(("대차 호출 -> 셀 앞으로 (태그 ID 1 · 자세 PL)", C.CART_CALL_S * slow, {"dist": C.CART_RETREAT}))
            mv.append((f"대차 도착 -> PL 도킹 ({kr} 받침이 로봇 앞)", C.CART_DOCK_S, {"dist": 0.0}))
        else:
            cart["state"] = "moving"
            if abs(cart["rot"] - rot) > 0.5:
                mv.append(("앞 2 자리 가득 -> 대차 물러남", C.CART_RETREAT_S, {"dist": C.CART_RETREAT}))
                if abs(cart["lat"]) > 0.5:
                    mv.append((f"대차 옆걸음 +{C.CART_LATERAL:g} (제자리로)", C.CART_LATERAL_S, {"lat": 0.0}))
                mv.append(("대차 제자리 180° 회전 (뒤 받침 2 자리가 셀 쪽으로 · 이제 뒤 카메라 CM-02)",
                           C.CART_ROTATE_S, {"rot": rot}))
                if abs(lat) > 0.5:
                    mv.append((f"대차 옆걸음 {lat:+g} (메카넘)", C.CART_LATERAL_S, {"lat": lat}))
                mv.append((f"대차 다시 도착 -> PL 도킹 ({kr} 받침이 로봇 앞)", C.CART_DOCK_S, {"dist": 0.0}))
            else:
                mv.append((f"대차 물러남 -{C.CART_RETREAT:g} (옆걸음은 물러난 자리에서만)", C.CART_RETREAT_S,
                           {"dist": C.CART_RETREAT}))
                mv.append((f"대차 옆걸음 {lat - cart['lat']:+g} (메카넘 · {kr} 받침이 로봇 앞으로)", C.CART_LATERAL_S,
                           {"lat": lat}))
                mv.append((f"대차 PL 도킹 ({kr} 받침이 로봇 앞)", C.CART_DOCK_S, {"dist": 0.0}))
        cart["moves"] = [{"caption": c, "dur": d, "left": d, "to": to, "from": None} for c, d, to in mv]
        self._log("info", "CART", cart["moves"][0]["caption"])

    def _cart_leave(self):
        cart = self.cart
        cart["state"] = "leaving"
        cart["moves"] = [{"caption": f"카세트 {len(C.CART_SEATS)} 개 실음 -> 대차 출발", "dur": C.CART_LEAVE_S,
                          "left": C.CART_LEAVE_S, "to": {"dist": 900.0}, "from": None}]
        self._log("info", "CART", cart["moves"][0]["caption"])

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
                    self._log("info", "CART", cart["moves"][0]["caption"])
        if not cart["moves"]:
            if cart["state"] == "moving":
                cart["state"] = "docked"
                self._log("info", "CART_DOCKED", f"대차 도킹 완료 - {C.CART_SEAT_KR[self._cart_aligned_seat()]} 받침 대기")
            elif cart["state"] == "leaving":
                for c in self.cassettes.values():
                    if c["where"].startswith("cart:"):
                        c["where"] = "gone"
                for w in self.wafers:
                    if w["loc"] == "cass" and self.cassettes[w["cass"]]["where"] == "gone":
                        w["loc"] = "shipped"
                self.cart_departed += 1
                self.cart = self._new_cart("away")
                self._log("info", "CART_LEFT", "대차 출발 완료 - 다음 OK 카세트가 차면 새 대차 호출")

    # ------------------------------------------------------------ 작업자
    def _request_op(self, task):
        if task in self.op_pending:
            return
        self.op_pending[task] = (self.t + C.OPERATOR_S) if self.auto_operator else None
        self._log("warn", "OP_WAIT", OP_TEXT[task] + " (대기)")

    def _do_op(self, task):
        if task == "input":
            old = self.dock["input"]
            if old is not None:
                for w in self._wafers_in(old):
                    w["loc"] = "removed"
                self.cassettes[old]["where"] = "gone"
            self._place_cassette("input", fill=True)
        elif task == "ok":
            if self.dock["ok"] is not None:
                raise CommandError("bad_state", "OK 도크에 이미 카세트가 있음")
            self._place_cassette("ok")
        elif task == "ng":
            old = self.dock["ng"]
            if old is not None:
                for w in self._wafers_in(old):
                    w["loc"] = "ng_out"
                self.cassettes[old]["where"] = "gone"
            self._place_cassette("ng")
        self.op_pending.pop(task, None)
        if task in ("ok", "ng"):
            r = T.classify(self._tof_measure(task))
            if r["kind"] != "empty":
                self._log("warn", "TOF_MISMATCH", f"{task.upper()} 도크: 빈 카세트를 놓았는데 TOF 가 '{r['kind']}' "
                                                   f"(슬롯 {r['slot']}) - 카세트 확인")
        self._tof_update()

    def _op_env(self):
        for task, due in list(self.op_pending.items()):
            if due is not None and self.t >= due:
                self._do_op(task)

    # ------------------------------------------------------------ 알람
    def _raise(self, code, detail=""):
        text = ALARM_TEXT.get(code, code) + (f": {detail}" if detail else "")
        step = self.program[self.pc].key if self.pc < len(self.program) else None
        self.alarm = {"code": code, "msg": text, "step": step, "slot": self.slot,
                      "fatal": code in FATAL, "t": round(self.t, 2)}
        self.state = "ALARM"
        self._log("alarm", code, text, step=step, slot=self.slot)
        for f in ONE_SHOT:
            self.faults.discard(f)
        if code in FATAL:
            self._abort(lose_home=True)

    def _abort(self, lose_home):
        if self._wafer_by("inspect") is not None or (self.phase == "wafer" and self.pc >= 4):
            self.insp_verified = False       # 검사 크래들에 웨이퍼가 남았을 수 있음(센서 없음)
        self.program, self.pc, self.ai, self.cur, self.phase = [], 0, 0, None, None
        self.hold = None
        if lose_home:
            self.homed = {k: False for k in self.homed}

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

    def _cmd_home(self):
        self._need("IDLE", "READY")
        if self.estop_in:
            raise CommandError("estop", "E-STOP 해제 필요")
        self._load(Q.homing_steps(), "home")
        self.state, self.homing_t0, self.slot = "HOMING", self.t, None
        self._log("info", "HOME", "원점 복귀 시작 (R -> Z -> Z105 -> θ1 -> 대기 120°)")

    def _cmd_start(self):
        self._need("READY")
        if not all(self.homed.values()):
            raise CommandError("not_homed", "원점 복귀 먼저")
        if self.occ().blade or self.sensors()["S4"]:
            raise CommandError("blade_wafer", "포크 위 웨이퍼 - 수동 제거 후 시작")
        if not self.insp_verified:
            raise CommandError("need_insp_confirm", "검사 크래들(120°)이 비었는지 확인 필요 (센서 없음)")
        self.state = "RUNNING"
        self._log("info", "START", "공정 시작")
        self._schedule()

    def _cmd_pause(self):
        self._need(*RUN_STATES, *WAIT_STATES)
        self.prev_state, self.state = self.state, "PAUSED"
        self._log("info", "PAUSE", "일시정지")

    def _cmd_resume(self):
        self._need("PAUSED")
        if self.estop_in:
            raise CommandError("estop", "E-STOP 해제 필요")
        self.state = self.prev_state or "RUNNING"
        if self.cur and self.cur["kind"] in ("expect", "check_pick"):
            self.cur["t0"] = self.t
        self._log("info", "RESUME", "재개")

    def _cmd_stop(self):
        self._need(*RUN_STATES, *WAIT_STATES, "PAUSED", "WAIT_OPERATOR")
        self._abort(lose_home=True)
        self.state, self.prompt, self.slot, self.wafer = "IDLE", None, None, None
        self._log("warn", "STOP", "소프트 정지 - 원점 복귀 필요 (E-STOP 아님)")

    def _cmd_estop(self):
        self.estop_in = True
        self._abort(lose_home=True)
        self.prompt, self.slot, self.wafer = None, None, None
        self.state = "ESTOP"
        self.alarm = {"code": "ESTOP", "msg": ALARM_TEXT["ESTOP"], "step": None, "slot": None,
                      "fatal": True, "t": round(self.t, 2)}
        self._log("alarm", "ESTOP", "비상정지 (시뮬) - 해제 후 원점 복귀")

    def _cmd_reset(self):
        self._need("ALARM", "ESTOP")
        if self.state == "ESTOP":
            self.estop_in = False
            self.state, self.alarm = "IDLE", None
            self._log("info", "RESET", "E-STOP 해제 - 원점 복귀 필요")
            return
        fatal = self.alarm and self.alarm.get("fatal")
        self.alarm = None
        if fatal or not self.program:
            self.state = "IDLE" if not all(self.homed.values()) else "READY"
        else:
            self.prev_state, self.state = "RUNNING", "PAUSED"
            if self.cur:
                self.cur["t0"] = self.t
        self._log("info", "RESET", "알람 해제")

    def _cmd_confirm(self, choice):
        if choice == "insp_clear":
            self._need("IDLE", "READY")
            self.insp_verified = True
            self._log("info", "INSP", "운전자 확인: 검사 크래들 비어 있음")
            return
        self._need("WAIT_OPERATOR")
        p, self.prompt = self.prompt, None
        n = p["slot"]
        if choice == "empty":
            self.input_empty.add(n)
            self._load(Q.empty_slot_steps(), "empty")
            self._log("warn", "EMPTY", f"투입 슬롯 {n} 빈 슬롯으로 처리 (운전자 확인)")
        elif choice == "retry":
            self._load(Q.retry_steps(n), "wafer")
            self._log("info", "RETRY", f"투입 슬롯 {n} 집기 재시도")
        else:
            self.prompt = p
            raise CommandError("bad_arg", "choice = empty | retry | insp_clear")
        self.state = "RUNNING"

    def _cmd_operator(self, task):
        """작업자가 교체/놓기를 끝냈다(버튼). 실장비에서도 쓰는 확인 명령."""
        if task not in OP_TEXT:
            raise CommandError("bad_arg", "task = input | ok | ng")
        if self.state in RUN_STATES and self.phase not in (None, "standby") and task == self._busy_dock():
            raise CommandError("bad_state", "로봇이 그 카세트에서 작업 중")
        self._do_op(task)

    def _busy_dock(self):
        if self.phase in ("wafer", "empty") and self.pc <= 2:
            return "input"
        if self.phase == "cart":
            return "ok"
        if self.phase == "sort":
            return self._sort_dest
        return None

    def _cmd_auto_operator(self, on=True):
        self.auto_operator = bool(on)
        for task in self.op_pending:
            self.op_pending[task] = (self.t + C.OPERATOR_S) if self.auto_operator else None
        self._log("info", "OPERATOR", f"(시뮬) 작업자 자동 {'ON' if on else 'OFF'}")

    def _cmd_next_ng(self, on=True):
        self.ng_next = bool(on)
        self._log("info", "NG_NEXT", f"(시뮬) 다음 웨이퍼 NG {'지정' if on else '해제'}")

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

    def _cmd_load_input(self, slots):
        """(시뮬) 다음에 놓을 투입 카세트의 웨이퍼 배치. IDLE/READY 면 지금 카세트도 바꾼다."""
        self.next_input_map = [bool(x) for x in slots][:C.SLOT_COUNT]
        if self.state in ("IDLE", "READY"):
            self._do_op("input")

    def _cmd_fault(self, name, on=True):
        if name not in FAULTS:
            raise CommandError("bad_arg", f"알 수 없는 고장 {name}")
        if on:
            self.faults.add(name)
        else:
            self.faults.discard(name)
        self._log("warn" if on else "info", "FAULT", f"(시뮬) 고장주입 {name} {'ON' if on else 'OFF'}")
        if name == "tof_invalid":
            self._tof_update()
    def _cmd_remove_wafer(self, loc):
        self._need("IDLE", "READY", "ALARM")
        w = self._wafer_by(loc)
        if w is None:
            raise CommandError("bad_arg", f"{loc} 에 웨이퍼 없음")
        w["loc"] = "removed"
        self._log("info", "REMOVE", f"(시뮬) 운전자가 {loc} 웨이퍼 수동 제거")

    def _cmd_debug_hold(self, step, frac=0.0, slot=None):
        self.hold = {"step": str(step), "frac": float(frac), "slot": slot}

    # ------------------------------------------------------------ 스케줄
    def _load(self, steps, phase):
        self.program, self.pc, self.ai, self.cur, self.phase = list(steps), 0, 0, None, phase

    def _at_standby(self):
        return (abs(self.theta - C.THETA_STANDBY) < 0.5 and abs(self.zc - C.ZC_SWING) < 0.3
                and self.rx <= G.RX_TOL)

    def _wait(self, state, task=None):
        """대기 상태로. 로봇이 대기 자세가 아니면 먼저 120° 로 돌아간다."""
        if task:
            self._request_op(task)
        if not self._at_standby():
            self._load([Q.Step("W", "home", f"대기 ({C.THETA_STANDBY:g}°) 로 복귀", "Z 105 · R 0",
                               (Q.MoveR(C.RX_RET), Q.Expect("S3", True), Q.MoveZ(C.ZC_SWING),
                                Q.Rotate(C.THETA_STANDBY)))], "standby")
            self.state = "RUNNING"
            return
        if self.state != state:
            self._log("info", state, {"WAIT_INPUT": "투입 카세트 교체 대기", "WAIT_OK": "빈 OK 카세트 대기",
                                       "WAIT_NG": "빈 NG 카세트 대기", "WAIT_CART": "대차 대기"}[state])
        self.state = state

    def _next_input_slot(self):
        """다음에 집을 투입 슬롯. 아래부터 꺼내므로 하단 TOF 가 본 '가장 아래 웨이퍼'가 곧 다음 슬롯이다.
        TOF 가 빈 카세트면 None(교체 대기), 판정 불가/가림이면 예전처럼 가장 아래 미시도 슬롯을 S4 로 확인."""
        if self.dock["input"] is None:
            return None
        untried = [k for k in range(1, C.SLOT_COUNT + 1) if k not in self.input_tried and k not in self.input_empty]
        t = self.tof["input"]
        if not t["blocked"]:
            if t["kind"] == "empty":
                return None
            if t["kind"] == "slot":
                cand = [k for k in untried if k >= t["slot"]]    # 그 아래는 빔이 지나감 = 빈칸 확인
                return cand[0] if cand else None
        return untried[0] if untried else None

    def _schedule(self):
        """프로그램이 끝났거나 대기 중일 때 다음 일을 고른다(비디오 흐름 순서)."""
        ok_n = self.dock_count("ok") if self.dock["ok"] is not None else None
        # 1) OK 카세트 가득 -> 대차
        if ok_n is not None and ok_n >= C.SLOT_COUNT:
            seat = self._cart_ready_seat()
            if seat is not None and self.cart["state"] == "docked":
                self.cart["nth"] += 1
                self._cart_seat = seat
                self._load(Q.cart_steps(seat, self.cart["nth"]), "cart")
                self.state = "RUNNING"
                self._log("info", "CART_LOAD", f"OK 카세트 -> 대차 {C.CART_SEAT_KR[seat]} 받침 ({self.cart['nth']} 번째)")
                return
            if self.cart["state"] in ("away", "docked") and not self.cart["moves"]:
                if self.cart["state"] == "away":
                    self._log("info", "CART_CALL", "OK 카세트 5 장 가득 -> 대차 호출")
                self._cart_plan()
            self._wait("WAIT_CART")
            return
        # 2) OK 도크 비었음 -> 빈 OK 카세트
        if ok_n is None:
            self._wait("WAIT_OK", "ok")
            return
        # 3) NG 가득 -> 빈 NG 카세트(사용자 결정: 멈추고 교체 요청)
        if self.dock["ng"] is None or self.dock_count("ng") >= C.SLOT_COUNT:
            self._wait("WAIT_NG", "ng")
            return
        # 4) 다음 투입 슬롯
        k = self._next_input_slot()
        if k is None:
            self._wait("WAIT_INPUT", "input")
            return
        self.slot = k
        self.input_tried.add(k)
        self._load(Q.wafer_steps(k), "wafer")
        self.state = "RUNNING"
        self.cycle_t0 = self.t

    def _program_done(self):
        phase = self.phase
        if self.state == "HOMING":
            self._load([], None)
            self.homing_t0 = None
            self.state = "READY"
            self._log("info", "HOMED", f"원점 복귀 완료 - 대기 θ{C.THETA_STANDBY:g} · Z 105 · R 0")
            return
        if phase == "wafer":
            w = self._wafer_by("blade")
            dest = "ng" if w and w["judgment"] == "NG" else "ok"
            slots = self.dock_slots(dest)
            k = next((k for k in range(C.SLOT_COUNT, 0, -1) if k not in slots), None)   # 위 슬롯부터
            if k is None:            # 검사 전에 확인하므로 정상 흐름에선 없음
                self._raise("INTERLOCK", f"{dest.upper()} 카세트 가득 - 넣을 슬롯 없음")
                return
            self._sort_dest = dest
            self._load(Q.sort_steps(dest, k), "sort")
            return
        if phase == "sort":
            dt = self.t - (self.cycle_t0 or self.t)
            self.cycles.append(round(dt, 2))
            w = next((w for w in self.wafers if w["id"] == self.wafer), None)
            if w:
                w["cycle_s"] = round(dt, 2)
            self._log("info", "CYCLE", f"웨이퍼 {w['no'] if w else '?'} 사이클 {dt:.1f}s (가정 속도 기준)")
            self.wafer, self.slot = None, None
        if phase == "cart":
            if all(v is not None for v in self.cart["seats"].values()):
                self._cart_leave()
        self._load([], None)
        self._schedule()

    def _wait_env(self):
        """대기 상태에서 조건이 풀렸는지 본다."""
        if self.state in WAIT_STATES:
            self._schedule()

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
        """환경: 시간, 대차 이동, 작업자, 원점 타임아웃, 대기 해제."""
        self.t += dt
        self._cart_env(dt)
        self._op_env()
        self._tof_update()
        if self.state == "HOMING" and self.homing_t0 is not None and self.t - self.homing_t0 > C.HOMING_TIMEOUT_S:
            self._raise("HOMING_TIMEOUT", f"{C.HOMING_TIMEOUT_S:.0f}s")
        self._wait_env()

    def _run(self, budget):
        """현재 동작을 최대 budget 초 진행. 사용 시간 반환(순간 동작이면 0)."""
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
                self.prev_state, self.state = self.state, "PAUSED"
                return 0.0
            if self.ai == 0:
                self._log("info", "STEP", f"[{step.key}] {step.title}", step=step.key, slot=self.slot)
            if not self._begin(act, step):
                return 0.0
            if self.cur is None:
                self.ai += 1
                return 0.0
        return self._progress(budget, step)

    def _hold_slot_ok(self):
        return self.hold.get("slot") in (None, self.slot)

    def _begin(self, act, step):
        o = self.occ()
        k = act.kind
        if k == "z":
            r = G.z_move(self.theta, self.rx, self.zc, act.value, o)
            if r["collision"]:
                self._raise("COLLISION", r["collision"])
                return False
            ev = []
            for kind, where, slot in r["events"]:
                if kind == "cass_pick":
                    z_cross = C.CART_LIFT_Z
                elif kind == "cass_place":
                    z_cross = C.CART_SEAT_LIFT_Z
                elif where == "inspect":
                    z_cross = C.CRADLE_SEAT_Z - C.BLADE_PLANE_OFS
                else:
                    z_cross = C.slot_z(slot) - C.BLADE_PLANE_OFS
                ev.append((z_cross, kind, where, slot))
            self.cur = {"kind": "z", "from": self.zc, "to": act.value, "events": ev}
        elif k == "r":
            col = G.r_move(self.theta, self.zc, self.rx, act.value, o)
            if col:
                self._raise("COLLISION", col)
                return False
            self.cur = {"kind": "r", "from": self.rx, "to": act.value}
        elif k == "rot":
            ok, why = Q.can_rotate(self.zc, self.rx, self._read("S3"), z_ok=act.z_ok, carrying=o.blade)
            if not ok:
                self._raise("INTERLOCK", why)
                return False
            if not C.THETA_MIN - 0.5 <= act.value <= C.THETA_MAX + 0.5:
                self._raise("INTERLOCK", f"θ {act.value:g} 범위 밖")
                return False
            pts = list(act.via) + [act.value]
            self.cur = {"kind": "rot", "from": self.theta, "pts": pts, "to": act.value}
        elif k in ("expect", "check_pick"):
            if self._read(act.sensor) == act.expect:
                self._on_expect_ok(act, step)
                return True
            if k == "check_pick":
                n = self.slot
                self.prompt = {"kind": "s4_missing", "slot": n, "options": ["empty", "retry"],
                               "msg": f"투입 슬롯 {n}: S4 없음 - 빈 슬롯인지 집기 실패인지 구분 불가. 눈으로 확인 후 선택"}
                self.state = "WAIT_OPERATOR"
                self._log("alarm", "S4_NONE", self.prompt["msg"], slot=n)
                self.faults.discard("s4_false_none")
                return False
            self.cur = {"kind": k, "t0": self.t, "sensor": act.sensor, "expect": act.expect}
        elif k == "capture":
            self.cur = {"kind": "capture", "left": C.CAPTURE_S}
            w = self._wafer_by("inspect")
            if w:
                self.inspected += 1
                w["no"] = w["no"] or self.inspected
                w["t_inspect"] = round(self.t, 2)
                ng = self.ng_next or self.rng.random() < self.ng_rate
                self.ng_next = False
                w["judgment"] = "NG" if ng else "OK"
                self.last_judgment = {"no": w["no"], "result": w["judgment"], "t": round(self.t, 2)}
                self.pending_captures.append({"slot": self.slot, "wafer": w["id"], "no": w["no"],
                                              "judgment": w["judgment"], "t": round(self.t, 2)})
                self._log("info" if not ng else "warn", "JUDGE",
                          f"웨이퍼 {w['no']} StreamCam 촬영 · 판정 {w['judgment']}", slot=self.slot)
        return True

    def _on_expect_ok(self, act, step):
        if act.sensor == "S2" and step.group == "home":
            self.homed["z"] = True
        if act.sensor == "S3" and step.group == "home":
            self.homed["r"] = True
        if act.sensor == "S1" and step.group == "home":
            self.homed["theta"] = True

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
        if self.hold and self.hold["step"] == step.key and self._hold_slot_ok() and k in ("z", "r", "rot", "capture"):
            frac_budget = self._hold_budget(c)
            if frac_budget is not None:
                if frac_budget <= 1e-9:
                    self.hold = None
                    self.prev_state, self.state = self.state, "PAUSED"
                    return 0.0
                limit = min(limit, frac_budget)
        if k == "z":
            new, used, done = self._move_toward(self.zc, c["to"], C.Z_SPEED, limit)
            lo, hi = sorted((self.zc, new))
            for ev in list(c["events"]):
                if lo - 1e-9 <= ev[0] <= hi + 1e-9:
                    self._apply_transfer(ev)
                    c["events"].remove(ev)
            self.zc = new
        elif k == "r":
            new, used, done = self._move_toward(self.rx, c["to"], C.R_SPEED, limit)
            self.rx = new
        elif k == "rot":
            new, used, done_pt = self._move_toward(self.theta, c["pts"][0], C.THETA_SPEED, limit)
            self.theta = new
            done = False
            if done_pt:
                c["pts"].pop(0)
                done = not c["pts"]
        elif k == "capture":
            used = min(limit, c["left"])
            c["left"] -= used
            done = c["left"] <= 1e-9
        else:   # expect : 시간 경과 대기
            used = limit
            act = step.actions[self.ai]
            if self._read(act.sensor) == act.expect:
                self._on_expect_ok(act, step)
                used, done = 0.0, True
            else:
                to = C.EXPECT_TIMEOUT_S
                if self.t + used - c["t0"] >= to:
                    used = max(0.0, c["t0"] + to - self.t)
                    self._expect_fail(act, step)
                    return used
                done = False
        if done:
            self.cur = None
            self.ai += 1
        return used

    def _hold_budget(self, c):
        """debug_hold: 목표 비율까지 남은 시간(초). 해당 없으면 None."""
        f = self.hold["frac"]
        if f <= 0:
            return None
        if c["kind"] == "capture":
            total, speed, done_amt = C.CAPTURE_S, 1.0, C.CAPTURE_S - c["left"]
        elif c["kind"] == "z":
            total, speed, done_amt = abs(c["to"] - c["from"]), C.Z_SPEED, abs(self.zc - c["from"])
        elif c["kind"] == "r":
            total, speed, done_amt = abs(c["to"] - c["from"]), C.R_SPEED, abs(self.rx - c["from"])
        else:
            total, speed, done_amt = abs(c["to"] - c["from"]), C.THETA_SPEED, abs(self.theta - c["from"])
        if total <= 0:
            return None
        return max(0.0, (f * total - done_amt) / speed)

    def _expect_fail(self, act, step):
        s, v = act.sensor, act.expect
        if s == "S3":
            code = "S3_NOT_CONFIRMED"
        elif s == "S4" and not v:
            code = "S4_REMAINS"
        elif s == "S4":
            code = "S4_MISSING"
        elif s == "S1":
            code = "HOMING_TIMEOUT" if step.group == "home" else "S1_MISMATCH"
        elif s == "S2":
            code = "S2_NOT_FOUND"
        else:
            code = "INTERLOCK"
        self._raise(code, f"단계 {step.key}")

    def _apply_transfer(self, ev):
        _, kind, where, slot = ev
        if kind == "cass_pick":
            cid = self.dock["ok"]
            if cid is not None:
                self.cassettes[cid]["where"] = "fork"
                self.dock["ok"] = None
                self._log("info", "CASS_PICK", "OK 카세트 (웨이퍼 5 장) 를 클램프 윗판째로 듦")
            return
        if kind == "cass_place":
            cid = next((c["id"] for c in self.cassettes.values() if c["where"] == "fork"), None)
            seat = self._cart_ready_seat()
            if cid is not None and seat is not None:
                self.cassettes[cid]["where"] = "cart:" + seat
                self.cart["seats"][seat] = cid
                self._log("info", "CASS_PLACE", f"OK 카세트 -> 대차 {C.CART_SEAT_KR[seat]} 받침")
            return
        if kind == "pick":
            if where == "input":
                w = next((w for w in self._wafers_in(self.dock["input"]) if w["slot"] == slot), None)
            else:
                w = self._wafer_by("inspect")
            if w:
                w["loc"] = "blade"
                if where == "input":
                    w["t_pick"] = round(self.t, 2)
                    self.wafer = w["id"]
                self._log("info", "PICK", f"집기: {'투입 슬롯 ' + str(slot) if where == 'input' else '검사 크래들'} -> 포크")
        else:
            w = self._wafer_by("blade")
            if w:
                if where == "inspect":
                    w["loc"] = "inspect"
                else:
                    w["loc"], w["cass"], w["slot"] = "cass", self.dock[where], slot
                    w["t_sort"] = round(self.t, 2)
                self._log("info", "PLACE", f"놓기: 포크 -> {'검사 크래들' if where == 'inspect' else where.upper() + ' 슬롯 ' + str(slot)}")

    # ------------------------------------------------------------ 스냅샷
    def interlock(self):
        step = self.program[self.pc] if self.pc < len(self.program) else None
        z_ok = C.ZC_SWING
        if step and self.ai < len(step.actions) and step.actions[self.ai].kind == "rot":
            z_ok = step.actions[self.ai].z_ok
        ok, why = Q.can_rotate(self.zc, self.rx, self.sensors()["S3"], z_ok=z_ok, carrying=self.occ().blade)
        return {"rotate_ok": ok, "reason": why}

    def _dock_view(self, kind):
        cid = self.dock[kind]
        if cid is None:
            t = self.tof[kind]
            return {"present": False, "count": 0, "slots": [{"n": k, "status": "none"} for k in range(1, C.SLOT_COUNT + 1)],
                    "tof": {"d": None if t["d"] is None else round(t["d"], 1), "kind": t["kind"], "slot": t["slot"],
                            "count_est": t["count_est"], "blocked": t["blocked"]}}
        here = self.dock_slots(kind)
        t = self.tof[kind]
        out = []
        for k in range(1, C.SLOT_COUNT + 1):
            w = here.get(k)
            if kind == "input":
                # 투입은 기계가 아는 만큼만: 가장 아래 웨이퍼(TOF 가 봄) = wafer, 그 위 = est(추정), 그 아래 = empty(빔 통과)
                if k in self.input_empty:
                    status = "empty_conf"
                elif k in self.input_tried and not w:
                    status = "done"
                elif t["kind"] == "slot":
                    status = "wafer" if k == t["slot"] else ("est" if k > t["slot"] else "empty")
                elif t["kind"] == "empty":
                    status = "empty"
                else:
                    status = "est"                         # TOF 판정 불가 -> 위치 모름
            else:
                status = ("ng" if kind == "ng" else "ok") if w else "empty"
            out.append({"n": k, "status": status, "no": w["no"] if w else None})
        count = len(here)
        if kind == "input":
            count = sum(1 for x in out if x["status"] in ("wafer", "est"))
        return {"present": True, "count": count, "slots": out,
                "tof": {"d": None if t["d"] is None else round(t["d"], 1), "kind": t["kind"], "slot": t["slot"],
                        "count_est": t["count_est"], "blocked": t["blocked"]}}

    def headline(self):
        """기본 화면 한 줄(비디오 큰 자막) + 둘째 줄(수치, 상세용)."""
        st = self.state
        step = self.program[self.pc] if self.pc < len(self.program) else None
        if st in ("ALARM", "ESTOP") and self.alarm:
            return self.alarm["msg"], ""
        if st == "WAIT_OPERATOR" and self.prompt:
            return f"투입 슬롯 {self.prompt['slot']} 확인 필요", self.prompt["msg"]
        if st in ("WAIT_INPUT", "WAIT_OK", "WAIT_NG"):
            task = {"WAIT_INPUT": "input", "WAIT_OK": "ok", "WAIT_NG": "ng"}[st]
            return OP_TEXT[task], f"로봇은 대기 ({C.THETA_STANDBY:g}°)"
        if st == "WAIT_CART" or (self.cart["moves"] and st in ("PAUSED",) and self.prev_state == "WAIT_CART"):
            cap = self.cart["moves"][0]["caption"] if self.cart["moves"] else "대차 대기"
            return cap, "대차가 움직이는 동안 로봇은 대기"
        if step and self.phase in ("wafer", "sort", "empty"):
            w = next((w for w in self.wafers if w["id"] == self.wafer), None)
            no = w["no"] if w and w["no"] else self.inspected + 1
            title = step.title
            if step.key == "6" and w and w["judgment"]:
                title += f" {w['judgment']}"
            return f"웨이퍼 {no} — {title}", step.detail
        if step:
            return step.title, step.detail
        if st == "READY":
            return f"대기 (θ{C.THETA_STANDBY:g}° · Z 105 · R 0)", "'시작' 을 누르면 투입 카세트부터 검사한다"
        if st == "IDLE":
            return "원점 복귀 필요", "R 수축 -> Z 최하단 -> Z 105 -> θ1 원점 -> 대기 120°"
        return "", ""

    def snapshot(self):
        step = self.program[self.pc] if self.pc < len(self.program) else None
        cur = self.cur or {}
        target = None
        if cur.get("kind") == "z":
            target = {"axis": "zc", "to": cur["to"]}
        elif cur.get("kind") == "r":
            target = {"axis": "rx", "to": cur["to"]}
        elif cur.get("kind") == "rot":
            target = {"axis": "theta", "to": cur["to"]}
        exp = None
        if step and self.ai < len(step.actions):
            a = step.actions[self.ai]
            if a.kind in ("expect", "check_pick"):
                exp = {"sensor": a.sensor, "value": a.expect}
        o = self.occ()
        stage = None
        if step:
            stage = Q.STAGE_OF_GROUP.get(step.group)
            if step.group == "sort":
                stage = self._sort_dest
        if self.state == "WAIT_CART":
            stage = "cart"
        fork_cass = next((c for c in self.cassettes.values() if c["where"] == "fork"), None)
        insp = self._wafer_by("inspect")
        blade = self._wafer_by("blade")
        head, sub = self.headline()
        docks = {k: self._dock_view(k) for k in C.CASSETTES}
        cart = self.cart
        seats = {s: (len(self._wafers_in(cid)) if cid is not None else None) for s, cid in cart["seats"].items()}
        return {
            "tick": self.tick_n, "t": round(self.t, 3), "source": self.source, "state": self.state,
            "speed": self.speed,
            "pose": {"theta": round(self.theta, 3), "zc": round(self.zc, 3), "rx": round(self.rx, 3)},
            "target": target,
            "homed": all(self.homed.values()), "homed_axes": dict(self.homed),
            "step": ({"key": step.key, "group": step.group, "title": step.title, "detail": step.detail,
                      "stage": stage, "action": self.ai, "actions": len(step.actions)} if step else None),
            "program": [s.key for s in self.program], "phase": self.phase, "stage": stage,
            "headline": head, "subline": sub,
            "slot": self.slot,
            "sensors": self.sensors(), "expect": exp,
            "docks": docks,
            "fork": {"wafer": blade is not None, "judgment": blade["judgment"] if blade else None,
                     "cassette": len(self._wafers_in(fork_cass["id"])) if fork_cass else None},
            "inspect": {"wafer": insp is not None, "judgment": insp["judgment"] if insp else None},
            "cart": {"state": cart["state"], "seats": seats, "rot": round(cart["rot"], 2),
                     "lat": round(cart["lat"], 2), "dist": round(cart["dist"], 2),
                     "aligned": self._cart_aligned_seat(), "caption": cart["moves"][0]["caption"] if cart["moves"] else None,
                     "loaded": sum(1 for v in cart["seats"].values() if v is not None), "departed": self.cart_departed,
                     "camera": "CM-02" if abs(cart["rot"] - 180.0) < 90 else "CM-01"},
            "operator": {"auto": self.auto_operator,
                         "pending": [{"task": t, "label": OP_BUTTON[t], "msg": OP_TEXT[t],
                                      "due_in": (round(max(0.0, d - self.t), 1) if d is not None else None)}
                                     for t, d in self.op_pending.items()]},
            "judgment": self.last_judgment, "ng_next": self.ng_next, "ng_rate": self.ng_rate,
            "occ": {"blade": o.blade, "inspect": o.inspect, "fork_cassette": o.fork_cassette},
            "wafers": [{"id": w["id"], "no": w["no"], "loc": w["loc"], "judgment": w["judgment"]}
                       for w in self.wafers if w["loc"] in ("blade", "inspect")],
            "alarm": self.alarm, "prompt": self.prompt,
            "interlock": self.interlock(),
            "counts": {"inspected": self.inspected,
                       "ok": sum(1 for w in self.wafers if w["judgment"] == "OK" and w["t_sort"] is not None),
                       "ng": sum(1 for w in self.wafers if w["judgment"] == "NG" and w["t_sort"] is not None),
                       "input": docks["input"]["count"],          # 투입은 TOF 기준(가장 아래 + 위 추정)
                       "ok_dock": self.dock_count("ok") if self.dock["ok"] is not None else None,
                       "ng_dock": self.dock_count("ng") if self.dock["ng"] is not None else None,
                       "cart": sum(1 for v in cart["seats"].values() if v is not None),
                       "cycle_last": self.cycles[-1] if self.cycles else None,
                       "cycle_avg": round(sum(self.cycles) / len(self.cycles), 1) if self.cycles else None},
            "faults": sorted(self.faults), "insp_verified": self.insp_verified,
            "captures": self.captures[-6:],
            "log_seq": self.seq,
        }
