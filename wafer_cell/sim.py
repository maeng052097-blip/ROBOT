"""CELL_V9 공정 시뮬레이터(순수 로직, 스레드 없음).

CellSim.tick(dt) 로 시간을 진행하고 command(name, **args) 로 조작한다.
  - 축은 등속 이동(가정 속도). 가속 무시 -> 사이클은 하한값.
  - 센서는 대본이 아니라 기하(geometry.py)에서 파생: S1 θ≈0, S2 ZC≤95.5, S3 RX≤0.5, S4 기하, S5 반출 점유.
  - 웨이퍼 이동(픽업/안착)은 블레이드 면이 높이를 교차하는 순간 적용, 충돌은 이동 시작 전에 구간 전체로 판정.
  - dt 잔여분은 동작 경계를 넘어 이월 -> 배속/틱 간격이 달라도 사이클 시간이 같다.
문자열(한글)은 화면용. 이 모듈은 print 하지 않는다.
"""
import random

from wafer_cell import cell_config as C
from wafer_cell import geometry as G
from wafer_cell import sequence as Q
from wafer_cell import tof as T

RUN_STATES = ("HOMING", "RUNNING", "WAIT_S5")
SIM_ONLY = ("speed", "load_map", "fault", "remove_wafer", "debug_hold")
FAULTS = ("s4_false_none", "s4_false_present", "s3_fail", "belt_skip", "next_robot_delay",
          "camera_fail", "sim_exception", "tof_ledge_in_beam")
ONE_SHOT = ("s4_false_none", "s4_false_present", "belt_skip", "sim_exception")

ALARM_TEXT = {
    "HOMING_TIMEOUT": "원점 복귀 시간 초과",
    "S3_NOT_CONFIRMED": "S3 후진 미확인 (선회/하강 금지)",
    "S4_REMAINS": "안착 후 S4 잔존 (웨이퍼가 블레이드에 남음?)",
    "S4_MISSING": "재픽업 후 S4 없음",
    "S1_MISMATCH": "0° 복귀 시 S1 불일치 (벨트 이빨 튐 의심)",
    "S2_NOT_FOUND": "Z 원점(S2) 미확인",
    "INTERLOCK": "인터록 위반",
    "COLLISION": "충돌 (시뮬 기하 판정)",
    "ESTOP": "비상정지",
    "CAMERA": "카메라 실패",
    "SIM_EXCEPTION": "시뮬 내부 오류",
}
FATAL = ("HOMING_TIMEOUT", "COLLISION", "SIM_EXCEPTION", "S2_NOT_FOUND")


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
        self.theta, self.zc, self.rx = 0.0, C.ZC_HOME, C.RX_RET     # 전원 투입 시 실제 자세(시뮬 진실)
        self.homed = {"theta": False, "z": False, "r": False}
        self.state = "IDLE"
        self.prev_state = None
        self.wafers = []                 # {id, slot, loc, t_pick, t_inspect, t_out, t_handoff, capture, cycle_s}
        self._wid = 0
        self.slot_conf = {}              # n -> 'present' | 'empty'  (S4 로 확정)
        self.cassette = {"ready": False, "n_est": None, "kind": None, "d": None, "manual": False}
        self.program, self.pc, self.ai = [], 0, 0
        self.cur = None                  # 진행 중 동작
        self.slot = None                 # 진행 중 슬롯
        self.queue = []
        self.alarm = None
        self.prompt = None
        self.faults = set()
        self.events = []
        self.seq = 0
        self.hold = None
        self.handoff_left = None
        self.homing_t0 = None
        self.c90_verified = True
        self.estop_in = False
        self.pending_captures = []       # 서버가 가져가 실제 카메라 저장
        self.captures = []
        self.cycles = []
        self.cycle_t0 = None
        self.tof_d = None
        self.load_map([True] * C.SLOT_COUNT, log=False)
        self._log("info", "BOOT", "시뮬레이터 시작 - 원점 복귀가 필요합니다")

    # ------------------------------------------------------------ 기본 상태
    def occ(self):
        o = G.Occupancy()
        for w in self.wafers:
            if w["loc"] == "cassette":
                o.slots[w["slot"]] = True
            elif w["loc"] == "blade":
                o.blade = True
            elif w["loc"] == "C90":
                o.C90 = True
            elif w["loc"] == "C180":
                o.C180 = True
        return o

    def _wafer_at(self, loc, slot=None):
        for w in self.wafers:
            if w["loc"] == loc and (slot is None or w["slot"] == slot):
                return w
        return None

    def sensors(self):
        o = self.occ()
        return {
            "S1": abs(G.wrap_deg(self.theta)) <= 0.5,
            "S2": self.zc <= C.ZC_HOME + 0.5,
            "S3": self.rx <= G.RX_TOL and "s3_fail" not in self.faults,
            "S4": G.s4_on(self.theta, self.zc, self.rx, o),
            "S5": o.C180,
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

    def top_slot(self):
        o = self.occ()
        ks = [k for k, v in o.slots.items() if v]
        return max(ks) if ks else 0

    def read_tof(self):
        floor = C.slot_z(1) - 0.0 if "tof_ledge_in_beam" in self.faults else None
        d = T.expected_distance(self.top_slot(), floor_z=floor) + self.rng.gauss(0.0, C.TOF_NOISE_SD)
        self.tof_d = round(d, 1)
        return self.tof_d

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
        if self.pc < len(self.program) and self.program[self.pc].key in ("9", "10", "11", "12", "13"):
            self.c90_verified = False       # 검사 크래들에 웨이퍼가 남았을 수 있음(센서 없음)
        self.program, self.pc, self.ai, self.cur = [], 0, 0, None
        self.queue, self.hold = [], None
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
        self._need("IDLE", "READY", "CASSETTE_DONE")
        if self.estop_in:
            raise CommandError("estop", "E-STOP 해제 필요")
        self.program, self.pc, self.ai, self.cur = Q.homing_steps(), 0, 0, None
        self.state, self.homing_t0, self.slot = "HOMING", self.t, None
        self._log("info", "HOME", "원점 복귀 시작 (R -> Z -> ZC105 -> θ1 -> 최하단)")

    def _cmd_cassette_ready(self, manual_n=None):
        self._need("IDLE", "READY", "CASSETTE_DONE")
        self.slot_conf = {}
        if manual_n is not None:
            n = int(manual_n)
            if not 0 <= n <= C.SLOT_COUNT:
                raise CommandError("bad_arg", "N 범위 0~10")
            self.cassette = {"ready": True, "n_est": n, "kind": "MANUAL", "d": self.tof_d, "manual": True}
            self._log("info", "CASSETTE", f"카세트 준비 (수동 입력 N={n})")
        else:
            d = self.read_tof()
            r = T.classify(d)
            if r["kind"] in ("AMBIG01", "INVALID"):
                self.cassette = {"ready": False, "n_est": None, "kind": r["kind"], "d": d, "manual": False}
                self._log("warn", "TOF", f"TOF {d:.1f}mm 판정 불가({r['kind']}) - N 수동 입력 필요")
                return {"need_manual": True, "kind": r["kind"], "d": d}
            n = r["n"] or 0
            self.cassette = {"ready": True, "n_est": n, "kind": r["kind"], "d": d, "manual": False}
            self._log("info", "CASSETTE", f"카세트 준비: TOF {d:.1f}mm -> 최상단 슬롯 {n} (연속 투입 가정 N={n})")
        if self.state == "CASSETTE_DONE":
            self.state = "READY" if all(self.homed.values()) else "IDLE"
        return {"n_est": self.cassette["n_est"]}

    def _remaining_slots(self):
        """N 이하 슬롯 중 빈 슬롯 확정/이미 처리한 것을 뺀 목록(아래 -> 위)."""
        n = self.cassette["n_est"] or 0
        return [k for k in range(1, n + 1)
                if self.slot_conf.get(k) != "empty" and not self._slot_processed(k)]

    def _slot_processed(self, k):
        return any(w["slot"] == k and w["loc"] != "cassette" and w.get("cassette_gen") == self._gen
                   for w in self.wafers)

    def _cmd_start(self):
        self._need("READY", "CASSETTE_DONE")
        if not all(self.homed.values()):
            raise CommandError("not_homed", "원점 복귀 먼저")
        if not self.cassette["ready"]:
            raise CommandError("no_cassette", "'카세트 준비'를 먼저 누르세요")
        if self.occ().blade or self.sensors()["S4"]:
            raise CommandError("blade_wafer", "블레이드 위 웨이퍼 - 수동 제거 후 시작")
        if not self.c90_verified:
            raise CommandError("need_c90_confirm", "검사 크래들(90°)이 비었는지 확인 필요 (센서 없음)")
        self.queue = self._remaining_slots()
        if not self.queue:
            raise CommandError("nothing", "처리할 슬롯 없음")
        self._next_slot()
        self._log("info", "START", f"공정 시작: 슬롯 {self.queue_desc()}")

    def queue_desc(self):
        return ",".join(str(k) for k in ([self.slot] if self.slot else []) + self.queue)

    def _next_slot(self):
        if self.queue:
            self.slot = self.queue.pop(0)
            self.program, self.pc, self.ai, self.cur = Q.wafer_steps(self.slot), 0, 0, None
            self.state = "RUNNING"
            self.cycle_t0 = self.t
        else:
            self.slot = None
            self.program, self.pc, self.ai, self.cur = Q.final_steps(), 0, 0, None
            self.state = "RUNNING"

    def _cmd_pause(self):
        self._need(*RUN_STATES)
        self.prev_state, self.state = self.state, "PAUSED"
        self._log("info", "PAUSE", "일시정지")

    def _cmd_resume(self):
        self._need("PAUSED")
        if self.estop_in:
            raise CommandError("estop", "E-STOP 해제 필요")
        self.state = self.prev_state or "RUNNING"
        if self.cur and self.cur["kind"] in ("expect", "wait_s5", "check_pick"):
            self.cur["t0"] = self.t
        self._log("info", "RESUME", "재개")

    def _cmd_stop(self):
        self._need(*RUN_STATES, "PAUSED", "WAIT_OPERATOR")
        self._abort(lose_home=True)
        self.state, self.prompt, self.slot = "IDLE", None, None
        self._log("warn", "STOP", "소프트 정지 - 원점 복귀 필요 (E-STOP 아님)")

    def _cmd_estop(self):
        self.estop_in = True
        self._abort(lose_home=True)
        self.prompt, self.slot = None, None
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
        if choice == "c90_clear":
            self._need("IDLE", "READY", "CASSETTE_DONE")
            self.c90_verified = True
            self._log("info", "C90", "운전자 확인: 검사 크래들 비어 있음")
            return
        self._need("WAIT_OPERATOR")
        p, self.prompt = self.prompt, None
        n = p["slot"]
        if choice == "empty":
            self.slot_conf[n] = "empty"
            self.program, self.pc, self.ai, self.cur = Q.empty_slot_steps(n), 0, 0, None
            self._log("warn", "EMPTY", f"슬롯 {n} 빈 슬롯으로 처리 (운전자 확인)")
        elif choice == "retry":
            self.program, self.pc, self.ai, self.cur = Q.wafer_steps(n), 0, 0, None
            self._log("info", "RETRY", f"슬롯 {n} 픽업 재시도")
        else:
            self.prompt = p
            raise CommandError("bad_arg", "choice = empty | retry | c90_clear")
        self.state = "RUNNING"

    def _cmd_speed(self, value):
        v = float(value)
        if not 0.1 <= v <= 50:
            raise CommandError("bad_arg", "배속 0.1~50")
        self.speed = v

    def _cmd_load_map(self, slots):
        self._need("IDLE", "READY", "CASSETTE_DONE")
        self.load_map([bool(x) for x in slots])

    def load_map(self, slots, log=True):
        self._gen = getattr(self, "_gen", 0) + 1
        self.wafers = [w for w in self.wafers if w["loc"] not in ("cassette", "handed_off")]
        for i, present in enumerate(slots[:C.SLOT_COUNT]):
            if present:
                self._wid += 1
                self.wafers.append({"id": self._wid, "slot": i + 1, "loc": "cassette", "cassette_gen": self._gen,
                                    "t_pick": None, "t_inspect": None, "t_out": None, "t_handoff": None,
                                    "capture": None, "cycle_s": None})
        self.slot_conf = {}
        self.cassette = {"ready": False, "n_est": None, "kind": None, "d": None, "manual": False}
        if log:
            n = sum(1 for s in slots if s)
            self._log("info", "LOAD", f"(시뮬) 카세트 교체: {n}장 투입 - '카세트 준비'를 누르세요")

    def _cmd_fault(self, name, on=True):
        if name not in FAULTS:
            raise CommandError("bad_arg", f"알 수 없는 고장 {name}")
        if on:
            self.faults.add(name)
        else:
            self.faults.discard(name)
        self._log("warn" if on else "info", "FAULT", f"(시뮬) 고장주입 {name} {'ON' if on else 'OFF'}")

    def _cmd_remove_wafer(self, loc):
        self._need("IDLE", "READY", "CASSETTE_DONE", "ALARM")
        w = self._wafer_at(loc)
        if w is None:
            raise CommandError("bad_arg", f"{loc} 에 웨이퍼 없음")
        w["loc"] = "removed"
        self._log("info", "REMOVE", f"(시뮬) 운전자가 {loc} 웨이퍼 수동 제거")

    def _cmd_debug_hold(self, step, frac=0.0, slot=None):
        self.hold = {"step": str(step), "frac": float(frac), "slot": slot}

    # ------------------------------------------------------------ 진행
    def tick(self, dt):
        self.tick_n += 1
        if "sim_exception" in self.faults:
            self.faults.discard("sim_exception")
            raise RuntimeError("injected sim exception")
        remaining = max(0.0, float(dt)) * self.speed
        guard = 0
        while remaining > 1e-9 and guard < 2000:
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
        """환경: 시간, 다음 이송로봇 인계, 원점 타임아웃."""
        self.t += dt
        if self.handoff_left is not None:
            self.handoff_left -= dt
            if self.handoff_left <= 0 and self.rx <= G.RX_TOL:
                w = self._wafer_at("C180")
                if w:
                    w["loc"], w["t_handoff"] = "handed_off", round(self.t, 2)
                    self._log("info", "HANDOFF", f"인계 완료: 슬롯 {w['slot']} 웨이퍼 -> 다음 이송로봇 (S5 비움)")
                self.handoff_left = None
        if self.state == "HOMING" and self.homing_t0 is not None and self.t - self.homing_t0 > C.HOMING_TIMEOUT_S:
            self._raise("HOMING_TIMEOUT", f"{C.HOMING_TIMEOUT_S:.0f}s")

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
                z_cross = (C.slot_z(slot) if where == "slot" else C.CRADLE_SEAT_Z) - C.BLADE_PLANE_OFS
                ev.append((z_cross, kind, where, slot))
            self.cur = {"kind": "z", "from": self.zc, "to": act.value, "events": ev}
        elif k == "r":
            col = G.r_move(self.theta, self.zc, self.rx, act.value, o)
            if col:
                self._raise("COLLISION", col)
                return False
            self.cur = {"kind": "r", "from": self.rx, "to": act.value}
        elif k == "rot":
            ok, why = Q.can_rotate(self.zc, self.rx, self._read("S3"), carrying=o.blade)
            if not ok:
                self._raise("INTERLOCK", why)
                return False
            pts = list(act.via) + [act.value]
            self.cur = {"kind": "rot", "from": self.theta, "pts": pts, "to": act.value}
        elif k in ("expect", "check_pick", "wait_s5"):
            if k == "wait_s5" and not self.sensors()["S5"]:
                return True
            if k != "wait_s5" and self._read(act.sensor) == act.expect:
                self._on_expect_ok(act, step)
                return True
            if k == "check_pick":
                n = self.slot
                self.prompt = {"kind": "s4_missing", "slot": n, "options": ["empty", "retry"],
                               "msg": f"슬롯 {n}: S4 없음 - 빈 슬롯인지 픽업 실패인지 구분 불가. 눈으로 확인 후 선택"}
                self.state = "WAIT_OPERATOR"
                self._log("alarm", "S4_NONE", self.prompt["msg"], slot=n)
                for f in ("s4_false_none",):
                    self.faults.discard(f)
                return False
            self.cur = {"kind": k, "t0": self.t, "sensor": act.sensor, "expect": act.expect, "warned": False}
            if k == "wait_s5":
                self.state = "WAIT_S5"
                self._log("info", "WAIT_S5", "반출 크래들 점유 - 다음 이송로봇 대기(S5)")
        elif k == "capture":
            self.cur = {"kind": "capture", "left": C.CAPTURE_S}
            w = self._wafer_at("C90")
            self.pending_captures.append({"slot": self.slot, "wafer": w["id"] if w else None, "t": round(self.t, 2)})
            if w:
                w["t_inspect"] = round(self.t, 2)
            self._log("info", "CAPTURE", f"슬롯 {self.slot} 웨이퍼 촬영 (백라이트)", slot=self.slot)
        return True

    def _on_expect_ok(self, act, step):
        if act.kind == "check_pick" and self.slot:
            self.slot_conf[self.slot] = "present"
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
        if self.hold and self.hold["step"] == step.key and self._hold_slot_ok() and k in ("z", "r", "rot"):
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
                zx = ev[0]
                if lo - 1e-9 <= zx <= hi + 1e-9:
                    self._apply_transfer(ev)
                    c["events"].remove(ev)
            self.zc = new
        elif k == "r":
            new, used, done = self._move_toward(self.rx, c["to"], C.R_SPEED, limit)
            self.rx = new
        elif k == "rot":
            target = c["pts"][0]
            new, used, done_pt = self._move_toward(self.theta, target, C.THETA_SPEED, limit)
            self.theta = new
            done = False
            if done_pt:
                c["pts"].pop(0)
                done = not c["pts"]
        elif k == "capture":
            used = min(limit, c["left"])
            c["left"] -= used
            done = c["left"] <= 1e-9
        else:   # expect / wait_s5 : 시간 경과 대기
            used = limit
            if k == "wait_s5":
                done = not self.sensors()["S5"]
                if not done and not c["warned"] and self.t + used - c["t0"] >= C.S5_WARN_S:
                    c["warned"] = True
                    self._log("warn", "S5_WAIT", f"S5 인계 대기 {C.S5_WARN_S:.0f}s 초과 (경고)")
                if done:
                    used = 0.0
                    self.state = "RUNNING"
            else:
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
        if c["kind"] == "z":
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
        if kind == "pick":
            w = self._wafer_at("cassette", slot) if where == "slot" else self._wafer_at(where)
            if w:
                w["loc"] = "blade"
                if where == "slot":
                    w["t_pick"] = round(self.t, 2)
                self._log("info", "PICK", f"픽업: {'슬롯 ' + str(slot) if where == 'slot' else where} -> 블레이드")
        else:
            w = self._wafer_at("blade")
            if w:
                if where == "slot":
                    w["loc"], w["slot"] = "cassette", slot
                else:
                    w["loc"] = where
                    if where == "C180":
                        w["t_out"] = round(self.t, 2)
                        self.handoff_left = C.HANDOFF_DELAY_FAULT_S if "next_robot_delay" in self.faults else C.HANDOFF_S
                self._log("info", "PLACE", f"안착: 블레이드 -> {'슬롯 ' + str(slot) if where == 'slot' else where}")

    def _program_done(self):
        if self.state == "HOMING":
            self.program, self.pc, self.ai = [], 0, 0
            self.homing_t0 = None
            self.state = "READY"
            self._log("info", "HOMED", "원점 복귀 완료 (Z 최하단 · R 수축 · θ1 0°)")
            return
        if self.program and self.program[0].key == "F1":
            self.program, self.pc, self.ai = [], 0, 0
            self.state = "CASSETTE_DONE"
            self._log("info", "DONE", "카세트 완료 - 교체 후 '카세트 준비'")
            return
        if self.slot is not None and self.slot_conf.get(self.slot) != "empty":
            dt = self.t - (self.cycle_t0 or self.t)
            self.cycles.append(round(dt, 2))
            w = next((w for w in self.wafers if w["slot"] == self.slot and w["loc"] in ("C180", "handed_off")
                      and w.get("cassette_gen") == self._gen and w.get("cycle_s") is None), None)
            if w:
                w["cycle_s"] = round(dt, 2)
            self._log("info", "CYCLE", f"슬롯 {self.slot} 사이클 {dt:.1f}s (가정 속도 기준)")
        self._next_slot()

    # ------------------------------------------------------------ 스냅샷
    def interlock(self):
        ok, why = Q.can_rotate(self.zc, self.rx, self.sensors()["S3"], carrying=self.occ().blade)
        return {"rotate_ok": ok, "reason": why}

    def slot_view(self):
        n_est = self.cassette["n_est"] if self.cassette["ready"] else None
        out = []
        for k in range(1, C.SLOT_COUNT + 1):
            w_here = self._wafer_at("cassette", k)
            w_any = next((w for w in self.wafers if w["slot"] == k and w.get("cassette_gen") == self._gen
                          and w["loc"] != "cassette"), None)
            conf = self.slot_conf.get(k)
            if w_any is not None:
                status = {"blade": "moving", "C90": "inspect", "C180": "output",
                          "handed_off": "done", "removed": "removed"}.get(w_any["loc"], "moving")
            elif conf == "empty":
                status = "empty"
            elif n_est is None:
                status = "unknown"
            elif k > n_est:
                status = "above_top"
            elif conf == "present":
                status = "present"
            else:
                status = "est_present"
            out.append({"n": k, "status": status, "sim_present": w_here is not None})
        return out

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
            elif a.kind == "wait_s5":
                exp = {"sensor": "S5", "value": False}
        sv = self.slot_view()
        n_est = self.cassette["n_est"]
        conf_present = sum(1 for k, v in self.slot_conf.items() if v == "present")
        done = sum(1 for w in self.wafers if w["loc"] == "handed_off" and w.get("cassette_gen") == self._gen)
        remaining = sum(1 for s in sv if s["status"] in ("est_present", "present"))
        mismatch = any(v == "empty" and n_est and k <= n_est for k, v in self.slot_conf.items())
        o = self.occ()
        tof_kind = None
        if self.tof_d is not None:
            tof_kind = T.classify(self.tof_d)
        return {
            "tick": self.tick_n, "t": round(self.t, 3), "source": self.source, "state": self.state,
            "speed": self.speed,
            "pose": {"theta": round(self.theta, 3), "zc": round(self.zc, 3), "rx": round(self.rx, 3)},
            "target": target,
            "homed": all(self.homed.values()), "homed_axes": dict(self.homed),
            "step": ({"key": step.key, "no": step.no, "group": step.group, "title": step.title,
                      "why": step.why, "stage": Q.STAGE_OF_GROUP.get(step.group),
                      "action": self.ai, "actions": len(step.actions)} if step else None),
            "slot": self.slot, "queue": list(self.queue),
            "sensors": self.sensors(), "expect": exp,
            "tof": {"d": self.tof_d, "kind": tof_kind},
            "cassette": dict(self.cassette, confirmed=conf_present, remaining=remaining, mismatch=mismatch),
            "slots": sv,
            "occ": {"blade": o.blade, "C90": o.C90, "C180": o.C180},
            "wafers": [{"id": w["id"], "slot": w["slot"], "loc": w["loc"], "cycle_s": w["cycle_s"],
                        "capture": w["capture"]} for w in self.wafers
                       if w["loc"] in ("blade", "C90", "C180") or w.get("cassette_gen") == self._gen],
            "alarm": self.alarm, "prompt": self.prompt,
            "interlock": self.interlock(),
            "counts": {"done": done,
                       "cycle_last": self.cycles[-1] if self.cycles else None,
                       "cycle_avg": round(sum(self.cycles) / len(self.cycles), 1) if self.cycles else None},
            "faults": sorted(self.faults), "c90_verified": self.c90_verified,
            "handoff_left": round(self.handoff_left, 1) if self.handoff_left is not None else None,
            "captures": self.captures[-6:],
            "log_seq": self.seq,
        }
