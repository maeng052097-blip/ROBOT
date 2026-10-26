"""데이터 소스 인터페이스 - server.py 는 이것만 쓴다. SIM(CellSim) 과 LIVE(LiveSource, live.py)가 같은 메서드를 낸다.

  tick(dt)                      시간 진행(SIM) / 수신 · 하트비트(LIVE)
  command(cmd, args)            ('done', 결과) 또는 ('pending', 명령 id)  - CommandError 를 낼 수 있음
  wait_ack(id, timeout)         LIVE: 잠금 밖에서 ACK 대기 -> {'ok', 'e', 'st', 'msg'} 또는 None(시간 초과)
  snapshot()                    화면 스냅샷(같은 모양)
  events_since(seq)             이벤트 로그
  take_capture_requests()       촬영 요청 [{id, n, w, wafer}]
  submit_judgment(id, r, dr, dth)
  finished_wafers()             끝난 웨이퍼 기록(CSV)
  raise_internal(code, d)       서버가 잡은 예외를 알람으로
  state, faults, kind ('SIM' | 'LIVE')
"""
from wafer_cell.sim import CellSim, CommandError, SIM_ONLY, LIVE_CMDS, FAULTS   # noqa: F401


class SimSource:
    kind = "SIM"

    def __init__(self, seed=1, speed=1.0):
        self.sim = CellSim(seed=seed, source="SIM", judge_mode="external")
        self.sim.speed = float(speed)

    @property
    def state(self):
        return self.sim.state

    @property
    def faults(self):
        return self.sim.faults

    def tick(self, dt):
        self.sim.tick(dt)

    def command(self, cmd, args=None):
        return ("done", self.sim.command(cmd, **(args or {})))

    def wait_ack(self, cid, timeout=None):
        return None

    def snapshot(self):
        return self.sim.snapshot()

    def events_since(self, seq):
        return self.sim.events_since(seq)

    def take_capture_requests(self):
        return self.sim.take_capture_requests()

    def submit_judgment(self, cid, result, dr=0.0, dth=0.0):
        return self.sim.submit_judgment(cid, result, dr, dth)

    def finished_wafers(self):
        return self.sim.finished_wafers()

    def raise_internal(self, code, d=""):
        return self.sim.raise_internal(code, d)


def make_source(kind="sim", serial=None, baud=None, seed=1, speed=1.0):
    if kind == "sim":
        return SimSource(seed=seed, speed=speed)
    from wafer_cell import cell_config as C
    from wafer_cell.live import LiveSource, make_transport
    return LiveSource(make_transport(serial, baud or C.SERIAL_BAUD, seed=seed, speed=speed))
