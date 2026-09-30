"""CELL_V9 충돌·여유·센서 기하 판정(순수 로직).

시뮬레이터가 '대본'이 아니라 기하로 판단하도록: Z/R 이동 한 구간(sweep)마다
  - 웨이퍼 이동(픽업/안착) 이벤트와
  - 충돌(블레이드·운반 웨이퍼 띠 vs 슬롯 웨이퍼·선반·크래들·허브)
을 계산한다. 반경 방향 겹침은 문서 수치로 미리 따진 결과를 규칙으로 쓴다:
  - 수축(RX 0) 블레이드 끝 R203 은 스테이션 웨이퍼(185.5~285.5)와 17.5 겹친다 -> RX 무관하게 높이만 보면 된다.
  - 수축 상태 운반 웨이퍼(134.5~234.5)도 스테이션 웨이퍼와 겹친다.
  - 운반 웨이퍼는 RX > 13.5 부터 카세트 선반(|y|47~57, x215~)에 닿는 구역에 들어간다.
높이 구간 비교는 경계 접촉을 겹침으로 보지 않는다(eps).
"""
from dataclasses import dataclass, field

from wafer_cell import cell_config as C

EPS = 1e-6
RX_TOL = 0.5
LEDGE_REACH_RX = 13.5      # 운반 웨이퍼가 선반 구역에 들어가기 시작하는 RX


@dataclass
class Occupancy:
    slots: dict = field(default_factory=dict)   # n -> True(웨이퍼 있음)
    blade: bool = False
    C90: bool = False
    C180: bool = False

    def cradle(self, key):
        return getattr(self, key)


def wrap_deg(a):
    return ((a + 180.0) % 360.0) - 180.0


def station_at(theta):
    """θ 가 스테이션 각도 ±STATION_TOL_DEG 이내면 그 이름, 아니면 None."""
    for name, ang in C.STATIONS.items():
        if abs(wrap_deg(theta - ang)) <= C.STATION_TOL_DEG:
            return name
    return None


CRADLE_KEY = {"inspect": "C90", "output": "C180"}


def _ov(a, b):
    return a[0] < b[1] - EPS and a[1] > b[0] + EPS


def _blade_band(zc):
    return (zc + C.BLADE_BODY[0], zc + C.BLADE_PLANE_OFS)


def _sweep(z0, z1, lo, hi):
    return (min(z0, z1) + lo, max(z0, z1) + hi)


def _rx_mode(rx):
    if abs(rx - C.RX_EXT) <= RX_TOL:
        return "ext"
    if abs(rx - C.RX_RET) <= RX_TOL:
        return "ret"
    return "mid"


def _ledges():
    bands = [C.CASSETTE_BLOCK_Z, C.CASSETTE_TOP_RAIL_Z]
    for k in range(2, C.SLOT_COUNT + 1):
        sk = C.slot_z(k)
        bands.append((sk - C.LEDGE_T, sk))
    return bands


def blade_tip(rx):
    return C.BLADE_TIP_X + rx


def z_move(theta, rx, zc0, zc1, occ, t=None):
    """Z 이동 한 구간. 반환 {'events': [(종류, 위치, 슬롯)], 'collision': None|사유(한글)}."""
    t = C.WAFER_T if t is None else t
    ev, col = [], None
    lo, hi = min(zc0, zc1), max(zc0, zc1)
    if lo < C.ZC_MIN - EPS or hi > C.ZC_MAX + EPS:
        return {"events": ev, "collision": f"Z 범위 초과 (ZC {lo:.1f}~{hi:.1f})"}
    st = station_at(theta)
    mode = _rx_mode(rx)
    p0, p1 = zc0 + C.BLADE_PLANE_OFS, zc1 + C.BLADE_PLANE_OFS
    blade_sw = _sweep(zc0, zc1, C.BLADE_BODY[0], C.BLADE_PLANE_OFS)
    carrying = occ.blade

    if st is None:
        if mode != "ret" or hi > C.ZC_SWING + 0.25:
            col = "스테이션 사이에서 상승/전진 금지"
        return {"events": ev, "collision": col}
    if mode == "mid":
        return {"events": ev, "collision": "RX 중간 위치에서 Z 이동"}

    if st == "cassette":
        occupied = [k for k in range(1, C.SLOT_COUNT + 1) if occ.slots.get(k)]
        moved = None           # 옮겨진 슬롯(충돌 검사에서 제외)
        carried = None         # 운반 웨이퍼 띠
        if mode == "ext" and not carrying and p1 > p0:
            for k in occupied:
                sk = C.slot_z(k)
                if p0 <= sk < p1:
                    ev.append(("pick", "slot", k))
                    moved, carried = k, (sk, p1 + t)
                    break
        elif mode == "ext" and carrying and p1 < p0:
            for k in sorted(range(1, C.SLOT_COUNT + 1), reverse=True):
                sk = C.slot_z(k)
                if not occ.slots.get(k) and p1 < sk <= p0:
                    ev.append(("place", "slot", k))
                    moved, carried = k, (sk, p0 + t)
                    break
        if carried is None and carrying:
            carried = _sweep(p0, p1, 0.0, t)
        for k in occupied:
            if k == moved:
                continue
            wb = (C.slot_z(k), C.slot_z(k) + t)
            if _ov(blade_sw, wb):
                return {"events": [], "collision": f"블레이드가 슬롯 {k} 웨이퍼와 충돌"}
            if carried and _ov(carried, wb):
                return {"events": [], "collision": f"운반 웨이퍼가 슬롯 {k} 웨이퍼와 충돌"}
        if carried and mode == "ext":
            for lb in _ledges():
                if _ov(carried, lb):
                    return {"events": [], "collision": "운반 웨이퍼가 카세트 선반/레일과 충돌 (여유 부족)"}
        if blade_tip(rx) > C.CASSETTE_BASE_X and _ov(blade_sw, C.CASSETTE_BASE_Z):
            return {"events": [], "collision": "블레이드가 카세트 베이스판과 충돌"}
        return {"events": ev, "collision": None}

    # 크래들(검사 90 / 반출 180)
    key = CRADLE_KEY[st]
    occupied = occ.cradle(key)
    seat = C.CRADLE_SEAT_Z
    moved = False
    carried = None
    if mode == "ext" and not carrying and occupied and p0 <= seat < p1:
        ev.append(("pick", key, 0))
        moved, carried = True, (seat, p1 + t)
    elif mode == "ext" and carrying and not occupied and p1 < seat <= p0:
        ev.append(("place", key, 0))
        moved, carried = True, (seat, p0 + t)
    if carried is None and carrying:
        carried = _sweep(p0, p1, 0.0, t)
    if occupied and not moved:
        wb = (seat, seat + t)
        if _ov(blade_sw, wb):
            return {"events": [], "collision": f"블레이드가 {key} 크래들 웨이퍼와 충돌"}
        if carried and _ov(carried, wb):
            return {"events": [], "collision": f"운반 웨이퍼가 {key} 크래들 웨이퍼와 충돌"}
    if blade_tip(rx) > C.HUB_R_MIN and _ov(blade_sw, C.HUB_Z):
        return {"events": [], "collision": f"블레이드 끝(R{blade_tip(rx):.0f})이 크래들 허브(R{C.HUB_R_MIN:.0f}~)와 충돌"}
    if mode != "ret" and lo < C.CRADLE_ZC_SEAT - 0.01:
        return {"events": [], "collision": "전진한 채 크래들 아래로 하강 (백라이트/구조물 충돌)"}
    return {"events": ev, "collision": None}


def r_move(theta, zc, rx0, rx1, occ, t=None):
    """R(실린더) 이동 한 구간. 충돌 사유(한글) 또는 None."""
    t = C.WAFER_T if t is None else t
    st = station_at(theta)
    rmax = max(rx0, rx1)
    if st is None:
        return "스테이션 사이에서 전진 금지" if rmax > RX_TOL else None
    bb = _blade_band(zc)
    p = zc + C.BLADE_PLANE_OFS
    carried = (p, p + t) if occ.blade else None
    if st == "cassette":
        for k in range(1, C.SLOT_COUNT + 1):
            if not occ.slots.get(k):
                continue
            wb = (C.slot_z(k), C.slot_z(k) + t)
            if _ov(bb, wb):
                return f"블레이드가 슬롯 {k} 웨이퍼를 밀어냄"
            if carried and _ov(carried, wb):
                return f"운반 웨이퍼가 슬롯 {k} 웨이퍼와 충돌"
        if carried and rmax > LEDGE_REACH_RX:
            for lb in _ledges():
                if _ov(carried, lb):
                    return "운반 웨이퍼가 카세트 선반/레일과 충돌 (여유 부족)"
        if blade_tip(rmax) > C.CASSETTE_BASE_X and _ov(bb, C.CASSETTE_BASE_Z):
            return "블레이드가 카세트 베이스판과 충돌"
        return None
    key = CRADLE_KEY[st]
    if occ.cradle(key):
        wb = (C.CRADLE_SEAT_Z, C.CRADLE_SEAT_Z + t)
        if _ov(bb, wb):
            return f"블레이드가 {key} 크래들 웨이퍼를 밀어냄"
        if carried and _ov(carried, wb):
            return f"운반 웨이퍼가 {key} 크래들 웨이퍼와 충돌"
    if blade_tip(rmax) > C.HUB_R_MIN and _ov(bb, C.HUB_Z):
        return f"블레이드 끝(R{blade_tip(rmax):.0f})이 크래들 허브(R{C.HUB_R_MIN:.0f}~)와 충돌"
    if rmax > RX_TOL and zc < C.CRADLE_ZC_SEAT - 0.01:
        return "크래들 아래 높이에서 전진 (백라이트/구조물 충돌)"
    return None


def s4_on(theta, zc, rx, occ, t=None):
    """S4(블레이드 어깨 반사형, 위를 봄) 기하 판정. 운반 웨이퍼 또는 0~5mm 위 스테이션 웨이퍼면 ON."""
    if occ.blade:
        return True
    st = station_at(theta)
    if st is None:
        return False
    r_s4 = C.BLADE_ORIGIN_X + C.S4_LOCAL_X + rx
    if abs(r_s4 - C.STATION_R) > C.WAFER_D / 2:
        return False
    surface = zc + C.BLADE_BODY[1]
    if st == "cassette":
        cands = [C.slot_z(k) for k in range(1, C.SLOT_COUNT + 1) if occ.slots.get(k)]
    else:
        cands = [C.CRADLE_SEAT_Z] if occ.cradle(CRADLE_KEY[st]) else []
    return any(C.S4_RANGE[0] <= z - surface <= C.S4_RANGE[1] for z in cands)


def clearances(preset=None, t=None):
    """문서 수치로 계산한 빠듯한 여유(mm) 목록 — 화면 인터록 패널용."""
    t = C.WAFER_T if t is None else t
    off, lift = C.pick_offsets(preset)
    plane_after_lift = off + lift + C.BLADE_PLANE_OFS          # Sn 기준 상대 높이
    return [
        {"key": "insert_gap", "label": "카세트 삽입 틈", "mm": round(-(off + C.BLADE_PLANE_OFS), 2)},
        {"key": "ledge_above", "label": "리프트 후 윗 선반까지", "mm": round(C.SLOT_PITCH - C.LEDGE_T - plane_after_lift - t, 2)},
        {"key": "top_rail", "label": "슬롯 10 -> 상단 레일",
         "mm": round(C.CASSETTE_TOP_RAIL_Z[0] - (C.slot_z(C.SLOT_COUNT) + plane_after_lift + t), 2)},
        {"key": "cradle_repick", "label": "크래들 재픽업 후 패드 위",
         "mm": round(C.CRADLE_ZC_PICK + C.BLADE_PLANE_OFS - C.CRADLE_SEAT_Z, 2)},
        {"key": "swing", "label": "선회 중 스테이션 아래",
         "mm": round(C.SWING_CLEAR_Z - (C.ZC_SWING + C.BLADE_PLANE_OFS + t), 2)},
        {"key": "hub", "label": "전진 블레이드 끝 -> 크래들 허브",
         "mm": round(C.HUB_R_MIN - blade_tip(C.RX_EXT), 2)},
    ]
