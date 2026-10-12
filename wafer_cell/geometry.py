"""R9 충돌 · 이송 이벤트 · 센서 기하 판정(순수 로직).

시뮬레이터가 '대본'이 아니라 기하로 판단하도록 Z/R 이동 한 구간마다
  - 웨이퍼 이동(집기/놓기), OK 카세트 이동(도크 -> 포크 -> 대차 받침) 이벤트와
  - 충돌(포크 · 운반 웨이퍼 띠 vs 카세트 슬롯 웨이퍼 · 크래들 웨이퍼, 받침 없는 대차 쪽 전진)
을 계산한다. 반경 방향 겹침은 수치로 미리 따진 결과를 규칙으로 쓴다(CELL_V9 와 같은 몸체):
  - 수축(R0) 포크 끝 R203 은 스테이션 웨이퍼(185.5~285.5)와 17.5 겹친다 -> R 과 무관하게 높이만 보면 된다.
    그래서 투입은 아래 슬롯부터 꺼내고, OK/NG 는 위 슬롯부터 채운다(R0 로 오르내릴 때 빈 슬롯만 지나감).
  - 운반 웨이퍼는 R > 13.5 부터 카세트 받침 이빨 구역에 들어간다.
높이 구간 비교는 경계 접촉을 겹침으로 보지 않는다(eps).
"""
from dataclasses import dataclass, field

from wafer_cell import cell_config as C

EPS = 1e-6
RX_TOL = 0.5
LEDGE_REACH_RX = 13.5
SIDE_POSES = (C.CART_SIDE_THETA,)      # T1/T2: OK 도크에서 40° 비킨 자리(Z 이동만 허용, R0)


@dataclass
class Occupancy:
    cass: dict = field(default_factory=dict)      # 'input'|'ng'|'ok' -> {n: True} (도크에 카세트가 있을 때만 키 존재)
    blade: bool = False                           # 포크 위 웨이퍼
    inspect: bool = False                         # 검사 크래들 웨이퍼
    fork_cassette: bool = False                   # 포크가 OK 카세트를 들고 있음
    cart_seat_free: bool = False                  # 대차가 도킹돼 받침 자리가 로봇 앞에 있고 비었음

    def present(self, key):
        return key in self.cass

    def slots(self, key):
        return self.cass.get(key, {})


def wrap_deg(a):
    return ((a + 180.0) % 360.0) - 180.0


def station_at(theta):
    """θ 가 스테이션 각도 ±STATION_TOL_DEG 이내면 그 이름, 비킨 자리면 'side', 아니면 None."""
    for name, ang in C.STATIONS.items():
        if abs(wrap_deg(theta - ang)) <= C.STATION_TOL_DEG:
            return name
    for ang in SIDE_POSES:
        if abs(wrap_deg(theta - ang)) <= C.STATION_TOL_DEG:
            return "side"
    return None


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
    if abs(rx - C.RX_CART) <= RX_TOL:
        return "cart"
    return "mid"


def _ledges():
    return [(C.slot_z(k) - C.LEDGE_T, C.slot_z(k)) for k in range(2, C.SLOT_COUNT + 1)]


def blade_tip(rx):
    return C.BLADE_TIP_X + rx


def z_move(theta, rx, zc0, zc1, occ, t=None):
    """Z 이동 한 구간. 반환 {'events': [(종류, 위치, 슬롯)], 'collision': None|사유(한글)}.
    종류: pick/place(웨이퍼), cass_pick(OK 도크 -> 포크), cass_place(포크 -> 대차 받침)."""
    t = C.WAFER_T if t is None else t
    ev = []
    lo, hi = min(zc0, zc1), max(zc0, zc1)
    if lo < C.ZC_MIN - EPS or hi > C.ZC_MAX + EPS:
        return {"events": ev, "collision": f"Z 범위 초과 (Z {lo:.1f}~{hi:.1f})"}
    st = station_at(theta)
    mode = _rx_mode(rx)
    if st is None:
        if mode != "ret" or hi > C.ZC_SWING + 0.25:
            return {"events": ev, "collision": "스테이션 사이에서 상승/전진 금지"}
        return {"events": ev, "collision": None}
    if st == "side":
        if mode != "ret":
            return {"events": ev, "collision": "비킨 자리에서는 R 0 으로만 오르내림"}
        return {"events": ev, "collision": None}
    if mode == "mid":
        return {"events": ev, "collision": "R 중간 위치에서 Z 이동"}

    p0, p1 = zc0 + C.BLADE_PLANE_OFS, zc1 + C.BLADE_PLANE_OFS
    blade_sw = _sweep(zc0, zc1, C.BLADE_BODY[0], C.BLADE_PLANE_OFS)
    carrying = occ.blade

    if st == "cart":
        if occ.fork_cassette and mode == "cart" and occ.cart_seat_free and zc1 < zc0 \
                and zc1 <= C.CART_SEAT_LIFT_Z <= zc0:
            ev.append(("cass_place", "cart", 0))
        elif occ.fork_cassette and mode == "cart" and not occ.cart_seat_free and zc1 < C.CART_SEAT_LIFT_Z:
            return {"events": [], "collision": "대차 받침이 로봇 앞에 없음 - 카세트를 내려놓을 수 없음"}
        return {"events": ev, "collision": None}

    if st in C.CASSETTES:
        if not occ.present(st):
            return {"events": ev, "collision": None}
        slots = occ.slots(st)
        occupied = [k for k in range(1, C.SLOT_COUNT + 1) if slots.get(k)]
        moved, carried = None, None
        if st == "ok" and mode == "ext" and not carrying and not occ.fork_cassette and zc1 > zc0 \
                and zc0 <= C.CART_LIFT_Z <= zc1:
            return {"events": [("cass_pick", "ok", 0)], "collision": None}   # 클램프 윗판째로 듦(웨이퍼보다 위)
        if mode == "ext" and not carrying and p1 > p0 and st == "input":
            for k in occupied:
                sk = C.slot_z(k)
                if p0 <= sk < p1:
                    ev.append(("pick", st, k))
                    moved, carried = k, (sk, p1 + t)
                    break
        elif mode == "ext" and carrying and p1 < p0:          # OK/NG 넣기, 투입은 '재시도' 때 제자리에 다시 놓기
            for k in sorted(range(1, C.SLOT_COUNT + 1), reverse=True):
                sk = C.slot_z(k)
                if not slots.get(k) and p1 < sk <= p0:
                    ev.append(("place", st, k))
                    moved, carried = k, (sk, p0 + t)
                    break
        if carried is None and carrying:
            carried = _sweep(p0, p1, 0.0, t)
        for k in occupied:
            if k == moved:
                continue
            wb = (C.slot_z(k), C.slot_z(k) + t)
            if _ov(blade_sw, wb):
                return {"events": [], "collision": f"포크가 {st.upper()} 슬롯 {k} 웨이퍼와 충돌"}
            if carried and _ov(carried, wb):
                return {"events": [], "collision": f"운반 웨이퍼가 {st.upper()} 슬롯 {k} 웨이퍼와 충돌"}
        if carried and mode == "ext":
            for lb in _ledges():
                if _ov(carried, lb):
                    return {"events": [], "collision": "운반 웨이퍼가 카세트 받침 이빨과 충돌 (여유 부족)"}
        return {"events": ev, "collision": None}

    # 검사 크래들(120°)
    seat = C.CRADLE_SEAT_Z
    occupied = occ.inspect
    moved, carried = False, None
    if mode == "ext" and not carrying and occupied and p0 <= seat < p1:
        ev.append(("pick", "inspect", 0))
        moved, carried = True, (seat, p1 + t)
    elif mode == "ext" and carrying and not occupied and p1 < seat <= p0:
        ev.append(("place", "inspect", 0))
        moved, carried = True, (seat, p0 + t)
    if carried is None and carrying:
        carried = _sweep(p0, p1, 0.0, t)
    if occupied and not moved:
        wb = (seat, seat + t)
        if _ov(blade_sw, wb):
            return {"events": [], "collision": "포크가 검사 크래들 웨이퍼와 충돌"}
        if carried and _ov(carried, wb):
            return {"events": [], "collision": "운반 웨이퍼가 검사 크래들 웨이퍼와 충돌"}
    if mode != "ret" and lo < C.CRADLE_ZC_SEAT - 0.01:
        return {"events": [], "collision": "전진한 채 크래들 아래로 하강 (백라이트 충돌)"}
    return {"events": ev, "collision": None}


def r_move(theta, zc, rx0, rx1, occ, t=None):
    """R 이동 한 구간. 충돌 사유(한글) 또는 None."""
    t = C.WAFER_T if t is None else t
    st = station_at(theta)
    rmax = max(rx0, rx1)
    if st is None or st == "side":
        return "스테이션 사이에서 전진 금지" if rmax > RX_TOL else None
    if rmax > C.RX_MAX + RX_TOL:
        return f"R {rmax:.0f} 은 최대 전진({C.RX_MAX:g}) 초과"
    if st == "cart":
        if rmax > RX_TOL and occ.fork_cassette and not occ.cart_seat_free and abs(rx1 - C.RX_RET) > RX_TOL:
            return "대차 받침이 로봇 앞에 없음 - 내밀기 금지"
        return None
    if rmax > C.RX_EXT + RX_TOL:
        return f"R {rmax:.0f} 은 대차 쪽(θ {C.STATIONS['cart']:g})에서만"
    bb = _blade_band(zc)
    p = zc + C.BLADE_PLANE_OFS
    carried = (p, p + t) if occ.blade else None
    if st in C.CASSETTES:
        slots = occ.slots(st)
        for k in range(1, C.SLOT_COUNT + 1):
            if not slots.get(k):
                continue
            wb = (C.slot_z(k), C.slot_z(k) + t)
            if _ov(bb, wb):
                return f"포크가 {st.upper()} 슬롯 {k} 웨이퍼를 밀어냄"
            if carried and _ov(carried, wb):
                return f"운반 웨이퍼가 {st.upper()} 슬롯 {k} 웨이퍼와 충돌"
        if carried and occ.present(st) and rmax > LEDGE_REACH_RX:
            for lb in _ledges():
                if _ov(carried, lb):
                    return "운반 웨이퍼가 카세트 받침 이빨과 충돌 (여유 부족)"
        return None
    if occ.inspect:
        wb = (C.CRADLE_SEAT_Z, C.CRADLE_SEAT_Z + t)
        if _ov(bb, wb):
            return "포크가 검사 크래들 웨이퍼를 밀어냄"
        if carried and _ov(carried, wb):
            return "운반 웨이퍼가 검사 크래들 웨이퍼와 충돌"
    if rmax > RX_TOL and zc < C.CRADLE_ZC_SEAT - 0.01:
        return "크래들 아래 높이에서 전진 (백라이트 충돌)"
    return None


def s4_on(theta, zc, rx, occ):
    """S4(포크 어깨 반사형, 위를 봄). 운반 웨이퍼 또는 0~5mm 위 스테이션 웨이퍼면 ON."""
    if occ.blade:
        return True
    st = station_at(theta)
    if st not in C.CASSETTES and st != "inspect":
        return False
    r_s4 = C.BLADE_ORIGIN_X + C.S4_LOCAL_X + rx
    if abs(r_s4 - C.STATION_R) > C.WAFER_D / 2:
        return False
    surface = zc + C.BLADE_BODY[1]
    if st == "inspect":
        cands = [C.CRADLE_SEAT_Z] if occ.inspect else []
    else:
        cands = [C.slot_z(k) for k, v in occ.slots(st).items() if v]
    return any(C.S4_RANGE[0] <= z - surface <= C.S4_RANGE[1] for z in cands)


def clearances(t=None):
    """비디오 수치로 계산한 여유(mm) — README/현장 확인용."""
    t = C.WAFER_T if t is None else t
    lifted = C.PICK_OFS + C.PICK_LIFT + C.BLADE_PLANE_OFS          # Sn 기준 상대 높이(+4.5)
    return [
        {"key": "insert_gap", "label": "카세트 삽입 틈 (포크 면 아래)", "mm": round(-(C.PICK_OFS + C.BLADE_PLANE_OFS), 2)},
        {"key": "lift", "label": "들기 후 받침 이빨 위", "mm": round(lifted, 2)},
        {"key": "ledge_above", "label": "들기 후 윗 슬롯 이빨까지", "mm": round(C.SLOT_PITCH - C.LEDGE_T - lifted - t, 2)},
        {"key": "cradle_above", "label": "크래들 진입 웨이퍼 하면 -> 패드", "mm": round(C.CRADLE_ZC_ABOVE + C.BLADE_PLANE_OFS - C.CRADLE_SEAT_Z, 2)},
        {"key": "swing", "label": "선회 중 스테이션 아래", "mm": round(C.SWING_CLEAR_Z - (C.ZC_SWING + C.BLADE_PLANE_OFS + t), 2)},
    ]
