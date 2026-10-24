"""R22 충돌 · 이송 이벤트 · 센서 기하 판정(순수 로직).

시뮬레이터가 '대본'이 아니라 기하로 판단하도록 Z/R 이동 한 구간마다
  - 웨이퍼 이동(집기/놓기: ① 슬롯 · ② 크래들 · ③ 슬롯 · UV 받침), 카세트 이동(대차 받침 <-> 포크 <-> ① 도크) 이벤트와
  - 충돌(포크 · 운반 웨이퍼 띠 vs 슬롯/크래들/받침 웨이퍼, 받침 이빨, 윗레일, 클램프 윗판, UV 구간)
을 계산한다. 반경 방향 겹침은 수치로 미리 따진 결과를 규칙으로 쓴다(R9 몸체 + R22 자리):
  - 후진(RX <= 2) 포크 끝은 R 205 이하. ① 웨이퍼는 257.5 - 50 = 207.5 부터라 빈 포크는 ① 앞에서 자유롭게 오르내린다.
    하지만 후진 운반 웨이퍼(반경 <= 236.5)는 ① 웨이퍼와 27~29 mm 겹친다 -> 든 채 Z 이동은 옆길(θ220)에서.
  - ②③ 웨이퍼(185.5~)는 후진 포크와도 겹친다 -> ③ 은 위 슬롯부터 채운다(R9 와 같음).
  - 받침 이빨(옆 날개 |y| >= 47)은 운반 웨이퍼가 RX > station_r - 222 부터 닿는다(③ 13.5, ① 35.5).
충돌 결과는 (코드, 자리, 슬롯) 튜플(ASCII). 화면 문구는 texts.py. 높이 구간 비교는 경계 접촉을 겹침으로 보지 않는다.
"""
import math
from dataclasses import dataclass, field

from wafer_cell import cell_config as C

EPS = 1e-6
CASS_PLATE_Z = 297.0       # 클램프 윗판 밑면 (R9 도면: 카세트 바닥 172 + 125)
NEAR_STATION_DEG = 25.0    # 후진 포크가 ②③ 웨이퍼(반경 235.5 ± 50)와 겹치는 각도 한계(계산 ±22° + 여유)


@dataclass
class Occupancy:
    cass: dict = field(default_factory=dict)      # 'input'|'ng' -> {n: True} (도크에 카세트가 있을 때만 키 존재)
    blade: bool = False                           # 포크 위 웨이퍼
    inspect: bool = False                         # 검사 크래들 웨이퍼
    uvh: bool = False                             # UV 받침 웨이퍼
    fork_cassette: bool = False                   # 포크가 카세트를 들고 있음
    cart_seat: str = None                         # 로봇 앞 대차 받침: None(대차 없음/정렬 안 됨) | 'free' | 'full'

    def present(self, key):
        return key in self.cass

    def slots(self, key):
        return self.cass.get(key, {})


def station_at(theta):
    """지령 θ 가 스테이션 ±STATION_TOL_DEG 이내면 그 이름, 옆길이면 'side', 아니면 None."""
    for name, ang in C.STATIONS.items():
        if abs(C.wrap_deg(theta - ang)) <= C.STATION_TOL_DEG:
            return name
    if abs(C.wrap_deg(theta - C.SIDE_THETA)) <= C.STATION_TOL_DEG:
        return "side"
    return None


def near_station(theta):
    """후진 포크가 ②③ 웨이퍼와 겹칠 수 있는 각도 안이면 그 이름."""
    for name in ("ng", "inspect"):
        if abs(C.wrap_deg(theta - C.STATIONS[name])) < NEAR_STATION_DEG:
            return name
    return None


def _ov(a, b):
    return a[0] < b[1] - EPS and a[1] > b[0] + EPS


def _sweep(z0, z1, lo, hi):
    return (min(z0, z1) + lo, max(z0, z1) + hi)


def rx_mode(st, rx):
    """'ret' | 'ext' | 'mid'. ② 는 보정 창(δr) 만큼 넓게."""
    if rx <= C.RX_RET_OK + EPS:
        return "ret"
    if st in C.STATION_RX:
        target = C.STATION_RX[st]
        lo, hi = target - C.RX_TOL, target + C.RX_TOL
        if st == "inspect":
            lo, hi = target + C.CORR_DR[0] - C.RX_TOL, target + C.CORR_DR[1] + C.RX_TOL
        if lo - EPS <= rx <= hi + EPS:
            return "ext"
    return "mid"


def ledge_reach_rx(st):
    return C.station_r(st) - 222.0


def _ledges():
    return [(C.slot_z(k) - C.LEDGE_T, C.slot_z(k)) for k in range(2, C.SLOT_COUNT + 1)]


def _rail():
    return (C.CASS_TOP_Z, C.CASS_TOP_Z + 4.0)


def blade_tip(rx):
    return C.BLADE_TIP_X + rx


def z_move(theta, rx, zc0, zc1, occ, t=None):
    """Z 이동 한 구간. 반환 {'events': [(종류, 자리, 슬롯, 교차 ZC)], 'collision': None|(코드, 자리, 슬롯)}.
    종류: pick/place(웨이퍼), cass_pick/cass_place(카세트)."""
    t = C.WAFER_T if t is None else t
    ev = []
    lo, hi = min(zc0, zc1), max(zc0, zc1)
    if lo < C.ZC_MIN - EPS or hi > C.ZC_SOFT_MAX + EPS:
        return {"events": ev, "collision": ("Z_RANGE", None, None)}
    st = station_at(theta)
    if st is None:
        if rx > C.RX_RET_OK + EPS or occ.blade or occ.fork_cassette:
            return {"events": ev, "collision": ("BETWEEN", None, None)}
        near = near_station(theta)
        if near and hi > C.ZC_SWING + 0.25:
            return {"events": ev, "collision": ("BETWEEN", near, None)}
        return {"events": ev, "collision": None}
    if st == "side":
        if rx > C.RX_RET_OK + EPS:
            return {"events": ev, "collision": ("BETWEEN_R", "side", None)}
        return {"events": ev, "collision": None}
    mode = rx_mode(st, rx)
    if mode == "mid":
        return {"events": ev, "collision": ("R_MID", st, None)}
    p0, p1 = zc0 + C.BLADE_PLANE_OFS, zc1 + C.BLADE_PLANE_OFS
    blade_sw = _sweep(zc0, zc1, C.BLADE_BODY[0], C.BLADE_PLANE_OFS)
    carrying = occ.blade

    if st == "cart":
        if mode == "ext":
            if occ.fork_cassette and zc1 < zc0 and zc1 <= C.CASS_SEAT_LIFT_Z <= zc0:
                if occ.cart_seat != "free":
                    return {"events": [], "collision": ("SEAT_FULL" if occ.cart_seat == "full" else "NO_SEAT", "cart", None)}
                ev.append(("cass_place", "cart", 0, C.CASS_SEAT_LIFT_Z))
            elif not occ.fork_cassette and not carrying and zc1 > zc0 and zc0 <= C.CASS_SEAT_LIFT_Z <= zc1 \
                    and occ.cart_seat == "full":
                ev.append(("cass_pick", "cart", 0, C.CASS_SEAT_LIFT_Z))
        return {"events": ev, "collision": None}

    if st == "uv":
        if hi > C.UV_ZC_IN + EPS:
            return {"events": [], "collision": ("UV_TOWER", "uv", None)}
        if mode == "ext":
            if lo < C.UV_ZC_HAND - EPS:
                return {"events": [], "collision": ("UV_ZONE", "uv", None)}
            seat = C.UV_SEAT_Z
            if not carrying and occ.uvh and p0 <= seat < p1:
                ev.append(("pick", "uv", 0, seat - C.BLADE_PLANE_OFS))
            elif carrying and not occ.uvh and p1 < seat <= p0:
                ev.append(("place", "uv", 0, seat - C.BLADE_PLANE_OFS))
            elif occ.uvh and _ov(blade_sw, (seat, seat + t)):
                return {"events": [], "collision": ("FORK_WAFER", "uv", 0)}
        return {"events": ev, "collision": None}

    if st == "inspect":
        seat = C.CRADLE_SEAT_Z
        occupied, moved, carried = occ.inspect, False, None
        if mode == "ext" and not carrying and occupied and p0 <= seat < p1:
            ev.append(("pick", "inspect", 0, seat - C.BLADE_PLANE_OFS))
            moved, carried = True, (seat, p1 + t)
        elif mode == "ext" and carrying and not occupied and p1 < seat <= p0:
            ev.append(("place", "inspect", 0, seat - C.BLADE_PLANE_OFS))
            moved, carried = True, (seat, p0 + t)
        if carried is None and carrying:
            carried = _sweep(p0, p1, 0.0, t)
        if occupied and not moved:
            wb = (seat, seat + t)
            if _ov(blade_sw, wb):
                return {"events": [], "collision": ("FORK_WAFER", "inspect", 0)}
            if carried and _ov(carried, wb):
                return {"events": [], "collision": ("CARRIED_WAFER", "inspect", 0)}
        if mode != "ret" and lo < C.CRADLE_ZC_SEAT - 0.01:
            return {"events": [], "collision": ("BACKLIGHT", "inspect", None)}
        return {"events": ev, "collision": None}

    # 카세트 (① input · ③ ng)
    if occ.fork_cassette:
        if st != "input" or mode != "ext":
            return {"events": [], "collision": ("CASS_Z", st, None)}
        if zc1 < zc0 and zc1 <= C.CASS_LIFT_Z <= zc0:
            if occ.present("input"):
                return {"events": [], "collision": ("DOCK_OCCUPIED", "input", None)}
            ev.append(("cass_place", "input", 0, C.CASS_LIFT_Z))
        return {"events": ev, "collision": None}
    if not occ.present(st):
        return {"events": ev, "collision": None}
    slots = occ.slots(st)
    occupied = [k for k in range(1, C.SLOT_COUNT + 1) if slots.get(k)]
    if mode == "ret" and st == "input" and not carrying:
        return {"events": ev, "collision": None}          # 후진 빈 포크(끝 <= 205)는 ① 웨이퍼(207.5~)에 안 닿음
    plate_zone = mode == "ext" and st == "input" and lo >= C.CASS_Z_UNDER - 1.0
    if plate_zone:
        if not carrying and zc1 > zc0 and zc0 <= C.CASS_LIFT_Z <= zc1:
            ev.append(("cass_pick", "input", 0, C.CASS_LIFT_Z))
        return {"events": ev, "collision": None}
    moved, carried = None, None
    if mode == "ext" and not carrying and p1 > p0:
        for k in occupied:
            sk = C.slot_z(k)
            if p0 <= sk < p1:
                ev.append(("pick", st, k, sk - C.BLADE_PLANE_OFS))
                moved, carried = k, (sk, p1 + t)
                break
    elif mode == "ext" and carrying and p1 < p0:
        for k in sorted(range(1, C.SLOT_COUNT + 1), reverse=True):
            sk = C.slot_z(k)
            if not slots.get(k) and p1 < sk <= p0:
                ev.append(("place", st, k, sk - C.BLADE_PLANE_OFS))
                moved, carried = k, (sk, p0 + t)
                break
    if carried is None and carrying:
        carried = _sweep(p0, p1, 0.0, t)
    for k in occupied:
        if k == moved:
            continue
        wb = (C.slot_z(k), C.slot_z(k) + t)
        if mode != "ret" or st != "input":
            if _ov(blade_sw, wb):
                return {"events": [], "collision": ("FORK_WAFER", st, k)}
        if carried and _ov(carried, wb):
            return {"events": [], "collision": ("CARRIED_WAFER", st, k)}
    if mode == "ext":
        if carried:
            for lb in _ledges():
                if _ov(carried, lb):
                    return {"events": [], "collision": ("CARRIED_LEDGE", st, None)}
        if _ov(blade_sw, _rail()):
            return {"events": [], "collision": ("TOP_RAIL", st, None)}
    return {"events": ev, "collision": None}


def r_move(theta, zc, rx0, rx1, occ, t=None):
    """R 이동 한 구간. 충돌 (코드, 자리, 슬롯) 또는 None."""
    t = C.WAFER_T if t is None else t
    if rx1 <= rx0 + EPS:
        return None                                       # 후진은 들어온 길을 되짚는다
    st = station_at(theta)
    rmax = max(rx0, rx1)
    if rmax > C.RX_MAX + C.RX_TOL:
        return ("R_MAX", st, None)
    if st is None or st == "side":
        return ("BETWEEN_R", st, None) if rmax > C.RX_RET_OK + EPS else None
    if rmax <= C.RX_RET_OK + EPS:
        return None
    reach = C.STATION_RX[st] + (C.CORR_DR[1] if st == "inspect" else 0.0) + C.RX_TOL
    if rmax > reach + EPS:
        return ("R_MAX", st, None)
    bb = (zc + C.BLADE_BODY[0], zc + C.BLADE_PLANE_OFS)
    p = zc + C.BLADE_PLANE_OFS
    carried = (p, p + t) if occ.blade else None
    if st == "cart":
        if occ.fork_cassette and occ.cart_seat != "free":
            return ("SEAT_FULL" if occ.cart_seat == "full" else "NO_SEAT", "cart", None)
        if not occ.fork_cassette and occ.cart_seat is None:
            return ("NO_SEAT", "cart", None)
        return None
    if st == "uv":
        if not C.UV_ZC_HAND - EPS <= zc <= C.UV_ZC_IN + EPS:
            return ("UV_ZONE", "uv", None)
        if occ.uvh:
            wb = (C.UV_SEAT_Z, C.UV_SEAT_Z + t)
            if _ov(bb, wb):
                return ("FORK_WAFER", "uv", 0)
            if carried and _ov(carried, wb):
                return ("CARRIED_WAFER", "uv", 0)
        return None
    if st == "inspect":
        if occ.inspect:
            wb = (C.CRADLE_SEAT_Z, C.CRADLE_SEAT_Z + t)
            if _ov(bb, wb):
                return ("FORK_WAFER", "inspect", 0)
            if carried and _ov(carried, wb):
                return ("CARRIED_WAFER", "inspect", 0)
        if zc < C.CRADLE_ZC_SEAT - 0.01:
            return ("BACKLIGHT", "inspect", None)
        return None
    # 카세트
    if occ.fork_cassette:
        if st == "input" and occ.present("input"):
            return ("DOCK_OCCUPIED", "input", None)
        return None if st == "input" else ("CASS_Z", st, None)
    if not occ.present(st):
        return None
    if st == "input" and p > CASS_PLATE_Z + EPS:
        return ("PLATE", st, None)
    for k in range(1, C.SLOT_COUNT + 1):
        if not occ.slots(st).get(k):
            continue
        wb = (C.slot_z(k), C.slot_z(k) + t)
        if _ov(bb, wb):
            return ("FORK_WAFER", st, k)
        if carried and _ov(carried, wb):
            return ("CARRIED_WAFER", st, k)
    if carried and rmax > ledge_reach_rx(st):
        for lb in _ledges():
            if _ov(carried, lb):
                return ("CARRIED_LEDGE", st, None)
    if _ov(bb, _rail()):
        return ("TOP_RAIL", st, None)
    return None


def s4_on(theta, zc, rx, occ):
    """S4(포크 뿌리 반사형, 위를 봄 - 자리 [보류]). 운반 웨이퍼 또는 S4_RANGE 안의 스테이션 웨이퍼면 ON."""
    if occ.blade:
        return True
    st = station_at(theta)
    if st not in ("input", "ng", "inspect", "uv"):
        return False
    r_s4 = C.BLADE_ORIGIN_X + C.S4_LOCAL_X + rx
    if math.hypot(r_s4 - C.station_r(st), C.S4_LOCAL_Y) > C.WAFER_D / 2:
        return False
    surface = zc + C.BLADE_BODY[1]
    if st == "inspect":
        cands = [C.CRADLE_SEAT_Z] if occ.inspect else []
    elif st == "uv":
        cands = [C.UV_SEAT_Z] if occ.uvh else []
    else:
        cands = [C.slot_z(k) for k, v in occ.slots(st).items() if v]
    return any(C.S4_RANGE[0] <= z - surface <= C.S4_RANGE[1] for z in cands)


def th_mask(theta_true, half=C.TH_LOW_HALF_DEG):
    """θ 자리 센서 5 개(STATION_ORDER 순) LOW 여부. 노치 1 개라 동시에 둘이 LOW 인 θ 는 없다(문서 §1)."""
    return [abs(C.wrap_deg(theta_true - C.STATIONS[k])) <= half for k in C.STATION_ORDER]


def clearances(t=None):
    """문서 · R9 수치로 계산한 여유(mm) - README/현장 확인용."""
    t = C.WAFER_T if t is None else t
    side_w = C.SIDE_OFS + C.BLADE_PLANE_OFS                     # 옆길 높이에서 웨이퍼 하면 (Sn 기준)
    tip = C.BLADE_TIP_X + C.RX_RET_OK
    return [
        {"key": "insert_gap", "label": "카세트 삽입 틈 (포크 면 아래)", "mm": round(-(C.PICK_OFS + C.BLADE_PLANE_OFS), 2)},
        {"key": "side_ledge", "label": "① 옆길 높이: 운반 웨이퍼 윗면 -> 윗 슬롯 이빨", "mm": round(C.SLOT_PITCH - C.LEDGE_T - side_w - t, 2)},
        {"key": "input_tip", "label": "① 후진 포크 끝(RX 2) -> ① 웨이퍼", "mm": round(C.station_r("input") - C.WAFER_D / 2 - tip, 2)},
        {"key": "cradle_above", "label": "② 진입 웨이퍼 하면 -> 패드", "mm": round(C.CRADLE_ZC_ABOVE + C.BLADE_PLANE_OFS - C.CRADLE_SEAT_Z, 2)},
        {"key": "uv_hand", "label": "UV 넘김: 포크 면 -> 받침 (ZC 113.5 / 121.5)",
         "mm": round(min(C.UV_SEAT_Z - (C.UV_ZC_HAND + C.BLADE_PLANE_OFS), C.UV_ZC_IN + C.BLADE_PLANE_OFS - C.UV_SEAT_Z), 2)},
        {"key": "swing", "label": "선회 중 스테이션 아래", "mm": round(C.SWING_CLEAR_Z - (C.ZC_SWING + C.BLADE_PLANE_OFS + t), 2)},
    ]
