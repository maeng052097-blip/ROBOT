"""카세트 최상단 TOF 거리 -> 최상단 슬롯/빈 카세트 판정(순수 로직).

TOF 는 '가장 가까운 면 = 최상단 웨이퍼'까지의 거리만 잰다. 아래 웨이퍼는 가려진다.
개수 = 최상단 슬롯 번호라고 보려면 '슬롯 1 부터 빈칸 없이 연속 투입' 가정이 필요하다(사용자 결정).
판정 순서(검토 반영): ① 무효 범위 ② 빈 카세트 먼저 ③ 슬롯1/빈 카세트 구분 불가 구간 ④ 슬롯 번호 + 잔차.
기준 거리는 현장 보정값(d1 = 슬롯 1 만 넣었을 때, d_empty = 빈 카세트)을 우선 쓴다.
"""
from wafer_cell import cell_config as C


def default_refs(t=None):
    """보정값이 없을 때 문서 기하(가정 포함)로 계산한 (d1, d_empty)."""
    t = C.WAFER_T if t is None else t
    d1 = C.TOF_D1 if C.TOF_D1 is not None else C.TOF_Z - (C.slot_z(1) + t)
    de = C.TOF_D_EMPTY if C.TOF_D_EMPTY is not None else C.TOF_Z - C.TOF_FLOOR_Z
    return d1, de


def expected_distance(top_slot, t=None, floor_z=None):
    """시뮬용: 최상단 슬롯(0=빈 카세트)일 때 센서가 볼 거리."""
    t = C.WAFER_T if t is None else t
    if top_slot <= 0:
        return C.TOF_Z - (C.TOF_FLOOR_Z if floor_z is None else floor_z)
    return C.TOF_Z - (C.slot_z(top_slot) + t)


def classify(d, d1=None, d_empty=None, tol=None, pitch=None, slots=None):
    """거리 d(mm) 판정. 반환 {'kind': EMPTY|AMBIG01|SLOT|INVALID, 'n': int|None, 'residual': float|None}."""
    ref1, refe = default_refs()
    d1 = ref1 if d1 is None else d1
    d_empty = refe if d_empty is None else d_empty
    tol = C.TOF_TOL if tol is None else tol
    pitch = C.SLOT_PITCH if pitch is None else pitch
    slots = C.SLOT_COUNT if slots is None else slots
    if d is None or d != d or d < C.TOF_MIN_RANGE:
        return {"kind": "INVALID", "n": None, "residual": None}
    separable = (d_empty - d1) >= 2 * tol
    if separable and d >= d_empty - tol:
        return {"kind": "EMPTY", "n": 0, "residual": round(d - d_empty, 2)}
    if not separable and d >= d1 - tol:
        return {"kind": "AMBIG01", "n": None, "residual": None}
    nf = (d1 - d) / pitch + 1.0
    n = int(round(nf))
    res = (nf - n) * pitch
    if n < 1:
        return {"kind": "AMBIG01", "n": None, "residual": None}
    if n > slots or abs(res) > tol:
        return {"kind": "INVALID", "n": None, "residual": round(res, 2)}
    return {"kind": "SLOT", "n": n, "residual": round(res, 2)}
