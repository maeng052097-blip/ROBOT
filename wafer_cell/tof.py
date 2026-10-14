"""카세트 하단 TOF(VL53L0X) 판정(순수 로직).

도크 바닥(z TOF_Z)에서 위로 쏜 빔이 처음 닿는 면까지의 거리 하나만 읽힌다.
  - 웨이퍼가 있으면 '가장 아래 웨이퍼' 하면(Sn) -> d = Sn - TOF_Z
  - 빈 카세트면 윗레일 밑면(CASS_TOP_Z), 카세트가 없으면 범위 밖(8190)
그래서 아는 것은 '가장 아래 웨이퍼가 몇 번 슬롯인가' 뿐이고, 개수는 위로 빈칸 없이 붙어 있다는 가정의 추정이다.
  - 투입은 아래부터 꺼내므로 매번 '다음에 꺼낼 웨이퍼'를 정확히 본다(중간 빈칸은 S4 없이 건너뜀).
  - OK/NG 는 위부터 채우므로 개수 = 5 - 가장 아래 슬롯 + 1 이 그대로 맞다.
포크가 그 도크 앞에서 전진했거나 웨이퍼/카세트를 든 채 지나가면 빔이 가려진다 -> 측정 무효(마지막 값 유지).
"""
from wafer_cell import cell_config as C
from wafer_cell import geometry as G

BLOCK_HALF_DEG = 10.0      # 포크 폭(±36 mm)이 반경 ~235 에서 차지하는 각도(±8.7°) + 여유


def classify(d):
    """거리(mm) -> {kind: 'slot'|'empty'|'none'|'invalid', slot, count_est}."""
    if d is None or d >= C.TOF_OUT_OF_RANGE:
        return {"kind": "none", "slot": None, "count_est": None}
    for n in range(1, C.SLOT_COUNT + 1):
        if abs(d - C.tof_expected(n)) <= C.TOF_TOL:
            return {"kind": "slot", "slot": n, "count_est": C.SLOT_COUNT - n + 1}
    if abs(d - C.tof_empty_d()) <= C.TOF_TOL:
        return {"kind": "empty", "slot": None, "count_est": 0}
    return {"kind": "invalid", "slot": None, "count_est": None}


def blocked(theta, rx, station, carrying):
    """station('input'|'ng'|'ok') 도크의 빔이 포크에 가려지는가."""
    if abs(G.wrap_deg(theta - C.STATIONS[station])) > BLOCK_HALF_DEG:
        return False
    return rx > G.RX_TOL or carrying


def geometry_problems():
    """장착 기하 점검(빈 목록이면 통과): 최소 거리, 슬롯 간 구분, 빈 카세트 구분."""
    out = []
    if C.tof_expected(1) < C.TOF_MIN_MM:
        out.append(f"slot 1 distance {C.tof_expected(1):g} < VL53L0X min {C.TOF_MIN_MM:g} mm")
    if C.SLOT_PITCH <= 2 * C.TOF_TOL:
        out.append(f"slot pitch {C.SLOT_PITCH:g} <= 2 x tol {C.TOF_TOL:g}")
    gap = C.tof_empty_d() - C.tof_expected(C.SLOT_COUNT)
    if gap <= 2 * C.TOF_TOL:
        out.append(f"empty vs top slot gap {gap:g} <= 2 x tol")
    return out
