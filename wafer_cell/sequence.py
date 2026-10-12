"""R9 공정 시퀀스(순수 로직) — 원점 절차 + 웨이퍼 1장 사이클 + OK 카세트 대차 적재(T1~T13).

비디오 'R9 웨이퍼 검사 셀' 자막 순서를 그대로 단계로 만든다.
  웨이퍼:  ① 투입(180°) 선회 -> 슬롯 n 밑으로 포크 -> 들기(+7.5) · 빼기 -> ② 검사(120°) 선회 -> 크래들에 놓기
           -> StreamCam 촬영 · 판정 -> 다시 들기 -> ④ OK(0°) / ③ NG(60°) 선회 -> 슬롯 k 에 넣기(위 슬롯부터) -> 대기(120°)
  대차:    T1 비킴 선회 -> T2 오르기 -> T3 들어가기 -> T4 클램프 윗판 밑으로 -> T5 들기 -> T7 빼기 -> T8 든 채 선회
           -> T9 내밀기 -> T10 받침에 내려놓기 -> T11 포크 빼기 -> T12 내리기 -> T13 대기   (T6 은 비디오에 없음)
  원점:    R 수축(S3) -> Z 최하단(S2) -> Z105 -> θ1 원점(S1) -> 대기 θ120  (사용자 결정)
선회 규칙: 늘 Z105 · R0 · S3. 예외는 대차 적재 중 T3(Z228.5, 빈 포크)과 T8(Z241, 카세트를 든 채)뿐.
0°(OK) 에 도착할 때마다 S1 로 벨트 이빨 튐을 확인한다(비디오에 없는 보강, 이전 CELL_V9 규칙 유지).
한 단계 안의 여러 동작은 순차 실행한다. 한글 문자열은 화면(브라우저) 전용 — 콘솔 출력 금지.
"""
from dataclasses import dataclass

from wafer_cell import cell_config as C


@dataclass(frozen=True)
class Action:
    kind: str            # 'z' | 'r' | 'rot' | 'expect' | 'capture' | 'check_pick'
    value: float = 0.0
    sensor: str = ""
    expect: bool = True
    via: tuple = ()
    z_ok: float = C.ZC_SWING     # rot 전용: 이 높이에서만 선회 허용


def MoveZ(z):
    return Action("z", float(z))


def MoveR(x):
    return Action("r", float(x))


def Rotate(theta, via=(), z_ok=C.ZC_SWING):
    return Action("rot", float(theta), via=tuple(float(v) for v in via), z_ok=float(z_ok))


def Expect(sensor, value=True):
    return Action("expect", sensor=sensor, expect=bool(value))


def Capture():
    return Action("capture")


def CheckPick():
    """투입 슬롯에서 들기 직후 S4 확인. 없으면 '빈 슬롯 또는 픽업 실패' -> 운전자 확인."""
    return Action("check_pick", sensor="S4", expect=True)


@dataclass(frozen=True)
class Step:
    key: str             # 'H1'..'H5', '1'..'10', 'T1'..'T13'
    group: str           # 'home' | 'input' | 'inspect' | 'sort' | 'cart'
    title: str           # 비디오 큰 자막(기본 화면)
    detail: str          # 비디오 둘째 줄(수치 · 상세에서만)
    actions: tuple


GROUP_LABELS = {"home": "원점", "input": "① 투입", "inspect": "② 검사", "sort": "③ NG · ④ OK", "cart": "대차"}
STAGE_OF_GROUP = {"home": None, "input": "input", "inspect": "inspect", "sort": None, "cart": "cart"}
DEST_KR = {"ok": "④ OK (0°)", "ng": "③ NG (60°)"}


def homing_steps():
    ret, home, swing = C.RX_RET, C.ZC_HOME, C.ZC_SWING
    return [
        Step("H1", "home", "원점 — R 수축 (S3)", "포크를 먼저 빼야 어느 높이에서 멈췄든 카세트를 긁지 않는다",
             (MoveR(ret), Expect("S3", True))),
        Step("H2", "home", "원점 — Z 최하단 (S2)", f"Z {home:g} · 모든 스테이션 아래로 피한다",
             (MoveZ(home), Expect("S2", True))),
        Step("H3", "home", "원점 — 선회 높이 Z 105", "선회는 늘 Z 105 · R 0",
             (MoveZ(swing),)),
        Step("H4", "home", "원점 — θ1 원점 (S1)", "θ1 은 반드시 마지막 (Z 높이를 모른 채 돌지 않는다)",
             (Rotate(0.0), Expect("S1", True))),
        Step("H5", "home", f"대기 (θ{C.THETA_STANDBY:g}° · Z 105 · R 0)", "θ 0 -> 120 은 NG 60° 를 지나간다 (Z 105 · R 0)",
             (Rotate(C.THETA_STANDBY),)),
    ]


def wafer_steps(n):
    """투입 슬롯 n(1..5) 한 장: 투입 -> 검사 -> 다시 들기까지(1~7). 분류(8~10)는 판정 뒤 sort_steps."""
    sn, pz = C.slot_z(n), C.pick_z(n)
    ext, ret, swing = C.RX_EXT, C.RX_RET, C.ZC_SWING
    above, seat = C.CRADLE_ZC_ABOVE, C.CRADLE_ZC_SEAT
    return [
        Step("1", "input", "① 투입 (180°) 로 선회", f"θ{C.THETA_STANDBY:g} -> 180 · Z 105 · R 0 (선회는 늘 Z 105 · R 0)",
             (Expect("S3", True), Rotate(C.STATIONS["input"]))),
        Step("2", "input", f"① 투입 슬롯 {n} 밑으로 포크 넣기",
             f"Z {pz:g} (슬롯 바닥 {sn:g} - {-C.PICK_OFS:g}) -> R 0 -> {ext:g} (실린더 2 개 같이 밈)",
             (MoveZ(pz), MoveR(ext))),
        Step("3", "input", f"① 들기 (+{C.PICK_LIFT:g}) -> 빼기",
             f"웨이퍼가 받침 이빨에서 {C.PICK_LIFT + C.PICK_OFS + C.BLADE_PLANE_OFS:g} 떠서 앞 턱 ({C.FRONT_LIP:g}) 을 넘음",
             (MoveZ(pz + C.PICK_LIFT), CheckPick(), MoveR(ret), Expect("S3", True), MoveZ(swing))),
        Step("4", "inspect", "② 검사 (120°) 로 선회", "θ 180 -> 120 · Z 105 · R 0",
             (Expect("S3", True), Rotate(C.STATIONS["inspect"]))),
        Step("5", "inspect", "② 검사 크래들에 놓기",
             f"Z {above:g} -> R {ext:g} -> Z {seat:g} (패드 윗면 {C.CRADLE_SEAT_Z:g} 에 내려놓음) -> R 0",
             (MoveZ(above), MoveR(ext), MoveZ(seat), Expect("S4", False), MoveR(ret), Expect("S3", True))),
        Step("6", "inspect", "② StreamCam 촬영 · 판정", "포크를 뺀 채 위에서 찍음 (백라이트 C-03N 위 크래들 C-02N)",
             (Capture(),)),
        Step("7", "inspect", "② 다시 들기", f"R {ext:g} -> Z {above:g} (S4) -> R 0 -> Z 105",
             (MoveR(ext), MoveZ(above), Expect("S4", True), MoveR(ret), Expect("S3", True), MoveZ(swing))),
    ]


def retry_steps(n):
    """S4 없음 -> '재시도': 그 자리에서 내려놓았다가(웨이퍼가 있었다면 슬롯에 다시 놓임) 다시 들기, 이후 4~7."""
    pz, swing = C.pick_z(n), C.ZC_SWING
    return [Step("3", "input", f"① 다시 집기 (투입 슬롯 {n})", f"Z {pz:g} -> {pz + C.PICK_LIFT:g} (S4) -> R 0 -> Z 105",
                 (MoveZ(pz), MoveZ(pz + C.PICK_LIFT), CheckPick(), MoveR(C.RX_RET), Expect("S3", True), MoveZ(swing)))
            ] + wafer_steps(n)[3:]


def sort_steps(dest, k):
    """판정 뒤: dest('ok'|'ng') 카세트 슬롯 k 에 넣고 대기(120°)로(8~10). 위 슬롯부터 채운다."""
    sk, az, pz = C.slot_z(k), C.place_z(k), C.pick_z(k)
    ext, ret, swing = C.RX_EXT, C.RX_RET, C.ZC_SWING
    via = (C.STATIONS["ng"],) if dest == "ok" else ()
    pass_note = " 은 NG 60° 를 지나감 (Z 105 · R 0)" if dest == "ok" else " (Z 105 · R 0)"
    return [
        Step("8", "sort", f"{DEST_KR[dest]} 로 선회", f"θ120 -> {C.STATIONS[dest]:g}{pass_note}",
             (Expect("S3", True), Rotate(C.STATIONS[dest])) + ((Expect("S1", True),) if dest == "ok" else ())),
        Step("9", "sort", f"{DEST_KR[dest]} 슬롯 {k} 에 넣기 (위 슬롯부터)",
             f"Z {az:g} -> R {ext:g} -> Z {pz:g} (내려놓음) -> R 0",
             (MoveZ(az), MoveR(ext), MoveZ(pz), Expect("S4", False), MoveR(ret), Expect("S3", True), MoveZ(swing))),
        Step("10", "sort", f"대기 ({C.THETA_STANDBY:g}°)", f"θ {C.STATIONS[dest]:g} -> {C.THETA_STANDBY:g} · Z 105 · R 0",
             (Expect("S3", True), Rotate(C.THETA_STANDBY))),
    ]


def empty_slot_steps():
    """S4 없음 -> 운전자가 '빈 슬롯'으로 확정: 빼고 Z105 로 내려 다음 슬롯으로(대기 120 은 거치지 않음)."""
    return [Step("3", "input", "① 빈 슬롯 - 포크 빼기", "R 0 -> Z 105",
                 (MoveR(C.RX_RET), Expect("S3", True), MoveZ(C.ZC_SWING)))]


def cart_steps(seat, nth):
    """OK 카세트(가득) -> 대차 받침 seat. nth = 대차의 몇 번째 카세트(1..4)."""
    ret, swing = C.RX_RET, C.ZC_SWING
    under, carry, seat_z = C.CART_Z_UNDER, C.CART_Z_CARRY, C.CART_Z_SEAT
    side, cart = C.CART_SIDE_THETA, C.STATIONS["cart"]
    p = f"대차 {nth} 번째 — "
    seat_kr = C.CART_SEAT_KR[seat]
    return [
        Step("T1", "cart", p + f"T1 비킴 선회 θ{C.THETA_STANDBY:g} -> {side:g}",
             "OK 도크에서 40° 비켜서 오름 (클램프 윗판 앞끝과 포크 끝이 부딪히지 않게)",
             (Expect("S3", True), Rotate(side))),
        Step("T2", "cart", p + f"T2 오르기 Z {under:g}", "클램프 윗판 밑 높이",
             (MoveZ(under),)),
        Step("T3", "cart", p + f"T3 들어가기 θ {side:g} -> 0", f"Z {under:g} · R 0 (빈 포크)",
             (Expect("S3", True), Rotate(C.STATIONS["ok"], z_ok=under), Expect("S1", True))),
        Step("T4", "cart", p + f"T4 클램프 윗판 밑으로 R 0 -> {C.RX_EXT:g}", "포크가 C-08K1S 앞 판 밑에 들어감",
             (MoveR(C.RX_EXT),)),
        Step("T5", "cart", p + f"T5 들기 Z {carry:g}", "카세트 + 웨이퍼 5 장을 클램프 윗판째로 듦",
             (MoveZ(carry),)),
        Step("T7", "cart", p + f"T7 빼기 R {C.RX_EXT:g} -> 0", "카세트를 든 채 OK 도크에서 빠짐",
             (MoveR(ret), Expect("S3", True))),
        Step("T8", "cart", p + f"T8 든 채 선회 θ 0 -> {cart:g}", f"Z {carry:g} · R 0 에서만",
             (Expect("S3", True), Rotate(cart, z_ok=carry))),
        Step("T9", "cart", p + f"T9 내밀기 R 0 -> {C.RX_CART:g}", f"받침 자리 한 곳 (θ {cart:g} · R {C.RX_CART:g}) · {seat_kr}",
             (MoveR(C.RX_CART),)),
        Step("T10", "cart", p + "T10 받침에 내려놓기", f"Z {carry:g} -> {seat_z:g}",
             (MoveZ(seat_z),)),
        Step("T11", "cart", p + "T11 포크 빼기", f"R {C.RX_CART:g} -> 0",
             (MoveR(ret), Expect("S3", True))),
        Step("T12", "cart", p + "T12 내리기", f"Z {seat_z:g} -> 105",
             (MoveZ(swing),)),
        Step("T13", "cart", p + f"T13 대기 ({C.THETA_STANDBY:g}°)", f"θ {cart:g} -> {C.THETA_STANDBY:g} · Z 105 · R 0",
             (Expect("S3", True), Rotate(C.THETA_STANDBY))),
    ]


def can_rotate(zc, rx, s3, z_ok=C.ZC_SWING, carrying=False):
    """선회 인터록. (허용 여부, 사유). 기본은 Z105 · R0 · S3, 대차 적재 T3/T8 만 지정 높이."""
    reasons = []
    if abs(zc - z_ok) > 0.25:
        reasons.append(f"Z {zc:.1f} (선회는 Z {z_ok:g}에서만)")
    if abs(rx - C.RX_RET) > 0.5:
        reasons.append(f"R {rx:.1f} (수축 아님)")
    if not s3:
        reasons.append("S3 후진 미확인")
    if carrying and z_ok == C.ZC_SWING and (zc + C.BLADE_PLANE_OFS + C.WAFER_T) >= C.SWING_CLEAR_Z:
        reasons.append("운반 웨이퍼가 스테이션 높이")
    if reasons:
        return False, "선회 금지: " + ", ".join(reasons)
    return True, "선회 허용"


def step_catalog():
    """타임라인/툴팁용 단계 목록(슬롯 무관 표기)."""
    out = []
    for st in homing_steps():
        out.append({"key": st.key, "group": st.group, "title": st.title, "detail": st.detail})
    for st in wafer_steps(1) + sort_steps("ok", 5):
        title = st.title.replace("슬롯 1 ", "슬롯 n ").replace("슬롯 5 ", "슬롯 k ").replace(DEST_KR["ok"], "OK/NG")
        out.append({"key": st.key, "group": st.group, "title": title, "detail": st.detail})
    for st in cart_steps("FL", 1):
        out.append({"key": st.key, "group": st.group, "title": st.title.replace("대차 1 번째 — ", ""),
                    "detail": st.detail})
    return out
