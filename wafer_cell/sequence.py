"""R22 공정 시퀀스(순수 로직) — 원점 + 대차->① + 웨이퍼 1 장(검사 · 불량 또는 UV · 원위치) + ①->대차.

사용자가 설명한 흐름(2026-10-24):
  대차에서 카세트를 꺼내 ① 투입에 놓음 -> 슬롯 1 -> 5 (아래부터) 웨이퍼마다
      집기 -> ② 검사(촬영 · 판정 · 치우침 δr, δθ) -> 보정해 다시 집기
      -> 불량이면 ③ 불량에 넣고(위 슬롯부터) 다음 웨이퍼
      -> 정상이면 θ0 UV 탑 받침에 넘김 -> UV 켜고 끔 -> 다시 집어 -> ① 원래 슬롯에 되돌림
  -> 5 장이 끝나면 ① 카세트를 들어 원래 대차 받침에 놓고 대차가 움직인다.
R22 문서 규칙:
  - ① 은 RX 73 · ②③ 51 · UV · 대차 85. 옆길 θ180 <-> 220 은 RX <= 2 에서만.
  - UV 탑 밑(RX > 2)에서는 ZC 113.5 ~ 121.5 만. 램프는 7 가지 조건(uv_interlock)일 때만.
  - S4: ① 들기(P3) 뒤 ON = 꺼냄 / OFF = 빈 슬롯으로 기록하고 다음 슬롯(E1). UV 넘긴 뒤(U5) OFF, 다시 집은 뒤(U7) ON,
    ① 되돌린 뒤(R5) OFF. '없음' 확인은 포크를 뺀 뒤(놓은 직후에는 아직 보임).
  - 원점: R 후진 -> Z 선회 높이(Z Sharp) -> θ 를 - 쪽으로 5°/s, 95° 안 첫 LOW = 그 자리 -> 대기.
    (문서는 Z 를 먼저 적었지만 포크가 카세트 안이면 긁으므로 R 먼저 - R9 에서 정한 순서)
해석(문서에 없는 부분, 사용자 확인 필요): ① 에서 웨이퍼를 든 채 Z 를 바꾸는 일은 모두 옆길(θ220)에서 한다
(후진한 운반 웨이퍼가 ① 아래 슬롯 웨이퍼와 반경 27 mm 겹침). 카세트 운반은 슬립링 짧은 길 180 <-> 220 <-> 270.
한 단계 안의 동작은 순차 실행한다. 한글 문자열은 화면(브라우저) 전용 - 콘솔 출력 금지.
"""
from dataclasses import dataclass

from wafer_cell import cell_config as C

ROT_MODES = ("swing", "side", "carry", "fine")


@dataclass(frozen=True)
class Action:
    kind: str            # z | r | rot | expect | expect_th | ret | vz | vr | capture | check_pick | uv | home_z | home_th
    value: float = 0.0
    sensor: str = ""
    expect: bool = True
    mode: str = "swing"  # rot 전용
    corr: bool = False   # r/rot: 마지막 판정의 δr/δθ 를 더함


def MoveZ(z):
    return Action("z", float(z))


def MoveR(x, corr=False):
    return Action("r", float(x), corr=corr)


def Rotate(theta, mode="swing", corr=False):
    assert mode in ROT_MODES
    return Action("rot", float(theta), mode=mode, corr=corr)


def Expect(sensor, value=True):
    return Action("expect", sensor=sensor, expect=bool(value))


def ExpectTh(station):
    """그 자리 θ 센서만 LOW (1 개 = 그 자리, 0 개 = 자리 사이, 2 개 이상 = 고장)."""
    return Action("expect_th", sensor=station)


def ExpectRet():
    """R Sharp 로 RX <= 2 (후진) 확인."""
    return Action("ret", sensor="R")


def VerifyZ():
    return Action("vz", sensor="Z")


def VerifyR():
    return Action("vr", sensor="R")


def Capture():
    return Action("capture")


def CheckPick():
    """① 들기 직후 S4: ON = 꺼냄, OFF = 빈 슬롯(기록하고 다음 슬롯 - 알람 아님, 문서 §5)."""
    return Action("check_pick", sensor="S4", expect=True)


def UvExpose():
    return Action("uv")


def HomeZ():
    return Action("home_z", sensor="Z")


def HomeTheta():
    return Action("home_th")


@dataclass(frozen=True)
class Step:
    key: str
    group: str
    title: str           # 기본 화면 한 줄
    detail: str          # 수치(상세에서만)
    actions: tuple


GROUP_LABELS = {"home": "원점 · 대기", "cart_in": "대차 -> ①", "input": "① 꺼내기", "inspect": "② 검사",
                "ng": "③ 불량", "uv": "UV", "ret": "① 원위치", "cart_out": "① -> 대차"}
STAGE_OF_GROUP = {"home": None, "cart_in": "cart", "input": "input", "inspect": "inspect", "ng": "ng",
                  "uv": "uv", "ret": "ret", "cart_out": "cart"}
STAGES = [("cart", "대차"), ("input", "투입"), ("inspect", "검사"), ("ng", "불량"), ("uv", "UV"), ("ret", "원위치")]

RXI, RXE, RXU, RXC = C.STATION_RX["input"], C.STATION_RX["inspect"], C.STATION_RX["uv"], C.STATION_RX["cart"]
SW, SIDE = C.ZC_SWING, C.SIDE_THETA


def homing_steps():
    return [
        Step("H1", "home", "원점 — R 후진", f"RX {C.RX_HOME:g} (R Sharp 로 RX <= {C.RX_RET_OK:g} 확인) · 포크를 먼저 빼야 어디서 멈췄든 긁지 않는다",
             (MoveR(C.RX_HOME), ExpectRet())),
        Step("H2", "home", "원점 — Z 선회 높이 (Z Sharp)", f"Z Sharp 로 높이를 읽고 ZC {SW:g} (거리 {C.z_sharp_d(SW):.1f} mm)",
             (HomeZ(), MoveZ(SW), VerifyZ())),
        Step("H3", "home", "원점 — θ 자리 찾기", f"- 쪽으로 {C.THETA_HOME_SPEED:g}°/s · {C.THETA_HOME_RANGE:g}° 안 첫 LOW 가 그 자리 (없거나 2 개면 멈춤)",
             (HomeTheta(),)),
        Step("H4", "home", f"대기 (θ{C.THETA_STANDBY:g} · Z {SW:g} · R 후진)", "선회는 Z 105 · RX <= 2",
             (Rotate(C.THETA_STANDBY), ExpectTh("inspect"))),
    ]


def standby_steps():
    return [Step("W", "home", f"대기 (θ{C.THETA_STANDBY:g}) 로", f"R 후진 -> Z {SW:g} -> θ{C.THETA_STANDBY:g}",
                 (MoveR(C.RX_RET), ExpectRet(), MoveZ(SW), Rotate(C.THETA_STANDBY), ExpectTh("inspect")))]


def _seat_t(seat):
    return f" ({C.CART_SEAT_KR[seat]})" if seat else ""


def cart_in_steps(seat):
    st_ = _seat_t(seat)
    under, carry, seat_z = C.CASS_Z_UNDER, C.CASS_Z_CARRY, C.CASS_Z_SEAT
    return [
        Step("C1", "cart_in", f"대차 받침{st_}으로 선회", f"θ {C.STATIONS['cart']:g} · Z {SW:g} · R 후진",
             (Rotate(C.STATIONS["cart"]), ExpectTh("cart"))),
        Step("C2", "cart_in", "받침 카세트 밑으로 포크", f"Z {seat_z:g} -> RX {RXC:g}",
             (MoveZ(seat_z), VerifyZ(), MoveR(RXC), VerifyR())),
        Step("C3", "cart_in", "카세트 들기 -> 빼기", f"Z {carry:g} (클램프 윗판째로) -> R 후진",
             (MoveZ(carry), VerifyZ(), MoveR(C.RX_RET), ExpectRet())),
        Step("C4", "cart_in", "든 채 ① 로 선회 (270 -> 220 -> 180)", f"Z {carry:g} · RX <= 2 · ① 도크 비었음 (TOF-IN)",
             (Expect("TOF_IN", False), Rotate(C.STATIONS["input"], "carry"), ExpectTh("input"))),
        Step("C5", "cart_in", "① 도크에 내려놓기", f"RX {RXI:g} -> Z {under:g} -> R 후진 · TOF-IN 있음 -> Z {SW:g}",
             (MoveR(RXI), VerifyR(), MoveZ(under), VerifyZ(), MoveR(C.RX_RET), ExpectRet(), Expect("TOF_IN", True),
              MoveZ(SW))),
    ]


def input_steps(n):
    sn, pz, sz = C.slot_z(n), C.pick_z(n), C.side_z(n)
    return [
        Step("P1", "input", "① 투입 (180°) 으로 선회", f"Z {SW:g} · R 후진 · TOF-IN 카세트 있음",
             (Rotate(C.STATIONS["input"]), ExpectTh("input"), Expect("TOF_IN", True))),
        Step("P2", "input", f"① 슬롯 {n} 밑으로 포크", f"Z {pz:g} (슬롯 바닥 {sn:g} - {-C.PICK_OFS:g}) -> RX {RXI:g}",
             (MoveZ(pz), VerifyZ(), MoveR(RXI), VerifyR())),
        Step("P3", "input", f"① 슬롯 {n} 들기 (S4) -> 빼기", f"Z {sz:g} (옆길 높이) · S4 ON = 꺼냄 / OFF = 빈 슬롯 -> R 후진",
             (MoveZ(sz), CheckPick(), MoveR(C.RX_RET), ExpectRet())),
    ]


def empty_steps(n):
    return [Step("E1", "input", f"① 슬롯 {n} 빈 슬롯 — 빼고 내리기", f"R 후진 -> 옆길 θ{SIDE:g} -> Z {SW:g} (다음 슬롯)",
                 (MoveR(C.RX_RET), ExpectRet(), Rotate(SIDE, "side"), MoveZ(SW)))]


def inspect_steps(n):
    sz = C.side_z(n)
    above, seat = C.CRADLE_ZC_ABOVE, C.CRADLE_ZC_SEAT
    return [
        Step("I1", "inspect", "옆길로 비켜 내린 뒤 ② 검사 (120°) 로 선회",
             f"θ 180 -> {SIDE:g} (Z {sz:g} · RX <= 2) -> Z {SW:g} -> θ 120",
             (Rotate(SIDE, "side"), MoveZ(SW), VerifyZ(), Rotate(C.STATIONS["inspect"]), ExpectTh("inspect"))),
        Step("I2", "inspect", "② 검사 크래들에 놓기", f"Z {above:g} -> RX {RXE:g} -> Z {seat:g} (패드 {C.CRADLE_SEAT_Z:g}) -> R 후진 · S4 없음",
             (MoveZ(above), MoveR(RXE), VerifyR(), MoveZ(seat), MoveR(C.RX_RET), ExpectRet(), Expect("S4", False))),
        Step("I3", "inspect", "② StreamCam 촬영 · 판정 (PC)", f"OK / NG + 치우침 δr, δθ · {C.JUDGE_TIMEOUT_S:g} s 안에 판정",
             (Capture(),)),
        Step("I4", "inspect", "② 보정해서 다시 집기", f"θ 120 + δθ · RX {RXE:g} + δr -> Z {above:g} (S4) -> R 후진 -> Z {SW:g}",
             (Rotate(C.STATIONS["inspect"], "fine", corr=True), MoveR(RXE, corr=True), VerifyR(), MoveZ(above),
              Expect("S4", True), MoveR(C.RX_RET), ExpectRet(), MoveZ(SW))),
    ]


def ng_steps(k):
    sk, az, pz = C.slot_z(k), C.place_z(k), C.pick_z(k)
    return [
        Step("N1", "ng", "③ 불량 (60°) 으로 선회", f"θ 120 -> 60 · Z {SW:g} · TOF-NG 카세트 있음",
             (Rotate(C.STATIONS["ng"]), ExpectTh("ng"), Expect("TOF_NG", True))),
        Step("N2", "ng", f"③ 슬롯 {k} 에 넣기 (위 슬롯부터)", f"Z {az:g} -> RX {RXE:g} -> Z {pz:g} -> R 후진 · S4 없음 -> Z {SW:g}",
             (MoveZ(az), VerifyZ(), MoveR(RXE), VerifyR(), MoveZ(pz), MoveR(C.RX_RET), ExpectRet(), Expect("S4", False),
              MoveZ(SW))),
    ]


def uv_steps():
    zin, zh = C.UV_ZC_IN, C.UV_ZC_HAND
    return [
        Step("U1", "uv", "UV 탑 (0°) 으로 선회", f"θ 120 -> 0 · Z {SW:g} · R 후진",
             (Rotate(C.STATIONS["uv"]), ExpectTh("uv"))),
        Step("U2", "uv", f"UV 넣기 높이 ZC {zin:g}", f"Z Sharp {C.z_sharp_d(zin):.0f} mm",
             (MoveZ(zin), VerifyZ())),
        Step("U3", "uv", f"UV 탑 밑으로 RX {RXU:g}", f"탑 밑(RX > 2)에서는 ZC {zh:g} ~ {zin:g} 만",
             (MoveR(RXU), VerifyR())),
        Step("U4", "uv", f"UV 받침에 넘기기 ZC {zh:g}", f"받침 {C.UV_SEAT_Z:g} (가정)",
             (MoveZ(zh), VerifyZ())),
        Step("U5", "uv", "포크 빼기 (S4 없음)", "넘김 = S4 ON -> OFF",
             (MoveR(C.RX_RET), ExpectRet(), Expect("S4", False))),
        Step("U6", "uv", f"UV 조사 {C.UV_S:g} s", f"Z {SW:g} (ZC <= {C.UV_ZC_LAMP_MAX:g}) · 조건 7 가지 · A_UV {C.UV_AUV_S:g} s 안 확인",
             (MoveZ(SW), UvExpose())),
        Step("U7", "uv", "UV 받침에서 다시 집기", f"Z {zh:g} -> RX {RXU:g} -> Z {zin:g} (S4) -> R 후진 -> Z {SW:g}",
             (MoveZ(zh), MoveR(RXU), VerifyR(), MoveZ(zin), Expect("S4", True), MoveR(C.RX_RET), ExpectRet(), MoveZ(SW))),
    ]


def return_steps(n):
    sn, pz, sz = C.slot_z(n), C.pick_z(n), C.side_z(n)
    return [
        Step("R1", "ret", "옆길 (θ220) 로 선회", f"θ 0 -> 60 -> 120 -> 180 -> {SIDE:g} · Z {SW:g} (긴 길 - 대차 쪽을 지나지 않음)",
             (Rotate(SIDE),)),
        Step("R2", "ret", f"옆길에서 슬롯 {n} 높이로", f"Z {sz:g} (θ{SIDE:g} · RX <= 2)",
             (MoveZ(sz), VerifyZ())),
        Step("R3", "ret", "옆길 -> ① (180°)", f"Z {sz:g} · RX <= 2 · TOF-IN 카세트 있음",
             (Rotate(C.STATIONS["input"], "side"), ExpectTh("input"), Expect("TOF_IN", True))),
        Step("R4", "ret", f"① 원래 슬롯 {n} 에 넣기", f"RX {RXI:g} -> Z {pz:g} (슬롯 바닥 {sn:g})",
             (MoveR(RXI), VerifyR(), MoveZ(pz))),
        Step("R5", "ret", "포크 빼고 내리기 (S4 없음)", f"R 후진 -> Z {SW:g}",
             (MoveR(C.RX_RET), ExpectRet(), Expect("S4", False), MoveZ(SW))),
    ]


def cart_out_steps(seat):
    st_ = _seat_t(seat)
    under, carry, seat_z = C.CASS_Z_UNDER, C.CASS_Z_CARRY, C.CASS_Z_SEAT
    return [
        Step("L1", "cart_out", f"① 클램프 윗판 밑 높이 Z {under:g}",
             "θ 180 (마지막 웨이퍼가 ③ 이면 60 에서 옴) · TOF-IN 카세트 있음 · 빈 포크라 ① 앞에서 바로 오름",
             (Rotate(C.STATIONS["input"]), ExpectTh("input"), Expect("TOF_IN", True), MoveZ(under), VerifyZ())),
        Step("L2", "cart_out", f"클램프 윗판 밑으로 RX {RXI:g}", "포크가 윗판 밑에 들어감",
             (MoveR(RXI), VerifyR())),
        Step("L3", "cart_out", f"카세트 들기 Z {carry:g} -> 빼기", "웨이퍼째로 듦 -> R 후진",
             (MoveZ(carry), VerifyZ(), MoveR(C.RX_RET), ExpectRet())),
        Step("L4", "cart_out", f"든 채 대차{st_}로 선회 (180 -> 220 -> 270)", f"Z {carry:g} · RX <= 2",
             (Rotate(C.STATIONS["cart"], "carry"), ExpectTh("cart"))),
        Step("L5", "cart_out", "대차 받침에 내려놓기", f"RX {RXC:g} -> Z {seat_z:g}",
             (MoveR(RXC), VerifyR(), MoveZ(seat_z), VerifyZ())),
        Step("L6", "cart_out", "포크 빼고 내리기", f"R 후진 -> Z {SW:g}",
             (MoveR(C.RX_RET), ExpectRet(), MoveZ(SW))),
        Step("L7", "cart_out", f"대기 (θ{C.THETA_STANDBY:g}) — 대차 이동", "대차가 움직이는 동안 로봇은 대기",
             (Rotate(C.THETA_STANDBY), ExpectTh("inspect"))),
    ]


# ------------------------------------------------------------ 인터록 (순수 함수)
def can_rotate(zc, rx_ret_ok, th_from, th_to, mode="swing", carrying_wafer=False, carrying_cassette=False):
    """선회 인터록. (허용 여부, 사유 코드 목록). 사유 코드는 ASCII(화면 문구는 texts.py).
    rx_ret_ok = R Sharp 로 RX <= 2 확인됨."""
    why = []
    lo, hi = min(th_from, th_to), max(th_from, th_to)
    if not rx_ret_ok:
        why.append("R_NOT_RET")
    if lo < C.THETA_FRAME[0] - 0.01 or hi > C.THETA_FRAME[1] - 0.01:
        why.append("THETA_RANGE")
    if mode == "swing":
        if abs(zc - C.ZC_SWING) > 0.25:
            why.append("Z_NOT_SWING")
        if carrying_cassette:
            why.append("CASS_SWING")
        if carrying_wafer and zc + C.BLADE_PLANE_OFS + C.WAFER_T >= C.SWING_CLEAR_Z:
            why.append("WAFER_HIGH")
    elif mode == "side":
        if lo < C.STATIONS["input"] - 0.5 or hi > C.SIDE_THETA + 0.5:
            why.append("SIDE_PATH")
        if carrying_cassette:
            why.append("CASS_SIDE")
        if not C.ZC_SWING - 0.25 <= zc <= C.ZC_SOFT_MAX:
            why.append("Z_RANGE")
    elif mode == "carry":
        if lo < C.STATIONS["input"] - 0.5 or hi > C.STATIONS["cart"] + 0.5:
            why.append("CARRY_PATH")
        if abs(zc - C.CASS_Z_CARRY) > 0.25:
            why.append("Z_NOT_CARRY")
    elif mode == "fine":
        if hi - lo > C.CORR_DTH + 1e-6:
            why.append("FINE_RANGE")
        if abs(zc - C.CRADLE_ZC_SEAT) > 0.25 or abs(C.wrap_deg(th_from - C.STATIONS["inspect"])) > C.STATION_TOL_DEG:
            why.append("FINE_POSE")
    else:
        why.append("MODE")
    return (not why), why


def uv_interlock(th0_low, theta, rx_ret_ok, zc, handed_over, judgment_ok, d9_ok):
    """UV 램프 켜기 7 조건(문서 §7). 못 채운 항목 코드 목록(빈 목록 = 켜도 됨)."""
    miss = []
    if not th0_low:
        miss.append("TH0")
    if abs(C.wrap_deg(theta - C.STATIONS["uv"])) > C.UV_THETA_TOL:
        miss.append("ANGLE")
    if not rx_ret_ok:
        miss.append("RX")
    if zc > C.UV_ZC_LAMP_MAX + 1e-6:
        miss.append("ZC")
    if not handed_over:
        miss.append("HAND")
    if not judgment_ok:
        miss.append("JUDGE")
    if not d9_ok:
        miss.append("D9")
    return miss


def corr_ok(dr, dth):
    return C.CORR_DR[0] - 1e-9 <= dr <= C.CORR_DR[1] + 1e-9 and abs(dth) <= C.CORR_DTH + 1e-9


# ------------------------------------------------------------ 목록
def all_steps(n=1, k=C.SLOT_COUNT, seat=None):
    return (homing_steps() + cart_in_steps(seat) + input_steps(n) + empty_steps(n) + inspect_steps(n)
            + ng_steps(k) + uv_steps() + return_steps(n) + cart_out_steps(seat) + standby_steps())


def describe(key, n=None, k=None, seat=None):
    """단계 키 -> (group, title, detail). 실장비(LIVE)는 키와 슬롯 번호만 받으므로 같은 문구를 여기서 만든다."""
    for st in all_steps(n or 1, k or C.SLOT_COUNT, seat):
        if st.key == key:
            title = st.title
            if n is None:
                title = title.replace("슬롯 1 ", "슬롯 n ")
            if k is None and st.group == "ng":
                title = title.replace(f"슬롯 {C.SLOT_COUNT} ", "슬롯 k ")
            return st.group, title, st.detail
    return None


def step_catalog():
    """타임라인/툴팁용 단계 목록(슬롯 무관 표기)."""
    return [dict(zip(("group", "title", "detail"), describe(st.key)), key=st.key) for st in all_steps()]


def step_keys():
    return [st.key for st in all_steps()]


def action_at(key, ai, n=None, k=None, seat=None):
    """단계 키 + 동작 번호 -> Action (실장비 화면에서 '지금 무엇을 기다리나' 를 보이려고). 없으면 None."""
    for st in all_steps(n or 1, k or C.SLOT_COUNT, seat):
        if st.key == key:
            return st.actions[ai] if ai is not None and 0 <= ai < len(st.actions) else None
    return None


def action_count(key):
    for st in all_steps():
        if st.key == key:
            return len(st.actions)
    return 0
