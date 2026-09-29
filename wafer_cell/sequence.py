"""CELL_V9 공정 시퀀스(순수 로직) — 원점 절차 + 장당 21단계를 데이터로 만든다.

문서 §A 표를 그대로 수치화하되 다음을 보강했다(사용자 승인):
  - 원점: R 수축(S3) -> Z 최하단(S2) -> ZC105 -> θ1 원점(S1) -> 최하단.  (문서는 θ1 -> Z)
  - 단계 10 에 S3 확인 추가(전진 상태로 11 하강 시 백라이트 충돌).
  - 단계 12 = ZC181 먼저 -> RX51, 단계 21 = ZC105 -> 180->90->0 역회전 + S1.
한 단계 안의 여러 동작은 순차 실행한다. 한글 문자열은 화면(브라우저) 전용 — 콘솔 출력 금지.
"""
from dataclasses import dataclass

from wafer_cell import cell_config as C


@dataclass(frozen=True)
class Action:
    kind: str            # 'z' | 'r' | 'rot' | 'expect' | 'capture' | 'wait_s5' | 'check_pick'
    value: float = 0.0
    sensor: str = ""
    expect: bool = True
    via: tuple = ()


def MoveZ(z):
    return Action("z", float(z))


def MoveR(x):
    return Action("r", float(x))


def Rotate(theta, via=()):
    return Action("rot", float(theta), via=tuple(float(v) for v in via))


def Expect(sensor, value=True):
    return Action("expect", sensor=sensor, expect=bool(value))


def Capture():
    return Action("capture")


def WaitS5Empty():
    return Action("wait_s5")


def CheckPick():
    """픽업 직후 S4 확인. 없으면 '빈 슬롯 또는 픽업 실패' -> 운전자 확인."""
    return Action("check_pick", sensor="S4", expect=True)


@dataclass(frozen=True)
class Step:
    key: str             # 'H1'..'H5', '1'..'21', 'F1'
    no: int              # 표시 번호(원점/종료는 0)
    group: str           # 'home' | 'take' | 'swing1' | 'inspect' | 'swing2' | 'out' | 'final'
    title: str
    why: str
    actions: tuple


GROUP_LABELS = {
    "home": "원점", "take": "① 카세트 취출", "swing1": "선회",
    "inspect": "② 검사 · 촬영", "swing2": "선회", "out": "③ 반출", "final": "종료",
}
# 공정 흐름 띠(처음 보는 사람용 4단계)
STAGE_OF_GROUP = {"home": None, "take": "cassette", "swing1": "cassette", "inspect": "inspect",
                  "swing2": "inspect", "out": "output", "final": None}


def homing_steps():
    ret, home, swing = C.RX_RET, C.ZC_HOME, C.ZC_SWING
    return [
        Step("H1", 0, "home", "R 수축 (S3 확인)",
             "블레이드를 먼저 빼야 어느 높이에서 멈췄든 스테이션을 긁지 않는다",
             (MoveR(ret), Expect("S3", True))),
        Step("H2", 0, "home", "Z 최하단 원점 (S2)",
             "아래 슬롯은 이미 비어 있고(아래->위 규칙) 모든 스테이션 아래로 피한다",
             (MoveZ(home), Expect("S2", True))),
        Step("H3", 0, "home", "선회 높이로 상승 (ZC 105)",
             "문서가 검증한 선회 높이에서만 돈다(ZC 95 는 S1 하우징과 1mm 간섭 가능)",
             (MoveZ(swing),)),
        Step("H4", 0, "home", "θ1 원점 (S1)",
             "θ1 은 반드시 마지막 - Z 높이를 모른 채 돌면 스테이션과 충돌한다",
             (Rotate(0.0), Expect("S1", True))),
        Step("H5", 0, "home", "원점 자세 (Z 최하단 · R 수축 · θ1 0°)",
             "원점 상태 = Z 최하단 + 실린더 수축 + θ1 0°",
             (MoveZ(home), Expect("S2", True))),
    ]


def final_steps():
    return [Step("F1", 0, "final", "원점 자세 복귀 (ZC 95)",
                 "카세트 작업 완료 - 다음 카세트를 기다린다",
                 (MoveZ(C.ZC_HOME),))]


def wafer_steps(n, preset=None):
    """슬롯 n(1..10) 한 장의 21단계."""
    off, lift = C.pick_offsets(preset)
    sn = C.slot_z(n)
    pick = sn + off
    ext, ret, swing = C.RX_EXT, C.RX_RET, C.ZC_SWING
    above, seat, repick = C.CRADLE_ZC_ABOVE, C.CRADLE_ZC_SEAT, C.CRADLE_ZC_PICK
    return [
        Step("1", 1, "take", f"슬롯 {n} 높이 정렬 (ZC {pick:.1f})",
             "블레이드 윗면을 웨이퍼 하면보다 조금 아래에 맞춰야 슬롯 틈으로 들어간다",
             (MoveZ(pick),)),
        Step("2", 2, "take", f"웨이퍼 밑으로 블레이드 삽입 (RX {ext:.0f})",
             "U자 포크를 웨이퍼 아래 틈으로 밀어 넣는다",
             (MoveR(ext),)),
        Step("3", 3, "take", f"리프트 +{lift:g} -> 픽업 (S4 확인)",
             "블레이드를 올려 웨이퍼를 슬롯 선반에서 들어 올리고, S4 로 실렸는지 확인한다",
             (MoveZ(pick + lift), CheckPick())),
        Step("4", 4, "take", "수축 (RX 0, S3 확인)",
             "웨이퍼를 든 채 카세트 밖으로 빼낸다",
             (MoveR(ret), Expect("S3", True))),
        Step("5", 5, "take", "선회 높이로 하강 (ZC 105)",
             "웨이퍼를 든 선회 반경(R234.5)이 모든 스테이션과 겹치므로 스테이션 아래(z180 미만)로 내려서 돈다",
             (MoveZ(swing),)),
        Step("6", 6, "swing1", "선회 0° -> 90°",
             "검사 스테이션으로 간다. ZC 105 고정, S3 후진 확인 후에만 돈다",
             (Expect("S3", True), Rotate(90.0))),
        Step("7", 7, "inspect", f"검사 크래들 위로 상승 (ZC {above:.0f})",
             "웨이퍼 하면을 크래들 안착면(z252)보다 5.5mm 위로 올린다",
             (MoveZ(above),)),
        Step("8", 8, "inspect", f"전진 (RX {ext:.0f})",
             "웨이퍼를 3점 크래들 바로 위로 보낸다",
             (MoveR(ext),)),
        Step("9", 9, "inspect", f"하강 -> 크래들 안착 (ZC {seat:.0f}, S4 없음)",
             "블레이드가 안착면 아래로 내려가며 웨이퍼를 크래들에 내려놓는다",
             (MoveZ(seat), Expect("S4", False))),
        Step("10", 10, "inspect", "수축 (RX 0, S3 확인)",
             "촬영 전에 블레이드를 빼낸다(추가한 S3 확인: 전진한 채 내려가면 백라이트와 충돌)",
             (MoveR(ret), Expect("S3", True))),
        Step("11", 11, "inspect", f"하강 후 촬영 (ZC {C.STEP11_ZC:.0f}, CAM)",
             "백라이트 투과광으로 웨이퍼 외곽(에지 깨짐 · 노치/플랫 · 중심 편차)을 촬영한다",
             (MoveZ(C.STEP11_ZC), Capture())),
        Step("12", 12, "inspect", f"재진입 (ZC {seat:.0f} -> RX {ext:.0f})",
             "다시 집으려고 블레이드를 웨이퍼 밑으로 넣는다. Z 먼저, 그다음 전진",
             (MoveZ(seat), MoveR(ext))),
        Step("13", 13, "inspect", f"리프트 -> 재픽업 (ZC {repick:.0f}, S4 확인)",
             "크래들에서 웨이퍼를 다시 들어 올린다",
             (MoveZ(repick), Expect("S4", True))),
        Step("14", 14, "inspect", "수축 (RX 0, S3 확인)",
             "웨이퍼를 든 채 크래들에서 빠져나온다",
             (MoveR(ret), Expect("S3", True))),
        Step("15", 15, "inspect", "선회 높이로 하강 (ZC 105)",
             "스테이션 아래로 내려서 돈다",
             (MoveZ(swing),)),
        Step("16", 16, "swing2", "선회 90° -> 180°",
             "반출 스테이션으로 간다. ZC 105 고정, S3 확인 후에만 돈다",
             (Expect("S3", True), Rotate(180.0))),
        Step("17", 17, "out", f"반출 크래들 비었는지(S5) 확인 후 상승 (ZC {above:.0f})",
             "다음 이송로봇이 이전 웨이퍼를 가져갔는지 S5 로 확인한 뒤 들어간다",
             (WaitS5Empty(), MoveZ(above))),
        Step("18", 18, "out", f"전진 (RX {ext:.0f})",
             "웨이퍼를 반출 크래들 바로 위로 보낸다",
             (MoveR(ext),)),
        Step("19", 19, "out", f"하강 -> 반출 크래들 안착 (ZC {seat:.0f}, S4 없음)",
             "반출 크래들이 다음 이송로봇과의 인계 지점이다",
             (MoveZ(seat), Expect("S4", False))),
        Step("20", 20, "out", "수축 (RX 0, S3 확인)",
             "블레이드를 빼서 인계 지점을 비운다",
             (MoveR(ret), Expect("S3", True))),
        Step("21", 21, "out", "하강 · 0°로 복귀 (180 -> 90 -> 0, S1 확인)",
             "다음 슬롯을 위해 카세트로 돌아간다. 0°마다 S1 로 벨트 이빨 튐을 확인한다",
             (MoveZ(C.STEP21_ZC), Expect("S3", True), Rotate(0.0, via=C.RETURN_VIA), Expect("S1", True))),
    ]


def empty_slot_steps(n):
    """S4 없음 -> 운전자가 '빈 슬롯'으로 확정했을 때: 수축 후 선회 높이로(다음 슬롯으로)."""
    s = wafer_steps(n)
    return [s[3], s[4]]


HANDOFF_INFO = {"key": "HO", "title": "인계 - 다음 이송로봇이 집어감 (S5 비움)",
                "why": "반출 크래들이 비어야 다음 웨이퍼를 내려놓을 수 있다"}


def can_rotate(zc, rx, s3, carrying=False):
    """선회 인터록. (허용 여부, 사유) — 원점 절차의 θ1 원점 복귀도 예외 없음."""
    reasons = []
    if abs(zc - C.ZC_SWING) > 0.25:
        reasons.append(f"ZC {zc:.1f} (선회는 ZC {C.ZC_SWING:.0f}에서만)")
    if abs(rx - C.RX_RET) > 0.5:
        reasons.append(f"RX {rx:.1f} (수축 아님)")
    if not s3:
        reasons.append("S3 후진 미확인")
    if carrying and (zc + C.BLADE_PLANE_OFS + C.WAFER_T) >= C.SWING_CLEAR_Z:
        reasons.append("운반 웨이퍼가 스테이션 높이")
    if reasons:
        return False, "선회 금지: " + ", ".join(reasons)
    return True, "선회 허용"


def step_catalog():
    """타임라인/설명용 단계 목록(슬롯 무관 표기)."""
    out = []
    for st in homing_steps():
        out.append({"key": st.key, "no": 0, "group": st.group, "title": st.title, "why": st.why})
    for st in wafer_steps(1):
        title = st.title.replace("슬롯 1 ", "슬롯 n ")
        out.append({"key": st.key, "no": st.no, "group": st.group, "title": title, "why": st.why})
    out.append({"key": HANDOFF_INFO["key"], "no": 0, "group": "handoff",
                "title": HANDOFF_INFO["title"], "why": HANDOFF_INFO["why"]})
    return out
