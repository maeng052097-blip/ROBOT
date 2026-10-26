"""화면 문구(한글) 한 곳 — 시뮬(SIM)과 실장비(LIVE)가 같은 글을 보이게 한다.

장비(Mega)와 시뮬은 ASCII 코드 + 짧은 ASCII 세부(d)만 낸다. 한글 문장은 여기서 만든다(브라우저 전용).
콘솔(cp949)에는 이 문자열을 출력하지 않는다.
"""
from wafer_cell import cell_config as C
from wafer_cell import sequence as Q

ST_KR = {"input": "① 투입", "inspect": "② 검사", "ng": "③ 불량", "uv": "UV 탑", "cart": "대차 받침"}
SEAT_KR = C.CART_SEAT_KR

# 코드 -> (문구, fatal = 원점 복귀 필요)
ALARMS = {
    "HOMING_TIMEOUT": ("원점 복귀 시간 초과", True),
    "HOME_NO_STATION": (f"원점: {C.THETA_HOME_RANGE:g}° 안에 θ 자리 센서 LOW 없음", True),
    "TH_SENSOR_FAULT": ("θ 자리 센서 2 개 이상 LOW (센서 고장)", True),
    "TH_MISMATCH": ("θ 자리 센서 불일치 (벨트 이빨 튐 또는 센서 고장)", True),
    "R_NOT_RETRACTED": ("R 후진 미확인 (R Sharp RX > 2)", False),
    "R_VERIFY": ("R 위치 확인 실패 (R Sharp 와 지령값 차이)", False),
    "Z_VERIFY": ("Z 높이 확인 실패 (Z Sharp 와 지령값 차이)", True),
    "S4_REMAINS": ("놓은 뒤 S4 잔존 (웨이퍼가 포크에 남음?)", False),
    "S4_MISSING": ("다시 집은 뒤 S4 없음", False),
    "CASS_MISSING": ("카세트 없음 (ToF)", False),
    "DOCK_OCCUPIED": ("① 도크에 이미 카세트가 있음 (TOF-IN)", False),
    "UV_INTERLOCK": ("UV 켜기 조건 미충족", False),
    "UV_FAULT": (f"UV 켜짐 미확인 (A_UV {C.UV_AUV_S:g} s)", False),
    "UV_LOST": ("UV 조사 중 꺼짐 (KSD9700 과열 또는 LED)", False),
    "CORR_RANGE": (f"치우침 보정 창 밖 (δr {C.CORR_DR[0]:g}~+{C.CORR_DR[1]:g} mm · |δθ| <= {C.CORR_DTH:g}°)", False),
    "JUDGE_TIMEOUT": (f"판정 응답 없음 (PC {C.JUDGE_TIMEOUT_S:g} s)", False),
    "CAMERA": ("카메라 / 판정 실패 (PC)", False),
    "INTERLOCK": ("인터록 위반", False),
    "COLLISION": ("충돌 (시뮬 기하 판정)", True),
    "ESTOP": ("비상정지 (D9 E-stop 사슬 끊김)", True),
    "SIM_EXCEPTION": ("시뮬 내부 오류", True),
    "DEVICE": ("장비 알람", True),
}

ROT_WHY = {
    "R_NOT_RET": "R 후진 미확인", "THETA_RANGE": "θ 범위 밖", "Z_NOT_SWING": f"Z {C.ZC_SWING:g} 아님",
    "CASS_SWING": "카세트를 든 채 Z 105 선회 금지", "WAFER_HIGH": "운반 웨이퍼가 스테이션 높이",
    "HIGH_PATH": f"높이 선회는 θ {C.STATIONS['inspect']:g} ~ {C.STATIONS['input']:g} 만", "CASS_HIGH": "카세트를 든 채 높이 선회 금지",
    "CRADLE_BAND": f"운반 웨이퍼가 ② 크래들 높이(z {C.CRADLE_BAND_Z[0]:g}~{C.CRADLE_BAND_Z[1]:g} ± {C.HIGH_SWING_MARGIN:g})에 걸림",
    "Z_RANGE": "Z 범위 밖", "CARRY_PATH": f"카세트 운반은 θ {C.STATIONS['input']:g} ~ {C.STATIONS['cart']:g} 만",
    "Z_NOT_CARRY": f"Z {C.CASS_Z_CARRY:g} 아님", "FINE_RANGE": f"미세 회전 {C.CORR_DTH:g}° 초과",
    "FINE_POSE": f"미세 회전은 ② Z {C.CRADLE_ZC_SEAT:g} 에서만", "MODE": "선회 방식 오류",
}
UV_WHY = {
    C.UV_TH_SENSOR: f"θ{C.STATIONS['uv']:g} (UV 자리) 센서 LOW 아님", "ANGLE": f"θ {C.STATIONS['uv']:g} ± {C.UV_THETA_TOL:g}° 아님", "RX": f"RX <= {C.RX_RET_OK:g} 아님",
    "ZC": f"ZC <= {C.UV_ZC_LAMP_MAX:g} 아님", "HAND": "받침에 넘김 미확인 (S4 ON -> OFF)",
    "JUDGE": "판정 OK 아님", "D9": "E-stop 사슬(D9) 끊김",
}
COLLISION_KR = {
    "Z_RANGE": f"Z 범위 밖 (ZC {C.ZC_MIN:g} ~ {C.ZC_SOFT_MAX:g})",
    "BETWEEN": "스테이션 사이에서 상승/운반",
    "BETWEEN_R": "스테이션 사이에서 전진",
    "R_MID": "R 중간 위치에서 Z 이동",
    "FORK_WAFER": "포크가 {st} {slot} 웨이퍼와 충돌",
    "CARRIED_WAFER": "운반 웨이퍼가 {st} {slot} 웨이퍼와 충돌",
    "CARRIED_LEDGE": "운반 웨이퍼가 {st} 받침 이빨과 충돌",
    "TOP_RAIL": "포크가 {st} 윗레일과 충돌",
    "PLATE": "포크가 ① 클램프 윗판과 충돌",
    "DOCK_OCCUPIED": "카세트를 이미 찬 ① 도크에 놓음",
    "CASS_Z": "카세트를 든 채 {st} 에서 Z 이동",
    "BACKLIGHT": "전진한 채 크래들 아래로 (백라이트 충돌)",
    "UV_ZONE": f"UV 탑 밑(RX > 2) 에서 ZC {C.UV_ZC_HAND:g} ~ {C.UV_ZC_IN:g} 밖",
    "UV_TOWER": f"UV 탑 앞에서 ZC {C.UV_ZC_IN:g} 초과",
    "NO_SEAT": "대차 받침이 로봇 앞에 없음",
    "SEAT_FULL": "대차 받침에 이미 카세트",
    "R_MAX": "R 최대 전진 초과",
}
CART_KR = {
    "CALL": "대차 호출 -> 셀 앞으로", "ARRIVE": "대차 도착 -> PL 도킹", "RETREAT": f"대차 물러남 -{C.CART_RETREAT:g}",
    "LATERAL": "대차 옆걸음 (메카넘)", "ROTATE": "대차 제자리 180° 회전 (뒤 받침이 셀 쪽)", "DOCK": "대차 PL 도킹",
    "LEAVE": "처리 끝 -> 대차 출발", "LEFT": "대차 떠남 - 새 대차 호출", "DOCKED": "대차 도킹 완료",
}
DEST_KR = {"K": ("①", "원위치"), "N": ("③", "불량")}
PAUSE_KR = {"USER": "일시정지", "LINK": "PC 연결 끊김 - 일시정지 (UV 끔)", "HOLD": "일시정지 (시뮬 단계 고정)"}


def _slot_t(st, slot):
    if st in ("input", "ng") and slot:
        return f"슬롯 {slot}"
    return ""


def collision_text(d):
    """d = 'CODE/st/slot' -> 한글."""
    parts = (d or "").split("/")
    code = parts[0]
    st = parts[1] if len(parts) > 1 and parts[1] not in ("", "None") else None
    slot = parts[2] if len(parts) > 2 and parts[2] not in ("", "None", "0") else None
    t = COLLISION_KR.get(code, code)
    return " ".join(t.format(st=ST_KR.get(st, st or ""), slot=_slot_t(st, slot)).split())


def items_text(d, table):
    return ", ".join(table.get(x, x) for x in (d or "").split("+") if x)


def alarm_text(code, d=""):
    base = ALARMS.get(code, (code, False))[0]
    if not d:
        return base
    if code == "COLLISION":
        return f"{base}: {collision_text(d)}"
    if code == "UV_INTERLOCK":
        return f"{base}: {items_text(d, UV_WHY)}"
    if code == "INTERLOCK":
        return f"{base}: 선회 금지 - {items_text(d, ROT_WHY)}"
    if code in ("CASS_MISSING", "TH_MISMATCH"):
        return f"{base}: {ST_KR.get(d, d)}"
    if code == "TH_SENSOR_FAULT":
        return f"{base}: " + " · ".join(ST_KR.get(x, x) for x in d.split("+"))
    if code in ("Z_VERIFY", "R_VERIFY") and "/" in d:
        got, want = d.split("/", 1)
        return f"{base} (Sharp {got} · 지령 {want} mm)"
    if code == "CORR_RANGE" and "/" in d:
        dr, dth = d.split("/", 1)
        return f"{base}: δr {dr} mm · δθ {dth}°"
    if code == "UV_LOST":
        return f"{base} - 조사 {d}"
    if code == "DEVICE":
        return f"{base}: {d}"
    return f"{base} ({d})"


def alarm_fatal(code):
    return ALARMS.get(code, (code, True))[1]


def step_title(key, n=None, k=None, seat=None):
    r = Q.describe(key, n, k, seat)
    return (r[1], r[2]) if r else (key, "")


def cart_text(d):
    head, _, arg = (d or "").partition(":")
    t = CART_KR.get(head, head)
    if head in ("DOCK", "DOCKED", "ARRIVE") and arg in SEAT_KR:
        t += f" ({SEAT_KR[arg]} 받침이 로봇 앞)"
    elif head == "LATERAL" and arg:
        t += f" {arg}"
    return t


def event_text(code, d="", step=None, n=None, w=None, k=None, seat=None, **_):
    """이벤트 코드 + ASCII 세부 -> 한글 한 줄."""
    if code in ALARMS:
        return alarm_text(code, d)
    if code == "STEP":
        title, _ = step_title(step, n, k, seat)
        return f"[{step}] {title}"
    st_kr = ST_KR.get(d, d)
    t = {
        "BOOT": "장비 시작 - 원점 복귀가 필요합니다",
        "HOME": "원점 복귀 시작 (R 후진 -> Z 선회 높이 -> θ 자리 찾기 -> 대기)",
        "HOMED": f"원점 복귀 완료 - 대기 θ{C.THETA_STANDBY:g} · Z {C.ZC_SWING:g} · R 후진",
        "TH_FOUND": f"θ 자리 찾음: {st_kr}",
        "START": "공정 시작", "PAUSE": "일시정지", "RESUME": "재개", "RESET": "알람 해제",
        "STOP": "소프트 정지 - 원점 복귀 필요 (E-STOP 아님)",
        "LINK_LOST": "PC 하트비트 끊김 -> 일시정지 · UV 끔",
        "PICK": f"집기: {st_kr} {_slot_t(d, n)}".strip() + " -> 포크",
        "PLACE": f"놓기: 포크 -> {st_kr} {_slot_t(d, n)}".strip(),
        "CASS_PICK": f"카세트 들기: {st_kr}",
        "CASS_PLACE": f"카세트 놓기: {st_kr}",
        "EMPTY_SLOT": f"① 슬롯 {n} 빈 슬롯 (S4 없음) - 다음 슬롯",
        "CAP": f"웨이퍼 {w} 촬영 요청 -> PC 판정",
        "JUDGE": f"웨이퍼 {w} 판정 {d}",
        "UV_ON": f"UV 켬 ({C.UV_S:g} s)", "UV_OFF": "UV 끔",
        "UV_ABORT": "UV 중단 (일시정지/비상) - 재개하면 처음부터 다시 조사",
        "WDONE": f"웨이퍼 {w} 완료 -> {DEST_KR.get(d, (d, ''))[0]} 슬롯 {k if d == 'N' else n} ({DEST_KR.get(d, ('', d))[1]})",
        "CYCLE": f"웨이퍼 {w} 사이클 {d} s (가정 속도)",
        "NG_FULL": "③ 불량 카세트 가득 - 교체 요청",
        "NG_SWAP": "③ 새 불량 카세트 (TOF-NG 없음 -> 있음, 포인터 0)",
        "NG_OUT": "③ 불량 카세트 빠짐 (TOF-NG 없음)",
        "CASS_DONE": f"① 카세트 처리 끝 -> 대차 {SEAT_KR.get(seat, '')} 받침으로",
        "CART": cart_text(d),
        "OPERATOR": f"(시뮬) 작업자: {d}",
        "FAULT": f"(시뮬) 고장 주입 {d}",
        "REMOVE": f"(시뮬) 운전자가 {ST_KR.get(d, d)} 웨이퍼 수동 제거",
        "SPEED": f"(시뮬) 배속 x{d}",
        "REBOOT": "장비 재시작 감지 - 원점 복귀 필요",
        "LINK_UP": f"장비 연결됨 {d}",
        "LINK_DOWN": f"장비 연결 끊김 {d}",
    }.get(code)
    return t if t is not None else (f"{code} {d}".strip())


def headline(s):
    """기본 화면 한 줄 + 둘째 줄(상세). s = 스냅샷(SIM/LIVE 공통 모양)."""
    st = s.get("state")
    step = s.get("step")
    if st in ("ALARM", "ESTOP") and s.get("alarm"):
        return s["alarm"]["msg"], ""
    if st == "PAUSED" and s.get("pause_reason") == "LINK":
        return PAUSE_KR["LINK"], "PC 하트비트가 돌아오면 '재개' 로 이어간다"
    if st == "WAIT_NG":
        return "③ 불량 카세트 가득 - 교체 요청", f"로봇은 대기 (θ{C.THETA_STANDBY:g})"
    if st == "WAIT_CART":
        cap = (s.get("cart") or {}).get("caption")
        return (cap or "대차 대기"), "대차가 움직이는 동안 로봇은 대기"
    if step:
        title, detail = step.get("title", ""), step.get("detail", "")
        w = s.get("wafer_no")
        if step.get("group") in ("input", "inspect", "ng", "uv", "ret") and w:
            title = f"웨이퍼 {w} — {title}"
        j = s.get("judgment") or {}
        if step.get("key") == "I3" and j.get("no") == w and j.get("result"):
            title += f" {j['result']}"
        if step.get("key") == "U6" and (s.get("uv") or {}).get("lamp") and s["uv"].get("left_s") is not None:
            title += f" (남은 {s['uv']['left_s']:.0f} s)"
        return title, detail
    if st == "READY":
        return f"대기 (θ{C.THETA_STANDBY:g} · Z {C.ZC_SWING:g} · R 후진)", "'시작' 을 누르면 대차의 카세트부터 처리한다"
    if st == "IDLE":
        return "원점 복귀 필요", "R 후진 -> Z 선회 높이 -> θ 자리 찾기 -> 대기"
    if st == "PAUSED":
        return PAUSE_KR.get(s.get("pause_reason") or "USER", "일시정지"), ""
    return "", ""
