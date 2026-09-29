"""웨이퍼 이송셀 CELL_V9 중앙 설정.

출처: Google Drive 'ROBOT/웨이퍼 이송셀 CELL_V9/index.html' (2026-09-24 개정 3).
각 값 옆 주석의 §A~§I 는 그 문서의 절이다. '가정' 표기 값은 문서에 없거나 미확정이라
사용자 승인 기본값으로 둔 것 -> ASSUMED 목록에 이름을 넣어 화면에 '가정' 배지를 붙인다.

좌표: 셀좌표 mm, 원점 = θ1 축, z0 = 상자 윗면(§ 표제란). θ1 0°=+x(카세트), 90°=+y(검사), 180°=-x(반출).
재활용 로봇 설정(common/config.py)과 섞지 않는다(별개 기계).
"""
from pathlib import Path

CELL_DIR = Path(__file__).resolve().parent
WEB_DIR = CELL_DIR / "web"
CAPTURE_DIR = CELL_DIR / "captures"        # 캡처 jpg + 웨이퍼 기록 CSV (.gitignore 의 captures/ 에 포함)

# ===== 카세트 (§A, §B, §D) =====
SLOT_COUNT = 10
SLOT_Z0 = 190.0            # 슬롯 1 웨이퍼 하면 높이. Sn = 190 + 12(n-1)
SLOT_PITCH = 12.0
LEDGE_T = 4.0              # 슬롯 받침 선반 두께: 선반 = [Sn-4, Sn]
CASSETTE_BLOCK_Z = (180.0, 190.0)   # 슬롯1 받침 블록
CASSETTE_TOP_RAIL_Z = (306.0, 310.0)
CASSETTE_BASE_Z = (172.0, 180.0)    # 뒤쪽 베이스판 x250~300
CASSETTE_BASE_X = 250.0
CASSETTE_BACKWALL_X = 288.0

# ===== 웨이퍼 =====
WAFER_D = 100.0            # 4" Ø100 (§ 표제란)
WAFER_T = 2.0              # 가정: 3D 프린트 모형, 두께 미정 -> 문서 측면도 값 2mm

# ===== 로봇 기하 (§A, §B 평면도, §D 측면도, §G 블레이드) =====
ZC_MIN, ZC_MAX = 95.0, 274.0        # 캐리지 ZC 범위
ZC_HOME = 95.0             # Z 원점(S2, 최하단)
ZC_SWING = 105.0           # 선회 높이(웨이퍼 z170.5, 카세트 하단과 9.5)
BLADE_PLANE_OFS = 65.5     # 블레이드 패드 윗면 = 웨이퍼 하면 = ZC + 65.5
BLADE_BODY = (60.0, 64.0)  # 블레이드 몸체 z = ZC+60 ~ ZC+64 (두께 4)
BLADE_ORIGIN_X = 107.0     # 수축 시 블레이드 시작 x (로봇 로컬)
BLADE_TIP_X = 203.0        # 수축 시 블레이드 끝 반경(R203)
BLADE_TINE_Y = (20.0, 34.0)  # 갈래 |y| 범위 (개구 40)
WAFER_ON_BLADE_X = 77.5    # 블레이드 원점 기준 웨이퍼 중심 -> 수축 184.5, 전진 235.5
S4_LOCAL_X = 33.0          # S4(블레이드 웨이퍼 유무) 블레이드 로컬 x -> 홈에서 (140,0)
S4_RANGE = (0.0, 5.0)      # S4 감지: 웨이퍼 하면이 센서면(ZC+64) 위 0~5mm (감지거리 2~5, §C)
STATION_R = 235.5          # 스테이션 웨이퍼 중심 반경

# ===== R 축 (전동실린더) =====
RX_RET = 0.0
RX_EXT = 51.0              # 공정 전진량(§A). 실측 스트로크 62 -> 51 에서 멈추는 스토퍼/리밋 필요(사용자 결정)
RX_STROKE_MEASURED = 62.0  # 사용자 실측(2026-09-29). 62 로 전진하면 블레이드 끝 R265 -> 크래들 허브(R262~) 충돌

# ===== 픽업 프리셋 (§A 현행 / §H② E 중앙화안) — 오프셋과 리프트는 한 쌍으로만 바꾼다 =====
PICK_PRESETS = {
    "doc": (-66.0, 5.0),        # 픽업 ZC = Sn-66, 리프트 +5 (삽입 틈 0.5)
    "centered": (-68.5, 7.5),   # 중앙화안: Sn-68.5, +7.5 (위아래 3.0)
}
PICK_PRESET = "doc"

# ===== 크래들 (검사 90° / 반출 180°, §A, §E) =====
CRADLE_SEAT_Z = 252.0      # 안착면
CRADLE_ZC_ABOVE = 192.0    # 진입 높이 (웨이퍼 하면 257.5)
CRADLE_ZC_SEAT = 181.0     # 안착 후 (블레이드 면 246.5)
CRADLE_ZC_PICK = 187.0     # 재픽업 (블레이드 면 252.5)
HUB_R_MIN = 262.0          # 크래들 허브 판 시작 반경
HUB_Z = (244.0, 250.0)     # 허브 판 높이
SWING_CLEAR_Z = 180.0      # 스테이션 최하단(카세트 z180). 선회 중 운반 웨이퍼 윗면은 이보다 아래

STATIONS = {               # 이름 -> θ1(deg)
    "cassette": 0.0,
    "inspect": 90.0,
    "output": 180.0,
}
STATION_TOL_DEG = 1.0      # 이 각도 이내면 '그 스테이션에 서 있음'

# ===== 속도 / 시간 (가정 — 문서에 없음) =====
THETA_SPEED = 60.0         # 가정: θ1 deg/s (90° 1.5s)
Z_SPEED = 10.0             # 가정: mm/s (리드 2 -> 모터 1800 deg/s, 현 펌웨어 기본 900 의 2배 [불명])
Z_SPEED_PRESETS = {"assumed": 10.0, "firmware_default": 5.0}
R_SPEED = 20.0             # 가정: mm/s (51mm ~ 2.55s)
CAPTURE_S = 0.5            # 가정: 백라이트 점등 + 촬영
HANDOFF_S = 5.0            # 가정: 다음 이송로봇이 반출 크래들에서 집어가기까지
HANDOFF_DELAY_FAULT_S = 90.0  # 고장주입 '다음 로봇 지연'

# ===== 타임아웃 (사용자 승인 기본값) =====
HOMING_TIMEOUT_S = 20.0
EXPECT_TIMEOUT_S = 2.0
S5_WARN_S = 60.0           # S5 인계 대기 경고(알람 아님)

# ===== TOF (카세트 최상단, 사용자 결정 — 모델/장착 미정) =====
TOF_Z = 360.0              # 가정: 센서 높이
TOF_FLOOR_Z = 180.0        # 가정: 빈 카세트일 때 빔이 닿는 면(베이스판 z180)
TOF_TOL = 4.0              # 가정: 슬롯 판정 허용 오차 ±mm
TOF_NOISE_SD = 0.8         # 시뮬 노이즈(mm)
TOF_MIN_RANGE = 30.0       # 이보다 가까우면 무효
TOF_D1 = None              # 현장 보정값: 슬롯 1 에만 웨이퍼를 넣었을 때 거리(mm). None=문서 기하로 계산
TOF_D_EMPTY = None         # 현장 보정값: 빈 카세트 거리(mm). None=문서 기하로 계산

# ===== 서버 / 카메라 =====
HTTP_HOST = "127.0.0.1"
HTTP_PORT = 8765
SIM_HZ = 50
SSE_HZ = 20
WAFER_CAM_W, WAFER_CAM_H, WAFER_CAM_FPS = 1280, 720, 30
CAM_JPEG_W = 960           # 화면 전송용 축소 폭
CAM_JPEG_Q = 75
CAM_STALE_S = 1.0          # FrameGrabber.n 이 이 시간 이상 멈추면 STALE
CELL_SERIAL_PORT = None    # 2단계: 셀 제어보드 COM. 재활용 로봇 MOTOR_PORT/LIDAR 포트와 달라야 함

# 화면에 '가정' 배지를 붙일 설정 이름
ASSUMED = [
    "WAFER_T", "THETA_SPEED", "Z_SPEED", "R_SPEED", "CAPTURE_S", "HANDOFF_S",
    "TOF_Z", "TOF_FLOOR_Z", "TOF_TOL", "STEP11_ZC", "STEP21_ZC", "RETURN_PATH",
]
STEP11_ZC = ZC_SWING       # 가정: 단계 11 '하강 후 촬영' 목표
STEP21_ZC = ZC_SWING       # 가정: 단계 21 '하강' 목표
RETURN_VIA = (90.0,)       # 가정: 180 -> 90 -> 0 역회전 복귀(슬립링 폐지안과도 호환)


def pick_offsets(preset=None):
    """(픽업 ZC 오프셋, 리프트) 한 쌍."""
    return PICK_PRESETS[preset or PICK_PRESET]


def slot_z(n):
    """슬롯 n(1..10) 웨이퍼 하면 높이 Sn."""
    return SLOT_Z0 + SLOT_PITCH * (n - 1)


def check_port_conflict(port=CELL_SERIAL_PORT):
    """2단계용: 셀 포트가 재활용 로봇 포트(모터/LiDAR)와 겹치면 에러 문자열, 아니면 None."""
    if not port:
        return None
    try:
        from common import config as rc
    except Exception:
        return None
    used = {getattr(rc, "MOTOR_PORT", None), getattr(rc, "LIDAR_X4_PORT", None),
            getattr(rc, "LIDAR_PORT", None)}
    if str(port).upper() in {str(p).upper() for p in used if p}:
        return f"CELL_SERIAL_PORT {port} conflicts with recycling-robot ports"
    return None


def ui_geometry():
    """브라우저 도면용 기하 묶음(JSON 직렬화 가능)."""
    off, lift = pick_offsets()
    return {
        "slot_count": SLOT_COUNT, "slot_z0": SLOT_Z0, "slot_pitch": SLOT_PITCH,
        "ledge_t": LEDGE_T, "wafer_d": WAFER_D, "wafer_t": WAFER_T,
        "zc_min": ZC_MIN, "zc_max": ZC_MAX, "zc_swing": ZC_SWING, "zc_home": ZC_HOME,
        "blade_plane_ofs": BLADE_PLANE_OFS, "blade_body": list(BLADE_BODY),
        "blade_origin_x": BLADE_ORIGIN_X, "blade_tip_x": BLADE_TIP_X,
        "wafer_on_blade_x": WAFER_ON_BLADE_X, "s4_local_x": S4_LOCAL_X,
        "station_r": STATION_R, "rx_ext": RX_EXT, "rx_stroke_measured": RX_STROKE_MEASURED,
        "cradle_seat_z": CRADLE_SEAT_Z, "hub_r_min": HUB_R_MIN, "hub_z": list(HUB_Z),
        "swing_clear_z": SWING_CLEAR_Z, "pick_offset": off, "pick_lift": lift,
        "tof_z": TOF_Z, "stations": STATIONS,
        "speeds": {"theta": THETA_SPEED, "z": Z_SPEED, "r": R_SPEED,
                   "capture": CAPTURE_S, "handoff": HANDOFF_S},
        "assumed": ASSUMED,
    }
