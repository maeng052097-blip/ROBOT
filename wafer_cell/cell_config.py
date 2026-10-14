"""웨이퍼 검사셀 R9 중앙 설정.

출처: 사용자 제공 비디오 'R9 웨이퍼 검사 셀' (KakaoTalk_20261012_170215712.mp4, 88초).
비디오 자막에 나온 수치는 그대로 쓰고, 비디오에 없는 값은 '가정' 으로 두어 ASSUMED 목록에 넣는다
(화면에 '가정' 배지). 기구 수치 일부(블레이드 면 오프셋 65.5, 크래들 패드 252)는 이전 CELL_V9 문서와 같다.

좌표: 셀좌표 mm, 원점 = θ1 축. θ1 0°=+x(OK 카세트), 90°=+y. 스테이션 시계방향(위에서 볼 때):
  ① 투입 180° · ② 검사 120° · ③ NG 60° · ④ OK 0°,  대차 받침 = θ -90°(셀 앞).
재활용 로봇 설정(common/config.py)과 섞지 않는다(별개 기계).
"""
from pathlib import Path

CELL_DIR = Path(__file__).resolve().parent
WEB_DIR = CELL_DIR / "web"
CAPTURE_DIR = CELL_DIR / "captures"        # 캡처 jpg + 웨이퍼 기록 CSV (.gitignore 의 captures/ 에 포함)

# ===== 스테이션 (비디오 표제) =====
STATIONS = {               # 이름 -> θ1(deg)
    "input": 180.0,        # ① 투입 카세트
    "inspect": 120.0,      # ② 검사 크래들(백라이트 C-03N 위 크래들 C-02N, StreamCam 위에서 촬영)
    "ng": 60.0,            # ③ NG 카세트
    "ok": 0.0,             # ④ OK 카세트(클램프 윗판 C-08K1S, 대차로 반출)
    "cart": -90.0,         # 대차 받침 자리(θ -90 · R 85)
}
CASSETTES = ("input", "ng", "ok")
STATION_TOL_DEG = 1.0      # 이 각도 이내면 '그 스테이션에 서 있음'
THETA_STANDBY = 120.0      # 대기 θ120 · Z105 · R0 (비디오)
THETA_MIN, THETA_MAX = -90.0, 180.0   # 선회 범위 270° (케이블/슬립링 경로 확인 필요)

# ===== 카세트 (비디오: 슬롯 바닥 190, 208 -> 피치 18, 5장) =====
SLOT_COUNT = 5
SLOT_Z0 = 190.0            # 슬롯 1 바닥(웨이퍼 하면) 높이. Sn = 190 + 18(n-1)
SLOT_PITCH = 18.0
LEDGE_T = 4.0              # 가정: 받침 이빨 두께 [Sn-4, Sn]
PICK_OFS = -68.5           # 집기: Z = Sn - 68.5 (비디오) -> 블레이드 면 Sn-3
PICK_LIFT = 7.5            # 들기 +7.5 -> 받침 이빨에서 4.5 뜸, 앞 턱 2.5 를 넘음 (비디오)
PLACE_OFS = -61.0          # 넣기: Z = Sn - 61 진입 -> Sn - 68.5 로 내려놓음 (비디오: 슬롯 5 Z201 -> 193.5)
FRONT_LIP = 2.5            # 카세트 앞 턱 높이 (비디오)

# ===== 웨이퍼 =====
WAFER_D = 100.0            # 4" Ø100
WAFER_T = 2.0              # 가정: 3D 프린트 모형, 두께 미정

# ===== 로봇 기하 (CELL_V9 와 같은 몸체) =====
ZC_MIN, ZC_MAX = 95.0, 274.0        # Z 범위
ZC_HOME = 95.0             # Z 원점(S2, 최하단)
ZC_SWING = 105.0           # 선회 높이: '선회는 늘 Z105 · R0' (비디오)
BLADE_PLANE_OFS = 65.5     # 포크 패드 윗면 = 웨이퍼 하면 = Z + 65.5
BLADE_BODY = (60.0, 64.0)  # 포크 몸체 z = Z+60 ~ Z+64
BLADE_ORIGIN_X = 107.0     # 수축 시 포크 시작 x (로봇 로컬)
BLADE_TIP_X = 203.0        # 수축 시 포크 끝 반경(R203)
WAFER_ON_BLADE_X = 77.5    # 포크 원점 기준 웨이퍼 중심 -> 수축 184.5, 전진 235.5
S4_LOCAL_X = 33.0          # S4(포크 웨이퍼 유무) 포크 로컬 x
S4_RANGE = (0.0, 4.0)      # 가정: S4 감지 = 웨이퍼 하면이 센서면(Z+64) 위 0~4mm. 포크 위 웨이퍼 1.5mm(ON),
                           #   내려놓은 뒤 슬롯 4.5mm · 크래들 5mm(OFF) -> 여유 0.5~1mm 뿐 [현장 확인 필요]
STATION_R = 235.5          # 스테이션 웨이퍼 중심 반경

# ===== R 축 (비디오: '실린더 2개 같이 밈') =====
RX_RET = 0.0
RX_EXT = 51.0              # 웨이퍼 집기/놓기 전진량 (비디오)
RX_CART = 85.0             # 대차 받침 내밀기 R 0 -> 85 (비디오). 단일 실린더 실측 스트로크 62 로는 불가
RX_MAX = 85.0
RX_STROKE_MEASURED = 62.0  # 사용자 실측(2026-09-29, 실린더 1개). R85 는 실린더 2개 직렬 전제
# R 구동 = 리니어 모터(양 끝단 리밋 스위치 내장, 사용자 확인) + L298. 후진 끝 리밋 = S3.
# 전진 끝 리밋은 풀 스트로크(62) 위치라 R51/R85 확인은 못 함 -> 스트로크를 51 에 맞추는 기구 스토퍼 권장.

# ===== 카세트 하단 TOF (VL53L0X, 도크마다 1 개, 웨이퍼 중심 반경 아래) =====
# 가장 아래 웨이퍼 하면까지의 거리 하나만 잰다 -> 개수는 '위로 빈칸 없이 붙어 있음' 가정의 추정.
TOF_Z = 140.0              # 가정: 센서 면 높이(카세트 바닥판 z172~180 아래 30 이상, VL53L0X 최소 거리 ~30)
CASS_TOP_Z = 284.0         # 가정(도면): 빈 카세트일 때 빔이 닿는 윗레일 밑면
TOF_TOL = 6.0              # 판정 허용 ±mm (슬롯 간격 18 의 절반 9 보다 작게)
TOF_NOISE = 1.5            # 가정: 시뮬 잡음 ±mm
TOF_MIN_MM = 30.0          # VL53L0X 신뢰 최소 거리(데이터시트 근사)
TOF_OUT_OF_RANGE = 8190    # VL53L0X 범위 밖 코드 -> 카세트 없음

# ===== 검사 크래들 (비디오: Z192 -> R51 -> Z183(패드 윗면 252 에 내려놓음) -> R0) =====
CRADLE_SEAT_Z = 252.0      # 패드 윗면
CRADLE_ZC_ABOVE = 192.0    # 진입/다시 들기 높이 (웨이퍼 하면 257.5)
CRADLE_ZC_SEAT = 183.0     # 내려놓은 뒤 (포크 면 248.5)
SWING_CLEAR_Z = 180.0      # 선회 중 운반 웨이퍼 윗면은 이보다 아래(스테이션 하단)

# ===== OK 카세트 -> 대차 (비디오 T1~T13) =====
CART_SIDE_THETA = -40.0    # T1 비킴 선회 θ120 -> -40 (클램프 윗판 앞끝과 포크 끝이 부딪히지 않게)
CART_Z_UNDER = 228.5       # T2 오르기 (클램프 윗판 밑 높이)
CART_Z_CARRY = 241.0       # T5 들기: 카세트 + 웨이퍼 5장을 클램프 윗판째로 듦, T8 은 Z241 · R0 에서만 선회
CART_Z_SEAT = 205.0        # T10 받침에 내려놓기 (비디오 상태줄 Z205)
CART_LIFT_Z = 235.0        # 가정: 이 높이를 지나며 카세트가 도크 <-> 포크로 옮겨진다(228.5 와 241 사이)
CART_SEAT_LIFT_Z = 223.0   # 가정: 이 높이를 지나며 카세트가 포크 -> 받침으로 옮겨진다(241 과 205 사이)
CART_SEATS = ("FL", "FR", "RL", "RR")   # 앞 왼 · 앞 오른 · 뒤 왼 · 뒤 오른 (1자, 앞 2 · 뒤 2)
CART_SEAT_KR = {"FL": "앞 왼", "FR": "앞 오른", "RL": "뒤 왼", "RR": "뒤 오른"}
CART_RETREAT = 120.0       # 대차 물러남 -120 (옆걸음은 물러난 자리에서만)
CART_LATERAL = 200.0       # 대차 옆걸음 ±200 (메카넘)

# ===== 속도 / 시간 (가정 — 비디오는 배속 편집) =====
THETA_SPEED = 60.0         # 가정: θ1 deg/s
Z_SPEED = 10.0             # 가정: mm/s
R_SPEED = 20.0             # 가정: mm/s
CAPTURE_S = 0.5            # 가정: 백라이트 점등 + StreamCam 촬영 + 판정
NG_RATE = 0.10             # 가정: 시뮬 NG 비율(고정 시드). '다음 웨이퍼 NG' 로 수동 지정 가능
OPERATOR_S = 8.0           # 가정: 작업자 자동 모드에서 교체/놓기까지 걸리는 시간
CART_CALL_S = 8.0          # 가정: 대차 호출 -> 셀 앞 도착
CART_DOCK_S = 2.0          # 가정: PL 도킹(태그 ID 1)
CART_RETREAT_S = 2.0       # 가정
CART_LATERAL_S = 3.0       # 가정
CART_ROTATE_S = 4.0        # 가정: 제자리 180° 회전
CART_LEAVE_S = 5.0         # 가정: 출발 -> 셀에서 사라짐

# ===== 타임아웃 =====
HOMING_TIMEOUT_S = 30.0
EXPECT_TIMEOUT_S = 2.0

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
    "WAFER_T", "LEDGE_T", "THETA_SPEED", "Z_SPEED", "R_SPEED", "CAPTURE_S", "NG_RATE", "OPERATOR_S",
    "CART_CALL_S", "CART_DOCK_S", "CART_RETREAT_S", "CART_LATERAL_S", "CART_ROTATE_S", "CART_LEAVE_S",
    "CART_LIFT_Z", "CART_SEAT_LIFT_Z", "S4_RANGE", "TOF_Z", "CASS_TOP_Z", "TOF_NOISE",
]


def slot_z(n):
    """슬롯 n(1..5) 바닥(웨이퍼 하면) 높이 Sn."""
    return SLOT_Z0 + SLOT_PITCH * (n - 1)


def pick_z(n):
    return slot_z(n) + PICK_OFS


def place_z(n):
    return slot_z(n) + PLACE_OFS


def tof_expected(n):
    """하단 TOF 기대 거리: 슬롯 n 이 가장 아래 웨이퍼일 때(웨이퍼 하면 = Sn)."""
    return slot_z(n) - TOF_Z


def tof_empty_d():
    """빈 카세트일 때 기대 거리(윗레일 밑면)."""
    return CASS_TOP_Z - TOF_Z


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
    return {
        "slot_count": SLOT_COUNT, "slot_z0": SLOT_Z0, "slot_pitch": SLOT_PITCH,
        "ledge_t": LEDGE_T, "wafer_d": WAFER_D, "wafer_t": WAFER_T,
        "tof_z": TOF_Z, "cass_top_z": CASS_TOP_Z,
        "zc_min": ZC_MIN, "zc_max": ZC_MAX, "zc_swing": ZC_SWING, "zc_home": ZC_HOME,
        "blade_plane_ofs": BLADE_PLANE_OFS, "blade_body": list(BLADE_BODY),
        "blade_origin_x": BLADE_ORIGIN_X, "blade_tip_x": BLADE_TIP_X,
        "wafer_on_blade_x": WAFER_ON_BLADE_X, "station_r": STATION_R,
        "rx_ext": RX_EXT, "rx_cart": RX_CART, "rx_max": RX_MAX, "rx_stroke_measured": RX_STROKE_MEASURED,
        "cradle_seat_z": CRADLE_SEAT_Z, "stations": STATIONS, "cassettes": list(CASSETTES),
        "theta_standby": THETA_STANDBY, "theta_min": THETA_MIN, "theta_max": THETA_MAX,
        "cart_seats": list(CART_SEATS), "cart_seat_kr": CART_SEAT_KR,
        "cart_retreat": CART_RETREAT, "cart_lateral": CART_LATERAL,
        "cart_z_under": CART_Z_UNDER, "cart_z_carry": CART_Z_CARRY, "cart_z_seat": CART_Z_SEAT,
        "speeds": {"theta": THETA_SPEED, "z": Z_SPEED, "r": R_SPEED, "capture": CAPTURE_S},
        "assumed": ASSUMED,
    }
