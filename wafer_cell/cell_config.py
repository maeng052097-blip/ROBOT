"""웨이퍼 검사셀 R22 중앙 설정.

출처: 사용자 제공 'R22 센서 정리' 문서(2026-10-24, CAD okmech22.py · 검사 okcheck17.py · SPEC_CELL §18~22).
문서에 나온 수치는 그대로 쓰고, 문서에 없는 값은 '가정' 으로 두어 ASSUMED 목록에 넣는다(화면 '가정' 배지).
몸체 치수(포크 면 오프셋 65.5, 슬롯 피치 18, 검사 크래들 252, 카세트 운반 높이)는 이전 R9 모니터 값을 이어 쓴다.

좌표: 셀좌표 mm, 원점 = θ 축. θ 0° = ③ 불량 방향(+x). 위에서 볼 때 반시계가 +.
  θ0 ③ 불량(NG) · θ60 UV 탑 · θ120 ② 검사 · θ180 ① 투입(원위치) · θ270 대차 받침,  옆길 θ220 (① 옆, 아무것도 없음).
  ※ 사용자 변경(2026-10-24): R22 문서는 θ0 UV · θ60 ③ 이었으나 둘의 자리를 맞바꿈(RX · 반경은 장소를 따라감).
θ 는 슬립링(18 회로)이라 계속 돌 수 있다. 화면 · 프로토콜의 θ 범위는 [-45, 315) 한 바퀴로 쓰고, 공정 경로는
그 안에서만 움직인다(0 -> 270 사이, 넘어가지 않음). 문서의 'θ270 = -90' 은 같은 자리다.
재활용 로봇 설정(common/config.py)과 섞지 않는다(별개 기계).
"""
from pathlib import Path

CELL = "R22"
CELL_DIR = Path(__file__).resolve().parent
WEB_DIR = CELL_DIR / "web"
CAPTURE_DIR = CELL_DIR / "captures"        # 캡처 jpg + 웨이퍼 기록 CSV (.gitignore 의 captures/ 에 포함)

# ===== 스테이션 (문서 §1 θ 자리 센서 5 개) =====
STATIONS = {               # 이름 -> θ(deg). 사용자 변경: ③ <-> UV 자리 맞바꿈(R22 문서는 UV 0 · ③ 60)
    "ng": 0.0,             # ③ 불량 카세트 (위 슬롯부터 채움)
    "uv": 60.0,            # UV 탑 (받침에 넘긴 뒤 램프)
    "inspect": 120.0,      # ② 검사 크래들 (백라이트 C-03N 위 C-02N, StreamCam 위에서 촬영)
    "input": 180.0,        # ① 투입 카세트 (정상품은 원래 슬롯으로 되돌림)
    "cart": 270.0,         # 대차 받침 (= -90)
}
STATION_ORDER = ("ng", "uv", "inspect", "input", "cart")   # θ 센서 마스크 순서(TH0, TH60, TH120, TH180, TH270) = 각도 순
TH_SENSORS = ("TH0", "TH60", "TH120", "TH180", "TH270")
TH_OF = dict(zip(STATION_ORDER, TH_SENSORS))   # 장소 -> 그 자리 θ 센서 이름
UV_TH_SENSOR = TH_OF["uv"]                     # UV 켜기 조건의 '자리 센서'(지금 TH60)
SIDE_THETA = 220.0         # 옆길: ① 에서 비켜 웨이퍼/카세트를 든 채 Z 를 바꾸는 자리 (문서: θ180 <-> 220 은 RX <= 2 에서만)
THETA_FRAME = (-45.0, 315.0)   # 화면 · 프로토콜 θ 범위(한 바퀴)
CASSETTES = ("input", "ng")
STATION_TOL_DEG = 1.0      # 지령 θ 가 이 각도 이내면 '그 자리에 서 있음'
TH_LOW_HALF_DEG = 1.65     # θ 자리 센서 LOW 구간 반폭 (문서 §1 [확인 - 계산])
THETA_STANDBY = 120.0      # 가정: 대기 θ120 · Z105 · R 후진 (R9 와 같음, R22 문서에 없음)
THETA_HOME_SPEED = 5.0     # 원점: θ 를 - 쪽으로 5°/s 이하 (문서 §1)
THETA_HOME_RANGE = 95.0    # 원점: 약 95° 안에 처음 LOW 난 센서 = 그 자리 (자리 간격 최대 90°)
UV_THETA_TOL = 0.2         # UV 켜기 조건: 스텝이 UV 자리 ± 0.2° (문서 §7, 문서는 θ0 기준)

# ===== R 축 (실린더 시간 구동 + R Sharp 확인, 문서 §3) =====
RX_HOME = -2.0             # 원점: R 을 RX -2 로 (후진 끝)
RX_RET = -2.0              # 후진 목표(끝까지 뺌)
RX_RET_OK = 2.0            # 후진 판정: R Sharp 로 RX <= 2 (옆길 · 선회 · UV 켜기 조건)
STATION_RX = {             # 집기/놓기 전진량 (문서 §3: ① 73 · ②③ 51 · UV · 대차 85)
    "input": 73.0, "inspect": 51.0, "ng": 51.0, "uv": 85.0, "cart": 85.0,
}
RX_MAX = 85.0
RX_TOL = 0.5               # 지령 RX 가 목표 ± 이 값이면 그 자리

# ===== 카세트 (R9 와 같은 몸체: 슬롯 바닥 190, 피치 18, 5 장) =====
SLOT_COUNT = 5
SLOT_Z0 = 190.0            # 슬롯 1 바닥(웨이퍼 하면) 높이. Sn = 190 + 18(n-1)
SLOT_PITCH = 18.0
LEDGE_T = 4.0              # 가정: 받침 이빨 두께 [Sn-4, Sn]
PICK_OFS = -68.5           # 집기: Z = Sn - 68.5 -> 포크 면 Sn - 3
PICK_LIFT = 7.5            # (R9) 들기 +7.5. R22 ① 은 옆길 높이까지 든다(SIDE_OFS)
PLACE_OFS = -61.0          # 넣기(③): Z = Sn - 61 로 들어가 Sn - 68.5 로 내려놓음
SIDE_OFS = -57.0           # 가정(역산): ① 옆길 높이 Z = Sn - 57 -> 슬롯 5 = 205 (문서 '205 (슬롯 5 옆길)')
                           #   웨이퍼 하면 Sn + 8.5, 윗면 Sn + 10.5 -> 위 슬롯 이빨(Sn+14)까지 3.5 mm
FRONT_LIP = 2.5            # 카세트 앞 턱 높이 (R9)
CASS_TOP_Z = 284.0         # 카세트 윗레일 밑면 (R9 도면)
INPUT_DOCK_SHIFT = 22.0    # R22: ① 도크를 22 mm 바깥으로 -> 후진 포크(끝 R201~205)가 ① 웨이퍼(207.5~)와 안 겹침

# ===== 웨이퍼 =====
WAFER_D = 100.0            # 4" Ø100
WAFER_T = 2.0              # 가정: 3D 프린트 모형, 두께 미정

# ===== 로봇 몸체 (R9 와 같음) =====
ZC_MIN = 95.0              # Z 최하단
ZC_SOFT_MAX = 245.0        # 소프트 상한: ZC 249 위는 Z Sharp 가 10 cm 안쪽이라 값이 거꾸로 읽힘 (문서 §2)
ZC_SWING = 105.0           # 선회 높이 (문서 §2: ZC 105 -> Sharp 244)
BLADE_PLANE_OFS = 65.5     # 포크 패드 윗면 = 웨이퍼 하면 = Z + 65.5
BLADE_BODY = (60.0, 64.0)  # 포크 몸체 z = Z+60 ~ Z+64
BLADE_ORIGIN_X = 107.0     # 수축(RX 0) 시 포크 시작 x (로봇 로컬)
BLADE_TIP_X = 203.0        # 수축(RX 0) 시 포크 끝 반경
WAFER_ON_BLADE_X = 77.5    # 포크 원점 기준 웨이퍼 중심 -> RX 0 에서 184.5
S4_LOCAL_X = 68.0          # 가정: S4 자리 [보류] (문서 §5: 포크 뿌리, 위를 봄). 화면은 R9 갈래 중간 자리
S4_LOCAL_Y = 27.0
S4_RANGE = (0.0, 10.0)     # 가정: 웨이퍼 하면이 센서면(Z+64) 위 0~10 mm 면 ON. '없음' 은 포크를 뺀 뒤 확인


def station_r(name):
    """스테이션 웨이퍼(카세트) 중심 반경 = 포크 위 웨이퍼 중심(184.5) + 그 자리 RX."""
    return BLADE_ORIGIN_X + WAFER_ON_BLADE_X + STATION_RX[name]


# ===== Sharp GP2Y0A21YK0F (문서 §2 · §3) =====
Z_SHARP_K = 349.1          # Z 거리 = 349.1 - ZC  (ZC 105 -> 244, 121.5 -> 228, 205 -> 144, 245 -> 104)
R_SHARP_K = 104.0          # R 거리 = 104 + RX    (RX 0 -> 104, 51 -> 155, 73 -> 177, 85 -> 189)
SHARP_MIN_MM = 100.0       # 10 cm 안쪽은 값이 거꾸로 -> 무효
SHARP_MAX_MM = 800.0
SHARP_TOL = 3.0            # 가정: 지령값과 Sharp 환산값 허용 차 ±mm (실측 반복 정밀도 [실측])
SHARP_NOISE = 0.5          # 가정: 시뮬 잡음(10~20 번 평균 뒤) ±mm


def z_sharp_d(zc):
    return Z_SHARP_K - zc


def r_sharp_d(rx):
    return R_SHARP_K + rx


def zc_from_sharp(d):
    return Z_SHARP_K - d


def rx_from_sharp(d):
    return d - R_SHARP_K


# ===== ToF VL53L0X x2 (문서 §4: 카세트 있음 · 없음만) =====
TOF_PRESENT_MM = 120.0     # 가정: 이 거리보다 가까우면 카세트 있음 (R22 에서 TOF-IN 문턱 다시 티칭)
TOF_CASS_D = 40.0          # 가정: 시뮬에서 카세트 밑면까지 거리
TOF_NOISE = 1.5            # 가정
TOF_OUT_OF_RANGE = 8190    # VL53L0X 범위 밖 코드 -> 없음

# ===== 검사 크래들 (R9: Z192 -> R51 -> Z183 -> R 후진) + 카메라 치우침 보정 (문서 §6) =====
CRADLE_SEAT_Z = 252.0      # 패드 윗면
CRADLE_ZC_ABOVE = 192.0    # 진입/다시 들기 높이 (웨이퍼 하면 257.5)
CRADLE_ZC_SEAT = 183.0     # 내려놓은 뒤 (포크 면 248.5) - 이 높이에서만 δθ 미세 회전(≤ 0.5°) 허용
SWING_CLEAR_Z = 180.0      # 선회 중 운반 웨이퍼 윗면은 이보다 아래(스테이션 하단)
CORR_DR = (-3.5, 0.5)      # 다시 집기 보정 창: δr (mm)
CORR_DTH = 0.5             # |δθ| (deg)

# ===== UV 탑 (θ60 = STATIONS['uv'], 문서 §7) =====
UV_ZC_IN = 121.5           # UV 넣기 높이 (Sharp 228)
UV_ZC_HAND = 113.5         # 받침에 넘기는 높이. UV 탑 밑(RX > 2)에서는 ZC 113.5 ~ 121.5 만
UV_ZC_LAMP_MAX = 110.0     # 램프 켜기 조건: ZC <= 110
UV_SEAT_Z = 183.0          # 가정: 받침 패드 윗면 (113.5+65.5=179 과 121.5+65.5=187 사이)
UV_S = 10.0                # 가정(사용자: 아직 모름): 조사 시간
UV_AUV_S = 0.3             # A_UV: 켠 뒤 0.3 s 안에 창 안이어야 함 (아니면 FAULT_UV)

# ===== 카세트 운반 (대차 <-> ①, R9 클램프 윗판 방식을 ① 에 적용) =====
CASS_Z_UNDER = 228.5       # ① 클램프 윗판 밑 높이 (R9 OK 도크 값)
CASS_Z_CARRY = 241.0       # 카세트 든 높이 (선회는 θ180~270 사이, RX <= 2 에서만)
CASS_Z_SEAT = 205.0        # 대차 받침에서 포크 높이 (R9 T10)
CASS_LIFT_Z = 235.0        # 가정: 이 높이를 지나며 카세트가 ① 도크 <-> 포크로 옮겨진다
CASS_SEAT_LIFT_Z = 223.0   # 가정: 이 높이를 지나며 카세트가 대차 받침 <-> 포크로 옮겨진다
CART_SEATS = ("FL", "FR", "RL", "RR")   # 앞 왼 · 앞 오른 · 뒤 왼 · 뒤 오른
CART_SEAT_KR = {"FL": "앞 왼", "FR": "앞 오른", "RL": "뒤 왼", "RR": "뒤 오른"}
CART_DEFAULT_MAP = (True, True, True, True)   # 가정: 대차 한 대에 카세트 4 개 (R9 영상). 시뮬 도구로 1~4
CART_RETREAT = 120.0       # 대차 물러남 -120 (옆걸음은 물러난 자리에서만)
CART_LATERAL = 200.0       # 대차 옆걸음 ±200 (메카넘)
# 대차 몸체(대차 로컬 mm: 앞 왼 받침 중심 = (0,0), +x = 오른쪽 받침, +y = 셀에서 멀어짐). 화면 · 3D 공용 (R9 화면 값)
CART_SEAT_XY = {"FL": (0.0, 0.0), "FR": (200.0, 0.0), "RL": (200.0, 260.0), "RR": (0.0, 260.0)}
CART_BODY = (-90.0, -80.0, 380.0, 420.0)    # 가정: x, y, 폭, 길이 -> 앞면은 받침 중심보다 80 mm 셀 쪽
CART_SEAT_TOP_Z = 163.5    # 받침 윗면 z (R9 화면 값: Z223 에서 카세트가 받침에 닿는 높이)
CASS_PLATE_UNDER = 125.0   # 카세트 바닥 -> 클램프 윗판 밑면 (R9 도면)

# ===== 책상 (사용자: 공정은 책상 위, 대차는 책상 옆 바닥. 치수는 [불명] -> 가정) =====
# 모든 장소(① 틀 · 클램프 x -328, ③ 틀 x +304, UV 탑 몸통 y +319, ② 꼭짓점 y +301, 로봇 받침 r 85)를 감싸는 최소 직사각형 + 여백 20 mm.
# 대차 쪽 끝(-y, θ270)은 로봇 받침(-85)과 대차 앞면(-(269.5 - 80) = -189.5) 사이 -140 -> 포크 R85 가 책상 끝을 넘어 받침에 닿음.
DESK_X0, DESK_X1 = -350.0, 324.0          # 가정: 셀 x (θ0 = +x)
DESK_Y0, DESK_Y1 = -140.0, 340.0          # 가정: 셀 y (θ90 = +y, 대차 쪽이 -y)
DESK_TOP_Z = -60.0         # 가정: 책상 윗면 z (θ 센서 링 z -37.5 · 노치판 -38.5 아래, 3D 전용)
DESK_H = 720.0             # 가정: 책상 높이 (3D 전용)
FLOOR_Z = DESK_TOP_Z - DESK_H

# ===== 속도 / 시간 (가정) =====
THETA_SPEED = 60.0         # 가정: θ deg/s
Z_SPEED = 10.0             # 가정: mm/s
R_SPEED = 20.0             # 가정: mm/s
CAPTURE_S = 0.5            # 가정: 백라이트 점등 + StreamCam 촬영 (판정은 PC)
NG_RATE = 0.10             # 가정: 시뮬 NG 비율(고정 시드). '다음 웨이퍼 NG' 로 수동 지정 가능
OPERATOR_S = 8.0           # 가정: 작업자가 NG 카세트를 바꾸는 시간(시뮬 작업자 자동)
CART_CALL_S = 8.0          # 가정: 대차 호출 -> 셀 앞 도착
CART_DOCK_S = 2.0
CART_RETREAT_S = 2.0
CART_LATERAL_S = 3.0
CART_ROTATE_S = 4.0
CART_LEAVE_S = 5.0

# ===== 타임아웃 =====
HOMING_TIMEOUT_S = 60.0    # R + Z(최대 14 s) + θ 찾기(95° / 5°/s = 19 s) + 대기
EXPECT_TIMEOUT_S = 2.0
JUDGE_TIMEOUT_S = 5.0      # 가정: CAP 뒤 이 시간 안에 PC 판정(JDG)이 없으면 JUDGE_TIMEOUT

# ===== 실장비 연결 (PROTOCOL.md) =====
PROTO_VERSION = 1
SERIAL_BAUD = 115200
HB_PERIOD_S = 0.5          # PC -> Mega 하트비트
LINK_LOSS_S = 1.5          # Mega: 하트비트가 이 시간 끊기면 PAUSED(LINK) + UV 끔
STALE_S = 1.0              # PC: ST 가 이 시간 없으면 '데이터 멈춤'
ACK_TIMEOUT_S = 1.0        # PC: CMD 뒤 ACK 대기
ST_HZ = 20
SL_PERIOD_S = 1.0          # SL 은 바뀔 때 + 1 s 마다
RECONNECT_S = 1.0
FAKE_TCP_PORT = 8770

# ===== 서버 / 카메라 =====
HTTP_HOST = "127.0.0.1"
HTTP_PORT = 8765
SIM_HZ = 50
SSE_HZ = 20
WAFER_CAM_W, WAFER_CAM_H, WAFER_CAM_FPS = 1280, 720, 30
CAM_JPEG_W = 960
CAM_JPEG_Q = 75
CAM_STALE_S = 1.0
CELL_SERIAL_PORT = None    # 셀 Mega COM. 재활용 로봇 MOTOR_PORT/LIDAR 포트와 달라야 함

# 화면에 '가정' 배지를 붙일 설정 이름
ASSUMED = [
    "WAFER_T", "LEDGE_T", "SIDE_OFS", "THETA_STANDBY", "S4_LOCAL_X", "S4_RANGE", "SHARP_TOL", "SHARP_NOISE",
    "TOF_PRESENT_MM", "TOF_CASS_D", "UV_SEAT_Z", "UV_S", "JUDGE_TIMEOUT_S", "CASS_LIFT_Z", "CASS_SEAT_LIFT_Z",
    "CART_DEFAULT_MAP", "CART_BODY", "DESK_X0", "DESK_X1", "DESK_Y0", "DESK_Y1", "DESK_TOP_Z", "DESK_H", "THETA_SPEED", "Z_SPEED", "R_SPEED", "CAPTURE_S", "NG_RATE", "OPERATOR_S",
    "CART_CALL_S", "CART_DOCK_S", "CART_RETREAT_S", "CART_LATERAL_S", "CART_ROTATE_S", "CART_LEAVE_S",
]


def slot_z(n):
    """슬롯 n(1..5) 바닥(웨이퍼 하면) 높이 Sn."""
    return SLOT_Z0 + SLOT_PITCH * (n - 1)


def pick_z(n):
    return slot_z(n) + PICK_OFS


def place_z(n):
    return slot_z(n) + PLACE_OFS


def side_z(n):
    return slot_z(n) + SIDE_OFS


def wrap_deg(a):
    return ((a + 180.0) % 360.0) - 180.0


def in_frame(theta):
    """θ 를 화면 · 프로토콜 범위 [-45, 315) 로."""
    lo, hi = THETA_FRAME
    return (theta - lo) % (hi - lo) + lo


def check_port_conflict(port=CELL_SERIAL_PORT):
    """셀 포트가 재활용 로봇 포트(모터/LiDAR)와 겹치면 에러 문자열, 아니면 None."""
    if not port or str(port).lower() in ("fake", "none") or str(port).lower().startswith("tcp:"):
        return None
    try:
        from common import config as rc
    except Exception:
        return None
    used = {getattr(rc, "MOTOR_PORT", None), getattr(rc, "LIDAR_X4_PORT", None),
            getattr(rc, "LIDAR_PORT", None)}
    if str(port).upper() in {str(p).upper() for p in used if p}:
        return f"cell serial port {port} conflicts with recycling-robot ports"
    return None


def ui_geometry():
    """브라우저 도면용 기하 묶음(JSON 직렬화 가능)."""
    return {
        "cell": CELL,
        "slot_count": SLOT_COUNT, "slot_z0": SLOT_Z0, "slot_pitch": SLOT_PITCH,
        "ledge_t": LEDGE_T, "wafer_d": WAFER_D, "wafer_t": WAFER_T, "cass_top_z": CASS_TOP_Z,
        "s4_local": [BLADE_ORIGIN_X + S4_LOCAL_X, S4_LOCAL_Y],
        "zc_min": ZC_MIN, "zc_soft_max": ZC_SOFT_MAX, "zc_swing": ZC_SWING,
        "blade_plane_ofs": BLADE_PLANE_OFS, "blade_body": list(BLADE_BODY),
        "blade_origin_x": BLADE_ORIGIN_X, "blade_tip_x": BLADE_TIP_X, "wafer_on_blade_x": WAFER_ON_BLADE_X,
        "stations": STATIONS, "station_order": list(STATION_ORDER), "th_sensors": list(TH_SENSORS),
        "station_rx": STATION_RX, "station_r": {k: station_r(k) for k in STATIONS},
        "side_theta": SIDE_THETA, "theta_frame": list(THETA_FRAME), "th_low_half": TH_LOW_HALF_DEG,
        "input_dock_shift": INPUT_DOCK_SHIFT, "cassettes": list(CASSETTES),
        "rx_ret": RX_RET, "rx_ret_ok": RX_RET_OK, "rx_max": RX_MAX,
        "cradle_seat_z": CRADLE_SEAT_Z, "cradle_zc_seat": CRADLE_ZC_SEAT, "cradle_zc_above": CRADLE_ZC_ABOVE,
        "uv_zc_in": UV_ZC_IN, "uv_zc_hand": UV_ZC_HAND, "uv_zc_lamp_max": UV_ZC_LAMP_MAX,
        "uv_seat_z": UV_SEAT_Z, "uv_s": UV_S,
        "cass_z_under": CASS_Z_UNDER, "cass_z_carry": CASS_Z_CARRY, "cass_z_seat": CASS_Z_SEAT,
        "z_sharp_k": Z_SHARP_K, "r_sharp_k": R_SHARP_K, "sharp_tol": SHARP_TOL,
        "corr_dr": list(CORR_DR), "corr_dth": CORR_DTH,
        "theta_standby": THETA_STANDBY,
        "cart_seats": list(CART_SEATS), "cart_seat_kr": CART_SEAT_KR,
        "cart_retreat": CART_RETREAT, "cart_lateral": CART_LATERAL,
        "cart_seat_xy": CART_SEAT_XY, "cart_body": list(CART_BODY), "cart_seat_top_z": CART_SEAT_TOP_Z,
        "cass_plate_under": CASS_PLATE_UNDER,
        "desk": {"x0": DESK_X0, "x1": DESK_X1, "y0": DESK_Y0, "y1": DESK_Y1, "top_z": DESK_TOP_Z, "h": DESK_H,
                 "floor_z": FLOOR_Z},
        "speeds": {"theta": THETA_SPEED, "z": Z_SPEED, "r": R_SPEED, "capture": CAPTURE_S},
        "assumed": ASSUMED,
    }
