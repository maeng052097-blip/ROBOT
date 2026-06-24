"""중앙 설정 모듈.

포트 / 통신속도 / 카메라 / 주행 루프 / LiDAR 안전 임계값 / 경로를
한 곳에서 관리한다. 여러 파일에 흩어져 있던 상수를 여기로 모은다.

다른 노트북/PC로 옮길 때는 이 파일의 포트 값(COM 번호)만 바꾸면 된다.
"""
from pathlib import Path

# ===== 경로 =====
# 이 파일 위치: <PROJECT_ROOT>/common/config.py  -> parent.parent == PROJECT_ROOT
PROJECT_ROOT = Path(__file__).resolve().parent.parent
MODELS_DIR = PROJECT_ROOT / "models"
# YOLO 학습 가중치. 저장소에는 포함되지 않으므로 직접 이 위치에 두어야 한다.
WEIGHTS_PATH = MODELS_DIR / "weights" / "best.pt"
# COCO 사전학습 가중치(person 등 80클래스) — 사람 인식용. 없으면 ultralytics 자동 다운로드.
COCO_WEIGHTS_PATH = MODELS_DIR / "yolov8n.pt"

# ===== 2-카메라(LiDAR 양 옆) 구성 =====
# 좌/우 카메라 인덱스 + LiDAR 중심에서의 가로 오프셋(mm). 좌=-OFFSET, 우=+OFFSET.
CAM_LEFT_INDEX = 1    # 왼쪽 카메라(=하양/white). off_x=-170. cv2 인덱스는 find_camera 로 확인.
CAM_RIGHT_INDEX = 2   # 오른쪽 카메라(=검정/black). off_x=+170.
# ⚠ 색(검정/하양)은 표시 라벨일 뿐 계산과 무관. 중요한 건 '왼/오른쪽 위치'->off_x 부호.
#   인덱스↔좌/우 확인: 왼쪽 카메라를 손으로 가려 어느 창이 어두워지는지로 확정(가려진 쪽=왼쪽=CAM_LEFT_INDEX).
# 두 카메라 간격 34cm -> LiDAR 중앙 기준 각 카메라 ±17cm. 좌=-170, 우=+170 (off_x).
# ⚠ 가정: LiDAR 가 두 카메라 정중앙. 비대칭 장착이면 좌/우 오프셋을 따로 둬야 함(현재 대칭).
CAM_SIDE_OFFSET_MM = 170
# 카메라 toe 각도(deg). 0=평행. ray_bearing 에 가산되는 시차 보정값.
# ※ 2026-06: 카메라를 toe-IN(안쪽 수렴)으로 재장착 -> track 의 state["toe"]는 보통 '양수'에서 맞음
#   (이전 toe-out 은 음수였음). 정확값은 track 에서 n/m 으로 '정면 물체가 좌/우 클릭 모두
#   rb≈0' 되게 현장 튜닝 후 여기 기입. (config 기본값은 0, 미튜닝 시 한 카메라 편향 가능)
CAM_TOE_DEG = 0.0
# YOLO 변환 데이터셋(images/labels) 위치. 대용량이라 저장소 밖(main)에 둔다.
# 현재 실제 데이터는 아래 위치에 있다(5,601장). 환경이 바뀌면 이 값만 수정.
# 재학습(train.py)·라벨검증(verify_labels.py)·변환(convert_*) 이 공유한다.
CONVERTED_DATA_DIR = Path(r"C:\Users\MSY\Desktop\main\data\converted")

# ===== 시리얼 포트 / 통신 =====
# LiDAR와 Arduino는 서로 다른 포트를 사용해야 한다 (보고서 9장).
#   이 PC 기준) Arduino(MOTOR)=COM3, LiDAR(X4)=COM8, LiDAR(X2)=COM12
#   다른 노트북으로 옮기면 장치관리자에서 COM 번호 확인 후 수정
MOTOR_PORT = "COM3"
MOTOR_BAUDRATE = 115200
LIDAR_PORT = "COM8"
LIDAR_BAUDRATE = 115200
# YDLIDAR X4(선택): 128000 bps. X4 의 COM 포트는 장치관리자에서 확인 후 --port 로 지정.
LIDAR_X4_BAUDRATE = 128000

# ===== 2-LiDAR 동시 사용 구성 =====
# 역할: X4=물체 거리 주력(앞-하단), X2=벽 맵핑 주력(뒤-상단). 포트는 장치관리자 기준.
LIDAR_X4_PORT = "COM8"
LIDAR_X2_PORT = "COM12"
LIDAR_X2_BAUDRATE = 115200
# X2 거리 보정 (실측으로 결정): corrected_mm = raw_mm * SCALE + OFFSET_MM.
# tests/lidar_calibrate.py 에 (실제cm:측정cm) 쌍을 넣어 값을 구한 뒤 여기에 기입.
LIDAR_X2_DIST_SCALE = 1.0
LIDAR_X2_DIST_OFFSET_MM = 0
# X4 거리 보정 (동일 공식). 실측 1점(실제130cm->표시132cm, +2cm)이 관찰됐으나
# ⚠ 1점만으론 상수/비율/측정기준점 차이를 구분 못 함 -> lidar_calibrate 로
#   50/100/150/200cm 실측쌍을 모아 산출한 값을 기입하기 전까지 항등 유지.
LIDAR_X4_DIST_SCALE = 1.0
LIDAR_X4_DIST_OFFSET_MM = 0
# 두 LiDAR 방향(yaw) 정렬 오프셋(deg): 같은 물체가 두 LiDAR 에서 같은 방위에 오도록.
# lidar_dual_view 의 'c'(대략 자동) + a/d(미세, 먼 벽 기준)로 맞춘 값을 여기에 기입.
LIDAR_X4_OFFSET_DEG = 0.0
LIDAR_X2_OFFSET_DEG = 0.0

# ===== 점유격자 맵(occupancy grid) =====
MAP_SIZE_M = 8.0     # 맵 반경(m): 맵은 [-SIZE,+SIZE] 정사각 (X2 사거리에 맞춤)
MAP_RES_M = 0.05     # 셀 크기(m) = 5cm
SERIAL_TIMEOUT = 1.0  # 초

# ===== 카메라 / 주행 루프 =====
CAMERA_INDEX = 1  # Logitech(외장). 0 = 노트북 내장캠. tests/find_camera.py 로 확인.
# 캡처 해상도(16:9). HFOV(70°)가 16:9 기준이라 4:3 기본값(640x480) 대신 권장.
# StreamCam 네이티브 = 1920x1080@60 (MJPEG 전용 -> open_camera 가 FOURCC=MJPG 설정).
# ⚠ USB-A 2.0 포트면 1080p30 제한(공식) -> USB 3 포트 사용. 줌 시 픽셀 2.25배 확보.
CAMERA_WIDTH = 1920
CAMERA_HEIGHT = 1080
CAMERA_FPS = 60
LOOP_DELAY = 0.05  # 초. 주행 루프 주기.

# ===== LiDAR 안전 판단 =====
# 차체 전방 ±SAFETY_ARC_DEG 범위의 최소 거리(mm)로 위험/감속을 판단한다.
FORWARD_ANGLE_DEG = 0     # 전방에 해당하는 LiDAR 각도(장착 후 실측해 조정)
SAFETY_ARC_DEG = 30       # 전방 부채꼴 반각(도)
DANGER_MM = 300           # 이보다 가까우면 STOP
SLOW_MM = 700             # 이보다 가까우면 SLOW
# 전방 부채꼴에 측정값이 하나도 없을 때의 처리.
#   True  -> SAFE 로 간주(기본)
#   False -> fail-safe 로 DANGER 처리
EMPTY_ARC_IS_SAFE = True

# LiDAR 데이터 신선도 한계(초). 이 시간 안에 새 데이터가 없으면 '끊김'으로 보고
# 안전하게 DANGER(정지) 처리한다.
LIDAR_MAX_AGE = 0.5

# LiDAR 필수 여부.
#   True(기본) -> 미연결 시 통합 주행을 중단(안전 우선).
#   False      -> 저하 모드로 LiDAR 없이 진행(전방 직진 금지=SLOW 취급, 회전 추적만).
#                 모터/웹캠만 단독으로 시험할 때 사용.
REQUIRE_LIDAR = True

# ===== 비전(YOLO) =====
CONF_THRESHOLD = 0.3  # 탐지 신뢰도 임계값(낮출수록 더 많이 잡힘·오탐↑). 0.25~0.4 권장.

# ===== 카메라-LiDAR 융합 =====
# StreamCam 수평 화각(deg). 공식은 대각 78°만 표기 -> 16:9 기준 유도값(~70.4).
# 융합 테스트(test_fusion)에서 실제와 비교해 보정 권장.
CAMERA_HFOV_DEG = 70.0
# 카메라 오른쪽(+베어링)이 LiDAR 각도 증가 방향이면 +1, 반대면 -1 (카메라 이미지 좌우반전 보정).
CAMERA_LIDAR_SIGN = 1
# ★ 라이다 장착 방향: 뒤집혀(상하 반전) 달려 있고 그 위에 카메라가 정방향으로 올라감.
#   뒤집힘 -> 라이다 각도 증가 방향이 좌우로 '거울 반전'(전방=빨간 점은 그대로).
#   True 면 코드가 raw 각도를 전방축 기준으로 미러링해 로봇 좌표(정면0/우측+)와 맞춘다.
#   유효 부호 = (LIDAR_FLIPPED?-1:+1) * CAMERA_LIDAR_SIGN  (fusion.lidar_dir_sign()).
#   ※ 2026-06 라이다를 '원상태(정방향, 뒤집힘 아님)'로 재장착 -> False 로 변경.
#   ⚠ 재장착 후 미검증: tests/test_flip.py + 정면/우측 20° 물체를 두고 레이더에서
#     오른쪽 물체가 오른쪽에 찍히는지 확인. 반대면 다시 True 로 토글. FORWARD_ANGLE_DEG 도 재측정.
LIDAR_FLIPPED = False
# 베어링 <-> LiDAR 각도 매칭 허용오차(deg).
FUSION_TOL_DEG = 4.0

# ===== 메카넘 물체-접근 (visualization/track_and_approach.py) =====
# 정지 목표 거리(mm) = LiDAR(중앙)에서 물체까지. 18cm 목표.
# ※ 물체가 18cm보다 가까우면 카메라(toe-in 수렴축+근접 사각)에서 사라짐(사용자 실측).
#   그래서 목표 180 + deadband 15 -> 멀리서 접근하면 deadband 상단(~19.5cm)에서 ARRIVED 발화,
#   18cm 사각 진입 '전'에 정지(가시범위 유지). 더 파고들지 않게 함.
APPROACH_TARGET_MM = 180
APPROACH_DEADBAND_MM = 15       # 접근 중 ~19.5cm 에서 도착판정(18cm 사각 진입 전 정지)
APPROACH_MIN_SAFE_MM = 130      # 이보다 가까우면(X4 사거리 한계) 정지(블라인드존 보호)
APPROACH_FACE_TOL_DEG = 8.0     # 베어링이 이 안이면 '정면 향함' -> 전진 허용
APPROACH_ARC_DEG = 6.0          # 추적각 ± 이 부채꼴의 최소거리를 물체거리로 사용
APPROACH_KX = 0.25             # 거리오차(mm) -> vx(%) 비례이득
APPROACH_KW = 1.2             # 베어링오차(deg) -> w(%) 비례이득
APPROACH_VX_MAX = 35           # 전진/후진 속도 상한(%) — 안전상 낮게
APPROACH_W_MAX = 30            # 회전 속도 상한(%)
# ROI 내 색 블롭 최소 면적비(카메라 '감지' 게이트).
# 실측 산술: 9cm 물체@1.3m = ROI(0.10~0.95밴드, 1x)의 3.4% -> 0.04면 미달(줌 강요).
# 클릭-색 잠금과 병행하므로 0.01 로 낮춰도 오탐 위험 낮음(잠근 색만 마스킹).
APPROACH_COLOR_MIN_AREA = 0.01
# 원거리 거리 스파이크 억제(RangeGate): 직전값 ±GATE_MM 만 수용, 게이트 밖/빔 미스는
# HOLD_S 동안 직전값 유지 후 상실. (2m+ 소형 물체는 빔 ~5개 이하라 미스 시 배경값이 튐)
APPROACH_GATE_MM = 300
APPROACH_HOLD_S = 0.8
# 카메라 단독 원거리 접근(Q1-b): LiDAR 가 표적을 아직 못 잡는 구간에서 단안 거리로 서행.
#   - CAM_APPROACH_MIN_MM 까지 왔는데도 LiDAR 미획득이면 정지(스캔평면 문제 -> 맹목 접근 금지)
#   - 전방 ±20° 에 OBSTACLE_STOP_MM 미만 장애물(라이다)이 보이면 정지(표적이 아니어도 안전)
CAM_APPROACH_MIN_MM = 600
CAM_APPROACH_VX = 22        # 서행 전진 속도(%)
OBSTACLE_STOP_MM = 350
