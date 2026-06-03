"""중앙 설정 모듈.

포트 / 통신속도 / 카메라 / 주행 루프 / LiDAR 안전 임계값 / 경로를
한 곳에서 관리한다. 여러 파일에 흩어져 있던 상수를 여기로 모은다.

다른 환경(예: 라즈베리파이)으로 옮길 때는 이 파일의 포트 값만 바꾸면 된다.
"""
from pathlib import Path

# ===== 경로 =====
# 이 파일 위치: <PROJECT_ROOT>/common/config.py  -> parent.parent == PROJECT_ROOT
PROJECT_ROOT = Path(__file__).resolve().parent.parent
MODELS_DIR = PROJECT_ROOT / "models"
# YOLO 학습 가중치. 저장소에는 포함되지 않으므로 직접 이 위치에 두어야 한다.
WEIGHTS_PATH = MODELS_DIR / "weights" / "best.pt"
# YOLO 변환 데이터셋(images/labels) 위치. 대용량이라 저장소 밖(main)에 둔다.
# 현재 실제 데이터는 아래 위치에 있다(5,601장). 환경이 바뀌면 이 값만 수정.
# 재학습(train.py)·라벨검증(verify_labels.py)·변환(convert_*) 이 공유한다.
CONVERTED_DATA_DIR = Path(r"C:\Users\MSY\Desktop\main\data\converted")

# ===== 시리얼 포트 / 통신 =====
# LiDAR와 Arduino는 서로 다른 포트를 사용해야 한다 (보고서 9장).
#   이 PC 기준) Arduino(MOTOR)=COM10, LiDAR=COM8
#   라즈베리파이 예) "/dev/ttyUSB0", "/dev/ttyUSB1"
MOTOR_PORT = "COM10"
MOTOR_BAUDRATE = 115200
LIDAR_PORT = "COM8"
LIDAR_BAUDRATE = 115200
SERIAL_TIMEOUT = 1.0  # 초

# ===== 카메라 / 주행 루프 =====
CAMERA_INDEX = 1  # Logitech(외장). 0 = 노트북 내장캠. tests/find_camera.py 로 확인.
# 캡처 해상도(16:9). HFOV(70°)가 16:9 기준이라 4:3 기본값(640x480) 대신 권장.
CAMERA_WIDTH = 1280
CAMERA_HEIGHT = 720
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
# 카메라 오른쪽(+베어링)이 LiDAR 각도 증가 방향이면 +1, 반대면 -1 (보정 대상).
CAMERA_LIDAR_SIGN = 1
# 베어링 <-> LiDAR 각도 매칭 허용오차(deg).
FUSION_TOL_DEG = 4.0
