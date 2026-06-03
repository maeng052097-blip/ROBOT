# Recycling Waste Detection Robot

재활용 쓰레기를 자동으로 인식하고 분류하는 자율주행 로봇 프로젝트

## 프로젝트 개요

카메라와 LiDAR를 활용하여 재활용 쓰레기(비닐, 스티로폼, 유리병, 종이류, 캔류, 페트병, 플라스틱류)를 실시간으로 인식하고, 물체까지의 거리를 측정하는 시스템입니다.

최종 목표는 Raspberry Pi 5에 탑재하여 자율주행 로봇으로 운용하는 것입니다.

## 시스템 구성

| 장비 | 용도 |
|------|------|
| HP VICTUS 15 (RTX 4060, 8GB VRAM) | 모델 학습 및 검증 |
| Logitech StreamCam x2 | 물체 인식 카메라 |
| YDLIDAR X2 | 2D 거리 측정 (360°, 8m) |
| Raspberry Pi 5 (예정) | 최종 배포 타겟 |

## 인식 가능한 쓰레기 분류 (7개 클래스)

| 클래스 | 영문명 | 학습 데이터 |
|--------|--------|-----------|
| 비닐 | vinyl | 800장 |
| 스티로폼 | styrofoam | 800장 |
| 유리병 | glass_bottle | 800장 |
| 종이류 | paper | 800장 |
| 캔류 | can | 800장 |
| 페트병 | pet_bottle | 800장 |
| 플라스틱류 | plastic | 800장 |

## 학습 결과

- **모델:** YOLOv8n (nano)
- **데이터:** AIHUB 생활 폐기물 이미지 5,600장 (train 4,480 / val 1,120)
- **학습 환경:** NVIDIA RTX 4060 Laptop GPU (8GB VRAM)

| 지표 | 값 |
|------|-----|
| mAP@0.5 | 0.756 |
| mAP@0.5:0.95 | 0.665 |
| 추론 속도 (GPU) | 1.3ms/장 |

### 클래스별 성능 (AP@0.5)

| 클래스 | AP |
|--------|------|
| 스티로폼 | 0.923 |
| 캔류 | 0.759 |
| 비닐 | 0.759 |
| 페트병 | 0.745 |
| 종이류 | 0.739 |
| 플라스틱류 | 0.707 |
| 유리병 | 0.658 |

## 프로젝트 구조

```
ddd/
├── common/                      # 공용 모듈
│   ├── config.py                #   포트·임계값·경로 중앙 설정
│   ├── classes.py               #   재활용 7개 클래스 정의
│   ├── safety.py                #   LiDAR 전방 안전판단(SAFE/SLOW/DANGER)
│   ├── fusion.py                #   카메라 베어링 + LiDAR 거리 융합
│   └── camera.py                #   카메라 빠른 열기(DSHOW) + 16:9
├── config/
│   └── data.yaml                # YOLO 학습 설정
├── data/scripts/                # 데이터 변환·검증
│   ├── convert_aihub_to_yolo_v3.py
│   └── verify_labels.py
├── drivers/
│   └── LidarX2.py               # YDLIDAR X2 드라이버
├── inference/
│   └── detector.py              # YOLO 목표 탐지(좌/중/우 방향)
├── models/weights/best.pt       # 학습 가중치 (직접 배치, git 제외)
├── tests/                       # 하드웨어 스모크 테스트 + 단위 테스트
│   ├── check_devices.py         #   Arduino/LiDAR/웹캠 연결 점검
│   ├── find_camera.py           #   카메라 인덱스 찾기(내장캠 vs Logitech)
│   ├── find_front.py            #   전방각(FORWARD_ANGLE_DEG) 찾기
│   ├── test_scan.py             #   LiDAR 전방거리·안전상태(텍스트)
│   ├── test_lidar_parser.py     #   LiDAR 패킷 파서 단위테스트(HW 불필요)
│   ├── test_fusion.py           #   카메라+LiDAR 물체거리 융합 테스트
│   └── webcam_test.py           #   웹캠 + YOLO 탐지
├── visualization/
│   ├── realtime_radar.py        #   LiDAR 실시간 레이더(극좌표) 시각화
│   └── realtime_view.py         #   카메라 + LiDAR 통합 뷰
├── urt/                         # 통합 주행 + 모터
│   ├── 웹캠_LiDAR_주행제어.py     #   통합 주행 컨트롤러
│   ├── 모터2개_시리얼테스트.py    #   모터 수동 테스트(w/a/d/s)
│   ├── motor_serial.py          #   시리얼 전송 헬퍼
│   └── arduino_motor/           #   Arduino 펌웨어(PlatformIO)
└── requirements.txt
```

## 설치 및 실행

### 1. 환경 설정

```bash
# Python 3.13 가상환경 생성
python -m venv recycling_env

# 활성화 (Windows)
recycling_env\Scripts\activate

# PyTorch 설치 (CUDA 12.x)
pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu124

# 나머지 패키지
pip install -r requirements.txt
```

### 2. 데이터 변환 (AIHUB → YOLO)

```bash
# AIHUB 생활 폐기물 이미지 데이터셋 필요
# https://aihub.or.kr/ (dataSetSn=140)
python data/scripts/convert_aihub_to_yolo_v3.py
```

### 3. 모델 학습

```bash
python -c "
from ultralytics import YOLO
model = YOLO('yolov8n.pt')
model.train(data='config/data.yaml', epochs=100, imgsz=640, batch=16, device=0)
"
```

### 4. LiDAR 테스트

```bash
python tests/test_scan.py
```

### 5. 하드웨어 브링업 / 통합 주행

VSCode 에서 `ddd` 폴더를 열면 Run and Debug(F5) 에 실행 구성이 준비돼 있다. 터미널로도 가능:

```bash
python tests/check_devices.py        # Arduino/LiDAR/웹캠 연결 점검
python urt/모터2개_시리얼테스트.py    # 모터 2개 수동 구동 (w/a/d/s)
python tests/webcam_test.py          # 웹캠 + YOLO 탐지/방향
python urt/웹캠_LiDAR_주행제어.py     # 통합 주행 (웹캠+LiDAR+Arduino)
```

- 학습 가중치 `best.pt` 는 `models/weights/best.pt` 에 둔다(git 제외).
- 포트/임계값은 `common/config.py` 에서 관리. LiDAR 없이 시험하려면 `REQUIRE_LIDAR=False`.
- Arduino 펌웨어는 PlatformIO 로 `urt/arduino_motor` 를 업로드.

## 진행 상황

- [x] 하드웨어 개별 테스트 (카메라, LiDAR, 모터)
- [x] 개발 환경 구축 (PyTorch + CUDA)
- [x] AIHUB 데이터 → YOLO 변환 / YOLOv8n 학습 (mAP@0.5 = 0.756)
- [x] 코드 통합: 공통 설정 · LiDAR 드라이버 · YOLO 탐지 · 모터 제어 연결
- [x] 통합 주행 컨트롤러 (웹캠 방향 + LiDAR 안전 SAFE/SLOW/DANGER + Arduino)
- [x] 하드웨어 브링업 스모크 테스트 (장치점검 / LiDAR / 웹캠)
- [ ] 실하드웨어 통합 주행 검증 및 임계값 튜닝
- [ ] 추가 데이터로 재학습 (정확도 향상)
- [ ] Raspberry Pi 5 이식 (ONNX/NCNN)

## 데이터셋 출처

- [AIHUB 생활 폐기물 이미지](https://aihub.or.kr/aihubdata/data/view.do?currMenu=115&topMenu=100&dataSetSn=140)
- 라벨과 이미지 배치 불일치로 인해 ZIP 카테고리 기반 자동 바운딩박스 생성 방식 사용

## 기술 스택

- **물체 인식:** YOLOv8 (ultralytics)
- **거리 측정:** YDLIDAR X2 (커스텀 드라이버)
- **카메라:** OpenCV + Logitech StreamCam
- **학습 GPU:** NVIDIA RTX 4060 Laptop (8GB VRAM)
- **배포 타겟:** Raspberry Pi 5 + NCNN

## 라이선스

MIT License
