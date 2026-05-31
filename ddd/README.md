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
recycling_robot/
├── config/
│   └── data.yaml                # YOLO 학습 설정
├── data/
│   └── scripts/
│       ├── convert_aihub_to_yolo_v3.py  # AIHUB → YOLO 변환
│       └── verify_labels.py             # 라벨 검증 시각화
├── drivers/
│   └── LidarX2.py               # YDLIDAR X2 드라이버
├── inference/
│   └── detect_camera.py         # 실시간 카메라 추론 (예정)
├── visualization/
│   └── realtime_radar.py        # LiDAR 레이더 시각화
├── tests/
│   ├── test_scan.py             # LiDAR 연결 테스트
│   └── webcam_test.py           # 카메라 테스트
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

### 5. LiDAR 레이더 시각화

```bash
python visualization/realtime_radar.py
```

## 진행 상황

- [x] Phase 0: 하드웨어 개별 테스트 (카메라, LiDAR)
- [x] Phase 1: 개발 환경 구축 (PyTorch + CUDA)
- [x] Phase 2: AIHUB 데이터 → YOLO 포맷 변환
- [x] Phase 3: YOLOv8n 모델 학습 (mAP@0.5 = 0.756)
- [ ] Phase 4: 카메라 실시간 추론 테스트
- [ ] Phase 5: LiDAR + 카메라 센서 퓨전
- [ ] Phase 6: Raspberry Pi 5 이식 (ONNX/NCNN)
- [ ] Phase 7~11: 자율주행 통합

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
