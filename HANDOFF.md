# 프로젝트 인수인계 문서 (HANDOFF)

> **작성 기준일:** 2026-06-09
> **검증 방식:** 이 문서의 "사실"은 추측이 아니라 **실제 저장소(코드·git·디스크)를 1차 출처로 확인**한 것이다.
> 각 항목 끝의 표기 — `[P]` 저장소에서 직접 확인 / `[S]` 2차/문서 근거 / `[I]` 추론 / `[불명]` 저장소로 확인 불가(이전 소유자만 앎).
> **핵심 한 줄:** *인지(YOLO 7클래스 + 색상) + LiDAR 거리/안전 + 차동 2모터 주행 펌웨어*는 코드로 존재하나, **(1) 학습 가중치·데이터셋이 git에 없고 (2) 모든 포트/캘리브 값이 이 PC 전용 placeholder이며 (3) 로봇팔·파지·Depth카메라는 코드가 아예 없다.** 따라서 "그대로 실행"은 불가, "이관+보정+신규개발"이 필요하다.

---

## 0. 30초 요약 (TL;DR)

| 질문 | 답 |
|---|---|
| 무엇인가 | 재활용 쓰레기 인식 자율주행 로봇. 노트북(HP VICTUS, RTX 4060)이 두뇌, Arduino Mega가 모터, YDLIDAR(X2/X4)가 거리, 웹캠이 인식. |
| 지금 되는 것 | YOLO 7클래스 탐지 코드, 색상 인식, LiDAR 안전판단, 시각화 도구 10여 종, 모터 펌웨어+수동테스트, 단위테스트 6종 통과. |
| 안 되는/없는 것 | 실하드웨어 통합주행 **미검증**, 로봇팔/그리퍼/파지/Orbbec Depth/분류함/SLAM/odometry **코드 없음**. |
| 새 사람이 첫날 할 일 | ①가중치·데이터셋 받기 ②포트/카메라인덱스 재설정 ③캘리브 측정 ④단위테스트로 코드 무결성 확인. (→ §8 체크리스트) |
| 저장소 | github.com/maeng052097-blip/ROBOT, 프로젝트는 `ddd/` 폴더. `main` = `origin/main` @ `4bb57cd`. [P] |

---

## 1. 목표 (현재 / 중간 / 최종)

- **현재(달성):** 카메라·LiDAR로 물체를 인식하고 거리를 재며, 차동 2륜으로 안전하게 멈추고 움직이는 코드 베이스. [P]
- **중간 목표(이전 소유자 구두):** 카메라가 물체를 인식하면 → 로봇이 **로봇팔이 집기 적합한 위치로 이동** → **로봇팔이 5×5×5cm 물체를 파지** → 차체 저장소에 넣고 **원위치(팔 고정 Depth카메라가 정면을 보는 자세)** 복귀. Depth=**Orbbec Gemini 335**(팔 장착, eye-in-hand). [불명: 코드 없음, 설계도 구두]
- **최종 목표:** 재활용 쓰레기를 **자동 인식·분류**하는 자율주행 로봇. [S: README 명시, 분류부는 미구현]

---

## 2. 하드웨어 구성

### 실제(코드/펌웨어로 확인됨)
| 장비 | 용도 | 비고 |
|---|---|---|
| HP VICTUS 15 (RTX 4060, 8GB) | 두뇌(추론·실행·학습) | [P] README/config |
| Arduino **Mega 2560** | 모터 2개 제어 | Cytron MDD10A, 펌웨어 `urt/arduino_motor` [P] |
| YDLIDAR **X2** | 2D 거리(115200) | `drivers/LidarX2.py` [P] |
| YDLIDAR **X4** | 2D 거리(128000) | `drivers/LidarX4.py` (X2 서브클래스, 0xA5 0x60 start) [P] |
| Logitech StreamCam ×2 | 인식 카메라 | HFOV≈70°(대각78°서 유도, 미실측) [P] |

### 계획(코드 없음 — 신규 개발 대상)
- **Orbbec Gemini 335** Depth 카메라(로봇팔 장착). [불명]
- **로봇팔 + 그리퍼**(5×5×5cm 파지). 모델/축수 미정. [불명]
- **(검토했으나 비권장)** LattePanda DFR0419 — Mega 대체로는 부적합(온보드 MCU가 약한 Leonardo, 핀/시리얼/ESTOP 신뢰성 후퇴). **디스플레이/HMI 노드 용도로만** 의미. 자세한 분석은 §9 참고.

---

## 3. 저장소 구조 (실제 파일, `ddd/`) [P]

```
ddd/
├── README.md                      # 프로젝트 개요(주의: 진행상황 체크박스가 과장됨 §7)
├── HANDOFF.md                     # (이 문서)
├── requirements.txt               # ⚠️ 버전 미고정(unpinned)
├── common/                        # 순수 로직(하드웨어 불필요, 단위테스트 대상)
│   ├── config.py                  #   ★ 모든 포트/임계값/경로 중앙설정 — 새 PC에서 여기부터 수정
│   ├── classes.py                 #   7개 재활용 클래스 + AIHUB 키워드 매핑
│   ├── safety.py                  #   전방 부채꼴 최소거리 → SAFE/SLOW/DANGER
│   ├── fusion.py                  #   카메라 베어링(핀홀)+LiDAR 거리 융합, 시차보정
│   ├── color.py                   #   HSV 대표색/색마스크
│   ├── occupancy_grid.py          #   로그오즈 점유격자(정지 포즈 전용)
│   ├── lidar_metrics.py           #   각도차/최근접점 등 순수 분석함수
│   ├── camera.py                  #   카메라 빠른열기(DSHOW)+디지털줌 크롭
│   └── viz.py                     #   OpenCV 시각화 헬퍼
├── drivers/                       # LidarX2.py, LidarX4.py, __init__.py(make_lidar 팩토리)
├── inference/detector.py          # YOLO 래퍼(best.pt 게이트, 3등분 방향+베어링)
├── config/data.yaml               # YOLO 학습 설정(경로가 외부 절대경로 ⚠️)
├── data/scripts/                  # convert_aihub_to_yolo_v3.py, verify_labels.py
├── models/                        # weights/best.pt, yolov8n.pt ⚠️git 제외 / recycling_v1·v12(학습 산출물 plot·csv만 git 포함)
├── tests/                         # 단위테스트 6종 + 하드웨어 스모크/캘리브 도구 (§6)
├── visualization/                 # 레이더/맵/색/사람/2카메라 등 OpenCV 도구 (§6)
└── urt/                           # 통합주행 + 모터
    ├── 웹캠_LiDAR_주행제어.py      #   ★ 통합 주행 컨트롤러(동작 코드, 단 실HW 미검증)
    ├── 모터2개_시리얼테스트.py    #   모터 수동 테스트(w/a/d/s)
    ├── motor_serial.py            #   시리얼 전송 헬퍼
    └── arduino_motor/             #   Mega 펌웨어(PlatformIO, board=megaatmega2560)
```

- **git 브랜치:** `feature/integrate-vision-lidar-motor`, `feature/lidar-fusion-bringup`, `feature/motor-encoder-ramp` — **모두 `main`에 병합 완료**(미병합 작업 없음). 무시/삭제해도 안전. [P]

---

## 4. 현재 진행 상태 (정직한 구분) [P]

| 상태 | 항목 |
|---|---|
| ✅ **검증됨(단위테스트 통과)** | `test_color`, `test_zoom_geom`(핀홀·시차·클러터), `test_lidar_probe`, `test_occupancy_grid`, `test_lidar_parser`, `lidar_calibrate`(계산). LiDAR 패킷 파서, 융합 기하, 색상, 점유격자 로직. |
| 🟡 **조건부(하드웨어+보정 필요)** | 시각화 도구 전부, `detector`(best.pt 필요), 카메라-LiDAR 융합 거리(캘리브 안 하면 틀림), LiDAR 거리(미보정). |
| 🟠 **미검증** | **통합 주행(`웹캠_LiDAR_주행제어.py`)을 실제 로봇에서 끝까지 돌린 적 없음**(README가 "실하드웨어 통합주행 검증"을 미완으로 표시). mAP 0.756 등 수치는 이전 소유자 보고값(재현 안 됨). |
| ❌ **코드 없음** | 로봇팔/그리퍼/IK/파지, Orbbec Gemini 335 연동, Depth 처리, 분류함 투입, 목표지점 접근 항법, odometry/SLAM(점유격자는 정지 포즈만), `train.py`. |

---

## 5. 환경 설정

- **Python 인터프리터:** 반드시 **`py -3.13`** 사용. 이 인터프리터에 cv2(4.13)·numpy(2.4.4)·pyserial·ultralytics(8.4.60)·torch(2.12 CPU)가 설치돼 있음. [P/I]
  - ⚠️ **VSCode 실행(▶) 버튼은 Python 3.14를 써서 패키지가 없음 → 사용 금지.** 터미널에서 `py -3.13` 로 실행하거나 VSCode 인터프리터를 3.13으로 지정.
  - ⚠️ `.vscode/`는 git 제외(gitignore) → 새 PC엔 launch.json/settings.json이 없음. 직접 인터프리터 지정 필요.
- **패키지:** `requirements.txt`는 **버전 미고정**(ultralytics, opencv-python, pyserial, matplotlib, scikit-learn). torch는 주석(수동 설치: cu124). → 새 PC에서 **버전 고정 권장**(`torch.cuda.is_available()`로 GPU 확인). [P]
- **GPU 학습:** RTX 4060, CUDA 12.4(cu124). [S]

---

## 6. 실행 방법 (모두 `ddd/`에서 `py -3.13`)

### 단위테스트(하드웨어 불필요 — 코드 무결성 확인용, 인수 직후 먼저 실행)
```
py -3.13 tests/test_zoom_geom.py
py -3.13 tests/test_color.py
py -3.13 tests/test_lidar_probe.py
py -3.13 tests/test_occupancy_grid.py
py -3.13 tests/test_lidar_parser.py
```

### 장치/포트 확인 & 캘리브
```
py -3.13 tests/check_devices.py                         # Arduino/LiDAR/카메라 연결 점검
py -3.13 tests/lidar_raw_probe.py --port COM8           # COM이 X2(115200)인지 X4(128000)인지 판별
py -3.13 tests/find_camera.py                           # 카메라 인덱스 찾기
py -3.13 tests/find_front.py                            # FORWARD_ANGLE_DEG 실측
py -3.13 tests/lidar_calibrate.py --pairs 30:39.5,100:109.5,200:209.5   # 거리 SCALE/OFFSET 산출
```

### 시각화 도구(하드웨어 필요)
```
py -3.13 visualization/lidar_probe_view.py --lidar x4 --port COM8   # 단일 레이더 실험뷰
py -3.13 visualization/lidar_dual_view.py                           # X4+X2 동시(정렬: c/a/d/[ ])
py -3.13 visualization/lidar_map_view.py --lidar x2 --port COM12    # 점유격자 맵(정지)
py -3.13 visualization/color_detect.py                              # 색상 인식 + 거리
py -3.13 visualization/dual_camera_aim.py --left 1 --right 2        # 2카메라 + 시차보정 거리
py -3.13 visualization/lidar_camera_aim.py --lidar x4 --port COM8   # 레이더 클릭조준 + 사람감시
py -3.13 visualization/person_cam.py                                # 사람(COCO) 인식
```
(공통 키: `q` 종료, `=`/`-` 카메라줌, `,`/`.` 레이더줌, `a`/`d` cal보정, 좌클릭 조준/포커스)

### 통합 주행 (⚠️ 실HW 미검증 — 모터 들어올린 채 먼저 테스트)
```
py -3.13 urt/모터2개_시리얼테스트.py     # 모터 수동(w/a/d/s) — 먼저 이게 되는지
py -3.13 urt/웹캠_LiDAR_주행제어.py      # 통합 주행(웹캠+LiDAR+Arduino)
```

---

## 7. ⚠️ 인수인계 블로커 & 주의사항 (우선순위순) — **가장 중요한 절**

| # | 심각도 | 항목 | 조치 |
|---|---|---|---|
| 1 | **블로커** | **학습 가중치 `best.pt`가 git에 없음**(`*.pt` gitignore, `git ls-files *.pt`=0). 새 클론엔 가중치 0 → `detector`가 `FileNotFoundError`. [P] | 이전 소유자에게 `best.pt` 파일을 직접 받아 `models/weights/best.pt`에 둘 것. 받은 파일이 recycling_v12(7클래스)인지 확인. 못 받으면 #2로 재학습. |
| 2 | **블로커** | **데이터셋이 외부 절대경로**(`CONVERTED_DATA_DIR=C:\Users\MSY\Desktop\main\data\converted`, `ZIP_ROOT=C:\Users\MSY\Desktop\Training`) — git 밖, 이 PC에만 존재. 다른 PC에선 사라짐. [P] | 변환 데이터셋을 별도 백업받거나, AIHUB 생활폐기물(dataSetSn=140) 재다운로드 후 경로 수정해 `convert_aihub_to_yolo_v3.py` 재실행. |
| 3 | **블로커** | **파지/분류/Orbbec/로봇팔 코드가 아예 없음**(grep: orbbec/gemini/grasp/arm/파지 = 0건). 최종·중간 목표는 신규 개발. [P] | 연속작업이 아니라 **그린필드**로 취급. 이전 소유자에게 팔 모델·그리퍼·Orbbec SDK(cp313 휠 이슈)·원위치 정의·파지 설계를 인터뷰로 받을 것(§10). |
| 4 | 주요 | **모든 포트/인덱스가 이 PC 전용**: MOTOR=COM10, LIDAR=COM8, X2=COM12, CAM=1, 좌1/우2. 새 PC에선 다름. 통합 컨트롤러엔 자동탐색 없음. [P] | 새 PC에서 `find_camera`·`check_devices`·`lidar_raw_probe`+장치관리자로 확인 후 `config.py` 전 값 수정. |
| 5 | 주요 | **config 내부 모순:** `LIDAR_PORT=COM8 + LIDAR_BAUDRATE=115200`(통합 컨트롤러가 **X2**로 사용)인데 `LIDAR_X4_PORT=COM8`(X4=128000). COM8은 둘 다일 수 없음 → 통합 주행 LiDAR 설정이 현재 2-LiDAR 배치와 불일치. [P] | 통합 주행에 쓸 LiDAR를 확정(예: 벽감지=X2@COM12)하고 `LIDAR_PORT`/`LIDAR_BAUDRATE`를 그에 맞게 정정. 필요시 컨트롤러를 `make_lidar`(X4 지원)로 교체. |
| 6 | 주요 | **카메라-LiDAR 융합·LiDAR 거리값이 전부 미보정 placeholder**: `FORWARD_ANGLE_DEG=0`, `CAMERA_HFOV_DEG=70`(유도값), `CAMERA_LIDAR_SIGN=1`, `LIDAR_X2_DIST_SCALE=1.0/OFFSET=0`, `X4/X2_OFFSET_DEG=0`. → 거리/방위가 틀림. [P] | 장착 후 `find_front`(전방각), `lidar_calibrate`(스케일/오프셋), `lidar_dual_view`(yaw 정렬), `test_fusion`(HFOV/부호) 로 실측해 채울 것. **측정 전엔 융합 신뢰 금지.** |
| 7 | 주요 | **2D 스캔평면 한계**(물리): X2/X4는 센서 높이의 수평면을 가르는 물체만 측정. 바닥의 캔/병 등 작은 물체는 평면을 벗어나 거리=None. 캘리브로 해결 불가. [S] | `lidar_range_probe`로 실제 대상물·실제 장착높이의 탐지 가능범위를 측정. 파지용 3D 위치는 LiDAR 말고 **Depth(Orbbec)** 로 계획. |
| 8 | 주요 | **통합 주행 실HW 미검증** + 컨트롤러 docstring이 stale("detect_target_from_webcam은 항상 NONE 자리표시자"라 적혔으나 실제론 YOLO 호출됨). README 체크박스도 과장. [P] | `[x]` 체크박스 불신. 통합주행은 미검증으로 취급, 모터 들고 `decide_drive_command`/ESTOP 경로부터 벤치 테스트. stale docstring 수정. |
| 9 | 경미 | **`train.py`가 없는데 코드가 참조**(`config.py:26`, `detector.py:47-48` "train.py 로 재학습"). [P] | `train.py`(YOLO one-liner 래핑, `models/recycling_v12/args.yaml` 하이퍼파라미터 사용) 추가하거나 참조 문구 정정. |
| 10 | 경미 | **취약한 import**: `웹캠_LiDAR_주행제어.py`가 `from motor_serial import ...` — `python urt/...py`로 실행할 때만 동작(스크립트 폴더 자동 추가 의존). `-m`이나 다른 cwd면 `ModuleNotFoundError`. [I] | `urt/__init__.py`+`from urt.motor_serial import` 로 견고화하거나 sys.path에 스크립트 폴더 명시. |
| 11 | 경미 | **USB 전력/대역폭**: 웹캠2+X2+X4+Arduino를 노트북에 동시 연결 시 프리즈/드롭 이력. [I] | 전원공급 USB 허브 + 호스트 컨트롤러 분산. 동시연결 부하 측정. |

---

## 8. 인수 첫날 체크리스트

1. `git clone` 후 `ddd/`에서 **단위테스트 5종 실행** → 전부 통과하면 코드 무결성 OK(§6).
2. **이전 소유자에게 받기**(§10): `best.pt`, (가능하면)변환 데이터셋, 로봇 배선 사진, 계획 메모.
3. `best.pt`를 `models/weights/best.pt`에 배치 → `tests/webcam_test.py`로 탐지 확인.
4. 새 PC **포트/카메라 인덱스 확정** → `config.py` 수정(§7-4,5).
5. `tests/check_devices.py`로 Arduino/LiDAR/카메라 4종 연결 확인.
6. **캘리브 측정**: `find_front`→`FORWARD_ANGLE_DEG`, `lidar_calibrate`→스케일/오프셋, `test_fusion`→HFOV/부호(§7-6).
7. 모터 **들어올린 채** `모터2개_시리얼테스트.py` → 그다음 `웹캠_LiDAR_주행제어.py`(ESTOP 동작 확인).

---

## 9. 앞으로의 계획 / 로드맵 (검증된 베이스 위에서)

1. **이관 & 보정**(§8): 가중치·데이터 이관, 포트/캘리브 확정. → 기존 인지+주행이 새 PC에서 동작.
2. **통합 주행 실HW 검증**: 벤치(모터 무부하) → 바닥. 안전(ESTOP/DANGER) 우선.
3. **파지로 가는 단계(신규)**:
   a. Orbbec Gemini 335 드라이버 — **별도 Python 3.12 venv**(pyorbbecsdk cp313 휠 부재 가능) + 독립 프로세스로 분리.
   b. `depth_view.py` — RGB+depth 정렬, 클릭 시 3D 좌표(X,Y,Z)+색 출력(기존 click-measure 패턴 재사용).
   c. **핸드아이 캘리브**(카메라→그리퍼) + **원위치 외부 캘리브**(Depth→차체/LiDAR 프레임) — "파지 위치로 이동"의 전제.
   d. 목표지점 접근 항법 → 팔 IK/그리퍼 → 차체 저장소 투입.
4. **(장기)** odometry/엔코더(펌웨어에 ENC 있음) → 움직이는 맵(SLAM), 분류함 분기.
- **참고(검토 완료):** Mega→LattePanda 교체는 **비권장**(얻는 것 없음+ESTOP 신뢰성 후퇴). 온보드 두뇌가 필요하면 **Jetson(CUDA)**, LattePanda는 **디스플레이/HMI 노드**로만.

---

## 10. 이전 소유자에게 반드시 받아야 할 것 (인수 인터뷰)

1. **`best.pt` 파일** + 그것이 recycling_v12(mAP 0.756) run인지 확인. [블로커]
2. **변환 데이터셋**(또는 AIHUB 원본 + 변환 재현 정보). [블로커]
3. **로봇 물리 조립 상태**: 차체/배선 사진, Mega 핀(DIR=22,23 / PWM=6,7 / ENC=2,3), 모터 극성(FORWARD_LEVEL), 엔코더 장착, 펌웨어 업로드 여부. [불명]
4. **계획 상세**: Orbbec Gemini 335 SDK 버전·cp313 우회법·장착 위치, 로봇팔 모델/축수/그리퍼, **원위치 정의**, 파지 시퀀스. [불명]
5. **실제 포트/카메라 인덱스 매핑**과 (있다면) 이미 측정한 캘리브 값. [불명]

---

## 11. 불명(검증 불가) 목록 — 사실로 쓰지 말 것

- `best.pt`의 정확한 출처/버전(git에 없음, 4월 6일자) — recycling_v12 run과 동일한지 불명.
- 디스크의 변환 데이터셋이 공개 가중치를 학습한 그 5,600장과 동일한지(변환 스크립트가 매 실행마다 덮어씀) 불명.
- Orbbec Gemini 335 연동 일체(SDK/장착/캘리브) — 저장소에 없음, 구두만.
- 로봇팔/그리퍼/파지 파이프라인 — 코드·설계문서 없음.
- 로봇이 실제로 조립·배선·업로드된 상태인지 불명.
- 통합 주행이 실로봇에서 **한 번이라도 끝까지 돈 적 있는지** 불명(README 미완 표시).
- 조립 로봇에서의 부호/방향 규약(모터 극성, CAMERA_LIDAR_SIGN, LiDAR yaw 0점) — 모두 placeholder.
