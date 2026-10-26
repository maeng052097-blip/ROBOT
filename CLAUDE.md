# CLAUDE.md — AI 협업 가이드라인 (재활용 분류 자율로봇)

이 문서는 **AI(Claude Code 등)가 이 프로젝트를 작업·실행할 때 반드시 따르는 규칙**이다.
새 작업을 시작하기 전에 이 파일을 먼저 읽고, 아래 환경/검증/안전 규칙을 지킨다.
더 깊은 맥락은 [HANDOFF.md](HANDOFF.md)(인수인계 문서), [README.md](README.md)(구조/진행)를 참조.

- Repo: https://github.com/maeng052097-blip/ROBOT (코드는 저장소 루트 — 예전 `ddd/` 경로는 커밋 34abf61 에서 루트로 이동). 기본 브랜치 `main`.
- 아래 포트/경로 값은 **현재 개발 PC 기준**이다. 다른 PC면 장치관리자/`tests/check_devices.py`로 확인 후 `common/config.py` 수정.

---

## 1. 프로젝트 한 줄 요약
재활용 폐기물 분류 **자율로봇**. 카메라로 물체를 고르면(클릭 또는 자율) → 색 잠금 + LiDAR 거리 측정 → **메카넘 휠로 물체 18cm 앞까지 접근·정지**.
핵심 통합 도구 = **`visualization/track_and_approach.py`** (LiDAR + 카메라 2대 + 모터 + 수동/자율 주행).

---

## 2. ★ 환경 규칙 (어기면 바로 깨진다)

- **Python 은 반드시 `py -3.13`** 로 실행한다. cv2 4.13 / numpy / pyserial / ultralytics / torch(CPU) 가 **이 버전에만** 설치돼 있다.
  VSCode ▶ 버튼(3.14)이나 다른 인터프리터엔 패키지가 없다 → 그것으로 실행/판단하지 말 것.
- **콘솔 인코딩은 cp949** 다. `print()` / cv2 메시지에 **cp949 에 없는 문자**(`—` em-dash, `≈`, `⚠`, `→`, `°` 등)를 넣으면 `UnicodeEncodeError` 로 **앱이 죽는다**. 한글은 OK.
  → 새 코드의 print/putText 문자열은 **ASCII 기호만**(`-`, `~`, `->`, `deg`). (GUI 4개 도구는 `sys.stdout.reconfigure(errors="replace")` 가드가 있으나, 그래도 넣지 말 것.)
- **시리얼 포트** (`common/config.py`): 모터 `MOTOR_PORT=COM3`(115200), LiDAR X4 `LIDAR_X4_PORT=COM8`(128000). 모터와 LiDAR 는 **서로 다른 포트**여야 한다.
  Windows 가 COM 번호를 바꾸면 → `track_and_approach.py --port auto`(자동탐지) 또는 config 수정.
- **카메라 2대는 MJPG 필수**. YUY2(무압축) 1080p 두 대는 USB 대역폭 초과로 **한 대가 검게 끊긴다**. `common/camera.open_camera` 가 MJPG 를 설정한다.
  끊기면 → `--width 640 --height 480` 로 낮추거나 두 카메라를 **서로 다른 USB 컨트롤러(허브)** 에 분리 연결.
- **PlatformIO 는 PATH 에 없다**. 펌웨어 빌드는 풀경로 실행: `"$env:USERPROFILE\.platformio\penv\Scripts\pio.exe"`. 업로드 전 시리얼을 점유하는 도구(track tool 등)는 **반드시 닫는다**.
- 셸: Windows PowerShell(주) + Bash 도구(POSIX). 위 셸별 문법 차이 주의(예: 반복문, `&` 호출연산자).

---

## 3. 실행 명령 (작업폴더 = 저장소 루트)

```powershell
# 의존성 설치(최초 1회). torch 는 requirements.txt 주석의 CUDA 안내 참고.
py -3.13 -m pip install -r requirements.txt

# 메인 통합 도구(LiDAR+카메라2+모터). 모터 없이 화면만: --motor-port none
py -3.13 visualization/track_and_approach.py --cam-left 1 --cam-right 2 --port COM8 --motor-port COM3
#   --port auto   : LiDAR COM 자동탐지(모터 포트 제외)
#   키: 좌클릭=추적+색잠금, 우클릭=해제, u=자율주행, g=수동ARM, space=STOP, e=ESTOP,
#       n/m=toe보정, =/-=줌, 방향키/wasd=수동주행, q/ESC=종료

# 라이다 레벨링(수평/전방각 보정): front-min >= 175cm 목표
py -3.13 visualization/lidar_level_view.py --lidar x4 --port COM8

# 카메라 물리 정렬: 전체화면 2창 / 클릭 정량비교
py -3.13 visualization/camera_full_view.py --cam-left 1 --cam-right 2
py -3.13 visualization/camera_align_view.py --cam-left 1 --cam-right 2

# 장치/포트 점검, 카메라 인덱스(검정/하양) 찾기
py -3.13 tests/check_devices.py
py -3.13 tests/find_camera.py

# 펌웨어 빌드+업로드 (Arduino Mega 2560 = mecanum_motor, COM3)
& "$env:USERPROFILE\.platformio\penv\Scripts\pio.exe" run -d urt\mecanum_motor -t upload
```

---

## 4. ★ 검증 절차 (코드 변경 후 반드시 — 빠뜨리지 말 것)

1. 컴파일: `py -3.13 -m py_compile visualization/track_and_approach.py` (변경 파일들)
2. 임포트: `py -3.13 -c "import sys; sys.path.insert(0,'.'); import visualization.track_and_approach"`
3. 단위테스트(전부 **하드웨어 불필요·순수 로직**, 24개 모두 통과해야 함):
   ```powershell
   Get-ChildItem tests\test_*.py | ForEach-Object { py -3.13 $_.FullName }
   ```
   재활용 로봇 14개: test_approach, test_color, test_depth_correct, test_dualcam, test_flip, test_frame_grabber,
    test_fusion, test_lidar_freshest, test_lidar_parser, test_lidar_probe, test_mecanum,
    test_occupancy_grid, test_scan, test_zoom_geom
   웨이퍼셀 10개(§9): test_wafer_sequence, test_wafer_geometry, test_wafer_tof, test_wafer_sim,
    test_wafer_protocol, test_wafer_live, test_wafer_fake_mega, test_wafer_sync, test_wafer_server, test_wafer_camera(가짜 캡처, cv2 없으면 SKIP)
4. **"동작 검증"을 하드웨어 없이 단정하지 말 것.** 하드웨어 의존값(toe 각, 전방각, 포트, 거리 스케일, 카메라 인덱스)은 **[불명]으로 명시**하고, 사용자가 현장에서 확인할 절차(명령/성공기준)를 함께 제시한다.
5. 순수 로직(common/)을 바꾸면 대응 단위테스트도 함께 갱신/추가한다.

---

## 5. 디렉터리 구조

| 경로 | 내용 |
|---|---|
| `common/` | 순수 로직(하드웨어 불필요, 단위테스트 대상). **`config.py`=모든 포트/임계값/경로 중앙설정(여기부터 수정)**, `fusion.py`(시차 거리), `approach.py`(접근 제어법칙), `mecanum.py`(믹싱), `camera.py`, `color.py`, `safety.py`, `lidar_metrics.py`, `occupancy_grid.py` |
| `drivers/` | `LidarX2.py`/`LidarX4.py`(시리얼), `make_lidar()` 팩토리 |
| `inference/` | `detector.py`(YOLO best.pt). ※현재 track 도구엔 미연결 — track 은 **색 기반** 검출 |
| `visualization/` | GUI 도구. **`track_and_approach.py`(메인 통합)**, `lidar_level_view`, `camera_full_view`, `camera_align_view` 등 |
| `urt/` | Arduino 펌웨어(PlatformIO). **`mecanum_motor`=현행 4모터 명령형** / `arduino_motor`=구버전 |
| `tests/` | 단위테스트(`test_*.py`) + 하드웨어 스모크/캘리브 도구(`check_devices`, `find_camera`, `lidar_calibrate` 등) |
| `models/` | 가중치(`weights/best.pt`, git 제외 — 직접 배치) |

---

## 6. 핵심 도메인 사실 (조언·코드가 틀리지 않게)

- **시차(parallax) 핵심**: 카메라가 LiDAR 중앙에서 ±170mm 옆(`off_x`)이라 *카메라 베어링 ≠ 로봇 방위*.
  `fusion.distance_along_ray(dd, off_x, 0, ray_bearing, FORWARD_ANGLE_DEG, ...)` → `(거리, 라이다각)`.
  **`robot_bearing = normalize_deg(lidar_angle - FORWARD_ANGLE_DEG)`**. ★`lidar_bearing()` 재적용 금지(플립 이중적용 버그).
- **카메라 색 라벨(검정/하양)은 표시용일 뿐** — 계산은 좌/우 **위치**(off_x: 좌=−170, 우=+170)로만 한다. 현재 좌=하양, 우=검정. cv2 인덱스↔좌/우는 손가림 테스트로 확정.
- **toe**: 카메라가 **toe-IN(안쪽 수렴)**. track 에서 `n`/`m` 으로 `state["toe"]`(보통 **양수**) 튜닝 — 정면 물체를 좌/우 클릭했을 때 `rb`(robot bearing)가 둘 다 ≈0 → 그 값을 `config.CAM_TOE_DEG` 에 기입.
- **자율 접근 = 양 카메라 평균 조향**: 양 카메라가 동시에 본 robot-bearing(같은 LiDAR 소스로 묶어)을 평균 → 물체를 **로봇 중심(bearing 0 = LiDAR 축)** 에 도착시킨다(한 카메라 편향 상쇄).
- **18cm 목표**: 물체가 18cm 보다 가까우면 카메라(toe-in 수렴축+근접 사각)에서 사라짐 → `APPROACH_TARGET_MM=180`/`DEADBAND=15` 로 접근 시 **~19.5cm 에서 도착정지**(사각 진입 전). X4 최소측정 ~12cm.
- **메카넘/모터**: 펌웨어가 `V vx vy w`(% , −100~100) 수신. 부호 = **vx+전진, vy+우평행, w+우회전**. 5kHz PWM, 데드맨 1.5s. 모터 매핑/방향 보정은 펌웨어 `urt/mecanum_motor/src/main.cpp`(POS_TO_MOTOR/INVERT/STRAFE_SIGN).
- **LiDAR**: 재장착 시 `LIDAR_FLIPPED`/`FORWARD_ANGLE_DEG` 재검증. 레벨링(front-min ≥175cm), 거리 스케일은 현장 보정(`tests/lidar_calibrate.py`).

---

## 7. ★ 안전 (모터를 실제로 움직이는 작업)

- **자율주행('u') 첫 시험은 반드시 '바퀴를 들고'.** 로봇이 스스로 움직인다.
- 안전장치 유지: 차단물 정지, 전방장애 정지, `e`(ESTOP), `space`(STOP), 펌웨어 데드맨 1.5s. **자율 중 안전정지가 걸리면 자율을 해제**한다(노이즈로 재무장 금지).
- AI 는 **임의로 자율/구동을 트리거하지 말 것.** 모터를 구동하는 변경·실행은 사용자 확인을 받는다.

---

## 8. 작업 규칙 (컨벤션)

- 기존 코드 스타일을 따른다(한글 주석, 중앙설정 `config.py` 우선). 새 상수/포트/경로는 `config.py` 에.
- **커밋/푸시는 사용자가 명시적으로 요청할 때만** 한다(자동 커밋 금지). 커밋 메시지 끝에 `Co-Authored-By: ...` 라인.
- 응답 시: **변경 전/후를 명확히 구분**, **가정·불명을 따로 표기**, **검증/실행 절차를 함께 제시**(사용자 선호). 근거 없는 칭찬·추측 금지, 약점부터.
- 새 도구는 `visualization/`, 순수 로직은 `common/`(+ `tests/` 단위테스트). 하드웨어 드라이버는 `drivers/`.

---

## 9. 웨이퍼 검사셀 R22 공정 모니터 (`wafer_cell/`) — 별개 기계

- **인수인계 문서**: 새 세션은 [`wafer_cell/handoff/README.md`](wafer_cell/handoff/README.md)부터 읽는다(현재 상태, 확정 결정, 센서, 미결정, 다음 할 일).
- 재활용 로봇과 **다른 기계**다. §1·§6(메카넘/LiDAR/카메라 시차)과 `common/config.py` 는 적용하지 않는다.
  설정은 `wafer_cell/cell_config.py`(사용자 제공 `R22 센서 정리` 문서 수치 + `가정` 값 목록 ASSUMED, 몸체 치수는 R9 값). 재사용하는 공용 코드는 `common/camera.py` 뿐(수정 금지).
- 구조: **Mega 가 공정(권한), PC 는 표시 · 카메라 판정 · 운전 명령 6 개**(원점/시작/일시정지/재개/정지/알람 해제). 규격 = `wafer_cell/PROTOCOL.md`
  (`$TYPE,k=v*HH` XOR, PC->Mega <= 60 B). 같은 화면이 SIM(`CellSim`)과 LIVE(`live.py`)를 `source.py` 인터페이스로 받는다.
  실행: `py -3.13 wafer_cell/server.py [--cam 1|none] [--speed 5] [--source live --serial fake|tcp:H:P|COMx]` -> `http://127.0.0.1:8765/`.
  3D 보기 = `/3d.html`(헤더 `3D`, 보기 전용, three.js r160 을 `web/` 에 포함 — CDN 금지, `web/` 은 하위 폴더 없음 = server `SAFE_NAME`).
  동작 과정 그림 = `docs/process_flow.svg`(`flow_diagram.py` 가 단계 동작 · 수치에서 생성). 자리 · 단계 · 높이를 바꾸면 `py -3.13 wafer_cell/flow_diagram.py` 로 다시 만든다(`test_wafer_sequence` 가 비교).
  책상(`DESK_*`, 674x480 가정)이 장소 4곳 + 로봇 받침을 감싸고, 대차는 책상 밖 바닥(앞면 y -189.5 < 책상 끝 -140)이다 — `test_wafer_geometry` 가 강제.
  LIVE 화면 = 장비 상태는 `test_wafer_sync`(가짜 장비 ST 마다 스냅샷 비교, 불일치 0)가 보증한다. 스냅샷 키를 늘리면 SIM · LIVE · 비교 목록을 같이 고친다.
  Mega 펌웨어는 아직 없다(다음 단계). **AI 는 LIVE 를 가짜 장비(`--serial fake`, `wafer_cell/fake_mega.py`)로만 쓴다** — 실제 COM 으로 명령을 보내는 것은
  사용자가 화면 버튼 + 확인 창으로만 한다(§7). 실제 COM 을 열면 Mega 가 리셋(DTR)된다 -> 공정 중 서버 재시작 금지.
- 셀: ① 투입 θ180(RX 73, 도크 +22 mm) · ② 검사 120(RX 51) · ③ 불량 0(RX 51) · UV 탑 60(RX 85) · 대차 270(RX 85). 옆길 없음(사용자 변경, 문서는 220). 카세트 5 슬롯 Sn = 190 + 18(n-1).
  ※ ③ · UV 자리는 **사용자 변경**(R22 문서는 UV 0 · ③ 60). 각도는 `cell_config.STATIONS` 한 곳 — 코드 · 화면 · 3D 는 이름으로 따라간다.
  θ 센서 이름: TH0 = ③, TH60 = UV(`UV_TH_SENSOR`, UV 켜기 조건 'θ 60 ± 0.2°'). 각도를 문구에 직접 쓰지 말고 `C.STATIONS` 로 만든다.
- 확정된 공정 규칙(사용자 설명/R22 문서, 코드·테스트가 강제):
  대차 카세트 -> ① -> 슬롯 1->5: 집기 -> ② 촬영 · PC 판정(OK/NG + δr, δθ) -> 보정 다시 집기(RX 51+δr, θ120+δθ, 창 δr -3.5~+0.5 · |δθ|<=0.5)
  -> NG 는 ③ 위 슬롯부터 · 다음 웨이퍼 / OK 는 UV 받침(ZC 121.5 넣고 113.5 넘김) -> ZC<=110 · R 후진에서 UV -> 다시 집어 ① 원래 슬롯
  -> 5 장 끝 -> ① 카세트를 원래 받침에 -> 대차 다음 받침 / 다 하면 출발. ③ 가득/없음 -> WAIT_NG(TOF-NG 없음->있음 = 새 카세트).
  P3 S4 OFF = 빈 슬롯(알람 아님). 선회는 Z105 · RX<=2(R Sharp), 예외: 높이 선회 ② 120<->① 180(사용자 변경 - 웨이퍼를 든 채 transfer_z 그대로, RX<=2, ② 크래들 띠 z244~252 ±5 밖, Z105 로 내리지 않음 · 되돌릴 때 높이는 ② 앞에서), 카세트 운반 180<->270(Z241), ② 미세 0.5°(Z183).
  UV 탑 밑(RX>2)은 ZC 113.5~121.5 만, ZC 소프트 상한 245. UV 램프 = 7 조건(`sequence.uv_interlock`). 원점 = R 후진 -> Z(Sharp) -> θ 를 - 쪽 5°/s, 95° 안 첫 LOW.
  하트비트 1.5 s 끊김 -> UV 즉시 끔 + 축 동작 끝낸 뒤 PAUSED(LINK). 판정 5 s 없음 -> JUDGE_TIMEOUT.
- 내 해석(사용자 확인 대기, `wafer_cell/README.md` '해석'): ① 앞 든 채 Z 이동은 다른 슬롯 높이를 안 지날 때만(들기 높이 SIDE_OFS -57 은 '205 슬롯 5 옆길' 역산),
  카세트 운반 짧은 길, 원점 R 먼저, 대차 카세트 1~4(기본 4), 판정 알고리즘 [미구현](기본 OK · δ 0).
- 화면 수치는 **지령값**이다(실측은 θ 자리 센서 · Sharp 로만). 하드웨어 의존값(Sharp 환산 · 반복 정밀도, θ ±1.65°, A_UV 문턱,
  TOF-IN 문턱(도크 +22 로 다시 티칭), 높이 선회 틈(② 크래들 · 백라이트), UV 받침 모양, S4 모듈 [보류], 축 속도, UV 시간, 대차 통신)은 [불명]/가정 — 확인 절차는 `wafer_cell/README.md`.
- 조작 API 는 loopback + JSON + Host/Origin + 기동 토큰 필수. 웹 '정지'는 소프트 정지이며 E-STOP 이 아니다(E-STOP 사슬 = D9).
