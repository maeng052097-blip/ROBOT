# 웨이퍼 이송셀 CELL_V9 공정 모니터 (`wafer_cell/`)

CELL_V9 셀(10슬롯 카세트 0° → 90° 백라이트 검사 → 180° 반출/인계, 3축 θ1·Z·R)의 공정을
브라우저에서 **실시간 애니메이션 + 상태 대시보드**로 보여준다. **1단계 = 시뮬레이션**이다(실장비 연동은 2단계).

- 설계 원본: Google Drive `ROBOT/웨이퍼 이송셀 CELL_V9/index.html` (2026-09-24 개정 3)
- 새 의존성 없음: 표준 라이브러리 HTTP 서버 + (카메라 쓸 때만) 기존 opencv. 인터넷(CDN) 불필요.

## 실행 (작업폴더 = 저장소 루트, Windows PowerShell)

```powershell
py -3.13 wafer_cell/server.py                  # SIM, 카메라 없음(개념도)
py -3.13 wafer_cell/server.py --cam 1          # 검사 웹캠 인덱스 1 (tests/find_camera.py 로 확인)
py -3.13 wafer_cell/server.py --speed 5        # 5배속
py -3.13 wafer_cell/server.py --lan            # 같은 네트워크에 공개(원격 = 보기 전용)
```
브라우저: **`http://127.0.0.1:8765/`** (localhost 는 ::1 로 풀려 느릴 수 있음). F11 전체화면. `?demo=1` = 시연 모드.
`--lan` 이면 Windows 방화벽 허용 창이 뜬다(개인 네트워크만 허용). 이 PC 에서도 조작하려면 127.0.0.1 로 접속한다.

사용 순서: **원점 복귀 → 카세트 준비(TOF 로 N 확정) → 시작**. 시뮬 전용: 배속, 투입 맵, 고장 주입, E-STOP 시뮬.

## 화면 구성
- 공정 흐름 띠(카세트 취출 → 검사 촬영 → 반출 → 인계) + **지금 하는 일 / 왜**
- 평면도(위에서 본 셀, 문서 좌표), 입면도(블레이드 방향 r–z 단면 + 블레이드 끝 ×4 확대)
- 검사 카메라(라이브 또는 SIM 개념도, STALE/NO SIGNAL 배지, 단계 11 캡처)
- 카세트 웨이퍼: 투입 추정 N(TOF + 연속 가정) / S4 확정 / 남은 수 / 슬롯 10칸 상태 / 불일치 경고
- 축 위치(**지령값**), 센서 칩(기대값 표시, 이름은 마우스 툴팁), 인터록(선회 허용/금지 사유)
- 문서 수치로 계산한 여유(mm)는 화면에서 뺐다(고정값) -> 아래 '현장 확인 절차' 4 참고
- 하단 타임라인(원점 5 + 21단계 + 인계), 조작부, 이벤트 로그

## 정직성 표기 (화면이 거짓 확신을 주지 않게)
| 표시 | 의미 |
|---|---|
| `시뮬레이션 — 실제 장비 아님` | 상시 배너. 데이터 소스 = 시뮬레이터 |
| `축 위치 · 지령값` | 스테퍼는 보낸 스텝으로 계산한 값. 내부 엔코더를 읽지 않음 |
| RX `전진 도달 미확인` | 실린더는 후진(S3)만 센서가 있음 |
| `가정 속도` | 사이클 시간은 가정 속도 + 등속(가속 무시) 기준의 하한값 |
| `추정` / `확인` | TOF 는 최상단 웨이퍼만 봄 → 연속 투입 가정 추정, 픽업 때 S4 로 확인 |
| `데이터 정지` 오버레이 | SSE 1.5s 무응답 또는 tick 정지 → 조작 버튼 비활성 |

## 가정값 (`cell_config.py` 에서 수정, 화면에 `가정` 표기)
| 이름 | 값 | 근거/상태 |
|---|---|---|
| WAFER_T | 2.0 mm | 3D 프린트 모형, 두께 미정 → 문서 측면도 값 |
| THETA_SPEED / Z_SPEED / R_SPEED | 60°/s · 10 mm/s · 20 mm/s | 문서에 없음. Z 10mm/s = 리드 2 에서 모터 1800°/s(현 펌웨어 기본 900 의 2배) [불명] |
| CAPTURE_S / HANDOFF_S | 0.5 s · 5 s | 문서에 없음 |
| STEP11_ZC / STEP21_ZC | 105 | 문서 표에 목표 ZC 없음 → 사용자 승인 기본값 |
| RETURN_VIA | 180 → 90 → 0 역회전 | 슬립링 폐지안(문서 H②D)과도 호환 |
| TOF_Z / TOF_FLOOR_Z / TOF_TOL | 360 · 180 · ±4 mm | 센서 모델/장착 미정 |
| RX_EXT | 51 | 문서 공정값. 실측 스트로크 62 → 51 스토퍼/리밋 필요(62 면 크래들 허브 충돌) |
| PICK_PRESET | `doc` (Sn−66, +5) | 문서 H②E 중앙화안 `centered` (Sn−68.5, +7.5) 로 전환 가능 — 한 쌍으로만 |

## 상태 스냅샷 (SSE `/events`, `/api/state`) — 2단계 시리얼 소스도 같은 필드로 맞출 것
`tick, t, wall, boot_id, source, state, speed, pose{theta,zc,rx}, target, homed, homed_axes, step{key,no,group,title,why,stage,action,actions},
slot, queue, sensors{S1..S5,ESTOP}, expect, tof{d,kind}, cassette{ready,n_est,kind,d,manual,confirmed,remaining,mismatch},
slots[{n,status,sim_present}], occ{blade,C90,C180}, wafers[], alarm, prompt, interlock, counts, faults, c90_verified,
handoff_left, captures[], cam{mode,status,seq,info}, log_seq, new_log[]`

상태: `IDLE, HOMING, READY, RUNNING, WAIT_S5, WAIT_OPERATOR, PAUSED, ALARM, ESTOP, CASSETTE_DONE`.
명령 `POST /api/cmd {cmd, args, expect_state}`: `home, cassette_ready{manual_n}, start, pause, resume, stop, reset,
confirm{choice: empty|retry|c90_clear}, estop` + SIM 전용 `speed, load_map, fault, remove_wafer, debug_hold`.
보호: loopback 클라이언트 + `Content-Type: application/json` + Host/Origin = 127.0.0.1/localhost + 기동 토큰(`X-Wafer-Token`).

## 기록
`wafer_cell/captures/` (git 제외): 단계 11 캡처 `sim_slotNN_wID_시각.jpg`(실카메라일 때), 웨이퍼별 CSV `wafer_log_YYYYMMDD.csv`.

## 현장 확인 절차 (하드웨어 의존 — 코드로 단정하지 않음)
1. **웹캠**: `--cam 1` → 라이브 영상과 LIVE 배지. USB 를 뽑으면 **1초 안에 STALE**. 초점/노출이 백라이트에서 포화되지 않는지 확인.
2. **이중 실행**: 서버를 하나 더 띄우면 `cannot start server on port 8765` 로 종료되어야 한다.
3. **TOF(2단계)**: 빈 카세트, 슬롯 1만, 1~2, … 1~10 을 넣으며 거리 기록. 합격: 인접 차이 12±2mm, 반복 편차 ±3mm,
   |d_empty − d1| ≥ 8mm. 빔 축은 y=0±5mm, x 225~240. 결과를 `TOF_D1`, `TOF_D_EMPTY` 에 기입.
   빔이 선반/뒤블록에 걸리면 빈 카세트를 1장으로 읽는다(고장 주입 `TOF 빔이 선반에 걸림` 으로 시연 가능).
4. **모형 웨이퍼**: 밝은색 필라멘트 권장(검은색은 IR 흡수 → S4/TOF 불안정). 두께·휨 실측 → 아래 여유(mm)와 비교
   (삽입 틈 0.5, 리프트 후 윗 선반 3.5−t, 크래들 재픽업 0.5).

## 2단계(실장비) 착수 조건 — 미구현
제어보드 선정(시퀀스·인터록은 MCU 에서), PC→MCU 하트비트(0.5s, 1.5s 끊기면 안전 지점 정지), 촬영 요청/응답,
입력 ≥ 8(S1~S5, E-STOP, AL×2, R 전진 리밋 51) + TOF I2C, 하드웨어 E-STOP 배선, COM 포트(COM3/COM8 과 달라야 함),
핀맵(`아두이노_핀맵.cell` 은 xlsx/CSV 로 필요).
