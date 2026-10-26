# 웨이퍼 검사셀 R22 — 인수인계 (새 세션은 여기부터)

- **브랜치**: `claude/gracious-cannon-lh630u`. 기본 브랜치는 `main`입니다.
- **PR**: [maeng052097-blip/ROBOT#1](https://github.com/maeng052097-blip/ROBOT/pull/1)이 열려 있고, 머지되지 않았습니다. 지켜보기는 해제했습니다.
- **직전 큰 변경**: R9 → **R22 전환** + **실장비(LIVE) 연결 경로**(프로토콜 · PC 수신 · 가짜 장비). 커밋 메시지 'R22'를 보면 됩니다.

## 0. 이 폴더와 원본
| 파일 | 내용 |
|---|---|
| `README.md` (이 파일) | 현재 상태, 확정 결정, 해석(확인 대기), 미결정, 다음 할 일, 규칙, 함정 |
| [`sensors.md`](sensors.md) | **R9 시절** 센서 추천 전문 스냅샷(이력). R22 센서는 `../센서_구매목록.md` 맨 위와 `../README.md` '센서' |
| [`plan_history.md`](plan_history.md) | 세션 계획 전부(최신이 위) |

원본 문서
- [`../README.md`](../README.md): 사용법, 공정, 센서, 화면, LIVE, 가정값, 현장 확인
- [`../PROTOCOL.md`](../PROTOCOL.md): 셀 Mega ↔ PC 줄 프로토콜(펌웨어 작성자용, 테스트가 코드와 대조함)
- [`../센서_구매목록.md`](../센서_구매목록.md): R22 최종 주문표가 맨 위에 있음
- 저장소 루트 `CLAUDE.md` §9: 공정 규칙 요약(자동으로 읽힘)
- 사용자 원본: `R22 센서 정리` 문서(MD/HTML, 2026-10-24 업로드). 저장소에는 없습니다. 수치는 `cell_config.py` 주석에 옮겨 두었습니다.

## 1. 현재 상태
| 항목 | 상태 |
|---|---|
| 셀 | **R22**: ① 투입 180(RX 73, +22 mm) · ② 검사 120 · **③ 불량 0 · UV 탑 60**(사용자 변경, 문서는 반대) · 대차 270, 옆길 220. OK 카세트는 없습니다(정상품은 ① 원래 슬롯으로). |
| 시뮬 공정 | 끝까지 동작합니다. 대차 1대(카세트 4) → 20장 → 출발 → 새 대차. 불량은 ③ 위부터, ③ 가득이면 WAIT_NG, 빈 슬롯은 알람 없이 넘어갑니다. |
| 센서 모형 | θ 자리 5(±1.65°), Z/R Sharp(환산식), ToF 2(있음/없음), S4, A_UV · D9 · KSD9700 → 고장 주입 15개, 각각 알람으로 확인했습니다. |
| LIVE | **프로토콜 · PC 수신(`live.py`) · 가짜 장비(`fake_mega.py`) · 서버 연결 · 화면까지** 되어 있습니다. **Mega 펌웨어는 없습니다**(다음 단계). |
| 화면 | 같은 화면이 SIM/LIVE를 보여 줍니다. 연결 배지, 6개 명령 확인 창, 장비 수락/거부 알림이 있습니다. 1920/1536 × 상세 OFF/ON × 14단계(56장)에서 겹침 · 잘림 0, 콘솔 오류 0입니다. 다크/라이트 글자 대비(WCAG 4.5/3.0)를 64 + 8 + 3D 4화면에서 재서 미달을 고쳤습니다(남은 것은 비활성 버튼 · '없음' 슬롯 번호, 의도). |
| 책상 · 대차 | 평면도에 **책상**(674 × 480 mm, 가정) 테두리. 장소 4곳 + 로봇 받침을 감싸고, 대차는 책상 밖 바닥에 도킹합니다(`test_wafer_geometry`가 강제). |
| 3D | **`/3d.html`**(헤더 `3D`, 보기 전용). 같은 SSE 스냅샷으로 로봇 · 카세트 · UV · 대차가 움직입니다. three.js r160을 `web/`에 포함(오프라인). 상자/원기둥 근사입니다. |
| 동기화 | `test_wafer_sync`: 가짜 장비의 `ST`마다 장비 스냅샷과 PC 스냅샷을 비교해 2,413 프레임 불일치 0. LIVE 보간은 `t_src`(ST 수신 시각) 기준, 연결 배지에 마지막 수신 ms, Sharp ≠ 지령이면 주황 경고, 움직이는 LIVE 대차는 점선. |
| 테스트 | 웨이퍼 10개 전부 통과(클라우드 python3 3.11). Windows `py -3.13`에서는 아직 돌려 보지 않았습니다 [불명]. |
| 구매 | R22 주문표를 정리했습니다. EE-SX670 5개는 이미 샀습니다(문서). 나머지는 아직 사지 않았습니다. |

## 2. 실행과 검증
```powershell
# 사용자 PC (Windows, 저장소 루트)
py -3.13 wafer_cell/server.py --cam none --speed 5                  # SIM -> http://127.0.0.1:8765/
py -3.13 wafer_cell/server.py --source live --serial fake --speed 5 # LIVE 경로를 가짜 장비로
Get-ChildItem tests\test_wafer_*.py | ForEach-Object { py -3.13 $_.FullName }
```
```bash
# 클라우드 세션 (Linux, python3)
for t in tests/test_wafer_*.py; do python3 $t | tail -1; done
python3 -m py_compile wafer_cell/*.py
SP=<스크래치패드>
(exec python3 wafer_cell/server.py --cam none --port 8791 --speed 20 > $SP/srv.log 2>&1 & echo $! > $SP/srv.pid)
NODE_PATH=/opt/node22/lib/node_modules node $SP/verify_r22.js $SP/out http://127.0.0.1:8791/ C4,I3,U6 all   # 직접 작성한 점검 스크립트
NODE_PATH=/opt/node22/lib/node_modules node $SP/verify_theme.js $SP/th http://127.0.0.1:8791/ sim         # 글자 대비(스크린샷 화소 기준)
# 3D: headless Chromium 도 SwiftShader 로 WebGL 이 된다. 2D 페이지로 debug_hold 를 걸고 /3d.html 을 캡처
# 주의: 점검 스크립트가 서버를 PAUSED 로 남기면 다음 실행의 'home' 이 거부돼 READY 를 영원히 기다린다 -> 서버를 다시 띄우고 돌린다
kill $(cat $SP/srv.pid)                                                # pkill -f 금지
```
- 화면 고정 · 고장 주입은 `POST /api/cmd`로 합니다(`debug_hold{step,frac}`, `fault{name,on}`, `speed`, `ng_rate`).
  - 조건: loopback, JSON, Host/Origin, 기동 토큰. 토큰은 페이지의 `window.BOOT.token`에 있고 `X-Wafer-Token` 헤더로 보냅니다.

## 3. 확정된 사용자 결정
| 항목 | 결정 |
|---|---|
| 장소 | **4곳**: 투입 · 검사 · UV · 불량. OK 카세트는 없앴습니다. |
| ③ · UV 자리 (2026-10-24) | **맞바꿈**: ③ θ0 · UV θ60. RX · 반경은 장소를 따라감. θ 센서는 그대로, TH0 = ③ · TH60 = UV(UV 켜기 조건 TH60 · θ 60 ± 0.2°). |
| 공정 | 대차 카세트 → ① → 슬롯마다 검사 → 불량은 ③ · 정상은 UV 뒤 **① 원래 자리** → 5장 끝나면 카세트를 원래 대차 자리로 → 대차가 움직입니다. |
| 불량 뒤 | 다음 웨이퍼로 계속합니다. |
| 구조 | **Mega가 공정, PC는 표시**(+ 카메라 판정) |
| 이번 범위 | 화면 · 서버 + 프로토콜(+ 가짜 장비). **Mega 펌웨어는 다음 단계** |
| PC → Mega 명령 | 원점 · 시작 · 일시정지 · 재개 · 정지 · 알람 해제(LIVE에서는 모두 확인 창) |
| UV 시간 | 모름 → 10 s로 가정합니다. |
| 제어보드 | Arduino Mega 2560. E-STOP 사슬 = D9(전원 차단 없음, R9 결정 유지) |

## 4. 내 해석 — 사용자 확인 대기 (`../README.md` '해석')
1. ① 에서 웨이퍼를 든 Z 이동은 모두 옆길 θ220에서 합니다. 옆길 높이 Sn − 57은 문서 'ZC 205 (슬롯 5 옆길)'에서 역산한 값입니다.
2. 카세트 운반은 180 ↔ 220 ↔ 270 짧은 길(슬립링)로 합니다.
3. 원점은 R 먼저 → Z → θ 순서입니다(문서는 Z 먼저).
4. 대차는 카세트 1~4개를 싣고 옵니다(기본 4).
5. 판정 알고리즘은 [미구현]입니다(기본 OK · δ 0).
6. 가정값 35개: 대기 θ120 · UV 받침 z183 · ToF 문턱 120 · Sharp ±3 · 판정 5 s · UV 10 s · 책상 674 × 480(윗면 z −60, 높이 720) · 대차 몸체 380 × 420 등

## 5. 미결정과 [불명]
| 항목 | 왜 중요한가 | 확인 방법 |
|---|---|---|
| Mega 펌웨어 | LIVE의 실제 장비 쪽이 아직 없습니다. | `../PROTOCOL.md`를 따라 작성합니다. 시퀀스는 `sequence.py`, 인터록은 `can_rotate` · `uv_interlock`. 가짜 장비 `fake_mega.py`가 기준 동작입니다. |
| 공정 중 정지 뒤 재시작 | 셀 안에 웨이퍼가 남으면 시뮬은 `cell_not_clear`로 시작을 거부합니다. LIVE 명령 6개로는 '치웠음'을 알릴 방법이 없습니다. | 사용자 결정(명령 추가 또는 펌웨어 규칙) |
| P3 S4 거짓 '없음' | 빈 슬롯으로 믿고 웨이퍼를 든 채 다음 슬롯으로 갑니다. 시뮬에서는 기하 판정이 충돌로 잡습니다(테스트 있음). 실장비에는 그 판정이 없습니다. | S4 모듈을 정할 때 두 번 확인할지 결정합니다(예: 빼기 전과 뒤). |
| S4 모듈 · 자리 | 문서 [보류] | R23 받침 설계 |
| 옆길 틈 · UV 받침 모양 | 계산은 R9 몸체 기준입니다(옆판까지 약 4 mm, 받침은 패드형으로 가정). | R22 도면, 저속 실측 |
| Sharp · θ 센서 · A_UV · TOF-IN 문턱 | 모두 가정값입니다. | `../README.md` '현장 확인 절차' 1–5 |
| 포트 열기 리셋(DTR) | 서버를 재시작하면 Mega가 리셋됩니다. | 운영 규칙(공정 중 재시작 금지) 또는 하드웨어 |
| 대차(AGV) 통신 | 시뮬은 시간 모형입니다. LIVE는 Mega가 보내는 받침 정렬만 씁니다. | 사용자 |
| 판정 알고리즘 | NG 판정과 δr · δθ | 다음 단계 |
| 책상 · 대차 실측 | 화면(2D · 3D) 모양과 도킹 틈 49.5 mm가 가정값입니다. | 책상 네 끝 · 높이, 대차 몸체를 재서 `DESK_*` · `CART_BODY` |
| 3D on 사용자 PC | headless(SwiftShader)로만 확인했습니다. | Chrome/Edge `chrome://gpu` WebGL, `/3d.html` 표시 |
| ③ · UV 교체의 실물 영향 | 불량 경로가 Z105에서 UV 탑 앞을 지남(계산 틈 19.1 mm), UV 빛이 ② 카메라 쪽(60°)으로 샐 수 있음, 배선 이동 | `../README.md` 현장 확인 9 |

## 6. 다음 할 일 (제안 순서)
1. 사용자가 §4 해석 1–6을 확인합니다(틀리면 `sequence.py` · `cell_config.py`를 고치고 테스트를 갱신).
2. 구매: `../센서_구매목록.md` R22 주문표(EE-1001-1 번호는 데이터시트로 확인)
3. Mega 펌웨어 1차: 센서 읽기만(θ 5 · Sharp 2 · ToF 2 · D9 · A_UV)과 `HEL`/`ST`/`SL` 송신 → `server.py --source live --serial COMx`로 화면에 값이 나오는지 봅니다(모터 구동 없음).
4. 펌웨어 2차: `CMD`/`ACK`, `HB` 끊김, 원점 → 한 단계씩 저속으로(CLAUDE.md §7: 사용자 확인 뒤, 하드웨어 E-STOP을 손에 쥐고)

## 6-1. 파일 지도 (`wafer_cell/`)
| 파일 | 역할 |
|---|---|
| `cell_config.py` | 모든 수치(R22 문서 값 + `ASSUMED` 35개, 책상 `DESK_*` · 대차 `CART_BODY`/`CART_SEAT_XY`), 링크 상수, `ui_geometry()` |
| `sequence.py` | 순수 로직. 단계 39개(H · C · P · E · I · N · U · R · L · W), `can_rotate`, `uv_interlock`, `corr_ok`, `describe`, `action_at` |
| `geometry.py` | 순수 로직. 충돌 · 이송 이벤트(`z_move`, `r_move`), `s4_on`, `th_mask`, 여유 |
| `tof.py` | `presence(d)`: 있음 · 없음 · 불가 |
| `texts.py` | 한글 문구 한 곳(알람 · 이벤트 · headline). 장비와 시뮬은 ASCII 코드만 냅니다. |
| `sim.py` | `CellSim` 상태 기계(실장비 Mega 모형). 판정 internal/external, 고장 주입, 스냅샷 |
| `protocol.py` · `PROTOCOL.md` | 줄 프로토콜(체크섬 · 파서 · SPEC 검증)과 펌웨어용 규격 |
| `live.py` | 전송(Serial · TCP · Fake · Queue) + `LiveSource`(스냅샷 변환, 연결 상태, ACK) |
| `fake_mega.py` | 가짜 셀 Mega(`CellSim`을 프로토콜 뒤에 둠). `--tcp 8770` / `--port COMx`로 따로 띄울 수도 있습니다. |
| `source.py` · `judge.py` | 소스 인터페이스(SimSource) · PC 판정 |
| `server.py` | HTTP · SSE · 명령(LIVE 503/504) · 캡처 · CSV |
| `camera_feed.py` | 웹캠(`common/camera.py` 재사용, common 수정 금지) |
| `web/` | 대시보드(평면도 −90° 회전 + 책상, 입면도, 카메라 개념도, 상세 열, LIVE 배지). 하위 폴더 금지(`SAFE_NAME`) |
| `web/3d.html` · `view3d.js` | 3D 보기(보기 전용). `three.module.min.js` · `OrbitControls.js` = three.js r160(MIT, `three.LICENSE`) |
| 테스트 | `tests/test_wafer_{sequence,geometry,tof,sim,protocol,live,fake_mega,sync,server,camera}.py` |

## 7. 작업 규칙 (CLAUDE.md에서 — 어기면 깨진다)
- 실행은 `py -3.13`으로 한다(Windows). 클라우드는 `python3`.
- 콘솔 print와 putText에는 **ASCII만** 쓴다(cp949). em-dash, `→`, `°` 등을 넣으면 앱이 죽는다. 웹 화면 글자는 예외다.
- **커밋과 푸시는 사용자가 요청할 때만** 한다. 다만 이 세션은 정지 훅이 미커밋 변경의 커밋·푸시를 요구했다.
  - 커밋 메시지 끝에 `Co-Authored-By: Claude …`와 `Claude-Session: …` 두 줄을 붙인다.
  - 모델 이름은 커밋, 코드, 문서에 넣지 않는다.
- 모터를 움직이는 명령이나 변경은 사용자 확인 없이 하지 않는다(CLAUDE.md §7, §9).
- 하드웨어 의존값은 [불명]으로 표시하고, 확인 절차(명령과 합격 기준)를 함께 준다.
- 사용자 선호
  - 비판적이고 직설적으로 쓴다. 약점을 먼저 말한다.
  - 근거 없는 칭찬은 하지 않는다.
  - 한국어로 쓰고, 변경 전/후와 가정·불명을 구분한다.
- 간단한 설명에는 Workflow(다중 에이전트)를 쓰지 않는다(사용자 지시).
- 프로세스 종료에 `pkill -f`를 쓰지 않는다. pid 파일로 `kill`한다.

## 8. 함정과 교훈 (실제로 겪음)
| 증상 | 원인 | 대처 |
|---|---|---|
| 사용자 PC 화면이 안 바뀜 | `.bat`이 `git pull`을 안 했다. 로컬 변경 때문에 pull이 막혔다 | `.bat`에 pull을 넣었다. 막히면 `git stash push -u` 후 pull, 그다음 Ctrl+F5 |
| 바탕화면 `.bat`이 `[ERROR] run this file from the ROBOT repo folder` | `%~dp0`가 바탕화면을 가리켰다 | 대체 경로 `C:\Users\MSY\Desktop\ROBOT\`, `...\ROBOT_wafer\` |
| 1536×864에서 하단 띠 잘림 | 미디어쿼리가 기본 규칙보다 앞에 있었다 | 미디어쿼리를 파일 끝으로 옮겼다(새 CSS도 기본 규칙 뒤에 둔다) |
| `debug_hold`가 촬영 단계에서 안 걸림 | hold가 z/r/rot에만 적용됐다 | capture 분기를 추가했다(테스트 있음) |
| 라이트 테마에서 카메라 칸이 검음 | 색이 하드코딩돼 있었다 | `--cambg` 테마 변수 |
| 푸시 non-fast-forward | 사용자가 같은 브랜치에 커밋했다(`928b2a8`) | fetch 후 내 커밋을 rebase하고 푸시. 강제 푸시 금지 |
| 플랜 모드에서 ExitPlanMode 거절 | 사용자가 질문하려고 거절했다 | 질문에 답하고 계획을 갱신한 뒤 다시 요청 |
| seengreat.com, pololu.com 접속 불가 | 클라우드 네트워크 정책 | 검색 결과와 사용자 캡처로 판단하고 [불명]을 표시한다 |
| (R22) 배속 ×50에서 ③ 교체 뒤 포인터가 0으로 안 감 | '빼기 → 새 카세트 놓기'가 한 tick 안에 끝나 TOF-NG '없음'을 못 봄 | 빼는 순간 전이를 기록합니다(테스트 있음). 실 펌웨어도 ToF 샘플 주기보다 빠른 교체에 주의 |
| (R22) 마지막 웨이퍼가 불량이면 L1에서 θ 불일치 | L1이 ① 앞(θ180)에 있다고 가정했는데, 실제로는 ③ 자리(당시 θ60, 지금 θ0)에서 시작 | L1 첫 동작에 θ180 선회를 넣었습니다(시뮬 테스트 ng_rate 1.0이 잡음). |
| (③ · UV 교체) 시뮬 원점 시험이 θ10에서 충돌 | ③이 θ0으로 와서 θ10 · Z140이 '③ ±25° 안 + Z 높음'(BETWEEN) 보수 규칙에 걸림 | 시험 시작을 θ30으로 옮기고, θ10은 충돌로 멈추는 것을 따로 단언. 각도 문구는 `C.STATIONS`로 만든다(PROTOCOL 단계 표는 코드와 대조) |
| (3D) 대차 카세트 4개가 모두 옆(-x)을 봄(사용자 지적) | 카세트 모형(입구 = 로컬 낮은 x)을 대차에 회전 없이 넣음 | 받침마다 `CART_SEAT_FACE`로 돌림: 앞 받침 셀 쪽, 뒤 받침 바깥(180° 돌면 셀 쪽). 2D 대차 · 포크 위 카세트도 같은 모양(`drawCassette`), 기하 테스트가 방향 단언. 같이 2D ① 클램프 윗판이 22 mm 바깥으로 밀려 그려지던 것(좌표 이중 이동)도 고침 |
| (3D) 대차를 통짜 상자로 그렸더니 비스듬한 시점에서 책상 앞을 다 가림 | 대차 몸체(바닥 −780 ~ 받침 133.5)가 셀보다 높고 큼 | 섀시 + 기둥 4 + 상판(속 빈 틀)으로 바꾸고 시점을 높였습니다. |
| (테마) 알람 상태 글자가 반쯤 안 보임 | 깜박임이 글자 투명도 0.35까지 내려감(대비 1.9) | 글자는 그대로 두고 바탕만 깜박이게(`flash`) 바꿨습니다. |
| (R22) 화면 점검 중 서버 처리 예외 1건 | 시뮬 HOLD 해제가 UV 동작을 지워, 램프는 켜졌는데 남은 시간이 None인 상태가 생김 | HOLD 해제는 그대로 이어가게 했고, 문구 생성도 방어했습니다. 서버 루프는 예외가 나도 살아 있게 했습니다(`ERROR loop ...` 로그). |

