"""track_and_approach.py - LiDAR + 2카메라(양옆) 클릭 추적 + 색/거리(시차보정) + 메카넘 접근.

장착(2026-06 재설계):
  - 카메라 2대가 LiDAR 양옆에 부착(간격 34cm). 왼쪽=하양/white(off_x -170mm),
    오른쪽=검정/black(off_x +170mm). LiDAR=중앙. ※색은 라벨일 뿐, 계산은 좌/우 위치로만.
  - LiDAR 는 '원상태(정방향)'로 재장착(config.LIDAR_FLIPPED=False, ⚠재검증). 전방=FORWARD_ANGLE_DEG.

핵심(시차 보정):
  카메라가 LiDAR 에서 10cm 옆이라 '카메라 베어링 != 로봇 방위'. 클릭한 카메라의 off_x 로
  fusion.distance_along_ray 를 쓰면 (거리, 라이다각도)를 얻고, 그 라이다각도가 곧 '로봇 기준'
  방위(검증: 정면 물체 -> 0°, 우측 20° -> 20°). 이 로봇 방위로 접근을 조향한다.
  ⚠ 로봇 방위 = signed_diff(lidar_angle, FORWARD) - lidar_bearing() 재적용 금지(플립 이중적용).

흐름(수동):
  1) 두 카메라 중 한쪽 화면에서 물체 클릭 -> 그 지점 색 잠금(화면은 3분할, 합치지 않음).
  2) 매 프레임 '양 카메라'에서 잠근 색을 검출: 둘 다 보이면 둘 다 포커싱(각자 박스),
     한쪽만 보이면 그쪽만. 시차로 거리/로봇방위 산출(둘 다면 박스 큰 쪽이 대표), 레이더 표시.
  3) 색 + LiDAR 거리 동시 감지 & ARM('g') -> 메카넘으로 목표거리(기본 18cm) 접근/정지.
흐름(자율 'u'):
  클릭 없이 매 프레임 '가장 큰 유채색 블롭'을 자동 표적선택 -> 자동 ARM -> 양 카메라가 동시에
  보며 그 평균 방위로 조향(물체가 '로봇 중심'에 오도록) -> 18cm 앞까지 접근/정지.
  (물체는 18cm보다 가까우면 카메라 사각이라, ~19.5cm 도착판정 또는 근접상실=도착으로 정지.)
  원거리에서 표적을 오래 놓치면 해제하고 재탐색. space/e/g 는 자율을 끈다.
  ⚠ 자율은 로봇이 스스로 움직임 -> 첫 시험은 반드시 '바퀴 들고'. 차단물·데드맨 안전 유지.

안전: 기본 DISARM. 차단물(<35cm)·도착·놓침·스테일 -> STOP+DISARM. 펌웨어 데드맨 1.5s.
  ⚠ 융합상수(LIDAR_FLIPPED/FORWARD_ANGLE_DEG/HFOV/CAM_TOE_DEG/카메라인덱스)는 재장착 후
    하드웨어로 재검증할 것. 첫 ARM 은 반드시 '바퀴 들고'.

실행: py -3.13 visualization/track_and_approach.py --cam-left 1 --cam-right 2 --port COM8 --motor-port COM3
키:
  좌클릭(좌/우 카메라)=그 물체 추적 + 색 잠금 | 좌클릭(레이더)=로봇방위 지정(색없음/표시용)
  우클릭=해제, u=자율주행(클릭없이 자동접근) 토글, g=수동ARM/DISARM, space=STOP, e=ESTOP, ESC=종료
  수동 주행(hold-to-drive): 방향키 ↑↓=전후 ←→=좌우회전 | w/s a/d z/c q/r v/b (메카넘 8방향)
    1/2/3=속도 20/35/50%, x=미세모드
  카메라줌 =/- (양 카메라 공유), 세로앵커 i/k, 포커스 f(AF토글) ,/.(±5, 양 카메라 동기)
  toe보정 n/m (±0.5°): toe-out 카메라의 시차 거리 보정. 정면 물체를 좌/우 클릭 모두 rb~0 되게.
  레이더범위 [ ]
  접근알림: 20cm 이내 진입 시 5초 배너(영문 'WITHIN 20 cm', 콘솔 한글)
"""
import argparse
import math
import sys
import time
import pathlib
from collections import deque

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))

try:    # 콘솔(cp949)에 없는 문자를 print 해도 앱이 죽지 않게(? 로 대체). 한글은 그대로.
    sys.stdout.reconfigure(errors="replace")
    sys.stderr.reconfigure(errors="replace")
except Exception:
    pass

from common.config import (                       # noqa: E402
    LIDAR_X4_PORT, LIDAR_X4_BAUDRATE, MOTOR_PORT, MOTOR_BAUDRATE,
    CAMERA_HFOV_DEG, CAMERA_LIDAR_SIGN, FORWARD_ANGLE_DEG,
    CAMERA_WIDTH, CAMERA_HEIGHT, CAMERA_FPS,
    CAM_LEFT_INDEX, CAM_RIGHT_INDEX, CAM_SIDE_OFFSET_MM, CAM_TOE_DEG,
    APPROACH_TARGET_MM, APPROACH_DEADBAND_MM, APPROACH_MIN_SAFE_MM,
    APPROACH_FACE_TOL_DEG, APPROACH_ARC_DEG, APPROACH_KX, APPROACH_KW,
    APPROACH_VX_MAX, APPROACH_W_MAX, APPROACH_COLOR_MIN_AREA,
    APPROACH_GATE_MM, APPROACH_HOLD_S,
    CAM_APPROACH_MIN_MM, CAM_APPROACH_VX, OBSTACLE_STOP_MM,
)
from common.fusion import (                        # noqa: E402
    lidar_bearing, bearing_to_lidar_angle, view_x_from_bearing, view_bearing_deg,
    effective_half_fov_deg, min_distance_in_arc, monocular_range_mm, blocking_distance,
    distance_along_ray,
)
from common.lidar_metrics import angular_diff, normalize_deg   # noqa: E402
from common.camera import (                           # noqa: E402
    open_camera, crop_center_zoom, set_manual_focus, enable_autofocus, FOCUS_STEP,
    FrameGrabber, camera_info,
)
from common.color import dominant_color, color_mask  # noqa: E402
from common.approach import approach_command, cam_approach_command, RangeGate  # noqa: E402

CHROMATIC = {"red", "orange", "yellow", "green", "cyan", "blue", "purple", "pink"}
PERP_TOL_MM = 200.0   # distance_along_ray: 시선-점 수직 허용
MANUAL_KEYS = {
    ord('w'): (1, 0, 0), ord('s'): (-1, 0, 0),
    ord('a'): (0, 0, -1), ord('d'): (0, 0, 1),
    ord('z'): (0, -1, 0), ord('c'): (0, 1, 0),
    ord('q'): (1, -1, 0), ord('r'): (1, 1, 0),
    ord('v'): (-1, -1, 0), ord('b'): (-1, 1, 0),
}
MANUAL_HOLD_S = 0.6
FINE_HOLD_S = 0.25
FINE_SPEED_MUL = 0.4
ARROW_KEYS = {2490368: (1, 0, 0), 2621440: (-1, 0, 0),
              2424832: (0, 0, -1), 2555904: (0, 0, 1)}
NOTICE_MM = 200.0
NOTICE_S = 5.0
NOTICE_REARM_MM = 250.0
FOLLOW_ALPHA = 0.5
REACQ_MISS_N = 5
# 자율('u'): 클릭 없이 가장 큰 유채색 블롭을 자동 표적선택 -> 자동 ARM -> 접근.
AUTO_MIN_AREA = 0.02   # 자동선택 최소 면적비(다운스케일 뷰 기준). 작은 노이즈 제외.
AUTO_RELEASE_S = 1.5   # 표적을 이 시간 이상 놓치면 해제(재탐색 or 도착판정).
CLOSE_ARRIVED_MM = 240  # 마지막 본 거리가 이 안(<=24cm)에서 사라지면 '도착'(18cm 사각)으로 보고 재탐색 금지.


def classify_click(x, cam_w):
    """캔버스 x -> ('L'|'R'|'radar', 패널내 x). 레이아웃: camL | camR | radar (각 cam_w)."""
    if x < cam_w:
        return "L", int(x)
    if x < 2 * cam_w:
        return "R", int(x - cam_w)
    return "radar", int(x - 2 * cam_w)


def side_off_x(side):
    """카메라 side -> LiDAR 기준 가로 오프셋(mm). 왼쪽(L)=-, 오른쪽(R)=+ (색 무관)."""
    return -CAM_SIDE_OFFSET_MM if side == "L" else (CAM_SIDE_OFFSET_MM if side == "R" else 0.0)


def side_toe(side):
    """카메라 side -> toe 보정(deg). L 은 +CAM_TOE_DEG, R 은 -CAM_TOE_DEG (안쪽 수렴 가정)."""
    return CAM_TOE_DEG if side == "L" else (-CAM_TOE_DEG if side == "R" else 0.0)


def main():
    ap = argparse.ArgumentParser(description="LiDAR + 2카메라 클릭추적 + 시차거리 + 메카넘 접근")
    ap.add_argument("--port", default=LIDAR_X4_PORT, help="LiDAR COM 포트. 'auto'=자동탐지(모터 제외)")
    ap.add_argument("--baud", type=int, default=LIDAR_X4_BAUDRATE)
    ap.add_argument("--lidar", default="x4", choices=["x2", "x4"])
    ap.add_argument("--cam-left", type=int, default=CAM_LEFT_INDEX, help="왼쪽(=하양) 카메라 인덱스")
    ap.add_argument("--cam-right", type=int, default=CAM_RIGHT_INDEX, help="오른쪽(=검정) 카메라 인덱스")
    ap.add_argument("--motor-port", default=MOTOR_PORT, help="모터 COM. 'none'이면 구동 비활성")
    ap.add_argument("--target-mm", type=float, default=APPROACH_TARGET_MM)
    ap.add_argument("--rmax", type=float, default=3000.0, help="레이더 표시 최대거리(mm)")
    ap.add_argument("--roi-half-deg", type=float, default=6.0)
    ap.add_argument("--roi-y1", type=float, default=0.10)
    ap.add_argument("--roi-y2", type=float, default=0.95)
    ap.add_argument("--anchor-y", type=float, default=0.65)
    ap.add_argument("--focus", type=int, default=-1, help="시작 수동포커스(0~250). -1=AF")
    ap.add_argument("--min-area", type=float, default=APPROACH_COLOR_MIN_AREA)
    ap.add_argument("--ui-scale", type=float, default=1.2, help="내부 렌더 배율(2카메라+레이더라 1.2 기본)")
    ap.add_argument("--win-w", type=int, default=1600, help="창 초기 가로폭(px). 화면보다 작게. 더 넓은 캔버스는 비율유지 축소")
    ap.add_argument("--obj-width-cm", type=float, default=9.0)
    ap.add_argument("--cam-offset-mm", type=float, default=float(CAM_SIDE_OFFSET_MM))
    # 두 카메라 동시구동 대역폭 조절(검정 끊김 방지). YUY2 면 낮추거나 USB 분리.
    ap.add_argument("--width", type=int, default=CAMERA_WIDTH, help="카메라 캡처 가로(대역폭 부족 시 640)")
    ap.add_argument("--height", type=int, default=CAMERA_HEIGHT, help="세로(640이면 480)")
    ap.add_argument("--fps", type=int, default=CAMERA_FPS, help="FPS(30으로 낮추면 대역폭 절반)")
    args = ap.parse_args()

    import cv2
    import numpy as np
    from drivers import make_lidar

    import serial.tools.list_ports as _lp

    def _avail_ports():
        try:
            return [(p.device, (p.description or "").encode("ascii", "replace").decode())
                    for p in _lp.comports()]
        except Exception:
            return []

    lidar = None
    if str(args.port).lower() == "auto":
        # 자동탐지: 모터 포트를 빼고 각 COM 을 열어 '라이다 데이터가 오는' 포트를 찾는다.
        cands = [d for d, _ in _avail_ports() if d != args.motor_port]
        print(f"[라이다 자동탐지] 후보 {cands or '(없음)'}")
        for d in cands:
            lz = make_lidar(args.lidar, d, args.baud)
            if not lz.open():
                continue
            t0 = time.time(); got = False
            while time.time() - t0 < 1.5:
                if lz.is_fresh(0.5):
                    got = True; break
                time.sleep(0.05)
            if got:
                lidar = lz; print(f"[OK] 라이다 자동탐지 = {d}"); break
            lz.close()
        if lidar is None:
            print("[경고] 라이다 자동탐지 실패 - USB/전원 확인. 거리/추적 비활성")
    else:
        lidar = make_lidar(args.lidar, args.port, args.baud)
        if lidar.open():
            t0 = time.time()    # 포트는 열려도 모터가 안 돌면 데이터가 없을 수 있다(스핀업/STOP상태)
            while time.time() - t0 < 2.0 and not lidar.is_fresh(0.5):
                time.sleep(0.05)
            if lidar.is_fresh(0.5):
                print(f"[OK] LiDAR({args.lidar}) {args.port} 스트리밍 중")
            else:
                print(f"[주의] LiDAR({args.lidar}) {args.port}: 포트는 열렸으나 데이터 없음"
                      f" -> 모터 회전/전원 확인(스핀업 중이면 곧 들어옴). 계속 진행")
        else:
            ports = _avail_ports()
            names = ", ".join(d for d, _ in ports) or "(COM 포트 없음 = 장치 미연결)"
            print(f"[경고] LiDAR 열기 실패 {args.port} -> 거리/추적 비활성")
            print(f"  현재 COM 포트: {names}")
            for d, desc in ports:
                print(f"    {d}: {desc}")
            print("  -> 라이다 USB/전원 확인 후 위 목록의 라이다 포트를 --port 로 지정 (또는 --port auto)")
            lidar = None

    # ---- 두 카메라 (각각 선택적) ----
    cams = {}    # side -> {"cap","grabber","ok","off_x"}
    for side, idx in (("L", args.cam_left), ("R", args.cam_right)):
        cap = None
        try:
            cap = open_camera(idx, args.width, args.height, args.fps)
            if cap.isOpened():
                name = "하양(L)" if side == "L" else "검정(R)"
                info = camera_info(cap)
                print(f"[OK] {name} 카메라 index {idx} ({info})")
                if "MJPG" not in info:
                    print(f"  [주의] {name}: MJPG 미적용(YUY2) -> 두 카메라 동시구동 시 USB 대역폭"
                          f" 초과로 끊길 수 있음. '--width 640 --height 480' 로 낮추거나 두 카메라를"
                          f" 서로 다른 USB 포트(허브)에 분리 연결하세요.")
                cams[side] = {"cap": cap, "grabber": FrameGrabber(cap), "ok": True,
                              "off_x": -args.cam_offset_mm if side == "L" else args.cam_offset_mm}
            else:
                print(f"[경고] {side} 카메라 index {idx} 열기 실패")
                cams[side] = {"cap": None, "grabber": None, "ok": False, "off_x": 0.0}
        except Exception as exc:
            print(f"[경고] {side} 카메라 오류: {exc}")
            cams[side] = {"cap": None, "grabber": None, "ok": False, "off_x": 0.0}
    if not any(c["ok"] for c in cams.values()):
        print("[경고] 카메라 둘 다 실패 -> 레이더 클릭만 가능")

    ser = None
    if args.motor_port.lower() != "none":
        try:
            import serial
            ser = serial.Serial(args.motor_port, MOTOR_BAUDRATE, timeout=0.2)
            time.sleep(2.0)
            print(f"[OK] 모터 {args.motor_port}")
            try:
                ser.reset_input_buffer()
            except Exception:
                pass
        except Exception as exc:
            print(f"[경고] 모터 포트 실패({args.motor_port}): {exc} -> 구동 비활성")
            ser = None
    else:
        print("[정보] --motor-port none -> 구동 비활성(표시만)")

    def send(line):
        if ser is None:
            return
        try:
            ser.write((line + "\n").encode("ascii"))
        except Exception:
            pass

    # ---- 화면: camL | camR | radar ----
    sui = max(1.0, args.ui_scale)
    CAM_W, CAM_H = int(640 * sui), int(360 * sui)
    RAD = CAM_H
    cx0, cy0 = RAD // 2, RAD // 2
    win = "track_and_approach (dual cam)"
    # 캔버스(카메라2+레이더)가 화면보다 넓으면 AUTOSIZE 창은 오른쪽(레이더)이 잘린다.
    # 리사이즈 가능 창 + 화면에 맞춘 초기 폭(캔버스 비율 유지)으로 레이더까지 보이게.
    cv2.namedWindow(win, cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)
    _canvas_w = 2 * CAM_W + RAD
    _disp_w = min(_canvas_w, args.win_w)
    cv2.resizeWindow(win, int(_disp_w), max(1, int(CAM_H * _disp_w / _canvas_w)))

    # mode: None | 'cam'(카메라 잠금) | 'radar'(로봇방위만)
    state = {"mode": None, "cam_side": None, "robot_bearing": 0.0,
             "color": None, "armed": False, "rmax": float(args.rmax), "cam_zoom": 1.0,
             "anchor_y": max(0.0, min(1.0, args.anchor_y)),
             "af": args.focus < 0, "focus": max(0, min(250, args.focus)),
             "views": {"L": None, "R": None},
             "manual": None, "manual_t": 0.0, "mspeed": 30,
             "notice_t": -1e9, "notice_armed": True, "fine": False, "wait_t": 0.0,
             "toe": float(CAM_TOE_DEG),   # 카메라 toe 보정(n/m 키). 양 카메라 시차거리 보정.
             "cb": {"L": 0.0, "R": 0.0},  # 각 카메라가 추종 중인 베어링(deg)
             "miss": {"L": 0, "R": 0},    # 각 카메라 색 상실 프레임 카운트(재탐색용)
             "auto": False, "lost_t": 0.0,  # 자율주행 on/off + 표적 상실 시각
             "last_seen_range": None}       # 마지막으로 본 표적 거리(근접 도착 판정용)

    range_gate = RangeGate(APPROACH_GATE_MM, APPROACH_HOLD_S)
    hist = deque(maxlen=3)
    hist_t = [0.0]

    def apply_focus():
        """양 카메라 동기 수동 포커스(같은 장면이라 동기로 충분)."""
        any_ok = False
        for c in cams.values():
            if c["ok"]:
                ok, v, rb = set_manual_focus(c["cap"], state["focus"])
                any_ok = any_ok or ok
                state["focus"] = v
        state["af"] = False
        print(f"[포커스] set={state['focus']} ok={any_ok}"
              + ("" if any_ok else "  <- 드라이버가 무시(MSMF/Logi Tune 확인)"))

    if args.focus >= 0 and any(c["ok"] for c in cams.values()):
        apply_focus()

    def on_mouse(event, x, y, flags, _param):
        if event == cv2.EVENT_LBUTTONDOWN:
            side, xl = classify_click(x, CAM_W)
            if side in ("L", "R"):
                if not cams[side]["ok"]:
                    return
                cb = view_bearing_deg(float(xl), float(CAM_W), CAMERA_HFOV_DEG, state["cam_zoom"])
                state["mode"] = "cam"
                state["cam_side"] = side
                state["cb"] = {"L": cb, "R": cb}   # 양 카메라 같은 색을 클릭 베어링 근처서 탐색 시작
                state["miss"] = {"L": 0, "R": 0}; state["last_seen_range"] = None
                # 색 잠금: 클릭 지점 주변 패치
                vw = state["views"][side]
                if vw is not None:
                    Hv, Wv = vw.shape[:2]
                    vx_ = int(xl * Wv / CAM_W)
                    vy_ = int(y * Hv / CAM_H)
                    px1, px2 = max(0, vx_ - 12), min(Wv, vx_ + 12)
                    py1, py2 = max(0, vy_ - 12), min(Hv, vy_ + 12)
                    cname = None
                    if px2 > px1 and py2 > py1:
                        try:
                            cname, _ = dominant_color(vw[py1:py2, px1:px2])
                        except Exception:
                            cname = None
                    state["color"] = cname if cname in CHROMATIC else None
                    print(f"[색 잠금] {state['color']}" if state["color"]
                          else f"[색 잠금 안 함] 무채색({cname}) -> 자동(지배색)")
                range_gate.reset()
                print(f"[클릭] {side}캠 x={xl} -> cam_bearing={cb:+.1f}deg")
            else:  # radar: 로봇 방위만(시차/색 없음, 표시·수동용)
                dx, dy = xl - cx0, cy0 - y
                if dx * dx + dy * dy < 16:
                    return
                state["mode"] = "radar"
                state["cam_side"] = None
                state["color"] = None
                state["robot_bearing"] = math.degrees(math.atan2(dx, dy))
                range_gate.reset()
                print(f"[클릭] 레이더 -> robot_bearing={state['robot_bearing']:+.1f}deg (색없음)")
        elif event == cv2.EVENT_RBUTTONDOWN:
            state.update(mode=None, cam_side=None, color=None, armed=False)
            state["cb"] = {"L": 0.0, "R": 0.0}; state["miss"] = {"L": 0, "R": 0}
            state["last_seen_range"] = None
            range_gate.reset()
            send("STOP")
            print("[해제] 추적/색 잠금 해제")

    cv2.setMouseCallback(win, on_mouse)
    print("좌클릭(카메라/레이더)=추적, 우클릭=해제, g=ARM, space=STOP, e=ESTOP, ESC=종료")

    def draw_radar(dd, rmax, scale, history):
        img = np.zeros((RAD, RAD, 3), np.uint8)
        ring = 500 if rmax > 2000 else (250 if rmax > 1000 else 100)
        for r_mm in range(ring, int(rmax) + 1, ring):
            cv2.circle(img, (cx0, cy0), int(r_mm * scale), (40, 40, 40), 1)
        for idx, old in enumerate(history):
            shade = 60 + 30 * idx
            for a, d in old.items():
                if d <= 0 or d > rmax:
                    continue
                bo = math.radians(lidar_bearing(a))
                pxo = int(cx0 + d * scale * math.sin(bo)); pyo = int(cy0 - d * scale * math.cos(bo))
                if 0 <= pxo < RAD and 0 <= pyo < RAD:
                    cv2.circle(img, (pxo, pyo), max(1, int(sui)), (shade, shade, shade), -1)
        cv2.line(img, (cx0, cy0), (cx0, 8), (0, 90, 0), 1)
        for s in (-1, 1):
            ang = math.radians(s * CAMERA_HFOV_DEG / 2.0)
            ex = int(cx0 + math.sin(ang) * (RAD / 2 - 12)); ey = int(cy0 - math.cos(ang) * (RAD / 2 - 12))
            cv2.line(img, (cx0, cy0), (ex, ey), (60, 60, 0), 1)
        for a, d in dd.items():
            if d <= 0 or d > rmax:
                continue
            b = math.radians(lidar_bearing(a))
            px = int(cx0 + d * scale * math.sin(b)); py = int(cy0 - d * scale * math.cos(b))
            if 0 <= px < RAD and 0 <= py < RAD:
                cv2.circle(img, (px, py), max(2, int(2 * sui)), (180, 180, 180), -1)
        return img

    fps_t = [time.time()]
    fps_n0 = [0]
    cam_fps = [0.0]

    try:
        while True:
            dd = lidar.getDistanceDict(freshest=True) if lidar is not None else {}
            fresh = lidar.is_fresh(0.5) if lidar is not None else False
            if dd and time.time() - hist_t[0] >= 0.15:
                hist.append(dd); hist_t[0] = time.time()

            zcam = state["cam_zoom"]
            eff_half = effective_half_fov_deg(CAMERA_HFOV_DEG, zcam)
            views = {}
            ntot = 0
            for side, c in cams.items():
                fr = c["grabber"].latest() if c["grabber"] is not None else None
                views[side] = crop_center_zoom(fr, zcam, state["anchor_y"]) if fr is not None else None
                if c["grabber"] is not None:
                    ntot += c["grabber"].n
            state["views"] = views
            if time.time() - fps_t[0] >= 1.0:
                cam_fps[0] = (ntot - fps_n0[0]) / (time.time() - fps_t[0])
                fps_t[0] = time.time(); fps_n0[0] = ntot

            # ----- 자율: 클릭 없이 표적 자동선택(가장 큰 유채색 블롭) -----
            if state["auto"] and (state["mode"] != "cam" or not state["color"]):
                best = None   # (area, side, color, cb)
                for s in ("L", "R"):
                    v = views.get(s)
                    if not cams[s]["ok"] or v is None:
                        continue
                    small = cv2.resize(v, (160, 90))
                    for col in CHROMATIC:
                        m = color_mask(small, col)
                        if float((m > 0).mean()) < AUTO_MIN_AREA:
                            continue
                        cnts, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                        if not cnts:
                            continue
                        cc = max(cnts, key=cv2.contourArea)
                        bx, _by, bw, _bh = cv2.boundingRect(cc)
                        ccx = (bx + bw / 2.0) * v.shape[1] / 160.0     # 원본 view x
                        cbb = view_bearing_deg(ccx, float(v.shape[1]), CAMERA_HFOV_DEG, zcam)
                        if abs(cbb) > eff_half:
                            continue
                        area = cv2.contourArea(cc)
                        if best is None or area > best[0]:
                            best = (area, s, col, cbb)
                if best is not None:
                    _, bs, bcol, bcb = best
                    state["mode"] = "cam"; state["cam_side"] = bs; state["color"] = bcol
                    state["cb"] = {"L": bcb, "R": bcb}; state["miss"] = {"L": 0, "R": 0}
                    state["lost_t"] = 0.0; state["last_seen_range"] = None
                    range_gate.reset()
                    print(f"[자율선택] {bcol} (cam {bs}, bearing {bcb:+.1f}deg)")

            # ----- 추적 분석 -----
            mode = state["mode"]
            robot_bearing = None
            obj_range = None
            gate_state = "IDLE"
            track_raw = None     # 클러스터/표면박스용 raw 라이다각
            cam_range = None
            boxes = {"L": None, "R": None}     # 각 카메라 검출 박스(view 좌표)
            present = {"L": False, "R": False}  # 각 카메라가 잠근 색 물체를 봄(=포커싱)

            if mode == "cam":
                rb_side = {"L": None, "R": None}; rng_side = {"L": None, "R": None}
                craw_side = {"L": None, "R": None}; camrng_side = {"L": None, "R": None}
                area_side = {"L": 0, "R": 0}
                # 색 잠금이 있으면 '양 카메라' 모두 검출(같은 색=교차매칭). 무채색(자동)이면
                # 교차매칭 불가 -> 클릭한 카메라만.
                sides_try = ("L", "R") if state["color"] else \
                    ((state["cam_side"],) if state["cam_side"] in ("L", "R") else ())
                for s in sides_try:
                    v = views.get(s)
                    if not cams[s]["ok"] or v is None:
                        continue
                    H, W = v.shape[:2]
                    cb = state["cb"][s]
                    box_s = None
                    # 1) cb 가 화면 안이면 그 주변 ROI 검출
                    if abs(cb) <= eff_half:
                        cxpx = view_x_from_bearing(cb, W, CAMERA_HFOV_DEG, zcam)
                        halfpx = abs(view_x_from_bearing(args.roi_half_deg, W, CAMERA_HFOV_DEG, zcam)
                                     - view_x_from_bearing(0.0, W, CAMERA_HFOV_DEG, zcam))
                        x1 = int(max(0, cxpx - halfpx)); x2 = int(min(W, cxpx + halfpx))
                        y1, y2 = int(H * args.roi_y1), int(H * args.roi_y2)
                        if x2 - x1 >= 4:
                            roi = v[y1:y2, x1:x2]
                            col = state["color"]
                            if not col:
                                try:
                                    col, _b = dominant_color(roi)
                                except Exception:
                                    col = None
                            if col in CHROMATIC:
                                m = color_mask(roi, col)
                                if float((m > 0).mean()) >= args.min_area:
                                    cnts, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                                    if cnts:
                                        cc = max(cnts, key=cv2.contourArea)
                                        bx, by, bw, bh = cv2.boundingRect(cc)
                                        box_s = (x1 + bx, y1 + by, x1 + bx + bw, y1 + by + bh)
                    # 2) ROI 실패 + 색 잠금 있으면 전체뷰 재탐색(반대편 첫 획득/상실 복구)
                    if box_s is None and state["color"]:
                        fm = color_mask(v, state["color"])
                        cs, _ = cv2.findContours(fm, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                        if cs:
                            c2 = max(cs, key=cv2.contourArea)
                            if cv2.contourArea(c2) >= 0.0008 * H * W:
                                bx, by, bw, bh = cv2.boundingRect(c2)
                                box_s = (bx, by, bx + bw, by + bh)
                    if box_s is not None:
                        boxes[s] = box_s; present[s] = True; state["miss"][s] = 0
                        area_side[s] = (box_s[2] - box_s[0]) * (box_s[3] - box_s[1])
                        bcx = (box_s[0] + box_s[2]) / 2.0
                        nb = view_bearing_deg(bcx, float(W), CAMERA_HFOV_DEG, zcam)
                        if state["color"]:   # 색잠금만 추종(전체뷰 복구 가능). 무채색은 클릭 ROI 고정(구버전)
                            state["cb"][s] = ((1 - FOLLOW_ALPHA) * cb + FOLLOW_ALPHA * nb
                                              if abs(cb) <= eff_half else nb)
                        wpx = box_s[2] - box_s[0]
                        if args.obj_width_cm > 0 and wpx > 2:
                            camrng_side[s] = monocular_range_mm(
                                float(wpx), float(W), CAMERA_HFOV_DEG, zcam, args.obj_width_cm * 10.0)
                        # toe 보정한 시선으로 LiDAR 거리/방위(그 카메라 off_x ±170)
                        ray = CAMERA_LIDAR_SIGN * state["cb"][s] + (state["toe"] if s == "L" else -state["toe"])
                        if fresh:
                            rr, la = distance_along_ray(
                                dd, cams[s]["off_x"], 0.0, ray, FORWARD_ANGLE_DEG, PERP_TOL_MM)
                            rng_side[s] = rr
                            if la is not None:
                                rb_side[s] = normalize_deg(la - FORWARD_ANGLE_DEG)  # ★플립 이중적용 금지
                                craw_side[s] = la
                        if rb_side[s] is None:
                            rb_side[s] = CAMERA_LIDAR_SIGN * state["cb"][s]   # LiDAR 미획득 근사(원거리)
                    else:
                        present[s] = False; state["miss"][s] += 1
                # 제어 대표값: 둘 다 보이면 양 카메라 robot-bearing/거리를 '평균' -> 물체가
                # 로봇 중심(LiDAR축, bearing 0)에 도착하도록 조향(한 카메라 편향 상쇄, 대칭).
                # 한쪽만 보이면 그쪽으로(시야 안으로 끌어옴). area_side 는 더 안 씀.
                focus_sides = [s for s in ("L", "R") if present[s]]
                if focus_sides:
                    # 거리·방위는 '같은 소스'로 묶어야 조향이 안 흔들린다(섞으면 편향).
                    # LiDAR 실측각(craw_side)이 있는 side 들로 거리+방위 함께 평균(로봇중심).
                    lidar_sides = [s for s in focus_sides if craw_side[s] is not None]
                    if lidar_sides:
                        robot_bearing = sum(rb_side[s] for s in lidar_sides) / len(lidar_sides)
                        raw_fused = sum(rng_side[s] for s in lidar_sides) / len(lidar_sides)
                        track_raw = craw_side[lidar_sides[0]]
                    else:   # LiDAR 미획득 -> 카메라근사 방위만(거리는 RangeGate HOLD 에 맡김)
                        robot_bearing = sum(rb_side[s] for s in focus_sides) / len(focus_sides)
                        raw_fused = None
                        track_raw = None
                    crs = [camrng_side[s] for s in focus_sides if camrng_side[s] is not None]
                    cam_range = sum(crs) / len(crs) if crs else None
                    obj_range, gate_state = range_gate.update(raw_fused, time.time())
                else:
                    obj_range, gate_state = range_gate.update(None, time.time())

            elif mode == "radar":
                robot_bearing = state["robot_bearing"]
                track_raw = bearing_to_lidar_angle(robot_bearing)
                if fresh:
                    raw_range = min_distance_in_arc(dd, track_raw, APPROACH_ARC_DEG)
                else:
                    raw_range = None
                obj_range, gate_state = range_gate.update(raw_range, time.time())

            color_present = present["L"] or present["R"]   # 어느 한 카메라라도 색 물체를 봄
            co_detect = (obj_range is not None) and color_present
            if co_detect and gate_state == "TRACK":
                state["last_seen_range"] = obj_range        # 근접 도착 판정용(HOLD 부풀림 제외, 실측만)

            # 자율: 표적 보이면 자동 ARM(안전게이트는 아래 armed 블록), 오래 놓치면:
            #   - 마지막 거리 <=24cm 였으면 '도착'(18cm 사각 진입)으로 보고 정지(재탐색 금지)
            #   - 그 외(원거리 상실)면 표적 해제 후 재탐색
            if state["auto"] and state["mode"] == "cam":
                if color_present:
                    state["lost_t"] = 0.0
                    state["armed"] = True
                else:
                    if state["lost_t"] == 0.0:
                        state["lost_t"] = time.time()
                    elif time.time() - state["lost_t"] > AUTO_RELEASE_S:
                        lr = state["last_seen_range"]
                        if lr is not None and lr <= CLOSE_ARRIVED_MM:
                            send("STOP"); state["armed"] = False; state["auto"] = False
                            print(f"[자율완료] 근접 도달(마지막 {lr/10:.0f}cm, <18cm는 카메라 사각) - 자율 해제")
                        else:
                            state.update(mode=None, color=None, armed=False)
                            state["cb"] = {"L": 0.0, "R": 0.0}; state["miss"] = {"L": 0, "R": 0}
                            state["last_seen_range"] = None
                            send("STOP"); range_gate.reset()
                            print("[자율] 표적 상실(원거리) -> 재탐색")
                        state["lost_t"] = 0.0

            # 표면 클러스터(레이더 유동 테두리용)
            cluster = []
            if fresh and obj_range is not None and track_raw is not None:
                for a2, d2 in dd.items():
                    if d2 > 0 and angular_diff(a2, track_raw) <= APPROACH_ARC_DEG + 2.0 \
                            and abs(d2 - obj_range) <= APPROACH_GATE_MM:
                        cluster.append((a2, d2))

            # 20cm 접근 알림
            if obj_range is not None and obj_range <= NOTICE_MM:
                if state["notice_armed"]:
                    state["notice_t"] = time.time(); state["notice_armed"] = False
                    print("[알림] 20cm 이내로 접근했습니다")
            elif obj_range is None or obj_range > NOTICE_REARM_MM:
                state["notice_armed"] = True

            # 진단: 최근접점
            near_a, near_d = None, None
            for a, d in dd.items():
                if d > 0 and (near_d is None or d < near_d):
                    near_a, near_d = a, d

            # ----- 자율 접근 -----
            if state["armed"]:
                if lidar is not None and not fresh:
                    send("STOP")
                    if time.time() - state["wait_t"] > 1.0:
                        state["wait_t"] = time.time(); print("[자율대기] LiDAR STALE")
                elif co_detect:
                    blocker = blocking_distance(dd, obj_range) if fresh else None
                    if blocker is not None and blocker < OBSTACLE_STOP_MM:
                        send("STOP"); state["armed"] = False; state["auto"] = False
                        print(f"[자율정지] 전방 차단물 {blocker/10:.0f}cm (표적 {obj_range/10:.0f}cm 앞) - 자율 해제, 안전확인 후 u")
                    else:
                        try:
                            vx, vy, w, st = approach_command(
                                obj_range, robot_bearing, args.target_mm,
                                deadband_mm=APPROACH_DEADBAND_MM, min_safe_mm=APPROACH_MIN_SAFE_MM,
                                face_tol_deg=APPROACH_FACE_TOL_DEG, kx=APPROACH_KX, kw=APPROACH_KW,
                                vx_max=APPROACH_VX_MAX, w_max=APPROACH_W_MAX)
                            send(f"V {vx} {vy} {w}")
                            if st in ("ARRIVED", "TOO_CLOSE", "LOST"):
                                send("STOP"); state["armed"] = False; state["auto"] = False
                                print(f"[자율완료/정지] {st} (range={obj_range}) - 자율 해제(다음 물체는 우클릭 후 u)")
                        except Exception as exc:
                            send("STOP"); state["armed"] = False; state["auto"] = False
                            print(f"[오류] approach -> STOP: {exc}")
                elif color_present and cam_range is not None and robot_bearing is not None:
                    obst = min_distance_in_arc(dd, FORWARD_ANGLE_DEG, 20.0) if fresh else None
                    if obst is not None and obst < OBSTACLE_STOP_MM:
                        send("STOP"); state["armed"] = False; state["auto"] = False
                        print(f"[자율정지] OBSTACLE {obst/10:.0f}cm - 자율 해제, 안전확인 후 u")
                    else:
                        try:
                            vx, vy, w, st = cam_approach_command(
                                cam_range, robot_bearing, CAM_APPROACH_MIN_MM,
                                face_tol_deg=APPROACH_FACE_TOL_DEG, kw=APPROACH_KW,
                                vx_far=CAM_APPROACH_VX, w_max=APPROACH_W_MAX)
                            send(f"V {vx} {vy} {w}")
                            if st in ("CAM_LIMIT", "LOST"):
                                send("STOP"); state["armed"] = False; state["auto"] = False
                                print(f"[자율정지] {st} - 60cm 도달 LiDAR 미획득(cam~{cam_range/10:.0f}cm) - 자율 해제")
                        except Exception as exc:
                            send("STOP"); state["armed"] = False; state["auto"] = False
                            print(f"[오류] cam approach -> STOP: {exc}")
                else:
                    send("STOP")
                    if time.time() - state["wait_t"] > 1.0:
                        state["wait_t"] = time.time()
                        print(f"[자율대기] 색={'O' if color_present else 'X'}"
                              f" 라이다거리={'O' if obj_range is not None else 'X'}"
                              f" cam거리={'O' if cam_range is not None else 'X'}"
                              f" mode={state['mode']} lock={state['color'] or 'auto/none'}")

            # 수동 주행(hold-to-drive)
            if state["manual"] is not None and not state["armed"]:
                hold = FINE_HOLD_S if state["fine"] else MANUAL_HOLD_S
                if time.time() - state["manual_t"] <= hold:
                    mvx, mvy, mw = state["manual"]; send(f"V {mvx} {mvy} {mw}")
                else:
                    send("STOP"); state["manual"] = None

            # ----- 그리기 (3분할: camL | camR | radar) -----
            panels = []
            for pside in ("L", "R"):
                pv = views.get(pside)
                panel = np.zeros((CAM_H, CAM_W, 3), np.uint8)
                seen = present[pside]            # 그 카메라가 물체를 보고 포커싱 중인지
                if pv is not None:
                    disp = pv.copy()
                    b_s = boxes[pside]
                    if b_s is not None:          # 그 카메라가 본 물체에 박스(둘 다 보면 둘 다)
                        bcol = (0, 255, 0) if co_detect else (0, 180, 255)
                        cv2.rectangle(disp, b_s[:2], b_s[2:], bcol, 3)
                        lbl = state["color"] or "obj"
                        if obj_range is not None:
                            lbl += f" {obj_range/10:.0f}cm"
                        elif cam_range is not None:
                            lbl += f" cam~{cam_range/10:.0f}cm"
                        cv2.putText(disp, lbl, (b_s[0], max(18, b_s[1] - 8)),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, bcol, 2)
                    panel = cv2.resize(disp, (CAM_W, CAM_H))
                # 포커싱 중인 카메라 조준선
                if seen and abs(state["cb"][pside]) <= eff_half:
                    axp = int(view_x_from_bearing(state["cb"][pside], CAM_W, CAMERA_HFOV_DEG, zcam))
                    cv2.line(panel, (axp, 0), (axp, CAM_H), (0, 200, 255), 1)
                tag = ("하양/L" if pside == "L" else "검정/R") + (" *FOCUS" if seen else "")
                cv2.putText(panel, tag, (6, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                            (0, 255, 255) if seen else (150, 150, 150), 1)
                if not cams[pside]["ok"]:
                    cv2.putText(panel, "cam off", (CAM_W // 3, CAM_H // 2),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (120, 120, 120), 2)
                # 20cm 배너(포커싱 중인 카메라마다)
                if seen and time.time() - state["notice_t"] <= NOTICE_S:
                    cv2.rectangle(panel, (int(CAM_W * 0.12), int(CAM_H * 0.06)),
                                  (int(CAM_W * 0.88), int(CAM_H * 0.22)), (25, 25, 25), -1)
                    cv2.rectangle(panel, (int(CAM_W * 0.12), int(CAM_H * 0.06)),
                                  (int(CAM_W * 0.88), int(CAM_H * 0.22)), (0, 255, 0), 2)
                    cv2.putText(panel, "WITHIN 20 cm", (int(CAM_W * 0.18), int(CAM_H * 0.175)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.9 * sui, (0, 255, 0), max(2, int(2 * sui)))
                panels.append(panel)

            rmax = state["rmax"]; scale = (RAD / 2 - 12) / rmax
            radar = draw_radar(dd, rmax, scale, hist)
            if robot_bearing is not None:
                b = math.radians(robot_bearing)
                bcol2 = (0, 255, 0) if co_detect else (0, 200, 255)
                drawn = False
                if len(cluster) >= 2 and obj_range is not None and obj_range <= rmax:
                    pts = []
                    for a2, d2 in cluster:
                        if d2 <= rmax:
                            bb = math.radians(lidar_bearing(a2))
                            pts.append([int(cx0 + d2 * scale * math.sin(bb)),
                                        int(cy0 - d2 * scale * math.cos(bb))])
                    if len(pts) >= 2:
                        rc, rs, ra = cv2.minAreaRect(np.array(pts, np.int32))
                        bpts = cv2.boxPoints((rc, (rs[0] + 8 * sui, rs[1] + 8 * sui), ra))
                        cv2.polylines(radar, [bpts.astype(np.int32)], True, bcol2, 2); drawn = True
                if not drawn and obj_range is not None and obj_range <= rmax:
                    pxr = int(cx0 + obj_range * scale * math.sin(b)); pyr = int(cy0 - obj_range * scale * math.cos(b))
                    half = max(int(10 * sui), int(obj_range * scale * math.tan(math.radians(APPROACH_ARC_DEG))) + int(6 * sui))
                    cv2.rectangle(radar, (pxr - half, pyr - half), (pxr + half, pyr + half), bcol2, 2)
                elif not drawn:
                    e1x = int(cx0 + (RAD / 2 - 30) * math.sin(b)); e1y = int(cy0 - (RAD / 2 - 30) * math.cos(b))
                    e2x = int(cx0 + (RAD / 2 - 10) * math.sin(b)); e2y = int(cy0 - (RAD / 2 - 10) * math.cos(b))
                    cv2.line(radar, (e1x, e1y), (e2x, e2y), (0, 200, 255), 2)
            if near_a is not None:
                nb = math.radians(lidar_bearing(near_a))
                npx = int(cx0 + near_d * scale * math.sin(nb)); npy = int(cy0 - near_d * scale * math.cos(nb))
                if 0 <= npx < RAD and 0 <= npy < RAD:
                    cv2.circle(radar, (npx, npy), max(4, int(4 * sui)), (255, 0, 255), -1)

            # 카메라 2개 + 레이더를 '각각 따로' 나란히(3분할). 합치기/겹침제거 없음.
            canvas = np.hstack(panels + [radar])
            arm_txt = ("AUTO " if state["auto"] else "") + ("ARMED" if state["armed"] else "disarmed")
            cv2.putText(canvas, arm_txt, (8, int(22 * sui)), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5 * sui, (0, 0, 255) if (state["armed"] or state["auto"]) else (160, 160, 160),
                        max(2, int(1.4 * sui)))
            foc = "+".join([s for s in ("L", "R") if present[s]]) or (state["mode"] or "none")
            stat = (f"L={'OK' if cams['L']['ok'] else 'X'} R={'OK' if cams['R']['ok'] else 'X'}"
                    f" lidar={'OK' if fresh else 'STALE'} focus={foc}")
            if obj_range is not None:
                stat += f" range={obj_range/10:.0f}cm" + ("(HOLD)" if gate_state == "HOLD" else "")
            if robot_bearing is not None:
                stat += f" rb={robot_bearing:+.1f}"
            stat += (f" codet={'Y' if co_detect else 'N'} tgt={args.target_mm/10:.0f}cm"
                     f" view={rmax/1000:.1f}m zoom={zcam:.2f}x {'AF' if state['af'] else 'f'+str(state['focus'])}"
                     f" col={state['color'] or 'auto'} spd={state['mspeed']}{' FINE' if state['fine'] else ''}"
                     f" toe={state['toe']:+.1f} fps={cam_fps[0]:.0f}")
            cv2.putText(canvas, stat, (8, CAM_H - int(8 * sui)), cv2.FONT_HERSHEY_SIMPLEX,
                        0.34 * sui, (0, 255, 255), 1)
            if near_a is not None:
                diag = f"near raw={near_a} bearing={lidar_bearing(near_a):+.0f}deg {near_d/10:.0f}cm (dead-front -> FORWARD_ANGLE_DEG=raw)"
                cv2.putText(canvas, diag, (8, CAM_H - int(24 * sui)), cv2.FONT_HERSHEY_SIMPLEX,
                            0.3 * sui, (255, 0, 255), 1)

            cv2.imshow(win, canvas)
            kx = cv2.waitKeyEx(30)
            k = kx & 0xFF
            if kx in ARROW_KEYS:
                ux, uy, uw = ARROW_KEYS[kx]; sp = state["mspeed"]
                state["armed"] = False
                state["manual"] = (ux * sp, uy * sp, uw * sp); state["manual_t"] = time.time()
            elif k == 27:
                break
            elif k == ord('u'):
                state["auto"] = not state["auto"]
                if not state["auto"]:
                    state["armed"] = False; send("STOP")
                print("[자율주행] " + ("ON - 바퀴 들고 시험! 클릭없이 가장 큰 색물체로 로봇중심 접근(18cm)"
                                      if state["auto"] else "OFF"))
            elif k == ord('g'):
                state["auto"] = False                 # 수동 ARM 토글은 자율 끔
                state["armed"] = not state["armed"]; state["manual"] = None
                if state["armed"]:
                    issues = []
                    if ser is None:
                        issues.append("모터 미연결")
                    if state["mode"] is None:
                        issues.append("추적 대상 없음 - 카메라의 물체를 클릭")
                    elif state["mode"] == "cam" and not state["color"]:
                        issues.append("색 잠금 없음")
                    elif state["mode"] == "radar":
                        issues.append("레이더 클릭은 색 없음 -> 자율 불가(카메라 클릭 필요)")
                    print("[ARM] 자율접근 켜짐" + (" | 주의: " + " / ".join(issues) if issues else ""))
                else:
                    send("STOP"); print("[DISARM]")
            elif k == ord(' '):
                state["auto"] = False; state["armed"] = False; state["manual"] = None; send("STOP")
            elif k == ord('e'):
                state["auto"] = False; state["armed"] = False; state["manual"] = None; send("ESTOP")
            elif k == ord('['):
                state["rmax"] = max(500.0, state["rmax"] / 1.3)
            elif k == ord(']'):
                state["rmax"] = min(12000.0, state["rmax"] * 1.3)
            elif k == ord('='):
                state["cam_zoom"] = min(5.0, state["cam_zoom"] * 1.25)
            elif k == ord('-'):
                state["cam_zoom"] = max(1.0, state["cam_zoom"] / 1.25)
            elif k == ord('i'):
                state["anchor_y"] = max(0.0, state["anchor_y"] - 0.05)
            elif k == ord('k'):
                state["anchor_y"] = min(1.0, state["anchor_y"] + 0.05)
            elif k == ord('f'):
                if any(c["ok"] for c in cams.values()):
                    if state["af"]:
                        apply_focus()
                    else:
                        for c in cams.values():
                            if c["ok"]:
                                enable_autofocus(c["cap"])
                        state["af"] = True; print("[포커스] AF 복귀")
            elif k == ord(','):
                if any(c["ok"] for c in cams.values()):
                    state["focus"] = max(0, state["focus"] - FOCUS_STEP); apply_focus()
            elif k == ord('.'):
                if any(c["ok"] for c in cams.values()):
                    state["focus"] = min(250, state["focus"] + FOCUS_STEP); apply_focus()
            elif k in (ord('1'), ord('2'), ord('3')):
                state["mspeed"] = {ord('1'): 20, ord('2'): 35, ord('3'): 50}[k]
                print(f"[수동속도] {state['mspeed']}%")
            elif k == ord('x'):
                state["fine"] = not state["fine"]; print(f"[미세모드] {'ON' if state['fine'] else 'OFF'}")
            elif k == ord('n'):
                state["toe"] -= 0.5; print(f"[toe] {state['toe']:+.1f}deg (정면 물체가 좌/우 클릭 모두 rb~0 될 때까지)")
            elif k == ord('m'):
                state["toe"] += 0.5; print(f"[toe] {state['toe']:+.1f}deg")
            elif k in MANUAL_KEYS:
                ux, uy, uw = MANUAL_KEYS[k]; sp = state["mspeed"]
                if state["fine"]:
                    sp = max(8, int(sp * FINE_SPEED_MUL))
                state["armed"] = False
                state["manual"] = (ux * sp, uy * sp, uw * sp); state["manual_t"] = time.time()

    except KeyboardInterrupt:
        pass
    finally:
        send("ESTOP"); send("STOP")
        for c in cams.values():
            if c["grabber"] is not None:
                c["grabber"].release()
            elif c["cap"] is not None:
                c["cap"].release()
        if lidar is not None:
            lidar.close()
        if ser is not None:
            try:
                ser.close()
            except Exception:
                pass
        cv2.destroyAllWindows()
        print("종료: 모터 정지 및 장치 해제")


if __name__ == "__main__":
    main()
