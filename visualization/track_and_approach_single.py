"""track_and_approach.py — LiDAR(X4, 뒤집힘) 클릭 추적 + 카메라 색/박스 + 거리 + 메카넘 접근.

흐름(요청 통합):
  1) 레이더(LiDAR) 화면에서 물체를 '마우스 클릭' -> 그 방위를 추적 대상으로 고정.
  2) 카메라가 그 방위(고정 카메라이므로 '디지털 영역+포커스')를 바라보며 그 물체에
     네모 테두리 + 색상(HSV) 표시. LiDAR로 그 물체까지의 거리(mm) 표시.
  3) 카메라(색 블롭)와 LiDAR(거리)가 '동시에' 그 물체를 감지하면 -> (ARM 상태일 때)
     메카넘 4모터를 굴려 목표거리(기본 18cm)까지 접근하고 정지.

좌표/장착:
  - 라이다는 '뒤집혀' 달림(config.LIDAR_FLIPPED) + 전방=빨간 점(FORWARD_ANGLE_DEG).
    -> 모든 표시/클릭/카메라 매핑을 fusion.lidar_bearing(정면0,우측+) 한 좌표로 통일.
    레이더 화면도 '로봇 기준'(위=정면, 오른쪽=우측)으로 그려 클릭이 직관적이다.

안전(중요):
  - 기본 DISARM(표시만, 모터 정지). 'g' 로 ARM 해야 자율접근.
  - 도착/놓침/너무가까움(블라인드존)/스테일 -> 자동 STOP + DISARM.
  - 펌웨어 데드맨(1500ms)과 종료시 ESTOP 으로 이중 안전.
  - ⚠ 융합 상수(전방각/부호/HFOV/거리스케일)와 LIDAR_FLIPPED 미러방향은 캘리브로 확정.
    캘리브 전엔 방향/거리가 틀릴 수 있으니 반드시 '바퀴 들고' 먼저 시험할 것.

실행 예:
  py -3.13 visualization/track_and_approach.py --port COM8 --cam-index 1 --motor-port COM10
키:
  마우스 좌클릭(카메라)=그 물체 추적 + '그 지점의 색'을 잠금(지배색 무시 -> 파란 의자
              앞의 빨간 물체도 빨강으로 추적) / 좌클릭(레이더)=방위만 지정(색 잠금 유지)
  우클릭=해제+정지(색 잠금도 해제)
  g=자율접근 ARM/DISARM, space=STOP+DISARM, e=ESTOP+DISARM, ESC=종료
  수동 주행(hold-to-drive: 누르는 동안 주행, 떼면 ~0.6s 내 정지. ARM 자동해제):
    방향키 ↑/↓=전/후, ←/→=좌/우회전 (Windows waitKeyEx 코드)
    w/s=전/후, a/d=좌/우회전, z/c=좌/우 strafe, q/r=전방 대각, v/b=후방 대각 (메카넘 8방향)
    1/2/3=속도 20/35/50%, x=미세모드 토글(짧은 펄스 0.25s + 저속 -> 탭으로 cm 단위 조정)
    (펌웨어 데드맨 1.5s 가 최후 안전망)
  추적(색 잠금 시): 로봇/물체가 움직여도 박스 중심으로 추적각을 매 프레임 따라가고(추종),
    놓치면 화면 전체에서 잠금 색을 재탐색해 재획득한다. 렉 제거: 배경 스레드가 항상
    최신 프레임만 유지(FrameGrabber) + 시작 시 실제 포맷(fourcc/fps) 콘솔 출력.
  접근 알림: 추적 물체가 20cm 이내로 들어오면 카메라 화면에 5초 배너(재무장 25cm).
    ※ cv2 폰트는 한글 미지원 -> 배너는 'WITHIN 20 cm', 콘솔엔 한글 출력.
  화면: --ui-scale(기본 1.5)로 카메라+레이더 확대. 레이더 추적 표시 = 감지된 표면 점들을
    감싸는 '유동 테두리'(minAreaRect, 매 프레임 표면 모양에 맞춰 변형. 동시감지=초록).
    점 1개면 고정 사각, 거리 미확보면 가장자리 틱. 이전 스캔 2~3개를 어둡게 잔상 표시.
  단안 거리: LiDAR 미검출인데 색 박스가 있으면 'cam~NNcm'(크기 기반, --obj-width-cm=9).
  자율접근(ARM, 'g') 2단계:
    [원거리] LiDAR 가 표적을 아직 못 잡으면 단안 거리로 '서행' 접근(정렬->전진 22%).
       안전: 전방 ±20° 라이다 장애물 <35cm 정지 / 단안 60cm 까지 와도 LiDAR 미획득이면
       정지+해제(스캔평면 문제 의심 — 맹목 접근 금지).
    [정밀] LiDAR 가 잡히는 순간 자동 인계 -> 목표거리(기본 18cm) 정지. 정밀 정지는 항상 LiDAR.
  거리 표시(원거리 안정화): 직전값 ±300mm 게이트. 배경 점프/빔 미스는 0.8s 유지(HOLD
    표시) 후 상실 — 2m+ 소형 물체에서 빔이 ~5개 이하라 미스 시 배경값이 튀는 것을 억제.
  레이더 범위: [ = 축소(줌인, 가까운 물체 크게) / ] = 확대(줌아웃, 멀리까지)
  카메라 줌:  = 줌인(×1.25, 최대 5x) / - 줌아웃 — 먼/작은 물체를 키워서 클릭·색검출.
              (중앙 크롭 디지털 줌. 클릭·박스·조준선은 줌 배율을 반영해 같은 베어링 유지)
  세로 앵커:  i = 위로 / k = 아래로 (줌인 시 크롭 창의 세로 위치. 가까운 물체=화면 아래
              -> 기본 0.65 로 낮게 시작. 가로 팬은 베어링 매핑 때문에 없음)
  포커스:    f = AF 켜기/끄기 토글, , = 멀리(-5) / . = 가까이(+5) (0~250, 0=원거리).
              StreamCam 은 근거리 AF 가 헤매므로 가까운 물체는 수동 고정 권장.
              ⚠ 드라이버가 무시할 수 있음(콘솔의 ok/읽기백 확인. Logi Tune/G HUB 끌 것)
"""
import argparse
import math
import sys
import time
import pathlib
from collections import deque

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))

from common.config import (                       # noqa: E402
    LIDAR_X4_PORT, LIDAR_X4_BAUDRATE, CAMERA_INDEX, MOTOR_PORT, MOTOR_BAUDRATE,
    CAMERA_HFOV_DEG,
    APPROACH_TARGET_MM, APPROACH_DEADBAND_MM, APPROACH_MIN_SAFE_MM,
    APPROACH_FACE_TOL_DEG, APPROACH_ARC_DEG, APPROACH_KX, APPROACH_KW,
    APPROACH_VX_MAX, APPROACH_W_MAX, APPROACH_COLOR_MIN_AREA,
    APPROACH_GATE_MM, APPROACH_HOLD_S,
    CAM_APPROACH_MIN_MM, CAM_APPROACH_VX, OBSTACLE_STOP_MM, FORWARD_ANGLE_DEG,
)
from common.fusion import (                        # noqa: E402
    lidar_bearing, bearing_to_lidar_angle, view_x_from_bearing, view_bearing_deg,
    effective_half_fov_deg, min_distance_in_arc, monocular_range_mm, blocking_distance,
)
from common.lidar_metrics import angular_diff          # noqa: E402
from common.camera import (                           # noqa: E402
    crop_center_zoom, set_manual_focus, enable_autofocus, FOCUS_STEP,
    FrameGrabber, camera_info,
)
from common.color import dominant_color, color_mask  # noqa: E402
from common.approach import approach_command, cam_approach_command, RangeGate  # noqa: E402

CHROMATIC = {"red", "orange", "yellow", "green", "cyan", "blue", "purple", "pink"}


def yolo_best_box(model, frame, target_class="any", conf=0.35):
    """YOLO 로 frame 에서 표적 1개 선택 -> (box_xyxy_int, label, conf) 또는 (None, None, 0.0).
    target_class='any' 면 최고신뢰도, 아니면 그 클래스 중 최고신뢰도. (GPU 자동 사용)"""
    res = model(frame, conf=conf, verbose=False)
    if not res:
        return None, None, 0.0
    boxes = res[0].boxes
    if boxes is None or len(boxes) == 0:
        return None, None, 0.0
    names = model.names
    best = None
    for i in range(len(boxes)):
        cf = float(boxes.conf[i])
        lbl = names.get(int(boxes.cls[i]), str(int(boxes.cls[i])))
        if target_class != "any" and lbl != target_class:
            continue
        if best is None or cf > best[2]:
            x1, y1, x2, y2 = (int(v) for v in boxes.xyxy[i].tolist())
            best = ((x1, y1, x2, y2), lbl, cf)
    return best if best else (None, None, 0.0)
# 수동 주행 키 -> (vx, vy, w) 단위방향(속도는 mspeed 배율). hold-to-drive:
# 키를 누르는 동안 OS 키반복이 시각을 갱신해 계속 주행, 떼면 MANUAL_HOLD_S 후 STOP.
MANUAL_KEYS = {
    ord('w'): (1, 0, 0), ord('s'): (-1, 0, 0),     # 전/후
    ord('a'): (0, 0, -1), ord('d'): (0, 0, 1),     # 좌/우 회전
    ord('z'): (0, -1, 0), ord('c'): (0, 1, 0),     # 좌/우 strafe (메카넘 전용)
    ord('q'): (1, -1, 0), ord('r'): (1, 1, 0),     # 전방 대각 좌/우 (메카넘 전용)
    ord('v'): (-1, -1, 0), ord('b'): (-1, 1, 0),   # 후방 대각 좌/우 (메카넘 전용)
}
MANUAL_HOLD_S = 0.6   # 마지막 키 입력 후 유지시간(OS 키반복 시작지연 ~0.5s 브리지)
FINE_HOLD_S = 0.25    # 미세 모드('x'): 짧은 펄스 + 저속 -> 탭 한 번에 조금만 이동
FINE_SPEED_MUL = 0.4  # 미세 모드 속도 배율(예: 30% -> 12%)
# 방향키(←→↑↓) -> (vx, vy, w). cv2.waitKeyEx 의 Windows 확장코드(&0xFF=0 이라 ASCII 와 무충돌).
ARROW_KEYS = {
    2490368: (1, 0, 0),    # ↑ 전진
    2621440: (-1, 0, 0),   # ↓ 후진
    2424832: (0, 0, -1),   # ← 좌회전
    2555904: (0, 0, 1),    # → 우회전
}
# 접근 알림: NOTICE_MM 이내 진입 시 1회, NOTICE_S 초 표시. REARM_MM 밖으로 나가면 재무장.
NOTICE_MM = 200.0
NOTICE_S = 5.0
NOTICE_REARM_MM = 250.0
# 카메라-추종(색 잠금 시): 매 프레임 박스 중심 베어링으로 추적각을 평활 갱신 ->
# 로봇이 움직여도 카메라가 물체를 따라간다. 상실이 이어지면 화면 전체에서 재탐색.
FOLLOW_ALPHA = 0.5      # 베어링 갱신 평활(0=고정, 1=즉시)
REACQ_MISS_N = 5        # 이 프레임 수만큼 연속 상실 시 전체화면 재획득 시도


def main():
    ap = argparse.ArgumentParser(description="LiDAR(뒤집힘) 클릭 추적 + 카메라 색/거리 + 메카넘 접근")
    ap.add_argument("--port", default=LIDAR_X4_PORT, help="X4 LiDAR COM 포트")
    ap.add_argument("--baud", type=int, default=LIDAR_X4_BAUDRATE)
    ap.add_argument("--lidar", default="x4", choices=["x2", "x4"])
    ap.add_argument("--cam-index", type=int, default=CAMERA_INDEX)
    ap.add_argument("--motor-port", default=MOTOR_PORT, help="모터(아두이노) COM. 'none'이면 구동 비활성")
    ap.add_argument("--target-mm", type=float, default=APPROACH_TARGET_MM)
    ap.add_argument("--rmax", type=float, default=3000.0, help="레이더 표시 최대거리(mm)")
    ap.add_argument("--roi-half-deg", type=float, default=6.0, help="추적각 ± ROI 가로 반각")
    ap.add_argument("--roi-y1", type=float, default=0.10, help="ROI 세로밴드 시작(뷰 높이 비율)")
    ap.add_argument("--roi-y2", type=float, default=0.95, help="ROI 세로밴드 끝(뷰 높이 비율)")
    ap.add_argument("--anchor-y", type=float, default=0.65,
                    help="줌 크롭 세로 앵커(0=상단,1=하단). 가까운 물체=화면 아래라 낮게")
    ap.add_argument("--focus", type=int, default=-1,
                    help="시작 수동포커스(0~250, 0=원거리). -1=오토포커스 유지")
    ap.add_argument("--min-area", type=float, default=APPROACH_COLOR_MIN_AREA,
                    help="ROI 내 색 블롭 최소 면적비(카메라 감지 게이트)")
    ap.add_argument("--ui-scale", type=float, default=1.5,
                    help="화면 배율(1.0=카메라 640x360+레이더 360, 1.5=960x540+540)")
    ap.add_argument("--obj-width-cm", type=float, default=9.0,
                    help="추적 물체 실폭(cm). LiDAR 미검출 시 카메라 단안 거리 표시(0=끔)")
    ap.add_argument("--forward-angle", type=float, default=None,
                    help="전방 LiDAR raw각(deg). 미지정시 config.FORWARD_ANGLE_DEG. "
                         "실행 중 'p'(정면 물체 가리키고)로 즉시 캡처 가능")
    ap.add_argument("--detect", choices=["color", "yolo"], default="color",
                    help="표적 검출: color=색 기반(기본), yolo=학습모델(red_box/blue_cylinder)")
    ap.add_argument("--weights",
                    default=str(pathlib.Path(__file__).resolve().parent.parent
                                / "YOLO" / "models" / "weights" / "best.pt"),
                    help="YOLO 가중치 경로(--detect yolo 일 때)")
    ap.add_argument("--yolo-conf", type=float, default=0.35, help="YOLO 신뢰도 임계값")
    ap.add_argument("--yolo-class", default="any",
                    help="추적 클래스(any=최고신뢰도 | red_box | blue_cylinder)")
    args = ap.parse_args()

    import cv2
    import numpy as np
    from drivers import make_lidar

    # ---- 장치 열기 (각각 선택적) ----
    lidar = make_lidar(args.lidar, args.port, args.baud)
    if lidar.open():
        print(f"[OK] LiDAR({args.lidar}) {args.port}")
    else:
        print(f"[경고] LiDAR 열기 실패 {args.port} -> 거리/추적 비활성")
        lidar = None

    cap = None
    grabber = None
    try:
        from common.camera import open_camera
        cap = open_camera(args.cam_index)
        if not cap.isOpened():
            print(f"[경고] 카메라 index {args.cam_index} 열기 실패 -> 색 감지 비활성")
            cap = None
        else:
            # 실제 적용 포맷 진단: MJPG 가 아니거나 fps 가 낮으면 렉/저화질의 원인.
            print(f"[OK] 카메라 index {args.cam_index} ({camera_info(cap)})")
            grabber = FrameGrabber(cap)   # 배경 스레드: 최신 프레임만 유지(렉 제거)
    except Exception as exc:
        print(f"[경고] 카메라 오류: {exc}")
        cap = None

    ser = None
    if args.motor_port.lower() != "none":
        try:
            import serial
            import time as _t
            ser = serial.Serial(args.motor_port, MOTOR_BAUDRATE, timeout=0.2)
            _t.sleep(2.0)  # 보드 리셋 대기
            print(f"[OK] 모터 {args.motor_port}")
            try:
                ser.reset_input_buffer()
            except Exception:
                pass
        except Exception as exc:
            print(f"[경고] 모터 포트 열기 실패({args.motor_port}): {exc} -> 구동 비활성")
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

    # ---- YOLO 검출기(--detect yolo): 학습모델 1회 로드(GPU 자동). 실패 시 색 검출로 폴백 ----
    detector_model = None
    if args.detect == "yolo":
        try:
            from ultralytics import YOLO
            detector_model = YOLO(args.weights)
            print(f"[OK] YOLO {args.weights}  classes={detector_model.names}  "
                  f"class-filter={args.yolo_class} conf>={args.yolo_conf}")
        except Exception as exc:
            print(f"[경고] YOLO 로드 실패({args.weights}): {exc} -> 색 검출로 폴백")
            detector_model = None

    # ---- 화면 구성 (--ui-scale 로 확대. RAD=CAM_H 라 hstack 높이 일치) ----
    sui = max(1.0, args.ui_scale)
    CAM_W, CAM_H = int(640 * sui), int(360 * sui)
    RAD = CAM_H
    cx0, cy0 = RAD // 2, RAD // 2
    win = "track_and_approach"
    cv2.namedWindow(win)

    # bearing=추적 베어링(정면0,우측+) / rmax=레이더범위(mm,[ ]) / cam_zoom=카메라줌(= -)
    # anchor_y=줌 크롭 세로앵커(i/k) / af=오토포커스 여부(f) / focus=수동 포커스값(, .)
    # color=클릭으로 잠근 추적 색(None=자동/지배색) / last_view=클릭 색 샘플링용 최근 뷰
    state = {"bearing": None, "armed": False, "rmax": float(args.rmax), "cam_zoom": 1.0,
             "anchor_y": max(0.0, min(1.0, args.anchor_y)),
             "af": args.focus < 0, "focus": max(0, min(250, args.focus)),
             "color": None, "last_view": None,
             "manual": None, "manual_t": 0.0, "mspeed": 30,
             "notice_t": -1e9, "notice_armed": True, "miss": 0, "fine": False,
             # 전방 LiDAR raw각(런타임). CLI > config. 'p' 키로 정면물체 기준 즉시 보정.
             "forward": float(args.forward_angle) if args.forward_angle is not None
                        else float(FORWARD_ANGLE_DEG)}

    # 전방각을 런타임 state["forward"] 로 다루기 위해 fusion 의 두 변환을 main 지역에서
    # 재정의(임포트본을 가림). 라이다 뒤집힘은 lidar_dir_sign(=config.LIDAR_FLIPPED)이 그대로 반영.
    from common.fusion import lidar_dir_sign as _dir_sign

    def lidar_bearing(raw_angle_deg, forward_deg=None):
        f = state["forward"] if forward_deg is None else forward_deg
        return _dir_sign() * (((raw_angle_deg - f + 180.0) % 360.0) - 180.0)

    def bearing_to_lidar_angle(bearing_deg):
        return (state["forward"] + _dir_sign() * bearing_deg) % 360.0

    # 원거리 거리 스파이크 억제 게이트(클릭으로 표적이 바뀌면 reset)
    range_gate = RangeGate(APPROACH_GATE_MM, APPROACH_HOLD_S)
    # S1 레이더 잔상: ~1회전(0.15s) 간격으로 최근 3개 스캔을 보관, 옛 것은 어둡게 표시
    hist = deque(maxlen=3)
    hist_t = 0.0

    def apply_focus():
        """현재 state['focus'] 를 카메라에 적용(AF off). 결과를 콘솔로 보고."""
        if cap is None:
            return
        ok, v, rb = set_manual_focus(cap, state["focus"])
        state["af"] = False
        state["focus"] = v
        print(f"[포커스] set={v} ok={ok} readback={rb:.0f}"
              + ("" if ok and abs(rb - v) < FOCUS_STEP else "  <- 드라이버가 무시했을 수 있음(MSMF/Logi Tune 확인)"))

    if cap is not None and args.focus >= 0:
        apply_focus()   # 시작부터 수동 포커스 고정 요청

    def on_mouse(event, x, y, flags, _param):
        if event == cv2.EVENT_LBUTTONDOWN:
            if x >= CAM_W:
                # 레이더(오른쪽) 클릭: 화면각 = 로봇 베어링(위=정면, 오른쪽=우측)
                dx, dy = (x - CAM_W) - cx0, cy0 - y
                if dx * dx + dy * dy < 16:
                    return
                b = math.degrees(math.atan2(dx, dy))
            else:
                # 카메라(왼쪽) 클릭: 그 픽셀의 카메라 베어링. 패널=크롭뷰의 균일 리사이즈라
                # x/CAM_W 비율이 보존되므로 현재 줌 배율로 핀홀 역산하면 정확하다.
                b = view_bearing_deg(float(x), float(CAM_W), CAMERA_HFOV_DEG, state["cam_zoom"])
                # 색 잠금: 클릭 '지점'(y좌표 활용) 주변 패치의 색을 추적 색으로 고정.
                # -> ROI 지배색(예: 뒤의 파란 의자)이 아니라 사용자가 고른 물체를 추적.
                vw = state["last_view"]
                if vw is not None:
                    Hv, Wv = vw.shape[:2]
                    vx_ = int(x * Wv / CAM_W)
                    vy_ = int(y * Hv / CAM_H)
                    px1, px2 = max(0, vx_ - 12), min(Wv, vx_ + 12)
                    py1, py2 = max(0, vy_ - 12), min(Hv, vy_ + 12)
                    cname = None
                    if px2 > px1 and py2 > py1:
                        try:
                            cname, _ = dominant_color(vw[py1:py2, px1:px2])
                        except Exception:
                            cname = None
                    if cname in CHROMATIC:
                        state["color"] = cname
                        print(f"[색 잠금] {cname}")
                    else:
                        print(f"[색 잠금 안 함] 클릭 지점이 무채색({cname}) -> 자동(지배색) 모드")
            state["bearing"] = b
            range_gate.reset()           # 표적 변경 -> 이전 거리 창 폐기
            print(f"[클릭] x={x} y={y} -> bearing={b:+.1f}deg")
        elif event == cv2.EVENT_RBUTTONDOWN:
            state["bearing"] = None
            state["color"] = None
            state["armed"] = False
            range_gate.reset()
            send("STOP")
            print("[해제] 추적/색 잠금 해제")

    cv2.setMouseCallback(win, on_mouse)
    print("좌클릭=추적, 우클릭=해제, g=ARM, space=STOP, e=ESTOP, [ ]=레이더범위, ESC=종료")

    def draw_radar(dd, rmax, scale, history=()):
        img = np.zeros((RAD, RAD, 3), np.uint8)
        ring = 500 if rmax > 2000 else (250 if rmax > 1000 else 100)   # 줌인 시 링 촘촘히
        for r_mm in range(ring, int(rmax) + 1, ring):
            cv2.circle(img, (cx0, cy0), int(r_mm * scale), (40, 40, 40), 1)
        # S1 잔상: 이전 스캔(최대 3개)을 어둡게 깔아 깜빡임 시각 완화(제어 무영향)
        for idx, old in enumerate(history):
            shade = 60 + 30 * idx
            for a, d in old.items():
                if d <= 0 or d > rmax:
                    continue
                bo = math.radians(lidar_bearing(a))
                pxo = int(cx0 + d * scale * math.sin(bo))
                pyo = int(cy0 - d * scale * math.cos(bo))
                if 0 <= pxo < RAD and 0 <= pyo < RAD:
                    cv2.circle(img, (pxo, pyo), max(1, int(sui)), (shade, shade, shade), -1)
        cv2.line(img, (cx0, cy0), (cx0, 8), (0, 90, 0), 1)               # 정면(위)
        for s in (-1, 1):                                                # 카메라 FOV 쐐기
            ang = math.radians(s * CAMERA_HFOV_DEG / 2.0)
            ex = int(cx0 + math.sin(ang) * (RAD / 2 - 12))
            ey = int(cy0 - math.cos(ang) * (RAD / 2 - 12))
            cv2.line(img, (cx0, cy0), (ex, ey), (60, 60, 0), 1)
        if dd:
            for a, d in dd.items():
                if d <= 0 or d > rmax:
                    continue
                b = math.radians(lidar_bearing(a))    # 뒤집힘 반영한 로봇 베어링으로 배치
                px = int(cx0 + d * scale * math.sin(b))
                py = int(cy0 - d * scale * math.cos(b))
                if 0 <= px < RAD and 0 <= py < RAD:
                    cv2.circle(img, (px, py), max(2, int(2 * sui)), (180, 180, 180), -1)
        return img

    fps_t = time.time()
    fps_n0 = 0
    cam_fps = 0.0
    try:
        while True:
            dd = lidar.getDistanceDict(freshest=True) if lidar is not None else {}
            fresh = lidar.is_fresh(0.5) if lidar is not None else False
            if dd and time.time() - hist_t >= 0.15:    # S1: ~1회전 간격으로 잔상 보관
                hist.append(dd)
                hist_t = time.time()

            # 비블로킹: 그래버의 최신 프레임(카메라 fps 에 UI 가 묶이지 않음)
            frame = grabber.latest() if grabber is not None else None
            if grabber is not None and time.time() - fps_t >= 1.0:
                cam_fps = (grabber.n - fps_n0) / (time.time() - fps_t)
                fps_t = time.time()
                fps_n0 = grabber.n
            # 카메라 디지털 줌: 크롭 뷰(view) 기준으로 ROI/색/박스/표시 전부 수행.
            # 세로 앵커(anchor_y)로 크롭 창을 내려 잡으면 '가까운(화면 아래)' 물체가 보존된다.
            zcam = state["cam_zoom"]
            view = crop_center_zoom(frame, zcam, state["anchor_y"]) if frame is not None else None
            state["last_view"] = view          # 클릭 시 그 지점 색 샘플링용
            eff_half = effective_half_fov_deg(CAMERA_HFOV_DEG, zcam)  # 줌 반영 유효 반화각

            # ----- 추적 대상 분석 (로봇 베어링 기준) -----
            bearing = state["bearing"]
            obj_range = None
            color_name = None
            color_present = False
            box = None
            gate_state = "IDLE"
            cluster = []     # S2: 추적 클러스터 (raw각, 거리) — 표면 맞춤 테두리용
            # ----- YOLO 검출(--detect yolo): 전체 뷰에서 표적 -> box/베어링(줌 반영). ROI/색/재획득 불필요 -----
            if detector_model is not None and view is not None:
                yb, ylabel, yconf = yolo_best_box(detector_model, view, args.yolo_class, args.yolo_conf)
                if yb is not None:
                    box = yb
                    color_present = True
                    color_name = f"{ylabel} {yconf:.2f}"
                    ncx = (yb[0] + yb[2]) / 2.0
                    nb = view_bearing_deg(ncx, float(view.shape[1]), CAMERA_HFOV_DEG, zcam)
                    state["bearing"] = nb if state["bearing"] is None \
                        else (1 - FOLLOW_ALPHA) * state["bearing"] + FOLLOW_ALPHA * nb
                    state["miss"] = 0
                else:
                    state["miss"] += 1        # 표적 미검출(HOLD 후 상실은 RangeGate 가 처리)
                bearing = state["bearing"]
            if bearing is not None:
                raw_range = None
                target_raw = bearing_to_lidar_angle(bearing)         # 베어링 -> raw 각도
                if fresh:
                    raw_range = min_distance_in_arc(dd, target_raw, APPROACH_ARC_DEG)
                # 원거리 스파이크 억제: 배경 점프/빔 미스는 직전값을 잠시 유지(HOLD)
                obj_range, gate_state = range_gate.update(raw_range, time.time())
                if fresh and obj_range is not None:
                    for a2, d2 in dd.items():    # 표적 거리 ±게이트 안의 실제 점들
                        if d2 > 0 and angular_diff(a2, target_raw) <= APPROACH_ARC_DEG + 2.0 \
                                and abs(d2 - obj_range) <= APPROACH_GATE_MM:
                            cluster.append((a2, d2))
                if detector_model is None and view is not None and abs(bearing) <= eff_half:
                    H, W = view.shape[:2]
                    # ROI: 중심픽셀 ± 반각의 픽셀폭(줌 반영). 가장자리에서도 폭이 안 무너지게
                    # '중심기준'으로 계산(양끝 각각 클램프 -> 폭0 버그 회피).
                    cxpx = view_x_from_bearing(bearing, W, CAMERA_HFOV_DEG, zcam)
                    halfpx = abs(view_x_from_bearing(args.roi_half_deg, W, CAMERA_HFOV_DEG, zcam)
                                 - view_x_from_bearing(0.0, W, CAMERA_HFOV_DEG, zcam))
                    x1 = int(max(0, cxpx - halfpx))
                    x2 = int(min(W, cxpx + halfpx))
                    y1, y2 = int(H * args.roi_y1), int(H * args.roi_y2)
                    if x2 - x1 >= 4:
                        roi = view[y1:y2, x1:x2]
                        if state["color"]:
                            # 클릭으로 잠근 색만 추적 — ROI 지배색(배경)이 달라도 무시.
                            color_name = state["color"]
                        else:
                            try:
                                color_name, _bgr = dominant_color(roi)
                            except Exception:
                                color_name = None     # 불량 프레임에 루프가 죽지 않도록
                        if color_name in CHROMATIC:
                            mask = color_mask(roi, color_name)
                            if float((mask > 0).mean()) >= args.min_area:
                                color_present = True
                                cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                                if cnts:
                                    c = max(cnts, key=cv2.contourArea)
                                    bx, by, bw, bh = cv2.boundingRect(c)
                                    box = (x1 + bx, y1 + by, x1 + bx + bw, y1 + by + bh)

            # ----- S4 카메라-추종(색 잠금 시): 박스 중심으로 추적각 갱신, 상실 시 재획득 -----
            #        (YOLO 모드는 위에서 이미 추적각을 갱신하므로 이 색-추종은 건너뜀)
            if detector_model is None and bearing is not None and state["color"] and view is not None:
                if box is not None:
                    state["miss"] = 0
                    bcx = (box[0] + box[2]) / 2.0
                    nb = view_bearing_deg(bcx, float(view.shape[1]), CAMERA_HFOV_DEG, zcam)
                    state["bearing"] = (1 - FOLLOW_ALPHA) * bearing + FOLLOW_ALPHA * nb
                else:
                    state["miss"] += 1
                    if state["miss"] >= REACQ_MISS_N:      # ROI 밖으로 사라짐 -> 전체 재탐색
                        try:
                            fm = color_mask(view, state["color"])
                            cnts2, _ = cv2.findContours(fm, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                            if cnts2:
                                c2 = max(cnts2, key=cv2.contourArea)
                                if cv2.contourArea(c2) >= 0.0005 * view.shape[0] * view.shape[1]:
                                    rx2, ry2, rw2, rh2 = cv2.boundingRect(c2)
                                    nb = view_bearing_deg(rx2 + rw2 / 2.0, float(view.shape[1]),
                                                          CAMERA_HFOV_DEG, zcam)
                                    state["bearing"] = nb
                                    state["miss"] = 0
                                    print(f"[재획득] {state['color']} bearing={nb:+.1f}deg")
                        except Exception:
                            pass

            co_detect = (obj_range is not None) and color_present

            # S3: LiDAR 미검출 시 카메라 단안 거리(크기 기반, 표시용. 접근은 LiDAR 필수)
            cam_range = None
            if box is not None and args.obj_width_cm > 0 and view is not None:
                wpx = box[2] - box[0]
                if wpx > 2:
                    cam_range = monocular_range_mm(
                        float(wpx), float(view.shape[1]), CAMERA_HFOV_DEG, zcam,
                        args.obj_width_cm * 10.0)

            # 20cm 접근 알림(1회성 래치): 진입 시 5초 표시, 25cm 밖으로 나가면 재무장
            if obj_range is not None and obj_range <= NOTICE_MM:
                if state["notice_armed"]:
                    state["notice_t"] = time.time()
                    state["notice_armed"] = False
                    print("[알림] 20cm 이내로 접근했습니다")
            elif obj_range is None or obj_range > NOTICE_REARM_MM:
                state["notice_armed"] = True

            # 진단: 최근접 점(전방각 보정/스캔평면 확인용). 물체를 정면에 두면
            # 그 raw 각도가 곧 FORWARD_ANGLE_DEG, 베어링은 ~0 이어야 한다.
            near_a, near_d = None, None
            for a, d in dd.items():
                if d > 0 and (near_d is None or d < near_d):
                    near_a, near_d = a, d

            # ----- 자율 접근 (ARM 이고 동시감지일 때만) -----
            if state["armed"]:
                if lidar is not None and not fresh:
                    send("STOP")
                    if time.time() - state.get("wait_t", 0) > 1.0:
                        state["wait_t"] = time.time()
                        print("[자율대기] LiDAR STALE — 데이터 끊김")
                elif co_detect:
                    # 차단물 가드: 표적보다 20cm 이상 가까운 점이 전방 ±20° + 35cm 안에
                    # 난입하면 즉시 정지. (RangeGate 는 난입을 스파이크로 거부하고 직전
                    # 표적거리로 계속 전진하므로 이 가드가 없으면 가로막혀도 박는다.)
                    blocker = blocking_distance(dd, obj_range) if fresh else None
                    if blocker is not None and blocker < OBSTACLE_STOP_MM:
                        send("STOP")
                        state["armed"] = False
                        print(f"[자율정지] 전방 차단물 {blocker/10:.0f}cm"
                              f" (표적 {obj_range/10:.0f}cm 앞을 가로막음)")
                    else:
                        try:
                            # 반환: (vx[+전진], vy[+우strafe], w[+우회전CW], state)
                            vx, vy, w, st = approach_command(
                                obj_range, bearing, args.target_mm,
                                deadband_mm=APPROACH_DEADBAND_MM, min_safe_mm=APPROACH_MIN_SAFE_MM,
                                face_tol_deg=APPROACH_FACE_TOL_DEG, kx=APPROACH_KX, kw=APPROACH_KW,
                                vx_max=APPROACH_VX_MAX, w_max=APPROACH_W_MAX)
                            send(f"V {vx} {vy} {w}")
                            if st in ("ARRIVED", "TOO_CLOSE", "LOST"):
                                send("STOP")
                                state["armed"] = False
                                print(f"[자율정지] {st} (range={obj_range})")
                        except Exception as exc:             # 제어 예외 -> 안전 정지 + 해제
                            send("STOP")
                            state["armed"] = False
                            print(f"[오류] approach 중단 -> STOP: {exc}")
                elif color_present and cam_range is not None:
                    # Q1-b 카메라 단독 원거리 단계: LiDAR 가 표적을 아직 못 잡는 구간을
                    # 단안 거리로 서행 접근. LiDAR 가 잡히는 순간 위 co_detect 분기로 인계.
                    obst = min_distance_in_arc(dd, FORWARD_ANGLE_DEG, 20.0) if fresh else None
                    if obst is not None and obst < OBSTACLE_STOP_MM:
                        send("STOP")                          # 표적이 아니어도 전방 장애물 정지
                        state["armed"] = False
                        print(f"[자율정지] OBSTACLE {obst/10:.0f}cm (전방 장애물)")
                    else:
                        try:
                            vx, vy, w, st = cam_approach_command(
                                cam_range, bearing, CAM_APPROACH_MIN_MM,
                                face_tol_deg=APPROACH_FACE_TOL_DEG, kw=APPROACH_KW,
                                vx_far=CAM_APPROACH_VX, w_max=APPROACH_W_MAX)
                            send(f"V {vx} {vy} {w}")
                            if st in ("CAM_LIMIT", "LOST"):
                                send("STOP")
                                state["armed"] = False
                                print(f"[자율정지] {st} — LiDAR 미획득 상태로 60cm 도달"
                                      f"(cam~{cam_range/10:.0f}cm). 스캔평면/레벨링 확인 필요")
                        except Exception as exc:
                            send("STOP")
                            state["armed"] = False
                            print(f"[오류] cam approach 중단 -> STOP: {exc}")
                else:
                    send("STOP")  # 동시감지 안되면 정지 후 대기
                    if time.time() - state.get("wait_t", 0) > 1.0:   # 막힌 이유를 말한다
                        state["wait_t"] = time.time()
                        print(f"[자율대기] 색={'O' if color_present else 'X'}"
                              f" 라이다거리={'O' if obj_range is not None else 'X'}"
                              f" cam거리={'O' if cam_range is not None else 'X'}"
                              f" bearing={'없음(클릭 필요)' if bearing is None else f'{bearing:+.0f}deg'}"
                              f" lock={state['color'] or 'auto(카메라쪽 물체를 클릭해 색 잠금 권장)'}")

            # ----- 수동 주행(hold-to-drive): 키 누르는 동안 반복 전송, 떼면 STOP -----
            if state["manual"] is not None and not state["armed"]:
                hold = FINE_HOLD_S if state["fine"] else MANUAL_HOLD_S
                if time.time() - state["manual_t"] <= hold:
                    mvx, mvy, mw = state["manual"]
                    send(f"V {mvx} {mvy} {mw}")
                else:
                    send("STOP")
                    state["manual"] = None

            # ----- 그리기 -----
            cam_panel = np.zeros((CAM_H, CAM_W, 3), np.uint8)
            if view is not None:
                disp = view.copy()
                if box is not None:
                    bcol = (0, 255, 0) if co_detect else (0, 180, 255)
                    cv2.rectangle(disp, box[:2], box[2:], bcol, 3)
                    label = color_name or "?"
                    if obj_range is not None:
                        label += f" {obj_range/10:.0f}cm"
                    elif cam_range is not None:
                        label += f" cam~{cam_range/10:.0f}cm"   # LiDAR 미검출 -> 단안 추정
                    cv2.putText(disp, label, (box[0], max(18, box[1] - 8)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, bcol, 2)
                cam_panel = cv2.resize(disp, (CAM_W, CAM_H))

            # 카메라 조준선: 추적 방위가 (줌 반영) 유효 FOV 안이면 그 픽셀에 세로선
            if bearing is not None and abs(bearing) <= eff_half:
                axp = int(view_x_from_bearing(bearing, CAM_W, CAMERA_HFOV_DEG, zcam))
                cv2.line(cam_panel, (axp, 0), (axp, CAM_H), (0, 200, 255), 1)

            # 20cm 접근 배너(5초). cv2 폰트는 한글 미지원 -> 영문 표기(콘솔엔 한글 출력됨).
            if time.time() - state["notice_t"] <= NOTICE_S:
                bx1, by1 = int(CAM_W * 0.14), int(CAM_H * 0.06)
                bx2, by2 = int(CAM_W * 0.86), int(CAM_H * 0.22)
                cv2.rectangle(cam_panel, (bx1, by1), (bx2, by2), (25, 25, 25), -1)
                cv2.rectangle(cam_panel, (bx1, by1), (bx2, by2), (0, 255, 0), 2)
                cv2.putText(cam_panel, "WITHIN 20 cm", (int(CAM_W * 0.24), int(CAM_H * 0.175)),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0 * sui, (0, 255, 0), max(2, int(2 * sui)))

            rmax = state["rmax"]
            scale = (RAD / 2 - 12) / rmax
            radar = draw_radar(dd, rmax, scale, hist)
            if bearing is not None:
                b = math.radians(bearing)
                bcol2 = (0, 255, 0) if co_detect else (0, 200, 255)
                drawn = False
                if len(cluster) >= 2 and obj_range is not None and obj_range <= rmax:
                    # S2: 감지된 '표면 점들'을 감싸는 회전 사각(매 프레임 표면에 맞춰 변형)
                    pts = []
                    for a2, d2 in cluster:
                        if d2 <= rmax:
                            bb = math.radians(lidar_bearing(a2))
                            pts.append([int(cx0 + d2 * scale * math.sin(bb)),
                                        int(cy0 - d2 * scale * math.cos(bb))])
                    if len(pts) >= 2:
                        (rcx, rcy), (rw, rh), rang = cv2.minAreaRect(np.array(pts, np.int32))
                        pad = 8 * sui
                        bpts = cv2.boxPoints(((rcx, rcy), (rw + pad, rh + pad), rang))
                        cv2.polylines(radar, [bpts.astype(np.int32)], True, bcol2, 2)
                        drawn = True
                if not drawn and obj_range is not None and obj_range <= rmax:
                    # 폴백: 점이 1개뿐이면 고정 사각(±ARC 각폭 비례)
                    pxr = int(cx0 + obj_range * scale * math.sin(b))
                    pyr = int(cy0 - obj_range * scale * math.cos(b))
                    half = max(int(10 * sui),
                               int(obj_range * scale * math.tan(math.radians(APPROACH_ARC_DEG))) + int(6 * sui))
                    cv2.rectangle(radar, (pxr - half, pyr - half), (pxr + half, pyr + half), bcol2, 2)
                elif not drawn and obj_range is None:
                    # 거리 미확보: 가장자리에 짧은 방향 틱만(가독성 위해 관통선 제거)
                    e1x = int(cx0 + (RAD / 2 - 30) * math.sin(b))
                    e1y = int(cy0 - (RAD / 2 - 30) * math.cos(b))
                    e2x = int(cx0 + (RAD / 2 - 10) * math.sin(b))
                    e2y = int(cy0 - (RAD / 2 - 10) * math.cos(b))
                    cv2.line(radar, (e1x, e1y), (e2x, e2y), (0, 200, 255), 2)
            if near_a is not None:                                  # 최근접점=마젠타(진단)
                nb = math.radians(lidar_bearing(near_a))
                npx = int(cx0 + near_d * scale * math.sin(nb))
                npy = int(cy0 - near_d * scale * math.cos(nb))
                if 0 <= npx < RAD and 0 <= npy < RAD:
                    cv2.circle(radar, (npx, npy), max(4, int(4 * sui)), (255, 0, 255), -1)

            canvas = np.hstack([cam_panel, radar])
            arm_txt = "ARMED" if state["armed"] else "disarmed"
            arm_col = (0, 0, 255) if state["armed"] else (160, 160, 160)
            cv2.putText(canvas, arm_txt, (8, int(24 * sui)), cv2.FONT_HERSHEY_SIMPLEX,
                        0.55 * sui, arm_col, max(2, int(1.4 * sui)))
            stat = f"lidar={'OK' if fresh else 'STALE'} cam={'OK' if frame is not None else 'NO'}"
            if obj_range is not None:
                stat += f" range={obj_range/10:.0f}cm" + ("(HOLD)" if gate_state == "HOLD" else "")
            if bearing is not None:
                stat += f" bearing={bearing:+.1f}deg"
            stat += (f" codetect={'Y' if co_detect else 'N'} target={args.target_mm/10:.0f}cm"
                     f" view={state['rmax']/1000:.1f}m zoom={zcam:.2f}x"
                     f" anc={state['anchor_y']:.2f}"
                     f" {'AF' if state['af'] else 'f=' + str(state['focus'])}"
                     f" lock={state['color'] or 'auto'} mspd={state['mspeed']}"
                     f"{' FINE' if state['fine'] else ''} fwd={state['forward']:.0f} fps={cam_fps:.0f}")
            cv2.putText(canvas, stat, (8, CAM_H - int(10 * sui)), cv2.FONT_HERSHEY_SIMPLEX,
                        0.4 * sui, (0, 255, 255), 1)
            if near_a is not None:   # 진단줄: 최근접점 raw각도/베어링/거리 (정면 보정용)
                diag = f"near raw={near_a} bearing={lidar_bearing(near_a):+.0f}deg {near_d/10:.0f}cm  (정면 물체 두고 'p' 누르면 이 raw 가 전방각)"
                cv2.putText(canvas, diag, (8, CAM_H - int(28 * sui)), cv2.FONT_HERSHEY_SIMPLEX,
                            0.34 * sui, (255, 0, 255), 1)

            cv2.imshow(win, canvas)
            kx = cv2.waitKeyEx(30)           # 방향키(확장코드) 지원. ASCII 는 하위바이트로 비교
            k = kx & 0xFF
            if kx in ARROW_KEYS:             # ←→↑↓ 수동 주행(hold-to-drive)
                ux, uy, uw = ARROW_KEYS[kx]
                sp = state["mspeed"]
                state["armed"] = False
                state["manual"] = (ux * sp, uy * sp, uw * sp)
                state["manual_t"] = time.time()
            elif k == 27:                    # ESC
                break
            elif k == ord('g'):
                state["armed"] = not state["armed"]
                state["manual"] = None
                if state["armed"]:
                    issues = []
                    if ser is None:
                        issues.append("모터 미연결(시작 로그의 [경고] 확인) — 절대 못 움직임")
                    if state["bearing"] is None:
                        issues.append("추적 대상 없음 — 먼저 물체를 클릭")
                    if not state["color"]:
                        issues.append("색 잠금 없음 — 카메라 화면의 물체를 클릭하면 잠김")
                    print("[ARM] 자율접근 켜짐" + (" | 주의: " + " / ".join(issues) if issues else ""))
                else:
                    send("STOP")
                    print("[DISARM] 자율접근 꺼짐")
            elif k == ord(' '):
                state["armed"] = False
                state["manual"] = None
                send("STOP")
            elif k == ord('e'):
                state["armed"] = False
                state["manual"] = None
                send("ESTOP")
            elif k == ord('['):                      # 레이더 범위 축소(줌인): 가까운 물체 크게
                state["rmax"] = max(500.0, state["rmax"] / 1.3)
            elif k == ord(']'):                      # 레이더 범위 확대(줌아웃): 멀리까지
                state["rmax"] = min(12000.0, state["rmax"] * 1.3)
            elif k == ord('='):                      # 카메라 줌인(먼/작은 물체 확대)
                state["cam_zoom"] = min(5.0, state["cam_zoom"] * 1.25)
            elif k == ord('-'):                      # 카메라 줌아웃
                state["cam_zoom"] = max(1.0, state["cam_zoom"] / 1.25)
            elif k == ord('i'):                      # 줌 크롭 창 위로
                state["anchor_y"] = max(0.0, state["anchor_y"] - 0.05)
            elif k == ord('k'):                      # 줌 크롭 창 아래로(가까운 물체)
                state["anchor_y"] = min(1.0, state["anchor_y"] + 0.05)
            elif k == ord('f'):                      # 오토포커스 토글
                if cap is not None:
                    if state["af"]:
                        apply_focus()                 # AF -> 수동(현재값 고정)
                    else:
                        ok = enable_autofocus(cap)
                        state["af"] = True
                        print(f"[포커스] 오토포커스 복귀 ok={ok}")
            elif k == ord(','):                      # 포커스 멀리(-5)
                if cap is not None:
                    state["focus"] = max(0, state["focus"] - FOCUS_STEP)
                    apply_focus()
            elif k == ord('.'):                      # 포커스 가까이(+5)
                if cap is not None:
                    state["focus"] = min(250, state["focus"] + FOCUS_STEP)
                    apply_focus()
            elif k in (ord('1'), ord('2'), ord('3')):     # 수동 속도 프리셋
                state["mspeed"] = {ord('1'): 20, ord('2'): 35, ord('3'): 50}[k]
                print(f"[수동속도] {state['mspeed']}%")
            elif k == ord('x'):                           # 미세 모드 토글(짧은 펄스+저속)
                state["fine"] = not state["fine"]
                print(f"[미세모드] {'ON (탭=조금씩)' if state['fine'] else 'OFF'}")
            elif k == ord('p'):                           # 전방각 캡처: 정면 물체의 최근접점 raw각
                if near_a is not None:
                    state["forward"] = float(near_a)
                    print(f"[전방각] FORWARD_ANGLE_DEG = {int(near_a)} (이번 세션 적용). "
                          f"영구히: config.py 의 FORWARD_ANGLE_DEG = {int(near_a)}")
                else:
                    print("[전방각] 최근접점 없음 -> 정면 30~100cm 에 물체 하나 두고 다시 p")
            elif k in MANUAL_KEYS:                        # hold-to-drive 수동 주행
                ux, uy, uw = MANUAL_KEYS[k]
                sp = state["mspeed"]
                if state["fine"]:
                    sp = max(8, int(sp * FINE_SPEED_MUL))
                state["armed"] = False
                state["manual"] = (ux * sp, uy * sp, uw * sp)
                state["manual_t"] = time.time()

    except KeyboardInterrupt:
        pass
    finally:
        send("ESTOP")
        send("STOP")
        if grabber is not None:
            grabber.release()          # 내부에서 cap.release() 까지 수행
        elif cap is not None:
            cap.release()
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
