"""camera_align_view.py - 두 카메라 '위치/중심' 물리 정렬 전용 도구. LiDAR/모터 없음.

목적: 본격 작업(겹침제거 crop, toe 보정) 전에 두 카메라를 손으로 맞춘다.
      lidar_level_view.py 의 카메라판. 화면 가이드 + 클릭 측정만 본다.

맞춰야 할 3가지 (이 도구가 각각 보여줌):
  1) roll(수평)  : 실제 수평 모서리(책상/바닥선)를 '공용 수평 기준선'과 격자에 맞춰 두
                   영상이 같은 기울기인지 본다. 한쪽이 기울면 그 카메라를 돌려 수평.
  2) 높이/피치   : 같은 물체를 좌/우에 클릭 -> Δy(높이차)가 0 이 되도록 한쪽 카메라를
                   위/아래로. (Δy≠0 이면 합친 와이드 띠에 세로 단차가 생김)
  3) 좌우 대칭/toe: 정면(로봇 전방축)에 둔 물체를 좌/우에 클릭하면 각 베어링을 표시.
                   - |bearingL|=|bearingR| (표시 |bL|-|bR|=0) -> 두 카메라 좌우 대칭(=LiDAR 정중앙).
                     이 대칭 점검은 '거리에 무관'하게 신뢰할 수 있다(주 용도).
                   - toe 추정 = (bL-bR)/2 = 물리 외향각 τ. 단 '먼 물체'에서만 정확하다:
                     가까우면 시차로 부풀려짐(1m·간격34cm면 +9.6deg 과다). 화면엔 부호 맞춘
                     CAM_TOE_DEG=-τ 도 같이 표시하나, '정확한' toe 값은 track_and_approach 에서
                     n/m 으로 맞추는 것을 쓴다(거긴 LiDAR 거리로 시차를 실제로 풀어 정확).

조작:
  좌클릭(좌/우 패널)   = 그 지점 표식(같은 물체를 양쪽에 찍어 비교)
  i / k                = 공용 수평 기준선 위/아래 이동
  g                    = 격자(thirds) on/off
  b                    = 겹쳐보기(반투명 blend + 차영상 absdiff) on/off  (원거리 물체로 미세정렬)
  r                    = 표식 지우기
  f                    = AF 복귀,  , / .  = 수동포커스 -/+ (양 카메라)
  q 또는 ESC           = 종료

실행:
  py -3.13 visualization/camera_align_view.py            (config 의 좌/우 인덱스 사용)
  py -3.13 visualization/camera_align_view.py --cam-left 1 --cam-right 2 --panel-w 720
"""
import argparse
import sys
import pathlib

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))

try:    # 콘솔(cp949)에 없는 문자를 print 해도 앱이 죽지 않게(? 로 대체). 한글은 그대로.
    sys.stdout.reconfigure(errors="replace")
    sys.stderr.reconfigure(errors="replace")
except Exception:
    pass

from common.config import (CAM_LEFT_INDEX, CAM_RIGHT_INDEX, CAMERA_HFOV_DEG)  # noqa: E402
from common.fusion import view_bearing_deg                                     # noqa: E402


def main():
    ap = argparse.ArgumentParser(description="두 카메라 위치/중심 정렬 (카메라 전용)")
    ap.add_argument("--cam-left", type=int, default=CAM_LEFT_INDEX, help="왼쪽(=하양) 카메라 cv2 인덱스")
    ap.add_argument("--cam-right", type=int, default=CAM_RIGHT_INDEX, help="오른쪽(=검정) 카메라 cv2 인덱스")
    ap.add_argument("--panel-w", type=int, default=640, help="패널 가로 픽셀(세로=9/16)")
    ap.add_argument("--hfov", type=float, default=CAMERA_HFOV_DEG, help="수평화각(deg)")
    args = ap.parse_args()

    import cv2
    import numpy as np
    from common.camera import (open_camera, camera_info, FrameGrabber,
                               set_manual_focus, enable_autofocus, FOCUS_STEP)

    PW = int(args.panel_w)
    PH = int(PW * 9 / 16)

    def open_side(idx, name):
        try:
            cap = open_camera(idx)
        except Exception as e:
            print(f"[{name}] open 예외: {e}")
            return None
        if cap is None or not cap.isOpened():
            print(f"[{name}] 카메라({idx}) 열기 실패 - 인덱스/USB 확인.")
            return None
        print(f"[{name}] 카메라({idx}) OK  {camera_info(cap)}")
        return FrameGrabber(cap)

    cams = {"L": open_side(args.cam_left, "하양/L"), "R": open_side(args.cam_right, "검정/R")}
    if cams["L"] is None and cams["R"] is None:
        print("[실패] 두 카메라 모두 열지 못함. --cam-left/--cam-right 인덱스 확인(find_camera).")
        return

    state = {"ref_y": PH // 2, "grid": True, "blend": False,
             "mark": {"L": None, "R": None}, "focus": None}
    win = "camera align view"
    cv2.namedWindow(win)

    def on_mouse(event, x, y, flags, _p):
        if event != cv2.EVENT_LBUTTONDOWN or state["blend"]:
            return
        if x < PW:
            state["mark"]["L"] = (int(x), int(y))
        elif x < 2 * PW:
            state["mark"]["R"] = (int(x - PW), int(y))

    cv2.setMouseCallback(win, on_mouse)

    def txt(img, s, org, color, sc=0.5, th=1):
        cv2.putText(img, s, org, cv2.FONT_HERSHEY_SIMPLEX, sc, color, th, cv2.LINE_AA)

    def get_panel(side):
        """그 카메라 최신 프레임을 PW×PH 패널로(없으면 'cam off' 검정)."""
        g = cams[side]
        fr = g.latest() if g is not None else None
        if fr is None:
            p = np.zeros((PH, PW, 3), np.uint8)
            txt(p, f"{side}: cam off", (PW // 3, PH // 2), (110, 110, 110), 0.7, 2)
            return p
        return cv2.resize(fr, (PW, PH))

    def draw_guides(panel, side):
        cx, cy = PW // 2, PH // 2
        if state["grid"]:
            for gx in (PW // 3, 2 * PW // 3):
                cv2.line(panel, (gx, 0), (gx, PH), (45, 45, 45), 1)
            for gy in (PH // 3, 2 * PH // 3):
                cv2.line(panel, (0, gy), (PW, gy), (45, 45, 45), 1)
        # 카메라 영상 중심 십자(=광축 기준)
        cv2.line(panel, (cx, 0), (cx, PH), (0, 200, 200), 1)
        cv2.line(panel, (0, cy), (PW, cy), (0, 200, 200), 1)
        # 공용 수평 기준선(양 패널 동일 y) - roll/높이 정렬용
        cv2.line(panel, (0, state["ref_y"]), (PW, state["ref_y"]), (0, 220, 255), 1)
        # 표식
        m = state["mark"][side]
        if m is not None:
            mx, my = m
            cv2.drawMarker(panel, (mx, my), (0, 0, 255), cv2.MARKER_CROSS, 18, 2)
            b = view_bearing_deg(float(mx), float(PW), args.hfov, 1.0)
            txt(panel, f"dx={mx - cx:+d} dy={my - cy:+d} ({b:+.1f}deg)",
                (6, PH - 10), (0, 0, 255), 0.45)
        txt(panel, side + ("/하양" if side == "L" else "/검정"), (6, 18), (0, 255, 255), 0.5)

    try:
        while True:
            pL, pR = get_panel("L"), get_panel("R")

            if state["blend"]:
                # 겹쳐보기: 반투명 blend + 차영상(absdiff). 원거리 물체가 잘 겹치면 정렬 양호.
                blend = cv2.addWeighted(pL, 0.5, pR, 0.5, 0)
                diff = cv2.convertScaleAbs(cv2.absdiff(pL, pR), alpha=2.0)
                cv2.line(blend, (0, state["ref_y"]), (PW, state["ref_y"]), (0, 220, 255), 1)
                cv2.line(blend, (PW // 2, 0), (PW // 2, PH), (0, 200, 200), 1)
                txt(blend, "BLEND (L+R) - 원거리물체가 겹치면 roll/높이 정렬됨", (6, 18), (0, 255, 255), 0.45)
                txt(diff, "DIFF (|L-R|) - 어두울수록 정렬 양호", (6, 18), (200, 200, 0), 0.45)
                canvas = np.hstack([blend, diff])
            else:
                draw_guides(pL, "L")
                draw_guides(pR, "R")
                canvas = np.hstack([pL, pR])
                cv2.line(canvas, (PW, 0), (PW, PH), (70, 70, 70), 1)   # 패널 경계

            # 대칭/높이/ toe 요약(양쪽 표식 있을 때)
            mL, mR = state["mark"]["L"], state["mark"]["R"]
            if mL is not None and mR is not None and not state["blend"]:
                bL = view_bearing_deg(float(mL[0]), float(PW), args.hfov, 1.0)
                bR = view_bearing_deg(float(mR[0]), float(PW), args.hfov, 1.0)
                dy = mL[1] - mR[1]
                toe_est = (bL - bR) / 2.0    # 물리 외향각 τ(거친값: 먼 물체일수록 정확)
                sym = abs(bL) - abs(bR)      # 0=좌우대칭(거리 무관, 신뢰 높음)
                line = (f"dy(L-R)={dy:+d}px(0=같은높이)  |bL|-|bR|={sym:+.1f}deg(0=대칭,신뢰O)  "
                        f"toe~{toe_est:+.1f} ->CAM_TOE_DEG={-toe_est:+.1f}(거친값,먼물체일수록정확)")
                bar = np.zeros((26, canvas.shape[1], 3), np.uint8)
                txt(bar, line, (8, 18), (0, 255, 0), 0.45)
                canvas = np.vstack([canvas, bar])

            footer = np.zeros((24, canvas.shape[1], 3), np.uint8)
            txt(footer, f"i/k 기준선  g 격자  b 겹쳐보기  r 표식지움  f AF  ,/. 포커스{('='+str(state['focus'])) if state['focus'] is not None else ''}  q 종료",
                (8, 16), (180, 180, 180), 0.42)
            canvas = np.vstack([canvas, footer])

            cv2.imshow(win, canvas)
            k = cv2.waitKey(30) & 0xFF
            if k in (ord('q'), 27):
                break
            elif k == ord('i'):
                state["ref_y"] = max(0, state["ref_y"] - 4)
            elif k == ord('k'):
                state["ref_y"] = min(PH - 1, state["ref_y"] + 4)
            elif k == ord('g'):
                state["grid"] = not state["grid"]
            elif k == ord('b'):
                state["blend"] = not state["blend"]
            elif k == ord('r'):
                state["mark"] = {"L": None, "R": None}
            elif k == ord('f'):
                for g in cams.values():
                    if g is not None:
                        enable_autofocus(g.cap)
                state["focus"] = None
                print("[포커스] AF 복귀")
            elif k in (ord(','), ord('.')):
                cur = 0 if state["focus"] is None else state["focus"]
                cur = max(0, min(250, cur + (FOCUS_STEP if k == ord('.') else -FOCUS_STEP)))
                state["focus"] = cur
                for g in cams.values():
                    if g is not None:
                        set_manual_focus(g.cap, cur)
                print(f"[포커스] set={cur}")
    except KeyboardInterrupt:
        pass
    finally:
        for g in cams.values():
            if g is not None:
                g.release()
        cv2.destroyAllWindows()
        print("종료")


if __name__ == "__main__":
    main()
