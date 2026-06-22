"""camera_full_view.py — 두 카메라를 '각각 전체 화면'으로 따로 띄우는 정렬용 뷰어.

용도: 물리적으로 카메라 위치를 맞출 때, 각 카메라의 '풀 프레임'을 독립 창으로 크게 보고
      가운데 기준선(세로/가로 십자)에 기준 물체를 올려 좌/우 카메라를 같은 중심에 맞춘다.
      (작은 패널로 줄이지 않음 — 창 2개를 각각 옮기거나 최대화해서 본다.)
      기준이 잡히면 합쳐 보는 건 track_and_approach.py 의 와이드/겹침제거(o) 로 본다.

표시(각 창):
  - 두 창 '동일 크기'(--view-w x 9/16). 카메라 해상도가 달라도 레터박스로 같은 크기·무왜곡.
  - 풀 프레임(잘라내지 않음, 비율 유지)
  - 가운데 세로선 = '가운데 기준'(밝은 노랑) + 가운데 가로선
  - g: 격자(thirds) on/off

조작(어느 창이든 키 입력 됨):
  q 또는 ESC = 종료,  g = 격자,  f = AF 복귀,  , / . = 수동포커스 -/+ (양 카메라)

실행:
  py -3.13 visualization/camera_full_view.py                         (config 의 좌/우 인덱스)
  py -3.13 visualization/camera_full_view.py --cam-left 1 --cam-right 2 --view-w 960
"""
import argparse
import sys
import pathlib

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))

from common.config import CAM_LEFT_INDEX, CAM_RIGHT_INDEX  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description="두 카메라 각각 전체화면 뷰어(정렬용)")
    ap.add_argument("--cam-left", type=int, default=CAM_LEFT_INDEX, help="왼쪽(=하양) 카메라 cv2 인덱스")
    ap.add_argument("--cam-right", type=int, default=CAM_RIGHT_INDEX, help="오른쪽(=검정) 카메라 cv2 인덱스")
    ap.add_argument("--view-w", type=int, default=960, help="각 창 표시 가로폭(px). 두 창 동일 크기(높이=9/16). 0=기본960")
    ap.add_argument("--width", type=int, default=None, help="카메라 캡처 가로(대역폭 부족 시 640)")
    ap.add_argument("--height", type=int, default=None, help="세로(640이면 480)")
    ap.add_argument("--fps", type=int, default=None, help="FPS(30으로 낮추면 대역폭 절반)")
    args = ap.parse_args()

    import cv2
    import numpy as np
    from common.camera import (open_camera, camera_info, FrameGrabber, CAMERA_WIDTH,
                               CAMERA_HEIGHT, CAMERA_FPS, set_manual_focus,
                               enable_autofocus, FOCUS_STEP)
    cw = args.width or CAMERA_WIDTH
    ch = args.height or CAMERA_HEIGHT
    cf = args.fps or CAMERA_FPS

    def open_side(idx, name):
        try:
            cap = open_camera(idx, cw, ch, cf)
        except Exception as e:
            print(f"[{name}] open 예외: {e}")
            return None
        if cap is None or not cap.isOpened():
            print(f"[{name}] 카메라({idx}) 열기 실패 — 인덱스/USB 확인(find_camera).")
            return None
        info = camera_info(cap)
        print(f"[{name}] 카메라({idx}) OK  {info}")
        if "MJPG" not in info:
            print(f"  [주의] {name}: MJPG 미적용(YUY2) -> 두 카메라 동시구동 시 USB 대역폭 초과로"
                  f" 끊길 수 있음. '--width 640 --height 480' 또는 USB 포트 분리.")
        return FrameGrabber(cap)

    # 두 창을 '같은 크기'로 강제(예전엔 OS 기본값대로 제각각이라 한쪽이 작게 떴음).
    # 프레임은 이 캔버스에 레터박스(비율 유지, 패딩)로 맞춰 카메라 해상도가 달라도 동일 크기.
    DW = int(args.view_w) if (args.view_w and args.view_w > 0) else 960
    DH = int(DW * 9 / 16)

    sides = [("L", "camera LEFT (white)", args.cam_left),
             ("R", "camera RIGHT (black)", args.cam_right)]
    cams = {}
    tile = 0
    for s, title, idx in sides:
        g = open_side(idx, title)
        if g is not None:
            cams[s] = {"grab": g, "win": title}
            cv2.namedWindow(title, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(title, DW, DH)            # 두 창 동일 크기 강제
            cv2.moveWindow(title, tile * (DW + 12), 0)  # 좌/우 타일 배치
            tile += 1
    if not cams:
        print("[실패] 두 카메라 모두 열지 못함. --cam-left/--cam-right 확인.")
        return
    print(f"두 창 동일 크기({DW}x{DH}).  q/ESC 종료, g 격자, f AF, ,/. 포커스")

    state = {"grid": True, "focus": None}

    def render(frame):
        """프레임을 DWxDH 캔버스에 레터박스(비율 유지)로 올리고 가운데 기준선/격자."""
        h0, w0 = frame.shape[:2]
        canvas = np.zeros((DH, DW, 3), np.uint8)
        scale = min(DW / w0, DH / h0)
        nw, nh = max(1, int(w0 * scale)), max(1, int(h0 * scale))
        ox, oy = (DW - nw) // 2, (DH - nh) // 2
        canvas[oy:oy + nh, ox:ox + nw] = cv2.resize(frame, (nw, nh))
        cx, cy = ox + nw // 2, oy + nh // 2          # 영상 중심(=캔버스 중심)
        if state["grid"]:
            for gx in (ox + nw // 3, ox + 2 * nw // 3):
                cv2.line(canvas, (gx, oy), (gx, oy + nh), (50, 50, 50), 1)
            for gy in (oy + nh // 3, oy + 2 * nh // 3):
                cv2.line(canvas, (ox, gy), (ox + nw, gy), (50, 50, 50), 1)
        # 가운데 기준 십자(세로=정렬 기준, 밝게)
        cv2.line(canvas, (cx, 0), (cx, DH), (0, 220, 255), 2)
        cv2.line(canvas, (0, cy), (DW, cy), (0, 200, 200), 1)
        cv2.drawMarker(canvas, (cx, cy), (0, 220, 255), cv2.MARKER_CROSS, 22, 1)
        return canvas

    try:
        while True:
            for s in cams:
                fr = cams[s]["grab"].latest()
                if fr is None:
                    fr = np.zeros((DH, DW, 3), np.uint8)
                    cv2.putText(fr, "no frame", (DW // 2 - 60, DH // 2),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (120, 120, 120), 2)
                    cv2.imshow(cams[s]["win"], fr)
                    continue
                cv2.imshow(cams[s]["win"], render(fr))

            k = cv2.waitKey(30) & 0xFF
            if k in (ord('q'), 27):
                break
            elif k == ord('g'):
                state["grid"] = not state["grid"]
            elif k == ord('f'):
                for c in cams.values():
                    enable_autofocus(c["grab"].cap)
                state["focus"] = None
                print("[포커스] AF 복귀")
            elif k in (ord(','), ord('.')):
                cur = 0 if state["focus"] is None else state["focus"]
                cur = max(0, min(250, cur + (FOCUS_STEP if k == ord('.') else -FOCUS_STEP)))
                state["focus"] = cur
                for c in cams.values():
                    set_manual_focus(c["grab"].cap, cur)
                print(f"[포커스] set={cur}")
    except KeyboardInterrupt:
        pass
    finally:
        for c in cams.values():
            c["grab"].release()
        cv2.destroyAllWindows()
        print("종료")


if __name__ == "__main__":
    main()
