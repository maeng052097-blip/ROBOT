"""depth_detect.py — Orbbec Gemini 335 뎁스 카메라: 실시간 뷰 + '무학습' 물체 검출 + 3D 좌표.

목적(로봇 이동 불필요, 카메라만 있으면 됨):
  자작 단색 도형(빨강/파랑 × 직육면체/구체)을 데이터 학습 없이 검출.
  방식 = 뎁스 전경분리(거리 밴드로 배경 잡동사니 제거) -> 색(HSV) -> 형상(원형도/직사각도)
        -> 3D 좌표(카메라 intrinsics 역투영, mm). 부품실에서 임계값만 튜닝하면 됨.

의존: pyorbbecsdk2 (import pyorbbecsdk), opencv-python, numpy.
실행:
  py -3.13 visualization/depth_detect.py                 # 실시간 창
  py -3.13 visualization/depth_detect.py --selftest 30   # GUI 없이 30프레임 검출 결과만 출력
조작(창): q/ESC 종료 | s 프레임 저장(captures/) | [ ] 근거리밴드 -/+ | , . 원거리밴드 -/+
          h HSV 값 콘솔출력(튜닝) | d 뎁스 컬러맵 토글
튜닝 포인트: 부품실 배경에 빨강/파랑 잡동사니가 있으면 depth 밴드(전경)로 우선 거르고,
            그래도 남으면 HSV 임계(--red/--blue)나 min-area 를 조정.
"""
import argparse
import sys
import pathlib
import math

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))

try:    # 콘솔(cp949)에 없는 문자 print 해도 앱이 안 죽게
    sys.stdout.reconfigure(errors="replace")
    sys.stderr.reconfigure(errors="replace")
except Exception:
    pass

import numpy as np
import cv2


# ===== 검출 파라미터 기본값 (부품실에서 튜닝) =====
# OpenCV HSV: H 0~179, S 0~255, V 0~255. 빨강은 H가 0/180 근처로 갈라져 두 구간.
RED_RANGES = [((0, 90, 60), (10, 255, 255)), ((170, 90, 60), (179, 255, 255))]
BLUE_RANGES = [((100, 90, 60), (130, 255, 255))]
SPHERE_CIRC = 0.85     # 원형도 >= 이 값이면 구(구체 실루엣=원)
BOX_RECT = 0.80        # 직사각도(윤곽면적/최소사각면적) >= 이 값이면 직육면체
KERNEL = np.ones((5, 5), np.uint8)


def _ranges_np(ranges):
    return [(np.array(lo, np.uint8), np.array(hi, np.uint8)) for lo, hi in ranges]


def parse_lut(s):
    """'측정:실제,측정:실제,...'(mm) -> [(meas, true), ...] 측정값 오름차순. 빈/None이면 None."""
    if not s:
        return None
    pts = []
    for tok in s.split(","):
        tok = tok.strip()
        if not tok:
            continue
        m, t = tok.split(":", 1)
        pts.append((float(m), float(t)))
    if not pts:
        return None
    pts.sort(key=lambda p: p[0])
    return pts


def correct_depth(zraw, scale=1.0, offset=0.0, lut=None):
    """깊이 보정: lut 있으면 구간별 선형보간(양끝은 끝구간 기울기로 외삽),
    없으면 단일선형 scale*z+offset. zraw<=0(무효)면 0."""
    if zraw <= 0:
        return 0.0
    if not lut:
        return scale * zraw + offset
    n = len(lut)
    if n == 1:                                   # 1점 -> 상수 오프셋만
        return zraw + (lut[0][1] - lut[0][0])
    if zraw <= lut[0][0]:                         # 하한 밖 -> 첫 구간으로 외삽
        (x0, y0), (x1, y1) = lut[0], lut[1]
    elif zraw >= lut[-1][0]:                      # 상한 밖 -> 끝 구간으로 외삽
        (x0, y0), (x1, y1) = lut[-2], lut[-1]
    else:
        i = 0
        while i < n - 1 and not (lut[i][0] <= zraw <= lut[i + 1][0]):
            i += 1
        (x0, y0), (x1, y1) = lut[i], lut[i + 1]
    if x1 == x0:
        return y0
    return y0 + (y1 - y0) * (zraw - x0) / (x1 - x0)


# Gemini 335 데이터시트 v1.1: 깊이 해상도별 최소측정거리(Min-Z, mm). 이보다 가까우면 사각(구멍/노이즈).
# 풀해상도 1280x800 은 260mm 라 ~200mm 파지엔 848x480(180)/640x360(140) 등으로 낮춰야 함.
DEPTH_MINZ_MM = {
    (1280, 800): 260, (1280, 720): 260, (640, 400): 260,
    (848, 480): 180, (848, 100): 180, (640, 480): 170,
    (640, 360): 140, (480, 270): 110, (424, 240): 100,
}


def _parse_res(s):
    """'848x480' -> (848,480). 빈/None -> None."""
    if not s:
        return None
    w, h = s.lower().split("x")
    return int(w), int(h)


def _enable_depth(cfg, ob, res_str):
    """깊이 스트림 활성화. res_str='848x480' 등이면 그 해상도(Y16, fps=any) 요청, 실패/미지정 시 기본."""
    res = _parse_res(res_str)
    if res is not None:
        try:
            cfg.enable_video_stream(ob.OBSensorType.DEPTH_SENSOR, res[0], res[1], 0, ob.OBFormat.Y16)
            return
        except Exception as e:
            print(f"[경고] 깊이 {res[0]}x{res[1]} 요청 실패({e}) -> 기본 해상도 사용")
    cfg.enable_stream(ob.OBSensorType.DEPTH_SENSOR)


def detect(color_bgr, depth_mm, zmin, zmax, colors, min_area_px, intr,
           depth_scale=1.0, depth_offset=0.0, depth_lut=None):
    """뎁스 전경(거리밴드) ∩ 색(HSV) 블롭 -> 형상 분류 + 3D 좌표. 검출 리스트 반환(면적 내림차순)."""
    hsv = cv2.cvtColor(color_bgr, cv2.COLOR_BGR2HSV)
    band = ((depth_mm >= zmin) & (depth_mm <= zmax)).astype(np.uint8) * 255   # 전경(거리) 마스크
    dets = []
    for cname, ranges in colors.items():
        cmask = None
        for lo, hi in ranges:
            m = cv2.inRange(hsv, lo, hi)
            cmask = m if cmask is None else cv2.bitwise_or(cmask, m)
        mask = cv2.bitwise_and(cmask, band)                    # 색 ∩ 전경
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, KERNEL)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, KERNEL)
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in cnts:
            area = cv2.contourArea(c)
            if area < min_area_px:
                continue
            per = cv2.arcLength(c, True)
            circ = 4.0 * math.pi * area / (per * per) if per > 0 else 0.0
            (_rx, _ry), (rw, rh), _ang = cv2.minAreaRect(c)
            rectangularity = area / (rw * rh) if rw * rh > 0 else 0.0
            if circ >= SPHERE_CIRC:
                shape = "sphere"
            elif rectangularity >= BOX_RECT:
                shape = "box"
            else:
                shape = "?"
            x, y, w, h = cv2.boundingRect(c)
            cxp, cyp = x + w // 2, y + h // 2
            # 3D: 중심 주변 패치의 유효 뎁스 중앙값(노이즈 억제)
            py1, py2 = max(0, cyp - 4), min(depth_mm.shape[0], cyp + 5)
            px1, px2 = max(0, cxp - 4), min(depth_mm.shape[1], cxp + 5)
            patch = depth_mm[py1:py2, px1:px2]
            valid = patch[(patch >= zmin) & (patch <= zmax)]
            Zraw = float(np.median(valid)) if valid.size else 0.0
            zstd = float(np.std(valid)) if valid.size > 1 else 0.0    # 패치내 깊이 산포(노이즈/구멍 지표)
            nvalid, npatch = int(valid.size), int(patch.size)         # 유효화소/전체(근접 사각서 급감)
            # 깊이 보정: lut 있으면 구간별, 없으면 단일선형(scale*Z+offset). tests/depth_calibrate.py 로 산출
            Z = correct_depth(Zraw, depth_scale, depth_offset, depth_lut)
            X = (cxp - intr["cx"]) * Z / intr["fx"] if Z > 0 else 0.0
            Y = (cyp - intr["cy"]) * Z / intr["fy"] if Z > 0 else 0.0
            dets.append({"color": cname, "shape": shape, "bbox": (x, y, w, h),
                         "c": (cxp, cyp), "xyz": (X, Y, Z),
                         "circ": circ, "rect": rectangularity, "area": area,
                         "zraw": Zraw, "zstd": zstd, "nvalid": nvalid, "npatch": npatch})
    dets.sort(key=lambda d: -d["area"])
    return dets


def _decode_color(cf):
    """컬러 프레임(MJPG) -> BGR ndarray. 실패 시 None."""
    import pyorbbecsdk as ob
    buf = np.frombuffer(cf.get_data(), np.uint8)
    fmt = cf.get_format()
    if fmt == ob.OBFormat.MJPG:
        return cv2.imdecode(buf, cv2.IMREAD_COLOR)
    if fmt == ob.OBFormat.BGR:
        return buf.reshape(cf.get_height(), cf.get_width(), 3)
    if fmt == ob.OBFormat.RGB:
        return cv2.cvtColor(buf.reshape(cf.get_height(), cf.get_width(), 3), cv2.COLOR_RGB2BGR)
    return cv2.imdecode(buf, cv2.IMREAD_COLOR)   # 최후 시도


def main():
    ap = argparse.ArgumentParser(description="Gemini 335 뎁스 검출/뷰어(무학습 색+형상+뎁스)")
    ap.add_argument("--zmin", type=int, default=150, help="전경 근거리 하한(mm)")
    ap.add_argument("--zmax", type=int, default=1000, help="전경 원거리 상한(mm)")
    ap.add_argument("--min-area", type=int, default=400, help="최소 블롭 면적(px)")
    # 깊이 보정 기본값 = 근거리(848x480, 파지) 실측 캘리브 (SN CP06563000NY, 2026-07).
    #   Z_true = 1.01609*Z + 0.29  (raw 180~400mm 5점, 잔차 <=1.1mm, 홀드아웃 1.0mm). 848x480 전용.
    #   ★해상도별로 계수가 다름(fx/노이즈 상이) -> 해상도에 맞는 프로파일을 쓸 것:
    #     - 근거리/파지  (848x480) : --depth-res 848x480  --depth-scale 1.01609 --depth-offset 0.29   (= 기본값)
    #     - 먼거리/탐지  (1280x800): --depth-res 1280x800 --zmin 500 --zmax 2200 --depth-lut "586:600,877:900,1161:1200,1528:1600,1878:2000"
    #                                (★밴드는 raw 깊이 기준: zmax<물체raw면 검출 안 됨. 2m=raw~1878 이라 zmax 2200.
    #                                 먼거리는 비선형이라 LUT. 바이어스만 보정, 노이즈~1%@2m 잔존 -> 거리는 LiDAR 권장)
    #   Min-Z: 848x480=180mm(그 미만 사각, 150mm 측정불가 실증), 1280x800=260mm. 재장착/프리셋 변경 시 재캘리브.
    #   보정 끄기(raw 측정용): --depth-scale 1.0 --depth-offset 0
    ap.add_argument("--depth-scale", type=float, default=1.01609,
                    help="깊이 선형보정 배율 (Z_true=scale*Z+offset). tests/depth_calibrate.py 로 산출")
    ap.add_argument("--depth-offset", type=float, default=0.29, help="깊이 선형보정 오프셋(mm)")
    ap.add_argument("--depth-lut", type=str, default="",
                    help="구간별 보정 '측정:실제,...'(mm). 주면 scale/offset 대신 구간선형보간 사용. "
                         "근/원 비선형 대응. tests/depth_calibrate.py 가 출력해 줌")
    ap.add_argument("--depth-res", type=str, default="",
                    help="깊이 해상도 'WxH'(예 848x480, 640x360). Min-Z(최소측정거리)가 해상도별로 다름: "
                         "1280x800=260mm, 848x480=180mm, 640x360=140mm. ~200mm 파지엔 848x480 이하 필요. "
                         "미지정 시 SDK 기본. 해상도 바꾸면 캘리브 재측정 필요")
    ap.add_argument("--target-color", choices=["any", "red", "blue"], default="any",
                    help="주검출 대상 색. red/blue 지정 시 그 색의 최대블롭을 표적으로 삼음 "
                         "(먼거리서 배경 벽이 최대블롭이 되는 것 방지 -> selftest 재측정에 필수). 기본 any")
    ap.add_argument("--selftest", type=int, default=0, help=">0 이면 GUI 없이 그만큼 프레임 검출 후 종료")
    args = ap.parse_args()
    depth_lut = parse_lut(args.depth_lut)

    import pyorbbecsdk as ob

    pipe = ob.Pipeline()
    cfg = ob.Config()
    _enable_depth(cfg, ob, args.depth_res)                  # 해상도 지정(Min-Z 제어) 또는 기본
    cfg.enable_stream(ob.OBSensorType.COLOR_SENSOR)
    pipe.start(cfg)
    align = ob.AlignFilter(ob.OBStreamType.COLOR_STREAM)   # 뎁스를 컬러 좌표계로 정렬

    # intrinsics(컬러 기준) — 정렬 후 뎁스도 컬러 좌표라 rgb_intrinsic 사용
    try:
        p = pipe.get_camera_param()
        ci = p.rgb_intrinsic
        intr0 = {"fx": ci.fx, "fy": ci.fy, "cx": ci.cx, "cy": ci.cy, "w": ci.width, "h": ci.height}
    except Exception as e:
        print("[경고] intrinsics 획득 실패, 근사값 사용:", e)
        intr0 = None

    colors = {"red": _ranges_np(RED_RANGES), "blue": _ranges_np(BLUE_RANGES)}
    state = {"zmin": args.zmin, "zmax": args.zmax, "show_depth": True}
    cap_dir = pathlib.Path(__file__).resolve().parent.parent / "captures"
    win = "depth_detect (Gemini335)"
    if not args.selftest:
        cv2.namedWindow(win, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(win, 1280, 540)

    def scaled_intr(W, H):
        if intr0 is None:
            f = 0.9 * W    # 근사(정확치 필요하면 캘리브). HFOV~약 90도 가정
            return {"fx": f, "fy": f, "cx": W / 2.0, "cy": H / 2.0}
        sx, sy = W / intr0["w"], H / intr0["h"]
        return {"fx": intr0["fx"] * sx, "fy": intr0["fy"] * sy,
                "cx": intr0["cx"] * sx, "cy": intr0["cy"] * sy}

    saved = [0]
    frames = 0
    zacc, nvacc = [], []        # selftest 요약용: 검출된 top 의 raw Z / 유효화소 누적
    printed_res = [False]
    try:
        while True:
            fs = pipe.wait_for_frames(200)
            if fs is None:
                if args.selftest and frames == 0:
                    pass
                continue
            af = align.process(fs)
            afs = af.as_frame_set() if af is not None else fs
            df = afs.get_depth_frame()
            cf = afs.get_color_frame()
            if df is None or cf is None:
                continue
            if not printed_res[0]:      # native(정렬 전) 깊이 해상도 + Min-Z (1회). 정렬 후는 컬러격자로 리샘플됨
                rdf = fs.get_depth_frame()          # 정렬 전 원본 = 깊이센서 native 해상도(Min-Z 결정)
                ndw, ndh = (rdf.get_width(), rdf.get_height()) if rdf is not None else (df.get_width(), df.get_height())
                mz = DEPTH_MINZ_MM.get((ndw, ndh))
                print(f"[깊이 native] {ndw}x{ndh}" +
                      (f"  Min-Z ~{mz}mm -> 이보다 가까우면 사각" if mz else "  (Min-Z 표에 없음)") +
                      f"  | 정렬후 {df.get_width()}x{df.get_height()}(컬러격자, 3D는 컬러 intrinsic)")
                printed_res[0] = True
            color = _decode_color(cf)
            if color is None:
                continue
            H, W = color.shape[:2]
            depth = np.frombuffer(df.get_data(), np.uint16).reshape(df.get_height(), df.get_width())
            if depth.shape != (H, W):
                depth = cv2.resize(depth, (W, H), interpolation=cv2.INTER_NEAREST)
            intr = scaled_intr(W, H)

            dets = detect(color, depth, state["zmin"], state["zmax"], colors, args.min_area, intr,
                          depth_scale=args.depth_scale, depth_offset=args.depth_offset, depth_lut=depth_lut)
            # 표적색 우선정렬: red/blue 지정 시 그 색 최대블롭을 primary 로(먼거리 배경 무시)
            if args.target_color != "any":
                tgt = [d for d in dets if d["color"] == args.target_color]
                dets = tgt + [d for d in dets if d["color"] != args.target_color]
                primary = tgt[0] if tgt else None
            else:
                primary = dets[0] if dets else None
            frames += 1

            if args.selftest:
                top = primary
                if top:
                    X, Y, Z = top["xyz"]
                    zacc.append(top["zraw"]); nvacc.append(top["nvalid"])
                    print(f"[{frames}] {top['color']} {top['shape']} "
                          f"raw={top['zraw']:.0f} corr={Z:.0f} std={top['zstd']:.1f} "
                          f"valid={top['nvalid']}/{top['npatch']} "
                          f"circ={top['circ']:.2f} rect={top['rect']:.2f} XY=({X:.0f},{Y:.0f})")
                else:
                    print(f"[{frames}] (검출 없음, band {state['zmin']}~{state['zmax']}mm)")
                if frames >= args.selftest:
                    print("---- 요약 ----")
                    if zacc:
                        arr = np.array(zacc)
                        med = float(np.median(arr)); sd = float(np.std(arr))
                        corr_on = not (args.depth_scale == 1.0 and args.depth_offset == 0.0 and not depth_lut)
                        print(f"검출 {len(zacc)}/{frames}프레임  raw중앙값={med:.1f}mm  프레임간std={sd:.1f}mm  "
                              f"raw범위[{arr.min():.0f},{arr.max():.0f}]  평균유효화소={np.mean(nvacc):.0f}")
                        print(f"  캘리브 입력용 -> '{med:.0f}:<실제거리mm>'  "
                              f"(예: py -3.13 tests/depth_calibrate.py {med:.0f}:<실제> ...)")
                        if corr_on:
                            print("  [주의] 깊이보정 적용중 -> 캘리브용 raw 측정이면 "
                                  "--depth-scale 1.0 --depth-offset 0 (또는 --depth-lut 비움)로 다시.")
                    else:
                        print("검출 없음 (band/거리/해상도/색 확인). 근접이면 --depth-res 848x480 등으로 Min-Z 낮추기.")
                    break
                continue

            # ----- 표시 -----
            vis = color.copy()
            for i, d in enumerate(dets[:4]):
                x, y, w, h = d["bbox"]; X, Y, Z = d["xyz"]
                col = (0, 0, 255) if d["color"] == "red" else (255, 0, 0)
                th = 3 if i == 0 else 1
                cv2.rectangle(vis, (x, y), (x + w, y + h), col, th)
                cv2.putText(vis, f"{d['color']} {d['shape']} {Z:.0f}mm", (x, max(16, y - 6)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, col, 2)
                if i == 0:
                    cv2.putText(vis, f"3D(mm) X{X:.0f} Y{Y:.0f} Z{Z:.0f} raw{d['zraw']:.0f} std{d['zstd']:.1f} "
                                     f"vld{d['nvalid']}/{d['npatch']}  circ{d['circ']:.2f} rect{d['rect']:.2f}",
                                (8, H - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.44, (0, 255, 255), 1)
            if depth_lut:
                cal = f"  cal:lut{len(depth_lut)}pt"
            elif args.depth_scale == 1.0 and args.depth_offset == 0.0:
                cal = ""
            else:
                cal = f"  cal:x{args.depth_scale:.4f}{args.depth_offset:+.1f}"
            cv2.putText(vis, f"band {state['zmin']}-{state['zmax']}mm  [ ] , .  s=save  q=quit  det={len(dets)}{cal}",
                        (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)

            if state["show_depth"]:
                dvis = np.clip(depth, state["zmin"], state["zmax"]).astype(np.float32)
                dvis = ((dvis - state["zmin"]) / max(1, state["zmax"] - state["zmin"]) * 255).astype(np.uint8)
                dvis = cv2.applyColorMap(dvis, cv2.COLORMAP_JET)
                dvis[depth == 0] = 0
                canvas = np.hstack([cv2.resize(vis, (W // 2, H // 2)), cv2.resize(dvis, (W // 2, H // 2))])
            else:
                canvas = vis
            cv2.imshow(win, canvas)

            k = cv2.waitKey(1) & 0xFF
            if k in (ord('q'), 27):
                break
            elif k == ord('['):
                state["zmin"] = max(0, state["zmin"] - 50)
            elif k == ord(']'):
                state["zmin"] = min(state["zmax"] - 50, state["zmin"] + 50)
            elif k == ord(','):
                state["zmax"] = max(state["zmin"] + 50, state["zmax"] - 50)
            elif k == ord('.'):
                state["zmax"] = state["zmax"] + 50
            elif k == ord('d'):
                state["show_depth"] = not state["show_depth"]
            elif k == ord('h'):
                d0 = dets[0] if dets else None
                print("[HSV 튜닝] 검출:", "없음" if not d0 else f"{d0['color']} {d0['shape']} 3D={d0['xyz']}")
            elif k == ord('s'):
                cap_dir.mkdir(exist_ok=True)
                n = saved[0]; saved[0] += 1
                cv2.imwrite(str(cap_dir / f"color_{n:03d}.png"), color)
                np.save(str(cap_dir / f"depth_{n:03d}.npy"), depth)   # uint16 mm
                print(f"[저장] captures/color_{n:03d}.png + depth_{n:03d}.npy")
    except KeyboardInterrupt:
        pass
    finally:
        pipe.stop()
        if not args.selftest:
            cv2.destroyAllWindows()
        print(f"종료 (처리 {frames}프레임)")


if __name__ == "__main__":
    main()
