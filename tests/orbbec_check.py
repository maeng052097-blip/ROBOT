"""orbbec_check.py — Orbbec Gemini 335 뎁스 카메라 기본 점검(연결/스트리밍). 하드웨어 필요.

확인 항목: SDK 가 카메라를 보는가(이름/SN/FW) + 뎁스/컬러 스트리밍 + 중앙 픽셀 거리(mm).
의존: pyorbbecsdk2  (★PyPI 패키지명은 'pyorbbecsdk2'(v2) 지만 import 는 'pyorbbecsdk')
  설치: py -3.13 -m pip install --upgrade pyorbbecsdk2   (Python 3.8~3.13 / Windows10+ x64 지원)
실행: py -3.13 tests/orbbec_check.py
GUI 확인 도구(선택): OrbbecViewer  (github.com/orbbec/OrbbecSDK_v2/releases, win_x64)
"""
import sys

try:    # 콘솔(cp949)에 없는 문자를 print 해도 앱이 안 죽게
    sys.stdout.reconfigure(errors="replace")
    sys.stderr.reconfigure(errors="replace")
except Exception:
    pass


def main():
    try:
        import pyorbbecsdk as ob
    except Exception as e:
        print("[실패] pyorbbecsdk 임포트:", e)
        print("  설치: py -3.13 -m pip install --upgrade pyorbbecsdk2")
        return
    try:
        print("[SDK] version", ob.get_version())
    except Exception:
        pass

    # 1) 장치 열거(SDK 가 카메라를 보는지). ★Context 를 변수로 살려둬야 함
    #    (ob.Context().query_devices() 처럼 임시로 두면 Context 가 GC 되어 deviceMgr NULL 오류)
    ctx = ob.Context()
    devs = ctx.query_devices()
    n = devs.get_count()
    print(f"[장치] Orbbec 장치 {n}개")
    if n == 0:
        print("  카메라 미인식 -> USB/케이블/포트 확인. 다른 앱(OrbbecViewer 등)이 점유 중인지도 확인.")
        return
    for i in range(n):
        info = devs.get_device_by_index(i).get_device_info()
        print(f"  - {info.get_name()} | SN {info.get_serial_number()} | FW {info.get_firmware_version()}")

    # 2) 뎁스+컬러 한 프레임 캡처(스트리밍 검증)
    try:
        import numpy as np
        pipe = ob.Pipeline()
        cfg = ob.Config()
        cfg.enable_stream(ob.OBSensorType.DEPTH_SENSOR)
        cfg.enable_stream(ob.OBSensorType.COLOR_SENSOR)
        pipe.start(cfg)
        got = False
        for _ in range(40):
            fs = pipe.wait_for_frames(200)
            if fs is None:
                continue
            df, cf = fs.get_depth_frame(), fs.get_color_frame()
            if df and cf:
                w, h = df.get_width(), df.get_height()
                print(f"[스트림] depth {w}x{h} {df.get_format()} / "
                      f"color {cf.get_width()}x{cf.get_height()} {cf.get_format()}")
                d = np.frombuffer(df.get_data(), dtype=np.uint16).reshape(h, w)
                print(f"  중앙 픽셀 거리(mm) ~ {int(d[h // 2, w // 2])} "
                      f"(depth_scale {df.get_depth_scale():.3f} -> 값*scale=mm)")
                got = True
                break
        pipe.stop()
        print("[OK] 뎁스 카메라 정상 스트리밍" if got else "[경고] 프레임 미수신(거리/조명/USB 확인)")
    except Exception as e:
        print("[스트림 오류]", type(e).__name__, e)


if __name__ == "__main__":
    main()
