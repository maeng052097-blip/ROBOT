"""tests/test_wafer_tof.py — 도크 ToF(VL53L0X x2) 판정(순수 로직, 하드웨어 불필요).

R22 문서 §4: ToF 는 카세트 있음 · 없음만 본다(웨이퍼 개수는 세지 않음).
  - 문턱(TOF_PRESENT_MM, 가정) 안쪽 = 있음, 범위 밖 코드(8190) · 먼 값 = 없음, 읽기 실패(None) = 판정 불가
  - 시뮬 기본 거리(TOF_CASS_D ± 잡음)는 문턱 안쪽

실행: py -3.13 tests/test_wafer_tof.py
"""
import sys
import pathlib

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))


def main():
    from wafer_cell import cell_config as C
    from wafer_cell import tof as T

    print("test_wafer_tof:")
    assert T.presence(None) == "invalid"
    assert T.presence(40.0) == "present" and T.presence(C.TOF_PRESENT_MM - 0.1) == "present"
    assert T.presence(C.TOF_PRESENT_MM) == "absent" and T.presence(500.0) == "absent"
    assert T.presence(C.TOF_OUT_OF_RANGE) == "absent" and T.presence(9000) == "absent"
    assert C.TOF_CASS_D + C.TOF_NOISE < C.TOF_PRESENT_MM
    print(f"  OK presence: < {C.TOF_PRESENT_MM:g} mm present, out-of-range/far absent, None invalid")
    print("OK (all passed)")


if __name__ == "__main__":
    main()
