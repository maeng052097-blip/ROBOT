"""tests/test_wafer_tof.py — 카세트 하단 TOF(VL53L0X) 판정 단위테스트(하드웨어 불필요).

  - 기대 거리: 슬롯 1~5 = Sn - TOF_Z, 빈 카세트 = 윗레일 밑면 - TOF_Z
  - 판정 경계: 기대값 ±TOF_TOL 은 그 슬롯, 더 벗어나면 판정 불가, 범위 밖(8190) = 카세트 없음
  - 개수 추정 = 5 - 가장 아래 슬롯 + 1 (위로 빈칸 없이 붙어 있음 가정)
  - 장착 기하: 슬롯 1 거리 >= VL53L0X 최소 거리, 슬롯 간 / 빈 카세트 구분 가능
  - 가림: 포크가 그 도크 앞(±10°)에서 전진했거나 웨이퍼/카세트를 들고 있으면 측정 무효

실행: py -3.13 tests/test_wafer_tof.py
"""
import sys
import pathlib

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))


def main():
    from wafer_cell import cell_config as C
    from wafer_cell import tof as T

    print("test_wafer_tof:")
    exp = [C.tof_expected(n) for n in range(1, C.SLOT_COUNT + 1)]
    assert exp == [50.0, 68.0, 86.0, 104.0, 122.0], exp
    assert C.tof_empty_d() == 144.0
    print(f"  OK expected distances slots 1..5 = {exp}, empty = {C.tof_empty_d():g}")

    for n, d in enumerate(exp, start=1):
        for dd in (0.0, C.TOF_TOL, -C.TOF_TOL):
            r = T.classify(d + dd)
            assert r == {"kind": "slot", "slot": n, "count_est": C.SLOT_COUNT - n + 1}, (n, dd, r)
    assert T.classify(86.0 + C.TOF_TOL + 1)["kind"] == "invalid"        # 슬롯 3 과 4 사이
    assert T.classify(C.tof_empty_d())["kind"] == "empty" and T.classify(C.tof_empty_d())["count_est"] == 0
    assert T.classify(C.tof_empty_d() - C.TOF_TOL - 1)["kind"] == "invalid"
    assert T.classify(C.TOF_OUT_OF_RANGE)["kind"] == "none" and T.classify(None)["kind"] == "none"
    assert T.classify(20.0)["kind"] == "invalid"                          # 최소 거리보다 가까움(바닥판 반사 등)
    print("  OK classify: +-tol -> slot, beyond -> invalid, 144 -> empty, 8190/None -> none")

    assert T.classify(exp[0])["count_est"] == 5 and T.classify(exp[-1])["count_est"] == 1
    print("  OK count estimate = 5 - bottom slot + 1 (slot 1 -> 5, slot 5 -> 1)")

    assert T.geometry_problems() == [], T.geometry_problems()
    assert C.tof_expected(1) >= C.TOF_MIN_MM
    print(f"  OK mount geometry: slot 1 at {C.tof_expected(1):g} mm >= min {C.TOF_MIN_MM:g}, pitch/empty separable")

    inp = C.STATIONS["input"]
    assert T.blocked(inp, C.RX_EXT, "input", False)            # 집으러 들어감
    assert T.blocked(inp, 0.0, "input", True)                  # 들고 빠져나옴
    assert not T.blocked(inp, 0.0, "input", False)             # 빈 포크 수축: 포크 끝 R203 < 빔 R235.5
    assert not T.blocked(C.THETA_STANDBY, C.RX_EXT, "input", True)   # 다른 스테이션
    assert T.blocked(inp + 8.0, 0.0, "input", True) and not T.blocked(inp + 12.0, 0.0, "input", True)
    print("  OK blocked: fork extended or carrying within +-10 deg of the dock")
    print("OK (all passed)")


if __name__ == "__main__":
    main()
