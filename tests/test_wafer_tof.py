"""tests/test_wafer_tof.py — 카세트 TOF 판정 단위테스트(하드웨어 불필요).

  - 슬롯 1~10 기대 거리(±3mm 오차)는 그 슬롯으로 판정
  - 빈 카세트 판정이 먼저: 로봇이 빔에 들어와도(d=201) 슬롯으로 오판하지 않음
  - 빈 카세트 기준면이 선반(z190)이면 슬롯1과 구분 불가 -> AMBIG01 (보정값 d_empty 반영 시)
  - 너무 가깝거나 잔차 큰 값은 INVALID

실행: py -3.13 tests/test_wafer_tof.py
"""
import sys
import pathlib

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))


def main():
    from wafer_cell import tof as T

    print("test_wafer_tof:")
    d1, de = T.default_refs()
    assert (d1, de) == (168.0, 180.0), (d1, de)
    for n in range(1, 11):
        d = T.expected_distance(n)
        for err in (-3.0, 0.0, 3.0):
            r = T.classify(d + err)
            assert r["kind"] == "SLOT" and r["n"] == n, (n, err, r)
    print("  OK slots 1..10 within +/-3mm")

    assert T.classify(T.expected_distance(0))["kind"] == "EMPTY"
    assert T.classify(201.0)["kind"] == "EMPTY"
    assert T.classify(176.5)["kind"] == "EMPTY"
    assert T.classify(174.0)["kind"] == "AMBIG01"
    print("  OK empty checked first (d=180, 201 -> EMPTY), 174 -> AMBIG01")

    d_empty_ledge = T.expected_distance(0, floor_z=190.0)
    assert d_empty_ledge == 170.0
    assert T.classify(170.0)["kind"] == "SLOT"          # 보정 없이: 빈 카세트를 1장으로 오판(위험 시연)
    r = T.classify(170.0, d_empty=d_empty_ledge)
    assert r["kind"] == "AMBIG01", r                    # 보정값 반영: 구분 불가로 정직하게 표시
    assert T.classify(156.0, d_empty=d_empty_ledge)["n"] == 2
    print("  OK ledge in beam: calibrated -> AMBIG01 (not a fake count)")

    assert T.classify(20.0)["kind"] == "INVALID"
    assert T.classify(None)["kind"] == "INVALID"
    assert T.classify(150.0)["kind"] == "INVALID"         # 잔차 6mm
    assert T.classify(40.0)["kind"] == "INVALID"          # 슬롯 11+ 위치
    print("  OK invalid readings")
    print("OK (all passed)")


if __name__ == "__main__":
    main()
