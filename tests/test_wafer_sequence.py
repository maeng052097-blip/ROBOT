"""tests/test_wafer_sequence.py — R9 시퀀스/인터록 단위테스트(하드웨어 불필요).

  - 원점: R 수축 -> Z 최하단 -> Z105 -> θ1 원점 -> 대기 θ120 (θ1 은 반드시 Z 뒤, Z105 에서만)
  - 웨이퍼: 비디오 수치(투입 슬롯 바닥 190+18(n-1), Z = 바닥-68.5, 들기 +7.5, 크래들 Z192/183, 넣기 Z201->193.5)
  - 대차 T1~T13: θ -40 · Z228.5 · θ0 · R51 · Z241 · θ -90 · R85 · Z205
  - 모든 선회는 R0 · 직전 S3 확인 + 지정 높이(Z105, 대차 T3=Z228.5, T8=Z241)에서만

실행: py -3.13 tests/test_wafer_sequence.py
"""
import sys
import pathlib

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))


def walk(steps, zc, rx):
    """동작을 순서대로 따라가며 (단계, 동작, 그 시점 zc, rx, 직전 S3 확인 여부) 를 낸다."""
    s3_ok = rx == 0
    for st in steps:
        for a in st.actions:
            yield st, a, zc, rx, s3_ok
            if a.kind == "z":
                zc = a.value
            elif a.kind == "r":
                rx = a.value
                s3_ok = False
            elif a.kind == "expect" and a.sensor == "S3" and a.expect:
                s3_ok = rx == 0


def moves(steps):
    return [(a.kind, a.value) for st in steps for a in st.actions if a.kind in ("z", "r", "rot")]


def main():
    from wafer_cell import cell_config as C
    from wafer_cell import sequence as Q

    print("test_wafer_sequence:")
    h = Q.homing_steps()
    assert moves(h) == [("r", 0.0), ("z", C.ZC_HOME), ("z", C.ZC_SWING), ("rot", 0.0), ("rot", 120.0)], moves(h)
    print("  OK homing R -> Z bottom -> Z105 -> theta1 origin -> standby 120")

    assert C.slot_z(1) == 190 and C.slot_z(2) == 208 and C.slot_z(5) == 262
    w1 = Q.wafer_steps(1)
    assert [s.key for s in w1] == ["1", "2", "3", "4", "5", "6", "7"]
    assert moves(w1) == [("rot", 180.0), ("z", 121.5), ("r", 51.0), ("z", 129.0), ("r", 0.0), ("z", 105.0),
                         ("rot", 120.0), ("z", 192.0), ("r", 51.0), ("z", 183.0), ("r", 0.0),
                         ("r", 51.0), ("z", 192.0), ("r", 0.0), ("z", 105.0)], moves(w1)
    assert moves(Q.wafer_steps(2))[1] == ("z", 139.5)      # 비디오: 슬롯 바닥 208 - 68.5
    assert any(a.kind == "check_pick" for a in w1[2].actions)
    assert any(a.kind == "capture" for a in w1[5].actions)
    print("  OK wafer pick/inspect numbers match the video (Z121.5 / 139.5, +7.5, Z192 -> 183)")

    s5 = Q.sort_steps("ok", 5)
    assert moves(s5) == [("rot", 0.0), ("z", 201.0), ("r", 51.0), ("z", 193.5), ("r", 0.0), ("z", 105.0),
                         ("rot", 120.0)], moves(s5)
    assert moves(Q.sort_steps("ok", 1))[1:3] == [("z", 129.0), ("r", 51.0)]
    assert moves(Q.sort_steps("ok", 3))[1] == ("z", 165.0)
    assert moves(Q.sort_steps("ng", 5))[0] == ("rot", 60.0)
    assert "OK" in s5[0].title and "NG" in Q.sort_steps("ng", 5)[0].title
    print("  OK sort: OK 0 deg / NG 60 deg, slot 5 Z201 -> 193.5, slot 3 Z165, slot 1 Z129")

    cs = Q.cart_steps("FL", 1)
    assert [s.key for s in cs] == ["T1", "T2", "T3", "T4", "T5", "T7", "T8", "T9", "T10", "T11", "T12", "T13"]
    assert moves(cs) == [("rot", -40.0), ("z", 228.5), ("rot", 0.0), ("r", 51.0), ("z", 241.0), ("r", 0.0),
                         ("rot", -90.0), ("r", 85.0), ("z", 205.0), ("r", 0.0), ("z", 105.0), ("rot", 120.0)], moves(cs)
    print("  OK cart T1..T13 numbers match the video")

    seqs = [("homing", h, C.ZC_SWING, 0.0)]
    for n in (1, 5):
        seqs.append((f"wafer{n}", Q.wafer_steps(n) + Q.sort_steps("ok", 6 - n), 105.0, 0.0))
        seqs.append((f"retry{n}", Q.retry_steps(n), C.pick_z(n) + C.PICK_LIFT, C.RX_EXT))
    seqs.append(("cart", cs, 105.0, 0.0))
    nrot = 0
    for name, steps, zc, rx in seqs:
        for st, a, z, r, s3 in walk(steps, zc, rx):
            if a.kind == "rot":
                nrot += 1
                assert abs(z - a.z_ok) < 1e-9 and r == 0 and s3, (name, st.key, z, r, s3)
                if a.z_ok != C.ZC_SWING:
                    assert st.key in ("T3", "T8"), (name, st.key)
    print(f"  OK all {nrot} rotations at R0 + S3, Z105 except cart T3 (Z228.5) / T8 (Z241)")

    ok, why = Q.can_rotate(150.0, 0.0, True)
    assert not ok and "Z 105" in why
    ok, _ = Q.can_rotate(105.0, 0.0, True)
    assert ok
    ok, why = Q.can_rotate(105.0, 51.0, True)
    assert not ok and "R" in why
    ok, _ = Q.can_rotate(241.0, 0.0, True, z_ok=241.0)
    assert ok
    print("  OK can_rotate interlock")

    cat = Q.step_catalog()
    keys = [c["key"] for c in cat]
    assert len(keys) == len(set(keys)) == 5 + 10 + 12, keys
    print("  OK step catalog", len(keys))
    print("OK (all passed)")


if __name__ == "__main__":
    main()
