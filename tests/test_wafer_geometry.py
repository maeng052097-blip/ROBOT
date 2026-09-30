"""tests/test_wafer_geometry.py — CELL_V9 충돌/이송/S4/여유 기하 단위테스트(하드웨어 불필요).

  - 정상 수치: 슬롯 1·10 픽업, 크래들 안착/재픽업이 정확히 한 번씩 일어난다
  - 충돌: 아래 슬롯이 남은 채 위 슬롯 픽업, 픽업 프리셋 혼용(Sn-66 + 7.5), RX 62(허브), 전진 상태 크래들 아래 하강
  - S4: 픽업 전 카세트 웨이퍼 아래에서도 ON, 빈 슬롯 리프트 후 OFF, 크래들 안착 후 OFF(7mm)
  - 여유: 삽입 0.5 / 윗 선반 1.5 / 상단 레일 1.5 / 재픽업 0.5 / 선회 7.5 / 허브 8

실행: py -3.13 tests/test_wafer_geometry.py
"""
import sys
import pathlib

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))


def main():
    from wafer_cell import cell_config as C
    from wafer_cell import geometry as G

    print("test_wafer_geometry:")
    ext = C.RX_EXT
    for n in (1, 10):
        sn = C.slot_z(n)
        occ = G.Occupancy(slots={k: True for k in range(n, 11)})
        r = G.z_move(0.0, 0.0, C.ZC_SWING, sn - 66, occ)
        assert r["collision"] is None and r["events"] == [], r
        assert G.r_move(0.0, sn - 66, 0.0, ext, occ) is None
        r = G.z_move(0.0, ext, sn - 66, sn - 61, occ)
        assert r["collision"] is None and r["events"] == [("pick", "slot", n)], r
    print("  OK pick slot 1 and 10 (one event each)")

    occ = G.Occupancy(blade=True)
    r = G.z_move(90.0, 0.0, 105, 192, occ)
    assert r["collision"] is None and r["events"] == []
    assert G.r_move(90.0, 192, 0, ext, occ) is None
    r = G.z_move(90.0, ext, 192, 181, occ)
    assert r["events"] == [("place", "C90", 0)] and r["collision"] is None
    occ = G.Occupancy(C90=True)
    assert G.r_move(90.0, 181, ext, 0, occ) is None
    assert G.z_move(90.0, 0.0, 181, 105, occ)["collision"] is None
    assert G.z_move(90.0, 0.0, 105, 181, occ)["collision"] is None
    assert G.r_move(90.0, 181, 0, ext, occ) is None
    r = G.z_move(90.0, ext, 181, 187, occ)
    assert r["events"] == [("pick", "C90", 0)] and r["collision"] is None
    print("  OK cradle place (step 9) and re-pick (step 13)")

    occ = G.Occupancy(slots={2: True, 3: True})
    r = G.z_move(0.0, 0.0, C.ZC_SWING, C.slot_z(3) - 66, occ)
    assert r["collision"] and "슬롯 2" in r["collision"], r
    print("  OK out-of-order (slot 2 still loaded while going to slot 3) -> collision")

    sn = C.slot_z(4)
    occ = G.Occupancy(slots={4: True, 5: True})
    r = G.z_move(0.0, ext, sn - 66, sn - 66 + 7.5, occ)
    assert r["collision"], r
    print("  OK mixed pick preset (Sn-66 with lift 7.5) -> ledge collision")

    old = C.RX_EXT
    try:
        C.RX_EXT = C.RX_STROKE_MEASURED
        occ = G.Occupancy(blade=True)
        assert G.r_move(180.0, 192, 0, C.RX_EXT, occ) is None
        r = G.z_move(180.0, C.RX_EXT, 192, 181, occ)
        assert r["collision"] and "허브" in r["collision"], r
    finally:
        C.RX_EXT = old
    print("  OK RX 62 (measured stroke) -> hub collision at step 19")

    occ = G.Occupancy()
    assert G.z_move(90.0, ext, 181, 150, occ)["collision"]
    assert G.z_move(45.0, 0.0, 105, 150, occ)["collision"]
    assert G.z_move(0.0, 20.0, 150, 160, occ)["collision"]
    print("  OK extended below cradle / between stations / RX mid -> rejected")

    sn = C.slot_z(3)
    occ = G.Occupancy(slots={3: True, 4: True})
    assert G.s4_on(0.0, sn - 66, ext, occ) is True
    occ = G.Occupancy(slots={4: True})
    assert G.s4_on(0.0, sn - 61, ext, occ) is False
    assert G.s4_on(90.0, 181, ext, G.Occupancy(C90=True)) is False
    assert G.s4_on(90.0, 181, 0.0, G.Occupancy(blade=True)) is True
    assert G.s4_on(0.0, sn - 66, 0.0, G.Occupancy(slots={3: True})) is False
    print("  OK S4 geometry (under cassette wafer ON, empty-slot lift OFF, after place OFF)")

    cl = {c["key"]: c["mm"] for c in G.clearances()}
    assert cl == {"insert_gap": 0.5, "ledge_above": 1.5, "top_rail": 1.5,
                  "cradle_repick": 0.5, "swing": 7.5, "hub": 8.0}, cl
    cl2 = {c["key"]: c["mm"] for c in G.clearances("centered")}
    assert cl2["insert_gap"] == 3.0 and cl2["ledge_above"] == 1.5
    print("  OK clearances", cl)
    print("OK (all passed)")


if __name__ == "__main__":
    main()
