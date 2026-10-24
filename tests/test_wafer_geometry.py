"""tests/test_wafer_geometry.py — R22 충돌 · 이송 이벤트 · 센서 기하(순수 로직, 하드웨어 불필요).

  - ① (도크 +22 mm): 후진 빈 포크는 가득 찬 ① 앞에서 오르내려도 안전, 웨이퍼를 든 채 후진 Z 이동은 충돌
    -> 옆길(θ220)에서는 든 채 오르내림 가능, 자리 사이에서 R 전진/운반은 금지
  - ① 슬롯 집기(Sn 교차) · 옆길 높이에서 원래 슬롯에 놓기, 윗레일 · 클램프 윗판 · 받침 이빨
  - ③ 위 슬롯부터 채움(아래부터 채우면 후진 포크가 웨이퍼를 긁음)
  - UV 탑: ZC 121.5 초과 금지, RX > 2 에서 113.5~121.5 밖 금지, 받침(183)에 넘김/다시 집기
  - 카세트: 대차 받침 223 / ① 도크 235 교차에서 옮김, 찬 받침 · 찬 도크에 놓기 금지, 소프트 상한 245
  - S4 · θ 자리 마스크(동시에 둘 LOW 없음) · 여유(mm)

실행: py -3.13 tests/test_wafer_geometry.py
"""
import sys
import pathlib

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))


def main():
    from wafer_cell import cell_config as C
    from wafer_cell import geometry as G

    print("test_wafer_geometry:")
    full = {k: True for k in range(1, 6)}
    I, NG, IN, UV, CART, SIDE = 120.0, 60.0, 180.0, 0.0, 270.0, C.SIDE_THETA
    RI, RE, RU = C.STATION_RX["input"], C.STATION_RX["ng"], C.STATION_RX["uv"]
    ret = C.RX_RET

    o = G.Occupancy(cass={"input": dict(full), "ng": {}})
    assert G.z_move(IN, ret, 105, 245, o)["collision"] is None
    assert G.z_move(IN, ret, 105, 228.5, o)["collision"] is None
    oc = G.Occupancy(cass={"input": {1: True, 3: True}, "ng": {}}, blade=True)
    col = G.z_move(IN, ret, 105, C.side_z(4), oc)["collision"]
    assert col and col[0] == "CARRIED_WAFER" and col[1] == "input", col
    assert G.z_move(SIDE, ret, 105, C.side_z(4), oc)["collision"] is None
    assert G.z_move(150, ret, 140, 105, G.Occupancy())["collision"] is None
    assert G.z_move(150, ret, 105, 140, G.Occupancy(blade=True))["collision"][0] == "BETWEEN"
    assert G.z_move(70, ret, 105, 140, G.Occupancy())["collision"][0] == "BETWEEN"     # ③ 가까이
    assert G.r_move(150, 105, ret, 30, G.Occupancy())[0] == "BETWEEN_R"
    assert G.r_move(SIDE, 105, ret, 30, G.Occupancy())[0] == "BETWEEN_R"
    assert G.r_move(150, 105, 30, ret, G.Occupancy()) is None                        # 후진은 언제나
    assert G.z_move(IN, 30, 105, 106, o)["collision"][0] == "R_MID"
    print("  OK input dock +22: retracted empty fork free, carrying needs side path, nothing between stations")

    o = G.Occupancy(cass={"input": {2: True, 3: True}, "ng": {}})
    r = G.z_move(IN, RI, C.pick_z(2), C.side_z(2), o)
    assert r["collision"] is None and [e[:3] for e in r["events"]] == [("pick", "input", 2)]
    assert abs(r["events"][0][3] - (C.slot_z(2) - C.BLADE_PLANE_OFS)) < 1e-9
    assert G.r_move(IN, C.pick_z(2), ret, RI, o) is None
    ob = G.Occupancy(cass={"input": {3: True}, "ng": {}}, blade=True)
    assert G.r_move(IN, C.side_z(2), ret, RI, ob) is None                             # 옆길 높이로 원래 슬롯에 넣기
    r = G.z_move(IN, RI, C.side_z(2), C.pick_z(2), ob)
    assert r["collision"] is None and [e[:3] for e in r["events"]] == [("place", "input", 2)]
    assert G.r_move(IN, C.pick_z(2) + 2, ret, RI, ob)[0] == "CARRIED_LEDGE"            # 높이 틀리면 받침 이빨
    assert G.r_move(IN, C.pick_z(3), ret, RI, G.Occupancy(cass={"input": {3: True}}, blade=True))
    oe = G.Occupancy(cass={"input": {}, "ng": {}})
    assert G.z_move(IN, RI, C.pick_z(5), 230, oe)["collision"][0] == "TOP_RAIL"
    assert G.r_move(IN, 235, ret, RI, G.Occupancy(cass={"input": dict(full)}))[0] == "PLATE"
    print("  OK input pick at Sn, return to original slot from side height, ledge / top rail / clamp plate")

    o = G.Occupancy(cass={"ng": {4: True, 5: True}}, blade=True)
    assert G.z_move(NG, ret, 105, C.place_z(3), o)["collision"] is None
    assert G.r_move(NG, C.place_z(3), ret, RE, o) is None
    r = G.z_move(NG, RE, C.place_z(3), C.pick_z(3), o)
    assert [e[:3] for e in r["events"]] == [("place", "ng", 3)]
    o = G.Occupancy(cass={"ng": {1: True}}, blade=True)
    col = G.z_move(NG, ret, 105, C.place_z(2), o)["collision"]
    assert col and col[0] in ("CARRIED_WAFER", "FORK_WAFER") and col[2] == 1, col
    print("  OK NG fills top slot first (bottom-first would hit slot 1 with the retracted fork)")

    assert G.z_move(UV, ret, 105, 121.6, G.Occupancy())["collision"][0] == "UV_TOWER"
    assert G.z_move(UV, RU, 121.5, 113.0, G.Occupancy())["collision"][0] == "UV_ZONE"
    assert G.r_move(UV, 105, ret, RU, G.Occupancy())[0] == "UV_ZONE"
    assert G.r_move(UV, C.UV_ZC_IN, ret, RU, G.Occupancy(blade=True)) is None
    r = G.z_move(UV, RU, C.UV_ZC_IN, C.UV_ZC_HAND, G.Occupancy(blade=True))
    assert [e[:3] for e in r["events"]] == [("place", "uv", 0)]
    assert G.r_move(UV, C.UV_ZC_HAND, ret, RU, G.Occupancy(uvh=True)) is None
    assert G.r_move(UV, C.UV_ZC_IN, ret, RU, G.Occupancy(uvh=True))[0] == "FORK_WAFER"
    r = G.z_move(UV, RU, C.UV_ZC_HAND, C.UV_ZC_IN, G.Occupancy(uvh=True))
    assert [e[:3] for e in r["events"]] == [("pick", "uv", 0)]
    print("  OK UV tower: ZC <= 121.5, under tower 113.5..121.5 only, hand-over / re-pick at seat 183")

    assert [e[:3] for e in G.z_move(CART, C.STATION_RX["cart"], C.CASS_Z_SEAT, C.CASS_Z_CARRY,
                                     G.Occupancy(cart_seat="full"))["events"]] == [("cass_pick", "cart", 0)]
    assert [e[:3] for e in G.z_move(CART, C.STATION_RX["cart"], C.CASS_Z_CARRY, C.CASS_Z_SEAT,
                                     G.Occupancy(cart_seat="free", fork_cassette=True))["events"]] == [("cass_place", "cart", 0)]
    assert G.z_move(CART, 85, 241, 205, G.Occupancy(cart_seat="full", fork_cassette=True))["collision"][0] == "SEAT_FULL"
    assert G.r_move(CART, 241, ret, 85, G.Occupancy(fork_cassette=True))[0] == "NO_SEAT"
    assert G.r_move(CART, 205, ret, 85, G.Occupancy()) == ("NO_SEAT", "cart", None)
    oi = G.Occupancy(cass={"ng": {}}, fork_cassette=True)
    assert G.r_move(IN, 241, ret, RI, oi) is None
    assert [e[:3] for e in G.z_move(IN, RI, 241, C.CASS_Z_UNDER, oi)["events"]] == [("cass_place", "input", 0)]
    assert G.z_move(IN, RI, 241, 228.5, G.Occupancy(cass={"input": {}}, fork_cassette=True))["collision"][0] == "DOCK_OCCUPIED"
    assert G.z_move(IN, ret, 241, 228.5, oi)["collision"][0] == "CASS_Z"
    assert [e[:3] for e in G.z_move(IN, RI, C.CASS_Z_UNDER, C.CASS_Z_CARRY,
                                     G.Occupancy(cass={"input": dict(full)}))["events"]] == [("cass_pick", "input", 0)]
    assert G.z_move(IN, ret, 105, 246, G.Occupancy())["collision"][0] == "Z_RANGE"
    print("  OK cassette moves at cart 223 / dock 235, full seat / occupied dock refused, soft max 245")

    oi = G.Occupancy(cass={"input": {3: True}})
    assert not G.s4_on(IN, C.side_z(2), RI, oi)                        # 빈 슬롯 2 들기 위치에서 슬롯 3 은 11 mm 위
    assert G.s4_on(IN, C.side_z(2), RI, G.Occupancy(blade=True))
    assert not G.s4_on(I, C.CRADLE_ZC_SEAT, ret, G.Occupancy(inspect=True))      # 놓고 뺀 뒤
    assert G.s4_on(I, C.CRADLE_ZC_SEAT, C.STATION_RX["inspect"], G.Occupancy(inspect=True))   # 놓은 직후는 아직 보임
    assert not G.s4_on(UV, C.UV_ZC_HAND, ret, G.Occupancy(uvh=True))
    print("  OK S4: on with wafer on fork, off after retract, empty-slot lift does not see the slot above")

    assert G.th_mask(0) == [True, False, False, False, False]
    assert G.th_mask(1.6)[0] and not any(G.th_mask(1.7))
    assert G.th_mask(270)[4] and G.th_mask(-90)[4] and G.th_mask(-91.6)[4] and not any(G.th_mask(-91.7))
    worst = max(sum(G.th_mask(x / 20.0)) for x in range(-1000, 7300))
    assert worst == 1
    print("  OK theta station mask: +-1.65 deg windows, 270 == -90, never two LOW")

    cl = {c["key"]: c["mm"] for c in G.clearances()}
    assert cl["insert_gap"] == 3.0 and cl["side_ledge"] == 3.5 and cl["input_tip"] == 2.5 and cl["uv_hand"] == 4.0
    assert cl["cradle_above"] == 5.5 and cl["swing"] == 7.5
    print("  OK clearances " + ", ".join(f"{k} {v}" for k, v in cl.items()))
    print("OK (all passed)")


if __name__ == "__main__":
    main()
