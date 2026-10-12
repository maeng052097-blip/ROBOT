"""tests/test_wafer_geometry.py — R9 충돌 · 이송 이벤트 · S4 기하 단위테스트(하드웨어 불필요).

  - 투입 슬롯 1 집기 이벤트(들기 +7.5), 아래 슬롯에 웨이퍼가 남은 채 위 슬롯을 집으러 R0 로 오르면 충돌
  - OK 슬롯 5 넣기 이벤트, 위 슬롯부터 채우지 않으면(아래 슬롯 점유) 넣고 내려오다 충돌
  - 검사 크래들 놓기/다시 들기, 전진한 채 크래들 아래로 하강 금지
  - 대차: Z241 로 들 때 OK 카세트가 포크로, 받침 없이 R85 내밀기 금지, Z205 로 내릴 때 받침으로
  - S4: 포크 위 웨이퍼 ON, 내려놓은 뒤(슬롯 4.5mm / 크래들 5mm) OFF

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
    o = G.Occupancy(cass={"input": dict(full), "ok": {}, "ng": {}})
    r = G.z_move(180.0, 0.0, 105.0, C.pick_z(1), o)
    assert r["collision"] is None, r
    assert G.r_move(180.0, C.pick_z(1), 0.0, 51.0, o) is None
    r = G.z_move(180.0, 51.0, C.pick_z(1), C.pick_z(1) + C.PICK_LIFT, o)
    assert r["collision"] is None and r["events"] == [("pick", "input", 1)], r
    print("  OK input slot 1 pick event on +7.5 lift")

    r = G.z_move(180.0, 0.0, 105.0, C.pick_z(2), o)          # 슬롯 1 이 남은 채 슬롯 2 로 오름
    assert r["collision"] and "슬롯 1" in r["collision"], r
    o2 = G.Occupancy(cass={"input": {k: True for k in range(2, 6)}, "ok": {}, "ng": {}})
    assert G.z_move(180.0, 0.0, 105.0, C.pick_z(2), o2)["collision"] is None
    print("  OK bottom-first rule: going up past an occupied lower slot collides")

    ob = G.Occupancy(cass={"input": {}, "ok": {}, "ng": {}}, blade=True)
    assert G.z_move(0.0, 0.0, 105.0, C.place_z(5), ob)["collision"] is None
    assert G.r_move(0.0, C.place_z(5), 0.0, 51.0, ob) is None
    r = G.z_move(0.0, 51.0, C.place_z(5), C.pick_z(5), ob)
    assert r["collision"] is None and r["events"] == [("place", "ok", 5)], r
    o_after = G.Occupancy(cass={"input": {}, "ok": {5: True}, "ng": {}})
    assert G.z_move(0.0, 0.0, C.pick_z(5), 105.0, o_after)["collision"] is None
    o_bad = G.Occupancy(cass={"input": {}, "ok": {5: True, 3: True}, "ng": {}})
    r = G.z_move(0.0, 0.0, C.pick_z(5), 105.0, o_bad)
    assert r["collision"] and "슬롯 3" in r["collision"], r
    print("  OK top-first rule: OK slot 5 place event, lower occupied slot would be hit on the way down")

    oi = G.Occupancy(cass={}, blade=True)
    r = G.z_move(120.0, 51.0, C.CRADLE_ZC_ABOVE, C.CRADLE_ZC_SEAT, oi)
    assert r["events"] == [("place", "inspect", 0)], r
    oc = G.Occupancy(cass={}, inspect=True)
    r = G.z_move(120.0, 51.0, C.CRADLE_ZC_SEAT, C.CRADLE_ZC_ABOVE, oc)
    assert r["events"] == [("pick", "inspect", 0)], r
    assert G.z_move(120.0, 51.0, C.CRADLE_ZC_SEAT, 150.0, G.Occupancy())["collision"]
    print("  OK inspection cradle place / re-pick, no descent below the cradle while extended")

    ok_full = G.Occupancy(cass={"input": {}, "ok": dict(full), "ng": {}})
    r = G.z_move(0.0, 51.0, C.CART_Z_UNDER, C.CART_Z_CARRY, ok_full)
    assert r["events"] == [("cass_pick", "ok", 0)], r
    carry = G.Occupancy(cass={"input": {}, "ng": {}}, fork_cassette=True, cart_seat_free=False)
    assert G.r_move(-90.0, C.CART_Z_CARRY, 0.0, 85.0, carry)            # 받침 없음
    carry.cart_seat_free = True
    assert G.r_move(-90.0, C.CART_Z_CARRY, 0.0, 85.0, carry) is None
    r = G.z_move(-90.0, 85.0, C.CART_Z_CARRY, C.CART_Z_SEAT, carry)
    assert r["events"] == [("cass_place", "cart", 0)], r
    assert G.r_move(0.0, C.CART_Z_UNDER, 0.0, 85.0, ok_full)            # R85 는 대차 쪽에서만
    assert G.z_move(-40.0, 51.0, 105.0, 200.0, G.Occupancy())["collision"]
    assert G.z_move(-40.0, 0.0, 105.0, C.CART_Z_UNDER, G.Occupancy())["collision"] is None
    print("  OK cart: cassette lifted at Z241, R85 only with a free seat, placed at Z205")

    assert G.s4_on(180.0, C.pick_z(1) + C.PICK_LIFT, 51.0, G.Occupancy(blade=True))
    assert not G.s4_on(0.0, C.pick_z(5), 51.0, o_after)
    assert not G.s4_on(120.0, C.CRADLE_ZC_SEAT, 51.0, oc)
    print("  OK S4: on with wafer on fork, off after placing (slot 4.5mm, cradle 5mm)")

    cl = {c["key"]: c["mm"] for c in G.clearances()}
    assert cl["insert_gap"] == 3.0 and cl["lift"] == 4.5 and cl["swing"] == 7.5, cl
    print("  OK clearances", cl)
    print("OK (all passed)")


if __name__ == "__main__":
    main()
