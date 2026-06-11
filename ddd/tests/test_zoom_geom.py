"""tests/test_zoom_geom.py — 카메라 디지털 줌의 방위/거리 불변성 증명(하드웨어 불필요).

핵심: 같은 물체는 줌 배율과 무관하게 '같은 카메라 베어링' -> 같은 LiDAR 방향 -> 같은 거리.
(카메라-LiDAR 동일선상=시차0 이면 이 베어링이 곧 LiDAR 방향이므로 거리가 정확.)
실행: py -3.13 tests/test_zoom_geom.py
"""
import sys
import math
import pathlib

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))

from common.fusion import view_bearing_deg  # noqa: E402


def approx(a, b, t=1e-6):
    return abs(a - b) <= t


def main():
    print("test_zoom_geom:")
    # 핀홀 모델: 실제 bearing=beta 인 물체의 풀프레임 픽셀을 초점거리로 역산.
    W, hfov, beta = 1280.0, 70.0, 5.0   # beta < eff_half_fov(4x=8.75) 라야 4x 화면 안
    f = (W / 2.0) / math.tan(math.radians(hfov / 2.0))
    cx_full = W / 2.0 + f * math.tan(math.radians(beta))
    assert approx(view_bearing_deg(cx_full, W, hfov, 1.0), beta, 1e-4), "1x"

    # 2x/3x/4x 줌: 중앙 크롭(폭 W/z, 오프셋 (W-W/z)/2)에서도 같은 물체 -> 같은 bearing
    for z in (2.0, 3.0, 4.0):
        vw = W / z
        cx_view = cx_full - (W - vw) / 2.0
        assert approx(view_bearing_deg(cx_view, vw, hfov, z), beta, 1e-4), f"{z}x bearing"
    print("  OK bearing zoom-invariant (pinhole, 5deg @ 1x/2x/3x/4x)")

    # 뷰 중앙은 어떤 줌에서도 정면(0deg)
    assert view_bearing_deg(W / 2, W, hfov, 1.0) == 0.0
    assert view_bearing_deg(320, 640, hfov, 5.0) == 0.0
    print("  OK view center = 0deg (front) at any zoom")

    # 시차(오프셋 카메라) 보정: 정면 1m 점을 오른쪽(+100mm) 카메라는 좌측으로 본다.
    from common.fusion import distance_along_ray
    dd = {0: 1000}
    beta = math.degrees(math.atan2(-100, 1000))   # 카메라가 보는 방위(-5.71deg)
    r, a = distance_along_ray(dd, 100.0, 0.0, beta, 0.0, 150.0)
    assert r == 1000 and a == 0, (r, a)
    # 시차 무시(bearing 0)면 시선이 점을 안 지나 -> None
    r2, _ = distance_along_ray(dd, 100.0, 0.0, 0.0, 0.0, 50.0)
    assert r2 is None, r2
    print("  OK parallax ray fusion (offset camera)")

    # C7 결함 수정 증명: 시선(정면 0도) 위 점(1000)과, 옆(15도)이지만 더 가까운
    # 클러터(600)가 함께 있을 때 -> '정렬된' 1000을 골라야 한다.
    # (옛 코드는 '가장 가까운 t'를 골라 클러터 600을 자신있게 반환했음)
    dd_clutter = {0: 1000, 15: 600}
    rc, ac = distance_along_ray(dd_clutter, 0.0, 0.0, 0.0, 0.0, 200.0)
    assert (rc, ac) == (1000, 0), (rc, ac)
    print("  OK ray fusion rejects off-axis clutter (1000 not 600)")

    # 카메라 디지털 줌 크롭(공용 helper) 모양 검증
    import numpy as np
    from common.camera import crop_center_zoom
    img = np.zeros((100, 200, 3), np.uint8)
    assert crop_center_zoom(img, 1.0).shape == (100, 200, 3)
    assert crop_center_zoom(img, 2.0).shape == (50, 100, 3)
    assert crop_center_zoom(img, 4.0).shape == (25, 50, 3)
    print("  OK crop_center_zoom shapes (1x/2x/4x)")

    # 세로 앵커: 0.0=상단 유지, 1.0=하단 유지(가까운/낮은 물체용), 가로는 항상 중앙.
    # 픽셀값 = 행 번호인 이미지를 만들어 어느 행이 보존되는지 '내용'으로 검증.
    rows = np.repeat(np.arange(100, dtype=np.uint8).reshape(100, 1, 1), 200, axis=1)
    rows = np.repeat(rows, 3, axis=2)                      # (100, 200, 3), 값=행번호
    v_top = crop_center_zoom(rows, 2.0, 0.0)
    v_mid = crop_center_zoom(rows, 2.0, 0.5)
    v_bot = crop_center_zoom(rows, 2.0, 1.0)
    assert v_top.shape == (50, 100, 3) and v_bot.shape == (50, 100, 3)
    assert v_top[0, 0, 0] == 0 and v_top[-1, 0, 0] == 49      # 상단 0~49 보존
    assert v_mid[0, 0, 0] == 25 and v_mid[-1, 0, 0] == 74     # 중앙(기존과 동일)
    assert v_bot[0, 0, 0] == 50 and v_bot[-1, 0, 0] == 99     # 하단 50~99 보존(근거리용)
    print("  OK crop anchor_y keeps top/center/bottom rows (0.0/0.5/1.0)")

    # 수동 포커스 클램프: UVC 규약(0~250, 5의 배수)
    from common.camera import clamp_focus
    assert clamp_focus(-10) == 0 and clamp_focus(0) == 0
    assert clamp_focus(7) == 5 and clamp_focus(8) == 10
    assert clamp_focus(250) == 250 and clamp_focus(999) == 250
    print("  OK clamp_focus (0~250, step5)")

    # view_x_from_bearing 은 view_bearing_deg 의 역함수 (왕복 일치) — LiDAR각->카메라픽셀에 사용
    from common.fusion import view_x_from_bearing
    for beta2 in (-20.0, -5.0, 0.0, 7.5, 25.0):
        cxb = view_x_from_bearing(beta2, 1280.0, 70.0, 1.0)
        assert approx(view_bearing_deg(cxb, 1280.0, 70.0, 1.0), beta2, 1e-6), f"roundtrip {beta2}"
    print("  OK view_x_from_bearing roundtrip (inverse of view_bearing_deg)")

    # 유효 반화각: 줌 z 에서 뷰가 덮는 각도 = atan(tan(hfov/2)/z). FOV 게이트는 이걸 써야 함.
    from common.fusion import effective_half_fov_deg
    assert approx(effective_half_fov_deg(70.0, 1.0), 35.0, 1e-9)
    e2 = effective_half_fov_deg(70.0, 2.0)
    assert approx(e2, math.degrees(math.atan(math.tan(math.radians(35.0)) / 2.0)), 1e-9)
    # 뷰 가장자리(cx=view_w)의 베어링 == 유효 반화각 (정의 일치)
    assert approx(view_bearing_deg(1280.0, 1280.0, 70.0, 2.0), e2, 1e-9)
    # 줌 상태 왕복: 유효 FOV 안의 베어링은 픽셀로 갔다가 정확히 돌아온다 (줌 클릭/박스 근거)
    for z in (1.0, 2.0, 4.0):
        for b2 in (-10.0, 0.0, 12.0):
            if abs(b2) < effective_half_fov_deg(70.0, z):
                cxz = view_x_from_bearing(b2, 1280.0, 70.0, z)
                assert approx(view_bearing_deg(cxz, 1280.0, 70.0, z), b2, 1e-6), (z, b2)
    print(f"  OK effective_half_fov (70deg: 1x=35.00, 2x={e2:.2f}) + zoomed roundtrip")

    # 단안(크기 기반) 거리: 9cm 물체@2m 의 픽셀폭을 핀홀로 만들고 역산 -> 2m 복원.
    # 중앙 크롭 줌은 픽셀 밀도 불변 -> 같은 픽셀폭이면 줌 배율과 무관하게 같은 거리.
    from common.fusion import monocular_range_mm
    f_full = 1920.0 / (2.0 * math.tan(math.radians(35.0)))
    wpx = 90.0 * f_full / 2000.0
    assert approx(monocular_range_mm(wpx, 1920.0, 70.0, 1.0, 90.0), 2000.0, 1e-6)
    assert approx(monocular_range_mm(wpx, 960.0, 70.0, 2.0, 90.0), 2000.0, 1e-6)  # 줌 불변
    assert monocular_range_mm(0, 1920.0, 70.0, 1.0, 90.0) is None
    print("  OK monocular_range (9cm@2m roundtrip, zoom-invariant, guard)")
    print("OK (all passed)")


if __name__ == "__main__":
    main()
