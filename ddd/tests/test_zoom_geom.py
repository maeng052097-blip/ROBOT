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
    print("OK (all passed)")


if __name__ == "__main__":
    main()
