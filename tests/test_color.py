"""tests/test_color.py — dominant_color 단위테스트(합성 단색 이미지). numpy/cv2 필요.

실행: py -3.13 tests/test_color.py
"""
import sys
import pathlib

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))


def main():
    import numpy as np
    from common.color import dominant_color

    def solid(b, g, r):
        return np.full((24, 24, 3), (b, g, r), np.uint8)

    cases = {
        "red": solid(0, 0, 255),
        "blue": solid(255, 0, 0),
        "green": solid(0, 255, 0),
        "yellow": solid(0, 255, 255),
        "white": solid(255, 255, 255),
        "black": solid(0, 0, 0),
    }
    print("test_color:")
    for expect, img in cases.items():
        name, bgr = dominant_color(img)
        assert name == expect, f"{expect} 기대했는데 {name} (bgr={bgr})"
        print(f"  OK {expect}")
    assert dominant_color(None)[0] == "unknown"

    from common.color import color_mask
    red = solid(0, 0, 255)
    assert color_mask(red, "red").mean() > 0, "red mask empty"
    assert color_mask(red, "blue").mean() == 0, "blue mask should be empty for red"
    print("  OK color_mask")

    # hue->색이름 매핑 단일 일관성: digitize(_HUE_BINS/_HUE_NAMES) == HSV_RANGES (드리프트 방지)
    from common.color import _HUE_BINS, _HUE_NAMES, HSV_RANGES

    def _name_from_ranges(hv):
        for nm, rr in HSV_RANGES.items():
            for lo, hi in rr:
                if lo <= hv < hi:
                    return nm
        return "red"   # 170~179 끝 -> red
    for hv in range(0, 180):
        dig = _HUE_NAMES[int(np.digitize(hv, np.array(_HUE_BINS)))]
        assert dig == _name_from_ranges(hv), f"hue {hv}: digitize={dig} ranges={_name_from_ranges(hv)}"
    print("  OK hue mapping single-source consistent (0..179)")

    # red hue wrap(175) 단색 -> red
    import cv2
    hsv = np.full((20, 20, 3), (175, 255, 255), np.uint8)
    assert dominant_color(cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR))[0] == "red"
    print("  OK red hue-wrap (175) -> red")

    # '클릭-색 잠금' 근거 회귀: 파란 배경(지배색) 속 작은 빨강 패치 ->
    #   지배색 방식은 blue 를 고르지만(현장 5번 이미지 버그 재현), 잠근 색('red')의
    #   color_mask 는 패치 위치만 정확히 잡는다(잠금이 이기는 시나리오).
    scene = solid(255, 0, 0)                          # 24x24 파랑
    scene = np.repeat(np.repeat(scene, 4, 0), 4, 1)   # 96x96 파랑
    scene[60:84, 36:60] = (0, 0, 255)                 # 24x24 빨강 패치(약 6% 면적)
    assert dominant_color(scene)[0] == "blue", "지배색은 blue(버그 재현 전제)"
    rm = color_mask(scene, "red")
    assert rm[60:84, 36:60].min() > 0, "잠근 red 마스크가 패치를 잡아야 함"
    assert rm[:60, :].max() == 0 and rm[:, :36].max() == 0, "패치 밖(파랑)은 0이어야 함"
    frac = float((rm > 0).mean())
    assert 0.01 <= frac <= 0.10, f"패치 면적비 {frac:.3f} (min-area 0.01 게이트 통과 확인)"
    print("  OK click-color-lock: dominant=blue 장면에서 red 잠금이 패치만 검출")
    print("OK (all passed)")


if __name__ == "__main__":
    main()
