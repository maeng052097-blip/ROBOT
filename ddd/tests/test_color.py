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
    print("OK (all passed)")


if __name__ == "__main__":
    main()
