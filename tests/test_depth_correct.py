"""test_depth_correct.py — 깊이 보정 순수 로직(correct_depth / parse_lut) 단위테스트. 하드웨어 불필요.

실행: py -3.13 tests/test_depth_correct.py
(visualization.depth_detect 를 임포트 -> cv2/numpy 만 필요, pyorbbecsdk 는 함수 내부라 불필요.)
"""
import sys
import pathlib

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))
from visualization.depth_detect import correct_depth, parse_lut


def approx(a, b, tol=1e-4):
    assert abs(a - b) <= tol, f"{a} != {b} (tol {tol})"


def test_parse_lut():
    assert parse_lut("") is None
    assert parse_lut(None) is None
    assert parse_lut("   ") is None
    assert parse_lut("296:300") == [(296.0, 300.0)]
    # 정렬됨(측정값 오름차순)
    assert parse_lut("686:700,296:300,493:500") == [(296.0, 300.0), (493.0, 500.0), (686.0, 700.0)]


def test_invalid_depth_zero():
    approx(correct_depth(0, 1.02561, -4.26), 0.0)
    approx(correct_depth(-5, 1.02561, -4.26), 0.0)
    approx(correct_depth(0, lut=[(296.0, 300.0), (686.0, 700.0)]), 0.0)


def test_single_linear():
    # lut 없으면 scale*z+offset
    approx(correct_depth(200, 1.02561, -4.26), 1.02561 * 200 - 4.26)
    approx(correct_depth(686, 1.02561, -4.26), 1.02561 * 686 - 4.26)
    # 기본(무보정)
    approx(correct_depth(500, 1.0, 0.0), 500.0)


def test_lut_single_point():
    # 1점 -> 상수 오프셋(true-meas)만 적용
    lut = parse_lut("296:300")
    approx(correct_depth(296, lut=lut), 300.0)     # 296 + (300-296)
    approx(correct_depth(500, lut=lut), 504.0)     # 500 + 4


def test_lut_passes_through_breakpoints():
    lut = parse_lut("296:300,493:500,686:700")
    approx(correct_depth(296, lut=lut), 300.0)
    approx(correct_depth(493, lut=lut), 500.0)
    approx(correct_depth(686, lut=lut), 700.0)


def test_lut_interpolation():
    lut = [(296.0, 300.0), (493.0, 500.0), (686.0, 700.0)]
    # 296~493 구간 내부 z=400
    (x0, y0), (x1, y1) = lut[0], lut[1]
    expect = y0 + (y1 - y0) * (400 - x0) / (x1 - x0)
    approx(correct_depth(400, lut=lut), expect)
    # 493~686 구간 내부 z=600
    (x0, y0), (x1, y1) = lut[1], lut[2]
    expect = y0 + (y1 - y0) * (600 - x0) / (x1 - x0)
    approx(correct_depth(600, lut=lut), expect)


def test_lut_extrapolation():
    lut = [(296.0, 300.0), (493.0, 500.0), (686.0, 700.0)]
    # 하한(296) 아래 -> 첫 구간 기울기로 외삽
    (x0, y0), (x1, y1) = lut[0], lut[1]
    approx(correct_depth(200, lut=lut), y0 + (y1 - y0) * (200 - x0) / (x1 - x0))
    # 상한(686) 위 -> 끝 구간 기울기로 외삽
    (x0, y0), (x1, y1) = lut[-2], lut[-1]
    approx(correct_depth(900, lut=lut), y0 + (y1 - y0) * (900 - x0) / (x1 - x0))


def test_lut_monotonic():
    # 단조 증가 보정 -> 입력 증가 시 출력도 증가
    lut = [(296.0, 300.0), (493.0, 500.0), (686.0, 700.0)]
    prev = -1.0
    for z in range(150, 1000, 25):
        c = correct_depth(float(z), lut=lut)
        assert c > prev, f"비단조 at z={z}: {c} <= {prev}"
        prev = c


if __name__ == "__main__":
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]
    for fn in fns:
        fn()
        print(f"  OK {fn.__name__}")
    print(f"[PASS] test_depth_correct ({len(fns)} tests)")
