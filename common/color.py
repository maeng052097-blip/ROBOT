"""common/color.py — 이미지 영역의 '대표 색' 추정 (HSV 기반).

YOLO 는 '무엇/어디'(객체 탐지)를 하지만 '무슨 색'은 픽셀에서 직접 읽는 게 정확하고
즉시 가능하다(재학습 불필요, 조명 적응적). HSV 로 채도(S)/명도(V)를 보고:
  - 채색 픽셀 비율이 낮으면  -> 명도로 black/white/gray
  - 그 외                    -> 채색 픽셀의 색상(Hue) 최빈값으로 색 이름
반환: (색이름, 대표 BGR 튜플).  cv2/numpy 는 함수 안에서 임포트(모듈 임포트 가벼움).
"""

# OpenCV Hue(0~179) 경계와 이름 (digitize 인덱스 0~8)
_HUE_BINS = (10, 22, 33, 78, 96, 131, 160, 170)
_HUE_NAMES = ("red", "orange", "yellow", "green", "cyan", "blue", "purple", "pink", "red")

# 채도(S)/명도(V) 임계값은 '용도'에 따라 의도적으로 다르다(억지로 통일하면 한쪽이 나빠짐):
#  - 분류(dominant_color): 이미 고른 crop 의 대표색 -> 느슨(낮은 채도도 색 인정).
#  - 검출(color_mask): 장면에서 '선명한 색 블롭'만 -> 엄격(배경/노이즈 거부).
CLASSIFY_SAT_MIN, CLASSIFY_VAL_MIN = 50, 40
DETECT_SAT_MIN, DETECT_VAL_MIN = 90, 70


def dominant_color(bgr, sat_min=CLASSIFY_SAT_MIN, val_min=CLASSIFY_VAL_MIN, chroma_frac=0.20):
    """BGR 영역의 대표 색 (이름, BGR). 빈 입력이면 ('unknown', 회색)."""
    import cv2
    import numpy as np

    if bgr is None or getattr(bgr, "size", 0) == 0:
        return "unknown", (128, 128, 128)

    img = cv2.resize(bgr, (48, 48), interpolation=cv2.INTER_AREA)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    h = hsv[:, :, 0].reshape(-1).astype(np.int32)
    s = hsv[:, :, 1].reshape(-1)
    v = hsv[:, :, 2].reshape(-1)
    flat = img.reshape(-1, 3)

    chroma = (s >= sat_min) & (v >= val_min)
    if float(chroma.mean()) < chroma_frac:
        vm = float(v.mean())
        name = "black" if vm < 60 else ("white" if vm > 200 else "gray")
        rep = flat.mean(axis=0)
        return name, (int(rep[0]), int(rep[1]), int(rep[2]))

    names = np.array(_HUE_NAMES)
    labels = names[np.digitize(h[chroma], np.array(_HUE_BINS))]
    vals, cnts = np.unique(labels, return_counts=True)
    name = str(vals[int(cnts.argmax())])
    sel = flat[chroma][labels == name]
    rep = sel.mean(axis=0) if len(sel) else flat[chroma].mean(axis=0)
    return name, (int(rep[0]), int(rep[1]), int(rep[2]))


# 색 이름 -> Hue 범위(들). red 는 0근처/180근처로 갈라진다.
HSV_RANGES = {
    "red":    [(0, 10), (170, 179)],
    "orange": [(10, 22)],
    "yellow": [(22, 33)],
    "green":  [(33, 78)],
    "cyan":   [(78, 96)],
    "blue":   [(96, 131)],
    "purple": [(131, 160)],
    "pink":   [(160, 170)],
}


def color_mask(img, name, sat_min=DETECT_SAT_MIN, val_min=DETECT_VAL_MIN, is_hsv=False):
    """name 색상의 이진 마스크(uint8). img 는 BGR(기본) 또는 HSV(is_hsv=True)."""
    import cv2
    import numpy as np
    hsv = img if is_hsv else cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = np.zeros(hsv.shape[:2], np.uint8)
    for lo, hi in HSV_RANGES.get(name, []):
        m = cv2.inRange(hsv, np.array([lo, sat_min, val_min], np.uint8),
                        np.array([hi, 255, 255], np.uint8))
        mask = cv2.bitwise_or(mask, m)
    return mask
