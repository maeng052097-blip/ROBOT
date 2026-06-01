"""재활용 쓰레기 7개 클래스 단일 정의.

YOLO 클래스 id <-> 영문명. data.yaml / 변환·검증 스크립트 / 추론이 공유한다.
이전에는 data.yaml 과 verify_labels.py 등에 중복 정의되어 있었다.
"""

# 클래스 id -> 영문명
CLASS_NAMES = {
    0: "vinyl",
    1: "styrofoam",
    2: "glass_bottle",
    3: "paper",
    4: "can",
    5: "pet_bottle",
    6: "plastic",
}

# AIHUB ZIP 파일명 한글 키워드 -> 클래스 id. 데이터 변환에 사용.
CATEGORY_KEYWORDS = {
    "비닐": 0,
    "스티로폼": 1,
    "유리병": 2,
    "종이류": 3,
    "캔류": 4,
    "페트병": 5,
    "플라스틱류": 6,
}

NUM_CLASSES = len(CLASS_NAMES)
