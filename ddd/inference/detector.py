"""YOLO 기반 목표 탐지 래퍼.

학습된 YOLOv8 모델로 웹캠 프레임에서 재활용 쓰레기를 탐지하고,
목표가 화면의 좌/중/우 어디에 있는지(LEFT/CENTER/RIGHT/NONE)를 돌려준다.

ultralytics(torch)는 무거우므로 모델을 만들 때만 임포트한다.
그래서 이 모듈 자체는 ultralytics 없이도 import 할 수 있다(기하 로직 테스트 용이).
"""
import pathlib

from common.config import WEIGHTS_PATH, CONF_THRESHOLD
from common.classes import CLASS_NAMES

# 목표 방향 (컨트롤러의 CAMERA_* 값과 동일한 문자열)
DIR_LEFT, DIR_CENTER, DIR_RIGHT, DIR_NONE = "LEFT", "CENTER", "RIGHT", "NONE"


def _direction_from_center_x(cx, width):
    """박스 중심 x 와 프레임 폭으로 좌/중/우를 판단한다.

    화면을 세로로 3등분: 왼쪽 1/3 -> LEFT, 오른쪽 1/3 -> RIGHT, 가운데 -> CENTER.
    """
    if width <= 0:
        return DIR_NONE
    if cx < width / 3.0:
        return DIR_LEFT
    if cx > width * 2.0 / 3.0:
        return DIR_RIGHT
    return DIR_CENTER


class TargetDetector:
    """YOLO 모델을 1회 로드해 두고, 프레임마다 목표 방향을 판단한다."""

    def __init__(self, weights_path=WEIGHTS_PATH, conf=CONF_THRESHOLD):
        weights_path = pathlib.Path(weights_path)
        if not weights_path.exists():
            raise FileNotFoundError(
                f"YOLO 가중치를 찾을 수 없습니다: {weights_path}\n"
                f"  학습된 best.pt 를 위 경로에 두거나 train.py 로 재학습하세요."
            )
        from ultralytics import YOLO  # 무거운 의존성 -> 지연 임포트
        self.model = YOLO(str(weights_path))
        self.conf = conf
        self.last = None  # 최근 탐지(디버그용): (label, confidence, direction)

    def detect_direction(self, frame):
        """프레임에서 가장 신뢰도 높은 목표의 방향을 반환. 목표 없으면 NONE.

        타겟 선택 정책: 신뢰도 최고 박스(클래스 무관). 필요 시 클래스 필터 추가 가능.
        """
        results = self.model(frame, conf=self.conf, verbose=False)
        if not results:
            self.last = None
            return DIR_NONE

        boxes = results[0].boxes
        if boxes is None or len(boxes) == 0:
            self.last = None
            return DIR_NONE

        # 신뢰도 최고 박스 선택
        confs = boxes.conf.tolist()
        best_i = max(range(len(confs)), key=lambda i: confs[i])
        x1, _y1, x2, _y2 = boxes.xyxy[best_i].tolist()
        cx = (x1 + x2) / 2.0
        width = frame.shape[1]
        direction = _direction_from_center_x(cx, width)

        cls_id = int(boxes.cls[best_i].item())
        label = CLASS_NAMES.get(cls_id, str(cls_id))
        self.last = (label, confs[best_i], direction)
        return direction
