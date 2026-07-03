"""
YOLO 라벨 변환 결과 검증 스크립트
- 무작위 10장을 선택하여 바운딩박스를 이미지 위에 그림
- 변환이 정확한지 눈으로 확인
"""
import sys
import pathlib
import cv2
import os
import random
import glob

# repo-root(ddd) 를 import 경로에 추가 -> common 패키지 사용
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2]))
from common.config import CONVERTED_DATA_DIR
from common.classes import CLASS_NAMES

OUTPUT_ROOT = str(CONVERTED_DATA_DIR)

COLORS = {
    0: (255, 100, 100),
    1: (100, 255, 100),
    2: (100, 100, 255),
    3: (255, 255, 100),
    4: (255, 100, 255),
    5: (100, 255, 255),
    6: (200, 200, 100),
}


def verify_labels(num_samples=10):
    """무작위 샘플을 선택하여 바운딩박스 시각화"""
    # train 이미지 목록
    img_dir = os.path.join(OUTPUT_ROOT, "images", "train")
    lbl_dir = os.path.join(OUTPUT_ROOT, "labels", "train")

    images = glob.glob(os.path.join(img_dir, "*.jpg"))
    if not images:
        print("검증할 이미지가 없습니다. 변환 스크립트를 먼저 실행하세요.")
        return

    # 무작위 선택
    random.seed(123)
    selected = random.sample(images, min(num_samples, len(images)))

    verify_dir = os.path.join(OUTPUT_ROOT, "_verify")
    os.makedirs(verify_dir, exist_ok=True)

    print(f"검증 이미지 {len(selected)}장 생성 중...\n")

    for img_path in selected:
        basename = os.path.splitext(os.path.basename(img_path))[0]
        lbl_path = os.path.join(lbl_dir, basename + ".txt")

        img = cv2.imread(img_path)
        if img is None:
            print(f"  이미지 읽기 실패: {img_path}")
            continue

        h, w = img.shape[:2]

        if os.path.exists(lbl_path):
            with open(lbl_path, "r") as f:
                lines = f.read().strip().split("\n")

            for line in lines:
                parts = line.strip().split()
                if len(parts) != 5:
                    continue

                cls_id = int(parts[0])
                cx, cy, bw, bh = float(parts[1]), float(parts[2]), float(parts[3]), float(parts[4])

                # YOLO 정규화 좌표 → 픽셀 좌표
                x1 = int((cx - bw / 2) * w)
                y1 = int((cy - bh / 2) * h)
                x2 = int((cx + bw / 2) * w)
                y2 = int((cy + bh / 2) * h)

                color = COLORS.get(cls_id, (255, 255, 255))
                label = CLASS_NAMES.get(cls_id, f"cls_{cls_id}")

                cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)
                cv2.putText(img, label, (x1, y1 - 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

            print(f"  {basename}: {len(lines)}개 객체, 클래스={[CLASS_NAMES.get(int(l.split()[0]), '?') for l in lines]}")
        else:
            print(f"  {basename}: 라벨 파일 없음")

        # 저장
        out_path = os.path.join(verify_dir, f"verify_{basename}.jpg")
        cv2.imwrite(out_path, img)

    print(f"\n검증 이미지 저장 완료: {verify_dir}")
    print("해당 폴더의 이미지를 열어서 바운딩박스 위치가 정확한지 확인하세요.")


if __name__ == "__main__":
    verify_labels()
