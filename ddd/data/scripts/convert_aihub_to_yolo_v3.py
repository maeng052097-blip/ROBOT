"""
AIHUB 재활용 쓰레기 → YOLO 데이터셋 변환 v3
=============================================
전략 변경:
- 라벨 JSON과 이미지 ZIP이 매칭되지 않는 문제 해결
- ZIP 파일명에서 카테고리 추출 (확실한 정보)
- 이미지에서 자동 바운딩박스 생성 (물체가 중앙에 1개씩 촬영됨)
- OpenCV로 전경/배경 분리하여 실제 물체 위치 검출

방식:
1) ZIP에서 카테고리별 이미지 추출
2) 각 이미지에서 OpenCV contour detection으로 물체 영역 검출
3) 검출 실패 시 중앙 70% 영역을 기본 bbox로 사용
4) YOLO TXT 라벨 자동 생성
"""
import os
import sys
import pathlib
import random
import shutil
import zipfile
import cv2
import numpy as np
from collections import defaultdict

# repo-root(ddd) 를 import 경로에 추가 -> common 패키지 사용
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2]))
from common.config import CONVERTED_DATA_DIR, PROJECT_ROOT
from common.classes import CATEGORY_KEYWORDS, CLASS_NAMES

# ============================================================
# 설정
# ============================================================

# AIHUB 원천 ZIP 들이 있는 폴더(저장소 밖 외부 데이터 — 환경에 맞게 수정).
ZIP_ROOT = r"C:\Users\MSY\Desktop\Training"
# 변환 결과 출력 위치(프로젝트 내부). common/config.py 에서 관리.
OUTPUT_ROOT = str(CONVERTED_DATA_DIR)

# 재활용 7개 카테고리(ZIP 파일명 키워드 → 클래스 id)는 common/classes.py 에서 가져온다.

SAMPLES_PER_CLASS = 800
VAL_RATIO = 0.2
RANDOM_SEED = 42

# ============================================================
# 1단계: ZIP → 카테고리 매핑
# ============================================================

def map_zips_to_categories():
    """ZIP 파일명에서 카테고리를 추출하여 매핑"""
    print("=" * 60)
    print("[1/5] ZIP 파일 → 카테고리 매핑")
    print("=" * 60)

    all_files = os.listdir(ZIP_ROOT)
    zip_files = [f for f in all_files if f.endswith(".zip") and "T원천" in f]

    zip_to_class = {}  # {zip_filename: class_id}

    for zf_name in zip_files:
        matched_cat = None
        for keyword, class_id in CATEGORY_KEYWORDS.items():
            if keyword in zf_name:
                matched_cat = (keyword, class_id)
                break

        if matched_cat:
            zip_to_class[zf_name] = matched_cat
            print(f"  {zf_name} → {matched_cat[0]} (class {matched_cat[1]})")

    print(f"\n  매핑된 ZIP: {len(zip_to_class)}개 / 전체 {len(zip_files)}개")
    return zip_to_class


# ============================================================
# 2단계: ZIP에서 이미지 추출 (카테고리별 균등 샘플링)
# ============================================================

def extract_sampled_images(zip_to_class):
    """카테고리별 균등 샘플링 후 이미지 추출"""
    print("\n" + "=" * 60)
    print("[2/5] 이미지 추출 중...")
    print("=" * 60)

    temp_dir = os.path.join(OUTPUT_ROOT, "_temp_images")
    os.makedirs(temp_dir, exist_ok=True)

    random.seed(RANDOM_SEED)

    # 카테고리별 ZIP 그룹화
    category_zips = defaultdict(list)
    for zf_name, (cat_name, class_id) in zip_to_class.items():
        category_zips[cat_name].append(zf_name)

    extracted = []  # [(image_path, class_id, cat_name), ...]

    for cat_name, class_id in CATEGORY_KEYWORDS.items():
        zips = category_zips.get(cat_name, [])
        if not zips:
            print(f"  {cat_name}: ZIP 없음, 건너뜀")
            continue

        # 이 카테고리의 모든 ZIP에서 이미지 목록 수집
        all_images = []  # [(zip_path, entry_name), ...]

        for zf_name in zips:
            zf_path = os.path.join(ZIP_ROOT, zf_name)
            try:
                with zipfile.ZipFile(zf_path, "r") as zf:
                    for entry in zf.namelist():
                        if entry.lower().endswith(".jpg"):
                            all_images.append((zf_path, entry))
            except Exception as e:
                print(f"  경고: {zf_name} 읽기 실패: {e}")

        # 샘플링
        n = min(SAMPLES_PER_CLASS, len(all_images))
        selected = random.sample(all_images, n)

        print(f"  {cat_name}: {len(all_images)}개 중 {n}개 선택 ({len(zips)}개 ZIP)")

        # 추출
        # ZIP별로 그룹화하여 효율적 추출
        by_zip = defaultdict(list)
        for zf_path, entry in selected:
            by_zip[zf_path].append(entry)

        cat_extracted = 0
        for zf_path, entries in by_zip.items():
            try:
                with zipfile.ZipFile(zf_path, "r") as zf:
                    for entry in entries:
                        img_name = entry.split("/")[-1]
                        # 파일명 충돌 방지: 카테고리 접두사 추가
                        safe_name = f"{class_id}_{img_name}"
                        target = os.path.join(temp_dir, safe_name)

                        if not os.path.exists(target):
                            data = zf.read(entry)
                            with open(target, "wb") as out:
                                out.write(data)

                        extracted.append((target, class_id, cat_name))
                        cat_extracted += 1
            except Exception as e:
                print(f"    ZIP 추출 오류: {e}")

        print(f"    → {cat_extracted}개 추출 완료")

    print(f"\n  총 추출: {len(extracted)}개")
    return extracted


# ============================================================
# 3단계: 자동 바운딩박스 생성
# ============================================================

def detect_object_bbox(image_path):
    """
    이미지에서 물체 영역을 자동 검출

    AIHUB 생활 폐기물 이미지 특성:
    - 단색 바닥(녹색 매트 등) 위에 물체 1개가 놓여있음
    - 물체가 이미지 중앙 부근에 위치

    방법:
    1) GrabCut 또는 Otsu 이진화로 전경 분리
    2) 가장 큰 contour의 bounding rect를 bbox로 사용
    3) 실패 시 중앙 60% 영역을 기본값으로 사용
    """
    img = cv2.imread(image_path)
    if img is None:
        return None

    h, w = img.shape[:2]

    try:
        # 방법: HSV 변환 + 배경색 제거
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # 여러 방법 시도하여 가장 좋은 결과 선택
        best_bbox = None
        best_area = 0

        # 방법 1: Otsu 이진화
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (11, 11), 0)
        _, thresh = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)

        # 노이즈 제거
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15, 15))
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)

        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            # 가장 큰 contour
            largest = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest)

            # 최소 면적 체크 (이미지의 3% 이상)
            if area > (w * h * 0.03):
                x, y, bw, bh = cv2.boundingRect(largest)

                # 약간의 여유(margin) 추가 (5%)
                margin_x = int(bw * 0.05)
                margin_y = int(bh * 0.05)
                x = max(0, x - margin_x)
                y = max(0, y - margin_y)
                bw = min(w - x, bw + 2 * margin_x)
                bh = min(h - y, bh + 2 * margin_y)

                best_bbox = (x, y, bw, bh)
                best_area = area

        # bbox가 유효한지 검증
        if best_bbox:
            x, y, bw, bh = best_bbox
            bbox_area = bw * bh
            img_area = w * h

            # bbox가 이미지의 5~90% 사이여야 유효
            if 0.05 < (bbox_area / img_area) < 0.90:
                return best_bbox

    except Exception:
        pass

    # 검출 실패 시 중앙 60% 영역 사용
    margin = 0.20
    x = int(w * margin)
    y = int(h * margin)
    bw = int(w * (1 - 2 * margin))
    bh = int(h * (1 - 2 * margin))
    return (x, y, bw, bh)


def generate_yolo_labels(extracted):
    """추출된 이미지에 대해 YOLO 라벨 생성"""
    print("\n" + "=" * 60)
    print("[3/5] 자동 바운딩박스 생성 중...")
    print("=" * 60)

    labeled = []  # [(image_path, safe_name, class_id, yolo_line, cat_name), ...]
    auto_detected = 0
    fallback_used = 0

    for idx, (image_path, class_id, cat_name) in enumerate(extracted):
        if (idx + 1) % 500 == 0:
            print(f"  진행: {idx+1}/{len(extracted)}")

        img = cv2.imread(image_path)
        if img is None:
            continue

        h, w = img.shape[:2]
        bbox = detect_object_bbox(image_path)

        if bbox is None:
            continue

        x, y, bw, bh = bbox

        # YOLO 정규화 좌표
        cx = (x + bw / 2.0) / w
        cy = (y + bh / 2.0) / h
        nw = bw / w
        nh = bh / h

        # 검출 vs 폴백 판단
        if abs(nw - 0.60) < 0.01 and abs(nh - 0.60) < 0.01:
            fallback_used += 1
        else:
            auto_detected += 1

        yolo_line = f"{class_id} {cx:.6f} {cy:.6f} {nw:.6f} {nh:.6f}"
        safe_name = os.path.basename(image_path)
        labeled.append((image_path, safe_name, class_id, yolo_line, cat_name))

    print(f"  자동 검출: {auto_detected}개")
    print(f"  중앙 기본값: {fallback_used}개")
    print(f"  총 라벨 생성: {len(labeled)}개")
    return labeled


# ============================================================
# 4단계: Train/Val 분할 및 데이터셋 생성
# ============================================================

def create_dataset(labeled):
    print("\n" + "=" * 60)
    print("[4/5] 데이터셋 분할 및 생성 중...")
    print("=" * 60)

    for split in ["train", "val"]:
        os.makedirs(os.path.join(OUTPUT_ROOT, "images", split), exist_ok=True)
        os.makedirs(os.path.join(OUTPUT_ROOT, "labels", split), exist_ok=True)

    # 층화 분할
    random.seed(RANDOM_SEED)
    by_class = defaultdict(list)
    for item in labeled:
        by_class[item[4]].append(item)

    train_items = []
    val_items = []

    for cat_name, items in by_class.items():
        random.shuffle(items)
        val_count = max(1, int(len(items) * VAL_RATIO))
        val_items.extend(items[:val_count])
        train_items.extend(items[val_count:])

    print(f"  분할: train {len(train_items)}개, val {len(val_items)}개")

    def write_split(items, split):
        count = 0
        for image_path, safe_name, class_id, yolo_line, cat_name in items:
            base = os.path.splitext(safe_name)[0]
            dst_img = os.path.join(OUTPUT_ROOT, "images", split, safe_name)
            dst_lbl = os.path.join(OUTPUT_ROOT, "labels", split, base + ".txt")

            if not os.path.exists(dst_img):
                shutil.copy2(image_path, dst_img)

            with open(dst_lbl, "w") as f:
                f.write(yolo_line + "\n")

            count += 1
        return count

    t = write_split(train_items, "train")
    v = write_split(val_items, "val")

    # 클래스별 통계
    print("\n  클래스별 분포:")
    for cat_name, class_id in CATEGORY_KEYWORDS.items():
        tc = sum(1 for i in train_items if i[4] == cat_name)
        vc = sum(1 for i in val_items if i[4] == cat_name)
        print(f"    [{class_id}] {cat_name}: train {tc}, val {vc}")

    return t, v


# ============================================================
# 5단계: data.yaml 생성
# ============================================================

def create_data_yaml():
    print("\n" + "=" * 60)
    print("[5/5] data.yaml 생성 중...")
    print("=" * 60)

    config_dir = os.path.join(str(PROJECT_ROOT), "config")
    os.makedirs(config_dir, exist_ok=True)

    yaml_path = os.path.join(config_dir, "data.yaml")
    abs_output = os.path.abspath(OUTPUT_ROOT)

    names_block = "\n".join(f"  {cid}: {name}" for cid, name in sorted(CLASS_NAMES.items()))
    yaml_content = f"""# AIHUB 재활용 쓰레기 데이터셋 - YOLO 학습 설정
# 자동 바운딩박스 생성 (ZIP 카테고리 기반)
path: {abs_output}
train: images/train
val: images/val

nc: {len(CLASS_NAMES)}
names:
{names_block}
"""

    with open(yaml_path, "w", encoding="utf-8") as f:
        f.write(yaml_content)

    print(f"  저장: {yaml_path}")
    return yaml_path


# ============================================================
# 메인
# ============================================================

def main():
    print("=" * 60)
    print("  AIHUB → YOLO 변환 v3 (자동 bbox 생성)")
    print(f"  카테고리: {len(CATEGORY_KEYWORDS)}개")
    print(f"  카테고리당 샘플: {SAMPLES_PER_CLASS}")
    print("=" * 60)

    # 이전 결과 정리
    for split in ["train", "val"]:
        for sub in ["images", "labels"]:
            d = os.path.join(OUTPUT_ROOT, sub, split)
            if os.path.isdir(d):
                shutil.rmtree(d)

    zip_to_class = map_zips_to_categories()
    extracted = extract_sampled_images(zip_to_class)
    labeled = generate_yolo_labels(extracted)
    train_count, val_count = create_dataset(labeled)
    yaml_path = create_data_yaml()

    print("\n" + "=" * 60)
    print("  변환 완료!")
    print("=" * 60)
    print(f"  train: {train_count}개")
    print(f"  val:   {val_count}개")
    print(f"  yaml:  {yaml_path}")
    print()
    print("  다음 단계:")
    print("  1) python data\\scripts\\verify_labels.py  (bbox 검증)")
    print("  2) python train.py  (YOLO 학습)")
    print("=" * 60)


if __name__ == "__main__":
    main()
