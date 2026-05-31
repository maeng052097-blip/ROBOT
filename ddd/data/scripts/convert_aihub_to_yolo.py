"""
AIHUB 생활 폐기물 데이터 → YOLO 포맷 변환 파이프라인
=====================================================
- 7개 재활용 카테고리만 추출
- 폴더명 기반 클래스 매핑 (JSON 내부 한글 깨짐 회피)
- ZIP에서 매칭 이미지 추출
- YOLO TXT 라벨 생성 + train/val 분할
"""
import json
import os
import sys
import glob
import random
import shutil
import zipfile
from pathlib import Path
from collections import defaultdict

# ============================================================
# 설정
# ============================================================

# 경로 설정 (사용자 환경에 맞게 수정)
LABEL_ROOT = r"C:\Users\MSY\Desktop\Training\labels"
ZIP_ROOT = r"C:\Users\MSY\Desktop\Training"
OUTPUT_ROOT = r"C:\Users\MSY\Desktop\main\data\converted"

# 재활용 7개 카테고리 → YOLO 클래스 인덱스
CLASS_MAP = {
    "비닐": 0,
    "스티로폼": 1,
    "유리병": 2,
    "종이류": 3,
    "캔류": 4,
    "페트병": 5,
    "플라스틱류": 6,
}

# 카테고리당 최대 샘플 수 (디스크 절약 + 빠른 첫 학습)
# 전체 데이터를 쓰려면 None으로 설정
SAMPLES_PER_CLASS = 800

# train/val 분할 비율
VAL_RATIO = 0.2

# 랜덤 시드 (재현성)
RANDOM_SEED = 42

# ============================================================
# 1단계: 라벨 스캔 및 수집
# ============================================================

def scan_labels():
    """라벨 폴더를 스캔하여 재활용 카테고리의 JSON 파일 목록을 수집"""
    print("=" * 60)
    print("[1/5] 라벨 파일 스캔 중...")
    print("=" * 60)

    label_info = []  # [(json_path, category_name, subcategory_name, session_code), ...]

    for category in CLASS_MAP.keys():
        cat_dir = os.path.join(LABEL_ROOT, category)
        if not os.path.isdir(cat_dir):
            print(f"  경고: 카테고리 폴더 없음 → {cat_dir}")
            continue

        # 하위 폴더(소분류) 탐색
        subcategories = [d for d in os.listdir(cat_dir)
                         if os.path.isdir(os.path.join(cat_dir, d))]

        cat_count = 0
        for subcat in subcategories:
            subcat_dir = os.path.join(cat_dir, subcat)

            # 세션 폴더 탐색
            sessions = [d for d in os.listdir(subcat_dir)
                        if os.path.isdir(os.path.join(subcat_dir, d))]

            for session in sessions:
                session_dir = os.path.join(subcat_dir, session)
                json_files = glob.glob(os.path.join(session_dir, "*.Json"))
                json_files += glob.glob(os.path.join(session_dir, "*.json"))

                for jf in json_files:
                    label_info.append((jf, category, subcat, session))
                    cat_count += 1

        print(f"  {category}: {cat_count}개 라벨 발견 (소분류: {len(subcategories)}개)")

    print(f"\n  총 라벨 수: {len(label_info)}개")
    return label_info


# ============================================================
# 2단계: 샘플링
# ============================================================

def sample_labels(label_info):
    """카테고리별 균등 샘플링"""
    print("\n" + "=" * 60)
    print("[2/5] 샘플링 중...")
    print("=" * 60)

    if SAMPLES_PER_CLASS is None:
        print(f"  전체 데이터 사용: {len(label_info)}개")
        return label_info

    random.seed(RANDOM_SEED)

    # 카테고리별 그룹화
    by_category = defaultdict(list)
    for item in label_info:
        by_category[item[1]].append(item)

    sampled = []
    for category, items in by_category.items():
        n = min(SAMPLES_PER_CLASS, len(items))
        selected = random.sample(items, n)
        sampled.extend(selected)
        print(f"  {category}: {len(items)}개 중 {n}개 선택")

    random.shuffle(sampled)
    print(f"\n  샘플링 완료: 총 {len(sampled)}개")
    return sampled


# ============================================================
# 3단계: JSON → YOLO 라벨 변환
# ============================================================

def parse_json_and_convert(json_path, category):
    """
    JSON 라벨 파일을 읽어 YOLO 포맷 문자열로 변환

    YOLO 포맷: class_id cx cy w h (모든 값 0~1 정규화)

    반환: (image_filename, resolution_str, yolo_lines_list) 또는 None
    """
    try:
        # JSON 읽기 (ASCII 필드만 사용하므로 인코딩 문제 없음)
        with open(json_path, "r", encoding="utf-8", errors="replace") as f:
            data = json.load(f)
    except (json.JSONDecodeError, UnicodeDecodeError):
        try:
            with open(json_path, "r", encoding="cp949", errors="replace") as f:
                data = json.load(f)
        except Exception:
            return None

    # 이미지 파일명
    image_filename = data.get("FILE NAME", "")
    if not image_filename:
        return None

    # 해상도 파싱: "2221*1080" → (2221, 1080)
    resolution_str = data.get("RESOLUTION", "")
    if "*" not in resolution_str:
        return None

    try:
        parts = resolution_str.split("*")
        img_w = int(parts[0].strip())
        img_h = int(parts[1].strip())
    except (ValueError, IndexError):
        return None

    if img_w <= 0 or img_h <= 0:
        return None

    # 바운딩박스 변환
    class_id = CLASS_MAP[category]
    boundings = data.get("Bounding", [])
    yolo_lines = []

    for bbox in boundings:
        try:
            x1 = int(bbox["x1"])
            y1 = int(bbox["y1"])
            x2 = int(bbox["x2"])
            y2 = int(bbox["y2"])
        except (KeyError, ValueError):
            continue

        # 좌표 유효성 검사
        if x1 >= x2 or y1 >= y2:
            continue
        if x1 < 0 or y1 < 0 or x2 > img_w or y2 > img_h:
            # 범위 클리핑
            x1 = max(0, x1)
            y1 = max(0, y1)
            x2 = min(img_w, x2)
            y2 = min(img_h, y2)

        # YOLO 정규화 좌표 계산
        cx = ((x1 + x2) / 2.0) / img_w
        cy = ((y1 + y2) / 2.0) / img_h
        w = (x2 - x1) / img_w
        h = (y2 - y1) / img_h

        # 범위 검증 (0~1)
        if not (0 <= cx <= 1 and 0 <= cy <= 1 and 0 < w <= 1 and 0 < h <= 1):
            continue

        yolo_lines.append(f"{class_id} {cx:.6f} {cy:.6f} {w:.6f} {h:.6f}")

    if not yolo_lines:
        return None

    return (image_filename, resolution_str, yolo_lines)


# ============================================================
# 4단계: ZIP에서 이미지 매칭 및 추출
# ============================================================

def build_zip_index():
    """ZIP 파일명 → ZIP 경로 매핑 생성"""
    zip_index = {}
    for zf in glob.glob(os.path.join(ZIP_ROOT, "*.zip")):
        basename = os.path.basename(zf)
        zip_index[basename] = zf
    return zip_index


def find_zip_for_label(category, subcategory, zip_index):
    """라벨의 카테고리/소분류로부터 해당 ZIP 파일을 찾기"""
    # ZIP 파일명 패턴: [T원천]{대분류}_{소분류}_{소분류}.zip
    pattern1 = f"[T원천]{category}_{subcategory}_{subcategory}.zip"

    if pattern1 in zip_index:
        return zip_index[pattern1]

    # 패턴 매칭 실패 시 부분 매칭 시도
    for name, path in zip_index.items():
        if category in name and subcategory in name:
            return path

    return None


def extract_images_from_zips(sampled_labels, zip_index, output_image_dir):
    """
    샘플링된 라벨에 대응하는 이미지를 ZIP에서 추출

    반환: {image_filename: extracted_path} 딕셔너리
    """
    print("\n" + "=" * 60)
    print("[3/5] ZIP에서 이미지 추출 중...")
    print("=" * 60)

    os.makedirs(output_image_dir, exist_ok=True)

    # ZIP별로 추출할 이미지를 그룹화
    # {zip_path: [(session_code, image_filename, target_path), ...]}
    zip_tasks = defaultdict(list)
    extracted_map = {}

    for json_path, category, subcategory, session_code in sampled_labels:
        # JSON 파일명에서 이미지 파일명 유추
        json_basename = os.path.basename(json_path)
        image_filename = os.path.splitext(json_basename)[0] + ".jpg"

        # 이미 추출된 경우 스킵
        target_path = os.path.join(output_image_dir, image_filename)
        if os.path.exists(target_path):
            extracted_map[image_filename] = target_path
            continue

        # ZIP 찾기
        zip_path = find_zip_for_label(category, subcategory, zip_index)
        if zip_path is None:
            continue

        # ZIP 내부 경로: session_code/image_filename
        zip_internal_path = f"{session_code}/{image_filename}"
        zip_tasks[zip_path].append((zip_internal_path, image_filename, target_path))

    # ZIP별로 일괄 추출
    total_zips = len(zip_tasks)
    total_images = sum(len(v) for v in zip_tasks.values())
    print(f"  추출 대상: {total_images}개 이미지, {total_zips}개 ZIP")

    extracted_count = 0
    failed_count = 0

    for zip_idx, (zip_path, tasks) in enumerate(zip_tasks.items(), 1):
        zip_name = os.path.basename(zip_path)
        print(f"  [{zip_idx}/{total_zips}] {zip_name} ({len(tasks)}개 이미지)...", end="", flush=True)

        try:
            with zipfile.ZipFile(zip_path, "r") as zf:
                # ZIP 내부 파일 목록을 한번만 조회
                zip_contents = set(zf.namelist())

                for zip_internal, img_name, target in tasks:
                    if zip_internal in zip_contents:
                        # 이미지 추출
                        data = zf.read(zip_internal)
                        with open(target, "wb") as out:
                            out.write(data)
                        extracted_map[img_name] = target
                        extracted_count += 1
                    else:
                        # 대소문자 차이 등으로 못 찾을 경우 유사 매칭
                        found = False
                        for entry in zip_contents:
                            if entry.lower().endswith(img_name.lower()):
                                data = zf.read(entry)
                                with open(target, "wb") as out:
                                    out.write(data)
                                extracted_map[img_name] = target
                                extracted_count += 1
                                found = True
                                break
                        if not found:
                            failed_count += 1

            print(f" 완료")
        except Exception as e:
            print(f" 오류: {e}")
            failed_count += len(tasks)

    print(f"\n  추출 완료: {extracted_count}개 성공, {failed_count}개 실패")
    return extracted_map


# ============================================================
# 5단계: YOLO 데이터셋 생성 + train/val 분할
# ============================================================

def create_yolo_dataset(sampled_labels, extracted_map):
    """YOLO 포맷 데이터셋 생성 및 train/val 분할"""
    print("\n" + "=" * 60)
    print("[4/5] YOLO 데이터셋 생성 중...")
    print("=" * 60)

    # 출력 디렉토리 생성
    dirs = [
        os.path.join(OUTPUT_ROOT, "images", "train"),
        os.path.join(OUTPUT_ROOT, "images", "val"),
        os.path.join(OUTPUT_ROOT, "labels", "train"),
        os.path.join(OUTPUT_ROOT, "labels", "val"),
    ]
    for d in dirs:
        os.makedirs(d, exist_ok=True)

    # 유효한 샘플 수집
    valid_samples = []  # [(image_path, yolo_lines, category), ...]

    convert_success = 0
    convert_fail = 0

    for json_path, category, subcategory, session_code in sampled_labels:
        result = parse_json_and_convert(json_path, category)
        if result is None:
            convert_fail += 1
            continue

        image_filename, resolution, yolo_lines = result

        # 이미지 파일 존재 확인
        if image_filename not in extracted_map:
            # JSON 파일명 기반으로 재시도
            json_basename = os.path.basename(json_path)
            alt_filename = os.path.splitext(json_basename)[0] + ".jpg"
            if alt_filename not in extracted_map:
                convert_fail += 1
                continue
            image_filename = alt_filename

        image_path = extracted_map[image_filename]
        valid_samples.append((image_path, image_filename, yolo_lines, category))
        convert_success += 1

    print(f"  라벨 변환: {convert_success}개 성공, {convert_fail}개 실패")

    # train/val 분할 (카테고리별 층화 추출)
    random.seed(RANDOM_SEED)

    by_category = defaultdict(list)
    for item in valid_samples:
        by_category[item[3]].append(item)

    train_samples = []
    val_samples = []

    for category, items in by_category.items():
        random.shuffle(items)
        val_count = max(1, int(len(items) * VAL_RATIO))
        val_samples.extend(items[:val_count])
        train_samples.extend(items[val_count:])

    print(f"  분할: train {len(train_samples)}개, val {len(val_samples)}개")

    # 파일 복사 및 라벨 생성
    def write_split(samples, split_name):
        img_dir = os.path.join(OUTPUT_ROOT, "images", split_name)
        lbl_dir = os.path.join(OUTPUT_ROOT, "labels", split_name)
        count = 0

        for image_path, image_filename, yolo_lines, category in samples:
            # 파일명 충돌 방지 (같은 이름이 다른 카테고리에 있을 수 있음)
            base_name = os.path.splitext(image_filename)[0]
            dst_img = os.path.join(img_dir, image_filename)
            dst_lbl = os.path.join(lbl_dir, base_name + ".txt")

            # 이미지 복사
            if not os.path.exists(dst_img):
                shutil.copy2(image_path, dst_img)

            # YOLO 라벨 파일 생성
            with open(dst_lbl, "w", encoding="utf-8") as f:
                f.write("\n".join(yolo_lines) + "\n")

            count += 1

        return count

    train_count = write_split(train_samples, "train")
    val_count = write_split(val_samples, "val")

    print(f"  파일 생성 완료: train {train_count}개, val {val_count}개")
    return train_count, val_count


# ============================================================
# 6단계: data.yaml 생성
# ============================================================

def create_data_yaml():
    """YOLOv8 학습용 data.yaml 생성"""
    print("\n" + "=" * 60)
    print("[5/5] data.yaml 생성 중...")
    print("=" * 60)

    config_dir = os.path.join(os.path.dirname(OUTPUT_ROOT), "..", "config")
    os.makedirs(config_dir, exist_ok=True)

    yaml_path = os.path.join(config_dir, "data.yaml")
    yaml_content = f"""# AIHUB 재활용 쓰레기 데이터셋 — YOLO 학습 설정
# 자동 생성됨

path: {os.path.abspath(OUTPUT_ROOT)}
train: images/train
val: images/val

nc: {len(CLASS_MAP)}
names:
  0: vinyl
  1: styrofoam
  2: glass_bottle
  3: paper
  4: can
  5: pet_bottle
  6: plastic
"""

    with open(yaml_path, "w", encoding="utf-8") as f:
        f.write(yaml_content)

    print(f"  저장: {yaml_path}")
    return yaml_path


# ============================================================
# 메인 실행
# ============================================================

def main():
    print("=" * 60)
    print("  AIHUB → YOLO 변환 파이프라인")
    print(f"  카테고리: {len(CLASS_MAP)}개 재활용 분류")
    print(f"  카테고리당 샘플: {SAMPLES_PER_CLASS or '전체'}")
    print("=" * 60)

    # 경로 검증
    if not os.path.isdir(LABEL_ROOT):
        print(f"오류: 라벨 폴더를 찾을 수 없습니다 → {LABEL_ROOT}")
        sys.exit(1)

    if not os.path.isdir(ZIP_ROOT):
        print(f"오류: ZIP 폴더를 찾을 수 없습니다 → {ZIP_ROOT}")
        sys.exit(1)

    # 실행
    label_info = scan_labels()
    sampled = sample_labels(label_info)
    zip_index = build_zip_index()

    print(f"\n  발견된 ZIP 파일: {len(zip_index)}개")

    # 임시 이미지 추출 폴더
    temp_image_dir = os.path.join(OUTPUT_ROOT, "_temp_images")
    extracted_map = extract_images_from_zips(sampled, zip_index, temp_image_dir)

    train_count, val_count = create_yolo_dataset(sampled, extracted_map)
    yaml_path = create_data_yaml()

    # 결과 요약
    print("\n" + "=" * 60)
    print("  변환 완료!")
    print("=" * 60)
    print(f"  train 이미지: {train_count}개")
    print(f"  val 이미지:   {val_count}개")
    print(f"  data.yaml:    {yaml_path}")
    print(f"  출력 경로:    {OUTPUT_ROOT}")
    print()
    print("  다음 단계:")
    print("  python train.py")
    print("=" * 60)

    # 임시 폴더 정리 여부 확인
    if os.path.isdir(temp_image_dir):
        print(f"\n  임시 이미지 폴더({temp_image_dir})를 삭제하려면:")
        print(f"  Remove-Item -Recurse -Force \"{temp_image_dir}\"")


if __name__ == "__main__":
    main()
