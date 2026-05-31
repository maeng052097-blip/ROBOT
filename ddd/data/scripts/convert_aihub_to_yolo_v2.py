"""
AIHUB 생활 폐기물 데이터 → YOLO 포맷 변환 파이프라인 v2
=======================================================
수정사항:
- glob 대신 os.listdir 사용 (대괄호 문제 해결)
- 전체 ZIP의 글로벌 파일명 인덱스 구축 (카테고리 추론 제거)
- 세션코드 기반 매칭으로 변경
"""
import json
import os
import sys
import random
import shutil
import zipfile
from pathlib import Path
from collections import defaultdict

# ============================================================
# 설정
# ============================================================

LABEL_ROOT = r"C:\Users\MSY\Desktop\Training\labels"
ZIP_ROOT = r"C:\Users\MSY\Desktop\Training"
OUTPUT_ROOT = r"C:\Users\MSY\Desktop\main\data\converted"

CLASS_MAP = {
    "비닐": 0,
    "스티로폼": 1,
    "유리병": 2,
    "종이류": 3,
    "캔류": 4,
    "페트병": 5,
    "플라스틱류": 6,
}

SAMPLES_PER_CLASS = 800
VAL_RATIO = 0.2
RANDOM_SEED = 42

# ============================================================
# 1단계: 라벨 스캔
# ============================================================

def scan_labels():
    print("=" * 60)
    print("[1/6] 라벨 파일 스캔 중...")
    print("=" * 60)

    label_info = []

    for category in CLASS_MAP.keys():
        cat_dir = os.path.join(LABEL_ROOT, category)
        if not os.path.isdir(cat_dir):
            print(f"  경고: 카테고리 폴더 없음 → {cat_dir}")
            continue

        cat_count = 0
        for root, dirs, files in os.walk(cat_dir):
            for f in files:
                if f.lower().endswith(".json"):
                    json_path = os.path.join(root, f)
                    # 세션코드 = 부모 폴더명
                    session_code = os.path.basename(root)
                    label_info.append((json_path, category, session_code))
                    cat_count += 1

        print(f"  {category}: {cat_count}개 라벨")

    print(f"\n  총 라벨 수: {len(label_info)}개")
    return label_info


# ============================================================
# 2단계: 샘플링
# ============================================================

def sample_labels(label_info):
    print("\n" + "=" * 60)
    print("[2/6] 샘플링 중...")
    print("=" * 60)

    if SAMPLES_PER_CLASS is None:
        print(f"  전체 데이터 사용: {len(label_info)}개")
        return label_info

    random.seed(RANDOM_SEED)

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
# 3단계: 전체 ZIP 글로벌 인덱스 구축
# ============================================================

def build_global_zip_index():
    """
    전체 ZIP 파일을 스캔하여 세션코드 → (zip_path) 매핑 생성
    ZIP 내부 구조: 세션코드/파일명.jpg
    """
    print("\n" + "=" * 60)
    print("[3/6] ZIP 글로벌 인덱스 구축 중...")
    print("=" * 60)

    # os.listdir으로 ZIP 파일 목록 (glob 대괄호 문제 회피)
    all_files = os.listdir(ZIP_ROOT)
    zip_files = [f for f in all_files if f.endswith(".zip") and "T원천" in f]

    print(f"  발견된 ZIP 파일: {len(zip_files)}개")

    # 세션코드 → zip_path 매핑
    session_to_zip = {}
    total_sessions = 0

    for idx, zf_name in enumerate(zip_files, 1):
        zf_path = os.path.join(ZIP_ROOT, zf_name)
        print(f"  [{idx}/{len(zip_files)}] {zf_name}...", end="", flush=True)

        try:
            with zipfile.ZipFile(zf_path, "r") as zf:
                entries = zf.namelist()

                # 세션코드 추출 (폴더 엔트리에서)
                sessions_in_zip = set()
                for entry in entries:
                    parts = entry.rstrip("/").split("/")
                    if len(parts) >= 1 and parts[0]:
                        sessions_in_zip.add(parts[0])

                for session in sessions_in_zip:
                    session_to_zip[session] = zf_path

                total_sessions += len(sessions_in_zip)
                print(f" {len(sessions_in_zip)}개 세션")

        except Exception as e:
            print(f" 오류: {e}")

    print(f"\n  인덱스 완료: {total_sessions}개 세션, {len(zip_files)}개 ZIP")
    return session_to_zip


# ============================================================
# 4단계: 이미지 추출
# ============================================================

def extract_images(sampled_labels, session_to_zip, output_image_dir):
    print("\n" + "=" * 60)
    print("[4/6] ZIP에서 이미지 추출 중...")
    print("=" * 60)

    os.makedirs(output_image_dir, exist_ok=True)

    # ZIP별로 추출할 작업 그룹화
    zip_tasks = defaultdict(list)
    not_found_sessions = set()

    for json_path, category, session_code in sampled_labels:
        # JSON 파일명 → 이미지 파일명
        json_basename = os.path.basename(json_path)
        img_name = os.path.splitext(json_basename)[0] + ".jpg"
        # 대소문자 맞추기: .Json → .jpg
        img_name = img_name.replace(".Json.jpg", ".jpg").replace(".JSON.jpg", ".jpg")
        # 정확한 변환: 확장자만 교체
        base_no_ext = os.path.splitext(json_basename)[0]
        img_name = base_no_ext + ".jpg"

        target_path = os.path.join(output_image_dir, img_name)

        if os.path.exists(target_path):
            continue

        if session_code in session_to_zip:
            zip_path = session_to_zip[session_code]
            zip_internal = f"{session_code}/{img_name}"
            zip_tasks[zip_path].append((zip_internal, img_name, target_path))
        else:
            not_found_sessions.add(session_code)

    total_images = sum(len(v) for v in zip_tasks.values())
    print(f"  추출 대상: {total_images}개 이미지, {len(zip_tasks)}개 ZIP")
    if not_found_sessions:
        print(f"  세션 미발견: {len(not_found_sessions)}개 (해당 이미지 없음)")

    extracted_count = 0
    failed_count = 0

    for zip_idx, (zip_path, tasks) in enumerate(zip_tasks.items(), 1):
        zip_name = os.path.basename(zip_path)
        print(f"  [{zip_idx}/{len(zip_tasks)}] {zip_name} ({len(tasks)}개)...", end="", flush=True)

        try:
            with zipfile.ZipFile(zip_path, "r") as zf:
                zip_contents = set(zf.namelist())

                for zip_internal, img_name, target in tasks:
                    if zip_internal in zip_contents:
                        data = zf.read(zip_internal)
                        with open(target, "wb") as out:
                            out.write(data)
                        extracted_count += 1
                    else:
                        # JPG 확장자 대소문자 차이 시도
                        alt = zip_internal.replace(".jpg", ".JPG")
                        if alt in zip_contents:
                            data = zf.read(alt)
                            with open(target, "wb") as out:
                                out.write(data)
                            extracted_count += 1
                        else:
                            failed_count += 1

            print(f" 완료")
        except Exception as e:
            print(f" 오류: {e}")
            failed_count += len(tasks)

    print(f"\n  추출 완료: {extracted_count}개 성공, {failed_count}개 실패")
    return extracted_count


# ============================================================
# 5단계: YOLO 라벨 변환 + 데이터셋 생성
# ============================================================

def parse_json_to_yolo(json_path, category):
    """JSON → YOLO 라벨 문자열 변환"""
    try:
        with open(json_path, "r", encoding="utf-8", errors="replace") as f:
            data = json.load(f)
    except (json.JSONDecodeError, UnicodeDecodeError):
        try:
            with open(json_path, "r", encoding="cp949", errors="replace") as f:
                data = json.load(f)
        except Exception:
            return None

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

        if x1 >= x2 or y1 >= y2:
            continue

        x1 = max(0, x1)
        y1 = max(0, y1)
        x2 = min(img_w, x2)
        y2 = min(img_h, y2)

        cx = ((x1 + x2) / 2.0) / img_w
        cy = ((y1 + y2) / 2.0) / img_h
        w = (x2 - x1) / img_w
        h = (y2 - y1) / img_h

        if not (0 <= cx <= 1 and 0 <= cy <= 1 and 0 < w <= 1 and 0 < h <= 1):
            continue

        yolo_lines.append(f"{class_id} {cx:.6f} {cy:.6f} {w:.6f} {h:.6f}")

    if not yolo_lines:
        return None

    return yolo_lines


def create_yolo_dataset(sampled_labels, temp_image_dir):
    print("\n" + "=" * 60)
    print("[5/6] YOLO 데이터셋 생성 중...")
    print("=" * 60)

    for split in ["train", "val"]:
        os.makedirs(os.path.join(OUTPUT_ROOT, "images", split), exist_ok=True)
        os.makedirs(os.path.join(OUTPUT_ROOT, "labels", split), exist_ok=True)

    valid_samples = []

    for json_path, category, session_code in sampled_labels:
        yolo_lines = parse_json_to_yolo(json_path, category)
        if yolo_lines is None:
            continue

        base_no_ext = os.path.splitext(os.path.basename(json_path))[0]
        img_name = base_no_ext + ".jpg"
        img_path = os.path.join(temp_image_dir, img_name)

        if not os.path.exists(img_path):
            continue

        valid_samples.append((img_path, img_name, base_no_ext, yolo_lines, category))

    print(f"  유효 샘플: {len(valid_samples)}개")

    # 층화 분할
    random.seed(RANDOM_SEED)
    by_category = defaultdict(list)
    for item in valid_samples:
        by_category[item[4]].append(item)

    train_samples = []
    val_samples = []
    for category, items in by_category.items():
        random.shuffle(items)
        val_count = max(1, int(len(items) * VAL_RATIO))
        val_samples.extend(items[:val_count])
        train_samples.extend(items[val_count:])

    print(f"  분할: train {len(train_samples)}개, val {len(val_samples)}개")

    def write_split(samples, split_name):
        count = 0
        for img_path, img_name, base_no_ext, yolo_lines, category in samples:
            dst_img = os.path.join(OUTPUT_ROOT, "images", split_name, img_name)
            dst_lbl = os.path.join(OUTPUT_ROOT, "labels", split_name, base_no_ext + ".txt")

            if not os.path.exists(dst_img):
                shutil.copy2(img_path, dst_img)

            with open(dst_lbl, "w", encoding="utf-8") as f:
                f.write("\n".join(yolo_lines) + "\n")
            count += 1
        return count

    train_count = write_split(train_samples, "train")
    val_count = write_split(val_samples, "val")

    # 클래스별 통계
    print("\n  클래스별 분포:")
    for category in CLASS_MAP:
        t = sum(1 for s in train_samples if s[4] == category)
        v = sum(1 for s in val_samples if s[4] == category)
        print(f"    {category}: train {t}, val {v}")

    return train_count, val_count


# ============================================================
# 6단계: data.yaml 생성
# ============================================================

def create_data_yaml():
    print("\n" + "=" * 60)
    print("[6/6] data.yaml 생성 중...")
    print("=" * 60)

    config_dir = os.path.join(r"C:\Users\MSY\Desktop\main", "config")
    os.makedirs(config_dir, exist_ok=True)

    yaml_path = os.path.join(config_dir, "data.yaml")
    abs_output = os.path.abspath(OUTPUT_ROOT)

    yaml_content = f"""# AIHUB 재활용 쓰레기 데이터셋 - YOLO 학습 설정
path: {abs_output}
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
# 메인
# ============================================================

def main():
    print("=" * 60)
    print("  AIHUB → YOLO 변환 파이프라인 v2")
    print(f"  카테고리: {len(CLASS_MAP)}개")
    print(f"  카테고리당 샘플: {SAMPLES_PER_CLASS or '전체'}")
    print("=" * 60)

    if not os.path.isdir(LABEL_ROOT):
        print(f"오류: 라벨 폴더 없음 → {LABEL_ROOT}")
        sys.exit(1)

    label_info = scan_labels()
    sampled = sample_labels(label_info)
    session_to_zip = build_global_zip_index()

    temp_image_dir = os.path.join(OUTPUT_ROOT, "_temp_images")
    extract_images(sampled, session_to_zip, temp_image_dir)

    train_count, val_count = create_yolo_dataset(sampled, temp_image_dir)
    yaml_path = create_data_yaml()

    print("\n" + "=" * 60)
    print("  변환 완료!")
    print("=" * 60)
    print(f"  train: {train_count}개")
    print(f"  val:   {val_count}개")
    print(f"  yaml:  {yaml_path}")
    print(f"  출력:  {OUTPUT_ROOT}")
    print()
    print("  다음 단계: python train.py")
    print("=" * 60)


if __name__ == "__main__":
    main()
