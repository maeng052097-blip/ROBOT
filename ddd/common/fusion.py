"""카메라 베어링 + LiDAR 거리 융합.

카메라(YOLO)가 준 물체의 방향(베어링)을 LiDAR 각도로 변환해, 그 방향의
LiDAR 거리(mm)를 얻는다. => "탐지한 물체까지의 거리".

좌표 약속:
  - bearing_deg: 화면 중앙=0, 오른쪽 +, 왼쪽 - (detector.bearing_from_cx 와 동일).
  - LiDAR 각도: FORWARD_ANGLE_DEG 가 정면. CAMERA_LIDAR_SIGN 으로 좌우 방향을 맞춘다.
순수 함수라 하드웨어 없이 단위 테스트 가능.
"""
import math

from common.config import FORWARD_ANGLE_DEG, CAMERA_LIDAR_SIGN, FUSION_TOL_DEG, LIDAR_FLIPPED
from common.lidar_metrics import angular_diff


def lidar_dir_sign():
    """LiDAR raw 각도 증가 -> 로봇 우측(+bearing) 부호. (라이다 '장착'만의 성질)

    라이다가 '뒤집혀' 달리면(LIDAR_FLIPPED) 각도 증가 방향이 좌우로 거울 반전 -> -1.
    (카메라 이미지 좌우반전 보정 CAMERA_LIDAR_SIGN 은 별개 축 -> object_distance_mm 에서만 적용.)
    """
    return -1.0 if LIDAR_FLIPPED else 1.0


def lidar_bearing(raw_angle_deg, forward_deg=FORWARD_ANGLE_DEG):
    """LiDAR raw 각도(deg) -> 로봇 베어링(정면 0, 우측 +). 뒤집힘/전방각 반영.

    레이더 표시·클릭·카메라 매핑이 모두 '로봇 베어링' 한 좌표를 쓰게 하는 단일 변환.
    """
    return lidar_dir_sign() * (((raw_angle_deg - forward_deg + 180.0) % 360.0) - 180.0)


def bearing_to_lidar_angle(bearing_deg):
    """로봇/카메라 베어링(deg) -> LiDAR raw 각도(deg, 0~359). lidar_bearing 의 역함수.

    뒤집힘(LIDAR_FLIPPED)과 전방각(FORWARD_ANGLE_DEG)을 반영하므로 object_distance_mm
    등 융합 전부가 자동으로 거울반전을 따른다.
    """
    return (FORWARD_ANGLE_DEG + lidar_dir_sign() * bearing_deg) % 360.0


def distance_at_angle(distance_dict, target_deg, tol_deg=FUSION_TOL_DEG):
    """target_deg 에 tol_deg 이내로 각도상 가장 가까운 측정 거리(mm). 없으면 None."""
    best_d, best_diff = None, None
    for a, d in distance_dict.items():
        if d <= 0:
            continue
        diff = angular_diff(a, target_deg)
        if diff <= tol_deg and (best_diff is None or diff < best_diff):
            best_diff, best_d = diff, d
    return best_d


def object_distance_mm(bearing_deg, distance_dict, tol_deg=FUSION_TOL_DEG):
    """카메라 베어링 방향에 있는 물체까지의 LiDAR 거리(mm). 측정값 없으면 None.

    카메라 베어링 -> (CAMERA_LIDAR_SIGN) 로봇 베어링 -> (뒤집힘) LiDAR raw 각도 순서로 변환.
    """
    return distance_at_angle(
        distance_dict, bearing_to_lidar_angle(CAMERA_LIDAR_SIGN * bearing_deg), tol_deg)


def min_distance_in_arc(distance_dict, center_deg, half_arc_deg):
    """center_deg ± half_arc_deg 안의 점들 중 '가장 가까운' 거리(mm). 없으면 None.

    물체(예: 사람)는 그 각도 폭 안에서 배경을 가리는 '전경'이므로, 그 구간의
    최소 거리가 곧 물체까지의 거리다. (중심 한 각도의 최근접점보다 배경 오염에 강함)
    """
    best = None
    for a, d in distance_dict.items():
        if d <= 0:
            continue
        if angular_diff(a, center_deg) <= half_arc_deg and (best is None or d < best):
            best = d
    return best


def view_bearing_deg(cx_view, view_w, hfov_deg, zoom=1.0):
    """줌(중앙 크롭)된 뷰에서 물체 중심 cx_view(px) -> 카메라 베어링(deg, 중앙0/우+).

    핀홀 모델: bearing = atan( 2*(cx/view_w - 0.5) * tan(hfov/2) / zoom ).
    선형근사 (cx/view_w-0.5)*hfov 와 달리 화면 중간대역에서 참 핀홀각과 일치한다
    (선형은 70도 화각에서 최대 ~1.8도 편향). 디지털 줌은 중앙 크롭이라 같은 물체는
    줌 배율과 무관하게 같은 베어링(불변). 카메라-LiDAR 동일선상=시차0 이면 곧 LiDAR 방향.
    (호출측은 검출이 크롭 위에서 일어나므로 cx_view 가 [0, view_w] 안임을 보장.)
    """
    if view_w <= 0 or zoom <= 0:
        return 0.0
    u = cx_view / view_w - 0.5
    return math.degrees(math.atan(2.0 * u * math.tan(math.radians(hfov_deg) / 2.0) / zoom))


def effective_half_fov_deg(hfov_deg, zoom=1.0):
    """디지털 줌 z 에서 뷰가 실제로 덮는 '반화각'(deg) = atan(tan(hfov/2)/z).

    줌인하면 보이는 각도 범위가 줄어든다. '베어링이 화면 안인가' 게이트는
    고정 hfov/2 가 아니라 이 값을 써야 한다(줌>1에서 고정값을 쓰면 화면 밖
    방위를 안이라고 오판). view_bearing_deg(view_w, ...) == 이 값 (가장자리 일치).
    """
    if zoom <= 0:
        return hfov_deg / 2.0
    return math.degrees(math.atan(math.tan(math.radians(hfov_deg) / 2.0) / zoom))


def view_x_from_bearing(bearing_deg, view_w, hfov_deg, zoom=1.0):
    """view_bearing_deg 의 역함수: 카메라 베어링(deg) -> 뷰 픽셀 x(cx_view).

    LiDAR에서 클릭한 방위를 카메라 화면의 어느 가로위치에 그릴지(박스/포커스 영역)
    계산할 때 쓴다. cx = view_w * (0.5 + tan(b)*zoom / (2*tan(hfov/2))).
    화면 밖 방향이면 [0, view_w]로 클램프(가장자리). view_bearing_deg 와 왕복 일치.
    """
    if view_w <= 0 or zoom <= 0:
        return view_w / 2.0
    u = math.tan(math.radians(bearing_deg)) * zoom / (2.0 * math.tan(math.radians(hfov_deg) / 2.0))
    cx = view_w * (0.5 + u)
    return 0.0 if cx < 0 else (view_w if cx > view_w else cx)


def monocular_range_mm(box_w_px, view_w_px, hfov_deg, zoom=1.0, obj_width_mm=90.0):
    """크기를 아는 물체의 '카메라 단독' 거리(mm). 핀홀: D = W_real * f_view / w_px.

    f_view = view_w * zoom / (2*tan(hfov/2)).  중앙 크롭 디지털 줌은 픽셀 밀도를
    바꾸지 않으므로(크롭만) box_w_px 가 줌과 무관 -> 결과도 줌 불변.
    용도: LiDAR 가 못 미치는 원거리/스캔평면 밖 물체의 '보조' 거리(표시용).
    정확도는 HFOV·실물폭 정확도에 비례(±수~10%) -> 정밀 정지는 LiDAR 로.
    """
    if box_w_px <= 0 or view_w_px <= 0 or zoom <= 0 or obj_width_mm <= 0:
        return None
    f_view = view_w_px * zoom / (2.0 * math.tan(math.radians(hfov_deg) / 2.0))
    return obj_width_mm * f_view / box_w_px


def distance_along_ray(dd, off_x_mm, off_y_mm, ray_bearing_deg,
                       forward_deg=0.0, perp_tol_mm=200.0, align_tol_mm=20.0):
    """LiDAR에서 (off_x,off_y)mm 비켜난 카메라의 시선(ray) 위 물체의 LiDAR 거리.

    카메라가 옆으로 떨어져 있으면(시차) 카메라 베어링과 LiDAR 방향이 다르다. 시선:
        P(t) = (off_x, off_y) + t*(sin b, cos b)   (전방+Y, 우+X, b=ray_bearing_deg)
    선택 규칙(중요): 수직거리(perp) <= perp_tol 인 점들 중 '시선에 가장 잘 정렬된
    (perp 최소)' 점을 고른다. perp 가 비슷하면(align_tol 이내) 더 가까운(전경) 점을
    택한다. -> 시선 '옆'을 스치는 더 가까운 클러터/배경점을 시선 위 물체로 오인하지
    않는다(과거엔 '가장 가까운 t'를 골라 옆 점을 오인했음). 없으면 (None, None).
    """
    b = math.radians(ray_bearing_deg)
    dx, dy = math.sin(b), math.cos(b)
    best = None  # (perp, t, range_mm, angle_deg)
    for a, d in dd.items():
        if d <= 0:
            continue
        ar = math.radians(a - forward_deg)
        wx = d * math.sin(ar) - off_x_mm
        wy = d * math.cos(ar) - off_y_mm
        t = wx * dx + wy * dy                 # 시선 따라 전방 투영거리
        if t <= 0:
            continue
        perp = math.hypot(wx - t * dx, wy - t * dy)
        if perp > perp_tol_mm:
            continue
        if best is None:
            best = (perp, t, d, a)
        elif perp < best[0] - align_tol_mm:                       # 분명히 더 정렬됨
            best = (perp, t, d, a)
        elif abs(perp - best[0]) <= align_tol_mm and t < best[1]:  # 정렬 비슷 -> 전경(가까운) 우선
            best = (perp, t, d, a)
    return (best[2], best[3]) if best else (None, None)
