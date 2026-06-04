"""하드웨어 드라이버 패키지.

이 파일이 있어야 `from drivers.LidarX2 import LidarX2` 처럼
repo-root 기준으로 import 할 수 있다.
"""


def make_lidar(model, port, baud=None):
    """모델명으로 LiDAR 드라이버 인스턴스를 만든다.

    model: "x2"(기본) 또는 "x4". baud 가 None 이면 모델 기본값 사용
           (x2=115200, x4=128000). serial 의존 모듈은 함수 안에서 import 한다
           (패키지 import 만으로 serial 을 끌어오지 않도록).
    """
    m = (model or "x2").lower()
    if m == "x4":
        from drivers.LidarX4 import LidarX4
        return LidarX4(port, baud or 128000)
    from drivers.LidarX2 import LidarX2
    try:
        from common.config import LIDAR_X2_DIST_SCALE, LIDAR_X2_DIST_OFFSET_MM
    except Exception:
        LIDAR_X2_DIST_SCALE, LIDAR_X2_DIST_OFFSET_MM = 1.0, 0.0
    return LidarX2(port, baud or 115200,
                   dist_scale=LIDAR_X2_DIST_SCALE, dist_offset_mm=LIDAR_X2_DIST_OFFSET_MM)
