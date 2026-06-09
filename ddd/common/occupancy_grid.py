"""common/occupancy_grid.py — 2D 점유격자(occupancy grid) 코어 (C-1).

LiDAR 스캔을 누적해 '점유(벽)/자유공간/미지'를 로그-오즈(log-odds)로 추정한다.
렌더링을 제외한 코어는 순수 파이썬 -> 하드웨어 없이 단위테스트 가능.

좌표 규약(레이더 화면과 일치):
  - 로봇기준 bearing=0  -> 월드 +Y(위),  bearing=+90 -> +X(오른쪽).
  - integrate_scan 의 점 각도는 '로봇기준 bearing(deg)'
    = (raw_angle - FORWARD_ANGLE_DEG + offset)  (호출측에서 변환해 전달).
  - pose=(x, y, theta): 월드 내 로봇 위치(m)와 헤딩(rad; 0 -> 정면이 +Y).
"""
import math

L_OCC = 0.85      # 점유 1회당 log-odds 증가
L_FREE = 0.40     # 자유 통과 1회당 log-odds 감소
L_CLAMP = 8.0     # log-odds 포화(±)


def bresenham(x0, y0, x1, y1):
    """정수 셀 (x0,y0)->(x1,y1) 직선상의 모든 셀 [(x,y),...] (양끝 포함)."""
    cells = []
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy
    x, y = x0, y0
    while True:
        cells.append((x, y))
        if x == x1 and y == y1:
            break
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x += sx
        if e2 < dx:
            err += dx
            y += sy
    return cells


class OccupancyGrid:
    def __init__(self, size_m=8.0, res_m=0.05):
        self.size_m = float(size_m)
        self.res = float(res_m)
        self.n = int(round(2 * self.size_m / self.res))
        self.origin = self.n // 2            # 월드 (0,0) 의 셀 인덱스
        self.log = [0.0] * (self.n * self.n)

    def in_bounds(self, ci, cj):
        return 0 <= ci < self.n and 0 <= cj < self.n

    def world_to_cell(self, x, y):
        # +1e-9: 셀 경계값의 부동소수 오차(예: 0.15/0.05=2.9999) 보정
        ci = self.origin + int(math.floor(x / self.res + 1e-9))
        cj = self.origin + int(math.floor(y / self.res + 1e-9))
        return ci, cj

    def _update(self, ci, cj, delta):
        k = cj * self.n + ci
        v = self.log[k] + delta
        if v > L_CLAMP:
            v = L_CLAMP
        elif v < -L_CLAMP:
            v = -L_CLAMP
        self.log[k] = v

    def prob(self, ci, cj):
        """셀의 점유확률(0~1). 0.5=미지, >0.5=점유, <0.5=자유."""
        l = self.log[cj * self.n + ci]
        return 1.0 - 1.0 / (1.0 + math.exp(l))

    def integrate_scan(self, pose, points, max_range_m=8.0):
        """pose=(x,y,theta)에서 본 points=[(bearing_deg, dist_mm), ...]를 격자에 누적.

        각 점: 로봇셀->끝점셀 직선(레이)을 '자유', 끝점을 '점유'로 갱신.
        거리가 max_range_m 초과면 끝점은 점유로 찍지 않는다(자유 레이만).
        """
        px, py, pth = pose
        rci, rcj = self.world_to_cell(px, py)
        for bearing_deg, dist_mm in points:
            if dist_mm <= 0:
                continue
            d = dist_mm / 1000.0
            hit = True
            if d > max_range_m:
                d = max_range_m
                hit = False
            ang = pth + math.radians(bearing_deg)
            wx = px + d * math.sin(ang)      # bearing 0 -> +Y, +90 -> +X
            wy = py + d * math.cos(ang)
            eci, ecj = self.world_to_cell(wx, wy)
            for ci, cj in bresenham(rci, rcj, eci, ecj)[:-1]:   # 끝점 제외 = 자유
                if self.in_bounds(ci, cj):
                    self._update(ci, cj, -L_FREE)
            if hit and self.in_bounds(eci, ecj):
                self._update(eci, ecj, +L_OCC)
