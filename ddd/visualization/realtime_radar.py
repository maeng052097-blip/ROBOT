"""LiDAR(YDLIDAR X2) 실시간 레이더 시각화 + 클릭 거리측정.

LidarX2 드라이버로 LiDAR만 돌려, 측정점을 극좌표(레이더) 화면에 실시간 표시한다.
=> 드라이버가 실제로 동작하는지 + 거리/방향이 맞는지 '눈으로' 확인하는 용도.

조작:
  - 레이더 위 **좌클릭** -> 그 방향에서 가장 가까운 측정점의 거리(mm)를 파란색으로
    표시(매 프레임 갱신). 콘솔에도 "선택: NNdeg -> DDDD mm" 출력.
  - **우클릭** -> 선택 해제.
  - 창을 닫거나 Ctrl+C -> 종료.

표시:
  - 0deg = 차체 정면(FORWARD_ANGLE_DEG 기준), 화면 위쪽. 시계방향으로 각도 증가.
  - 점 색: DANGER(빨강) / SLOW(주황) / SAFE(초록). 가이드 링: DANGER_MM, SLOW_MM.

확인 팁:
  - 1m 앞에 물체 -> 위쪽(정면) 약 1000mm 위치 점. 클릭하면 "~1000 mm" 표시되면 정확.
  - 좌우가 반대로 보이면 set_theta_direction 의 -1 <-> 1 을 바꾼다.
  - 정면이 위가 아니면 tests/test_scan.py 로 측정해 FORWARD_ANGLE_DEG 를 조정.

필요: matplotlib (requirements.txt 포함), 데스크톱 GUI 환경.
실행: python visualization/realtime_radar.py
"""
import sys
import pathlib
import math

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))

from common.config import (
    LIDAR_PORT, LIDAR_BAUDRATE, LIDAR_MAX_AGE,
    FORWARD_ANGLE_DEG, DANGER_MM, SLOW_MM,
)
from drivers.LidarX2 import LidarX2

RMAX_MM = 4000        # 레이더 표시 최대 거리(mm). 필요시 조정.
SELECT_TOL_DEG = 5    # 선택 방향에서 이 각도 이내의 측정값을 그 방향 거리로 본다.


def zone_color(distance_mm):
    """거리에 따른 점 색(안전상태와 동일 기준)."""
    if distance_mm < DANGER_MM:
        return "red"
    if distance_mm < SLOW_MM:
        return "orange"
    return "green"


def to_plot_theta(lidar_angle_deg):
    """LiDAR 각도(deg) -> 플롯 theta(rad). 정면(FORWARD_ANGLE_DEG)을 0(위)으로."""
    return math.radians((lidar_angle_deg - FORWARD_ANGLE_DEG) % 360)


def angle_diff_deg(a, b):
    """두 각도(deg)의 최소 차이(0~180)."""
    return abs((a - b + 180) % 360 - 180)


def nearest_angle(distance_dict, target_deg):
    """target_deg 에 각도상 가장 가까운 (angle, distance, diff). 없으면 (None, None, None)."""
    best_a = best_d = best_diff = None
    for a, d in distance_dict.items():
        if d <= 0:
            continue
        diff = angle_diff_deg(a, target_deg)
        if best_diff is None or diff < best_diff:
            best_diff, best_a, best_d = diff, a, d
    return best_a, best_d, best_diff


def main():
    import numpy as np
    import matplotlib.pyplot as plt

    lidar = LidarX2(LIDAR_PORT, LIDAR_BAUDRATE)
    if not lidar.open():
        print(f"LiDAR({LIDAR_PORT}) 연결 실패. config.py 의 LIDAR_PORT 를 확인하세요.")
        return
    print(f"LiDAR 연결: {LIDAR_PORT}.")
    print("레이더 위 좌클릭 -> 그 방향 거리 측정 / 우클릭 -> 해제 / 창 닫기·Ctrl+C -> 종료")

    target = {"angle": None}  # 선택된 LiDAR 각도(deg) 또는 None

    def on_click(event):
        if event.inaxes is None or event.xdata is None:
            return
        if event.button == 3:  # 우클릭 -> 해제
            target["angle"] = None
            print("선택 해제")
            return
        click_theta = event.xdata  # 플롯 데이터 theta(rad)
        # 클릭 방향과 가장 가까운(각도차 최소) 현재 측정점 선택
        best, best_diff = None, None
        for a, d in lidar.getDistanceDict().items():
            if not (0 < d <= RMAX_MM):
                continue
            diff = abs((to_plot_theta(a) - click_theta + math.pi) % (2 * math.pi) - math.pi)
            if best_diff is None or diff < best_diff:
                best_diff, best = diff, (a, d)
        if best is not None:
            target["angle"] = best[0]
            print(f"선택: {best[0]}deg -> {best[1]} mm")
        else:
            rel = math.degrees(click_theta) % 360
            target["angle"] = round((rel + FORWARD_ANGLE_DEG) % 360)
            print(f"선택 방향: {target['angle']}deg (그 방향에 측정값 없음)")

    plt.rcParams["axes.unicode_minus"] = False
    plt.ion()
    fig = plt.figure(figsize=(7, 7))
    ax = fig.add_subplot(111, projection="polar")
    fig.canvas.mpl_connect("button_press_event", on_click)
    ring_theta = np.linspace(0, 2 * np.pi, 181)

    try:
        while plt.fignum_exists(fig.number):
            dd = lidar.getDistanceDict()

            ax.clear()
            ax.set_theta_zero_location("N")   # 0deg = 위(정면)
            ax.set_theta_direction(-1)        # 시계방향
            ax.set_rmax(RMAX_MM)
            ax.set_rticks([DANGER_MM, SLOW_MM, RMAX_MM])
            ax.grid(True, alpha=0.3)

            ax.plot(ring_theta, [DANGER_MM] * len(ring_theta), color="red", lw=1, alpha=0.5)
            ax.plot(ring_theta, [SLOW_MM] * len(ring_theta), color="orange", lw=1, alpha=0.5)

            thetas, radii, colors = [], [], []
            for a, d in dd.items():
                if 0 < d <= RMAX_MM:
                    thetas.append(to_plot_theta(a))
                    radii.append(d)
                    colors.append(zone_color(d))
            if thetas:
                ax.scatter(thetas, radii, s=6, c=colors)

            # 선택 방향의 거리 측정 표시
            sel_text = "selected: none"
            if target["angle"] is not None:
                na, nd, ndiff = nearest_angle(dd, target["angle"])
                if na is not None and ndiff is not None and ndiff <= SELECT_TOL_DEG:
                    th = to_plot_theta(na)
                    ax.scatter([th], [nd], s=150, facecolors="none",
                               edgecolors="blue", linewidths=2, zorder=5)
                    ax.annotate(f"{nd} mm", xy=(th, nd),
                                xytext=(th, min(nd + 450, RMAX_MM)),
                                color="blue", fontsize=12, ha="center", zorder=6)
                    sel_text = f"selected: {na}deg -> {nd} mm"
                else:
                    th = to_plot_theta(target["angle"])
                    ax.plot([th, th], [0, RMAX_MM], color="blue", lw=0.8, alpha=0.6)
                    sel_text = f"selected: {target['angle']}deg -> (no reading)"

            fresh = "OK" if lidar.is_fresh(LIDAR_MAX_AGE) else "STALE"
            ax.set_title(
                f"YDLIDAR X2 @ {LIDAR_PORT}   points:{len(dd)}   [{fresh}]\n"
                f"left-click=measure dir, right-click=clear   |   {sel_text}",
                fontsize=9,
            )
            plt.pause(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        lidar.close()
        plt.ioff()
        print("\n종료")


if __name__ == "__main__":
    main()
