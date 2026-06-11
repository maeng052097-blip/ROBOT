"""물체 접근 제어 법칙 (순수 함수, 하드웨어 불필요).

LiDAR로 잰 물체까지의 거리(range_mm)와 정면 기준 방위(bearing_deg, +우측)를 받아
메카넘 차체 속도명령 (vx, vy, w) 과 상태를 돌려준다. 부호는 common.mecanum 과 동일
(vx +전진 / w +우회전).

정책(안전 우선, 단순·견고):
  1) range_mm 없음(None)            -> 정지, "LOST"   (물체 놓침/스캔평면 이탈)
  2) range_mm < min_safe_mm         -> 정지, "TOO_CLOSE" (X4 사거리 한계=블라인드존)
  3) 정면 향하고(±face_tol) 목표±deadband -> 정지, "ARRIVED"
  4) 그 외                           -> "APPROACH":
       - 먼저 물체를 정면으로(회전): w = clamp(kw*bearing)   (+우측물체 -> +우회전)
       - 정면을 향했을 때만 전진/후진: vx = clamp(kx*(range-target))  (멀면 +전진, 가까우면 후진)
       - vy(strafe)는 자동접근에선 0 (물체를 카메라 정면에 유지하려 회전으로 정렬)
"""


def _clamp(v, m):
    return max(-m, min(m, v))


def cam_approach_command(cam_range_mm, bearing_deg, min_cam_mm=600.0,
                         face_tol_deg=8.0, kw=1.2, vx_far=22, w_max=30):
    """카메라 단독(단안 거리) '원거리' 접근 — LiDAR 가 표적을 아직 못 잡는 구간용.

    동작: 표적을 회전으로 정면 정렬(±face_tol) -> 정면일 때만 서행 전진(vx_far).
    LiDAR 가 잡히면 호출측이 일반 approach_command 로 인계한다(이 함수는 모름).
    안전: cam_range < min_cam_mm 인데도 여기 머문다(=LiDAR 미획득)면 'CAM_LIMIT'
    정지 — 단안 오차(±수~10%)로 더 파고드는 맹목 접근을 금지. 측정 없으면 LOST.
    returns (vx, vy, w, state)  state: CAM_APPROACH | CAM_LIMIT | LOST
    """
    if cam_range_mm is None:
        return 0, 0, 0, "LOST"
    if cam_range_mm < min_cam_mm:
        return 0, 0, 0, "CAM_LIMIT"
    w = int(round(_clamp(kw * bearing_deg, w_max)))
    vx = int(vx_far) if abs(bearing_deg) <= face_tol_deg else 0
    return vx, 0, w, "CAM_APPROACH"


class RangeGate:
    """원거리 거리 추적 게이트(스파이크 억제) — 순수 로직, 하드웨어 불필요.

    배경: X4 각해상도 ~0.5deg 에서 9cm 물체는 2m=~5빔, 3.5m=~3빔만 받는다.
    회전마다 빔이 빗나가면 min_distance_in_arc 가 '배경' 거리를 반환해 값이 튀고,
    freshest 모드는 시간 평활이 없어 그대로 통과한다(실측: 2m+에서 거리 폭증).
    median 복원은 움직이는 물체를 가리므로(test_lidar_freshest) 대신 '추적 게이트':

      - 첫 측정은 그대로 수용(TRACK).
      - 이후 |새값-직전값| <= gate_mm 면 수용(TRACK) -> 움직이는 표적도 창이 따라감.
      - 게이트 밖(배경 점프)/측정 없음(빔 미스)은 직전값을 hold_s 동안 유지(HOLD).
      - hold_s 초과 시 상실(LOST, None) -> 다음 유효 측정을 새로 수용.
    """

    def __init__(self, gate_mm=300.0, hold_s=0.8):
        self.gate_mm = float(gate_mm)
        self.hold_s = float(hold_s)
        self.last = None      # 마지막 '수용'된 거리(mm)
        self.last_t = None    # 그 수용 시각(s)

    def reset(self):
        """추적 대상 변경(새 클릭) 시 호출 — 이전 표적의 거리 창을 버린다."""
        self.last = None
        self.last_t = None

    def update(self, meas_mm, now_s):
        """측정 1회 반영. returns (거리 mm 또는 None, 상태 'TRACK'|'HOLD'|'LOST')."""
        if meas_mm is not None and (self.last is None or abs(meas_mm - self.last) <= self.gate_mm):
            self.last = meas_mm
            self.last_t = now_s
            return meas_mm, "TRACK"
        if self.last is not None and (now_s - self.last_t) <= self.hold_s:
            return self.last, "HOLD"     # 스파이크/미스 -> 직전값 유지(coast)
        self.reset()
        if meas_mm is not None:          # 상실 후 재획득: 새 값을 그대로 수용
            self.last = meas_mm
            self.last_t = now_s
            return meas_mm, "TRACK"
        return None, "LOST"


def approach_command(range_mm, bearing_deg, target_mm,
                     deadband_mm=25.0, min_safe_mm=130.0, face_tol_deg=8.0,
                     kx=0.25, kw=1.2, vx_max=35, w_max=30):
    """returns (vx, vy, w, state). vx/vy/w 는 정수 퍼센트(-100~100)."""
    if range_mm is None:
        return 0, 0, 0, "LOST"
    if range_mm < min_safe_mm:
        return 0, 0, 0, "TOO_CLOSE"

    facing = abs(bearing_deg) <= face_tol_deg
    err = range_mm - target_mm
    if facing and abs(err) <= deadband_mm:
        return 0, 0, 0, "ARRIVED"

    w = int(round(_clamp(kw * bearing_deg, w_max)))        # +우측물체 -> +우회전
    vx = int(round(_clamp(kx * err, vx_max))) if facing else 0  # 정면 향했을 때만 전/후진
    return vx, 0, w, "APPROACH"
