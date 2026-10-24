"""도크 ToF (VL53L0X x2, TCA9548A ch0 = TOF-IN ① · ch1 = TOF-NG ③) 판정(순수 로직).

R22 문서 §4: 도크 밑에서 위를 봐 카세트 밑면까지 거리 -> 카세트 있음 · 없음만 본다(웨이퍼 개수는 세지 않음).
  - TOF-IN: 투입 비었음 · 카세트 놓기 확인 · 빈 자리 집기 막음
  - TOF-NG: NG 카세트 있음 · 없음 (가득은 NG 포인터 카운터로). 없음 -> 있음 으로 바뀌면 새 빈 카세트로 보고 포인터 0.
문턱 TOF_PRESENT_MM 은 가정 - R22 에서 ① 도크가 22 mm 옮겨져 TOF-IN 은 다시 티칭해야 한다.
"""
from wafer_cell import cell_config as C


def presence(d):
    """거리(mm) -> 'present' | 'absent' | 'invalid'(읽기 실패)."""
    if d is None:
        return "invalid"
    if d >= C.TOF_OUT_OF_RANGE:
        return "absent"
    if d < C.TOF_PRESENT_MM:
        return "present"
    return "absent"
