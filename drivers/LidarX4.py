"""LidarX4.py — YDLIDAR X4 드라이버 (LidarX2 와 동일 인터페이스).

확인된 사실(공식 매뉴얼/커뮤니티):
  - 통신: 128000 bps, 8N1.                    (X2 는 115200)
  - 스캔 시작 명령: 0xA5 0x60 (연속 포인트클라우드 출력 시작).
  - 스캔 데이터 패킷은 YDLIDAR 표준 계열(헤더 0xAA 0x55, Distance(mm)=raw/4,
    Angle=(raw>>1)/64) 로 X2 와 동일 -> 파서(_parse_packets) 그대로 재사용.

가정/불명(실험으로 검증):
  - 거리 raw/4 스케일은 X2 와 동일하다고 보고 사용 -> probe 의 dist 를 줄자와 대조해 확인.
  - 거리의존 각도보정(AngCorrect)은 X2 드라이버와 동일하게 생략(±수 deg, 본 실험 영향 적음).
  - 일부 X4 보드는 모터가 DTR 로 제어됨. 데이터가 안 오면 _on_port_open 의 DTR 줄을 시도.

출처: YDLIDAR X4 Development Manual (baud 128000, start 0xA5 0x60).
"""
from drivers.LidarX2 import LidarX2

CMD_SYNC = 0xA5
CMD_START_SCAN = 0x60
CMD_STOP_SCAN = 0x65


class LidarX4(LidarX2):
    """YDLIDAR X4. baud 128000 + 스캔 시작 명령만 다르고 나머지는 X2 와 동일."""

    def __init__(self, port, baudrate=128000, send_start=True, dist_scale=1.0, dist_offset_mm=0.0):
        super().__init__(port, baudrate, dist_scale, dist_offset_mm)
        self.send_start = send_start

    def _on_port_open(self):
        """X4: 스캔 시작 명령(0xA5 0x60) 전송. (자동 스트리밍 보드면 무시되어도 무해)"""
        if not self.send_start:
            return
        try:
            # [문제해결] 데이터가 전혀 안 오면 아래 한 줄의 주석을 풀어 DTR 로 모터를
            #           켜 보세요(보드에 따라 False/True 가 다름).
            # self.serial.dtr = False
            self.serial.reset_input_buffer()
            self.serial.write(bytes([CMD_SYNC, CMD_START_SCAN]))
            self.serial.flush()
        except Exception as e:
            print(f"[X4] 스캔 시작 명령 전송 실패(계속 진행): {e}")

    def close(self):
        """정지 명령(0xA5 0x65) 후 닫기."""
        try:
            if self.serial and self.serial.is_open:
                self.serial.write(bytes([CMD_SYNC, CMD_STOP_SCAN]))
                self.serial.flush()
        except Exception:
            pass
        super().close()
