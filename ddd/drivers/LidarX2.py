"""
LidarX2.py — YDLIDAR X2 전용 드라이버
- 보레이트: 115200 (X2 전용)
- 패킷 헤더: 0xAA 0x55
- X2는 전원 연결 시 자동으로 스캔 시작
- nesnes/YDLidarX2_python 프로토콜 참고
"""
import time
import threading

import serial


class LidarX2:
    def __init__(self, port, baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.serial = None
        self.thread = None
        self.running = False
        self.measures = []  # [(angle, distance), ...]
        self.last_update = None  # 마지막으로 데이터를 수신한 시각(time.time())
        self.lock = threading.Lock()

    def open(self):
        """시리얼 포트 열기"""
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=1.0
            )
            self.running = True
            self.thread = threading.Thread(target=self._read_loop, daemon=True)
            self.thread.start()
            return True
        except Exception as e:
            print(f"포트 열기 실패: {e}")
            return False

    def close(self):
        """시리얼 포트 닫기"""
        self.running = False
        if self.thread:
            self.thread.join(timeout=2)
        if self.serial and self.serial.is_open:
            self.serial.close()

    def getMeasures(self):
        """현재 스캔 데이터 반환: [(angle, distance), ...]"""
        with self.lock:
            return list(self.measures)

    def getDistanceDict(self):
        """각도(0~359) → 거리(mm) 딕셔너리 반환"""
        result = {}
        with self.lock:
            for angle, distance in self.measures:
                int_angle = int(round(angle)) % 360
                result[int_angle] = distance
        return result

    def seconds_since_update(self):
        """마지막 데이터 수신 후 경과 시간(초). 한 번도 못 받았으면 None."""
        if self.last_update is None:
            return None
        return time.time() - self.last_update

    def is_fresh(self, max_age=0.5):
        """최근 max_age 초 안에 데이터를 받았는가(= LiDAR 가 살아서 스캔 중인가)."""
        age = self.seconds_since_update()
        return age is not None and age <= max_age

    @staticmethod
    def _parse_packets(buf):
        """버퍼에서 가능한 모든 YDLIDAR X2 패킷을 파싱한다. (동작은 기존과 동일)

        반환: (measures, 남은_버퍼)
          measures = [(angle_deg, distance_mm), ...]  (distance > 0 인 것만)
        패킷 구조: 헤더 0xAA 0x55 | type | sample_count | start_raw(2,LE)
                  | end_raw(2,LE) | (8,9 미사용) | distances(sample_count*2, LE)
                  angle = (raw >> 1) / 64.0
        하드웨어 없이 단위 테스트할 수 있도록 순수 함수로 분리했다.
        """
        measures = []
        while len(buf) >= 10:
            # 헤더 0xAA 0x55 찾기
            header_pos = -1
            for i in range(len(buf) - 1):
                if buf[i] == 0xAA and buf[i + 1] == 0x55:
                    header_pos = i
                    break

            if header_pos < 0:
                buf = buf[-1:]  # 마지막 바이트만 유지(다음 수신분과 이어붙임)
                break
            if header_pos > 0:
                buf = buf[header_pos:]  # 헤더 앞 잡음 버림
            if len(buf) < 10:
                break

            sample_count = buf[3]
            start_angle_raw = buf[4] | (buf[5] << 8)
            end_angle_raw = buf[6] | (buf[7] << 8)

            packet_size = 10 + sample_count * 2
            if len(buf) < packet_size:
                break  # 패킷이 아직 다 안 들어옴 -> 보류

            start_angle = (start_angle_raw >> 1) / 64.0
            end_angle = (end_angle_raw >> 1) / 64.0
            if end_angle < start_angle:
                angle_diff = (360 + end_angle - start_angle)
            else:
                angle_diff = (end_angle - start_angle)

            for j in range(sample_count):
                idx = 10 + j * 2
                distance = buf[idx] | (buf[idx + 1] << 8)
                if sample_count > 1:
                    angle = start_angle + (angle_diff / (sample_count - 1)) * j
                else:
                    angle = start_angle
                angle = angle % 360.0
                if distance > 0:
                    measures.append((angle, distance))

            buf = buf[packet_size:]  # 처리한 패킷 제거

        return measures, buf

    def _read_loop(self):
        """백그라운드 스레드: 시리얼 수신 → 파싱 → 측정값 누적."""
        buf = bytearray()
        while self.running:
            try:
                data = self.serial.read(128)
                if not data:
                    continue
                self.last_update = time.time()  # 데이터 수신 = LiDAR 살아있음
                buf.extend(data)

                new_measures, buf = self._parse_packets(buf)
                if new_measures:
                    with self.lock:
                        self.measures.extend(new_measures)
                        # 최근 720개만 유지 (약 2회전)
                        if len(self.measures) > 720:
                            self.measures = self.measures[-720:]

            except serial.SerialException:
                break
            except Exception:
                continue
