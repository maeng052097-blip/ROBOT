"""
LidarX2.py — YDLIDAR X2 전용 드라이버
- 보레이트: 115200 (X2 전용)
- 패킷 헤더: 0xAA 0x55
- X2는 전원 연결 시 자동으로 스캔 시작
- nesnes/YDLidarX2_python 프로토콜 참고
"""
import serial
import threading
import math


class LidarX2:
    def __init__(self, port, baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.serial = None
        self.thread = None
        self.running = False
        self.measures = []  # [(angle, distance), ...]
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

    def _read_loop(self):
        """백그라운드 스레드: 시리얼 데이터 수신 및 파싱"""
        buf = bytearray()

        while self.running:
            try:
                # 시리얼에서 데이터 읽기
                data = self.serial.read(128)
                if not data:
                    continue
                buf.extend(data)

                # 패킷 파싱
                while len(buf) >= 10:
                    # 패킷 헤더 0xAA 0x55 찾기
                    header_pos = -1
                    for i in range(len(buf) - 1):
                        if buf[i] == 0xAA and buf[i + 1] == 0x55:
                            header_pos = i
                            break

                    if header_pos < 0:
                        buf = buf[-1:]  # 마지막 바이트만 유지
                        break

                    if header_pos > 0:
                        buf = buf[header_pos:]  # 헤더 앞 데이터 버림

                    if len(buf) < 10:
                        break

                    # 패킷 구조 파싱
                    packet_type = buf[2]
                    sample_count = buf[3]
                    start_angle_raw = buf[4] | (buf[5] << 8)
                    end_angle_raw = buf[6] | (buf[7] << 8)

                    # 패킷 전체 크기 확인
                    packet_size = 10 + sample_count * 2
                    if len(buf) < packet_size:
                        break

                    # 각도 계산 (고정소수점 → 실수)
                    start_angle = (start_angle_raw >> 1) / 64.0
                    end_angle = (end_angle_raw >> 1) / 64.0

                    # 각도 차이 계산 (360도 경계 처리)
                    if end_angle < start_angle:
                        angle_diff = (360 + end_angle - start_angle)
                    else:
                        angle_diff = (end_angle - start_angle)

                    # 각 샘플의 거리 및 각도 계산
                    new_measures = []
                    for j in range(sample_count):
                        idx = 10 + j * 2
                        distance = buf[idx] | (buf[idx + 1] << 8)

                        # 각도 보간
                        if sample_count > 1:
                            angle = start_angle + (angle_diff / (sample_count - 1)) * j
                        else:
                            angle = start_angle

                        angle = angle % 360.0

                        if distance > 0:
                            new_measures.append((angle, distance))

                    # 측정 데이터 업데이트
                    with self.lock:
                        self.measures.extend(new_measures)
                        # 최근 720개만 유지 (약 2프레임)
                        if len(self.measures) > 720:
                            self.measures = self.measures[-720:]

                    # 처리한 패킷 제거
                    buf = buf[packet_size:]

            except serial.SerialException:
                break
            except Exception:
                continue
