"""모터 제어용 공용 시리얼 헬퍼.

Arduino로 명령 문자열을 보내는 로직을 한 곳에서 관리한다.
모터2개_시리얼테스트.py 와 웹캠_LiDAR_주행제어.py 가 공유한다.
"""


def send_command(arduino, command):
    """명령 문자열 + '\\n' 을 Arduino로 전송."""
    arduino.write((command + "\n").encode("utf-8"))
    arduino.flush()
    print(f"전송: {command}")
