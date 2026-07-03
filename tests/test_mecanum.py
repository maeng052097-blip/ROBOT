"""tests/test_mecanum.py — 메카넘 믹싱 단위테스트(하드웨어 불필요).

실행: py -3.13 tests/test_mecanum.py
"""
import sys
import pathlib

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))

from common.mecanum import mix  # noqa: E402


def main():
    print("test_mecanum:")

    # 전진: 4휠 동일 +
    assert mix(50, 0, 0) == (50, 50, 50, 50), mix(50, 0, 0)
    print("  OK forward (all wheels equal +)")

    # 우회전(CW): 좌측(FL,RL)+ / 우측(FR,RR)-
    assert mix(0, 0, 40) == (40, -40, 40, -40), mix(0, 0, 40)
    print("  OK turn-right (left + / right -)")

    # 우측 strafe: FL+,FR-,RL-,RR+
    assert mix(0, 40, 0) == (40, -40, -40, 40), mix(0, 40, 0)
    print("  OK strafe-right (FL+,FR-,RL-,RR+)")

    # 정지
    assert mix(0, 0, 0) == (0, 0, 0, 0)
    print("  OK stop")

    # 합성 초과 시 비례 축소(방향 보존). mix(80,80,80): raw=(240,-80,80,80)->scale 100/240
    fl, fr, rl, rr = mix(80, 80, 80)
    assert fl == 100 and fr == -33 and rl == 33 and rr == 33, (fl, fr, rl, rr)
    assert max(abs(fl), abs(fr), abs(rl), abs(rr)) <= 100
    print("  OK normalize when sum exceeds 100% (direction preserved)")

    print("OK (all passed)")


if __name__ == "__main__":
    main()
