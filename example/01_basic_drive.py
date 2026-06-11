#!/usr/bin/env python3
"""
01. 기본 주행 및 정지

로봇을 잠깐 전진 → 제자리 회전 → 정지시키는 가장 기본적인 예제.
"""
import time
from vicpinky_api import WheelController


def main():
    robot = WheelController()          # 기본 포트 /dev/motor
    if not robot.connect():
        print("모터 연결 실패")
        return

    try:
        print("1) 0.1 m/s 로 2초간 전진")
        robot.move(linear=0.1, angular=0.0)
        time.sleep(2)

        print("2) 0.5 rad/s 로 1초간 제자리 회전")
        robot.move(linear=0.0, angular=0.5)
        time.sleep(1)

        print("3) 정지")
        robot.stop()
    finally:
        robot.disconnect()             # 종료는 항상 보장


if __name__ == "__main__":
    main()
