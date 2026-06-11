#!/usr/bin/env python3
"""
02. 사각형 궤적 주행

블로킹 명령(move_distance / turn_relative)으로 0.5m x 0.5m 사각형을 그린다.
각 명령은 목표에 도달할 때까지 반환하지 않는다.
"""
import math
from vicpinky_api import WheelController


def main():
    robot = WheelController()
    if not robot.connect():
        print("모터 연결 실패")
        return

    side_length = 0.5            # m
    turn_angle = math.pi / 2     # 90도 (rad)

    try:
        for i in range(4):
            print(f"{i + 1}번째 변 이동")
            robot.move_distance(side_length, speed=0.15)

            print(f"{i + 1}번째 모퉁이 회전")
            robot.turn_relative(turn_angle, speed=0.5)

        print("사각형 주행 완료")
    finally:
        robot.disconnect()


if __name__ == "__main__":
    main()
