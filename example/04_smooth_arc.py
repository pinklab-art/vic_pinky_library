#!/usr/bin/env python3
"""
04. 부드러운 곡선 주행

멈춰서 회전하지 않고, 이동하면서 방향을 부드럽게 꺾어 목표로 이동한다.
"""
from vicpinky_api import WheelController


def main():
    robot = WheelController()
    if not robot.connect():
        print("모터 연결 실패")
        return

    try:
        print("S자 곡선 주행")
        robot.smooth_move_to(0.5, 0.3, speed=0.2)
        robot.smooth_move_to(1.0, 0.0, speed=0.2)
        robot.smooth_move_to(0.0, 0.0, speed=0.3)
        print("곡선 주행 완료")
    finally:
        robot.disconnect()


if __name__ == "__main__":
    main()
