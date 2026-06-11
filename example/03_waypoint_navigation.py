#!/usr/bin/env python3
"""
03. 좌표 기반 내비게이션

절대 좌표(x, y) 목록을 순서대로 방문한다.
move_position 은 '먼저 목표 방향으로 회전 후 직진' 방식.
"""
from vicpinky_api import WheelController


WAYPOINTS = [
    (0.5, 0.0),
    (0.5, 0.5),
    (0.0, 0.5),
    (0.0, 0.0),
]


def main():
    robot = WheelController()
    if not robot.connect():
        print("모터 연결 실패")
        return

    try:
        robot.reset_odometry()               # 시작점을 (0,0,0) 으로
        for target_x, target_y in WAYPOINTS:
            print(f"목표 이동: ({target_x}, {target_y})")
            robot.move_position(target_x, target_y, speed=0.2)
        print("모든 웨이포인트 방문 완료")
    finally:
        robot.disconnect()


if __name__ == "__main__":
    main()
