#!/usr/bin/env python3
"""
05. 실시간 상태 모니터링

주행하는 동안 백그라운드에서 갱신되는 pose 를 주기적으로 읽어 출력한다.
"""
import time
from vicpinky_api import WheelController


def main():
    robot = WheelController()
    if not robot.connect():
        print("모터 연결 실패")
        return

    try:
        print("전진하며 5초간 pose 모니터링")
        robot.move(0.1, 0.0)

        start = time.time()
        while time.time() - start < 5:
            status = robot.get_status()
            x = status["pose"]["x"]
            y = status["pose"]["y"]
            th = status["pose"]["theta"]
            print(f"\rPose: x={x:+.2f}  y={y:+.2f}  th={th:+.2f}", end="")
            time.sleep(0.1)

        robot.stop()
    finally:
        robot.disconnect()
        print("\n모니터링 종료")


if __name__ == "__main__":
    main()
