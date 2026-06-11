#!/usr/bin/env python3
"""
07. 정면 장애물 감지

정면 부채꼴(fov) 안에 일정 거리 이내 장애물이 있는지 주기적으로 확인한다.
Ctrl+C 로 종료.
"""
import time
from vicpinky_api import LidarController


def main():
    lidar = LidarController()
    if not lidar.connect():
        print("라이다 연결 실패")
        return

    try:
        while True:
            if lidar.is_obstacle_ahead(distance=0.4, fov=40):
                d = lidar.get_min_distance(340, 20)
                print(f"\r장애물 감지! 정면 {d:.2f} m   ", end="")
            else:
                print("\r정면 안전                    ", end="")
            time.sleep(0.2)
    except KeyboardInterrupt:
        pass
    finally:
        lidar.disconnect()
        print()


if __name__ == "__main__":
    main()
