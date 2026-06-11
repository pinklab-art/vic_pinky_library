#!/usr/bin/env python3
"""
06. 라이다 기본 거리 측정

라이다를 연결하고 주요 방향(정면/좌/우/후)의 거리를 한 번 읽는다.
거리 단위 m, 각도 단위 deg (정면 0°, 반시계 +).
"""
from vicpinky_api import LidarController


def main():
    lidar = LidarController()          # 기본 포트 /dev/rplidar, 460800
    if not lidar.connect():
        print("라이다 연결 실패")
        return

    try:
        print(f"정면 : {lidar.get_distance_at(0):.2f} m")
        print(f"좌측 : {lidar.get_distance_at(90):.2f} m")
        print(f"후면 : {lidar.get_distance_at(180):.2f} m")
        print(f"우측 : {lidar.get_distance_at(270):.2f} m")

        angle, dist = lidar.get_closest()
        print(f"가장 가까운 점: {angle}° / {dist:.2f} m")
    finally:
        lidar.disconnect()


if __name__ == "__main__":
    main()
