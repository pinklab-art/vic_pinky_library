# 예제 (example)

`vicpinky_api` 라이브러리 사용 예제 모음입니다. 모든 스크립트는 단독 실행 가능합니다.

```bash
# 패키지 루트(setup.py 위치)에서 먼저 설치
pip install -e .

# 예제 실행
python3 example/01_basic_drive.py
```

## 목록

| 파일 | 내용 | 필요 하드웨어 |
|------|------|---------------|
| `01_basic_drive.py` | 전진 · 회전 · 정지 기본 | 모터 |
| `02_square_path.py` | 사각형 궤적 (블로킹 명령) | 모터 |
| `03_waypoint_navigation.py` | 좌표 기반 내비게이션 | 모터 |
| `04_smooth_arc.py` | 부드러운 곡선 주행 | 모터 |
| `05_status_monitor.py` | 실시간 pose 모니터링 | 모터 |
| `06_lidar_basic_read.py` | 라이다 거리 측정 | 라이다 |
| `07_obstacle_detection.py` | 정면 장애물 감지 | 라이다 |
| `08_keyboard_teleop.py` | 키보드 WASD 실시간 조종 + 속도 제어 (표준 라이브러리, 추가 설치 불필요) | 모터 |

## 주의

- 포트 기본값은 `/dev/motor`, `/dev/rplidar` 입니다. 다르면 생성자 인자로 지정하세요.
- 실행은 주변 여유 공간을 확보하고 진행하세요.
