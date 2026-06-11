# vic_pinky_library

Vic Pinky 로봇을 위한 고수준 Python API. ROS 2 없이 순수 Python으로
차동 구동(differential-drive) 주행과 RPLIDAR C1 라이다를 제어합니다.

- **`WheelController`** — ZLAC 모터 드라이버 제어, 기구학, 속도 프로파일,
  오도메트리 추정을 추상화한 주행 컨트롤러
- **`LidarController`** — RPLIDAR C1를 동기·스레드 안전 API로 감싼 라이다 컨트롤러
- **`ZLACDriver`** — ZLAC Modbus RTU 저수준 드라이버

## 설치

GitHub에서 직접 설치:

```bash
pip install git+https://github.com/pinklab-art/vic_pinky_library.git
```

또는 로컬에서 개발용 설치:

```bash
git clone https://github.com/pinklab-art/vic_pinky_library.git
cd vic_pinky_library
pip install -e .
```

의존성: `pyserial`, `rplidarc1`. Linux 전용이며 Python 3.8 이상이 필요합니다.

## 빠른 시작

### 주행

```python
import time
from vicpinky_api import WheelController

robot = WheelController()          # 기본 포트 /dev/motor
robot.connect()

robot.move(linear=0.1, angular=0.0)   # 0.1 m/s 전진
time.sleep(2)
robot.turn_relative(1.57)             # 90도(라디안) 제자리 회전
robot.move_position(1.0, 0.5)         # (x, y) 좌표로 이동

robot.disconnect()                    # 종료 시 항상 호출 (안전 정지)
```

### 라이다

```python
from vicpinky_api import LidarController

lidar = LidarController(port="/dev/rplidar")
lidar.connect()

if lidar.is_obstacle_ahead(distance=0.4):
    print("정면 0.4m 이내 장애물")

print(lidar.get_distance_at(90))      # 90도 방향 거리(m)
lidar.disconnect()
```

## 주요 API

### WheelController

| 메서드 | 설명 |
|--------|------|
| `connect()` / `disconnect()` | 연결·안전 종료 (백그라운드 20Hz 제어 루프 시작/정지) |
| `move(linear, angular)` | 비블로킹 속도 명령 (m/s, rad/s) |
| `turn_to(theta)` / `turn_relative(angle)` | 절대/상대 각도 회전 (블로킹) |
| `move_position(x, y)` | 좌표로 이동 (회전 후 직진, 블로킹) |
| `smooth_move_to(x, y)` | 곡선 궤적으로 부드럽게 이동 (블로킹) |
| `move_distance(d)` | 현재 방향으로 d 미터 직진 (블로킹) |
| `stop()` | 즉시 정지 |
| `get_pose()` / `get_status()` | 현재 pose / 종합 상태 |

### LidarController

| 메서드 | 설명 |
|--------|------|
| `connect()` / `disconnect()` | 연결·종료 |
| `is_obstacle_ahead(distance, fov)` | 정면 cone 내 장애물 여부 |
| `get_distance_at(angle)` | 특정 방향 거리(m) |
| `get_min_distance(start, end)` | 각도 구간 내 최소 거리(m) |
| `get_closest()` | 최근접 점 `(angle, distance)` |
| `get_scan()` / `get_scan_dict()` | 전체 스캔 데이터 |

> 거리는 **미터**, 각도는 센서 기준 **도(0~360)** 단위입니다.

## 예제

`example/` 디렉터리에 단독 실행 가능한 예제 8종이 있습니다 (기본 주행, 사각형 궤적,
좌표 내비게이션, 곡선 주행, 상태 모니터링, 라이다 측정, 장애물 감지, 키보드 텔레오퍼레이션).
자세한 내용은 [`example/README.md`](example/README.md)를 참고하세요.

```bash
python3 example/01_basic_drive.py
```

## 주의

- 포트 기본값은 모터 `/dev/motor`, 라이다 `/dev/rplidar` 입니다. 다르면 생성자
  인자로 지정하세요 (`WheelController(port=...)`, `LidarController(port=...)`).
- 실제 주행 시 주변 여유 공간을 확보하세요.
- RPLIDAR C1의 통신 속도는 **460800** 입니다 (115200 아님).
