#!/usr/bin/env python3
"""
10. 키보드 텔레오퍼레이션 (WASD) + 속도 제어

키보드로 로봇을 실시간 조종한다. 외부 패키지 없이 표준 라이브러리만 사용
(termios / tty / select) — 리눅스 터미널 전용.

키 안내 (각 줄 '/' 뒤는 같은 기능의 대체 키):
    w          전진
    s          후진
    a          좌회전
    d          우회전
    space      정지
    + / =      선속도 증가   (0.05 ~ 0.60 m/s)
    - / _      선속도 감속
    ] / }      각속도 증가   (0.20 ~ 2.00 rad/s)
    [ / {      각속도 감속
    q          종료

키를 누르면 '그 방향으로 현재 속도만큼' 움직인다(누적 아님). 멈추려면 space.
선속도/각속도는 언제든 조절할 수 있고, 움직이는 중에도 바로 반영된다.
"""
import sys
import termios
import tty
import select

from vicpinky_api import WheelController

LIN_MIN, LIN_MAX, LIN_INC = 0.05, 0.60, 0.05      # 선속도 범위/증분 (m/s)
ANG_MIN, ANG_MAX, ANG_INC = 0.20, 2.00, 0.10      # 각속도 범위/증분 (rad/s)


def get_key(timeout=0.1):
    """timeout 초 동안 키 입력을 기다린다. 없으면 '' 반환 (논블로킹)."""
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ready, _, _ = select.select([sys.stdin], [], [], timeout)
        if ready:
            return sys.stdin.read(1)
        return ""
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def main():
    robot = WheelController()
    if not robot.connect():
        print("모터 연결 실패")
        return

    lin_speed = 0.15      # 현재 선속도 설정 (m/s)
    ang_speed = 0.50      # 현재 각속도 설정 (rad/s)
    lin = 0.0             # 현재 명령 선속도
    ang = 0.0             # 현재 명령 각속도

    print(
        "─ 키 안내 ──────────────────\n"
        "  w          전진\n"
        "  s          후진\n"
        "  a          좌회전\n"
        "  d          우회전\n"
        "  space      정지\n"
        "  + / =      선속도 증가\n"
        "  - / _      선속도 감속\n"
        "  ] / }      각속도 증가\n"
        "  [ / {      각속도 감속\n"
        "  q          종료\n"
        "────────────────────────────\n"
    )

    def show():
        print(f"\r명령 lin={lin:+.2f} ang={ang:+.2f} | "
              f"설정 선속도={lin_speed:.2f} m/s 각속도={ang_speed:.2f} rad/s     ",
              end="", flush=True)

    try:
        show()
        while True:
            key = get_key()

            # --- 방향 (WASD, 현재 속도 설정값으로 세팅) ---
            if key == "w":
                lin, ang = lin_speed, 0.0
            elif key == "s":
                lin, ang = -lin_speed, 0.0
            elif key == "a":
                lin, ang = 0.0, ang_speed
            elif key == "d":
                lin, ang = 0.0, -ang_speed
            elif key == " ":
                lin, ang = 0.0, 0.0

            # --- 속도 제어 ---
            elif key in ("+", "="):
                lin_speed = clamp(lin_speed + LIN_INC, LIN_MIN, LIN_MAX)
            elif key in ("-", "_"):
                lin_speed = clamp(lin_speed - LIN_INC, LIN_MIN, LIN_MAX)
            elif key in ("]", "}"):
                ang_speed = clamp(ang_speed + ANG_INC, ANG_MIN, ANG_MAX)
            elif key in ("[", "{"):
                ang_speed = clamp(ang_speed - ANG_INC, ANG_MIN, ANG_MAX)

            elif key == "q":
                break

            # 움직이는 중 속도를 바꿨으면 현재 방향에 새 속도 반영
            if lin > 0:
                lin = lin_speed
            elif lin < 0:
                lin = -lin_speed
            if ang > 0:
                ang = ang_speed
            elif ang < 0:
                ang = -ang_speed

            robot.move(lin, ang)
            show()
    except KeyboardInterrupt:
        pass
    finally:
        robot.stop()
        robot.disconnect()
        print("\n종료")


if __name__ == "__main__":
    main()
