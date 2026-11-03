#!/usr/bin/env python3
"""
servo_center.py - 서보를 가운데(0도)에 고정해 두는 프로그램 (혼/레이저 방향 맞출 때)
    python3 servo_center.py        -> 0도에 고정. 이 상태에서 혼을 빼서 레이저가 정면을 보게 다시 끼움
    python3 servo_center.py 3      -> 3도에 고정 (미세 조정 값 찾을 때). 찾은 값을 robot_server.py 의 SERVO_TRIM 에 적기
고정 중에 숫자를 입력하고 Enter 하면 그 각도로 바로 움직임. q 를 입력하면 끝
"""
import sys
import time

angle = float(sys.argv[1]) if len(sys.argv) > 1 else 0.0

servo = None
try:
    from robot_server import HardwareServo
    servo = HardwareServo()
    print("하드웨어 PWM으로 고정 (GPIO18)")
except Exception as e:
    print(f"하드웨어 PWM 못 씀 ({e}) -> 소프트웨어 PWM")
    from gpiozero import AngularServo
    servo = AngularServo(18, min_angle=-90, max_angle=90, min_pulse_width=0.0005, max_pulse_width=0.0025)

try:
    while True:
        servo.angle = angle
        print(f"\n지금 {angle:+.1f}도에 고정 중.")
        print("  - 0도일 때 레이저가 정면이 아니면: 혼을 빼서 정면을 보게 다시 끼우세요")
        print("  - 거의 맞는데 살짝 틀어지면: 숫자(예: 2 또는 -2)를 넣어 맞춘 뒤 그 값을 SERVO_TRIM 에 적으세요")
        cmd = input("각도 입력 (q = 끝): ").strip()
        if cmd.lower() == "q":
            break
        try:
            angle = max(-90.0, min(90.0, float(cmd)))
        except ValueError:
            print("숫자를 넣어 주세요")
finally:
    try:
        servo.detach()
        servo.close()
    except Exception:
        pass
    print(f"끝. 마지막 각도 {angle:+.1f}도")
