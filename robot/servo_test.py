#!/usr/bin/env python3
"""
servo_test.py - 서보만 따로 테스트 (라즈베리파이에서 실행)
하드웨어 PWM(config.txt 의 dtoverlay=pwm-2chan)이 켜져 있으면 그걸로, 아니면 소프트웨어 PWM으로 움직임
서보 신호선 = GPIO18 (물리 핀 12)
"""
import glob
import os
import time

STEPS = [("가운데", 0), ("왼쪽", -45), ("오른쪽", 45), ("가운데", 0)]


def _pick_pwm_chip():
    """서보용 PWM 칩 고르기: 채널 4개짜리 중 냉각팬이 쓰는 칩은 빼고, PWM0(주소 ...98000)을 우선"""
    import glob
    import os
    good = []
    for c in sorted(glob.glob("/sys/class/pwm/pwmchip*")):
        try:
            npwm = int(open(c + "/npwm").read())
            dev = os.path.realpath(c + "/device")
            names = " ".join(os.listdir(c + "/device"))
        except (OSError, ValueError):
            continue
        if npwm < 4 or "fan" in names or "fan" in dev:
            continue
        good.append((0 if "98000" in dev else 1, c))
    if not good:
        raise RuntimeError("PWM 장치 없음 (config.txt 에 dtoverlay=pwm-2chan 넣고 재부팅)")
    return sorted(good)[0][1]


def hw_servo():
    chips = [_pick_pwm_chip()]
    base = chips[0] + "/pwm2"

    def w(path, v):
        with open(path, "w") as f:
            f.write(str(int(v)))

    if not os.path.exists(base):
        w(chips[0] + "/export", 2)
    for _ in range(50):
        if os.access(base + "/period", os.W_OK):
            break
        time.sleep(0.02)
    w(base + "/enable", 0)
    w(base + "/duty_cycle", 0)
    w(base + "/period", 20_000_000)
    w(base + "/enable", 1)
    try:                                              # GPIO18 을 PWM 기능(a3)으로 확실히 전환
        import subprocess
        subprocess.run(["pinctrl", "set", "18", "a3"], timeout=3, capture_output=True)
    except Exception:
        pass
    print(f"하드웨어 PWM 사용 ({base}, {os.path.realpath(chips[0] + '/device')})")

    def set_angle(deg):
        w(base + "/duty_cycle", (500 + (deg + 90) / 180 * 2000) * 1000)

    def close():
        w(base + "/duty_cycle", 0)
        w(base + "/enable", 0)

    return set_angle, close


def sw_servo():
    from gpiozero import AngularServo
    s = AngularServo(18, min_angle=-90, max_angle=90, min_pulse_width=0.0005, max_pulse_width=0.0025)
    print("소프트웨어 PWM 사용")

    def set_angle(deg):
        s.angle = deg

    def close():
        s.detach()
        s.close()

    return set_angle, close


import sys
if len(sys.argv) > 1 and sys.argv[1] == "sw":          # python3 servo_test.py sw -> 소프트웨어 PWM 강제
    set_angle, close = sw_servo()
else:
    try:
        set_angle, close = hw_servo()
    except Exception as e:
        print(f"하드웨어 PWM 못 씀: {e}")
        set_angle, close = sw_servo()

try:
    for _ in range(2):
        for name, deg in STEPS:
            print(f"  {name} ({deg}도)")
            set_angle(deg)
            time.sleep(1.2)
finally:
    close()
    print("끝. 서보가 안 움직였으면 25J-24J 사이가 5V인지, 26J와 파이 핀12가 이어졌는지 확인하세요.")
