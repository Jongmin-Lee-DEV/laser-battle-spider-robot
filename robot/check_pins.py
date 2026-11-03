#!/usr/bin/env python3
"""
check_pins.py - robot_server.py 실행 전 점검 (라즈베리파이에서 Thonny로 열어 Run)

확인하는 것
    1. /boot/firmware/config.txt 에 SPI 설정 두 줄이 있는지
    2. SPI 장치(/dev/spidev0.0)가 있는지
    3. 로봇이 쓰는 핀을 하나씩 잡아 보고, 막힌 핀이 어디인지
    4. 같은 핀을 쓸 만한 다른 파이썬 프로그램이 켜져 있는지
    5. 카메라 라이브러리가 있는지
"""
import os
import subprocess

PINS = {
    7: "오른쪽 모터 앞", 8: "오른쪽 모터 뒤", 25: "오른쪽 모터 속도",
    20: "왼쪽 모터 앞", 21: "왼쪽 모터 뒤", 16: "왼쪽 모터 속도",
    23: "레이저", 18: "서보", 24: "MCP3008 CS",
}
problems = []


def title(t):
    print("\n" + "=" * 10, t, "=" * 10)


# 1. config.txt
title("1. config.txt")
cfg_path = "/boot/firmware/config.txt"
if not os.path.exists(cfg_path):
    cfg_path = "/boot/config.txt"
try:
    lines = [l.strip() for l in open(cfg_path, encoding="utf-8", errors="ignore")]
    active = [l for l in lines if l and not l.startswith("#")]
    spi_on = any(l.replace(" ", "") == "dtparam=spi=on" for l in active)
    no_cs = any(l.replace(" ", "").startswith("dtoverlay=spi0-0cs") for l in active)
    print(f"  파일: {cfg_path}")
    print(f"  dtparam=spi=on      : {'있음' if spi_on else '없음  <-- 추가 필요'}")
    print(f"  dtoverlay=spi0-0cs  : {'있음' if no_cs else '없음  <-- 추가 필요 (모터 GPIO7, 8 충돌 원인)'}")
    if not spi_on:
        problems.append("config.txt 에 dtparam=spi=on 추가")
    if not no_cs:
        problems.append("config.txt 에 dtoverlay=spi0-0cs 추가")
except OSError as e:
    print(f"  읽기 실패: {e}")

# 2. SPI 장치
title("2. SPI 장치")
spidevs = sorted(f for f in os.listdir("/dev") if f.startswith("spidev"))
print("  " + (", ".join(spidevs) if spidevs else "없음"))
if "spidev0.0" not in spidevs:
    problems.append("/dev/spidev0.0 없음 -> SPI 설정 후 재부팅")

# 3. 핀 하나씩 잡아 보기
title("3. 핀 점검")
pin_info = {}
try:
    out = subprocess.run(["pinctrl", "get", ",".join(str(p) for p in PINS)],
                         capture_output=True, text=True, timeout=5).stdout
    for line in out.strip().splitlines():
        print("  " + line.strip())
        num = line.strip().split(":")[0].strip()
        if num.isdigit():
            pin_info[int(num)] = line
        if "SPI0_CE" in line:
            problems.append(f"SPI가 핀을 잡고 있음: {line.strip()}  -> dtoverlay=spi0-0cs 추가 후 재부팅")
except Exception as e:
    print(f"  pinctrl 실행 실패: {e}")

print()
try:
    from gpiozero import DigitalOutputDevice
    for pin, name in PINS.items():
        if pin == 18 and "PWM" in pin_info.get(18, ""):
            print(f"  GPIO18 {name:<12} 하드웨어 PWM 사용 중 (정상)")
            continue
        try:
            d = DigitalOutputDevice(pin)
            d.close()
            print(f"  GPIO{pin:<2} {name:<12} OK")
        except Exception as e:
            print(f"  GPIO{pin:<2} {name:<12} 막힘  ({e})")
            problems.append(f"GPIO{pin} ({name}) 사용 불가")
except ImportError:
    print("  gpiozero 없음: sudo apt install python3-gpiozero python3-lgpio")
    problems.append("gpiozero 설치")

# 4. 다른 파이썬 프로그램
title("4. 켜져 있는 파이썬 프로그램")
me = os.getpid()
try:
    out = subprocess.run(["ps", "-eo", "pid,args"], capture_output=True, text=True).stdout
    found = False
    for line in out.splitlines()[1:]:
        pid, _, args = line.strip().partition(" ")
        if "python" in args and int(pid) != me and "thonny" not in args.lower() \
                and "check_pins" not in args:
            print(f"  PID {pid}: {args}")
            found = True
    if found:
        print("  -> 위 프로그램이 핀을 쓰고 있을 수 있음. 끄려면: sudo kill PID번호")
    else:
        print("  없음")
except Exception as e:
    print(f"  확인 실패: {e}")

# 4-2. MCP3008 응답
title("4-2. MCP3008 응답 (패널 값 원본)")
try:
    import spidev
    from gpiozero import DigitalOutputDevice
    spi = spidev.SpiDev()
    spi.open(0, 0)
    spi.max_speed_hz = 250_000   # 선이 길어서 느리게 (값 튀는 것 방지)
    spi.mode = 0
    try:
        spi.no_cs = True
    except (OSError, AttributeError):
        pass
    cs = DigitalOutputDevice(24, initial_value=True)
    raws, vals = [], []
    for ch in range(8):
        cs.off()
        r = spi.xfer2([1, (8 + ch) << 4, 0])
        cs.on()
        raws.append(r)
        vals.append(((r[1] & 3) << 8) | r[2])
    spi.close()
    cs.close()
    print("  채널값:", vals)
    print("  원본  :", [bytes(r).hex() for r in raws])
    if all(v == 0 for v in vals):
        print("  -> 전부 0: 칩이 응답을 안 함")
        print("     칩 전원(3J=3.3V), VREF(4열 위쪽=3.3V), MISO 선(7열 위쪽 -> 핀21), CS 선(9J -> 핀18), 칩 방향 확인")
        problems.append("MCP3008 값이 전부 0 (칩 전원 / VREF / MISO / CS 확인)")
    elif all(v == 1023 for v in vals):
        print("  -> 전부 1023: MISO 선이 떠 있거나 칩이 선택되지 않음 (CS 선 9J -> 핀18, MISO 7열 -> 핀21)")
        problems.append("MCP3008 값이 전부 1023 (MISO / CS 확인)")
    else:
        print("  -> 칩 응답 정상 (값이 0인 채널은 그 패널 선만 확인)")
except Exception as e:
    print(f"  읽기 실패: {e}")
    problems.append(f"MCP3008 읽기 실패: {e}")

# 5. 카메라
title("5. 카메라")
try:
    import picamera2  # noqa: F401
    import simplejpeg  # noqa: F401
    print("  picamera2 / simplejpeg 있음")
    try:
        from picamera2 import Picamera2
        cams = Picamera2.global_camera_info()
        print(f"  연결된 카메라: {len(cams)}개 {[c.get('Model') for c in cams]}")
        if not cams:
            problems.append("카메라가 인식되지 않음 (리본 케이블 확인)")
    except Exception as e:
        print(f"  카메라 확인 실패: {e}")
except ImportError as e:
    print(f"  없음 ({e})  -> sudo apt install python3-picamera2")
    problems.append("picamera2 설치 (영상 없이도 로봇은 동작)")

# 결과
title("결과")
if problems:
    for p in problems:
        print("  - " + p)
    if any("config.txt" in p or "재부팅" in p for p in problems):
        print("\n  config.txt 고친 뒤에는 꼭  sudo reboot")
else:
    print("  모두 정상! robot_server.py 를 실행하세요.")
