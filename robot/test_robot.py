#!/usr/bin/env python3
"""
거미로봇 부품 테스트 (Raspberry Pi 5, 납땜 기판 최종 설계 기준)

[파이 핀 연결]
  핀1  3.3V    -> 기판 3J (MCP3008 VDD)
  핀6  GND     -> 기판 5J
  핀23 SCLK    -> 기판 6J
  핀21 MISO    -> 기판 7J
  핀19 MOSI    -> 기판 8J
  핀18 GPIO24  -> 기판 9J (MCP3008 CS, 코드가 직접 제어)
  핀16 GPIO23  -> 기판 20J (레이저 신호)
  핀12 GPIO18  -> 서보 주황 (직결)
  핀2  5V      -> 레이저 빨강,  핀14 GND -> 레이저 검정
  모터 기판 J9 +5V -> 기판 25I (서보 전원)
  패널 P1~P8 + -> 기판 3A~10A (CH0~CH7),  - -> 12A~19A

[준비]
  /boot/firmware/config.txt 에  dtparam=spi=on  과  dtoverlay=spi0-0cs  두 줄
  sudo apt install python3-spidev python3-gpiozero python3-lgpio

[실행]
  python3 test_robot.py          -> 메뉴
  python3 test_robot.py check    -> 이름으로 바로 실행 (check/adc/laser/servo/spike/blink)
"""
import math
import sys
import time

# ---------------- 설정 ----------------
LASER_PIN = 23        # GPIO23 = 물리 핀 16
SERVO_PIN = 18        # GPIO18 = 물리 핀 12
CS_PIN = 24           # GPIO24 = 물리 핀 18 (모터가 GPIO7, 8을 쓰므로 CE0 대신 사용)
SERVO_LIMIT = 45      # 서보 좌우 최대 각도

PANELS = [0, 1, 2, 3, 4, 6, 7]   # 사용할 MCP3008 채널. P6(CH5)은 떼어내서 뺌
PANEL_NAMES = {                     # 패널 위치에 맞게 이름을 바꿔 쓰세요
    0: "P1", 1: "P2", 2: "P3", 3: "P4",
    4: "P5", 5: "P6", 6: "P7", 7: "P8",
}

# 깜빡임 감지(메뉴 6) 설정 - 테스트로 잘 맞았던 값
FREQ = 25.0           # 레이저 깜빡임 주파수(Hz)
WINDOW = 0.2          # 한 번 판정에 쓰는 시간(초)
RATE = 250.0          # 패널 측정 횟수(초당, 패널마다)
CONFIRM = 3           # 몇 구간 연속이어야 HIT
KEEP = 0.5            # 첫 구간 이후엔 기준의 이 비율만 넘어도 인정
PHASE_TOL = 60.0      # 연속 구간 타이밍 허용 차이(도)


def name(ch):
    return PANEL_NAMES.get(ch, f"CH{ch}")


# ---------------- MCP3008 ----------------
def open_adc():
    """MCP3008 연결. CS는 GPIO24로 코드가 직접 제어. (read(ch), close()) 반환"""
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
    cs = DigitalOutputDevice(CS_PIN, initial_value=True)   # HIGH = 선택 안 됨

    def read(ch):
        cs.off()
        r = spi.xfer2([1, (8 + ch) << 4, 0])
        cs.on()
        return ((r[1] & 3) << 8) | r[2]

    def close():
        spi.close()
        cs.close()

    return read, close


# ---------------- 1. 납땜 후 점검 ----------------
def test_check():
    """패널 8개를 한 번씩 읽어서 납땜·연결 상태를 진단"""
    read, close = open_adc()
    try:
        print("\n패널 연결 점검 (각 패널 0.5초 평균)")
        print("레이저는 끄고, 평소 조명 상태로 두세요.\n")
        ok = 0
        for ch in PANELS:
            vals = []
            end = time.monotonic() + 0.5
            while time.monotonic() < end:
                vals.append(read(ch))
                time.sleep(0.005)
            avg = sum(vals) / len(vals)
            spread = max(vals) - min(vals)
            if avg < 3:
                msg = "0 근처: 선 빠짐 / + 와 GND 닿음 / 너무 어두움"
            elif avg > 1015:
                msg = "꽉 참: 부하 저항 빠짐 또는 너무 큼 / 3.3V와 닿음"
            elif spread > 40:
                msg = "값이 많이 흔들림: 접촉 불량 의심"
            else:
                msg = "정상"
                ok += 1
            print(f"  {name(ch):>3} (CH{ch}): 평균 {avg:6.1f}, 흔들림 {spread:3d}  -> {msg}")
        print(f"\n정상 {ok}개 / {len(PANELS)}개")
        print("이상한 패널은 손으로 가렸다 치워 보세요. 그 패널만 값이 바뀌면 번호는 맞게 꽂힌 거예요.")
    finally:
        close()


# ---------------- 2. 패널 값 실시간 ----------------
def test_adc():
    read, close = open_adc()
    print("\n패널 값 실시간 (0~1023). 가리면 내려가고 비추면 올라가야 정상. Ctrl+C로 종료")
    try:
        while True:
            vals = [read(ch) for ch in PANELS]
            print("  ".join(f"{name(ch)}:{v:4d}" for ch, v in zip(PANELS, vals)), end="\r")
            time.sleep(0.1)
    except KeyboardInterrupt:
        print()
    finally:
        close()


# ---------------- 3. 레이저 ----------------
def test_laser():
    from gpiozero import LED
    laser = LED(LASER_PIN, initial_value=False)
    print("\n레이저를 벽 쪽으로 향하게 두세요. 1초 간격으로 5번 깜빡입니다.")
    try:
        for i in range(5):
            laser.on()
            print(f"  {i+1}: ON")
            time.sleep(1)
            laser.off()
            print(f"  {i+1}: OFF")
            time.sleep(1)
    finally:
        laser.off()
        laser.close()
        print("종료: 레이저가 꺼져 있는지 확인하세요")


# ---------------- 4. 서보 ----------------
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


class _HwServo:
    """하드웨어 PWM(config.txt 의 dtoverlay=pwm-2chan)으로 GPIO18 서보 제어"""
    def __init__(self):
        import os
        chips = [_pick_pwm_chip()]
        self.base = chips[0] + "/pwm2"
        if not os.path.exists(self.base):
            self._w(chips[0] + "/export", 2)
        for _ in range(50):
            if os.access(self.base + "/period", os.W_OK):
                break
            time.sleep(0.02)
        self._w(self.base + "/enable", 0)
        self._w(self.base + "/duty_cycle", 0)
        self._w(self.base + "/period", 20_000_000)
        self._w(self.base + "/enable", 1)

    @staticmethod
    def _w(path, v):
        with open(path, "w") as f:
            f.write(str(int(v)))

    def set(self, deg):
        self._w(self.base + "/duty_cycle", (500 + (deg + 90) / 180 * 2000) * 1000)

    def detach(self):
        self._w(self.base + "/duty_cycle", 0)

    def close(self):
        try:
            self.detach()
            self._w(self.base + "/enable", 0)
        except OSError:
            pass


def test_servo():
    try:
        hw = _HwServo()
        print("\n하드웨어 PWM으로 서보를 움직입니다.")
        try:
            for label, angle in [("가운데", 0), ("왼쪽", -SERVO_LIMIT), ("오른쪽", SERVO_LIMIT), ("가운데", 0)]:
                print(f"  {label} ({angle}도)")
                hw.set(angle)
                time.sleep(1.5)
        finally:
            hw.close()
        return
    except Exception as e:
        print(f"\n(하드웨어 PWM 못 씀: {e} -> 소프트웨어 PWM으로 테스트)")
    from gpiozero import AngularServo
    servo = AngularServo(SERVO_PIN, min_angle=-90, max_angle=90,
                         min_pulse_width=0.0005, max_pulse_width=0.0025)
    steps = [("가운데", 0), ("왼쪽", -SERVO_LIMIT), ("오른쪽", SERVO_LIMIT), ("가운데", 0)]
    print(f"\n서보를 ±{SERVO_LIMIT}도 범위에서 움직입니다.")
    try:
        for label, angle in steps:
            print(f"  {label} ({angle}도)")
            servo.angle = angle
            time.sleep(1.5)
    finally:
        servo.detach()
        servo.close()
        print("종료: 파이가 재부팅되지 않았는지, 'vcgencmd get_throttled' 가 0x0 인지 확인하세요")


# ---------------- 5. 레이저 쐈을 때 값 차이 ----------------
def test_spike():
    """패널마다 레이저를 껐다 켜며 값 차이를 측정 (s 입력 시 건너뜀)"""
    from gpiozero import LED
    read, close = open_adc()
    laser = LED(LASER_PIN, initial_value=False)

    def avg(ch, sec):
        vals = []
        end = time.monotonic() + sec
        while time.monotonic() < end:
            vals.append(read(ch))
            time.sleep(0.01)
        return sum(vals) / len(vals), max(vals) - min(vals)

    results = {}
    try:
        for ch in PANELS:
            ans = input(f"\n로봇 레이저를 {name(ch)} 에 겨누고 Enter (건너뛰려면 s): ").strip().lower()
            if ans == "s":
                continue
            offs, ons, noise = [], [], 0
            for r in range(3):
                laser.off()
                a, n = avg(ch, 1.0)
                offs.append(a)
                noise = max(noise, n)
                laser.on()
                a, _ = avg(ch, 1.0)
                ons.append(a)
                print(f"  {r+1}회: 꺼짐 {offs[-1]:.0f} -> 켜짐 {ons[-1]:.0f}")
            laser.off()
            jump = sum(ons) / 3 - sum(offs) / 3
            results[ch] = jump
            print(f"  {name(ch)} 결과: 평균 +{jump:.0f} (평소 흔들림 {noise})")
    except KeyboardInterrupt:
        print("\n중단됨")
    finally:
        laser.off()
        laser.close()
        close()
    if results:
        print("\n요약: " + ", ".join(f"{name(c)} +{v:.0f}" for c, v in results.items()))


# ---------------- 6. 깜빡임 방식 피격 감지 (패널 8개 동시) ----------------
def _analyze(ts, vs):
    """(FREQ 성분 세기, 위상[도]). 그림자처럼 한 방향으로 변하는 흐름은 먼저 제거"""
    n = len(vs)
    if n < 4:
        return 0.0, 0.0
    mt, mv = sum(ts) / n, sum(vs) / n
    var = sum((t - mt) ** 2 for t in ts) or 1e-9
    slope = sum((t - mt) * (v - mv) for t, v in zip(ts, vs)) / var
    rs = [v - mv - slope * (t - mt) for t, v in zip(ts, vs)]
    c = sum(r * math.cos(2 * math.pi * FREQ * t) for t, r in zip(ts, rs))
    s = sum(r * math.sin(2 * math.pi * FREQ * t) for t, r in zip(ts, rs))
    return 2 * math.sqrt(c * c + s * s) / n, math.degrees(math.atan2(s, c))


def _phase_diff(a, b):
    d = abs(a - b) % 360
    return min(d, 360 - d)


def _is_hit(recent, thr):
    if len(recent) < CONFIRM or recent[0][0] < thr:
        return False
    if any(m < thr * KEEP for m, _ in recent[1:]):
        return False
    return all(_phase_diff(a[1], b[1]) <= PHASE_TOL for a, b in zip(recent, recent[1:]))


def test_blink():
    """
    1단계: 레이저 깜빡임(벽 쪽) + 로봇 흔들기 -> 패널마다 감지 기준 자동 설정
    2단계: 레이저 깜빡임 + 아무 패널에나 겨눴다 뗐다 -> 어느 패널이 맞았는지 표시
    """
    from gpiozero import PWMLED
    read, close = open_adc()
    laser = PWMLED(LASER_PIN, frequency=FREQ, initial_value=0)
    T0 = time.monotonic()

    def run(label, duration, thresholds=None):
        hist = {ch: [] for ch in PANELS}
        t_start = time.monotonic()
        while time.monotonic() - t_start < duration:
            ts = {ch: [] for ch in PANELS}
            vs = {ch: [] for ch in PANELS}
            w0 = time.monotonic()
            nxt = w0
            while time.monotonic() - w0 < WINDOW:
                for ch in PANELS:
                    ts[ch].append(time.monotonic() - T0)
                    vs[ch].append(read(ch))
                nxt += 1 / RATE
                time.sleep(max(0, nxt - time.monotonic()))
            parts, hits = [], []
            for ch in PANELS:
                cur = _analyze(ts[ch], vs[ch])
                hist[ch].append(cur)
                parts.append(f"{name(ch)}:{cur[0]:4.1f}")
                if thresholds and _is_hit(hist[ch][-CONFIRM:], thresholds[ch]):
                    hits.append(name(ch))
            mark = f"  <-- HIT {', '.join(hits)}" if hits else ""
            print(f"  [{label}] " + " ".join(parts) + mark)
        return hist

    def count_hits(h, thr):
        return sum(1 for i in range(CONFIRM, len(h) + 1) if _is_hit(h[i - CONFIRM:i], thr))

    try:
        print("\n[1단계] 레이저가 깜빡이지만 패널이 아닌 벽 쪽을 향하게 두세요.")
        print("        10초 동안 로봇을 움직이거나 패널 위로 손 그림자를 지나가게 해 보세요.")
        input("준비되면 Enter")
        laser.value = 0.5
        time.sleep(0.3)
        quiet = run("배경", 10.0)
        laser.value = 0

        thresholds = {}
        print("\n  패널별 감지 기준:")
        for ch in PANELS:
            mags = sorted(m for m, _ in quiet[ch])
            typical = mags[int(len(mags) * 0.8)]
            thresholds[ch] = max(typical * 3, 1.0)
            print(f"    {name(ch)}: 평소 {typical:.2f}, 최대 {mags[-1]:.2f} -> 기준 {thresholds[ch]:.2f}")

        print("\n[2단계] 레이저는 계속 깜빡입니다. 20초 동안 여러 패널에")
        print("        번갈아 겨눴다 뗐다 해 보세요 (한 번에 1초 정도씩 머물게).")
        input("준비되면 Enter")
        laser.value = 0.5
        time.sleep(0.3)
        hit = run("레이저", 20.0, thresholds)
        laser.value = 0

        print("\n결과")
        total_false = 0
        for ch in PANELS:
            f = count_hits(quiet[ch], thresholds[ch])
            h = count_hits(hit[ch], thresholds[ch])
            total_false += f
            best = max(m for m, _ in hit[ch])
            print(f"  {name(ch)}: 가짜 HIT {f}번 / 겨눴을 때 HIT {h}번 (최대 세기 {best:.2f})")
        if total_false == 0:
            print("1단계 가짜 HIT 없음. 겨눈 패널에서만 HIT가 떴다면 성공이에요.")
        else:
            print("1단계에서 가짜 HIT가 있었어요. 해당 패널 선의 접촉 상태를 확인해 보세요.")
    except KeyboardInterrupt:
        print("\n중단됨")
    finally:
        laser.value = 0
        laser.close()
        close()


# ---------------- 메뉴 ----------------
TESTS = {
    "check": ("납땜 후 패널 연결 점검 (먼저 해 보세요)", test_check),
    "adc":   ("패널 값 실시간 보기", test_adc),
    "laser": ("레이저 깜빡이기", test_laser),
    "servo": ("서보 좌우로 움직이기", test_servo),
    "spike": ("레이저 쐈을 때 값 차이 측정", test_spike),
    "blink": ("깜빡임 방식 피격 감지 (패널 8개 동시)", test_blink),
}

if __name__ == "__main__":
    if len(sys.argv) == 2 and sys.argv[1] in TESTS:
        TESTS[sys.argv[1]][1]()
        sys.exit(0)

    keys = list(TESTS)
    while True:
        print("\n=== 거미로봇 테스트 ===")
        for i, k in enumerate(keys, 1):
            print(f"  {i}. {TESTS[k][0]}")
        print("  0. 종료")
        choice = input("번호 입력 후 Enter: ").strip()
        if choice == "0":
            break
        if choice.isdigit() and 1 <= int(choice) <= len(keys):
            try:
                TESTS[keys[int(choice) - 1]][1]()
            except Exception as e:
                print(f"\n오류: {e}")
                print("SPI 설정(dtoverlay=spi0-0cs), 배선, 라이브러리 설치를 확인해 주세요.")
        else:
            print("잘못된 번호예요. 다시 입력해 주세요.")
