#!/usr/bin/env python3
"""
robot_server.py - 거미로봇 전투 서버 (라즈베리파이 5에서 실행)

실행
    python3 robot_server.py A      <- 1번 로봇
    python3 robot_server.py B      <- 2번 로봇

하는 일
    - PC(hud_battle.py)에서 조종 신호를 받아 다리 모터 / 서보 / 레이저를 움직임
    - 카메라 영상을 PC로 실시간 전송
    - 태양광 패널 8개로 '상대 레이저'만 골라서 감지 -> PC에 명중 알림
        A 로봇: 25Hz로 깜빡이며 쏘고, 35Hz만 감지
        B 로봇: 35Hz로 깜빡이며 쏘고, 25Hz만 감지
      내 레이저가 반사돼서 내 패널에 들어오거나 배선으로 새어 들어와도 주파수가 달라서 무시됨 (본인 피격 없음)
    - 미로 모드 / 체력 0 / 시간 종료 때는 레이저 꺼짐, 서보 가운데 고정, 명중 감지 꺼짐

준비 (한 번만)
    sudo apt install python3-spidev python3-gpiozero python3-lgpio python3-picamera2
    /boot/firmware/config.txt 에 아래 두 줄이 있어야 함 (모터가 GPIO7, 8을 쓰므로 SPI CS 핀을 끔)
        dtparam=spi=on
        dtoverlay=spi0-0cs
    파이 IP 확인: hostname -I   -> PC의 hud_battle.py 에 이 IP를 넣으면 됨

핀 (괄호 = 물리 핀 번호)
    다리 모터  오른쪽 앞 GPIO7(26)  뒤 GPIO8(24)  속도 GPIO25(22)
               왼쪽   앞 GPIO20(38) 뒤 GPIO21(40) 속도 GPIO16(36)
    레이저 신호 GPIO23(16)     서보 신호 GPIO18(12)
    MCP3008    SCLK(23) MISO(21) MOSI(19) CS=GPIO24(18), 패널 P1~P8 = CH0~CH7
"""
import json
import math
import os
import multiprocessing as mp
import queue
import socket
import struct
import sys
import threading
import time

# ==================== 설정 ====================
ROBOT_ID = "A"            # 이 로봇의 이름 (기본값). 같은 폴더에 robot_id.txt 가 있으면 그 안의 글자(A 또는 B)가 우선
                          #   -> 2번 로봇은 robot_id.txt 에 B 라고 한 번만 적어 두면, 이 파일을 새로 받아도 계속 B

FREQS = {"A": (25.0, 35.0), "B": (35.0, 25.0)}   # 로봇 이름: (내가 쏘는 주파수, 감지할 상대 주파수)

CTRL_PORT = 5000          # 조종 신호
VIDEO_PORT = 5001         # 카메라 영상

# ---- 다리 모터 (기존 핀맵) ----
R_FWD, R_BACK, R_PWM = 7, 8, 25
L_FWD, L_BACK, L_PWM = 20, 21, 16
MOTOR_PWM_HZ = 100
DRIVE_SPEED = 0.6         # 전진/후진 속도 (0~1). 전원이 약하면 파이가 꺼져서 낮춤 (원래 0.8)
TURN_SPEED = 0.55         # 제자리 회전 속도 (0~1). 원래 0.7
MOTOR_RAMP = 1.5          # 모터 속도를 1초에 이만큼씩 천천히 올림 (0.4초 정도에 최고속도). 출발할 때 전류가 확 몰려 파이가 꺼지는 것 방지
RIGHT_INVERT = False      # 오른쪽 다리가 반대로 움직이면 True
LEFT_INVERT = False       # 왼쪽 다리가 반대로 움직이면 True
TURN_SWAP = True          # 좌우 회전이 반대로 되면 True (PC에서 온 왼쪽/오른쪽을 서로 바꿈). 앞뒤는 그대로

# ---- 레이저 / 서보 ----
LASER_PIN = 23
FIRE_TIME = 1.0           # 한 번 발사할 때 레이저가 켜져 있는 시간(초). 명중 판정에 약 0.6초 필요
SERVO_PIN = 18
SERVO_HW = True           # 하드웨어 PWM으로 서보 제어 (떨림 적음). config.txt 에 dtoverlay=pwm-2chan 필요.
                          #   준비 안 돼 있으면 자동으로 소프트웨어 PWM(예전 방식)으로 동작
PAN_MAX = 45.0
PAN_INVERT = False        # 서보가 반대로 돌면 True
SERVO_TRIM = 0.0          # 가운데 미세 조정(도). 레이저가 가운데에서 오른쪽을 보면 -2 처럼 반대로 조금씩 (servo_center.py로 찾은 값)
SERVO_IDLE = 1.0          # 이 시간 동안 각도가 그대로면 서보 신호를 끊어 떨림 방지
SERVO_SPEED = 70.0        # 서보가 목표 각도로 따라가는 최대 속도(도/초). PC 조준 속도(60)보다 살짝 크게
SERVO_STEP = 0.3          # 이 각도 이상 바뀔 때마다 서보에 새 각도를 보냄 (작을수록 부드러움)
SERVO_UPDATE = 0.02       # 서보에 새 각도를 보내는 최소 간격(초). 서보 신호 한 주기(20ms)보다 자주 보내면 튐
SERVO_BLANK = 0.4         # 내 서보가 움직이는 동안 + 멈춘 뒤 이 시간은 명중 판정 쉼 (서보 전기 잡음으로 가짜 명중 방지)

# ---- 명중 감지 (테스트로 맞춘 값) ----
CS_PIN = 24
PANELS = [0, 1, 2, 3, 4, 6, 7]   # 사용하는 MCP3008 채널 (P1~P8 = CH0~CH7). P6(CH5)은 떼어내서 뺌
RATE = 250.0              # 패널 측정 횟수(초당)
WINDOW = 0.2              # 한 번 판정에 쓰는 시간(초)
CONFIRM = 3               # 몇 구간 연속이어야 명중
KEEP = 0.5                # 첫 구간 이후엔 기준의 이 비율만 넘어도 인정
PHASE_TOL = 60.0          # 연속 구간 타이밍 허용 차이(도)
THR_MULT = 3.0            # 감지 기준 = 평소 잡음 x 이 값
MIN_THR = 1.0
CAL_TIME = 3.0            # 보정 시간(초). 시작할 때와 PC에서 F5 누를 때
HIT_GUARD = 1.5           # 한 번 맞으면 이 시간 동안은 다시 안 맞음 (한 발이 여러 번 세어지는 것 방지)
ARM_GRACE = 1.0           # 리셋 / 모드 전환 / 잠금 해제 직후 이 시간 동안은 명중 무시

# ---- 카메라 ----
CAM_SIZE = (640, 320)
CAM_FPS = 20
JPEG_QUALITY = 70
CAM_ROTATE_180 = False    # 카메라가 거꾸로 달려 있으면 True

WATCHDOG = 0.5            # PC 신호가 이 시간 동안 안 오면 정지


def motor_outputs(go=False, back=False, left=False, right=False):
    """HUD 와 같은 규칙. (오른쪽, 왼쪽)  +1 앞 / -1 뒤 / 0 정지.  우선순위: 전진 > 후진 > 우 > 좌"""
    if go:
        if bool(left) == bool(right):
            return 1, 1
        return (1, -1) if left else (-1, 1)
    if back:
        if bool(left) == bool(right):
            return -1, -1
        return (-1, 1) if left else (1, -1)
    if right:
        return -1, 1
    if left:
        return 1, -1
    return 0, 0


# ==================== 명중 감지 (별도 프로세스: 측정 타이밍이 흔들리지 않게) ====================
def _analyze(ts, vs, freq):
    """(freq 성분 세기, 위상[도]).
    그림자처럼 한 방향으로 변하는 흐름은 먼저 빼고, 구간 양 끝을 부드럽게 줄여서(Hann 창)
    갑자기 밝아지거나 어두워지는 순간이 레이저로 오인되는 것을 줄임"""
    n = len(vs)
    if n < 4:
        return 0.0, 0.0
    mt, mv = sum(ts) / n, sum(vs) / n
    var = sum((t - mt) ** 2 for t in ts) or 1e-9
    slope = sum((t - mt) * (v - mv) for t, v in zip(ts, vs)) / var
    t_a, span = ts[0], (ts[-1] - ts[0]) or 1e-9
    ws = [0.5 - 0.5 * math.cos(2 * math.pi * (t - t_a) / span) for t in ts]
    w = 2 * math.pi * freq
    c = s = 0.0
    for t, v, h in zip(ts, vs, ws):
        r = (v - mv - slope * (t - mt)) * h
        c += r * math.cos(w * t)
        s += r * math.sin(w * t)
    return 2 * math.sqrt(c * c + s * s) / (sum(ws) or 1e-9), math.degrees(math.atan2(s, c))


def _phase_diff(a, b):
    d = abs(a - b) % 360
    return min(d, 360 - d)


def _is_hit(recent, thr):
    if len(recent) < CONFIRM or recent[0][0] < thr:
        return False
    if any(m < thr * KEEP for m, _ in recent[1:]):
        return False
    return all(_phase_diff(a[1], b[1]) <= PHASE_TOL for a, b in zip(recent, recent[1:]))


def detector_main(enemy_freq, hit_q, info_q, armed, cal_evt, stop_evt, busy):
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
    cs = DigitalOutputDevice(CS_PIN, initial_value=True)

    def read(ch):
        cs.off()
        r = spi.xfer2([1, (8 + ch) << 4, 0])
        cs.on()
        return ((r[1] & 3) << 8) | r[2]

    t0 = time.monotonic()
    n_ch = len(PANELS)

    def window():
        ts = [[] for _ in PANELS]
        vs = [[] for _ in PANELS]
        w0 = time.monotonic()
        nxt = w0
        while time.monotonic() - w0 < WINDOW:
            for i, ch in enumerate(PANELS):
                ts[i].append(time.monotonic() - t0)
                vs[i].append(read(ch))
            nxt += 1.0 / RATE
            d = nxt - time.monotonic()
            if d > 0:
                time.sleep(d)
            else:
                nxt = time.monotonic()
        res = [_analyze(ts[i], vs[i], enemy_freq) for i in range(n_ch)]
        levels = [sum(v) / len(v) for v in vs]
        return res, len(ts[0]) / WINDOW, levels

    def calibrate():
        mags = [[] for _ in PANELS]
        rate, levels = 0.0, [0] * n_ch
        end = time.monotonic() + CAL_TIME
        while time.monotonic() < end:
            res, rate, levels = window()
            for i, (m, _) in enumerate(res):
                mags[i].append(m)
        thr = []
        for m in mags:
            s = sorted(m)
            thr.append(max(s[int(len(s) * 0.8)] * THR_MULT, MIN_THR))
        info_q.put({"t": "cal", "thr": [round(x, 2) for x in thr],
                    "level": [round(x) for x in levels], "rate": round(rate)})
        return thr

    try:
        thr = calibrate()
        hist = [[] for _ in PANELS]
        guard_until = 0.0
        was_armed = False
        while not stop_evt.is_set():
            if cal_evt.is_set():
                cal_evt.clear()
                thr = calibrate()
                hist = [[] for _ in PANELS]
                continue
            res, _, _ = window()
            now = time.monotonic()
            armed_now = bool(armed.value)
            if armed_now and not was_armed:          # 막 켜짐(리셋 등): 꺼져 있던 동안 쌓인 기록 버리고 잠깐 무시
                hist = [[] for _ in PANELS]
                guard_until = max(guard_until, now + ARM_GRACE)
            was_armed = armed_now
            if now < busy.value:                     # 내 서보가 움직이는 중: 이 구간은 버림
                hist = [[] for _ in PANELS]
                continue
            for i in range(n_ch):
                hist[i].append(res[i])
                del hist[i][:-CONFIRM]
            if not armed.value or now < guard_until:
                continue
            for i in range(n_ch):
                if _is_hit(hist[i], thr[i]):
                    hit_q.put(PANELS[i])
                    guard_until = now + HIT_GUARD
                    hist = [[] for _ in PANELS]
                    break
    finally:
        spi.close()
        cs.close()


# ==================== 하드웨어 ====================
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


class HardwareServo:
    """라즈베리파이 5 하드웨어 PWM으로 서보 제어 (GPIO18 = PWM0 2번 채널).
    소프트웨어 PWM보다 신호가 정확해서 서보가 덜 떨림. gpiozero AngularServo 와 같은 방식으로 사용"""
    CHANNEL = 2
    PERIOD_NS = 20_000_000            # 50Hz
    MIN_US, MAX_US = 500, 2500        # -90도 ~ +90도

    def __init__(self):
        chip = _pick_pwm_chip()
        chips = [chip]
        self.base = f"{chips[0]}/pwm{self.CHANNEL}"
        if not os.path.exists(self.base):
            self._write(chips[0] + "/export", self.CHANNEL)
        for _ in range(50):                               # 권한이 잡힐 때까지 잠깐 기다림
            if os.access(self.base + "/period", os.W_OK):
                break
            time.sleep(0.02)
        self._write(self.base + "/enable", 0)
        self._write(self.base + "/duty_cycle", 0)
        self._write(self.base + "/period", self.PERIOD_NS)
        self._write(self.base + "/enable", 1)
        try:                                              # GPIO18 을 PWM 기능(a3)으로 확실히 전환
            import subprocess
            subprocess.run(["pinctrl", "set", "18", "a3"], timeout=3, capture_output=True)
        except Exception:
            pass
        self.angle = 0

    @staticmethod
    def _write(path, value):
        with open(path, "w") as f:
            f.write(str(int(value)))

    @property
    def angle(self):
        return None

    @angle.setter
    def angle(self, deg):
        deg = max(-90.0, min(90.0, float(deg)))
        us = self.MIN_US + (deg + 90.0) / 180.0 * (self.MAX_US - self.MIN_US)
        self._write(self.base + "/duty_cycle", us * 1000)

    def detach(self):
        self._write(self.base + "/duty_cycle", 0)         # 신호 끊기 (서보 힘 풀림)

    def close(self):
        try:
            self.detach()
            self._write(self.base + "/enable", 0)
        except OSError:
            pass


class Robot:
    def __init__(self, own_freq, busy):
        self.busy = busy                         # 서보 움직임을 감지 프로세스에 알림
        from gpiozero import OutputDevice, PWMOutputDevice, PWMLED, AngularServo

        def make(name, pin, cls, **kw):
            try:
                return cls(pin, **kw)
            except Exception as e:
                hint = ""
                if pin in (7, 8):
                    hint = ("\n   -> SPI가 GPIO7/8을 잡고 있을 가능성이 큽니다. /boot/firmware/config.txt 에 "
                            "dtoverlay=spi0-0cs 를 넣고 재부팅하세요.")
                raise SystemExit(f"\n[오류] {name} (GPIO{pin}) 을 쓸 수 없음: {e}"
                                 f"\n   -> 다른 프로그램(이전에 켠 robot_server, 기존 로봇 코드, test_robot)이 "
                                 f"이 핀을 쓰고 있는지 확인하세요.{hint}")

        self.rf = make("오른쪽 모터 앞", R_FWD, OutputDevice)
        self.rb = make("오른쪽 모터 뒤", R_BACK, OutputDevice)
        self.rp = make("오른쪽 모터 속도", R_PWM, PWMOutputDevice, frequency=MOTOR_PWM_HZ)
        self.lf = make("왼쪽 모터 앞", L_FWD, OutputDevice)
        self.lb = make("왼쪽 모터 뒤", L_BACK, OutputDevice)
        self.lp = make("왼쪽 모터 속도", L_PWM, PWMOutputDevice, frequency=MOTOR_PWM_HZ)
        self.laser = make("레이저", LASER_PIN, PWMLED, frequency=own_freq, initial_value=0)   # 반은 켜짐으로 깜빡이며 발사
        self.servo = None
        if SERVO_HW:
            try:
                self.servo = HardwareServo()
                print("[servo] 하드웨어 PWM 사용 (GPIO18)")
            except Exception as e:
                print(f"[servo] 하드웨어 PWM 못 씀 ({e}) -> 소프트웨어 PWM으로 동작")
        if self.servo is None:
            self.servo = make("서보", SERVO_PIN, AngularServo, min_angle=-90, max_angle=90,
                              min_pulse_width=0.0005, max_pulse_width=0.0025)
        try:
            self.servo.angle = SERVO_TRIM            # 시작할 때 가운데(+미세 조정)로
        except Exception:
            pass
        self.pan = 0.0                           # 지금 서보 각도 (부드럽게 따라가는 중인 값)
        self.target = 0.0                        # PC가 원하는 각도
        self.sent = 0.0                          # 마지막으로 서보에 보낸 각도
        self.sent_t = 0.0
        self.last_tick = time.monotonic()
        self.pan_t = time.monotonic()
        self.servo_on = True
        self.fire_until = 0.0
        self.motion = (0, 0)
        self.m_cur = {"r": 0.0, "l": 0.0}       # 지금 모터 속도 (천천히 올라감)
        self.m_dir = {"r": 0, "l": 0}           # 지금 방향
        self.m_tgt = {"r": (0, 0.0), "l": (0, 0.0)}   # 목표 (방향, 속도)

    @staticmethod
    def _motor(fwd, back, pwm, d, speed, invert):
        if invert:
            d = -d
        if d > 0:
            back.off()
            fwd.on()
        elif d < 0:
            fwd.off()
            back.on()
        else:
            fwd.off()
            back.off()
        pwm.value = speed if d else 0.0

    def drive(self, f=False, b=False, l=False, r=False, force=False):
        rr, ll = motor_outputs(f, b, l, r)
        if (rr, ll) == self.motion and not force:
            return
        self.motion = (rr, ll)
        speed = DRIVE_SPEED if rr == ll else TURN_SPEED
        self.m_tgt = {"r": (rr, speed), "l": (ll, speed)}
        for side, d in (("r", rr), ("l", ll)):
            if d == 0 or d != self.m_dir[side]:          # 멈춤 / 방향 바뀜은 바로 0으로 (그다음 천천히 출발)
                self.m_cur[side] = 0.0
                self.m_dir[side] = d
                self._apply(side)

    def _apply(self, side):
        if side == "r":
            self._motor(self.rf, self.rb, self.rp, self.m_dir["r"], self.m_cur["r"], RIGHT_INVERT)
        else:
            self._motor(self.lf, self.lb, self.lp, self.m_dir["l"], self.m_cur["l"], LEFT_INVERT)

    def _ramp(self, dt):
        for side in ("r", "l"):
            d, v = self.m_tgt[side]
            if d == 0 or d != self.m_dir[side]:
                continue
            cur = self.m_cur[side]
            new = min(v, cur + MOTOR_RAMP * dt) if cur < v else v
            if abs(new - cur) > 1e-4:
                self.m_cur[side] = new
                self._apply(side)

    def set_pan(self, angle):
        """목표 각도만 정함. 실제 서보는 tick()에서 조금씩 부드럽게 따라감"""
        self.target = max(-PAN_MAX, min(PAN_MAX, float(angle)))

    def fire(self):
        self.laser.value = 0.5
        self.fire_until = time.monotonic() + FIRE_TIME

    def laser_off(self):
        if self.fire_until:
            self.laser.value = 0
            self.fire_until = 0.0

    def tick(self):
        now = time.monotonic()
        dt = min(0.05, now - self.last_tick)
        self.last_tick = now
        self._ramp(dt)
        diff = self.target - self.pan
        if abs(diff) > 0.01:                     # 목표까지 최대 SERVO_SPEED 속도로 조금씩 이동
            step = SERVO_SPEED * dt
            self.pan += max(-step, min(step, diff))
            arrived = abs(self.target - self.pan) <= 0.01
            if (now - self.sent_t >= SERVO_UPDATE or arrived) and \
                    (abs(self.pan - self.sent) >= SERVO_STEP or arrived):
                self.sent, self.sent_t = self.pan, now
                self.servo.angle = (-self.pan if PAN_INVERT else self.pan) + SERVO_TRIM
                self.servo_on = True
                self.pan_t = now
                self.busy.value = now + SERVO_BLANK
        if self.fire_until and now >= self.fire_until:
            self.laser_off()
        if self.servo_on and now - self.pan_t > SERVO_IDLE:
            self.servo.detach()
            self.servo_on = False
            self.busy.value = now + SERVO_BLANK

    def stop_all(self):
        self.drive(force=True)
        self.laser.value = 0
        self.fire_until = 0.0

    def close(self):
        for d in (self.rf, self.rb, self.rp, self.lf, self.lb, self.lp, self.laser, self.servo):
            try:
                d.close()
            except Exception:
                pass


# ==================== 카메라 ====================
class Camera:
    def __init__(self):
        self.frame = None
        self.seq = 0
        self.cond = threading.Condition()
        try:
            from picamera2 import Picamera2
            import simplejpeg
        except ImportError as e:
            print(f"[cam] 카메라 라이브러리 없음 ({e}) -> 영상 없이 실행")
            return
        try:
            self.cam = Picamera2()
            kw = {}
            if CAM_ROTATE_180:
                from libcamera import Transform
                kw["transform"] = Transform(hflip=1, vflip=1)
            cfg = self.cam.create_video_configuration(main={"size": CAM_SIZE, "format": "RGB888"},
                                                      controls={"FrameRate": CAM_FPS}, **kw)
            self.cam.configure(cfg)
            self.cam.start()
        except Exception as e:
            print(f"[cam] 카메라 시작 실패 ({e}) -> 영상 없이 실행")
            return
        self.encode = simplejpeg.encode_jpeg
        threading.Thread(target=self._loop, daemon=True).start()
        print(f"[cam] 카메라 시작 {CAM_SIZE[0]}x{CAM_SIZE[1]} {CAM_FPS}fps")

    def _loop(self):
        while True:
            try:
                arr = self.cam.capture_array("main")
                jpg = self.encode(arr, quality=JPEG_QUALITY, colorspace="BGR")
            except Exception as e:
                print(f"[cam] 오류: {e}")
                time.sleep(1)
                continue
            with self.cond:
                self.frame = jpg
                self.seq += 1
                self.cond.notify_all()

    def wait_frame(self, last_seq, timeout=2.0):
        with self.cond:
            self.cond.wait_for(lambda: self.seq != last_seq, timeout)
            return self.seq, self.frame


def video_server(cam):
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("", VIDEO_PORT))
    srv.listen(4)
    while True:
        conn, addr = srv.accept()
        threading.Thread(target=_video_client, args=(cam, conn, addr), daemon=True).start()


def _video_client(cam, conn, addr):
    print(f"[cam] 영상 연결: {addr[0]}")
    conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    seq = -1
    try:
        while True:
            seq, jpg = cam.wait_frame(seq)
            if jpg is not None:
                conn.sendall(struct.pack(">I", len(jpg)) + jpg)
    except OSError:
        pass
    finally:
        conn.close()
        print(f"[cam] 영상 연결 끊김: {addr[0]}")


# ==================== 조종 신호 ====================
class Server:
    def __init__(self, robot_id, own, enemy, robot, armed, cal_evt):
        self.hello = {"t": "hello", "id": robot_id, "own": own, "enemy": enemy}
        self.robot, self.armed, self.cal_evt = robot, armed, cal_evt
        self.srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.srv.bind(("", CTRL_PORT))
        self.srv.listen(2)
        self.srv.setblocking(False)
        self.client = None
        self.buf = b""
        self.last_ctl = 0.0
        self.last_ping = 0.0
        self.last_cal = None
        self.lock = True
        self.maze = False

    def accept(self):
        try:
            conn, addr = self.srv.accept()
        except (BlockingIOError, OSError):
            return
        if self.client:
            self.drop("새 연결로 교체")
        conn.setblocking(False)
        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.client, self.buf, self.last_ctl = conn, b"", 0.0
        print(f"[ctl] PC 연결: {addr[0]}")
        self.send(self.hello)
        if self.last_cal:
            self.send(self.last_cal)

    def drop(self, why):
        if self.client:
            try:
                self.client.close()
            except OSError:
                pass
            self.client = None
            print(f"[ctl] 연결 끊김 ({why}) -> 정지")
        self.robot.stop_all()
        self.armed.value = 0

    def send(self, obj):
        if self.client is None:
            return
        try:
            self.client.sendall((json.dumps(obj) + "\n").encode())
        except OSError:
            self.drop("보내기 실패")

    def poll(self):
        if self.client is None:
            return
        try:
            data = self.client.recv(4096)
        except BlockingIOError:
            return
        except OSError:
            self.drop("연결 오류")
            return
        if not data:
            self.drop("PC가 연결을 끊음")
            return
        self.buf += data
        while b"\n" in self.buf:
            line, self.buf = self.buf.split(b"\n", 1)
            try:
                self.handle(json.loads(line))
            except ValueError:
                pass

    def handle(self, m):
        t = m.get("t")
        if t == "ctl":
            self.last_ctl = time.monotonic()
            self.lock, self.maze = bool(m.get("lock")), bool(m.get("maze"))
            if self.lock:
                self.robot.drive()
            else:
                l, r = m.get("l"), m.get("r")
                if TURN_SWAP:
                    l, r = r, l
                self.robot.drive(m.get("f"), m.get("b"), l, r)
            if self.lock or self.maze:
                self.robot.laser_off()
            self.robot.set_pan(0.0 if self.maze else m.get("pan", 0.0))
            self.armed.value = int(not self.lock and not self.maze)
        elif t == "fire":
            if not self.lock and not self.maze:
                self.robot.fire()
                print("[laser] 발사")
        elif t == "cal":
            self.cal_evt.set()
            print(f"[cal] 패널 다시 보정 ({CAL_TIME:.0f}초)")

    def tick(self):
        now = time.monotonic()
        if self.client and now - self.last_ping > 1.0:
            self.last_ping = now
            self.send({"t": "ping"})
        if now - self.last_ctl > WATCHDOG:              # PC 신호 끊김 -> 정지, 감지 끔
            self.robot.drive()
            self.robot.laser_off()
            self.armed.value = 0


def main():
    robot_id = ROBOT_ID
    id_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), "robot_id.txt")
    if os.path.exists(id_file):                     # 같은 폴더의 robot_id.txt 가 있으면 그 글자 사용 (파일 업데이트해도 유지)
        robot_id = open(id_file, encoding="utf-8").read().strip() or robot_id
    if len(sys.argv) > 1:
        robot_id = sys.argv[1]
    robot_id = robot_id.upper()
    if robot_id not in FREQS:
        print(f"로봇 이름이 '{robot_id}' 입니다. 파일 위쪽 ROBOT_ID 를 \"A\" 또는 \"B\" 로 바꿔 주세요.")
        return
    own, enemy = FREQS[robot_id]
    print(f"=== 로봇 {robot_id}: {own:.0f}Hz로 쏘고, {enemy:.0f}Hz만 감지 ===")

    ctx = mp.get_context("spawn")
    hit_q, info_q = ctx.Queue(), ctx.Queue()
    armed = ctx.Value("b", 0)
    cal_evt, stop_evt = ctx.Event(), ctx.Event()
    busy = ctx.Value("d", 0.0)                       # 이 시각(monotonic)까지 서보가 움직이는 중
    det = ctx.Process(target=detector_main, args=(enemy, hit_q, info_q, armed, cal_evt, stop_evt, busy),
                      daemon=True)
    det.start()
    print(f"[cal] 패널 보정 중 ({CAL_TIME:.0f}초) - 레이저를 비추지 마세요")

    robot = Robot(own, busy)
    cam = Camera()
    threading.Thread(target=video_server, args=(cam,), daemon=True).start()
    server = Server(robot_id, own, enemy, robot, armed, cal_evt)
    print(f"[ctl] PC 연결 기다리는 중 (포트 {CTRL_PORT}, 영상 {VIDEO_PORT})")

    try:
        while True:
            server.accept()
            server.poll()
            server.tick()
            robot.tick()
            while True:
                try:
                    ch = hit_q.get_nowait()
                except queue.Empty:
                    break
                if armed.value:
                    print(f"[hit] P{ch + 1} 명중!")
                    server.send({"t": "hit", "panel": ch})
            while True:
                try:
                    info = info_q.get_nowait()
                except queue.Empty:
                    break
                server.last_cal = info
                print(f"[cal] 완료  측정 {info['rate']}회/초  평소값 {info['level']}  기준 {info['thr']}")
                if info["rate"] < RATE * 0.8:
                    print("[cal] 경고: 측정 속도가 느립니다. 다른 무거운 프로그램을 꺼 주세요.")
                server.send(info)
            if not det.is_alive():
                print("[hit] 감지 프로세스가 멈췄습니다. SPI 설정과 배선을 확인하세요.")
                det.join()
                break
            time.sleep(0.005)
    except KeyboardInterrupt:
        pass
    finally:
        stop_evt.set()
        robot.stop_all()
        robot.close()
        print("종료")


if __name__ == "__main__":
    main()
