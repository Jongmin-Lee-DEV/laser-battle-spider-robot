#!/usr/bin/env python3
"""
hit_probe.py - 가만히 있어도 뜨는 '가짜 명중'이 언제, 어느 패널에서, 왜 뜨는지 찾는 테스트
(라즈베리파이에서 실행. robot_server.py 는 꺼 둔 상태로. 이 파일 하나만 있으면 됨)

    python3 hit_probe.py          -> 메뉴
    python3 hit_probe.py 1 60     -> 1번 모드를 60초 동안

모드
    1  가만히 (아무것도 안 움직임)       -> 여기서 뜨면: 조명 / 접촉 불량 / 전원 잡음
    2  다리 모터 켰다 껐다 (로봇 들어서!) -> 모터 잡음
    3  서보 좌우로 계속 움직임           -> 서보 잡음 (실제 서버는 이때 판정을 쉬지만, 여기선 그대로 보여줌)
    4  내 레이저 켰다 껐다               -> 내 레이저가 새어 들어오는지
    5  2~4 전부 같이

화면 한 줄 = 0.2초. 숫자는 '감지 기준 대비 세기' (1.0 이상이면 기준 넘음, * 표시)
명중으로 판정되면 그 패널의 자세한 정보가 나오고, 끝나면 요약과 원인 추정이 나옴.
모든 기록은 hit_probe_log.csv 에 저장 (PC로 가져와서 보내 주면 같이 볼 수 있음)
"""
import csv
import math
import multiprocessing as mp
import os
import sys
import threading
import time

# ==================== robot_server.py 없이도 돌아가게 필요한 부분을 여기에 넣음 ====================
import types
import json
rs = types.SimpleNamespace()
ROBOT_ID = "A"
FREQS = {"A": (25.0, 35.0), "B": (35.0, 25.0)}
R_FWD, R_BACK, R_PWM = 7, 8, 25
L_FWD, L_BACK, L_PWM = 20, 21, 16
MOTOR_PWM_HZ = 100
DRIVE_SPEED = 0.5         # 테스트용 모터 속도 (전원이 약하면 파이가 꺼지므로 실제보다 낮게)
RAMP_STEP = 0.01          # 0.01초마다 이만큼씩 천천히 속도를 올림 (갑자기 큰 전류가 흐르지 않게)
LASER_PIN = 23
SERVO_PIN = 18
PAN_MAX = 45.0
SERVO_SPEED = 70.0
CS_PIN = 24
PANELS = [0, 1, 2, 3, 4, 6, 7]   # P6(CH5) 뺌
RATE = 250.0
WINDOW = 0.2
CONFIRM = 3
KEEP = 0.5
PHASE_TOL = 60.0
THR_MULT = 3.0
MIN_THR = 1.0
CAL_TIME = 3.0
HIT_GUARD = 1.5


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
    """테스트용 간단 버전: 다리 모터, 레이저, 서보"""
    def __init__(self, own_freq, busy=None):
        from gpiozero import OutputDevice, PWMOutputDevice, PWMLED, AngularServo
        self.rf, self.rb = OutputDevice(R_FWD), OutputDevice(R_BACK)
        self.rp = PWMOutputDevice(R_PWM, frequency=MOTOR_PWM_HZ)
        self.lf, self.lb = OutputDevice(L_FWD), OutputDevice(L_BACK)
        self.lp = PWMOutputDevice(L_PWM, frequency=MOTOR_PWM_HZ)
        self.laser = PWMLED(LASER_PIN, frequency=own_freq, initial_value=0)
        try:
            self.servo = HardwareServo()
            print("[servo] 하드웨어 PWM")
        except Exception as e:
            print(f"[servo] 하드웨어 PWM 못 씀 ({e}) -> 소프트웨어 PWM")
            self.servo = AngularServo(SERVO_PIN, min_angle=-90, max_angle=90,
                                      min_pulse_width=0.0005, max_pulse_width=0.0025)
        self.pan = self.target = 0.0
        self.last = time.monotonic()

    def drive(self, f=False, force=False):
        """f=True 면 DRIVE_SPEED 까지 천천히 올리고, False 면 바로 멈춤"""
        want = DRIVE_SPEED if f and not force else 0.0
        sp = getattr(self, "speed", 0.0)
        sp = min(want, sp + RAMP_STEP) if want > sp else want
        self.speed = sp
        for fwd, back, pwm in ((self.rf, self.rb, self.rp), (self.lf, self.lb, self.lp)):
            back.off()
            fwd.on() if sp > 0 else fwd.off()
            pwm.value = sp

    def set_pan(self, a):
        self.target = max(-PAN_MAX, min(PAN_MAX, a))

    def tick(self):
        now = time.monotonic()
        dt = min(0.05, now - self.last)
        self.last = now
        d = self.target - self.pan
        if abs(d) > 0.01:
            st = SERVO_SPEED * dt
            self.pan += max(-st, min(st, d))
            self.servo.angle = self.pan

    def close(self):
        for d in (self.rf, self.rb, self.rp, self.lf, self.lb, self.lp, self.laser):
            try:
                d.close()
            except Exception:
                pass
        try:
            self.servo.close()
        except Exception:
            pass


for _k, _v in list(globals().items()):
    if not _k.startswith("__") and _k not in ("rs", "types"):
        setattr(rs, _k, _v)
# ==================== 여기까지 ====================

HERE = os.path.dirname(os.path.abspath(__file__))
LOG = os.path.join(HERE, "hit_probe_log.csv")
SPEC_F = [10, 15, 20, 25, 30, 35, 40, 45, 50, 60, 75, 100, 120]   # 명중 때 보여줄 주파수들
JUMP = 150                     # 0.2초 안에 값이 이만큼 이상 튀면 '값 튐'
MODES = {"1": "가만히", "2": "다리 모터", "3": "서보", "4": "내 레이저", "5": "전부 같이"}


def robot_id():
    p = os.path.join(HERE, "robot_id.txt")
    rid = rs.ROBOT_ID
    if os.path.exists(p):
        rid = open(p, encoding="utf-8").read().strip() or rid
    return rid.upper()


# ---------- 측정 (별도 프로세스: 실제 서버와 같은 방식) ----------
def sampler(out_q, stop_evt):
    import spidev
    from gpiozero import DigitalOutputDevice
    spi = spidev.SpiDev()
    spi.open(0, 0)
    spi.max_speed_hz = 250_000
    spi.mode = 0
    try:
        spi.no_cs = True
    except (OSError, AttributeError):
        pass
    cs = DigitalOutputDevice(rs.CS_PIN, initial_value=True)

    def read(ch):
        cs.off()
        r = spi.xfer2([1, (8 + ch) << 4, 0])
        cs.on()
        return ((r[1] & 3) << 8) | r[2]

    t0 = time.monotonic()
    try:
        while not stop_evt.is_set():
            ts = [[] for _ in rs.PANELS]
            vs = [[] for _ in rs.PANELS]
            w0 = nxt = time.monotonic()
            while time.monotonic() - w0 < rs.WINDOW:
                for i, ch in enumerate(rs.PANELS):
                    ts[i].append(time.monotonic() - t0)
                    vs[i].append(read(ch))
                nxt += 1.0 / rs.RATE
                d = nxt - time.monotonic()
                if d > 0:
                    time.sleep(d)
                else:
                    nxt = time.monotonic()
            out_q.put((time.monotonic(), ts, vs))
    finally:
        spi.close()
        cs.close()


# ---------- 잡음 일으키기 ----------
class Busy:
    value = 0.0


def disturb(robot, mode, own, stop):
    """mode 에 맞게 모터/서보/레이저를 움직임. 지금 뭘 하는지 state[0] 에 적어 둠"""
    motor = mode in ("2", "5")
    servo = mode in ("3", "5")
    laser = mode in ("4", "5")
    t_start = time.monotonic()
    side = 1
    while not stop.is_set():
        now = time.monotonic()
        phase_on = int((now - t_start) / 2.0) % 2 == 0          # 2초 켜고 2초 끔
        acts = []
        if motor:
            robot.drive(f=phase_on)
            acts.append("모터" if phase_on else "")
        if laser:
            if phase_on:
                robot.laser.value = 0.5
            else:
                robot.laser.value = 0
            acts.append("레이저" if phase_on else "")
        if servo:
            if abs(robot.pan - robot.target) < 0.5:
                side = -side
                robot.set_pan(side * rs.PAN_MAX)
            acts.append("서보")
        robot.tick()
        STATE[0] = "+".join(a for a in acts if a) or "-"
        time.sleep(0.01)
    robot.drive(force=True)
    robot.laser.value = 0


STATE = ["-"]


def PN(i):
    """목록 순서 i -> 패널 이름 (채널 번호 + 1)"""
    return f"P{rs.PANELS[i] + 1}"


def ratios_line(t, ratios, flags):
    cells = []
    for i, r in enumerate(ratios):
        mark = "*" if r >= 1.0 else " "
        cells.append(f"{PN(i)}{r:4.1f}{mark}")
    return f"{t:6.1f}s [{STATE[0]:^10}] " + " ".join(cells) + ("  " + flags if flags else "")


def run(mode, seconds):
    rid = robot_id()
    own, enemy = rs.FREQS.get(rid, rs.FREQS["A"])
    print(f"\n=== 로봇 {rid}: 감지 주파수 {enemy:.0f}Hz / 모드 {mode} ({MODES[mode]}) / {seconds}초 ===")
    if mode in ("2", "5"):
        print("!! 다리 모터가 돕니다. 로봇을 들거나 받침 위에 올려 두세요. 3초 뒤 시작")
        time.sleep(3)

    ctx = mp.get_context("spawn")
    q, stop_evt = ctx.Queue(), ctx.Event()
    proc = ctx.Process(target=sampler, args=(q, stop_evt), daemon=True)
    proc.start()
    robot = rs.Robot(own, Busy())

    # 1) 보정: 실제 서버와 똑같이 (아무것도 안 움직이는 상태)
    print(f"[보정] {rs.CAL_TIME:.0f}초 - 레이저 비추지 말고 그대로 두세요")
    mags = [[] for _ in rs.PANELS]
    end = time.monotonic() + rs.CAL_TIME
    while time.monotonic() < end:
        _, ts, vs = q.get()
        for i in range(len(rs.PANELS)):
            mags[i].append(rs._analyze(ts[i], vs[i], enemy)[0])
    thr = []
    for m in mags:
        s = sorted(m)
        thr.append(max(s[int(len(s) * 0.8)] * rs.THR_MULT, rs.MIN_THR))
    print("[보정] 기준값:", " ".join(f"{PN(i)}={x:.1f}" for i, x in enumerate(thr)))

    stop = threading.Event()
    th = threading.Thread(target=disturb, args=(robot, mode, own, stop), daemon=True)
    th.start()

    hist = [[] for _ in rs.PANELS]
    raw_hist = [[] for _ in rs.PANELS]          # 최근 1초 원본 (명중 때 주파수 분석용)
    stats = [{"hit": 0, "over": 0, "max": 0.0, "lvl": [], "spread": [], "jump": 0} for _ in rs.PANELS]
    multi = 0                                    # 여러 패널이 동시에 기준을 넘은 횟수
    events = []
    guard_until = 0.0
    t_begin = time.monotonic()
    new = not os.path.exists(LOG)
    f = open(LOG, "a", newline="", encoding="utf-8")
    w = csv.writer(f)
    if new:
        w.writerow(["시각", "모드", "동작"] + [f"P{i + 1}_비율" for i in rs.PANELS] +
                   [f"P{i + 1}_평균" for i in rs.PANELS] + [f"P{i + 1}_흔들림" for i in rs.PANELS] + ["명중"])
    try:
        while time.monotonic() - t_begin < seconds:
            _, ts, vs = q.get()
            now = time.monotonic()
            t = now - t_begin
            res = [rs._analyze(ts[i], vs[i], enemy) for i in range(len(rs.PANELS))]
            ratios = [m / thr[i] for i, (m, _) in enumerate(res)]
            lvls = [sum(v) / len(v) for v in vs]
            spreads = [max(v) - min(v) for v in vs]
            over = [i for i, r in enumerate(ratios) if r >= 1.0]
            flags = []
            if len(over) >= 3:
                multi += 1
                flags.append(f"동시{len(over)}개")
            for i in range(len(rs.PANELS)):
                st = stats[i]
                st["max"] = max(st["max"], ratios[i])
                st["lvl"].append(lvls[i])
                st["spread"].append(spreads[i])
                if ratios[i] >= 1.0:
                    st["over"] += 1
                if spreads[i] >= JUMP:
                    st["jump"] += 1
                    flags.append(f"{PN(i)}값튐{spreads[i]}")
                hist[i].append(res[i])
                del hist[i][:-rs.CONFIRM]
                raw_hist[i].append((ts[i], vs[i]))
                del raw_hist[i][:-5]
            hit = None
            if now >= guard_until:
                for i in range(len(rs.PANELS)):
                    if rs._is_hit(hist[i], thr[i]):
                        hit = i
                        break
            if hit is not None:
                guard_until = now + rs.HIT_GUARD
                hist = [[] for _ in rs.PANELS]
                stats[hit]["hit"] += 1
                flags.append(f"<<< 명중 {PN(hit)}")
            print(ratios_line(t, ratios, " ".join(flags)))
            w.writerow([f"{t:.2f}", mode, STATE[0]] + [f"{r:.2f}" for r in ratios] +
                       [f"{x:.0f}" for x in lvls] + spreads + [f"{PN(hit)}" if hit is not None else ""])
            if hit is not None:
                tt = [x for a, _ in raw_hist[hit] for x in a]
                vv = [x for _, b in raw_hist[hit] for x in b]
                spec = [rs._analyze(tt, vv, fr)[0] for fr in SPEC_F]
                top = max(spec) or 1e-9
                print(f"      ┌ {PN(hit)} 명중 상세  (동작: {STATE[0]}, 평균값 {lvls[hit]:.0f}/1023, "
                      f"0.2초 흔들림 {spreads[hit]})")
                print("      │ 최근 1초 주파수별 세기 (가장 센 것 = 10칸)")
                for fr, sp in zip(SPEC_F, spec):
                    bar = "█" * round(sp / top * 10)
                    mark = "  <- 감지 주파수" if fr == enemy else ""
                    print(f"      │ {fr:4d}Hz {sp:6.2f} {bar}{mark}")
                peak = SPEC_F[spec.index(top)]
                broad = sum(1 for sp in spec if sp > top * 0.5)
                events.append({"p": hit, "act": STATE[0], "peak": peak, "broad": broad,
                               "spread": spreads[hit], "lvl": lvls[hit], "over": len(over)})
                print(f"      └ 제일 센 주파수 {peak}Hz, 절반 넘는 주파수 {broad}개 / 13개")
    except KeyboardInterrupt:
        print("\n(중간에 멈춤)")
    finally:
        stop.set()
        th.join(timeout=2)
        stop_evt.set()
        proc.join(timeout=2)
        robot.close()
        f.close()

    # ---------- 요약 ----------
    dur = time.monotonic() - t_begin
    print(f"\n========== 요약: 모드 {mode} ({MODES[mode]}), {dur:.0f}초 ==========")
    print(" 패널 | 명중 | 기준넘음 | 최대비율 | 평균값 | 평균흔들림 | 값튐")
    for i, st in enumerate(stats):
        n = len(st["lvl"]) or 1
        print(f"  {PN(i)}  | {st['hit']:4d} | {st['over']:8d} | {st['max']:8.1f} | "
              f"{sum(st['lvl']) / n:6.0f} | {sum(st['spread']) / n:10.0f} | {st['jump']:4d}")
    print(f" 3개 이상 패널이 동시에 기준 넘은 구간: {multi}번")

    print("\n[원인 추정]")
    hints = set()
    for i, st in enumerate(stats):
        n = len(st["lvl"]) or 1
        lvl = sum(st["lvl"]) / n
        if lvl < 3:
            hints.add(f"{PN(i)}: 평균값이 0에 가까움 -> 패널 선이 끊겼거나 납땜 불량")
        if lvl > 1015:
            hints.add(f"{PN(i)}: 평균값이 1023 근처 -> 너무 밝거나 패널 +가 3.3V에 붙음")
        if st["jump"] > n * 0.05:
            hints.add(f"{PN(i)}: 값이 자주 튐 -> 접촉 불량(헐거운 선, 냉땜). 선을 흔들며 다시 보세요")
    for e in events:
        if e["over"] >= 3:
            hints.add("여러 패널이 한꺼번에 튐 -> 패널이 아니라 전원/GND 잡음 (GND 버스, 3.3V, 모터 전원)")
        elif e["spread"] >= JUMP:
            hints.add(f"{PN(e['p'])}: 명중 순간 값이 크게 튐 -> 접촉 불량 쪽 가능성 큼")
        elif e["broad"] >= 6:
            hints.add(f"{PN(e['p'])}: 모든 주파수가 고르게 셈 -> 순간적인 충격성 잡음(접촉, 그림자 급변)")
        elif e["peak"] == enemy and e["broad"] <= 2:
            hints.add(f"{PN(e['p'])}: 정확히 {enemy:.0f}Hz 빛이 들어옴 -> 상대 로봇 레이저가 켜져 있거나, "
                      f"그 주파수로 깜빡이는 조명/화면이 근처에 있음")
        elif e["peak"] in (100, 120):
            hints.add(f"{PN(e['p'])}: 100/120Hz 쪽이 셈 -> 형광등/LED 조명 깜빡임")
        if e["act"] not in ("-", ""):
            hints.add(f"'{e['act']}' 동작 중에 명중 -> 그 장치의 전기 잡음이 들어옴")
    if not events:
        hints.add("이번 모드에서는 가짜 명중 없음. 다른 모드도 해 보세요")
    for h in sorted(hints):
        print(" -", h)
    print(f"\n기록 파일: {LOG}")


def main():
    if len(sys.argv) >= 2 and sys.argv[1] in MODES:
        run(sys.argv[1], float(sys.argv[2]) if len(sys.argv) >= 3 else 30)
        return
    while True:
        print("\n가짜 명중 찾기 - 모드를 고르세요 (q = 끝)")
        for k, v in MODES.items():
            print(f"  {k}. {v}")
        k = input("> ").strip()
        if k.lower() == "q":
            break
        if k in MODES:
            sec = input("몇 초? (엔터 = 30) > ").strip()
            run(k, float(sec) if sec else 30)


if __name__ == "__main__":
    main()
