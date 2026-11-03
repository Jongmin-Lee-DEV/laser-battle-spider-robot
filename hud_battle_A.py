"""
hud_battle.py - 거미로봇 전투 HUD (PC에서 실행)

실행:  py hud_battle.py 192.168.137.125        <- 내 로봇(라즈베리파이) IP
       * 로봇 쪽에서는 python3 robot_server.py A (또는 B) 가 먼저 켜져 있어야 합니다.
       * 카메라 영상 표시에 Pillow 필요:  py -m pip install pillow
       * 로봇에 연결이 안 되면 예전처럼 연습용 가짜 화면으로 동작합니다.

키 배치 (엑스박스 패드 매핑용 = 잘 안 쓰는 키.  괄호 안 원래 키도 같이 동작)
    Home        전진              (↑)
    End         후진              (↓)
    Delete      왼쪽 제자리 회전  (←)
    Page Down   오른쪽 제자리 회전 (→)
    Insert      레이저 왼쪽으로   (Q)
    Page Up     레이저 오른쪽으로 (W)
    F9          레이저 발사       (1)   쿨타임 5초, 로봇 레이저는 1초 동안 깜빡이며 켜짐
    F8          전투 / 미로 모드 전환 (4)
    F6          미로 도착 표시 (5)  실제 미로 출구에 도착했을 때 누르기
    F5          패널 다시 보정 (6)  3초 동안 레이저를 쏘지 마세요
    F11         전체화면 (3)
    F12         리셋 (R)  체력 가득, 미로 타이머 다시 시작
    Esc         종료
    2           테스트 피격 (로봇에 연결 안 됐을 때만)

이동 규칙 (실제 로봇과 같음)
    두 다리 앞 = 전진 / 두 다리 뒤 = 후진 / 서로 반대 = 제자리 회전
    우선순위: 전진 > 후진 > 우 > 좌

모드
    전투: 카메라 영상 + 체력 / 레이저.  상대 레이저에 맞으면 로봇이 알려 줘서 체력 -1
          (A 로봇 25Hz, B 로봇 35Hz로 깜빡여 쏘고 상대 주파수만 감지 -> 내 레이저로는 안 맞음)
    미로: 카메라 영상 + 100초 타이머.  레이저 발사 / 조준 / 피격 모두 꺼짐 (로봇 쪽에서도 막음)

Aseprite 이미지(hud_assets 폴더)로 꾸미는 방법은 hud_test.py 와 같습니다.
"""

import base64
import io
import os
import random
import math
import time
import tkinter as tk
import json
import queue
import socket
import struct
import sys
import threading


# ==================== 설정 (숫자/색 바꾸는 곳) ====================
MAX_HP = 10          # 체력 칸 수
COOLDOWN = 5.0       # 레이저 쿨타임(초)
HIT_TIME = 2.0       # 피격 시 화면이 붉게 물드는 시간(초)
BEAM_TIME = 1.0      # 레이저 빔이 화면에 보이는 시간(초) = 로봇 레이저가 실제로 켜져 있는 시간

# ---- 가짜 로봇 / 레이저 좌우 (테스트 화면 반응용) ----
MOVE_SPEED = 3.0     # 전진/후진 속도 (미로에서 1초에 몇 칸 가는지)
TURN_RATE = 90.0     # 제자리 회전 속도 (도/초)
PAN_MAX = 45.0       # 레이저 좌우 최대 각도 (도). 가운데 0, 왼쪽 -, 오른쪽 +
PAN_RATE = 60.0      # 레이저 좌우로 돌아가는 속도 (도/초)
PAN_PX = 290         # 레이저가 최대 각도일 때 조준점이 가운데에서 떨어지는 거리 (화면 픽셀)
AIM_DX = 55           # 조준점 보정: 실제 레이저 점이 화면 가운데보다 오른쪽(+)/왼쪽(-)으로 떨어진 거리 (기본 화면 기준 픽셀)
AIM_DY = 35         # 조준점 보정: 실제 레이저 점이 가운데보다 아래(+)/위(-)로 떨어진 거리. 영상으로 맞춘 값
PX_PER_DEG = 6       # 회전할 때 기둥이 화면에서 흐르는 속도 (1도당 픽셀)

# ---- 로봇 연결 ----
ROBOT_IP = "192.168.137.125"    # 1번 로봇 (A) IP. IP가 바뀌면 여기만 고치면 됨 (실행할 때 py hud_battle_A.py 192.168.137.XX 처럼 줘도 됨)
HUD_ROBOT = "A"           # 이 HUD는 1번 로봇 (A) 전용
CTRL_PORT = 5000
VIDEO_PORT = 5001
SEND_INTERVAL = 0.05         # 조종 신호 보내는 간격(초)
SPAWN_GUARD = 1.5            # 리셋 / 모드 전환 직후 이 시간 동안은 피격 무시
CAM_TIMEOUT = 1.5            # 이 시간 동안 영상이 안 오면 NO SIGNAL
CAM_FLIP_H = True            # 카메라 영상 좌우 반전 (거울처럼 보이면 True)
CAM_FLIP_V = True            # 카메라 영상 위아래 반전 (거꾸로 보이면 True, 둘 다 True면 180도 회전)
# 채널 번호(CH0~CH7) 순서로 패널 이름. 로봇이 접속할 때 알려 주는 이름(A/B)에 맞춰 골라 씀
PANEL_NAMES_BY_ID = {
    "A": ["P1", "P2", "P3", "P4", "P5", "P6(없음)", "P7", "P8"],   # 1번 로봇: P6 떼어냄
    "B": ["P1(없음)", "P2", "P3", "P4", "P5", "P6", "P7", "P8"],   # 2번 로봇: 1번과 같은 순서 (4A~10A), P1 떼어냄
}
PANEL_NAMES = PANEL_NAMES_BY_ID[HUD_ROBOT]

# ---- 키 배치 (Tk 키 이름). 엑스박스 매핑 프로그램에서 이 키들을 패드 버튼에 할당 ----
KEYS = {
    "fwd":      {"Home", "Up"},
    "back":     {"End", "F4", "Down"},  # 노트북에서 End 대신 F4가 입력되는 경우도 후진
    "left":     {"Delete", "Left"},
    "right":    {"Next", "Right"},          # Next = Page Down
    "pan_l":    {"Insert", "q"},
    "pan_r":    {"Prior", "w"},             # Prior = Page Up
    "fire":     {"F9", "1", "KP_1"},
    "maze":     {"F8", "4", "KP_4"},
    "clear":    {"F6", "5", "KP_5"},
    "cal":      {"F5", "6", "KP_6"},
    "full":     {"F11", "3", "KP_3"},
    "reset":    {"F12", "r"},
    "test_hit": {"2", "KP_2"},
}

C = {
    "bg": "#14171c",
    "bezel": "#101010",
    "bezel_line": "#3a3f47",
    "bar": "#23262b",
    "panel": "#1a1c20",
    "panel_line": "#3a3f47",
    "text": "#e8ecf1",
    "muted": "#8b95a5",
    "hp": "#e02f2f",
    "hp_blink": "#ff6a6a",
    "hp_empty": "#3b1d1f",
    "hp_empty_line": "#552a2d",
    "ok": "#3ddc84",
    "warn": "#ffb84d",
    "danger": "#ff2a2a",
    "wall": "#22262d",          # 전체화면 좌우 벽 (기본 벽돌 무늬)
    "wall_line": "#15181d",
}
BRICK_W, BRICK_H = 48, 24       # 기본 벽돌 한 장 크기 (기본 배율 기준)

W, H = 720, 552
BEZEL = (10, 8, 710, 370)
SX0, SY0, SX1, SY1 = 30, 22, 690, 356          # 모니터 화면 영역
CX, CY = (SX0 + SX1) // 2, (SY0 + SY1) // 2    # 화면 중앙
AIM_Y = CY + AIM_DY                              # 조준점 높이 (실제 레이저 점 높이)
PAN_LEFT = min(PAN_MAX, (CX + AIM_DX - SX0 - 20) / PAN_PX * PAN_MAX)    # 왼쪽 최대 각도 (조준점이 화면 안에 있는 만큼)
PAN_RIGHT = min(PAN_MAX, (SX1 - 20 - CX - AIM_DX) / PAN_PX * PAN_MAX)   # 오른쪽 최대 각도
BAR_Y = 378

# ---- 상태바 안의 요소 위치 (이미지로 꾸밀 때 이 자리에 맞춰 그리면 됩니다) ----
HP_X, HP_Y, HP_W, HP_H, HP_STEP = 24, BAR_Y + 40, 20, 38, 24   # 체력 칸: 첫 칸 위치/크기/간격
FACE_CX, FACE_CY, FACE_SIZE = 360, BAR_Y + 74, 80              # 얼굴 중심/크기
LASER_X0, LASER_Y0, LASER_X1, LASER_Y1 = 440, BAR_Y + 40, 696, BAR_Y + 68   # 쿨타임 막대 칸
HIT_BORDER_W = 8                                # 이미지 테두리 사용 시, 피격 때 구멍 가장자리에 덧그리는 빨간 선 두께

SHOW_HELP = True        # 맨 밑 조작법 글자. 이미지에 직접 그려 넣었다면 False
ART_ZOOM = 1            # 이미지 확대 배율 (1 또는 2 권장)
ASSET_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "hud_assets")
ASSET_SIZES = {         # 파일 이름: 권장 크기 (확대 적용 후)
    "frame": (W, BAR_Y), "bar": (W, H - BAR_Y),
    "hp_on": (HP_W, HP_H), "hp_off": (HP_W, HP_H), "hp_low": (HP_W, HP_H), "hp_flash": (HP_W, HP_H),
    "face_happy": (FACE_SIZE, FACE_SIZE), "face_normal": (FACE_SIZE, FACE_SIZE),
    "face_hurt": (FACE_SIZE, FACE_SIZE), "face_ouch": (FACE_SIZE, FACE_SIZE),
    "face_dead": (FACE_SIZE, FACE_SIZE),
}

FONT_NUM = ("Consolas", 24, "bold")
FONT_LABEL = ("Consolas", 10, "bold")
FONT_SMALL = ("Malgun Gothic", 9)


# ==================== 게임 규칙 (화면과 분리: 나중에 그대로 재사용 가능) ====================
class GameState:
    def __init__(self, clock=time.time):
        self.clock = clock
        self.reset()

    def reset(self):
        self.hp = MAX_HP
        self.last_fire = -1e9
        self.hit_until = 0.0
        self.lost_idx = -1
        self.lost_until = 0.0
        self.deny_until = 0.0
        self.shots = 0

    @property
    def alive(self):
        return self.hp > 0

    def cooldown_left(self):
        return max(0.0, self.last_fire + COOLDOWN - self.clock())

    def cooldown_frac(self):
        """0.0(막 쐈음) ~ 1.0(발사 가능)"""
        return 1.0 - self.cooldown_left() / COOLDOWN

    def fire(self):
        """발사 시도. 실제로 발사되면 True, 쿨타임/사망이면 False"""
        now = self.clock()
        if not self.alive:
            return False
        if self.cooldown_left() > 0:
            self.deny_until = now + 0.5
            return False
        self.last_fire = now
        self.shots += 1
        return True

    def beam_on(self):
        return self.clock() - self.last_fire < BEAM_TIME

    def hit(self):
        """피격. 체력 1칸 감소. 이미 사망이면 False"""
        if not self.alive:
            return False
        now = self.clock()
        self.hp -= 1
        self.lost_idx = self.hp          # 방금 사라진 칸 번호 (깜빡임 효과용)
        self.lost_until = now + 0.3
        self.hit_until = now + HIT_TIME
        return True

    def hit_left(self):
        return max(0.0, self.hit_until - self.clock())


def motor_outputs(go=False, back=False, left=False, right=False, brake=False):
    """실제 로봇 motor_cont.drive() 와 똑같은 규칙으로 (오른쪽 모터, 왼쪽 모터) 방향을 계산합니다.
    +1 = 앞으로, -1 = 뒤로, 0 = 정지.   우선순위: 브레이크 > 전진 > 후진 > 우 > 좌
    (후진 + 좌우의 방향이 반대인 것까지 drive() 코드 그대로입니다.)"""
    if brake:
        return 0, 0
    if go:
        if bool(left) == bool(right):            # 직진 (둘 다 안 눌렀거나 둘 다 눌렀을 때)
            return 1, 1
        return (1, -1) if left else (-1, 1)      # 좌: 오른쪽 앞 / 왼쪽 뒤.  우: 오른쪽 뒤 / 왼쪽 앞
    if back:
        if bool(left) == bool(right):            # 후진
            return -1, -1
        return (-1, 1) if left else (1, -1)      # 후진+좌: 오른쪽 뒤 / 왼쪽 앞.  후진+우: 오른쪽 앞 / 왼쪽 뒤
    if right:
        return -1, 1
    if left:
        return 1, -1
    return 0, 0


# ==================== 테스트 미로 ====================
# '#' = 붉은 벽돌 벽, 'B' = 푸른 벽, 'E' = 출구 벽(초록), '.' = 길.  글자만 바꾸면 미로 모양이 바뀝니다.
# 출발: 왼쪽 아래 칸 / 도착: 오른쪽 위 칸 (출구 벽 바로 앞). 도착 칸에 들어가면 클리어.
MAZE = [
    "#####################",
    "#.............#.....E",
    "#.#####.#####.#####.#",
    "#...#...#...#.....#.#",
    "###.#####.#.#####.#.#",
    "#...#...#.#.#.....#.#",
    "#.###.#.#.#.#####.#.#",
    "#.....#...#.#...#...#",
    "###########.#.#.#####",
    "#.....#...#...#.....#",
    "#.###.#.#.#######.#.#",
    "#...#.#.#.......#.#.#",
    "###.#.#.#######.###.#",
    "#...#...#...........#",
    "#####################",
]
MAZE_START = (1.5, 13.5, 90.0)       # 출발 위치 (x, y, 바라보는 방향 도: 0=북 90=동)
MAZE_GOAL = (19, 1)                  # 도착 칸 (이 칸에 들어가면 클리어)
MAZE_W, MAZE_H = len(MAZE[0]), len(MAZE)
MAZE_TIME = 100.0            # 미로 모드 제한 시간 (초) = 1분 40초

# 레이저 테스트용 맵: 넓은 방 (벽 가장자리 + 기둥 몇 개)
ARENA = [
    "###############",
    "#.............#",
    "#.............#",
    "#..B.......B..#",
    "#.............#",
    "#.............#",
    "#.............#",
    "#..B.......B..#",
    "#.............#",
    "#.............#",
    "###############",
]
START_POS = (1.5, 9.5)       # 시작 위치 (왼쪽 아래 칸 가운데). 시작 방향은 위(북쪽)
FOV_PLANE = 0.70             # 시야각 (0.70 = 약 70도)
BODY_R = 0.2                 # 로봇 몸 반지름 (벽에 이만큼 이상 못 붙음)
WALL_RGB = {"#": (190, 90, 70), "B": (70, 110, 180), "E": (60, 210, 110)}
COLUMN_PX = 8                # 모니터 화면을 이 폭(픽셀)씩 잘라 벽을 그림


def maze_cell(x, y, grid=None):
    grid = grid or ARENA
    xi, yi = int(x // 1), int(y // 1)
    if 0 <= yi < len(grid) and 0 <= xi < len(grid[0]):
        return grid[yi][xi]
    return "#"


def raycast(px, py, rdx, rdy, grid=None):
    """(px,py)에서 (rdx,rdy) 방향으로 쏜 시선이 처음 닿는 벽: (거리, 닿은 면(0=세로면/1=가로면), 벽 글자, 벽 위 위치 0~1)"""
    mx, my = int(px // 1), int(py // 1)
    ddx = abs(1 / rdx) if rdx else 1e30
    ddy = abs(1 / rdy) if rdy else 1e30
    sx, sdx = (1, (mx + 1 - px) * ddx) if rdx > 0 else (-1, (px - mx) * ddx)
    sy, sdy = (1, (my + 1 - py) * ddy) if rdy > 0 else (-1, (py - my) * ddy)
    side = 0
    for _ in range(64):
        if sdx < sdy:
            sdx += ddx; mx += sx; side = 0
        else:
            sdy += ddy; my += sy; side = 1
        ch = maze_cell(mx, my, grid)
        if ch != ".":
            dist = (sdx - ddx) if side == 0 else (sdy - ddy)
            wx = (py + dist * rdy) if side == 0 else (px + dist * rdx)
            return max(dist, 0.01), side, ch, wx - (wx // 1)
    return 64.0, 0, "#", 0.0


def _shade(rgb, f):
    return "#%02x%02x%02x" % tuple(max(0, min(255, int(v * f))) for v in rgb)


class RobotSim:
    """테스트용 가짜 로봇. 키 입력을 실제 로봇과 같은 규칙(motor_outputs)으로 모터 방향으로 바꾸고,
    그 결과로 앞뒤 이동 / 제자리 회전 / 레이저 좌우 각도를 계산합니다.
    진짜 로봇을 붙이면 이 값들 대신 로봇이 보내 주는 실제 상태가 들어갑니다."""

    def __init__(self):
        self.grid = ARENA        # 지금 맵 (레이저 테스트 = ARENA / 미로 모드 = MAZE)
        self.start = START_POS + (0.0,)      # (x, y, 방향)
        self.reset()

    def reset(self):
        self.dist = 0.0          # 움직인 거리 (전진 +, 후진 -)
        self.x, self.y, self.heading = self.start   # 위치(칸 단위) / 바라보는 방향(도, 0~360)
        self.pan = 0.0           # 레이저 좌우 각도 (도)
        self.motors = (0, 0)     # (오른쪽 모터, 왼쪽 모터)  +1 앞 / -1 뒤 / 0 정지
        self.cmd = "STOP"        # 지금 이동 글자

    def _free(self, x, y):
        r = BODY_R
        return all(maze_cell(x + dx, y + dy, self.grid) == "." for dx in (-r, r) for dy in (-r, r))

    @property
    def motor_text(self):
        sym = {1: "+", -1: "-", 0: "0"}
        return f"R{sym[self.motors[0]]} L{sym[self.motors[1]]}"

    def update(self, dt, fwd=False, back=False, left=False, right=False,
               pan_left=False, pan_right=False, locked=False):
        if locked:                                           # 체력 0: 아무것도 못 움직임
            fwd = back = left = right = pan_left = pan_right = False
        r, l = motor_outputs(go=fwd, back=back, left=left, right=right)
        self.motors = (r, l)
        move = (r + l) / 2                                   # 두 모터가 같은 방향이면 이동
        turn = (l - r) / 2                                   # 서로 반대면 제자리 회전 (+ 오른쪽 / - 왼쪽)
        self.dist += move * MOVE_SPEED * dt
        self.heading = (self.heading + turn * TURN_RATE * dt) % 360
        if move:                                             # 바라보는 방향으로 이동 (벽은 통과 못 함, 벽을 따라 미끄러짐)
            h = math.radians(self.heading)
            step = move * MOVE_SPEED * dt
            nx, ny = self.x + math.sin(h) * step, self.y - math.cos(h) * step
            if self._free(nx, self.y):
                self.x = nx
            if self._free(self.x, ny):
                self.y = ny
        aim = int(pan_right) - int(pan_left)
        self.pan = max(-PAN_LEFT, min(PAN_RIGHT, self.pan + aim * PAN_RATE * dt))
        self.cmd = {(1, 1): "FWD", (-1, -1): "BACK", (1, -1): "TURN LEFT",
                    (-1, 1): "TURN RIGHT"}.get((r, l), "STOP")


# ==================== 전체화면 확대 ====================
def _resize_png_b64(path, w, h):
    """PNG 파일을 (w, h)로 '최근접' 방식(도트가 뭉개지지 않게)으로 확대/축소해 base64 PNG 데이터로 돌려줌.
    Pillow -> OpenCV 순서로 시도하고, 둘 다 없거나 실패하면 None."""
    try:
        from PIL import Image
        im = Image.open(path).convert("RGBA").resize((w, h), Image.NEAREST)
        buf = io.BytesIO()
        im.save(buf, "PNG")
        return base64.b64encode(buf.getvalue()).decode("ascii")
    except ImportError:
        pass
    except Exception:
        return None
    try:
        import cv2
        im = cv2.imread(path, cv2.IMREAD_UNCHANGED)
        if im is None:
            return None
        im = cv2.resize(im, (w, h), interpolation=cv2.INTER_NEAREST)
        ok, buf = cv2.imencode(".png", im)
        return base64.b64encode(buf.tobytes()).decode("ascii") if ok else None
    except Exception:
        return None


def _wide_png_b64(path, w, h, left, right):
    """PNG 를 (w, h) 로 키운 뒤, 맨 왼쪽/맨 오른쪽 1픽셀 세로줄을 양옆(left, right 픽셀)으로 늘려
    가로로 더 넓힌 이미지를 base64 PNG 데이터로 돌려줌. Pillow -> OpenCV 순서로 시도, 실패하면 None."""
    try:
        from PIL import Image
        im = Image.open(path).convert("RGBA").resize((w, h), Image.NEAREST)
        out = Image.new("RGBA", (left + w + right, h), (0, 0, 0, 0))
        out.paste(im, (left, 0))
        if left:
            out.paste(im.crop((0, 0, 1, h)).resize((left, h), Image.NEAREST), (0, 0))
        if right:
            out.paste(im.crop((w - 1, 0, w, h)).resize((right, h), Image.NEAREST), (left + w, 0))
        buf = io.BytesIO()
        out.save(buf, "PNG")
        return base64.b64encode(buf.getvalue()).decode("ascii")
    except ImportError:
        pass
    except Exception:
        return None
    try:
        import cv2
        import numpy as np
        im = cv2.imread(path, cv2.IMREAD_UNCHANGED)
        if im is None:
            return None
        if im.ndim == 2:
            im = cv2.cvtColor(im, cv2.COLOR_GRAY2BGRA)
        elif im.shape[2] == 3:
            im = cv2.cvtColor(im, cv2.COLOR_BGR2BGRA)
        im = cv2.resize(im, (w, h), interpolation=cv2.INTER_NEAREST)
        parts = ([np.repeat(im[:, :1], left, axis=1)] if left else []) + [im] \
            + ([np.repeat(im[:, -1:], right, axis=1)] if right else [])
        ok, buf = cv2.imencode(".png", np.concatenate(parts, axis=1))
        return base64.b64encode(buf.tobytes()).decode("ascii") if ok else None
    except Exception:
        return None


class View:
    """그리기 코드는 기본 크기(720x552) 좌표로 그리고, 이 클래스가 전체화면 배율에 맞춰
    좌표 / 선 굵기 / 글자 크기 / 이미지를 키워서 실제 캔버스에 대신 그려 줍니다."""

    def __init__(self, canvas, scale_image):
        self.cv = canvas
        self.scale_image = scale_image
        self.f, self.ox, self.oy = 1.0, 0.0, 0.0

    def set_view(self, f, ox, oy):
        self.f, self.ox, self.oy = f, ox, oy

    @property
    def identity(self):
        return self.f == 1.0 and self.ox == 0 and self.oy == 0

    def _pts(self, xy):
        if self.identity:
            return list(xy)
        return [(self.ox + v * self.f) if i % 2 == 0 else (self.oy + v * self.f) for i, v in enumerate(xy)]

    def sy(self, y):
        """기본 좌표 y -> 실제 화면 y"""
        return self.oy + y * self.f

    def x_span(self, total_w):
        """실제 화면 가로 전체(0 ~ total_w)에 해당하는 기본 좌표 x 범위"""
        return (-self.ox / self.f, (total_w - self.ox) / self.f)

    def _kw(self, kw, line=False):
        kw = dict(kw)
        kw.setdefault("tags", "dyn")                          # 매 프레임 지우는 항목 표시 (벽은 따로 유지)
        if self.f == 1.0:
            return kw
        if line:                                              # 선 굵기
            kw["width"] = max(1, round(kw.get("width", 1) * self.f))
        font = kw.get("font")
        if font:                                              # 글자 크기
            kw["font"] = (font[0], max(6, round(font[1] * self.f))) + tuple(font[2:])
        return kw

    def create_rectangle(self, *xy, **kw): return self.cv.create_rectangle(*self._pts(xy), **self._kw(kw, True))
    def create_line(self, *xy, **kw):      return self.cv.create_line(*self._pts(xy), **self._kw(kw, True))
    def create_oval(self, *xy, **kw):      return self.cv.create_oval(*self._pts(xy), **self._kw(kw, True))
    def create_arc(self, *xy, **kw):       return self.cv.create_arc(*self._pts(xy), **self._kw(kw, True))
    def create_text(self, x, y, **kw):     return self.cv.create_text(*self._pts((x, y)), **self._kw(kw))

    def create_image(self, x, y, image=None, **kw):
        return self.cv.create_image(*self._pts((x, y)), image=self.scale_image(image, self.f), **self._kw(kw))

    def delete(self, *a):
        self.cv.delete(*a)


# ==================== 로봇 통신 ====================
class RobotLink:
    """로봇(라즈베리파이)과 통신: 조종 신호 보내기 / 명중 알림 받기 / 카메라 영상 받기. 끊기면 자동 재연결."""

    def __init__(self, ip):
        self.ip = ip
        self.connected = False
        self.events = queue.Queue()
        self.frame = None            # 최신 카메라 JPEG
        self.frame_seq = 0
        self.frame_time = 0.0
        self._sock = None
        self._lock = threading.Lock()
        threading.Thread(target=self._ctl_loop, daemon=True).start()
        threading.Thread(target=self._video_loop, daemon=True).start()

    def send(self, obj):
        with self._lock:
            s = self._sock
            if s is None:
                return
            try:
                s.sendall((json.dumps(obj) + "\n").encode())
            except OSError:
                pass

    def _ctl_loop(self):
        while True:
            s = None
            try:
                s = socket.create_connection((self.ip, CTRL_PORT), timeout=2)
                s.settimeout(3)                          # 로봇이 1초마다 ping을 보냄
                s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                with self._lock:
                    self._sock = s
                self.connected = True
                self.events.put({"t": "conn", "on": True})
                buf = b""
                while True:
                    data = s.recv(4096)
                    if not data:
                        break
                    buf += data
                    while b"\n" in buf:
                        line, buf = buf.split(b"\n", 1)
                        try:
                            self.events.put(json.loads(line))
                        except ValueError:
                            pass
            except OSError:
                pass
            finally:
                with self._lock:
                    self._sock = None
                if s is not None:
                    try:
                        s.close()
                    except OSError:
                        pass
                if self.connected:
                    self.connected = False
                    self.events.put({"t": "conn", "on": False})
            time.sleep(1.0)

    def _video_loop(self):
        while True:
            try:
                with socket.create_connection((self.ip, VIDEO_PORT), timeout=2) as s:
                    s.settimeout(3)
                    while True:
                        n = struct.unpack(">I", self._recvn(s, 4))[0]
                        if n > 5_000_000:
                            break
                        self.frame = self._recvn(s, n)
                        self.frame_time = time.time()
                        self.frame_seq += 1
            except OSError:
                pass
            time.sleep(1.0)

    @staticmethod
    def _recvn(s, n):
        buf = bytearray()
        while len(buf) < n:
            chunk = s.recv(n - len(buf))
            if not chunk:
                raise ConnectionError("closed")
            buf += chunk
        return bytes(buf)


# ==================== 화면 ====================
class HUD:
    def __init__(self, root):
        self.root = root
        self.state = GameState()
        self.sim = RobotSim()                    # 가짜 로봇 (이동/회전/레이저 좌우)
        self.pressed = set()                     # 지금 눌려 있는 키 (연속 입력 방지 + 누르고 있는 동안 동작용)
        self._last = None                        # 이전 프레임 시각
        self._prev_cmd = "STOP"
        self._prev_locked = False
        self.maze_mode = False                   # 4번: 미로 모드
        self.maze_t0 = 0.0
        self.timeup = False
        self.cleared = False                     # 도착 칸에 들어옴
        self.clear_left = 0.0                    # 클리어했을 때 남은 시간
        self.imgs = {}                           # 불러온 이미지 (참조를 들고 있어야 화면에서 안 사라짐)
        self.src = {}                            # 이미지 -> 원본 파일 경로 (전체화면 확대용)
        self._scaled_cache = {}
        self._warned_scale = False
        self.fullscreen = False
        self.sw, self.sh = W, H                  # 지금 캔버스의 실제 크기 (전체화면이면 화면 크기)
        self.link = RobotLink(ROBOT_IP)          # 로봇 연결 (백그라운드에서 자동 재연결)
        self.robot_info = {}
        self._last_send = 0.0
        self._cam_seq = -1
        self._cam_photo = None
        self._cam_size = None
        self._cam_live = False
        self._pil_warned = False
        self.notice, self.notice_until = "", 0.0
        self.last_hit, self.last_hit_until = None, 0.0
        self.guard_until = 0.0                  # 리셋 직후 무적 시간

        root.title(f"Spider Battle HUD [1번 로봇 (A)] - {ROBOT_IP}")
        root.configure(bg=C["bg"])
        root.resizable(False, False)
        self.cv = tk.Canvas(root, width=W, height=H, bg=C["bg"], highlightthickness=0)
        self.cv.pack()
        self.c = View(self.cv, self._scaled)     # 이후 모든 그리기는 self.c (배율 적용) 를 거침
        self.load_assets()

        root.bind("<KeyPress>", self.on_press)
        root.bind("<KeyRelease>", self.on_release)
        root.bind("<FocusOut>", lambda e: self.pressed.clear())   # 창이 포커스를 잃으면 눌린 키 모두 해제 (키가 눌린 채 멈추는 것 방지)
        root.focus_force()
        self.draw()

    # ---------- 이미지 불러오기 ----------
    def load_assets(self):
        """hud_assets 폴더의 PNG를 불러옵니다. 없는 파일은 건너뛰고 코드로 그린 기본 모양을 씁니다."""
        for name, size in list(ASSET_SIZES.items()) + [("wall", None)]:    # wall 은 크기 제한 없음(타일)
            path = os.path.join(ASSET_DIR, name + ".png")
            if not os.path.exists(path):
                continue
            try:
                img = tk.PhotoImage(file=path)
                if ART_ZOOM > 1:
                    img = img.zoom(ART_ZOOM)
            except Exception as e:
                print(f"[hud] 이미지 불러오기 실패 ({name}.png): {e}")
                continue
            if size is not None and (img.width(), img.height()) != size:
                print(f"[hud] 경고: {name}.png 크기가 {img.width()}x{img.height()} 입니다. "
                      f"권장 {size[0]}x{size[1]} (위치가 어긋날 수 있음)")
            self.imgs[name] = img
            self.src[id(img)] = path
        if self.imgs:
            print("[hud] 이미지 사용:", ", ".join(sorted(self.imgs)))

    # ---------- 전체화면 ----------
    def _scaled(self, img, f):
        """이미지를 배율 f에 맞게 키운 버전을 돌려줌 (같은 크기는 캐시)."""
        if img is None or abs(f - 1.0) < 1e-6:
            return img
        w, h = max(1, round(img.width() * f)), max(1, round(img.height() * f))
        path = self.src.get(id(img))
        key = (path, w, h)
        if key in self._scaled_cache:
            return self._scaled_cache[key]
        out = None
        if path is not None:
            data = _resize_png_b64(path, w, h)
            if data is not None:
                try:
                    out = tk.PhotoImage(data=data)
                except Exception as e:
                    print(f"[hud] 확대 이미지 만들기 실패: {e}")
        if out is None:                                       # Pillow/OpenCV 없음 -> 정수 배율로만 확대
            if not self._warned_scale:
                print("[hud] 경고: Pillow 또는 OpenCV 가 없어 전체화면에서 이미지 크기가 정확히 맞지 않습니다. "
                      "(py -m pip install pillow)")
                self._warned_scale = True
            out = img.zoom(int(f)) if f >= 2 else img
        self._scaled_cache[key] = out
        return out

    def _wide_bar(self, bar):
        """전체화면용: bar.png 를 배율에 맞게 키우고 양옆 가장자리를 늘려 화면 가로 전체 크기로 만든 이미지.
        Pillow/OpenCV 가 없거나 실패하면 None (이때는 상태바 띠 + 이미지 그대로 그림)."""
        path = self.src.get(id(bar))
        if path is None:
            return None
        f = self.c.f
        w, h = max(1, round(bar.width() * f)), max(1, round(bar.height() * f))
        left = max(0, round(self.c.ox))
        right = max(0, self.sw - left - w)
        key = ("wide", path, w, h, left, right)
        if key in self._scaled_cache:
            return self._scaled_cache[key]
        out = None
        data = _wide_png_b64(path, w, h, left, right)
        if data is not None:
            try:
                out = tk.PhotoImage(data=data)
            except Exception as e:
                print(f"[hud] 상태바 이미지 만들기 실패: {e}")
        self._scaled_cache[key] = out                         # 실패(None)도 기억해서 매 프레임 재시도하지 않음
        return out

    def _build_wall(self):
        """전체화면일 때 모니터 좌우(와 위) 여백을 채우는 벽. 한 번만 그려 두고 계속 유지."""
        cv = self.cv
        cv.delete("wall")
        if self.c.identity:                                   # 창 모드에는 여백이 없음
            return
        f, sw, sh = self.c.f, self.sw, self.sh
        tile = self.imgs.get("wall")
        if tile is not None:                                  # 내가 그린 벽 타일을 반복해서 깔기
            t = self._scaled(tile, f)
            tw, th = max(1, t.width()), max(1, t.height())
            for y in range(0, sh, th):
                for x in range(0, sw, tw):
                    cv.create_image(x, y, image=t, anchor="nw", tags="wall")
        else:                                                 # 기본: 벽돌 무늬
            cv.create_rectangle(0, 0, sw, sh, fill=C["wall"], outline="", tags="wall")
            bw, bh = max(8, round(BRICK_W * f)), max(4, round(BRICK_H * f))
            lw = max(1, round(f))
            for r, y in enumerate(range(0, sh, bh)):
                cv.create_line(0, y, sw, y, fill=C["wall_line"], width=lw, tags="wall")
                off = bw // 2 if r % 2 else 0
                for x in range(-off, sw, bw):
                    cv.create_line(x, y, x, y + bh, fill=C["wall_line"], width=lw, tags="wall")
        cv.tag_lower("wall")                                  # 항상 맨 아래

    def toggle_fullscreen(self):
        r = self.root
        self.fullscreen = not self.fullscreen
        if self.fullscreen:
            r.resizable(True, True)
            r.attributes("-fullscreen", True)
            r.update_idletasks()
            sw, sh = r.winfo_screenwidth(), r.winfo_screenheight()
            f = min(sw / W, sh / H)                           # 비율 유지하며 화면에 꽉 차게
            self.sw, self.sh = sw, sh
            self.cv.config(width=sw, height=sh)
            # 가로는 가운데, 세로는 아래 맞춤 (상태바가 항상 화면 맨 아래에 붙음)
            self.c.set_view(f, (sw - W * f) / 2, sh - H * f)
            self._build_wall()
            print(f"[hud] 전체화면 ON  ({sw}x{sh}, 배율 {f:.2f})")
        else:
            r.attributes("-fullscreen", False)
            self.sw, self.sh = W, H
            self.cv.config(width=W, height=H)
            self.c.set_view(1.0, 0.0, 0.0)
            self._build_wall()                                # 벽 지우기
            r.resizable(False, False)
            r.geometry("")                                    # 원래 크기로
            print("[hud] 전체화면 OFF")

    # ---------- 키 입력 ----------
    @staticmethod
    def _key(e):
        """Shift/CapsLock 때문에 'Q' 와 'q' 가 섞여도 같은 키로 취급"""
        k = e.keysym
        return k.lower() if len(k) == 1 else k

    def on_press(self, e):
        k = self._key(e)
        if k == "Escape":
            self.root.destroy()
            return
        if k in self.pressed:                    # 꾹 눌러도 한 번만 인식
            return
        self.pressed.add(k)
        if k in KEYS["fire"]:
            if self.maze_mode:
                print("[laser] 미로 모드에서는 레이저를 쓸 수 없음")
            elif self.state.fire():
                self.link.send({"t": "fire"})
                print("[laser] 발사!")
            else:
                print("[laser] 쿨타임 중 또는 사망 상태")
        elif k in KEYS["test_hit"]:
            if self.link.connected:
                print("[hit] 로봇에 연결된 상태에서는 테스트 피격 키가 꺼져 있음")
            elif self.maze_mode:
                print("[hit] 미로 모드에서는 피격 없음")
            elif self.state.hit():
                print(f"[hit] 테스트 피격! 남은 체력 {self.state.hp}/{MAX_HP}")
        elif k in KEYS["full"]:
            self.toggle_fullscreen()
        elif k in KEYS["maze"]:
            self.toggle_maze()
        elif k in KEYS["clear"]:
            self.mark_clear()
        elif k in KEYS["cal"]:
            if self.link.connected:
                self.link.send({"t": "cal"})
                self.say("패널 다시 보정 중 (3초) - 레이저를 쏘지 마세요", 3.5)
            else:
                self.say("로봇에 연결되어 있지 않음")
        elif k in KEYS["reset"]:
            self.guard_until = time.time() + SPAWN_GUARD
            self.state.reset()
            self.sim.reset()
            self.timeup = False
            self.cleared = False
            self.maze_t0 = time.time()
            print("[reset]")

    def toggle_maze(self):
        """4번: 미로 모드 <-> 레이저 테스트 모드. 바꿀 때마다 출발 지점으로 이동하고 미로는 100초 타이머가 새로 시작."""
        self.maze_mode = not self.maze_mode
        self.guard_until = time.time() + SPAWN_GUARD
        self.sim.grid = MAZE if self.maze_mode else ARENA
        self.sim.start = MAZE_START if self.maze_mode else START_POS + (0.0,)
        self.sim.reset()
        self.timeup = False
        self.cleared = False
        self.maze_t0 = time.time()
        print("[mode] 미로 (100초 타이머 시작, 레이저 꺼짐)" if self.maze_mode else "[mode] 전투")

    def maze_left(self, now):
        if self.cleared:
            return self.clear_left                            # 클리어하면 타이머 멈춤
        return max(0.0, MAZE_TIME - (now - self.maze_t0))

    def maze_locked(self):
        return self.maze_mode and (self.timeup or self.cleared)

    def on_release(self, e):
        self.pressed.discard(self._key(e))

    # ---------- 가짜 로봇 ----------
    def on_lock_change(self, locked):
        """체력 0 -> 이동 잠금 / 리셋 -> 잠금 해제.  나중에 여기서 진짜 로봇으로 신호를 보냅니다."""
        print("[lock] 이동 잠금 신호 (체력 0 또는 시간 종료)" if locked else "[unlock] 잠금 해제 신호")

    def update_robot(self, dt):
        p = self.pressed
        if self.maze_mode and not self.timeup and not self.cleared and self.maze_left(time.time()) <= 0:
            self.timeup = True
            print("[timeup] 100초 종료 -> 로봇 이동 잠금 신호")
        if self.maze_mode and not self.link.connected and not self.cleared and not self.timeup and \
                (int(self.sim.x), int(self.sim.y)) == MAZE_GOAL:
            self.clear_left = self.maze_left(time.time())    # 먼저 남은 시간을 기록하고 나서 정지
            self.cleared = True
            print(f"[clear] 도착! 남은 시간 {int(self.clear_left // 60)}:{int(self.clear_left % 60):02d} -> 로봇 정지 신호")
        locked = (not self.state.alive) or self.maze_locked()
        if locked != self._prev_locked:
            self._prev_locked = locked
            self.on_lock_change(locked)
        self.sim.update(dt, fwd=self.held("fwd"), back=self.held("back"), left=self.held("left"),
                        right=self.held("right"),
                        pan_left=self.held("pan_l") and not self.maze_mode,
                        pan_right=self.held("pan_r") and not self.maze_mode, locked=locked)
        self.send_ctl(locked)
        if self.sim.cmd != self._prev_cmd:                    # 이동 명령이 바뀔 때만 한 줄 출력
            self._prev_cmd = self.sim.cmd
            print(f"[move] {self.sim.cmd}  {self.sim.motor_text}")

    # ---------- 로봇 연결 ----------
    def held(self, action):
        """그 동작에 할당된 키 중 하나라도 눌려 있으면 True"""
        return any(k in self.pressed for k in KEYS[action])

    def say(self, text, sec=2.5):
        self.notice, self.notice_until = text, time.time() + sec
        print(f"[info] {text}")

    def mark_clear(self):
        """F6: 실제 미로 출구에 도착했을 때 직접 눌러서 클리어 처리 (타이머 멈춤, 로봇 정지)"""
        if self.maze_mode and not self.cleared and not self.timeup:
            self.clear_left = self.maze_left(time.time())
            self.cleared = True
            print(f"[clear] 도착! 남은 시간 {int(self.clear_left // 60)}:{int(self.clear_left % 60):02d}")

    def send_ctl(self, locked):
        """조종 상태를 로봇에 보냄 (0.05초마다). 로봇은 0.5초 동안 못 받으면 스스로 멈춤"""
        now = time.time()
        if now - self._last_send < SEND_INTERVAL:
            return
        self._last_send = now
        self.link.send({"t": "ctl",
                        "f": int(self.held("fwd")), "b": int(self.held("back")),
                        "l": int(self.held("left")), "r": int(self.held("right")),
                        "pan": 0.0 if self.maze_mode else round(self.sim.pan, 1),
                        "lock": int(locked), "maze": int(self.maze_mode)})

    def handle_events(self):
        """로봇에서 온 소식 처리: 연결 / 명중 / 보정 결과"""
        while True:
            try:
                m = self.link.events.get_nowait()
            except queue.Empty:
                return
            t = m.get("t")
            if t == "conn":
                self.say("로봇 연결됨" if m.get("on") else "로봇 연결 끊김 - 다시 연결 중...")
            elif t == "hello":
                self.robot_info = m
                if str(m.get("id", "")).upper() != HUD_ROBOT:
                    print(f"[link] 경고: 이 HUD는 {HUD_ROBOT} 로봇용인데 접속한 로봇은 {m.get('id')} 입니다. IP를 확인하세요.")
                    self.say(f"주의: {m.get('id')} 로봇에 연결됨 (이 HUD는 {HUD_ROBOT}용)")
                print(f"[link] 로봇 {m.get('id')}: {m.get('own'):.0f}Hz로 쏘고 {m.get('enemy'):.0f}Hz 감지")
            elif t == "hit":
                if self.maze_mode or not self.state.alive:
                    continue
                if time.time() < self.guard_until:
                    print("[hit] 리셋 직후라 무시")
                    continue
                if self.state.hit():
                    i = m.get("panel", 0)
                    name = PANEL_NAMES[i] if 0 <= i < len(PANEL_NAMES) else f"CH{i}"
                    self.last_hit, self.last_hit_until = name, time.time() + 1.5
                    print(f"[hit] {name} 명중! 남은 체력 {self.state.hp}/{MAX_HP}")
            elif t == "cal":
                self.say("패널 보정 완료")
                print(f"[cal] 평소값 {m.get('level')}  기준 {m.get('thr')}  측정 {m.get('rate')}회/초")

    def camera_photo(self):
        """최신 카메라 프레임을 모니터 크기에 맞춘 이미지로. 영상이 없으면 None"""
        link = self.link
        if link.frame is None or time.time() - link.frame_time > CAM_TIMEOUT:
            return None
        f = self.c.f
        size = (max(1, round((SX1 - SX0) * f)), max(1, round((SY1 - SY0) * f)))
        if link.frame_seq != self._cam_seq or size != self._cam_size:
            try:
                from PIL import Image, ImageTk
            except ImportError:
                if not self._pil_warned:
                    print("[cam] Pillow 가 없어 영상을 못 띄웁니다:  py -m pip install pillow")
                    self._pil_warned = True
                return None
            try:
                im = Image.open(io.BytesIO(link.frame)).convert("RGB").resize(size, Image.BILINEAR)
                if CAM_FLIP_H:
                    im = im.transpose(Image.FLIP_LEFT_RIGHT)
                if CAM_FLIP_V:
                    im = im.transpose(Image.FLIP_TOP_BOTTOM)
                self._cam_photo = ImageTk.PhotoImage(im)
                self._cam_seq, self._cam_size = link.frame_seq, size
            except Exception:
                pass
        return self._cam_photo

    def aim_x(self):
        """레이저 좌우 각도에 따른 조준점의 화면 x 좌표"""
        return CX + AIM_DX + self.sim.pan / PAN_MAX * PAN_PX

    # ---------- 매 프레임 ----------
    def draw(self):
        self.cv.delete("dyn")                    # 벽("wall")은 지우지 않고 유지
        now = time.time()
        dt = 0.0 if self._last is None else min(0.1, max(0.0, now - self._last))
        self._last = now
        self.handle_events()
        self.update_robot(dt)
        self.draw_monitor(now)
        self.draw_status(now)
        self.draw_help()
        # 그리는 데 걸린 시간을 빼고 다음 프레임 예약 (그리기가 느려도 입력 반응이 밀리지 않게)
        self.root.after(max(1, 16 - int((time.time() - now) * 1000)), self.draw)   # 최대 약 60fps

    # ---------- 모니터 ----------
    def draw_monitor(self, now):
        c, s = self.c, self.state
        hit_left = s.hit_left()
        red = hit_left > 0 or not s.alive
        frame = self.imgs.get("frame")

        # 테두리: 피격 후 2초 동안 빨갛게  (frame.png 가 있으면 그 이미지를 쓰고 아래에서 덧그림)
        if frame is None:
            if red:
                c.create_rectangle(*BEZEL, fill=C["bezel"], outline=C["danger"], width=8)
            else:
                c.create_rectangle(*BEZEL, fill=C["bezel"], outline=C["bezel_line"], width=4)

        self.draw_feed(now, hit_left)

        # 레이저 빔
        if s.beam_on() and not self.maze_mode:
            ax = self.aim_x()
            c.create_line(CX, SY1, ax, AIM_Y, fill="#ff3b3b", width=6)
            c.create_line(CX, SY1, ax, AIM_Y, fill="#ffffff", width=2)
            c.create_oval(ax - 12, AIM_Y - 12, ax + 12, AIM_Y + 12, fill="#ffd0d0", outline="")

        # 붉은 화면 효과 (점점 옅어짐)
        if not s.alive:
            c.create_rectangle(SX0, SY0, SX1, SY1, fill="#8a0000", stipple="gray50", outline="")
        elif hit_left > 0:
            frac = hit_left / HIT_TIME
            stip = "gray75" if frac > 0.66 else ("gray50" if frac > 0.33 else "gray25")
            c.create_rectangle(SX0, SY0, SX1, SY1, fill="#ff0000", stipple=stip, outline="")

        # 내가 그린 테두리 이미지는 화면 위에 덮고, 피격 중에는 구멍 가장자리에 빨간 선을 덧그림
        if frame is not None:
            c.create_image(0, 0, image=frame, anchor="nw")
            if red:
                c.create_rectangle(SX0, SY0, SX1, SY1, outline=C["danger"], width=HIT_BORDER_W)

        if self.maze_mode and self.cleared:
            c.create_text(CX, CY - 10, text="ESCAPED!", fill="#5dff9a", font=("Consolas", 40, "bold"))
            c.create_text(CX, CY + 36, text="R: 다시 도전   4: 전투 모드로", fill=C["text"], font=FONT_SMALL)
        elif self.maze_mode and self.timeup:
            c.create_text(CX, CY - 10, text="TIME UP", fill="#ff4040", font=("Consolas", 40, "bold"))
            c.create_text(CX, CY + 36, text="R: 다시 도전   4: 전투 모드로", fill=C["text"], font=FONT_SMALL)
        if not s.alive:
            c.create_text(CX, CY - 10, text="YOU DIED", fill="#ff4040", font=("Consolas", 40, "bold"))
            c.create_text(CX, CY + 36, text="R 키를 누르면 다시 시작", fill=C["text"], font=FONT_SMALL)
        if self.last_hit and now < self.last_hit_until and s.alive:
            c.create_text(CX, SY0 + 40, text=f"HIT!  {self.last_hit}", fill="#ff4040", font=("Consolas", 26, "bold"))
        if now < self.notice_until:
            c.create_text(CX, SY1 - 44, text=self.notice, fill=C["warn"], font=FONT_LABEL)

    def draw_sim_world(self, hit_left):
        """연습용 가짜 화면 (로봇 영상이 없을 때): 레이캐스팅 복도 + 미니맵"""
        c = self.c
        shake = random.randint(-4, 4) if hit_left > HIT_TIME - 0.3 else 0   # 맞은 직후 화면 흔들림
        sim = self.sim
        c.create_rectangle(SX0, SY0, SX1, SY1, fill="#0a1410", outline="")          # 모니터 바탕
        c.create_rectangle(SX0, SY0, SX1, (SY0 + SY1) / 2, fill="#14141c", outline="")   # 천장
        mid = (SY0 + SY1) / 2
        for i, col in enumerate(("#1d1812", "#221c15", "#28211a", "#2e261d")):      # 바닥 (멀수록 어둡게)
            c.create_rectangle(SX0, mid + i * (SY1 - mid) / 4, SX1, mid + (i + 1) * (SY1 - mid) / 4, fill=col, outline="")

        # 벽: 화면을 세로 띠로 잘라 한 줄씩 시선을 쏴서 그림 (거리에 따라 높이/밝기, 벽돌 줄눈)
        hr = math.radians(sim.heading)
        dx, dy = math.sin(hr), -math.cos(hr)                 # 바라보는 방향
        pxv, pyv = -dy * FOV_PLANE, dx * FOV_PLANE           # 시야 폭 방향(오른쪽)
        ncol = int((SX1 - SX0) // COLUMN_PX)
        cw = (SX1 - SX0) / ncol
        half = (SY1 - SY0) * 0.5
        for i in range(ncol):
            cam = 2 * (i + 0.5) / ncol - 1
            dist, side, ch, wx = raycast(sim.x, sim.y, dx + pxv * cam, dy + pyv * cam, sim.grid)
            hgt = min(half, half / dist)                      # 벽 반 높이 (모니터 높이를 넘지 않게)
            f = max(0.18, 1.0 / (1.0 + dist * 0.28)) * (0.72 if side else 1.0)
            if 0.0 < (wx * 3) % 1 < 0.07:                     # 벽돌 세로 줄눈
                f *= 0.6
            x0 = max(SX0, SX0 + i * cw + shake)               # 흔들려도 모니터 밖으로는 안 나가게 자름
            x1 = min(SX1, SX0 + i * cw + shake + cw + 0.6)
            if x1 <= x0:
                continue
            c.create_rectangle(x0, mid - hgt, x1, mid + hgt, fill=_shade(WALL_RGB.get(ch, (150, 150, 150)), f), outline="")

        # 미니맵 (오른쪽 위): 내 위치와 방향
        grid = sim.grid
        cs, mx0, my0 = 7, SX1 - 8 - 7 * len(grid[0]), SY0 + 8
        c.create_rectangle(mx0 - 3, my0 - 3, mx0 + cs * len(grid[0]) + 3, my0 + cs * len(grid) + 3, fill="#05080a", outline="#2c7a4f")
        for r, row in enumerate(grid):                       # 같은 벽끼리 가로로 묶어서 그림 (그리는 개수를 줄여 빠르게)
            q = 0
            while q < len(row):
                if row[q] == ".":
                    q += 1
                    continue
                q2 = q
                while q2 + 1 < len(row) and row[q2 + 1] == row[q]:
                    q2 += 1
                c.create_rectangle(mx0 + q * cs, my0 + r * cs, mx0 + (q2 + 1) * cs, my0 + r * cs + cs,
                                   fill=_shade(WALL_RGB.get(row[q], (150, 150, 150)), 0.55), outline="")
                q = q2 + 1
        if self.maze_mode:                                   # 출발(파랑) / 도착(초록) 칸 표시
            sx, sy = int(MAZE_START[0]), int(MAZE_START[1])
            gx, gy = MAZE_GOAL
            c.create_rectangle(mx0 + sx * cs, my0 + sy * cs, mx0 + sx * cs + cs, my0 + sy * cs + cs, fill="#4d8dff", outline="")
            c.create_rectangle(mx0 + gx * cs, my0 + gy * cs, mx0 + gx * cs + cs, my0 + gy * cs + cs, fill="#3cd270", outline="")
        ppx, ppy = mx0 + sim.x * cs, my0 + sim.y * cs
        c.create_line(ppx, ppy, ppx + dx * 9, ppy + dy * 9, fill="#ffd34d", width=2)
        c.create_oval(ppx - 3, ppy - 3, ppx + 3, ppy + 3, fill="#ffd34d", outline="")

    def draw_feed(self, now, hit_left):
        """모니터 화면: 로봇 카메라 영상. 영상이 안 오면 연습용 가짜 화면"""
        c = self.c
        sim = self.sim
        photo = self.camera_photo()
        self._cam_live = photo is not None
        if self._cam_live:
            self.cv.create_image(*c._pts((SX0, SY0)), image=photo, anchor="nw", tags="dyn")
        else:
            self.draw_sim_world(hit_left)
            c.create_text(CX, SY1 - 20, text="NO SIGNAL - 로봇 영상 없음 (연습 화면)",
                          fill=C["warn"], font=FONT_SMALL)

        # 조준점 (레이저 좌우 각도에 따라 좌우로 움직임)
        ax = self.aim_x()
        if not self.maze_mode:                               # 미로 모드에서는 조준점/레이저 없음
            c.create_line(ax - 18, AIM_Y, ax - 6, AIM_Y, fill=C["text"], width=2)
            c.create_line(ax + 6, AIM_Y, ax + 18, AIM_Y, fill=C["text"], width=2)
            c.create_line(ax, AIM_Y - 18, ax, AIM_Y - 6, fill=C["text"], width=2)
            c.create_line(ax, AIM_Y + 6, ax, AIM_Y + 18, fill=C["text"], width=2)

        # 화면 문구: 지금 이동 명령 / 레이저 각도 (체력 0이면 LOCKED)
        locked = (not self.state.alive) or self.maze_locked()
        ok_color = "#7fd4a0"
        pan = int(round(self.sim.pan))
        c.create_text(SX0 + 12, SY0 + 10, text=("MAZE" if self.maze_mode else "BATTLE") + ("  CAM" if self._cam_live else "  SIM"), anchor="nw", fill=ok_color, font=FONT_LABEL)
        c.create_text(SX0 + 12, SY0 + 28,
                      text="MOVE   " + ("LOCKED" if locked else f"{self.sim.cmd}  {self.sim.motor_text}"),
                      anchor="nw", fill=C["danger"] if locked else ok_color, font=FONT_LABEL)
        if not self._cam_live:                               # 방향은 가짜 화면에서만 의미 있음
            compass = "NESW"[int(((sim.heading + 45) % 360) // 90)]
            c.create_text(SX0 + 12, SY0 + 60, text=f"HEAD   {compass} {int(sim.heading):03d}°", anchor="nw", fill=ok_color, font=FONT_LABEL)
        info = self.robot_info
        if self.link.connected and info:
            line = f"ROBOT  {info.get('id', '?')}  FIRE {info.get('own', 0):.0f}Hz / SENSE {info.get('enemy', 0):.0f}Hz"
        else:
            line = f"LINK   {ROBOT_IP} 연결 중..."
        c.create_text(SX0 + 12, SY0 + 76, text=line, anchor="nw",
                      fill=ok_color if self.link.connected else C["warn"], font=FONT_LABEL)
        if not self.maze_mode:
            c.create_text(SX0 + 12, SY0 + 44, text="LASER  " + ("0" if pan == 0 else f"{pan:+d}") + "°"
                          + ("  LOCKED" if locked else ""),
                          anchor="nw", fill=C["danger"] if locked else ok_color, font=FONT_LABEL)

    # ---------- 상태바 ----------
    def _cell(self, name, x, fallback_fill, fallback_outline=""):
        """체력 칸 하나: 이미지가 있으면 이미지, 없으면 사각형"""
        img = self.imgs.get(name)
        if img is not None:
            self.c.create_image(x, HP_Y, image=img, anchor="nw")
        else:
            self.c.create_rectangle(x, HP_Y, x + HP_W, HP_Y + HP_H,
                                    fill=fallback_fill, outline=fallback_outline)

    def draw_status(self, now):
        c, s = self.c, self.state
        bar = self.imgs.get("bar")
        top = BAR_Y + 10

        # 전체화면에서 bar.png 가 있으면: 양옆 가장자리를 늘려 화면 가로 전체를 채운 이미지 (없으면 None)
        wide = self._wide_bar(bar) if (bar is not None and not c.identity) else None
        x0, x1 = c.x_span(self.sw)                            # 화면 가로 전체에 해당하는 좌표 범위 (창 모드: 0 ~ 720)

        if wide is not None:
            self.cv.create_image(0, round(c.sy(BAR_Y)), image=wide, anchor="nw", tags="dyn")
        elif bar is not None:
            if not c.identity:                                # 이미지를 못 늘릴 때는 띠만 가로 전체에 깔고 이미지는 가운데에
                c.create_rectangle(x0, BAR_Y, x1, H + 2, fill=C["bar"], outline="")
            c.create_image(0, BAR_Y, image=bar, anchor="nw")
        else:
            c.create_rectangle(x0, BAR_Y, x1, H + 2, fill=C["bar"], outline="")
            c.create_line(x0, BAR_Y, x1, BAR_Y, fill="#4a4f57", width=2)
            for x0, x1 in ((14, 290), (300, 420), (430, 706)):
                c.create_rectangle(x0, top, x1, BAR_Y + 138, fill=C["panel"], outline=C["panel_line"])
            c.create_text(24, top + 8, text="HP", anchor="nw", fill=C["muted"], font=FONT_LABEL)
            c.create_text(440, top + 8, text="LA", anchor="nw", fill=C["muted"], font=FONT_LABEL)

        if not self.maze_mode:
            # --- 체력 ---
            low = 0 < s.hp <= 3
            for i in range(MAX_HP):
                x = HP_X + i * HP_STEP
                if i < s.hp:
                    blink = low and int(now * 4) % 2
                    name = "hp_low" if (blink and "hp_low" in self.imgs) else "hp_on"
                    self._cell(name, x, C["hp_blink"] if blink else C["hp"])
                elif i == s.lost_idx and now < s.lost_until:
                    self._cell("hp_flash", x, "#ffffff")
                else:
                    self._cell("hp_off", x, C["hp_empty"], C["hp_empty_line"])
            hp_color = C["danger"] if s.hp <= 3 else C["text"]
            c.create_text(24, top + 78, text=f"{s.hp}/{MAX_HP}", anchor="nw", fill=hp_color, font=FONT_NUM)

        # --- 얼굴 ---
        self.draw_face(FACE_CX, FACE_CY, now)

        if not self.maze_mode:
            # --- 레이저 ---
            denied = now < s.deny_until
            if bar is None:
                c.create_rectangle(LASER_X0, LASER_Y0, LASER_X1, LASER_Y1, fill="#101215",
                                   outline=C["danger"] if denied else C["panel_line"], width=2 if denied else 1)
            elif denied:                                          # 쿨타임 중 발사 시도: 막대 칸에 빨간 테두리
                c.create_rectangle(LASER_X0, LASER_Y0, LASER_X1, LASER_Y1, outline=C["danger"], width=2)
            frac = s.cooldown_frac() if s.alive else 0.0
            if frac > 0:
                ready = s.cooldown_left() == 0
                c.create_rectangle(LASER_X0 + 1, LASER_Y0 + 1,
                                   LASER_X0 + 1 + (LASER_X1 - LASER_X0 - 2) * frac, LASER_Y1 - 1,
                                   fill=C["ok"] if ready else C["warn"], outline="")
            if not s.alive:
                label, color = "OFFLINE", C["muted"]
            elif s.cooldown_left() == 0:
                label, color = "READY", C["ok"]
            else:
                label, color = f"{s.cooldown_left():.1f}s", C["warn"]
            c.create_text(440, top + 78, text=label, anchor="nw", fill=color, font=FONT_NUM)

        # --- 미로 모드: 왼쪽 칸(체력)을 100초 타이머로, 오른쪽 칸(레이저)은 비움 ---
        if self.maze_mode:
            for (xa, xb) in ((14, 290), (430, 706)):
                c.create_rectangle(xa, top, xb, BAR_Y + 138, fill=C["panel"], outline=C["panel_line"])
            left = self.maze_left(now)
            m, sec = int(left // 60), int(left % 60)
            if self.cleared:
                txt, col = f"{m}:{sec:02d}", C["ok"]
            elif self.timeup:
                txt, col = "TIME UP", C["danger"]
            else:
                txt, col = f"{m}:{sec:02d}", (C["danger"] if left <= 60 else C["text"])
            c.create_text(24, top + 8, text="CLEAR" if self.cleared else "TIME", anchor="nw", fill=C["muted"], font=FONT_LABEL)
            c.create_text(152, top + 70, text=txt, anchor="center", fill=col, font=("Consolas", 40, "bold"))

    def face_name(self):
        s = self.state
        if not s.alive:
            return "face_dead"
        if s.hit_left() > HIT_TIME - 0.6:
            return "face_ouch"
        if s.hp > 7:
            return "face_happy"
        return "face_normal" if s.hp > 3 else "face_hurt"

    def draw_face(self, fx, fy, now):
        """둠처럼 체력에 따라 표정이 바뀌는 얼굴"""
        c, s = self.c, self.state
        img = self.imgs.get(self.face_name())
        if img is not None:                                   # 내가 그린 얼굴 이미지
            c.create_image(fx, fy, image=img, anchor="center")
            return
        r = 40
        skin = "#e0b48a" if s.alive else "#8a8a8a"
        c.create_oval(fx - r, fy - r, fx + r, fy + r, fill=skin, outline="#3a2a1a", width=2)

        ouch = s.alive and s.hit_left() > HIT_TIME - 0.6      # 맞은 직후
        if not s.alive:                                       # X 눈
            for ex in (fx - 15, fx + 15):
                c.create_line(ex - 6, fy - 12, ex + 6, fy, fill="#222222", width=3)
                c.create_line(ex - 6, fy, ex + 6, fy - 12, fill="#222222", width=3)
            c.create_line(fx - 14, fy + 20, fx + 14, fy + 20, fill="#222222", width=3)
            return

        if ouch:                                              # 찡그린 눈
            for ex in (fx - 15, fx + 15):
                c.create_line(ex - 6, fy - 12, ex + 6, fy - 6, fill="#222222", width=3)
            c.create_oval(fx - 8, fy + 12, fx + 8, fy + 30, fill="#5a1a1a", outline="#222222", width=2)
        else:
            for ex in (fx - 15, fx + 15):
                c.create_oval(ex - 5, fy - 14, ex + 5, fy - 2, fill="#222222", outline="")
            if s.hp > 7:                                      # 웃는 입
                c.create_arc(fx - 16, fy + 2, fx + 16, fy + 28, start=200, extent=140,
                             style="arc", outline="#222222", width=3)
            elif s.hp > 3:                                    # 일자 입
                c.create_line(fx - 12, fy + 20, fx + 12, fy + 20, fill="#222222", width=3)
            else:                                             # 찡그린 입
                c.create_arc(fx - 16, fy + 16, fx + 16, fy + 42, start=20, extent=140,
                             style="arc", outline="#222222", width=3)

        if s.hp <= 5:                                         # 상처 (체력 낮을수록 늘어남)
            c.create_line(fx - 24, fy - 28, fx - 10, fy - 14, fill="#a01818", width=3)
        if s.hp <= 3:
            c.create_line(fx + 26, fy - 22, fx + 12, fy - 6, fill="#a01818", width=3)
            c.create_oval(fx + 18, fy + 14, fx + 26, fy + 22, fill="#a01818", outline="")

    # ---------- 조작법 ----------
    def draw_help(self):
        if not SHOW_HELP:
            return
        self.c.create_text(W // 2, H - 14, fill=C["muted"], font=FONT_SMALL,
                           text="[1] 발사  [방향키] 이동  [Q/W] 조준  [4] 미로  [5] 도착  [6] 보정  [3] 전체화면  [R] 리셋  [Esc] 종료")


def run():
    global ROBOT_IP
    if len(sys.argv) > 1:
        ROBOT_IP = sys.argv[1]
    print(f"[link] 로봇 {ROBOT_IP} 에 연결 시도 (조종 {CTRL_PORT} / 영상 {VIDEO_PORT})")
    root = tk.Tk()
    HUD(root)
    root.mainloop()


if __name__ == "__main__":
    run()
