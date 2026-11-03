#!/usr/bin/env python3
"""거미로봇 케이스 (박스 모형 그대로): 한 덩어리 사각 통 + 서보 받침
좌표: X = 로봇 오른쪽(+), Y = 앞(+), Z = 위. 단위 mm. 숫자 바꾸고 다시 실행하면 새 STL.

사용법 (PC PowerShell, 이 파일이 있는 폴더에서)
    py -m pip install numpy          <- 처음 한 번만
    py make_sleeve.py case_out       <- case_out 폴더에 부품별 STL/3MF + 통합 파일(spider_case_all) 생성
"""
import os
import struct
import sys

import numpy as np

OUT = sys.argv[1] if len(sys.argv) > 1 else "."

T = 1.6                 # 벽 두께 (노즐 0.4 x 4줄)
BT = 2.5                # 서보 받침 두께 (힘 받는 곳이라 두껍게)
W, D, H = 90.0, 145.0, 116.0   # 바깥 가로(뒷판·앞판) / 깊이(옆판) / 높이
BAND = (25.0, 70.0)     # 패널 칸 높이 (바닥에서 2.5~7.0cm)
SPAN = 45.0             # 박스에서 잡은 패널 칸
POCKET, POCKET_D = 44.6, 0.8   # 바깥에서 파는 패널 홈 (패널 44 + 여유), 깊이
HOLE = 30.0             # 관통 구멍 (패널 뒤 선·납땜 지나가는 곳). 패널은 바깥 홈에 붙으니 작아도 됨 -> 판이 튼튼
WIRE_SLOT = 5.0         # 패널 선을 옆으로 밀어 넣는 홈 폭 (구멍 아래 ~ 판 아래끝)
CORNER = 1.4            # 모서리에서 두 패널이 만나면 서로 안 부딪히게 띄우는 거리
LASER_Z = sum(BAND) / 2 # 레이저 중심 높이 = 패널 칸 가운데 (47.5)

# SG90 (거꾸로 매담: 축이 아래)
S_L, S_W = 22.8, 12.2   # 몸통
S_TAB_FROM_BOTTOM = 15.9
S_TOP_ABOVE_TAB = 6.8   # 날개 아래로 나오는 기어 부분
HORN_GAP = 3.5          # 기어 끝 ~ 혼 아래면
LASER_D = 10.0
AXIS_X = 67.5           # 서보 축 X (로봇 오른쪽 칸 45~90 가운데)
CAM_W, CAM_H = 26.0, 22.0   # 카메라 구멍 가로, 높이 (바닥부터)
USBC_X, USBC_Z = 25.0, 82.0   # C타입 선 구멍 중심 (로봇 왼쪽 끝에서 / 바닥에서). 카메라(가운데 45)에서 2cm 왼쪽
USBC_W, USBC_H = 16.0, 20.0   # 구멍 크기 (ㄱ자 C타입 플러그가 지나가게)
S_TAB = 4.7             # 서보 날개가 몸통 밖으로 나온 길이
AXIS_GAP = 5.5          # 앞판 바깥면 ~ 서보 몸통 사이 (뒤쪽 날개가 들어갈 자리)


class Grid:
    def __init__(self):
        self.ops = []

    def add(self, *b):
        self.ops.append((1, b))

    def sub(self, *b):
        self.ops.append((0, b))

    def mesh(self):
        cs = [sorted({round(v, 4) for _, b in self.ops for v in (b[k], b[k + 3])}) for k in range(3)]
        cs = [np.array(c) for c in cs]
        occ = np.zeros([len(c) - 1 for c in cs], dtype=bool)
        for val, b in self.ops:
            sl = tuple(slice(np.searchsorted(cs[k], round(b[k], 4)), np.searchsorted(cs[k], round(b[k + 3], 4)))
                       for k in range(3))
            occ[sl] = bool(val)
        tris = []
        pad = np.pad(occ, 1)
        for ax in range(3):
            diff = pad[1:, :, :] != pad[:-1, :, :] if ax == 0 else (
                pad[:, 1:, :] != pad[:, :-1, :] if ax == 1 else pad[:, :, 1:] != pad[:, :, :-1])
            idx = np.argwhere(diff)
            for p in idx:
                i, j, k = p
                lo = (i, j, k)
                # 경계면 위치 (ax 방향 경계 index), 나머지는 셀 index (pad 보정 -1)
                c = [lo[0] - 1, lo[1] - 1, lo[2] - 1]
                c[ax] += 1                       # 경계 좌표 index
                inside_first = (pad[i, j, k] if True else None)
                # pad 기준으로 앞쪽 셀이 채워져 있으면 면은 +ax 방향을 봄
                a_cell = pad[i, j, k]
                o = [cs[m][c[m]] if m == ax else None for m in range(3)]
                u, v = [m for m in range(3) if m != ax]
                u0, u1 = cs[u][c[u]], cs[u][c[u] + 1]
                v0, v1 = cs[v][c[v]], cs[v][c[v] + 1]

                def P(uu, vv):
                    q = [0.0, 0.0, 0.0]
                    q[ax] = cs[ax][c[ax]]
                    q[u], q[v] = uu, vv
                    return tuple(float(x) for x in q)
                quad = [P(u0, v0), P(u1, v0), P(u1, v1), P(u0, v1)]
                # (u, v, ax) 가 오른손 좌표면 u0v0->u1v0->u1v1 은 +ax 방향
                right = (u, v) in ((1, 2), (2, 0), (0, 1))
                facing_plus = bool(a_cell)
                if facing_plus != right:
                    quad = quad[::-1]
                tris.append((quad[0], quad[1], quad[2]))
                tris.append((quad[0], quad[2], quad[3]))
        return tris


def check(tris):
    e = {}
    for t in tris:
        for a, b in ((t[0], t[1]), (t[1], t[2]), (t[2], t[0])):
            e[(a, b)] = e.get((a, b), 0) + 1
    bad = sum(1 for (a, b), n in e.items() if n != 1 or e.get((b, a)) != 1)
    vol = sum((a[0] * (b[1] * c[2] - b[2] * c[1]) - a[1] * (b[0] * c[2] - b[2] * c[0])
               + a[2] * (b[0] * c[1] - b[1] * c[0])) / 6 for a, b, c in tris)
    return bad, vol


def write_stl(path, tris, name):
    with open(path, "wb") as f:
        f.write(name.encode()[:80].ljust(80, b" "))
        f.write(struct.pack("<I", len(tris)))
        for a, b, c in tris:
            n = np.cross(np.subtract(b, a), np.subtract(c, a))
            n = n / (np.linalg.norm(n) or 1)
            f.write(struct.pack("<12fH", *n, *a, *b, *c, 0))


def panel_span(a, b, corner_lo=False, corner_hi=False):
    """박스의 칸 [a,b] 안에 홈/구멍 위치. 모서리에서 다른 패널과 만나면 그쪽으로 CORNER만큼 띄움"""
    if corner_lo:
        p0 = a + CORNER
    elif corner_hi:
        p0 = b - CORNER - POCKET
    else:
        p0 = (a + b) / 2 - POCKET / 2
    h0 = p0 + (POCKET - HOLE) / 2
    return (p0, p0 + POCKET), (h0, h0 + HOLE)


# ---- 슬라이드 결합 치수 ----
CLR = 0.3               # 끼우는 곳 여유
BP = 1.6                # 뒷판 두께 (통 뒤쪽 끝을 덮음)
SIDE_Y0 = BP + CLR      # 옆판이 시작하는 Y (뒷판 바로 앞)
RAIL_X = 4.0            # 옆판 안쪽에 붙는 레일이 안쪽으로 나온 길이
RAIL_T = 1.2            # 레일 두께 (Y 방향)
HOOK_T = 1.6            # 뒷판 갈고리 두께
HOOK_STEM_X = (6.0, 7.6)        # 갈고리 기둥 (옆판 안쪽면 기준 아님, 바깥면 기준 X)
TONGUE_Z = ((0.0, 24.0), (71.0, H - 6.0))   # 갈고리 구간 (패널 칸 피해서, 뚜껑 홈 아래까지)
LID_T = 1.6
LEDGE_Z = (H - 5.2, H - 3.6)    # 뚜껑 받침턱
LID_Z = (H - 3.4, H - 1.8)      # 뚜껑
LIP_Z = (H - 1.6, H)            # 뚜껑 누름턱
LIP_X = 3.0                      # 턱이 안쪽으로 나온 길이
SIDE_PANEL_BACK = 2.2            # 옆판 뒤쪽 패널 홈 시작 Y (GPIO 홈과의 사이를 넓힘)
HOOK_PATH_Y = SIDE_Y0 + RAIL_T + CLR + HOOK_T + CLR   # 갈고리가 위아래로 지나가는 길 끝 (턱은 여기부터)


def zbands():
    zp = (BAND[0] + (SPAN - POCKET) / 2, BAND[1] - (SPAN - POCKET) / 2)
    zh = (BAND[0] + (SPAN - HOLE) / 2, BAND[1] - (SPAN - HOLE) / 2)
    return zp, zh


def span_from(p0):
    h0 = p0 + (POCKET - HOLE) / 2
    return (p0, p0 + POCKET), (h0, h0 + HOLE)


def sleeve():
    """통: 왼판 + 오른판 + 앞판. 뒤는 뚫려 있고 뒷판이 위에서 미끄러져 들어감. 위는 뚜껑이 뒤에서 밀려 들어감"""
    g = Grid()
    g.add(0, SIDE_Y0, 0, W, D, H)
    g.sub(T, SIDE_Y0 - 1, -1, W - T, D - T, H + 1)
    zp, zh = zbands()

    def panel_y(face, span):
        (p0, p1), (h0, h1) = span
        c = (h0 + h1) / 2
        if face == "left":
            g.sub(-1, p0, zp[0], POCKET_D, p1, zp[1])
            g.sub(-1, h0, zh[0], T + 1, h1, zh[1])
            g.sub(-1, c - WIRE_SLOT / 2, -1, T + 1, c + WIRE_SLOT / 2, zh[0] + 1)
        else:
            g.sub(W - POCKET_D, p0, zp[0], W + 1, p1, zp[1])
            g.sub(W - T - 1, h0, zh[0], W + 1, h1, zh[1])
            g.sub(W - T - 1, c - WIRE_SLOT / 2, -1, W + 1, c + WIRE_SLOT / 2, zh[0] + 1)

    # 왼판: 앞에서부터 여백2.7 / 패널4.5 / 여백0.5 / GPIO1.8 / 여백0.5 / 패널4.5
    panel_y("left", panel_span(D - 72, D - 27))
    g.sub(-1, D - 95, -1, T + 1, D - 77, 70)
    panel_y("left", span_from(SIDE_PANEL_BACK))
    # 오른판: 뒤에서부터 패널 / 여백 / USB / 전원 / 패널
    panel_y("right", span_from(SIDE_PANEL_BACK))
    g.sub(W - T - 1, 65, 60, W + 1, 85, 70)
    g.sub(W - T - 1, 85, 25, W + 1, 100, 45)
    panel_y("right", panel_span(100, 145))
    # 앞판
    (p0, p1), (h0, h1) = panel_span(0, 45)
    g.sub(p0, D - POCKET_D, zp[0], p1, D + 1, zp[1])
    g.sub(h0, D - T - 1, zh[0], h1, D + 1, zh[1])
    g.sub((h0 + h1) / 2 - WIRE_SLOT / 2, D - T - 1, -1, (h0 + h1) / 2 + WIRE_SLOT / 2, D + 1, zh[0] + 1)
    g.sub(W / 2 - CAM_W / 2, D - T - 1, -1, W / 2 + CAM_W / 2, D + 1, CAM_H)   # 카메라 구멍 (앞판 가운데 아래)
    g.sub(USBC_X - USBC_W / 2, D - T - 1, USBC_Z - USBC_H / 2, USBC_X + USBC_W / 2, D + 1, USBC_Z + USBC_H / 2)   # C타입 선
    # 선 구멍 (로봇 오른쪽 칸, 서보 받침 위아래)
    st = LASER_Z + LASER_D / 2 + HORN_GAP + S_TOP_ABOVE_TAB          # 서보 받침 윗면 높이
    g.sub(AXIS_X - 6, D - T - 1, st + 2, AXIS_X + 6, D + 1, st + 14)   # 서보 선 (받침 위, 서보 뒤끝 바로 뒤)
    g.sub(AXIS_X - 8, D - T - 1, LASER_Z - 8, AXIS_X + 8, D + 1, st - BT - 8 - 1.5)   # 레이저 선 (받침 아래)
    # 뒷판 레일 (양쪽 옆판 안쪽, 뒤 끝)
    for xa, xb in ((T, T + RAIL_X), (W - T - RAIL_X, W - T)):
        g.add(xa, SIDE_Y0, 0, xb, SIDE_Y0 + RAIL_T, LEDGE_Z[0] - 0.4)
    # 뚜껑 홈: 받침턱 + 누름턱 (양쪽 옆판 안쪽, 뒤 끝부터 앞판까지)
    for xa, xb in ((T, T + LIP_X), (W - T - LIP_X, W - T)):
        g.add(xa, HOOK_PATH_Y, LEDGE_Z[0], xb, D - T, LEDGE_Z[1])
        g.add(xa, HOOK_PATH_Y, LIP_Z[0], xb, D - T, LIP_Z[1])
    # 앞판 안쪽에도 받침턱 (뚜껑 앞끝 받침)
    g.add(T, D - T - LIP_X, LEDGE_Z[0], W - T, D - T, LEDGE_Z[1])
    return g


def back_plate():
    """뒷판: 위에서 아래로 밀어 넣음. 안쪽 갈고리가 옆판 레일 뒤에 걸려서 뒤로 안 빠짐"""
    g = Grid()
    g.add(0, 0, 0, W, BP, H)
    zp, zh = zbands()
    for a, b in ((0, 45), (45, 90)):
        (p0, p1), (h0, h1) = panel_span(a, b)
        g.sub(p0, -1, zp[0], p1, POCKET_D, zp[1])
        g.sub(h0, -1, zh[0], h1, BP + 1, zh[1])
        g.sub((h0 + h1) / 2 - WIRE_SLOT / 2, -1, -1, (h0 + h1) / 2 + WIRE_SLOT / 2, BP + 1, zh[0] + 1)
    hook_y = (SIDE_Y0 + RAIL_T + CLR, SIDE_Y0 + RAIL_T + CLR + HOOK_T)
    for z0, z1 in TONGUE_Z:
        for side in (0, 1):
            sx0, sx1 = HOOK_STEM_X if side == 0 else (W - HOOK_STEM_X[1], W - HOOK_STEM_X[0])
            g.add(sx0, BP, z0, sx1, hook_y[1], z1)                     # 기둥
            if side == 0:
                g.add(T + CLR, hook_y[0], z0, sx1, hook_y[1], z1)      # 바깥쪽으로 꺾인 갈고리
            else:
                g.add(sx0, hook_y[0], z0, W - T - CLR, hook_y[1], z1)
    g.sub(W / 2 - 8, -1, 88, W / 2 + 8, BP + 1, 95)                    # 손가락 걸이 (밀어 올릴 때)
    return g


def lid():
    g = Grid()
    x0, x1 = T + CLR, W - T - CLR
    y0, y1 = BP + 0.4, D - T - CLR
    g.add(x0, y0, LID_Z[0], x1, y1, LID_Z[1])
    g.sub(W / 2 - 8, y0 + 4, LID_Z[0] - 1, W / 2 + 8, y0 + 10, LID_Z[1] + 1)   # 손가락 걸이
    nx = HOOK_STEM_X[1] + CLR                                           # 뒷판 갈고리가 지나갈 귀퉁이 홈
    g.sub(x0 - 1, y0 - 1, LID_Z[0] - 1, nx, HOOK_PATH_Y, LID_Z[1] + 1)
    g.sub(W - nx, y0 - 1, LID_Z[0] - 1, x1 + 1, HOOK_PATH_Y, LID_Z[1] + 1)
    return g


def servo_bracket():
    """앞판 바깥면(Y=D)에 붙이는 L자 받침. 서보 긴 쪽이 앞뒤 방향, 축은 앞쪽 끝.
    서보를 위에서 거꾸로(축이 아래) 끼우면 앞뒤 날개가 선반에 걸림"""
    shelf_top = LASER_Z + LASER_D / 2 + HORN_GAP + S_TOP_ABOVE_TAB
    by0 = D + AXIS_GAP                 # 몸통 뒤끝 (앞판 쪽)
    by1 = by0 + S_L                    # 몸통 앞끝
    ay = by1 - 5.9                     # 축 (앞쪽 끝에서 5.9)
    hx0, hx1 = AXIS_X - S_W / 2 - 0.2, AXIS_X + S_W / 2 + 0.2
    hy0, hy1 = by0 - 0.2, by1 + 0.2
    x0, x1 = hx0 - 3, hx1 + 3
    y1 = hy1 + S_TAB + 2               # 앞쪽 날개까지 받치게
    g = Grid()
    g.add(x0, D, shelf_top - BT - 8, x1, D + BT, shelf_top)       # 앞판에 붙는 면
    g.add(x0, D, shelf_top - BT, x1, y1, shelf_top)              # 선반
    g.sub(hx0, hy0, shelf_top - BT - 1, hx1, hy1, shelf_top + 1)  # 서보 구멍
    RIM = 3.0                                                     # 벽 양옆 테두리
    g.sub(x0 + RIM, D - 1, shelf_top - BT - 8 - 1, x1 - RIM, D + BT + 1, shelf_top - BT)   # 벽 가운데를 아래끝까지 틈 (양옆 테두리만 남김)
    return g, shelf_top, ay, (x0, x1)


def write_3mf(path, tris, name):
    import zipfile
    idx, verts, faces = {}, [], []
    for t in tris:
        f = []
        for p in t:
            k = (round(p[0], 4), round(p[1], 4), round(p[2], 4))
            if k not in idx:
                idx[k] = len(verts)
                verts.append(k)
            f.append(idx[k])
        faces.append(f)
    vx = "".join(f'<vertex x="{x:.4f}" y="{y:.4f}" z="{z:.4f}"/>' for x, y, z in verts)
    tr = "".join(f'<triangle v1="{a}" v2="{b}" v3="{c}"/>' for a, b, c in faces)
    model = ('<?xml version="1.0" encoding="UTF-8"?>'
             '<model unit="millimeter" xml:lang="en-US" xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02">'
             f'<resources><object id="1" type="model" name="{name}"><mesh><vertices>{vx}</vertices>'
             f'<triangles>{tr}</triangles></mesh></object></resources><build><item objectid="1"/></build></model>')
    ct = ('<?xml version="1.0" encoding="UTF-8"?><Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">'
          '<Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/>'
          '<Default Extension="model" ContentType="application/vnd.ms-package.3dmanufacturing-3dmodel+xml"/></Types>')
    rels = ('<?xml version="1.0" encoding="UTF-8"?><Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">'
            '<Relationship Target="/3D/3dmodel.model" Id="rel0" Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel"/></Relationships>')
    with zipfile.ZipFile(path, "w", zipfile.ZIP_DEFLATED) as z:
        z.writestr("[Content_Types].xml", ct)
        z.writestr("_rels/.rels", rels)
        z.writestr("3D/3dmodel.model", model)


ALL = []


def save(g, name, label, rot=None):
    tris = g.mesh()
    bad, vol = check(tris)
    if rot:
        tris = [tuple(rot(p) for p in t) for t in tris]
    # 바닥(z=0)에 내려놓고 저장
    zmin = min(p[2] for t in tris for p in t)
    xmin = min(p[0] for t in tris for p in t)
    ymin = min(p[1] for t in tris for p in t)
    tris = [tuple((p[0] - xmin, p[1] - ymin, p[2] - zmin) for p in t) for t in tris]
    write_stl(os.path.join(OUT, name), tris, label)
    write_3mf(os.path.join(OUT, name.replace(".stl", ".3mf")), tris, label)
    ALL.append((name.replace(".stl", ""), tris))
    print(f"{name:26s} 삼각형 {len(tris):5d} 열린모서리 {bad} 부피 {vol / 1000:.1f}cm3")
    return bad


def write_all(base):
    """부품 4개를 한 줄로(10mm 간격) 늘어놓아 통합 STL(한 덩어리)과 통합 3MF(부품별 물체)로 저장"""
    import zipfile
    x, placed = 0.0, []
    for name, tris in ALL:
        xs = [p[0] for t in tris for p in t]
        dx = x - min(xs)
        placed.append((name, [tuple((p[0] + dx, p[1], p[2]) for p in t) for t in tris]))
        x += max(xs) - min(xs) + 10
    write_stl(base + ".stl", [t for _, tt in placed for t in tt], "spider case all parts")
    res, items = [], []
    for i, (name, tris) in enumerate(placed, 1):
        idx, verts, faces = {}, [], []
        for t in tris:
            f = []
            for p in t:
                k = (round(p[0], 4), round(p[1], 4), round(p[2], 4))
                if k not in idx:
                    idx[k] = len(verts)
                    verts.append(k)
                f.append(idx[k])
            faces.append(f)
        vx = "".join(f'<vertex x="{a:.4f}" y="{b:.4f}" z="{c:.4f}"/>' for a, b, c in verts)
        tr = "".join(f'<triangle v1="{a}" v2="{b}" v3="{c}"/>' for a, b, c in faces)
        res.append(f'<object id="{i}" type="model" name="{name}"><mesh><vertices>{vx}</vertices><triangles>{tr}</triangles></mesh></object>')
        items.append(f'<item objectid="{i}"/>')
    model = ('<?xml version="1.0" encoding="UTF-8"?>'
             '<model unit="millimeter" xml:lang="en-US" xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02">'
             f'<resources>{"".join(res)}</resources><build>{"".join(items)}</build></model>')
    ct = ('<?xml version="1.0" encoding="UTF-8"?><Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">'
          '<Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/>'
          '<Default Extension="model" ContentType="application/vnd.ms-package.3dmanufacturing-3dmodel+xml"/></Types>')
    rels = ('<?xml version="1.0" encoding="UTF-8"?><Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">'
            '<Relationship Target="/3D/3dmodel.model" Id="rel0" Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel"/></Relationships>')
    with zipfile.ZipFile(base + ".3mf", "w", zipfile.ZIP_DEFLATED) as z:
        z.writestr("[Content_Types].xml", ct)
        z.writestr("_rels/.rels", rels)
        z.writestr("3D/3dmodel.model", model)
    print(f"통합 파일: {base}.stl, {base}.3mf (가로 {x - 10:.0f}mm)")


if __name__ == "__main__":
    os.makedirs(OUT, exist_ok=True)
    save(sleeve(), "case_1_body.stl", "spider case body")
    # 뒷판: 바깥면을 바닥에 (X축 기준 -90도: y->z) 갈고리가 위로
    save(back_plate(), "case_2_back.stl", "spider case back", rot=lambda p: (p[0], H - p[2], p[1]))
    save(lid(), "case_3_lid.stl", "spider case lid", rot=lambda p: (p[0], p[1], p[2] - LID_Z[0]))
    g, st, ay, (x0, x1) = servo_bracket()
    save(g, "case_4_servo_bracket.stl", "spider servo bracket", rot=lambda p: (p[0] - x0, D + 30 - p[1], st - p[2]))
    print(f"서보 받침 윗면 {st:.1f}mm, 축은 앞판 바깥면에서 {ay - D:.1f}mm")
    write_all(os.path.join(OUT, "spider_case_all"))

