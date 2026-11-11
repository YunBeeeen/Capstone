#!/usr/bin/env python3
"""school_floor 월드 생성기 — 학교 건물 한 층 (20 × 12 m), VSLAM / Nav2 시험용 (Gazebo Classic 11).

  python3 src/gazebo/tools/make_world.py          # capstone_ws 에서. 결과물은 커밋되어 있으니 배치/텍스처 바꿀 때만
  → worlds/school_floor.world
    models/capstone_materials/materials/{scripts/capstone.material, textures/*.png}
    docs/school_floor_plan.png  (평면도)

평면 (x 동, y 북, 원점 = 남서쪽 바깥 모서리):

    y=12 ┌──────────────────────── 북쪽 복도 (창문, 게시판) ─────────────────────────┐
         │   ┌──────────────┬─────────[연구실 문]─┬────────────────┐                 │
         │ 엘 │   행정실(301) │      연구실(302)     │    휴게실(303)  │  동쪽 복도      │
         │ 리 │  책상 4 + 서류 │  실험대, 선반        │  소파, 냉장고    │  (흰 벽 구간 —  │
         │ 베 │  캐비닛, 책장  │                     │  자판기(복도)    │   VSLAM 어려움) │
         │ 이 └──[행정실 문]──┴─────────────────────┴──[휴게실 문]──┘                 │
         │ 터                       남쪽 복도 (창문, 사물함, 포스터)                     │
    y=0  └──────────────────────────────────────────────────────────────────────────┘
        x=0                                                                        x=20
  복도 폭 2.5 m, 방 블록 x 2.5~17.5 / y 2.5~9.5, 문 폭 1.0 m. 복도가 블록을 한 바퀴 돌아서 루프 클로저 시험 가능.
  스폰 기본 (1.25, 1.25) = 남서쪽 모서리 복도.

VSLAM 메모: 텍스처는 전부 서로 다르게 생성 (같은 포스터가 두 곳에 있으면 잘못된 루프 클로저).
  벽은 2.5 m 이하 조각으로 나눠서 텍스처가 늘어지지 않게 (Gazebo 박스는 면마다 텍스처 한 장을 늘려 붙임).
  천장 없음 (조명 단순화). 카메라를 약간 아래로 보면 안 보임.
"""
import math
import random
import textwrap
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFilter, ImageFont

PKG = Path(__file__).resolve().parents[1]
TEX = PKG / "models" / "capstone_materials" / "materials" / "textures"
SCRIPT = PKG / "models" / "capstone_materials" / "materials" / "scripts" / "capstone.material"
WORLD = PKG / "worlds" / "school_floor.world"
PLAN = PKG / "docs" / "school_floor_plan.png"

W, D = 20.0, 12.0          # 바깥 크기 [m]
BX0, BX1, BY0, BY1 = 2.5, 17.5, 2.5, 9.5  # 방 블록
H, T = 2.6, 0.15           # 벽 높이, 두께
DOOR_H = 2.1
PI = math.pi

rng = random.Random(7)
nrng = np.random.default_rng(7)

# =============================================================== 텍스처
FONT_CJK = "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc"
FONT_LAT = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"


def font(size, korean=False):
    for path, idx in ([(FONT_CJK, 1)] if korean else []) + [(FONT_LAT, 0), (FONT_CJK, 1)]:
        try:
            return ImageFont.truetype(path, size, index=idx)
        except OSError:
            continue
    return ImageFont.load_default()


MATERIALS = []  # (이름, 파일)


def save(img, name):
    TEX.mkdir(parents=True, exist_ok=True)
    img.convert("RGB").save(TEX / f"{name}.png", optimize=True)
    MATERIALS.append(name)
    return f"Capstone/{name}"


def smooth_noise(h, w, scale, seed):
    """0~1 부드러운 잡음 (저해상도 난수 → 확대 + 블러)."""
    g = np.random.default_rng(seed).random((max(2, h // scale), max(2, w // scale)))
    im = Image.fromarray((g * 255).astype(np.uint8)).resize((w, h), Image.BICUBIC)
    return np.asarray(im.filter(ImageFilter.GaussianBlur(scale / 3)), dtype=np.float32) / 255.0


def tint(base, noise, amp):
    arr = np.array(base, dtype=np.float32)[None, None, :] * (1 + amp * (noise[..., None] - 0.5))
    return Image.fromarray(np.clip(arr, 0, 255).astype(np.uint8))


def tex_wall(name, base, seed, band=None):
    """칠한 벽 + (선택) 아래쪽 띠 (학교 복도 걸레받이/징두리)."""
    n = smooth_noise(512, 512, 24, seed) * 0.6 + smooth_noise(512, 512, 4, seed + 1) * 0.4
    img = tint(base, n, 0.10)
    if band:
        d = ImageDraw.Draw(img)
        y = int(512 * (1 - 1.0 / H))  # 바닥에서 1.0 m
        d.rectangle([0, y, 512, 512], fill=band)
        d.rectangle([0, y - 6, 512, y], fill=tuple(int(c * 0.7) for c in band))
        img = Image.blend(img, tint((128, 128, 128), smooth_noise(512, 512, 6, seed + 2), 0.2), 0.08)
    return save(img, name)


def tex_floor_tiles(name, base, seed, n=4):
    """2 m 바닥 조각 한 장에 n×n 타일 (줄눈 + 타일마다 밝기 차 + 점 무늬 + 얼룩·긁힘).
    조각마다 다른 시드 변형을 섞어 씀 — 같은 그림이 2 m 마다 반복되면 시각 odometry 가 2 m 전 위치와 헷갈림
    (2026-11-11 시험: 반복 바닥에서 복도 직진 중 VO lost 13 %)."""
    s = 512
    img = Image.new("RGB", (s, s), base)
    d = ImageDraw.Draw(img)
    r = random.Random(seed)
    step = s // n
    for i in range(n):
        for j in range(n):
            k = r.uniform(0.9, 1.08)
            d.rectangle([i * step, j * step, (i + 1) * step, (j + 1) * step], fill=tuple(int(c * k) for c in base))
    arr = np.asarray(img, dtype=np.float32)
    speck = np.random.default_rng(seed).random((s, s)) < 0.02
    arr[speck] *= 0.6
    img = Image.fromarray(np.clip(arr, 0, 255).astype(np.uint8))
    d = ImageDraw.Draw(img)
    for _ in range(r.randint(4, 9)):  # 얼룩
        x, y, rr = r.randint(0, s), r.randint(0, s), r.randint(8, 40)
        k = r.uniform(0.75, 0.92)
        d.ellipse([x - rr, y - rr * r.uniform(0.4, 1), x + rr, y + rr], fill=tuple(int(c * k) for c in base))
    for _ in range(r.randint(3, 8)):  # 긁힘 (바퀴·신발 자국)
        x, y = r.randint(0, s), r.randint(0, s)
        a, ln = r.uniform(0, 3.14), r.randint(20, 120)
        d.line([(x, y), (x + ln * math.cos(a), y + ln * math.sin(a))], fill=(110, 105, 100), width=r.randint(2, 4))
    for i in range(n + 1):
        d.line([(i * step, 0), (i * step, s)], fill=(90, 90, 90), width=4)
        d.line([(0, i * step), (s, i * step)], fill=(90, 90, 90), width=4)
    return save(img, name)


def tex_carpet(name, base, seed):
    n = smooth_noise(512, 512, 3, seed) * 0.7 + smooth_noise(512, 512, 40, seed + 1) * 0.3
    return save(tint(base, n, 0.35), name)


def tex_poster(name, seed, title, korean=False):
    r = random.Random(seed)
    w, h = 360, 512
    bg = tuple(r.randint(30, 230) for _ in range(3))
    img = Image.new("RGB", (w, h), bg)
    d = ImageDraw.Draw(img)
    for _ in range(r.randint(5, 10)):
        c = tuple(r.randint(0, 255) for _ in range(3))
        x0, y0 = r.randint(-40, w), r.randint(90, h - 120)
        sz = r.randint(30, 160)
        kind = r.choice(["ellipse", "rect", "tri"])
        if kind == "ellipse":
            d.ellipse([x0, y0, x0 + sz, y0 + sz], fill=c)
        elif kind == "rect":
            d.rectangle([x0, y0, x0 + sz, y0 + sz * r.uniform(0.4, 1.4)], fill=c)
        else:
            d.polygon([(x0, y0 + sz), (x0 + sz, y0 + sz), (x0 + sz / 2, y0)], fill=c)
    d.rectangle([0, 0, w, 80], fill=tuple(255 - c for c in bg))
    f = font(30 if not korean else 34, korean)
    for k, line in enumerate(textwrap.wrap(title, 16)[:2]):
        d.text((14, 6 + k * 36), line, font=f, fill=bg)
    for k in range(5):  # 본문 줄
        y = h - 110 + k * 18
        d.rectangle([16, y, 16 + r.randint(150, 320), y + 8], fill=(250, 250, 250))
    return save(img, name)


def tex_notice_board(name, seed):
    r = random.Random(seed)
    img = tint((165, 120, 75), smooth_noise(384, 640, 3, seed), 0.5).resize((640, 384))
    d = ImageDraw.Draw(img)
    f = font(14)
    for _ in range(14):
        pw, ph = r.randint(70, 140), r.randint(80, 150)
        x, y = r.randint(10, 640 - pw - 10), r.randint(10, 384 - ph - 10)
        col = r.choice([(255, 255, 255), (255, 250, 200), (220, 240, 255), (255, 220, 230)])
        d.rectangle([x, y, x + pw, y + ph], fill=col, outline=(120, 120, 120))
        for k in range(r.randint(3, 7)):
            yy = y + 12 + k * 14
            if yy < y + ph - 6:
                d.rectangle([x + 8, yy, x + 8 + r.randint(20, pw - 16), yy + 5], fill=(60, 60, 60))
        d.ellipse([x + pw // 2 - 5, y - 5, x + pw // 2 + 5, y + 5], fill=r.choice([(220, 30, 30), (30, 30, 220)]))
        if r.random() < 0.3:
            d.text((x + 6, y + ph - 20), f"No.{r.randint(1, 99)}", font=f, fill=(150, 0, 0))
    d.rectangle([0, 0, 639, 383], outline=(90, 60, 30), width=14)
    return save(img, name)


def tex_books(name, seed):
    r = random.Random(seed)
    w, h = 384, 768
    img = Image.new("RGB", (w, h), (110, 75, 45))
    d = ImageDraw.Draw(img)
    shelves = 5
    sh = h // shelves
    for s in range(shelves):
        x = 10
        top = s * sh + 12
        while x < w - 14:
            bw = r.randint(10, 26)
            bh = r.randint(int(sh * 0.55), sh - 18)
            c = tuple(r.randint(20, 230) for _ in range(3))
            d.rectangle([x, top + (sh - 18 - bh), x + bw, top + sh - 18], fill=c)
            if r.random() < 0.7:
                yy = top + (sh - 18 - bh) + r.randint(8, 30)
                d.rectangle([x + 3, yy, x + bw - 3, yy + 4], fill=(240, 220, 120))
            x += bw + r.randint(0, 2)
        d.rectangle([0, s * sh + sh - 16, w, s * sh + sh], fill=(85, 55, 30))
    return save(img, name)


def tex_whiteboard(name, seed):
    r = random.Random(seed)
    img = Image.new("RGB", (768, 384), (245, 245, 242))
    d = ImageDraw.Draw(img)
    f = font(26)
    colors = [(20, 20, 160), (180, 20, 20), (20, 120, 20), (20, 20, 20)]
    for _ in range(4):
        d.text((r.randint(20, 500), r.randint(20, 320)), r.choice(["SLAM", "Nav2", "x = Kx + Bu", "TODO", "odom", "IK/FK"]),
               font=f, fill=r.choice(colors))
    for _ in range(6):
        pts = [(r.randint(20, 740), r.randint(20, 360)) for _ in range(4)]
        d.line(pts, fill=r.choice(colors), width=4)
    d.rectangle([0, 0, 767, 383], outline=(160, 160, 165), width=10)
    return save(img, name)


def tex_cabinet(name, base, seed, drawers=4):
    img = tint(base, smooth_noise(512, 256, 8, seed), 0.08).resize((256, 512))
    d = ImageDraw.Draw(img)
    hh = 512 // drawers
    for k in range(drawers):
        d.rectangle([10, k * hh + 8, 246, (k + 1) * hh - 8], outline=(70, 70, 75), width=4)
        d.rectangle([98, k * hh + 30, 158, k * hh + 42], fill=(40, 40, 40))
        d.rectangle([110, k * hh + 52, 146, k * hh + 70], fill=(240, 240, 240), outline=(60, 60, 60))
    return save(img, name)


def tex_locker(name, seed):
    r = random.Random(seed)
    base = r.choice([(70, 110, 160), (160, 70, 60), (80, 140, 90)])
    img = tint(base, smooth_noise(512, 192, 10, seed), 0.1).resize((192, 512))
    d = ImageDraw.Draw(img)
    for k in range(5):
        d.rectangle([60, 40 + k * 10, 132, 44 + k * 10], fill=(30, 30, 30))
    d.rectangle([20, 230, 40, 300], fill=(200, 200, 200))
    d.rectangle([66, 120, 126, 150], fill=(250, 250, 250))
    d.text((74, 122), f"{r.randint(1, 60):02d}", font=font(22), fill=(0, 0, 0))
    d.rectangle([2, 2, 189, 509], outline=(30, 30, 30), width=4)
    return save(img, name)


def tex_vending(name, seed):
    r = random.Random(seed)
    img = Image.new("RGB", (384, 768), (180, 20, 30))
    d = ImageDraw.Draw(img)
    d.rectangle([20, 20, 290, 560], fill=(20, 25, 35))
    for row in range(6):
        for col in range(5):
            x, y = 32 + col * 52, 34 + row * 88
            c = tuple(r.randint(30, 255) for _ in range(3))
            d.rectangle([x, y, x + 40, y + 70], fill=c)
            d.rectangle([x + 6, y + 20, x + 34, y + 34], fill=(255, 255, 255))
    d.rectangle([305, 120, 365, 300], fill=(60, 60, 60))
    d.rectangle([40, 600, 270, 700], fill=(30, 30, 30))
    d.text((50, 715), "DRINKS", font=font(32), fill=(255, 255, 255))
    return save(img, name)


def tex_metal(name, seed, seam=True):
    n = np.asarray(Image.fromarray((np.random.default_rng(seed).random((512, 64)) * 255).astype(np.uint8))
                   .resize((512, 512), Image.BILINEAR), dtype=np.float32) / 255.0
    img = tint((175, 178, 182), n, 0.18)
    if seam:
        ImageDraw.Draw(img).line([(256, 0), (256, 512)], fill=(60, 60, 60), width=6)
    return save(img, name)


def tex_elevator_panel(name):
    img = Image.new("RGB", (128, 384), (150, 152, 156))
    d = ImageDraw.Draw(img)
    f = font(20)
    for k, lab in enumerate(["5", "4", "3", "2", "1", "B1"]):
        y = 30 + k * 50
        d.ellipse([34, y, 94, y + 40], fill=(230, 230, 225), outline=(60, 60, 60), width=3)
        d.text((52 if len(lab) == 1 else 46, y + 8), lab, font=f, fill=(20, 20, 20))
    d.rectangle([24, 330, 104, 370], fill=(20, 20, 20))
    d.text((40, 336), "3F", font=font(26), fill=(255, 80, 40))
    return save(img, name)


def tex_window(name, seed):
    r = random.Random(seed)
    w, h = 512, 384
    y = np.linspace(0, 1, h)[:, None]
    sky = np.stack([120 + 80 * y, 170 + 60 * y, 235 + 0 * y], -1).repeat(w, 1)
    img = Image.fromarray(sky.astype(np.uint8))
    d = ImageDraw.Draw(img)
    for _ in range(6):  # 구름 / 건너편 건물
        x0, y0 = r.randint(0, w), r.randint(0, h // 2)
        d.ellipse([x0, y0, x0 + r.randint(60, 160), y0 + r.randint(20, 50)], fill=(245, 245, 250))
    for _ in range(3):
        x0 = r.randint(0, w - 120)
        top = r.randint(h // 2, h - 60)
        d.rectangle([x0, top, x0 + r.randint(60, 140), h], fill=tuple(r.randint(120, 200) for _ in range(3)))
        for yy in range(top + 10, h, 24):
            d.rectangle([x0 + 8, yy, x0 + 20, yy + 10], fill=(60, 70, 90))
    for k in range(4):
        x = k * w // 3
        d.rectangle([x - 8, 0, x + 8, h], fill=(220, 220, 215))
    d.rectangle([0, h // 2 - 8, w, h // 2 + 8], fill=(220, 220, 215))
    d.rectangle([0, 0, w - 1, h - 1], outline=(220, 220, 215), width=16)
    return save(img, name)


def tex_door(name, base, seed, label=None):
    r = random.Random(seed)
    w, h = 256, 512
    x = np.arange(w)[None, :]
    grain = 0.5 + 0.5 * np.sin(x / r.uniform(3, 6) + 3 * smooth_noise(h, w, 30, seed))
    img = tint(base, grain, 0.18)
    d = ImageDraw.Draw(img)
    d.rectangle([30, 40, 226, 220], outline=tuple(int(c * 0.6) for c in base), width=6)
    d.rectangle([30, 260, 226, 480], outline=tuple(int(c * 0.6) for c in base), width=6)
    d.rectangle([90, 80, 166, 180], fill=(170, 200, 220), outline=(80, 80, 80), width=3)  # 작은 창
    d.rectangle([200, 240, 236, 252], fill=(200, 200, 200))
    if label:
        d.text((40, 225), label, font=font(24), fill=(255, 255, 255))
    return save(img, name)


def tex_plate(name, number, en, ko):
    img = Image.new("RGB", (512, 192), (30, 60, 110))
    d = ImageDraw.Draw(img)
    d.text((20, 20), number, font=font(80), fill=(255, 255, 255))
    d.text((210, 26), ko, font=font(56, korean=True), fill=(255, 255, 255))
    d.text((210, 110), en, font=font(34), fill=(200, 220, 255))
    d.rectangle([0, 0, 511, 191], outline=(200, 200, 200), width=8)
    return save(img, name)


def tex_sign(name, text, ko, bg, fg=(255, 255, 255)):
    img = Image.new("RGB", (512, 160), bg)
    d = ImageDraw.Draw(img)
    d.text((20, 30), text, font=font(64), fill=fg)
    d.text((330, 40), ko, font=font(56, korean=True), fill=fg)
    d.rectangle([0, 0, 511, 159], outline=fg, width=8)
    return save(img, name)


def tex_screen(name, seed):
    r = random.Random(seed)
    img = Image.new("RGB", (512, 320), tuple(r.randint(10, 80) for _ in range(3)))
    d = ImageDraw.Draw(img)
    for _ in range(3):
        x, y = r.randint(10, 300), r.randint(10, 180)
        ww, hh = r.randint(120, 220), r.randint(80, 140)
        d.rectangle([x, y, x + ww, y + hh], fill=(235, 235, 235), outline=(40, 40, 40), width=3)
        d.rectangle([x, y, x + ww, y + 16], fill=tuple(r.randint(40, 200) for _ in range(3)))
        for k in range(4):
            d.rectangle([x + 8, y + 26 + k * 14, x + r.randint(40, ww - 8), y + 32 + k * 14], fill=(90, 90, 90))
    d.rectangle([0, 296, 512, 320], fill=(30, 30, 30))
    return save(img, name)


def tex_paper(name, seed):
    r = random.Random(seed)
    img = Image.new("RGB", (256, 352), (250, 250, 248))
    d = ImageDraw.Draw(img)
    d.rectangle([20, 20, 236, 50], fill=(30, 30, 30))
    for k in range(16):
        d.rectangle([20, 70 + k * 16, 20 + r.randint(80, 216), 76 + k * 16], fill=(80, 80, 80))
    return save(img, name)


def make_textures():
    for f in TEX.glob("*.png"):
        f.unlink()
    t = {}
    t["wall_corridor_a"] = tex_wall("wall_corridor_a", (226, 222, 208), 11, band=(96, 128, 112))
    t["wall_corridor_b"] = tex_wall("wall_corridor_b", (220, 224, 214), 12, band=(110, 100, 90))
    t["wall_room_admin"] = tex_wall("wall_room_admin", (232, 228, 214), 13)
    t["wall_room_lab"] = tex_wall("wall_room_lab", (214, 226, 232), 14)
    t["wall_room_lounge"] = tex_wall("wall_room_lounge", (240, 226, 200), 15)
    t["wall_white"] = tex_wall("wall_white", (236, 236, 234), 16)  # 동쪽 '흰 벽' 구간 — 거의 무늬 없음
    t["floor_corridor"] = [tex_floor_tiles(f"floor_corridor_{k}", (196, 192, 184), 21 + k) for k in range(6)]
    t["floor_lab"] = [tex_floor_tiles(f"floor_lab_{k}", (205, 214, 210), 31 + k, n=6) for k in range(3)]
    t["floor_admin"] = tex_carpet("floor_admin", (95, 110, 135), 23)
    t["posters"] = [
        tex_poster("poster_0", 100, "CAPSTONE EXPO 2026"),
        tex_poster("poster_1", 101, "SAFETY FIRST"),
        tex_poster("poster_2", 102, "로봇공학 세미나", korean=True),
        tex_poster("poster_3", 103, "LIBRARY HOURS 9-18"),
        tex_poster("poster_4", 104, "취업 박람회", korean=True),
        tex_poster("poster_5", 105, "HACKATHON"),
        tex_poster("poster_6", 106, "동아리 모집", korean=True),
        tex_poster("poster_7", 107, "AI SYMPOSIUM"),
        tex_poster("poster_8", 108, "봉사활동 안내", korean=True),
        tex_poster("poster_9", 109, "CLEAN CAMPUS"),
        tex_poster("poster_10", 110, "MAKER SPACE"),
        tex_poster("poster_11", 111, "장학금 신청", korean=True),
    ]
    t["notice"] = [tex_notice_board("notice_0", 200), tex_notice_board("notice_1", 201),
                   tex_notice_board("notice_2", 202)]
    t["books"] = [tex_books(f"books_{k}", 300 + k) for k in range(4)]
    t["whiteboard"] = [tex_whiteboard("whiteboard_0", 400), tex_whiteboard("whiteboard_1", 401)]
    t["cabinet"] = tex_cabinet("cabinet", (150, 155, 160), 500)
    t["cabinet_beige"] = tex_cabinet("cabinet_beige", (200, 190, 165), 501, drawers=3)
    t["lockers"] = [tex_locker(f"locker_{k}", 600 + k) for k in range(6)]
    t["vending"] = tex_vending("vending", 700)
    t["metal"] = tex_metal("metal", 800)
    t["metal_plain"] = tex_metal("metal_plain", 801, seam=False)
    t["elevator_panel"] = tex_elevator_panel("elevator_panel")
    t["windows"] = [tex_window(f"window_{k}", 900 + k) for k in range(8)]
    t["door_admin"] = tex_door("door_admin", (140, 95, 55), 1000)
    t["door_lab"] = tex_door("door_lab", (120, 85, 60), 1001)
    t["door_lounge"] = tex_door("door_lounge", (155, 110, 70), 1002)
    t["plate_301"] = tex_plate("plate_301", "301", "ADMIN OFFICE", "행정실")
    t["plate_302"] = tex_plate("plate_302", "302", "ROBOTICS LAB", "연구실")
    t["plate_303"] = tex_plate("plate_303", "303", "LOUNGE", "휴게실")
    t["sign_exit"] = tex_sign("sign_exit", "EXIT", "비상구", (20, 140, 60))
    t["sign_fire"] = tex_sign("sign_fire", "FIRE", "소화기", (200, 30, 30))
    t["sign_elevator"] = tex_sign("sign_elevator", "ELEV.", "승강기", (40, 40, 45))
    t["screens"] = [tex_screen(f"screen_{k}", 1100 + k) for k in range(6)]
    t["papers"] = [tex_paper(f"paper_{k}", 1200 + k) for k in range(3)]
    # 머티리얼 스크립트
    lines = []
    for name in MATERIALS:
        lines.append(textwrap.dedent(f"""\
            material Capstone/{name}
            {{
              technique
              {{
                pass
                {{
                  ambient 0.9 0.9 0.9 1.0
                  diffuse 1.0 1.0 1.0 1.0
                  specular 0.05 0.05 0.05 1.0 10
                  texture_unit
                  {{
                    texture {name}.png
                    filtering anisotropic
                    max_anisotropy 8
                  }}
                }}
              }}
            }}
            """))
    SCRIPT.parent.mkdir(parents=True, exist_ok=True)
    SCRIPT.write_text("// make_world.py 가 생성 — 직접 고치지 말 것\n" + "\n".join(lines))
    return t


# =============================================================== SDF 조립
def material_xml(m):
    if m.startswith("Gazebo/"):
        return (f"<material><script><uri>file://media/materials/scripts/gazebo.material</uri>"
                f"<name>{m}</name></script></material>")
    if m.startswith("Capstone/"):
        return ("<material><script><uri>model://capstone_materials/materials/scripts</uri>"
                "<uri>model://capstone_materials/materials/textures</uri>"
                f"<name>{m}</name></script></material>")
    r, g, b = (int(m[i:i + 2], 16) / 255 for i in (1, 3, 5))  # '#rrggbb'
    return (f"<material><ambient>{r:.3f} {g:.3f} {b:.3f} 1</ambient><diffuse>{r:.3f} {g:.3f} {b:.3f} 1</diffuse>"
            f"<specular>0.1 0.1 0.1 1</specular></material>")


class Model:
    count = 0
    all = []
    plan = []  # 평면도용 (kind, x, y, sx, sy, yaw, color)

    def __init__(self, name, x, y, yaw=0.0, kind="furniture"):
        Model.count += 1
        self.name = f"{name}_{Model.count}"
        self.x, self.y, self.yaw, self.kind = x, y, yaw, kind
        self.parts = []
        self.layer = None  # Gazebo GUI 레이어 (왼쪽 패널 Layers 탭에서 켜고 끔). 센서 카메라엔 항상 보임
        Model.all.append(self)

    def _add(self, geom, xyz, rpy, mat, collide, plan_size=None):
        self.parts.append((geom, xyz, rpy, mat, collide))
        if plan_size:
            c, s = math.cos(self.yaw), math.sin(self.yaw)
            px, py = self.x + c * xyz[0] - s * xyz[1], self.y + s * xyz[0] + c * xyz[1]
            Model.plan.append((self.kind, px, py, plan_size[0], plan_size[1], self.yaw + rpy[2], mat))

    def box(self, size, xyz, mat, rpy=(0, 0, 0), collide=True, plan=True):
        sx, sy, sz = size
        self._add(f"<box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box>", xyz, rpy, mat, collide,
                  (sx, sy) if plan else None)
        return self

    def cyl(self, r, length, xyz, mat, collide=True, plan=True):
        self._add(f"<cylinder><radius>{r:.3f}</radius><length>{length:.3f}</length></cylinder>", xyz, (0, 0, 0), mat,
                  collide, (2 * r, 2 * r) if plan else None)
        return self

    def sphere(self, r, xyz, mat, collide=False):
        self._add(f"<sphere><radius>{r:.3f}</radius></sphere>", xyz, (0, 0, 0), mat, collide, None)
        return self

    def sdf(self):
        out = [f'<model name="{self.name}"><static>true</static>'
               f"<pose>{self.x:.3f} {self.y:.3f} 0 0 0 {self.yaw:.4f}</pose><link name=\"link\">"]
        for k, (g, xyz, rpy, mat, col) in enumerate(self.parts):
            pose = f"<pose>{xyz[0]:.3f} {xyz[1]:.3f} {xyz[2]:.3f} {rpy[0]:.4f} {rpy[1]:.4f} {rpy[2]:.4f}</pose>"
            meta = f"<meta><layer>{self.layer}</layer></meta>" if self.layer is not None else ""
            out.append(f'<visual name="v{k}">{meta}{pose}<geometry>{g}</geometry>{material_xml(mat)}'
                       "<cast_shadows>false</cast_shadows></visual>")
            if col:
                out.append(f'<collision name="c{k}">{pose}<geometry>{g}</geometry></collision>')
        out.append("</link></model>")
        return "".join(out)


# ---------------------------------------------------------------- 벽
def wall(p0, p1, mats, openings=(), seg=2.5, lintel_mat=None):
    """p0→p1 직선 벽. openings = [(시작, 끝)] (p0 에서 거리). 2.5 m 이하 조각으로 나눠 mats 를 번갈아 씀."""
    (x0, y0), (x1, y1) = p0, p1
    L = math.hypot(x1 - x0, y1 - y0)
    ux, uy = (x1 - x0) / L, (y1 - y0) / L
    yaw = math.atan2(uy, ux)
    solid, s = [], -T / 2
    for a, b in sorted(openings):
        solid.append((s, a))
        s = b
    solid.append((s, L + T / 2))
    k = 0
    for a, b in solid:
        n = max(1, math.ceil((b - a) / seg - 1e-9))
        for i in range(n):
            sa, sb = a + (b - a) * i / n, a + (b - a) * (i + 1) / n
            cx, cy = x0 + ux * (sa + sb) / 2, y0 + uy * (sa + sb) / 2
            Model("wall", cx, cy, yaw, kind="wall").box((sb - sa, T, H), (0, 0, H / 2), mats[k % len(mats)])
            k += 1
    for a, b in openings:  # 문 위 인방
        cx, cy = x0 + ux * (a + b) / 2, y0 + uy * (a + b) / 2
        m = Model("lintel", cx, cy, yaw, kind="wall")
        m.box((b - a, T, H - DOOR_H), (0, 0, (H + DOOR_H) / 2), lintel_mat or mats[0], plan=False)
        m.box((0.08, T + 0.04, DOOR_H), (-(b - a) / 2 + 0.04, 0, DOOR_H / 2), "#5a4632", plan=False)  # 문틀
        m.box((0.08, T + 0.04, DOOR_H), ((b - a) / 2 - 0.04, 0, DOOR_H / 2), "#5a4632", plan=False)


def on_wall(axis, coord, side, along, z, w, h, mat, d=0.02, collide=False, name="deco"):
    """벽면 부착물. axis='x' = x 방향으로 뻗은 벽(y=coord), 'y' = y 방향 벽(x=coord). side = 붙는 면 (+1/-1)."""
    if axis == "x":
        x, y, yaw = along, coord + side * (T / 2 + d / 2), 0.0 if side > 0 else PI
    else:
        x, y, yaw = coord + side * (T / 2 + d / 2), along, -PI / 2 if side > 0 else PI / 2
    return Model(name, x, y, yaw, kind="deco").box((w, d, h), (0, 0, z), mat, collide=collide, plan=False)


# ---------------------------------------------------------------- 가구 (앞 = 로컬 +y)
def desk(x, y, yaw, t, docs=False):
    m = Model("desk", x, y, yaw)
    m.box((1.2, 0.6, 0.03), (0, 0, 0.735), "Gazebo/Wood")
    for sx in (-0.56, 0.56):
        for sy in (-0.26, 0.26):
            m.box((0.04, 0.04, 0.72), (sx, sy, 0.36), "#3c3c40", plan=False)
    m.box((1.1, 0.02, 0.35), (0, -0.28, 0.55), "#6e6e73", plan=False)  # 뒤판
    m.box((0.5, 0.03, 0.32), (0, -0.15, 0.96), rng.choice(t["screens"]), collide=False, plan=False)  # 모니터
    m.box((0.06, 0.06, 0.2), (0, -0.19, 0.85), "#202020", collide=False, plan=False)
    m.box((0.42, 0.14, 0.02), (0, 0.1, 0.76), "#2a2a2a", collide=False, plan=False)  # 키보드
    if docs:  # 서류함 + 서류 더미 (최종 목표 "서류 가져오기" 대상)
        m.box((0.34, 0.26, 0.03), (0.38, 0.05, 0.765), "#d23c2d", collide=False, plan=False)  # 서류함 (0.75~0.78)
        for k in range(3):
            m.box((0.30, 0.22, 0.008), (0.38, 0.05, 0.785 + 0.009 * k), rng.choice(t["papers"]), collide=False,
                  rpy=(0, 0, rng.uniform(-0.15, 0.15)), plan=False)
    chair(x + math.cos(yaw) * 0 - math.sin(yaw) * 0.6, y + math.sin(yaw) * 0 + math.cos(yaw) * 0.6, yaw + PI)
    return m


def chair(x, y, yaw, color=None):
    color = color or rng.choice(["#2d4f8c", "#8c2d2d", "#2d2d2d", "#3c6e4b"])
    m = Model("chair", x, y, yaw)
    m.box((0.45, 0.45, 0.06), (0, 0, 0.46), color)
    m.box((0.45, 0.05, 0.45), (0, -0.21, 0.72), color, plan=False)
    m.cyl(0.03, 0.4, (0, 0, 0.23), "#505050", plan=False)
    m.box((0.5, 0.06, 0.03), (0, 0, 0.03), "#303030", plan=False)
    m.box((0.06, 0.5, 0.03), (0, 0, 0.03), "#303030", plan=False)
    return m


def cabinet(x, y, yaw, mat, h=1.2, w=0.5):
    m = Model("cabinet", x, y, yaw)
    m.box((w, 0.6, h), (0, 0, h / 2), "#8c8f94")
    m.box((w - 0.02, 0.01, h - 0.02), (0, 0.305, h / 2), mat, collide=False, plan=False)
    return m


def bookshelf(x, y, yaw, mat, w=0.9, h=1.8):
    m = Model("bookshelf", x, y, yaw)
    m.box((w, 0.35, h), (0, 0, h / 2), "Gazebo/Wood")
    m.box((w - 0.05, 0.01, h - 0.08), (0, 0.18, h / 2), mat, collide=False, plan=False)
    return m


def plant(x, y, big=True):
    m = Model("plant", x, y, rng.uniform(0, PI))
    r = 0.17 if big else 0.12
    m.cyl(r, 0.38, (0, 0, 0.19), "#a0522d")
    for k in range(3 if big else 2):
        m.sphere(r * rng.uniform(1.2, 1.8), (rng.uniform(-0.08, 0.08), rng.uniform(-0.08, 0.08), 0.55 + 0.22 * k),
                 rng.choice(["#2e7d32", "#388e3c", "#1b5e20", "#4caf50"]))
    return m


def sofa(x, y, yaw, color="#5d6d7e", w=1.8):
    m = Model("sofa", x, y, yaw)
    m.box((w, 0.8, 0.42), (0, 0, 0.21), color)
    m.box((w, 0.2, 0.45), (0, -0.3, 0.64), color, plan=False)
    m.box((0.18, 0.8, 0.62), (-w / 2 + 0.09, 0, 0.31), color, plan=False)
    m.box((0.18, 0.8, 0.62), (w / 2 - 0.09, 0, 0.31), color, plan=False)
    for k in (-0.45, 0.45):
        m.box((0.5, 0.15, 0.35), (k * w / 1.8, -0.15, 0.6), rng.choice(["#e0b04a", "#c0392b", "#ecf0f1"]),
              collide=False, plan=False)
    return m


def table(x, y, yaw, w, d, h, top="Gazebo/Wood"):
    m = Model("table", x, y, yaw)
    m.box((w, d, 0.04), (0, 0, h), top)
    for sx in (-w / 2 + 0.05, w / 2 - 0.05):
        for sy in (-d / 2 + 0.05, d / 2 - 0.05):
            m.box((0.05, 0.05, h), (sx, sy, h / 2), "#404040", plan=False)
    return m


def round_table(x, y, n_chairs=3):
    m = Model("round_table", x, y)
    m.cyl(0.45, 0.04, (0, 0, 0.73), "#f5f5f0")
    m.cyl(0.04, 0.72, (0, 0, 0.36), "#404040", plan=False)
    m.cyl(0.25, 0.03, (0, 0, 0.015), "#404040", plan=False)
    for k in range(n_chairs):
        a = 2 * PI * k / n_chairs + 0.4
        chair(x + 0.75 * math.cos(a), y + 0.75 * math.sin(a), a + PI / 2)
    return m


def fridge(x, y, yaw):
    m = Model("fridge", x, y, yaw)
    m.box((0.7, 0.7, 1.8), (0, 0, 0.9), "Capstone/metal_plain")
    m.box((0.66, 0.01, 0.01), (0, 0.355, 1.2), "#202020", collide=False, plan=False)
    m.box((0.04, 0.04, 0.5), (0.28, 0.37, 1.45), "#d0d0d0", collide=False, plan=False)
    m.box((0.04, 0.04, 0.5), (0.28, 0.37, 0.7), "#d0d0d0", collide=False, plan=False)
    m.box((0.3, 0.01, 0.2), (-0.1, 0.355, 1.5), "Capstone/paper_1", collide=False, plan=False)
    return m


def counter(x, y, yaw, w=2.0):
    m = Model("counter", x, y, yaw)
    m.box((w, 0.6, 0.88), (0, 0, 0.44), "Capstone/cabinet_beige")
    m.box((w + 0.04, 0.64, 0.04), (0, 0, 0.9), "#e8e4dc", plan=False)
    m.box((0.5, 0.38, 0.3), (-w / 2 + 0.4, -0.05, 1.07), "#2b2b2b", collide=False, plan=False)  # 전자레인지
    m.box((0.32, 0.01, 0.2), (-w / 2 + 0.36, 0.145, 1.07), "#5a7a8a", collide=False, plan=False)
    m.cyl(0.12, 0.45, (w / 2 - 0.3, 0, 1.15), "#d6eaf8", collide=False, plan=False)  # 정수기 물통
    m.box((0.3, 0.3, 0.3), (w / 2 - 0.3, 0, 0.77), "#f0f0f0", plan=False)
    m.box((0.12, 0.12, 0.2), (0, 0.05, 1.02), "#c0392b", collide=False, plan=False)  # 커피메이커
    return m


def vending(x, y, yaw):
    m = Model("vending", x, y, yaw)
    m.box((0.9, 0.8, 1.8), (0, 0, 0.9), "#b4141e")
    m.box((0.88, 0.01, 1.78), (0, 0.405, 0.9), "Capstone/vending", collide=False, plan=False)
    return m


def lockers(x, y, yaw, n, t):
    m = Model("lockers", x, y, yaw)
    for k in range(n):
        lx = (k - (n - 1) / 2) * 0.4
        m.box((0.4, 0.5, 1.8), (lx, 0, 0.9), "#5a6e8c")
        m.box((0.38, 0.01, 1.76), (lx, 0.255, 0.9), t["lockers"][k % len(t["lockers"])], collide=False, plan=False)
    return m


def bench(x, y, yaw, w=1.5):
    m = Model("bench", x, y, yaw)
    m.box((w, 0.4, 0.05), (0, 0, 0.45), "Gazebo/Wood")
    for sx in (-w / 2 + 0.1, w / 2 - 0.1):
        m.box((0.06, 0.36, 0.43), (sx, 0, 0.215), "#303030", plan=False)
    return m


def trash(x, y, color="#607080"):
    return Model("trash", x, y).cyl(0.18, 0.6, (0, 0, 0.3), color)


def extinguisher(axis, coord, side, along):
    on_wall(axis, coord, side, along, 1.4, 0.3, 0.1, "Capstone/sign_fire", name="ext_sign")
    if axis == "x":
        x, y = along, coord + side * (T / 2 + 0.1)
    else:
        x, y = coord + side * (T / 2 + 0.1), along
    m = Model("extinguisher", x, y)
    m.cyl(0.08, 0.5, (0, 0, 0.3), "#c0392b")
    m.cyl(0.03, 0.1, (0, 0, 0.6), "#202020", plan=False)
    return m


def shelving(x, y, yaw, w=1.2):
    m = Model("shelving", x, y, yaw)
    for z in (0.05, 0.5, 0.95, 1.4, 1.85):
        m.box((w, 0.45, 0.02), (0, 0, z), "#9aa0a6", plan=(z == 0.05))
    for sx in (-w / 2, w / 2):
        for sy in (-0.2, 0.2):
            m.box((0.03, 0.03, 1.9), (sx, sy, 0.95), "#5f6368", plan=False)
    for z in (0.06, 0.51, 0.96, 1.41):
        xx = -w / 2 + 0.05
        while xx < w / 2 - 0.2:
            bw = rng.uniform(0.15, 0.35)
            bh = rng.uniform(0.15, 0.38)
            m.box((bw, rng.uniform(0.25, 0.4), bh), (xx + bw / 2, 0, z + bh / 2 + 0.01),
                  rng.choice(["#c8a165", "#3d85c6", "#e06666", "#93c47d", "#f1c232", "#d9d9d9"]),
                  collide=False, plan=False)
            xx += bw + 0.03
    return m


def elevator(coord_x, y):
    """서쪽 바깥벽(x=coord_x) 안쪽 면, 문 중심 y."""
    on_wall("y", coord_x, +1, y, 1.05, 1.6, 2.1, "#3a3a3e", d=0.03, name="elev_frame")
    on_wall("y", coord_x, +1, y - 0.4, 1.05, 0.78, 2.0, "Capstone/metal_plain", d=0.05, name="elev_door_l")
    on_wall("y", coord_x, +1, y + 0.4, 1.05, 0.78, 2.0, "Capstone/metal_plain", d=0.05, name="elev_door_r")
    on_wall("y", coord_x, +1, y + 1.15, 1.2, 0.2, 0.6, "Capstone/elevator_panel", d=0.03, name="elev_panel")
    on_wall("y", coord_x, +1, y, 2.3, 0.8, 0.25, "Capstone/sign_elevator", d=0.03, name="elev_sign")


def door_leaf(cx, cy, axis, side, mat, open_dir=+1):
    """문짝 — 개구부(cx, cy) 의 한쪽 경첩에 90° 열린 상태로, side 쪽(방 안) 으로."""
    if axis == "x":  # x 방향 벽(y=const) 의 문 — 경첩 x = cx + open_dir·0.5
        hx = cx + open_dir * 0.48
        m = Model("door", hx, cy + side * (T / 2 + 0.48), PI / 2)
    else:
        hy = cy + open_dir * 0.48
        m = Model("door", cx + side * (T / 2 + 0.48), hy, 0.0)
    m.box((0.95, 0.04, 2.05), (0, 0, 1.03), mat)
    return m


# =============================================================== 배치
def build(t):
    C = (t["wall_corridor_a"], t["wall_corridor_b"])
    # ---- 바깥벽 (복도 쪽 면이 보임)
    wall((-T / 2, 0), (W + T / 2, 0), C)
    wall((-T / 2, D), (W + T / 2, D), C)
    wall((0, 0), (0, D), C)
    wall((W, 0), (W, D), (t["wall_white"],))  # 동쪽: 흰 벽 구간
    # ---- 방 블록 (복도 쪽 면 = 복도 텍스처, 방 안은 아래에서 얇은 판으로 덮음)
    wall((BX0, BY0), (BX1, BY0), C, openings=[(2.5, 3.5), (12.0, 13.0)])  # 행정실 문 x5~6, 휴게실 문 x14.5~15.5
    wall((BX0, BY1), (BX1, BY1), C, openings=[(7.5, 8.5)])  # 연구실 문 x10~11
    wall((BX0, BY0), (BX0, BY1), C)
    wall((BX1, BY0), (BX1, BY1), (t["wall_white"],))
    wall((8.5, BY0), (8.5, BY1), (t["wall_room_admin"],))
    wall((13.0, BY0), (13.0, BY1), (t["wall_room_lounge"],))

    # ---- 방 안쪽 벽면 덮개 (방마다 다른 벽지)
    def room_skin(x0, x1, y0, y1, mat, south_open=(), north_open=()):
        inset = T / 2 + 0.006
        for (axis, coord, side, a0, a1, opens) in (("x", y0, +1, x0, x1, south_open), ("x", y1, -1, x0, x1, north_open),
                                                   ("y", x0, +1, y0, y1, ()), ("y", x1, -1, y0, y1, ())):
            cuts, s = [], a0 + inset
            for a, b in sorted(opens):
                cuts.append((s, a))
                s = b
            cuts.append((s, a1 - inset))
            for a, b in cuts:
                n = max(1, math.ceil((b - a) / 2.5))
                for i in range(n):
                    sa, sb = a + (b - a) * i / n, a + (b - a) * (i + 1) / n
                    on_wall(axis, coord, side, (sa + sb) / 2, H / 2 - 0.005, sb - sa, H - 0.02, mat, d=0.008,
                            name="skin")

    room_skin(BX0, 8.5, BY0, BY1, t["wall_room_admin"], south_open=[(5.0, 6.0)])
    room_skin(8.5, 13.0, BY0, BY1, t["wall_room_lab"], north_open=[(10.0, 11.0)])
    room_skin(13.0, BX1, BY0, BY1, t["wall_room_lounge"], south_open=[(14.5, 15.5)])

    # ---- 바닥 (2 m 조각 — 텍스처가 늘어지지 않게)
    def floor(x0, x1, y0, y1, mat, step=2.0):  # mat 이 리스트면 조각마다 무작위
        nx, ny = max(1, round((x1 - x0) / step)), max(1, round((y1 - y0) / step))
        m = Model("floor", (x0 + x1) / 2, (y0 + y1) / 2, kind="floor")
        for i in range(nx):
            for j in range(ny):
                sx, sy = (x1 - x0) / nx, (y1 - y0) / ny
                mm = rng.choice(mat) if isinstance(mat, list) else mat
                m.box((sx, sy, 0.01), (x0 + sx * (i + 0.5) - m.x, y0 + sy * (j + 0.5) - m.y, 0.005), mm,
                      rpy=(0, 0, rng.choice([0, PI / 2, PI, -PI / 2])) if sx == sy else (0, 0, 0),
                      collide=False, plan=False)

    floor(0, W, 0, BY0, t["floor_corridor"])
    floor(0, W, BY1, D, t["floor_corridor"])
    floor(0, BX0, BY0, BY1, t["floor_corridor"], step=2.33)
    floor(BX1, W, BY0, BY1, t["floor_corridor"], step=2.33)
    floor(BX0, 8.5, BY0, BY1, t["floor_admin"])
    floor(8.5, 13.0, BY0, BY1, t["floor_lab"], step=2.25)
    floor(13.0, BX1, BY0, BY1, "Gazebo/WoodFloor", step=2.25)

    # ---- 천장 (2 m 타일, 충돌 없음). 그림자를 꺼 둬서 조명은 안 가림. GUI 에선 레이어 1 → Layers 탭에서 끄면 위에서 보임
    ceil = Model("ceiling", W / 2, D / 2, kind="ceiling")
    ceil.layer = 1
    for i in range(10):
        for j in range(6):
            ceil.box((2.0, 2.0, 0.02), (-W / 2 + 1.0 + 2 * i, -D / 2 + 1.0 + 2 * j, H + 0.01), "Gazebo/CeilingTiled",
                     collide=False, plan=False)

    # ---- 문짝, 명판
    door_leaf(5.5, BY0, "x", +1, t["door_admin"], open_dir=-1)
    door_leaf(10.5, BY1, "x", -1, t["door_lab"], open_dir=+1)
    door_leaf(15.0, BY0, "x", +1, t["door_lounge"], open_dir=+1)
    on_wall("x", BY0, -1, 6.45, 1.65, 0.48, 0.18, t["plate_301"], name="plate")
    on_wall("x", BY1, +1, 11.45, 1.65, 0.48, 0.18, t["plate_302"], name="plate")
    on_wall("x", BY0, -1, 15.95, 1.65, 0.48, 0.18, t["plate_303"], name="plate")

    # ---- 창문 (남·북 바깥벽, 서로 다른 그림)
    wins = iter(t["windows"])
    for x in (3.5, 7.5, 11.5):
        on_wall("x", 0, +1, x, 1.6, 1.8, 1.2, next(wins), d=0.03, name="window")
    for x in (3.0, 7.0, 13.0, 17.0):
        on_wall("x", D, -1, x, 1.6, 1.8, 1.2, next(wins), d=0.03, name="window")

    # ---- 포스터 / 게시판 (복도)
    P = iter(t["posters"])
    for x in (3.5, 8.2, 10.5, 12.6):  # 남쪽 복도, 방 블록 남쪽 면
        on_wall("x", BY0, -1, x, 1.5, 0.6, 0.85, next(P), name="poster")
    for x in (4.0, 7.0, 13.5, 16.3):  # 북쪽 복도, 방 블록 북쪽 면
        on_wall("x", BY1, +1, x, 1.5, 0.6, 0.85, next(P), name="poster")
    on_wall("x", BY1, +1, 8.5, 1.45, 1.6, 1.0, t["notice"][0], name="notice")
    for y in (3.6, 8.4):  # 서쪽 복도, 방 블록 서쪽 면
        on_wall("y", BX0, -1, y, 1.5, 0.6, 0.85, next(P), name="poster")
    on_wall("y", BX0, -1, 6.0, 1.45, 1.6, 1.0, t["notice"][1], name="notice")
    on_wall("x", D, -1, 10.0, 1.5, 0.6, 0.85, next(P), name="poster")
    on_wall("x", 0, +1, 9.5, 1.5, 0.6, 0.85, next(P), name="poster")

    # ---- 복도 물건
    elevator(0.0, 6.0)
    on_wall("x", 0, +1, 0.9, 2.3, 0.6, 0.19, t["sign_exit"], d=0.03, name="exit_sign")
    on_wall("x", D, -1, 19.1, 2.3, 0.6, 0.19, t["sign_exit"], d=0.03, name="exit_sign")
    lockers(13.9, T / 2 + 0.25, 0.0, 6, t)
    bench(8.0, 0.35, 0.0)
    bench(5.0, D - 0.35, PI)
    vending(16.9, T / 2 + 0.4, 0.0)
    plant(1.0, D - 0.6)
    plant(1.6, 0.45, big=False)
    plant(BX1 - 0.4, 9.95, big=False)
    trash(2.1, 0.35)
    trash(9.2, D - 0.3, "#3a7d44")
    trash(19.6, 6.0)  # 동쪽(흰 벽) 구간엔 이것뿐
    extinguisher("y", 0.0, +1, 2.6)
    extinguisher("x", D, -1, 15.0)
    extinguisher("y", W, -1, 9.0)

    # ---- 행정실 (x 2.5~8.5, y 2.5~9.5) — 책상 4 + 서류, 캐비닛, 책장, 프린터
    desk(4.3, 6.0, PI, t, docs=True)
    desk(4.3, 7.6, 0.0, t)
    desk(6.7, 6.0, PI, t)
    desk(6.7, 7.6, 0.0, t, docs=True)
    for k, y in enumerate((3.4, 3.95, 4.5)):
        cabinet(BX0 + T / 2 + 0.3, y, -PI / 2, t["cabinet"])
    bookshelf(3.6, BY1 - T / 2 - 0.2, PI, t["books"][0])
    bookshelf(4.55, BY1 - T / 2 - 0.2, PI, t["books"][1])
    cabinet(7.9, 3.3, PI / 2, t["cabinet_beige"], h=0.75, w=0.6)
    Model("printer", 7.9, 3.3, PI / 2).box((0.5, 0.45, 0.35), (0, 0, 0.93), "#e8e8e8").box(
        (0.4, 0.02, 0.06), (0, 0.22, 0.85), "#303030", collide=False, plan=False)
    on_wall("y", 8.5, -1, 6.8, 1.45, 1.8, 0.9, t["whiteboard"][0], name="whiteboard")
    plant(8.05, 9.05)
    trash(3.1, 3.0)

    # ---- 연구실 (x 8.5~13, y 2.5~9.5) — 실험대, 선반
    table(10.6, 4.3, 0.0, 2.0, 0.8, 0.85, top="#dfe6e9")
    table(10.6, 6.3, 0.0, 2.0, 0.8, 0.85, top="#dfe6e9")
    for (x, y) in ((10.0, 3.6), (11.2, 3.6), (10.0, 7.0), (11.2, 7.0)):
        chair(x, y, 0.0 if y < 5 else PI, "#2d2d2d")
    shelving(12.55, 4.2, PI / 2)
    shelving(12.55, 5.8, PI / 2)
    desk(9.3, 8.6, PI, t)
    on_wall("x", BY0, +1, 10.6, 1.45, 2.0, 1.0, t["whiteboard"][1], name="whiteboard")
    lab_items = Model("lab_items", 10.6, 4.3)
    lab_items.cyl(0.08, 0.35, (-0.6, 0, 1.05), "#f39c12", collide=False, plan=False)
    lab_items.box((0.4, 0.3, 0.25), (0.2, 0.05, 0.995), "#34495e", collide=False, plan=False)
    lab_items.box((0.3, 0.2, 0.1), (0.7, -0.1, 0.92), "#27ae60", collide=False, plan=False)
    bookshelf(13.0 - T / 2 - 0.175, 7.6, PI / 2, t["books"][2])  # 동쪽 벽, 선반 북쪽 (서쪽 통로 막지 않게)
    trash(12.6, 8.9, "#7f8c8d")

    # ---- 휴게실 (x 13~17.5, y 2.5~9.5) — 소파, 냉장고, 조리대, 원탁
    sofa(15.25, BY1 - T / 2 - 0.45, PI)
    table(15.25, 7.9, 0.0, 1.0, 0.5, 0.4)
    fridge(BX1 - T / 2 - 0.4, 3.4, PI / 2)
    counter(BX1 - T / 2 - 0.35, 5.3, PI / 2)
    round_table(14.4, 5.4)
    plant(13.5, 9.0)
    bookshelf(13.0 + T / 2 + 0.175, 7.4, -PI / 2, t["books"][3], w=0.8, h=1.2)
    on_wall("y", 13.0, +1, 4.0, 1.5, 1.6, 1.0, t["notice"][2], name="notice")


def world_xml():
    models = "\n".join(m.sdf() for m in Model.all)
    return f"""<?xml version="1.0"?>
<!-- make_world.py 가 생성 — 직접 고치지 말 것 (배치는 build(), 텍스처는 make_textures()) -->
<sdf version="1.6">
  <world name="school_floor">
    <physics type="ode">
      <max_step_size>0.002</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>500</real_time_update_rate>
    </physics>
    <scene>
      <ambient>0.55 0.55 0.55 1</ambient>
      <background>0.75 0.8 0.85 1</background>
      <shadows>false</shadows>
      <grid>false</grid>
    </scene>
    <!-- 해·바닥을 직접 정의 (model://sun, ground_plane 을 찾다가 온라인 DB 조회로 멈추는 일 없게) -->
    <light name="sun" type="directional">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>
    <light name="fill" type="directional">
      <cast_shadows>false</cast_shadows>
      <pose>10 6 10 0 0 0</pose>
      <diffuse>0.35 0.35 0.35 1</diffuse>
      <specular>0.05 0.05 0.05 1</specular>
      <direction>0.3 0.5 -1</direction>
    </light>
    <model name="ground_plane"><static>true</static><link name="link">
      <collision name="c"><geometry><plane><normal>0 0 1</normal><size>60 60</size></plane></geometry></collision>
      <visual name="v"><cast_shadows>false</cast_shadows><geometry><plane><normal>0 0 1</normal><size>60 60</size></plane></geometry>
        <material><ambient>0.45 0.47 0.45 1</ambient><diffuse>0.45 0.47 0.45 1</diffuse></material></visual>
    </link></model>
    <!-- /gazebo/set_entity_state, get_entity_state — 시험 때 로봇 순간이동 / 참값 조회 -->
    <plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so">
      <ros><namespace>/gazebo</namespace></ros>
      <update_rate>1.0</update_rate>
    </plugin>
    <gui fullscreen="0">
      <camera name="user_camera">
        <pose>10 -6 16 0 0.95 1.5708</pose>
      </camera>
    </gui>
{models}
  </world>
</sdf>
"""


def draw_plan():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle
    from matplotlib import transforms, font_manager

    try:
        font_manager.fontManager.addfont("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc")
        plt.rcParams["font.family"] = "Noto Sans CJK JP"
    except Exception:
        pass
    fig, ax = plt.subplots(figsize=(12, 7.6))
    colors = {"wall": "#333333", "furniture": "#8fa9c7"}
    for kind, x, y, sx, sy, yaw, _ in Model.plan:
        if kind not in colors:
            continue
        r = Rectangle((-sx / 2, -sy / 2), sx, sy, facecolor=colors[kind], edgecolor="none" if kind == "wall" else "#4a6a8a",
                      lw=0.5, transform=transforms.Affine2D().rotate(yaw).translate(x, y) + ax.transData)
        ax.add_patch(r)
    for txt, x, y in (("행정실 301\n(책상 4, 서류)", 5.5, 4.9), ("연구실 302", 10.75, 5.3), ("휴게실 303", 15.25, 6.6),
                      ("남쪽 복도", 9.5, 1.6), ("북쪽 복도", 9.5, 10.9), ("엘리베이터", 0.9, 6.0),
                      ("흰 벽 구간", 18.75, 6.0)):
        ax.text(x, y, txt, ha="center", va="center", fontsize=10, color="#222")
    ax.plot([1.25], [1.25], "r^", ms=12)
    ax.text(1.25, 0.55, "스폰", ha="center", color="r")
    ax.set_xlim(-0.5, W + 0.5)
    ax.set_ylim(-0.5, D + 0.5)
    ax.set_aspect("equal")
    ax.grid(alpha=0.25)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title("school_floor — 20 × 12 m (복도가 방 블록을 한 바퀴 → 루프 클로저 시험)")
    PLAN.parent.mkdir(parents=True, exist_ok=True)
    fig.tight_layout()
    fig.savefig(PLAN, dpi=100)
    plt.close(fig)


def main():
    t = make_textures()
    build(t)
    WORLD.parent.mkdir(parents=True, exist_ok=True)
    WORLD.write_text(world_xml())
    draw_plan()
    print(f"월드 {WORLD.relative_to(PKG)}: 모델 {len(Model.all)} 개, 텍스처 {len(MATERIALS)} 장, 평면도 {PLAN.relative_to(PKG)}")


if __name__ == "__main__":
    main()
