"""ODrive(XDrive) 속도-전류 캐스케이드 블록도 생성.  python make_fig_cascade.py → fig_cascade.png / .svg"""
import os
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, Circle

from matplotlib import font_manager
for _f in ("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc", "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc"):
    if os.path.exists(_f):
        font_manager.fontManager.addfont(_f)
plt.rcParams["font.family"] = ["Noto Sans CJK JP", "DejaVu Sans"]
plt.rcParams["svg.fonttype"] = "none"

fig, ax = plt.subplots(figsize=(16, 7.0))
ax.set_xlim(0, 16)
ax.set_ylim(0, 7.0)
ax.axis("off")

C_OUT, C_IN, C_BLK, C_FB = "#e8f0fb", "#fdf0e3", "#ffffff", "#555555"
Y = 4.0  # main forward path


def region(x0, y0, x1, y1, color, edge, label, sub):
    ax.add_patch(FancyBboxPatch((x0, y0), x1 - x0, y1 - y0, boxstyle="round,pad=0.02,rounding_size=0.15",
                                fc=color, ec=edge, lw=1.5, zorder=0))
    ax.text(x0 + 0.15, y1 - 0.15, label, ha="left", va="top", fontsize=13, weight="bold", color=edge)
    ax.text(x0 + 0.15, y1 - 0.48, sub, ha="left", va="top", fontsize=10, color=edge)


def block(x, y, w, h, text, fs=11, fc=C_BLK, ec="#222"):
    ax.add_patch(FancyBboxPatch((x - w / 2, y - h / 2), w, h, boxstyle="round,pad=0.02,rounding_size=0.08",
                                fc=fc, ec=ec, lw=1.4, zorder=2))
    ax.text(x, y, text, ha="center", va="center", fontsize=fs, zorder=3)


def summer(x, y):
    ax.add_patch(Circle((x, y), 0.2, fc="white", ec="#222", lw=1.4, zorder=2))
    ax.text(x, y, "Σ", ha="center", va="center", fontsize=11, zorder=3)
    ax.text(x - 0.32, y + 0.2, "+", fontsize=11, ha="center", va="center")
    ax.text(x + 0.2, y - 0.38, "−", fontsize=13, ha="center", va="center")


def arrow(p0, p1, color="#222", ls="-", lw=1.5):
    ax.annotate("", xy=p1, xytext=p0, zorder=1,
                arrowprops=dict(arrowstyle="-|>", color=color, lw=lw, ls=ls, shrinkA=0, shrinkB=0, mutation_scale=14))


def line(pts, color="#222", ls="-", lw=1.5):
    xs, ys = zip(*pts)
    ax.plot(xs, ys, color=color, ls=ls, lw=lw, zorder=1)


def label(x, y, t, fs=9.5, color="#333", **kw):
    ax.text(x, y, t, ha="center", va="center", fontsize=fs, color=color, **kw)


# ---- regions (nested = cascade)
region(1.08, 0.35, 15.0, 6.95, C_OUT, "#2b5d9c", "속도 루프 (바깥 · 느림)",
       "목표 속도를 내려면 토크가 얼마나 필요한가?  →  Iq 명령을 만든다")
region(5.75, 2.05, 14.55, 5.75, C_IN, "#b8650f", "전류 루프 (안쪽 · 빠름, 8 kHz)",
       "그 토크(= Iq)가 실제로 흐르도록 전압을 맞춘다  →  PWM")

# ---- forward path
label(0.5, Y + 0.6, "input_vel", fs=10.5, weight="bold")
label(0.5, Y + 0.3, "(turn/s)", fs=9)
arrow((0.1, Y), (1.3, Y))
summer(1.5, Y)
arrow((1.7, Y), (2.1, Y))
block(3.0, Y, 1.8, 0.8, "속도 PI")
label(3.0, Y - 0.65, "P: vel_gain = 0.238", fs=9.5)
label(3.0, Y - 0.93, "I: vel_integrator_gain = 1.19", fs=9.5)
arrow((3.9, Y), (4.45, Y))
label(4.17, Y + 0.6, "토크 Te\n(Nm)", fs=9)
block(4.95, Y, 0.9, 0.7, "÷ Kt", fs=11)
arrow((5.4, Y), (6.1, Y))
label(5.25, Y + 0.6, "Iq 명령\n(A)", fs=9.5, weight="bold")
summer(6.3, Y)
arrow((6.5, Y), (6.9, Y))
block(7.8, Y, 1.8, 0.8, "전류 PI\n(d·q 축)", fs=11)
label(7.8, Y - 0.78, "Kp = L·ω_bw ≈ 0.016\nKi = R·ω_bw ≈ 28", fs=9)
arrow((8.7, Y), (9.05, Y))
label(8.88, Y + 0.6, "Vd, Vq", fs=9)
block(9.9, Y, 1.6, 0.8, "역 Park\n+ SVPWM", fs=10.5)
arrow((10.7, Y), (11.05, Y))
label(10.88, Y + 0.6, "duty", fs=9)
block(11.8, Y, 1.4, 0.8, "인버터\nMOSFET ×6", fs=10.5)
arrow((12.5, Y), (12.9, Y))
label(12.7, Y + 0.6, "3상", fs=9)
block(13.6, Y, 1.3, 0.8, "BLDC\n허브모터", fs=10.5, fc="#eeeeee")
arrow((14.25, Y), (15.55, Y))
label(15.3, Y + 0.3, "바퀴 회전", fs=11, weight="bold")

# ---- inner feedback: phase current
block(11.8, 2.7, 1.4, 0.65, "상전류 센서", fs=10, fc="#fff8f0")
line([(11.8, Y - 0.4), (11.8, 3.03)], color=C_FB)
arrow((11.8, 3.2), (11.8, 3.03), color=C_FB)
block(9.9, 2.7, 1.6, 0.65, "Clarke · Park", fs=10, fc="#fff8f0")
arrow((11.1, 2.7), (10.7, 2.7), color=C_FB)
line([(9.1, 2.7), (6.3, 2.7)], color=C_FB)
arrow((6.3, 2.7), (6.3, Y - 0.2), color=C_FB)
label(7.6, 2.45, "Iq 측정 (A)", fs=9.5, color="#b8650f", weight="bold")

# ---- outer feedback: Hall → velocity
block(13.6, 1.05, 1.3, 0.65, "Hall 센서", fs=10, fc="#f2f6fc")
line([(13.6, Y - 0.4), (13.6, 1.38)], color=C_FB)
arrow((13.6, 1.6), (13.6, 1.38), color=C_FB)
block(7.8, 1.05, 2.2, 0.65, "속도·전기각 추정 (PLL)", fs=10, fc="#f2f6fc")
arrow((12.95, 1.05), (8.9, 1.05), color=C_FB)
label(11.0, 1.3, "hall_state (60°마다 변화)", fs=8.5, color="#555")
line([(6.7, 1.05), (1.5, 1.05)], color=C_FB)
arrow((1.5, 1.05), (1.5, Y - 0.2), color=C_FB)
label(4.0, 0.8, "vel_estimate (turn/s)", fs=9.5, color="#2b5d9c", weight="bold")

# θe to Park transforms (dashed)
line([(8.9, 1.2), (9.9, 1.2), (9.9, 2.37)], color="#888", ls="--", lw=1.2)
arrow((9.9, 2.2), (9.9, 2.37), color="#888", lw=1.2)
line([(9.9, 3.03), (9.9, 3.6)], color="#888", ls="--", lw=1.2)
arrow((9.9, 3.45), (9.9, 3.6), color="#888", lw=1.2)
label(10.25, 1.75, "θe", fs=10, color="#666", style="italic")
label(10.25, 3.3, "θe", fs=10, color="#666", style="italic")

out = os.path.dirname(os.path.abspath(__file__))
fig.savefig(os.path.join(out, "fig_cascade.png"), dpi=160, bbox_inches="tight", facecolor="white")
fig.savefig(os.path.join(out, "fig_cascade.svg"), bbox_inches="tight", facecolor="white")
print("saved", os.path.join(out, "fig_cascade.png"))
