"""Gambar sudut belok untuk laporan (skripsi): definisi, sudut belok vs sudut bank, dan rencana jalur.

Gambar 1 dan 2 adalah ilustrasi umum. Gambar 3 dibuat dari dua file .waypoints
Mission Planner (segmen, sudut belok Dpsi, dan koridor toleransi +-45 m).

Pakai:
    python gambar_sudut_belok.py OUT_DIR "skenario1.waypoints" "skenario2.waypoints"
Tanpa argumen: gambar ditulis ke ./gambar_sudut_belok dan memakai file skenario
default di folder PLAN WP (lihat DEFAULT_*).
"""
import math, os, sys, importlib.util
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Arc, Circle, FancyArrowPatch

DEFAULT_S1 = r"E:/SKRIPSI/RAFAEL VIDENTIO S/PLAN WP/COBA_SKRIPSI_FW_3 WAYPOINT.waypoints"
DEFAULT_S2 = r"E:/SKRIPSI/RAFAEL VIDENTIO S/PLAN WP/COBA_SKRIPSI_FW_WAYPOINT.waypoints"
HERE = os.path.dirname(os.path.abspath(__file__))
OUT = sys.argv[1] if len(sys.argv) > 1 else os.path.join(HERE, "gambar_sudut_belok")
PLAN1 = sys.argv[2] if len(sys.argv) > 2 else DEFAULT_S1
PLAN2 = sys.argv[3] if len(sys.argv) > 3 else DEFAULT_S2
os.makedirs(OUT, exist_ok=True)
plt.rcParams.update({"font.size": 11, "axes.linewidth": 0.8})


def bvec(b):  # bearing (derajat dari utara, searah jarum jam) -> vektor satuan (timur, utara)
    r = math.radians(b)
    return np.array([math.sin(r), math.cos(r)])


def arc(ax, c, r, b1, b2, **kw):
    """Busur dari bearing b1 searah jarum jam ke b2 (digambar dengan sudut matematis)."""
    ax.add_patch(Arc(c, 2 * r, 2 * r, theta1=90 - b2, theta2=90 - b1, **kw))


def arrow(ax, p, q, **kw):
    ax.add_patch(FancyArrowPatch(p, q, arrowstyle="-|>", mutation_scale=14, shrinkA=0, shrinkB=0, **kw))


# ----------------------------------------------------------------- Gambar 1
fig, ax = plt.subplots(figsize=(8.4, 6.2))
L = 10.0
b_in, b_out = 45.0, 105.0
turn = b_out - b_in  # +60 derajat (belok kanan)
W0 = np.array([0.0, 0.0])
Wm = -L * bvec(b_in)
Wp = L * bvec(b_out)
ext = 0.62 * L * bvec(b_in)
ax.plot([Wm[0], 0], [Wm[1], 0], "-", color="tab:red", lw=2.6)
ax.plot([0, Wp[0]], [0, Wp[1]], "-", color="tab:red", lw=2.6)
ax.plot([0, ext[0]], [0, ext[1]], "--", color="0.35", lw=1.4)
arrow(ax, Wm + 0.40 * (W0 - Wm), Wm + 0.52 * (W0 - Wm), color="tab:red", lw=0)
arrow(ax, 0.40 * Wp, 0.52 * Wp, color="tab:red", lw=0)
for p, name, off in ((Wm, "WP(k−1)", (-0.4, -1.3)), (W0, "WP(k)", (-1.9, 0.6)), (Wp, "WP(k+1)", (-1.6, 0.8))):
    ax.plot(*p, "o", ms=9, color="k", zorder=5)
    ax.annotate(name, p + np.array(off), fontsize=12, weight="bold")
arc(ax, W0, 4.2, b_in, b_out, color="tab:blue", lw=2.2)
ax.annotate("Δψ = sudut belok\n= β$_{keluar}$ − β$_{masuk}$ = %d°\n(belok kanan)" % turn,
            (5.4, 2.4), color="tab:blue", fontsize=11.5, weight="bold")
arc(ax, W0, 2.4, b_out, b_in + 180, color="tab:green", lw=2.0)
ax.annotate("θ = sudut dalam\n= 180° − Δψ = %d°" % (180 - turn), (-1.6, -5.8), color="tab:green",
            fontsize=11.5, weight="bold")
ax.annotate("leg masuk\n(segmen WP(k−1) → WP(k))", (-10.6, -2.5), fontsize=10.5, color="tab:red")
ax.annotate("leg keluar\n(segmen WP(k) → WP(k+1))", (4.6, -4.7), fontsize=10.5, color="tab:red")
ax.annotate("perpanjangan leg masuk\n(arah bila tidak berbelok)", (-5.2, 5.2), fontsize=10, color="0.25")
arrow(ax, (-10.2, 5.2), (-10.2, 7.6), color="k", lw=1.4)
ax.annotate("Utara", (-11.0, 7.9), fontsize=11)
ax.text(-11.2, -8.6, "β = arah (bearing) segmen, diukur dari utara searah jarum jam.\n"
        "Δψ > 0: belok kanan, Δψ < 0: belok kiri.\n"
        "Sudut belok adalah perubahan arah LINTASAN di waypoint,\nbukan kemiringan sayap (roll) wahana.",
        fontsize=10, va="top", bbox=dict(boxstyle="round", fc="#f6f6f6", ec="0.6"))
ax.set_xlim(-11.8, 12); ax.set_ylim(-12.4, 9.8)
ax.set_aspect("equal")
ax.axis("off")
ax.set_title("Definisi sudut belok pada waypoint", fontsize=13, weight="bold")
fig.tight_layout()
fig.savefig(os.path.join(OUT, "gambar1_definisi_sudut_belok.png"), dpi=200)
plt.close(fig)

# ----------------------------------------------------------------- Gambar 2
V, bank, g = 18.0, 45.0, 9.80665
R = V ** 2 / (g * math.tan(math.radians(bank)))
d_lead = R * math.tan(math.radians(90) / 2)
fig, (a1, a2) = plt.subplots(1, 2, figsize=(12.8, 5.8), gridspec_kw=dict(width_ratios=[1.15, 1]))
Lg = 100.0
a1.plot([0, 0], [-Lg, 0], "-", color="tab:red", lw=2.6)
a1.plot([0, Lg], [0, 0], "-", color="tab:red", lw=2.6)
a1.plot(0, 0, "o", ms=9, color="k")
a1.annotate("WP(k)", (-20, 7), fontsize=12, weight="bold")
c = np.array([R, -R])
th = np.linspace(math.pi, math.pi / 2, 60)
a1.plot(c[0] + R * np.cos(th), c[1] + R * np.sin(th), "-", color="tab:blue", lw=3.0)
a1.plot(*c, "+", color="tab:blue", ms=10)
a1.plot([c[0], c[0]], [c[1], 0], ":", color="tab:blue", lw=1)
a1.plot([0, c[0]], [c[1], c[1]], ":", color="tab:blue", lw=1)
a1.annotate("R = V² / (g·tan φ)\n= %.0f m" % R, (c[0] + 3, c[1] - 6), color="tab:blue", fontsize=11.5, weight="bold")
a1.annotate("", (-5, 0), (-5, -R), arrowprops=dict(arrowstyle="<->", color="tab:green", lw=1.6))
a1.annotate("jarak awal belok\nd = R·tan(Δψ/2)\n= %.0f m" % d_lead, (-66, -R / 2 - 6), color="tab:green", fontsize=11, weight="bold")
a1.add_patch(Circle((0, 0), 15, fill=False, ls="--", ec="0.4", lw=1.2))
a1.annotate("radius klaim\nWP 15 m", (-44, 22), fontsize=10, color="0.3")
a1.add_patch(Arc((0, 0), 2 * 58, 2 * 58, theta1=0, theta2=90, color="tab:orange", lw=2.0))
a1.plot([0, 0], [0, 66], "--", color="0.35", lw=1.3)
a1.annotate("perpanjangan\nleg masuk", (-50, 46), fontsize=10, color="0.3")
a1.annotate("Δψ = 90°\n(sudut belok)", (36, 44), color="tab:orange", fontsize=11.5, weight="bold")
arrow(a1, (0, -Lg + 14), (0, -Lg + 38), color="tab:red", lw=0)
arrow(a1, (Lg - 38, 0), (Lg - 14, 0), color="tab:red", lw=0)
a1.annotate("leg masuk", (6, -Lg + 22), color="tab:red", fontsize=10.5)
a1.annotate("leg keluar", (Lg - 72, 8), color="tab:red", fontsize=10.5)
a1.annotate("lintasan wahana saat belok\n(menyinggung kedua leg)", (R + 6, -R - 40), color="tab:blue", fontsize=10)
a1.set_xlim(-72, Lg + 8)
a1.set_ylim(-Lg - 6, 72)
a1.set_aspect("equal")
a1.axis("off")
a1.set_title("Tampak atas: sudut belok (Δψ) dan radius belok (R)", fontsize=12.5, weight="bold")

phi = math.radians(bank)
a2.axhline(0, color="0.4", ls="--", lw=1.2)
a2.annotate("horizon", (-5.7, 0.15), fontsize=10, color="0.35")
half = 3.6
wl = np.array([-half * math.cos(phi), +half * math.sin(phi)])
wr = np.array([+half * math.cos(phi), -half * math.sin(phi)])
a2.plot([wl[0], wr[0]], [wl[1], wr[1]], "-", color="k", lw=7, solid_capstyle="round")
a2.plot(0, 0, "o", ms=16, color="k")
a2.add_patch(Arc((0, 0), 2 * 2.4, 2 * 2.4, theta1=-bank, theta2=0, color="tab:orange", lw=2.4))
a2.annotate("φ = sudut bank (roll)\n= %d°" % bank, (2.9, -1.0), color="tab:orange", fontsize=12, weight="bold")
lift = 3.0 * np.array([math.sin(phi), math.cos(phi)])
arrow(a2, (0, 0), lift.tolist(), color="tab:purple", lw=2.2)
a2.annotate("gaya angkat L\n(miring ke arah\ndalam belok)", (lift[0] - 3.4, lift[1] + 0.3), color="tab:purple", fontsize=10.5)
arrow(a2, (0, 0), (0, -2.4), color="tab:brown", lw=2.2)
a2.annotate("berat W = m·g", (-3.4, -2.7), color="tab:brown", fontsize=10.5)
arrow(a2, (0, 0), (2.1, 0), color="tab:green", lw=2.2)
a2.annotate("L·sin φ\n(gaya sentripetal,\nmenuju pusat belokan)", (2.3, 0.25), color="tab:green", fontsize=10.5)
a2.text(-5.8, -4.0, "Semakin besar bank φ atau semakin kecil kecepatan V,\nsemakin kecil radius belok R. Pada V = 18 m/s:\n"
        "φ = 36° → R = 45 m   |   φ = 45° → R = 33 m", fontsize=10.2, va="top",
        bbox=dict(boxstyle="round", fc="#f6f6f6", ec="0.6"))
a2.set_xlim(-6, 8.0)
a2.set_ylim(-5.9, 4.4)
a2.set_aspect("equal")
a2.axis("off")
a2.set_title("Tampak belakang wahana: sudut bank (φ)", fontsize=12.5, weight="bold")
fig.suptitle("Sudut belok (geometri lintasan) berbeda dengan sudut bank (kemiringan wahana)",
             fontsize=13.5, weight="bold", y=1.0)
fig.tight_layout()
fig.savefig(os.path.join(OUT, "gambar2_sudut_belok_vs_sudut_bank.png"), dpi=200, bbox_inches="tight")
plt.close(fig)

# ----------------------------------------------------------------- Gambar 3 (rencana nyata)
spec = importlib.util.spec_from_file_location(
    "a", os.path.join(HERE, "analyze_tlog_l1_auto.py"))
a = importlib.util.module_from_spec(spec)
spec.loader.exec_module(a)


def load(p):
    rows = [l.split("\t") for l in open(p, encoding="utf-8").read().splitlines()[1:] if l.strip()]
    la0, lo0 = float(rows[0][8]), float(rows[0][9])
    kx = 111320 * math.cos(math.radians(la0))
    return np.array([[(float(r[9]) - lo0) * kx, (float(r[8]) - la0) * 111320] for r in rows])


plans = [("Skenario 1 (%d waypoint)" % (sum(1 for l in open(PLAN1, encoding="utf-8") if l.strip()) - 2), PLAN1),
         ("Skenario 2 (%d waypoint)" % (sum(1 for l in open(PLAN2, encoding="utf-8") if l.strip()) - 2), PLAN2)]
fig, axes = plt.subplots(1, 2, figsize=(14, 7.4))
for ax, (title, p) in zip(axes, plans):
    xy = load(p)
    H, W = xy[0], xy[1:]
    cx, cy = a.corridor(W[:, 0], W[:, 1], 45.0)
    ax.fill(cx, cy, color="#dce8ff", zorder=0)
    ax.plot(cx, cy, ":", color="navy", lw=1.2, label="koridor toleransi ±45 m")
    ax.plot(W[:, 0], W[:, 1], "-", color="tab:red", lw=2.4, label="jalur rencana", zorder=3)
    ax.plot(*H, "k^", ms=10, label="HOME", zorder=4)
    ax.plot([H[0], W[0, 0]], [H[1], W[0, 1]], "--", color="0.5", lw=1.2)
    bear = np.degrees(np.arctan2(np.diff(W[:, 0]), np.diff(W[:, 1]))) % 360
    legs = np.hypot(*np.diff(W, axis=0).T)
    for i, w in enumerate(W):
        ax.plot(*w, "o", ms=8, color="k", zorder=5)
        ax.annotate("WP%d" % (i + 1), w + np.array([14, 12]), fontsize=12, weight="bold")
    for i, ln in enumerate(legs):
        m = (W[i] + W[i + 1]) / 2
        d = (W[i + 1] - W[i]) / ln
        nrm = np.array([-d[1], d[0]])
        ax.annotate("%.0f m" % ln, m + nrm * 26, ha="center", va="center", fontsize=10.5, color="tab:red",
                    bbox=dict(boxstyle="round,pad=0.15", fc="white", ec="none", alpha=0.85))
    for i in range(1, len(W) - 1):
        dpsi = ((bear[i] - bear[i - 1] + 180) % 360) - 180
        side = "kanan" if dpsi > 0 else "kiri"
        b1, b2 = (bear[i - 1], bear[i]) if dpsi > 0 else (bear[i], bear[i - 1])
        ext_pt = W[i] + bvec(bear[i - 1]) * 85
        ax.plot([W[i][0], ext_pt[0]], [W[i][1], ext_pt[1]], "--", color="0.4", lw=1.2, zorder=2)
        arc(ax, W[i], 62, b1, b2, color="tab:blue", lw=2.2)
        mid_b = bear[i - 1] + dpsi / 2
        ax.annotate("Δψ = %d°\n(%s)" % (abs(round(dpsi)), side), W[i] + bvec(mid_b) * 120, ha="center", va="center",
                    fontsize=10.5, color="tab:blue", weight="bold",
                    bbox=dict(boxstyle="round,pad=0.2", fc="white", ec="tab:blue", lw=0.8), zorder=6)
    ax.set_aspect("equal")
    ax.grid(alpha=0.25)
    ax.set_xlabel("timur dari HOME [m]")
    ax.set_ylabel("utara dari HOME [m]")
    ax.set_title(title, fontsize=13, weight="bold")
    xs, ys = np.r_[cx, H[0]], np.r_[cy, H[1]]
    ax.set_xlim(xs.min() - 40, xs.max() + 40)
    ax.set_ylim(ys.min() - 40, ys.max() + 40)
from matplotlib.lines import Line2D
handles = [Line2D([0], [0], color="tab:red", lw=2.4), Line2D([0], [0], color="navy", ls=":", lw=1.4),
           Line2D([0], [0], marker="^", color="k", ls="", ms=9), Line2D([0], [0], color="0.4", ls="--", lw=1.2),
           Line2D([0], [0], color="tab:blue", lw=2.2)]
fig.legend(handles, ["jalur rencana", "koridor toleransi ±45 m", "HOME", "perpanjangan segmen masuk", "sudut belok Δψ"],
           loc="lower center", ncol=5, fontsize=10, frameon=False)
fig.suptitle("Rencana jalur kedua skenario pengujian: panjang segmen dan sudut belok di setiap waypoint",
             fontsize=13.5, weight="bold")
fig.tight_layout(rect=(0, 0.05, 1, 1))
fig.savefig(os.path.join(OUT, "gambar3_skenario_sudut_belok.png"), dpi=190)
plt.close(fig)
print("gambar ditulis ke", OUT)
