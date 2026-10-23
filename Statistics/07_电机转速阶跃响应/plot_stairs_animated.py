# -*- coding: utf-8 -*-
"""Animated multi-level speed step response (0 -> 30% -> 60% -> 100% -> 60% -> 30% -> 0%).

Curves are drawn one after another into a single axes: first the target
staircase, then the measured speed of M1, M2, M3.  Each curve is revealed
with an ease-out profile (fast at the beginning, slowing down towards the
end), so the transient at each step stays readable.

Style: academic / publication look (serif fonts, boxed axes with inward
ticks on all sides, dashed grid, framed legend), following the reference
figures in ../img.

Outputs (written next to this script):
  staircase_response_animated.gif  - the animation
  staircase_response_animated.png  - final static overlay (all curves)

Usage:
  python plot_stairs_animated.py              # save GIF/PNG, then show window
  python plot_stairs_animated.py --save-only  # save files only (headless)
"""

import argparse
import re
from pathlib import Path

import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter

# ----------------------------- configuration -----------------------------
FS       = 180.0                # rad/s, "full speed" = 100%
T_WIN    = (-3.3, 22.5)         # display window (s), 0 = first step
DT       = 0.04                 # resampled grid step (s) = 1/FPS
FPS      = 25                   # animation frame rate
T_TARGET = 2.5                  # drawing time of the target staircase (s)
T_CURVE  = 4.5                  # drawing time of each motor curve (s)
T_PAUSE  = 0.5                  # pause between curves (s)
T_HOLD   = 1.2                  # final hold on the finished plot (s)
EASE     = 2.0                  # ease-out exponent: 1 = constant, larger = stronger deceleration

MOTORS   = [('M1', 'm1_stairs.csv'), ('M2', 'm2_stairs.csv'), ('M3', 'm3_stairs.csv')]
COLORS   = {'M1': '#C62828',    # deep red
            'M2': '#1565C0',    # deep blue
            'M3': '#2E7D32'}    # deep green
C_TARGET = '#666666'

# ------------------------------- data I/O --------------------------------
FRAME_RE = re.compile(r'^([0-9.]+),([0-9.]+),"\$TELE,(.*)#"$')


def read_motor(path, idx):
    """Parse one motor channel (M#target, S#filtered speed) from a $TELE log."""
    tgt_re = re.compile(r'M%d:([-+0-9.Ee]+)' % idx)
    act_re = re.compile(r'S%d:([-+0-9.Ee]+)' % idx)
    t, tgt, act = [], [], []
    with open(path, 'r', encoding='utf-8', errors='ignore') as f:
        for line in f:
            m = FRAME_RE.match(line.strip())
            if not m:
                continue
            mt, ma = tgt_re.search(m.group(3)), act_re.search(m.group(3))
            if not (mt and ma):
                continue
            t.append(float(m.group(2)) / 1000.0)
            tgt.append(float(mt.group(1)))
            act.append(float(ma.group(1)))
    if len(t) < 50:
        raise RuntimeError('too few telemetry rows in %s (%d)' % (path, len(t)))
    t, tgt, act = map(np.asarray, (t, tgt, act))
    o = np.argsort(t)
    t, tgt, act = t[o], tgt[o], act[o]
    started = np.flatnonzero(tgt > 0.01 * FS)          # first step (30%) onset
    t0 = t[started[0]] if started.size else t[0]
    return t - t0, tgt, act


def step_interp(xq, x, y):
    """Zero-order-hold interpolation, keeps the target a true staircase."""
    if x.size == 0:
        return np.zeros_like(xq)
    x2 = np.concatenate([x, x + 1e-6])
    y2 = np.concatenate([y, y])
    o = np.argsort(x2)
    return np.interp(xq, x2[o], y2[o])


# ------------------------------ figure setup -----------------------------
plt.rcParams.update({
    'font.family':      'serif',
    'font.serif':       ['Times New Roman', 'STIXGeneral', 'DejaVu Serif'],
    'mathtext.fontset': 'stix',
    'font.size':        12,
    'axes.labelsize':   13,
    'axes.linewidth':   0.9,
    'xtick.direction':  'in',
    'ytick.direction':  'in',
    'xtick.top':        True,
    'ytick.right':      True,
    'xtick.minor.visible': True,
    'ytick.minor.visible': True,
    'legend.frameon':   True,
    'legend.edgecolor': 'black',
    'legend.framealpha': 1.0,
    'legend.fontsize':  11,
})

HERE = Path(__file__).resolve().parent
data = {}
for name, fn in MOTORS:
    data[name] = read_motor(HERE / fn, int(name[1]))

t_u = np.arange(T_WIN[0], T_WIN[1] + DT / 2, DT)
y_target = step_interp(t_u, data['M1'][0], data['M1'][1])
y_motor = {k: np.interp(t_u, data[k][0], data[k][2]) for k, _ in MOTORS}

fig, ax = plt.subplots(figsize=(8.6, 5.4), dpi=100)
fig.subplots_adjust(left=0.11, right=0.89, top=0.88, bottom=0.13)
ax.set_axisbelow(True)
ax.grid(True, linestyle='--', linewidth=0.55, color='0.72')

for v in (54.0, 108.0, 180.0):                       # step levels (30/60/100 %)
    ax.axhline(v, color='0.82', linewidth=0.6, linestyle=':', zorder=0.5)

ax.set_xlim(*T_WIN)
ax.set_ylim(-10, 200)
ax.set_xlabel('Time (s)')
ax.set_ylabel('Angular velocity (rad/s)')
ax.set_title('Multi-level step speed response of three reaction wheels\n'
             r'$0\%\rightarrow30\%\rightarrow60\%\rightarrow100\%'
             r'\rightarrow60\%\rightarrow30\%\rightarrow0\%$',
             fontsize=13)

secax = ax.secondary_yaxis('right', functions=(lambda v: 100.0 * v / FS,
                                               lambda p: p * FS / 100.0))
secax.set_ylabel('Speed (% of full speed)')
secax.set_yticks([0, 30, 60, 100])

lines = {'target': ax.plot([], [], '--', color=C_TARGET, lw=1.4, label='Target speed')[0]}
for name, _ in MOTORS:
    lines[name] = ax.plot([], [], '-', color=COLORS[name], lw=1.8, label=name)[0]
leg = [lines['target']] + [lines[n] for n, _ in MOTORS]
ax.legend(leg, [ln.get_label() for ln in leg], loc='upper left')

# --------------------------- animation timeline ---------------------------
ease_out = lambda u: 1.0 - (1.0 - u) ** EASE      # fast -> slow drawing speed


def add_phase(frames, key, T):
    n = max(2, int(round(T * FPS)))
    frames.extend({key: ease_out(i / n)} for i in range(1, n + 1))


frames = []
add_phase(frames, 'target', T_TARGET)
frames.append({})                                  # pause
for name, _ in MOTORS:
    add_phase(frames, name, T_CURVE)
    frames.append({})                              # pause
frames.extend([{}] * int(T_HOLD * FPS))            # hold the finished plot

full = {k: (t_u, y) for k, y in [('target', y_target)] + list(y_motor.items())}


def update(fi):
    for key, p in frames[fi].items():
        tq, yq = full[key]
        n = max(2, int(round(p * tq.size)))
        lines[key].set_data(tq[:n], yq[:n])
    return []


def init():
    for ln in lines.values():
        ln.set_data([], [])
    return []


anim = FuncAnimation(fig, update, frames=len(frames), init_func=init,
                     interval=1000 / FPS, blit=False, repeat=True)

# ------------------------------- outputs ---------------------------------
ap = argparse.ArgumentParser()
ap.add_argument('--save-only', action='store_true', help='do not open a window')
args = ap.parse_args()

gif_path = HERE / 'staircase_response_animated.gif'
png_path = HERE / 'staircase_response_animated.png'

print('Writing %s (%d frames, %.1f s) ...' % (gif_path.name, len(frames), len(frames) / FPS))
anim.save(gif_path, writer=PillowWriter(fps=FPS))
print('Writing %s ...' % png_path.name)
for key in full:                                   # finish every curve
    lines[key].set_data(*full[key])
fig.savefig(png_path, dpi=300)
print('Done.')

if not args.save_only:
    if matplotlib.get_backend().lower() != 'agg':
        plt.show()
