#!/usr/bin/env python3
"""
Live flight-trace viewer — reads the growing CSV while the drone is flying.

Usage:
    python3 tools/live_trace.py               # auto-attach to latest/newest trace
    python3 tools/live_trace.py <trace.csv>   # specific file

The window updates at ~5 Hz as drone_sim.py / cesium_scene.py writes new rows.
Close the window to stop.
"""
import argparse
import csv
import os
import sys
import time

try:
    import matplotlib.pyplot as plt
    import matplotlib.animation as animation
    from matplotlib.patches import Circle
    import numpy as np
except ImportError:
    sys.exit("matplotlib + numpy required:  pip install matplotlib numpy")

HERE      = os.path.dirname(os.path.abspath(__file__))
TRACE_DIR = os.path.join(HERE, "..", "simulator", "flight_traces")

WP_NORTH, WP_EAST = 531.2, -453.9
WP_RADIUS         = 60.0
TARGET_AGL        = 90.0

# ── helpers ────────────────────────────────────────────────────────────────────

def newest_trace():
    os.makedirs(TRACE_DIR, exist_ok=True)
    while True:
        files = sorted(
            [f for f in os.listdir(TRACE_DIR)
             if f.startswith("trace_") and f.endswith(".csv")],
            reverse=True,
        )
        if files:
            return os.path.join(TRACE_DIR, files[0])
        print("Waiting for a trace file to appear …", end="\r", flush=True)
        time.sleep(1)


def read_csv_fast(path):
    """Return (t, east, north, agl) numpy arrays from a partially-written CSV."""
    t, e, n, agl = [], [], [], []
    try:
        with open(path, "r") as f:
            reader = csv.DictReader(f)
            for row in reader:
                try:
                    t.append(float(row["t_s"]))
                    e.append(float(row["east_m"]))
                    n.append(float(row["north_m"]))
                    agl.append(float(row["agl_m"]))
                except (ValueError, KeyError):
                    pass   # skip incomplete last row
    except FileNotFoundError:
        pass
    return (np.array(t), np.array(e), np.array(n), np.array(agl))


# ── build figure ───────────────────────────────────────────────────────────────

fig, (ax_top, ax_alt) = plt.subplots(1, 2, figsize=(13, 6))
fig.patch.set_facecolor("#1e1e2e")
for ax in (ax_top, ax_alt):
    ax.set_facecolor("#2a2a3e")
    ax.tick_params(colors="white")
    ax.xaxis.label.set_color("white")
    ax.yaxis.label.set_color("white")
    ax.title.set_color("white")
    for spine in ax.spines.values():
        spine.set_edgecolor("#555577")
    ax.grid(True, color="#444466", linewidth=0.5)

# Top view static elements
wp_circle = Circle((WP_EAST, WP_NORTH), WP_RADIUS,
                   fill=False, color="#ff6060", linestyle="--", linewidth=1.2)
ax_top.add_patch(wp_circle)
ax_top.plot(WP_EAST, WP_NORTH, "*", color="#ff6060", markersize=14, label="Waypoint")
ax_top.plot(0, 0, "^", color="#aaffaa", markersize=9, label="Home")
ax_top.set_xlabel("East (m)"); ax_top.set_ylabel("North (m)")
ax_top.set_title("Top view — live", pad=8)
ax_top.set_aspect("equal")
ax_top.legend(facecolor="#2a2a3e", edgecolor="#555577", labelcolor="white", fontsize=8)

# Altitude view static elements
ax_alt.axhline(TARGET_AGL, color="#ffaa44", linestyle=":", linewidth=1.0, label=f"Target {TARGET_AGL:.0f} m")
ax_alt.set_xlabel("Time (s)"); ax_alt.set_ylabel("AGL (m)")
ax_alt.set_title("Altitude — live", pad=8)
ax_alt.legend(facecolor="#2a2a3e", edgecolor="#555577", labelcolor="white", fontsize=8)

# Dynamic line objects
trace_line,  = ax_top.plot([], [], color="#4488ff", linewidth=1.5)
pos_dot,     = ax_top.plot([], [], "o", color="#ffffff", markersize=7, zorder=5)
alt_line,    = ax_alt.plot([], [], color="#44ddaa", linewidth=1.5)
time_dot,    = ax_alt.plot([], [], "o", color="#ffffff", markersize=7, zorder=5)

status_txt = fig.text(0.5, 0.01, "", ha="center", color="#aaaacc", fontsize=9)
plt.tight_layout(rect=[0, 0.03, 1, 1])

# ── animation ──────────────────────────────────────────────────────────────────

_csv_path = [None]

# Axis limits — expand as data grows
_xlim = [-600, 100]
_ylim = [-100, 600]
_tlim = [0, 60]
_alim = [0, 100]

ax_top.set_xlim(*_xlim)
ax_top.set_ylim(*_ylim)
ax_alt.set_xlim(*_tlim)
ax_alt.set_ylim(*_alim)


def _expand(ax, set_xlim, set_ylim, xs, ys, margin=50):
    changed = False
    if len(xs) == 0:
        return False
    xmin, xmax = np.min(xs) - margin, np.max(xs) + margin
    ymin, ymax = np.min(ys) - margin, np.max(ys) + margin
    cur_xl, cur_xr = ax.get_xlim()
    cur_yb, cur_yt = ax.get_ylim()
    if xmin < cur_xl or xmax > cur_xr:
        set_xlim(min(xmin, cur_xl), max(xmax, cur_xr)); changed = True
    if ymin < cur_yb or ymax > cur_yt:
        set_ylim(min(ymin, cur_yb), max(ymax, cur_yt)); changed = True
    return changed


def update(_frame):
    path = _csv_path[0]
    if path is None:
        return trace_line, pos_dot, alt_line, time_dot, status_txt

    t, e, n, agl = read_csv_fast(path)

    if len(t) < 2:
        status_txt.set_text(f"Waiting for data … ({os.path.basename(path)})")
        return trace_line, pos_dot, alt_line, time_dot, status_txt

    trace_line.set_data(e, n)
    pos_dot.set_data([e[-1]], [n[-1]])
    alt_line.set_data(t, agl)
    time_dot.set_data([t[-1]], [agl[-1]])

    # Auto-expand axes
    _expand(ax_top, ax_top.set_xlim, ax_top.set_ylim,
            np.append(e, [WP_EAST, 0]), np.append(n, [WP_NORTH, 0]))
    ax_top.set_aspect("equal", adjustable="datalim")

    t_range = max(t[-1] + 10, ax_alt.get_xlim()[1])
    ax_alt.set_xlim(0, t_range)
    agl_max = max(np.max(agl) + 10, ax_alt.get_ylim()[1])
    ax_alt.set_ylim(0, agl_max)

    dist = np.hypot(e[-1] - WP_EAST, n[-1] - WP_NORTH)
    status_txt.set_text(
        f"t={t[-1]:.0f}s  E={e[-1]:+.0f}  N={n[-1]:+.0f}  "
        f"AGL={agl[-1]:.1f}m  dist_to_WP={dist:.0f}m"
    )
    return trace_line, pos_dot, alt_line, time_dot, status_txt


ani = animation.FuncAnimation(fig, update, interval=200, blit=False, cache_frame_data=False)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv", nargs="?", help="CSV trace file (default: latest)")
    args = ap.parse_args()

    if args.csv:
        path = args.csv
        if not os.path.isfile(path):
            sys.exit(f"File not found: {path}")
    else:
        path = newest_trace()

    print(f"Live trace: {path}")
    fig.canvas.manager.set_window_title(f"Live trace — {os.path.basename(path)}")
    _csv_path[0] = path
    plt.show()


if __name__ == "__main__":
    main()
