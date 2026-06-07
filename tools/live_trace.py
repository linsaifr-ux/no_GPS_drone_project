#!/usr/bin/env python3
"""
Live flight-trace viewer — reads the growing CSV while the drone is flying.

Usage:
    python3 tools/live_trace.py               # auto-attach to latest/newest trace
    python3 tools/live_trace.py <trace.csv>   # specific file

Overlays:
  - Planned survey route (12 waypoints, 6-strip lawnmower)
  - Raw detection zone boundary (solid white)
  - Buffered boundary 30 m inward (orange dashed)
  - Detected vehicles from detections.csv (refreshed live)
  - AGL target line at 65 m

The window updates at ~5 Hz as drone_sim.py / cesium_scene.py writes new rows.
Close the window to stop.
"""
import argparse
import csv
import datetime
import math
import os
import sys
import time

try:
    import matplotlib.pyplot as plt
    import matplotlib.animation as animation
    from matplotlib.patches import Polygon as MplPolygon
    import numpy as np
except ImportError:
    sys.exit("matplotlib + numpy required:  pip install matplotlib numpy")

HERE      = os.path.dirname(os.path.abspath(__file__))
PROJ_ROOT = os.path.join(HERE, "..")
TRACE_DIR = os.path.join(PROJ_ROOT, "simulator", "flight_traces")
DET_LOG   = os.path.join(PROJ_ROOT, "detections.csv")

# ── Survey constants (mirror of px4_commander.py) ─────────────────────────────
HOME_LAT  = 23.450868
HOME_LON  = 120.286135
COS_LAT   = math.cos(math.radians(HOME_LAT))
M_PER_DEG = 111_320.0

TARGET_AGL = 65.0
WP_RADIUS  = 60.0

# (north_m, east_m) — same order as SURVEY_WPS in px4_commander.py
SURVEY_WPS = [
    (210.0,   -545.0),
    (517.0,   -545.0),
    (545.0,   -695.0),
    (8.0,     -695.0),
    (36.0,    -845.0),
    (573.0,   -845.0),
    (601.0,   -995.0),
    (65.0,    -995.0),
    (93.0,   -1145.0),
    (629.0,  -1145.0),
    (408.0,  -1250.0),
    (113.0,  -1250.0),
]

# Raw detection zone boundary (actual area corners), CW: NW→NE→SE→SW
RAW_ZONE_VERTS = [
    (677.0, -1240.0),  # NW
    (531.0,  -454.0),  # NE
    (-48.0,  -563.0),  # SE
    ( 97.0, -1327.0),  # SW
]

# Buffered zone boundary (30 m inward), CW: NW'→NE'→SE'→SW'
ZONE_VERTS = [
    (642.0, -1215.0),
    (507.0,  -489.0),
    (-13.0,  -587.0),
    (121.0, -1293.0),
]

# Category colours for detection markers
_CAT_COLOR = {"car": "#ff4444", "van": "#ff88aa", "truck": "#ff8800", "bus": "#ffcc00"}


# ── Helpers ────────────────────────────────────────────────────────────────────

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
        with open(path) as f:
            reader = csv.DictReader(f)
            for row in reader:
                try:
                    t.append(float(row["t_s"]))
                    e.append(float(row["east_m"]))
                    n.append(float(row["north_m"]))
                    agl.append(float(row["agl_m"]))
                except (ValueError, KeyError):
                    pass
    except FileNotFoundError:
        pass
    return np.array(t), np.array(e), np.array(n), np.array(agl)


def read_detections(min_timestamp=0.0):
    """Return list of (east_m, north_m, category) from detections.csv.

    Only rows whose Unix timestamp >= min_timestamp are returned, so detections
    from previous flights (which share the same CSV file) are not shown.
    """
    dets = []
    try:
        with open(DET_LOG) as f:
            reader = csv.DictReader(f)
            for row in reader:
                try:
                    if float(row["timestamp"]) < min_timestamp:
                        continue
                    lat = float(row["lat"])
                    lon = float(row["lon"])
                    north = (lat - HOME_LAT) * M_PER_DEG
                    east  = (lon - HOME_LON) * M_PER_DEG * COS_LAT
                    dets.append((east, north, row["category"]))
                except (ValueError, KeyError):
                    pass
    except FileNotFoundError:
        pass
    return dets


# ── Build figure ───────────────────────────────────────────────────────────────

fig, (ax_top, ax_alt) = plt.subplots(1, 2, figsize=(14, 7))
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

# ── Top view — static elements ─────────────────────────────────────────────────

# Raw detection zone boundary — solid white outline, no fill
_raw_zone_xy = [(e, n) for n, e in RAW_ZONE_VERTS]
raw_zone_poly = MplPolygon(_raw_zone_xy, closed=True,
                            fill=False, edgecolor="#ffffff",
                            linestyle="-", linewidth=1.5, zorder=1)
ax_top.add_patch(raw_zone_poly)

# Buffered zone (30 m inward) — orange dashed, faint fill
_zone_xy = [(e, n) for n, e in ZONE_VERTS]
zone_poly = MplPolygon(_zone_xy, closed=True,
                        fill=True, facecolor="#ff880011", edgecolor="#ff8800",
                        linestyle="--", linewidth=1.0, zorder=1)
ax_top.add_patch(zone_poly)

# Planned survey route — connect WPs in order (east, north)
_route_e = [0.0] + [e for _, e in SURVEY_WPS]
_route_n = [0.0] + [n for n, _ in SURVEY_WPS]
ax_top.plot(_route_e, _route_n,
            color="#666688", linewidth=0.8, linestyle=":", zorder=2, label="Planned route")

# WP markers + labels
for idx, (wn, we) in enumerate(SURVEY_WPS):
    label = "ENTRY" if idx == 0 else f"WP{idx:02d}"
    ax_top.plot(we, wn, ".", color="#8888bb", markersize=6, zorder=3)
    ax_top.annotate(label, (we, wn), textcoords="offset points", xytext=(4, 2),
                    color="#8888bb", fontsize=6, zorder=3)

# Home
ax_top.plot(0, 0, "^", color="#aaffaa", markersize=9, zorder=4, label="Home")

ax_top.set_xlabel("East (m)")
ax_top.set_ylabel("North (m)")
ax_top.set_title("Top view — live", pad=8)
ax_top.set_aspect("equal")

# ── Altitude view — static elements ───────────────────────────────────────────

ax_alt.axhline(TARGET_AGL, color="#ffaa44", linestyle=":", linewidth=1.0,
               label=f"Target {TARGET_AGL:.0f} m")
ax_alt.set_xlabel("Time (s)")
ax_alt.set_ylabel("AGL (m)")
ax_alt.set_title("Altitude — live", pad=8)

# ── Dynamic artists ────────────────────────────────────────────────────────────

trace_line, = ax_top.plot([], [], color="#4488ff", linewidth=1.5, zorder=5, label="Actual path")
pos_dot,    = ax_top.plot([], [], "o", color="#ffffff", markersize=7, zorder=7)
det_scatter = ax_top.scatter([], [], s=100, marker="*", zorder=8,
                              color="#ff4444", label="Detection")

alt_line,  = ax_alt.plot([], [], color="#44ddaa", linewidth=1.5)
time_dot,  = ax_alt.plot([], [], "o", color="#ffffff", markersize=7, zorder=5)

from matplotlib.lines import Line2D as _Line2D
_legend_handles = [
    _Line2D([0],[0], color="#ffffff",  lw=1.5, ls="-",  label="Zone boundary"),
    _Line2D([0],[0], color="#ff8800",  lw=1.0, ls="--", label="Buffered boundary (30 m)"),
    _Line2D([0],[0], color="#666688",  lw=0.8, ls=":",  label="Planned route"),
    _Line2D([0],[0], color="#4488ff",  lw=1.5, ls="-",  label="Actual path"),
    _Line2D([0],[0], marker="^", color="#aaffaa", ms=8, ls="none", label="Home"),
    _Line2D([0],[0], marker="*", color="#ff4444", ms=10, ls="none", label="Detection"),
]
ax_top.legend(handles=_legend_handles,
              facecolor="#2a2a3e", edgecolor="#555577", labelcolor="white", fontsize=7,
              loc="upper right")
ax_alt.legend(facecolor="#2a2a3e", edgecolor="#555577", labelcolor="white", fontsize=8)

status_txt = fig.text(0.5, 0.01, "", ha="center", color="#aaaacc", fontsize=9)
plt.tight_layout(rect=[0, 0.03, 1, 1])

# ── Axis limits ────────────────────────────────────────────────────────────────

# Pre-set to cover the survey zone
ax_top.set_xlim(-1450, 250)
ax_top.set_ylim(-200, 800)
ax_alt.set_xlim(0, 120)
ax_alt.set_ylim(0, TARGET_AGL + 20)

_csv_path          = [None]
_det_last_mtime    = [0.0]
_det_cache         = [[]]   # list of (east, north, cat)
_flight_start_epoch = [0.0]  # Unix timestamp of the current flight (from trace filename)


def _expand_top(xs, ys, margin=80):
    if len(xs) == 0:
        return
    xl, xr = ax_top.get_xlim(); yb, yt = ax_top.get_ylim()
    nxl = min(np.min(xs) - margin, xl); nxr = max(np.max(xs) + margin, xr)
    nyb = min(np.min(ys) - margin, yb); nyt = max(np.max(ys) + margin, yt)
    if nxl != xl or nxr != xr: ax_top.set_xlim(nxl, nxr)
    if nyb != yb or nyt != yt: ax_top.set_ylim(nyb, nyt)
    ax_top.set_aspect("equal", adjustable="datalim")


def update(_frame):
    path = _csv_path[0]
    if path is None:
        return trace_line, pos_dot, alt_line, time_dot, det_scatter, status_txt

    t, e, n, agl = read_csv_fast(path)

    if len(t) < 2:
        status_txt.set_text(f"Waiting for data … ({os.path.basename(path)})")
        return trace_line, pos_dot, alt_line, time_dot, det_scatter, status_txt

    trace_line.set_data(e, n)
    pos_dot.set_data([e[-1]], [n[-1]])
    alt_line.set_data(t, agl)
    time_dot.set_data([t[-1]], [agl[-1]])

    # Auto-expand top view to keep drone visible
    all_e = np.append(e, _route_e)
    all_n = np.append(n, _route_n)
    _expand_top(all_e, all_n)

    ax_alt.set_xlim(0, max(t[-1] + 15, ax_alt.get_xlim()[1]))
    ax_alt.set_ylim(0, max(np.max(agl) + 10, ax_alt.get_ylim()[1]))

    # Refresh detections if file changed
    try:
        mtime = os.path.getmtime(DET_LOG)
    except FileNotFoundError:
        mtime = 0.0
    if mtime != _det_last_mtime[0]:
        _det_cache[0] = read_detections(min_timestamp=_flight_start_epoch[0])
        _det_last_mtime[0] = mtime

    dets = _det_cache[0]
    if dets:
        det_e = np.array([d[0] for d in dets])
        det_n = np.array([d[1] for d in dets])
        det_scatter.set_offsets(np.c_[det_e, det_n])
        det_scatter.set_sizes([100] * len(dets))

    # Nearest survey WP
    dists = [math.hypot(e[-1] - we, n[-1] - wn) for wn, we in SURVEY_WPS]
    nearest_idx = int(np.argmin(dists))
    nearest_d   = dists[nearest_idx]
    wp_label = "ENTRY" if nearest_idx == 0 else f"WP{nearest_idx:02d}"

    det_info = f"  dets={len(dets)}" if dets else ""
    status_txt.set_text(
        f"t={t[-1]:.0f}s  E={e[-1]:+.0f}  N={n[-1]:+.0f}  "
        f"AGL={agl[-1]:.1f}m  nearest={wp_label}({nearest_d:.0f}m){det_info}"
    )
    return trace_line, pos_dot, alt_line, time_dot, det_scatter, status_txt


ani = animation.FuncAnimation(fig, update, interval=200,
                               blit=False, cache_frame_data=False)


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

    # Parse flight start time from filename (trace_YYYYMMDD_HHMMSS.csv).
    # Detections older than this are from previous flights and are hidden.
    fname = os.path.basename(path)
    try:
        dt = datetime.datetime.strptime(fname, "trace_%Y%m%d_%H%M%S.csv")
        _flight_start_epoch[0] = dt.timestamp()
        print(f"Flight start: {dt.strftime('%Y-%m-%d %H:%M:%S')} — older detections hidden")
    except ValueError:
        _flight_start_epoch[0] = 0.0   # unknown format: show all detections

    print(f"Live trace: {path}")
    if os.path.exists(DET_LOG):
        print(f"Detections: {DET_LOG}")
    fig.canvas.manager.set_window_title(f"Live trace — {fname}")
    _csv_path[0] = path
    plt.show()


if __name__ == "__main__":
    main()
