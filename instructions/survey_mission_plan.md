# Survey Mission Plan — Detection Zone Lawnmower + Car Detection Response

## Overview

Autonomous lawnmower survey of a defined detection zone west of home. When YOLO detects a
vehicle inside the zone, the drone diverts to centre the target in frame, logs the
geolocation, then resumes the survey route.

| Parameter | Value |
|-----------|-------|
| Cruise AGL | 65 m |
| Cruise speed | 12 m/s |
| Strip spacing | 150 m E-W |
| Strips | 6 (4 full + 2 partial edge strips) |
| Total distance | ≈ 5.4 km |
| Estimated flight time | ≈ 7.6 min |
| AnyLoc error | ~20 m → 30 m inward buffer |
| Camera footprint at 65 m | 125 m × 83 m (HFOV 88°, VFOV 65.1°) |
| Sidelap | 125 − 150 = −25 m (25 m gap between strips) |

---

## Detection Zone

### Raw boundary corners (lat, lon)

| Corner | Lat | Lon |
|--------|-----|-----|
| NW | 23.45695 | 120.27399 |
| NE | 23.45564 | 120.28169 |
| SE | 23.45044 | 120.28062 |
| SW | 23.45174 | 120.27314 |

### In NED metres from home (23.450868°N, 120.286135°E)

COS_LAT ≈ 0.9175; M_PER_DEG_LAT = 111 320; M_PER_DEG_LON ≈ 102 136.

| Corner | North (m) | East (m) |
|--------|-----------|----------|
| NW | +677 | −1240 |
| NE | +531 | −454 |
| SE | −48  | −563 |
| SW | +97  | −1327 |

The NE corner (531, −454) is the existing single test waypoint — coordinate math confirmed.

### Buffered boundary (30 m inward — exceeds AnyLoc ~20 m error)

Each boundary edge shifted 30 m toward the polygon interior. New corners at edge
intersections:

| Corner | North (m) | East (m) |
|--------|-----------|----------|
| NW' | +642 | −1215 |
| NE' | +507 | −489 |
| SE' | −13  | −587 |
| SW' | +121 | −1293 |

Polygon edges (CW):
- **Northern:** NW'(642,−1215) → NE'(507,−489)
- **Eastern:**  NE'(507,−489)  → SE'(−13,−587)
- **Southern:** SE'(−13,−587)  → SW'(121,−1293)
- **Western:**  SW'(121,−1293) → NW'(642,−1215)

---

## Lawnmower Strip Plan

### Design rationale

- E-W strip spacing 150 m, camera footprint width 125 m → 25 m gap between strips.
- 6 strips cover 88% of the buffered zone (≈705 m of ≈810 m E-W span).
- At 12 m/s + 5 fps the drone advances 2.4 m between frames — detection quality is
  unchanged from slower speeds.
- 12 m/s is within PX4's default `MPC_XY_VEL_MAX` (12 m/s).

### Strip limits

North limits at each strip east position are clipped to the buffered polygon.  
Helper: `_strip_limits(east_m)` intersects `x = east_m` with all four buffered edges and
returns `(south_m, north_m)`.

| Strip | East (m) | S end (m N) | N end (m N) | Direction | Height |
|-------|----------|-------------|-------------|-----------|--------|
| E  | −545  | 210 | 517 | S→N | 307 m — partial (NE wedge) |
| 1  | −695  | 8   | 545 | N→S | 537 m — full |
| 2  | −845  | 36  | 573 | S→N | 537 m — full |
| 3  | −995  | 65  | 601 | N→S | 536 m — full |
| 4  | −1145 | 93  | 629 | S→N | 536 m — full |
| W  | −1250 | 113 | 408 | N→S | 295 m — partial (SW wedge) |

---

## Ordered Waypoint Sequence

All coordinates: `(north_m, east_m, 65.0)` — relative to home, metres.

```
HOME    (0, 0)              takeoff to 65 m AGL, fly at 12 m/s

ENTRY:  (210,  −545)        south end of strip E
WP01:   (517,  −545)        north end of strip E
WP02:   (545,  −695)        north end of strip 1  (transition NW)
WP03:   (8,    −695)        south end of strip 1
WP04:   (36,   −845)        south end of strip 2  (transition SW)
WP05:   (573,  −845)        north end of strip 2
WP06:   (601,  −995)        north end of strip 3  (transition NW)
WP07:   (65,   −995)        south end of strip 3
WP08:   (93,   −1145)       south end of strip 4  (transition SW)
WP09:   (629,  −1145)       north end of strip 4
WP10:   (408,  −1250)       north end of strip W  (transition SW)
WP11:   (113,  −1250)       south end of strip W

HOME    (0, 0)              RTL / land
```

### Reference lat/lon for each waypoint

| WP | North (m) | East (m) | Lat | Lon |
|----|-----------|----------|-----|-----|
| ENTRY | 210 | −545 | 23.452755 | 120.280799 |
| WP01  | 517 | −545 | 23.455512 | 120.280799 |
| WP02  | 545 | −695 | 23.455763 | 120.279330 |
| WP03  | 8   | −695 | 23.450940 | 120.279330 |
| WP04  | 36  | −845 | 23.451191 | 120.277862 |
| WP05  | 573 | −845 | 23.456015 | 120.277862 |
| WP06  | 601 | −995 | 23.456266 | 120.276393 |
| WP07  | 65  | −995 | 23.451452 | 120.276393 |
| WP08  | 93  | −1145 | 23.451703 | 120.274925 |
| WP09  | 629 | −1145 | 23.456518 | 120.274925 |
| WP10  | 408 | −1250 | 23.454532 | 120.273896 |
| WP11  | 113 | −1250 | 23.451883 | 120.273896 |

---

## Detection Response

### Trigger

Subscribe to `/yolo/detections` (`vision_msgs/Detection2DArray`).  
Trigger on any detection whose:
- canonical label is `car`, `van`, `truck`, or `bus`
- computed ground position is inside the buffered polygon

### Ground position from bounding box

```python
GSD_x = 2 * AGL * tan(radians(HFOV / 2)) / CAM_W   # ≈ 0.1226 m/px at 65 m
GSD_y = 2 * AGL * tan(radians(VFOV / 2)) / CAM_H   # ≈ 0.1082 m/px at 65 m

Δeast  =  (bbox_cx − CAM_W / 2) * GSD_x
Δnorth = −(bbox_cy − CAM_H / 2) * GSD_y   # pixel Y down = south

obj_north = cur_north + Δnorth
obj_east  = cur_east  + Δeast
```

### Divert procedure

1. Save `(resume_north, resume_east)` = current strip waypoint target + index into route.
2. Publish `SurveyState = DIVERT`.
3. Fly to `(obj_north, obj_east)` at 12 m/s; wait for horiz_err < 10 m.
4. Log detection to `detections.csv`:
   ```
   timestamp, category, confidence, lat, lon, agl_m
   ```
   where lat/lon derived from `(cur_north, cur_east)` after centering.
5. If multiple vehicles detected in one frame, log all but fly to highest-confidence only.
6. Set `SurveyState = SURVEY` and resume from saved waypoint index.

### Boundary guard

Before issuing any setpoint (survey or divert):

```python
def _in_buffered_zone(north_m, east_m):
    """Point-in-polygon test against buffered boundary vertices."""
    # NW'(642,−1215)  NE'(507,−489)  SE'(−13,−587)  SW'(121,−1293)
    # Uses crossing number algorithm
    ...
```

If a computed divert target lies outside the zone, log the detection at the current
position and skip the divert flight.

---

## Code Changes Required (`control/px4_commander.py`)

### 1. Constants

```python
SURVEY_SPEED   = 12.0    # m/s — strip cruise speed
DETECT_RADIUS  = 10.0    # m — centering arrival threshold
SURVEY_WPS = [           # (north_m, east_m, agl_m)
    (210.0,   -545.0,  65.0),  # ENTRY: south end strip E
    (517.0,   -545.0,  65.0),  # WP01: north end strip E
    (545.0,   -695.0,  65.0),  # WP02: north end strip 1
    (8.0,     -695.0,  65.0),  # WP03: south end strip 1
    (36.0,    -845.0,  65.0),  # WP04: south end strip 2
    (573.0,   -845.0,  65.0),  # WP05: north end strip 2
    (601.0,   -995.0,  65.0),  # WP06: north end strip 3
    (65.0,    -995.0,  65.0),  # WP07: south end strip 3
    (93.0,   -1145.0,  65.0),  # WP08: south end strip 4
    (629.0,  -1145.0,  65.0),  # WP09: north end strip 4
    (408.0,  -1250.0,  65.0),  # WP10: north end strip W
    (113.0,  -1250.0,  65.0),  # WP11: south end strip W
]
```

### 2. State machine

```python
class SurveyState(Enum):
    SURVEY = "survey"
    DIVERT = "divert"
    RESUME = "resume"
```

### 3. YOLO subscription

```python
from vision_msgs.msg import Detection2DArray

self._survey_state  = SurveyState.SURVEY
self._resume_idx    = 0
self._resume_target = None
self.create_subscription(Detection2DArray, "/yolo/detections",
                         self._cb_detections, 1)
```

### 4. `_cb_detections(msg)`

```python
def _cb_detections(self, msg):
    if self._survey_state != SurveyState.SURVEY:
        return   # already diverting
    vehicles = [d for d in msg.detections
                if d.results[0].hypothesis.class_id in ("car","van","truck","bus")]
    if not vehicles:
        return
    best = max(vehicles, key=lambda d: d.results[0].hypothesis.score)
    cx = best.bbox.center.position.x
    cy = best.bbox.center.position.y
    agl = self._drone_agl
    gsd_x = 2 * agl * math.tan(math.radians(44.0)) / 1024
    gsd_y = 2 * agl * math.tan(math.radians(32.55)) / 768
    dn = -(cy - 384) * gsd_y
    de =  (cx - 512) * gsd_x
    obj_n = self._cur_north + dn
    obj_e = self._cur_east  + de
    if not _in_buffered_zone(obj_n, obj_e):
        return
    self._divert_target = (obj_n, obj_e)
    self._divert_cat    = best.results[0].hypothesis.class_id
    self._divert_conf   = best.results[0].hypothesis.score
    self._survey_state  = SurveyState.DIVERT
```

### 5. `_in_buffered_zone(north_m, east_m)` helper

```python
BUFFERED_VERTS = [
    (642, -1215),  # NW'
    (507,  -489),  # NE'
    (-13,  -587),  # SE'
    (121, -1293),  # SW'
]

def _in_buffered_zone(north_m, east_m):
    verts = BUFFERED_VERTS
    n = len(verts)
    inside = False
    j = n - 1
    for i in range(n):
        ni, ei = verts[i]
        nj, ej = verts[j]
        if ((ei > east_m) != (ej > east_m)) and \
           (north_m < (nj - ni) * (east_m - ei) / (ej - ei) + ni):
            inside = not inside
        j = i
    return inside
```

### 6. `_strip_limits(east_m)` helper

```python
def _strip_limits(east_m):
    """Return (south_m, north_m) of the buffered polygon at given east."""
    # Intersect vertical line x=east_m with each edge and collect crossings.
    edges = [
        ((642,-1215),(507,-489)),    # northern
        ((507,-489), (-13,-587)),    # eastern
        ((-13,-587), (121,-1293)),   # southern
        ((121,-1293),(642,-1215)),   # western
    ]
    crossings = []
    for (n1,e1),(n2,e2) in edges:
        if (e1 <= east_m < e2) or (e2 <= east_m < e1):
            t = (east_m - e1) / (e2 - e1)
            crossings.append(n1 + t * (n2 - n1))
    if len(crossings) < 2:
        return None
    return min(crossings), max(crossings)
```

### 7. Detection log

```python
DET_LOG = "detections.csv"

# On first detection (or startup):
with open(DET_LOG, "w") as f:
    f.write("timestamp,category,confidence,lat,lon,agl_m\n")

# On each detection after centering:
lat = HOME_LAT + cur_north / M_PER_DEG
lon = HOME_LON + cur_east  / (M_PER_DEG * COS_LAT)
with open(DET_LOG, "a") as f:
    f.write(f"{time.time():.3f},{category},{conf:.3f},{lat:.6f},{lon:.6f},{agl:.1f}\n")
```

---

## Flight Time Budget

| Segment | Distance | Time (12 m/s) |
|---------|----------|----------------|
| Home → ENTRY | 584 m | 48.7 s |
| Strip E | 307 m | 25.6 s |
| Transitions (×5) | 5 × 153 m = 765 m | 63.8 s |
| Strips 1–4 (×4) | 4 × 537 m = 2148 m | 179.0 s |
| Transition to W strip | 245 m | 20.4 s |
| Strip W | 295 m | 24.6 s |
| W → Home | 1255 m | 104.6 s |
| **Total** | **≈ 5444 m** | **≈ 466 s ≈ 7.8 min** |

Detection diversions add ~20–30 s each (10 m approach + log + resume).

---

## Coverage Summary

| Metric | Value |
|--------|-------|
| Buffered zone area | ≈ 0.46 km² |
| Strips | 6 (4 full + 2 partial edge strips) |
| Zone width covered | ≈ 705 m of 810 m (≈ 87%) |
| Strip spacing / footprint | 150 m / 125 m → 25 m gap |
| Along-track: footprint / advance per frame | 83 m / 2.4 m → heavy overlap |
| AnyLoc buffer from boundary | 30 m |
| Estimated flight time (no detections) | ≈ 7.8 min |
