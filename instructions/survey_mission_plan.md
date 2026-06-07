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
Trigger on any detection whose canonical label is `car`, `van`, `truck`, or `bus`.  
**The survey route is never interrupted.** Car positions are logged in-flight from the
pixel offset alone; the drone continues to the next waypoint without diverting.

### Ground position from bounding box (yaw-corrected)

```python
GSD_x = 2 * AGL * tan(radians(HFOV / 2)) / CAM_W   # ≈ 0.1226 m/px at 65 m
GSD_y = 2 * AGL * tan(radians(VFOV / 2)) / CAM_H   # ≈ 0.1082 m/px at 65 m

# Drone heading from pose quaternion (camera top follows drone nose)
q       = drone.pose.orientation
yaw_enu = atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y² + q.z²))
h       = -yaw_enu                        # NED heading (rad, CW from north)

dx_m =  (bbox_cx − CAM_W / 2) * GSD_x   # drone-right  (+east when h=0)
dy_m = −(bbox_cy − CAM_H / 2) * GSD_y   # drone-forward (+north when h=0)

Δeast  = dx_m * cos(h) + dy_m * sin(h)
Δnorth = −dx_m * sin(h) + dy_m * cos(h)

obj_north = cur_north + Δnorth
obj_east  = cur_east  + Δeast
```

### Log procedure

1. Compute `(obj_north, obj_east)` using the yaw-corrected formula above.
2. Check dedup: skip if within `DEDUP_RADIUS = 30 m` of any already-logged position.
3. Log highest-confidence vehicle to `detections.csv`:
   ```
   timestamp, category, confidence, lat, lon, agl_m
   ```
   where lat/lon derived from `(obj_north, obj_east)`.
4. Append `(obj_north, obj_east)` to `_logged_positions`.

### Deduplication guard

After a vehicle is logged its position `(north_m, east_m)` is appended to
`self._logged_positions`. Any subsequent detection whose estimated ground position
falls within `DEDUP_RADIUS = 30 m` of an already-logged entry is silently discarded.
The 30 m radius covers the ~20 m AnyLoc position uncertainty, preventing the same
physical car from being re-logged on successive passes or frames.

### Boundary visualisation

`_in_buffered_zone()` and `ZONE_VERTS` are kept in `px4_commander.py` for use by
`tools/live_trace.py` (zone polygon overlay). They are no longer used to gate
detection logging.

---

## Implementation (`control/px4_commander.py`)

### Constants

```python
SURVEY_SPEED  = 12.0   # m/s — strip cruise speed
DEDUP_RADIUS  = 30.0   # m — suppress duplicate log within this radius
SURVEY_WPS = [         # (north_m, east_m, agl_m)
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

### YOLO subscription

```python
from vision_msgs.msg import Detection2DArray

self._logged_positions = []   # (north_m, east_m) dedup list
self.create_subscription(Detection2DArray, "/yolo/detections",
                         self._cb_detections, sensor_qos)
```

### `_cb_detections(msg)`

```python
def _cb_detections(self, msg):
    if self._drone is None:
        return
    vehicles = [d for d in msg.detections
                if d.results[0].hypothesis.class_id in VEHICLE_CLASSES]
    if not vehicles:
        return
    ds    = self._drone.pose.position
    cur_n, cur_e = ds.y, ds.x
    agl   = max(1.0, ds.z - HOME_ALT_MSL)
    gsd_x = 2 * agl * math.tan(math.radians(HFOV_DEG / 2)) / CAM_W
    gsd_y = 2 * agl * math.tan(math.radians(VFOV_DEG / 2)) / CAM_H
    best  = max(vehicles, key=lambda d: d.results[0].hypothesis.score)
    cx, cy = best.bbox.center.position.x, best.bbox.center.position.y
    q       = self._drone.pose.orientation
    yaw_enu = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y**2 + q.z**2))
    h = -yaw_enu
    dx_m =  (cx - CAM_W/2) * gsd_x
    dy_m = -(cy - CAM_H/2) * gsd_y
    de = dx_m*math.cos(h) + dy_m*math.sin(h)
    dn = -dx_m*math.sin(h) + dy_m*math.cos(h)
    obj_n, obj_e = cur_n + dn, cur_e + de
    for ln, le in self._logged_positions:
        if math.hypot(obj_n - ln, obj_e - le) < DEDUP_RADIUS:
            return
    self._log_detection(best.results[0].hypothesis.class_id,
                        best.results[0].hypothesis.score,
                        obj_n, obj_e, agl)
```

### Survey loop

```python
wp_idx = 0
while wp_idx < len(SURVEY_WPS):
    wn, we, wagl = SURVEY_WPS[wp_idx]
    reached = cmd.go_to_ned(wn, we, wagl, timeout=WAYPOINT_TIMEOUT,
                            speed=SURVEY_SPEED)
    if reached:
        ...log arrival...
    else:
        print(f"WP {wp_idx+1} TIMEOUT — skipping")
    wp_idx += 1
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

No detection diversions — cars are logged in-flight; flight time is fixed at ~7.8 min.

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
