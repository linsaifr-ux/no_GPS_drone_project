# Survey Mission Plan — Detection Zone Lawnmower + Car Detection Response

## Overview

Autonomous lawnmower survey of a defined detection zone west of home. When YOLO detects a
vehicle inside the zone, its ground position is computed via yaw-corrected GSD projection
and logged to `detections.csv`. The drone never diverts — the survey route runs uninterrupted.

| Parameter | Value |
|-----------|-------|
| Cruise AGL | 65 m |
| Cruise speed | 12 m/s |
| Strip orientation | E-W (long axis of zone) |
| Strip spacing | 150 m N-S |
| Strips | 5 (3 full + 2 partial edge strips) |
| Total distance | ≈ 5.9 km |
| Estimated flight time | ≈ 8.2 min |
| AnyLoc error | ~20 m → 30 m inward buffer |
| Camera footprint at 65 m | 125 m × 83 m (HFOV 88°, VFOV 65.1°) |
| Cross-track swath (N-S, ⊥ to flight) | 125 m → 25 m gap between strips |

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

- Strips run E-W (long axis of zone, ~730 m) rather than N-S (~540 m), reducing
  the number of strips and simplifying transitions.
- N-S spacing 150 m, cross-track swath 125 m → 25 m gap between strips.
- 5 strips cover the buffered zone N-S extent (655 m).
- At 12 m/s + 5 fps the drone advances 2.4 m between frames — unchanged.
- 12 m/s is within PX4's default `MPC_XY_VEL_MAX` (12 m/s).
- Enter from east side (closest to home); boustrophedon S→N.

### Strip limits

East limits at each strip N position clipped to the buffered polygon edges.

| Strip | North (m) | E west (m) | E east (m) | Direction | Length |
|-------|-----------|------------|------------|-----------|--------|
| S  |  60  | −972  | −573  | E→W | 399 m — partial (SE boundary) |
| 1  | 210  | −1280 | −545  | W→E | 735 m — full |
| 2  | 360  | −1257 | −517  | E→W | 740 m — full |
| 3  | 510  | −1235 | −505  | W→E | 730 m — full |
| N  | 580  | −1224 | −882  | E→W | 342 m — partial (NW corner) |

---

## Ordered Waypoint Sequence

All coordinates: `(north_m, east_m, 65.0)` — relative to home, metres.

```
HOME    (0, 0)               takeoff to 65 m AGL, fly at 12 m/s

ENTRY:  (60,   −573)         E end strip S                    → fly W
WP01:   (60,   −972)         W end strip S
WP02:   (210, −1280)         W end strip 1  (transition NW)  → fly E
WP03:   (210,  −545)         E end strip 1
WP04:   (360,  −517)         E end strip 2  (transition N)   → fly W
WP05:   (360, −1257)         W end strip 2
WP06:   (510, −1235)         W end strip 3  (transition N)   → fly E
WP07:   (510,  −505)         E end strip 3
WP08:   (580,  −882)         E end strip N  (transition NW)  → fly W
WP09:   (580, −1224)         W end strip N

HOME    (0, 0)               fly home → AUTO.LAND
```

### Reference lat/lon for each waypoint

lat = 23.450868 + N/111320;  lon = 120.286135 + E/(111320 × 0.9175)

| WP | North (m) | East (m) | Lat | Lon |
|----|-----------|----------|-----|-----|
| ENTRY |  60 | −573  | 23.451407 | 120.280528 |
| WP01  |  60 | −972  | 23.451407 | 120.276620 |
| WP02  | 210 | −1280 | 23.452755 | 120.273608 |
| WP03  | 210 | −545  | 23.452755 | 120.280799 |
| WP04  | 360 | −517  | 23.454103 | 120.281073 |
| WP05  | 360 | −1257 | 23.454103 | 120.273902 |
| WP06  | 510 | −1235 | 23.455451 | 120.274117 |
| WP07  | 510 | −505  | 23.455451 | 120.281269 |
| WP08  | 580 | −882  | 23.456080 | 120.277578 |
| WP09  | 580 | −1224 | 23.456080 | 120.274228 |

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
| Home → ENTRY | 576 m | 48.0 s |
| Strip S (partial) | 399 m | 33.3 s |
| WP01 → WP02 (diagonal NW) | 342 m | 28.5 s |
| Strip 1 | 735 m | 61.3 s |
| WP03 → WP04 (short N) | 153 m | 12.8 s |
| Strip 2 | 740 m | 61.7 s |
| WP05 → WP06 (short N) | 152 m | 12.7 s |
| Strip 3 | 730 m | 60.8 s |
| WP07 → WP08 (diagonal NW) | 383 m | 31.9 s |
| Strip N (partial) | 342 m | 28.5 s |
| WP09 → Home | 1354 m | 112.8 s |
| **Total** | **≈ 5906 m** | **≈ 492 s ≈ 8.2 min** |

No detection diversions — cars are logged in-flight; flight time is fixed at ~8.2 min.

---

## Coverage Summary

| Metric | Value |
|--------|-------|
| Buffered zone area | ≈ 0.46 km² |
| Strips | 5 (3 full + 2 partial edge strips) |
| Zone N-S extent covered | ≈ 643 m of 655 m (≈ 98%) |
| Strip spacing / footprint | 150 m / 125 m → 25 m gap |
| Along-track: footprint / advance per frame | 83 m / 2.4 m → heavy overlap |
| AnyLoc buffer from boundary | 30 m |
| Estimated flight time (no detections) | ≈ 8.2 min |
