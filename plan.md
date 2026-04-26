# AWR1843BOOST Radar Navigation Pipeline Plan

## Goal

Build a testable navigation layer on top of the existing AWR1843BOOST UART parser.

The parser already handles:
- UART packet synchronization using the TI magic word
- frame header parsing
- TLV parsing
- detected point decoding
- side-info decoding
- combining point data with SNR/noise data
- basic point filtering and navigation helpers

Do **not** rewrite the parser unless a decoding bug is found.

The new goal is:

```text
decoded radar frame
→ filtered points
→ clusters
→ zone evidence
→ smoothed scores
→ hysteresis decision
→ command lock
→ pygame visualization
→ logs/replay for tuning
```

The first target is not motor control. The first target is a stable live visualization and command estimate.

---

## Core Principle

The radar does not directly give clean physical objects.

It gives sparse detected reflection points:

```text
point = x, y, z, doppler, optional snr_raw, optional noise_raw
```

The software should infer useful obstacle evidence from those points.

The correct mental model:

```text
radar reflection points
→ maybe an obstacle blob
→ maybe a danger zone
→ maybe a navigation command
```

Avoid this unsafe assumption:

```text
no points = definitely free space
```

Use this safer assumption:

```text
repeated points/clusters = likely obstacle evidence
no points = weak evidence of no obstacle, not proof
```

---

## Recommended File Structure

Create new files instead of modifying the parser heavily.

```text
project/
├── mmwave_uart.py          # existing parser, mostly unchanged
├── radar_nav/
│   ├── __init__.py
│   ├── config.py                  # NavConfig dataclass
│   ├── models.py                  # RadarCluster, NavState, NavOutput
│   ├── filtering.py               # point filtering
│   ├── clustering.py              # DBSCAN/simple clustering
│   ├── decision.py                # smoothing, hysteresis, command logic
│   ├── logging.py                 # JSONL logger
│   ├── replay.py                  # replay log loader
│   └── pygame_viz.py              # pygame visualization
├── run_nav_live.py                # live UART + visualization
├── run_nav_replay.py              # replay from logged frames
└── tests/
    ├── test_clustering.py
    ├── test_decision.py
    └── test_synthetic_scenarios.py
```

If the coding model wants fewer files initially, it can start with:

```text
radar_nav_pipeline.py
radar_nav_pygame.py
run_nav_live.py
run_nav_replay.py
```

Then split later.

---

## Existing Parser Interface

Assume the existing parser exposes:

```python
parser = MmwaveUartParser(data_port="COM5")
decoded = parser.read_decoded_frame(nav_config=None)
```

The returned decoded frame should contain:

```python
decoded["header"]
decoded["combined_points"]
decoded["navigation"]
```

Each point in `combined_points` should look like:

```python
{
    "x": float,
    "y": float,
    "z": float,
    "doppler": float,
    "snr_raw": int,       # if side info exists
    "noise_raw": int      # if side info exists
}
```

The new pipeline should use `combined_points`.

---

## Coordinate Convention

Use a 2D top-down navigation model.

```text
y = forward distance from radar
x = lateral position
x < 0 = left
x > 0 = right
```

Top-down view:

```text
          y forward
             ↑

    left     |     right
             |
             |
           radar
```

For v1, ignore `z` except maybe for debugging display.

---

## Config

Create a config dataclass.

```python
from dataclasses import dataclass

@dataclass
class NavConfig:
    # Point filtering
    min_y: float = 0.15
    max_y: float = 2.5
    lateral_limit: float = 1.2
    min_snr_raw: int | None = 120

    # Clustering
    cluster_eps_m: float = 0.35
    cluster_min_points: int = 2
    keep_singletons: bool = True

    # Zones
    front_half_width: float = 0.35
    left_right_deadband: float = 0.10

    # Evidence/danger scoring
    danger_near_y: float = 0.25
    danger_far_y: float = 2.5
    singleton_weight: float = 0.25
    cluster_weight: float = 1.0

    # Smoothing
    alpha: float = 0.15

    # Hysteresis
    front_on_thresh: float = 0.70
    front_off_thresh: float = 0.40
    side_margin: float = 0.15

    # Command lock
    command_lock_s: float = 0.35
    emergency_stop_thresh: float = 0.90

    # Visualization bounds
    viz_x_min: float = -1.5
    viz_x_max: float = 1.5
    viz_y_min: float = 0.0
    viz_y_max: float = 3.0
```

Make all important values tunable from command-line args later.

---

## Data Models

Create these dataclasses.

```python
from dataclasses import dataclass, field
from typing import Literal

Zone = Literal["left", "front", "right", "unknown"]
Command = Literal["forward", "turn_left", "turn_right", "stop"]

@dataclass
class RadarCluster:
    points: list[dict]
    cx: float
    cy: float
    cz: float
    mean_doppler: float
    mean_snr_raw: float | None
    count: int
    is_singleton: bool
    confidence: float
    zone: Zone

@dataclass
class NavState:
    left_score: float = 0.0
    front_score: float = 0.0
    right_score: float = 0.0

    front_blocked: bool = False

    command: Command = "stop"
    last_command_time: float = 0.0

@dataclass
class NavOutput:
    timestamp: float
    frame_number: int | None

    raw_points: list[dict]
    filtered_points: list[dict]
    clusters: list[RadarCluster]

    current_left: float
    current_front: float
    current_right: float

    left_score: float
    front_score: float
    right_score: float

    front_blocked: bool
    command: Command
```

---

## Stage 1: Point Filtering

Implement:

```python
def filter_points(points: list[dict], cfg: NavConfig) -> list[dict]:
    ...
```

Rules:

```text
keep point if:
- cfg.min_y <= y <= cfg.max_y
- abs(x) <= cfg.lateral_limit
- snr_raw >= cfg.min_snr_raw, if snr_raw exists and min_snr_raw is not None
```

Do not filter by Doppler yet.

Reason:
- Static objects may have near-zero Doppler.
- For basic obstacle avoidance, stationary obstacles matter.

Possible optional future filters:
- remove points below/above certain `z`
- remove impossible high-speed Doppler
- remove points with very bad noise/SNR ratio

---

## Stage 2: Clustering

Implement DBSCAN-like clustering in pure Python first.

Do not require scikit-learn for v1.

Function:

```python
def cluster_points(points: list[dict], cfg: NavConfig) -> list[RadarCluster]:
    ...
```

Distance metric:

```python
distance = sqrt((x1 - x2)**2 + (y1 - y2)**2)
```

Ignore `z` for v1.

Parameters:
- `eps = cfg.cluster_eps_m`
- `min_points = cfg.cluster_min_points`

Important:
The radar often gives only a handful of points. Do **not** throw away all single points.

Use this rule:

```text
cluster with >= min_points points → normal cluster
unclustered point → singleton weak cluster if cfg.keep_singletons is True
```

So the output includes:
- real clusters
- optional singleton clusters with lower confidence

### Simple DBSCAN Logic

Pseudocode:

```python
visited = set()
assigned = set()
clusters = []

for each point i:
    if i visited:
        continue

    mark visited

    neighbors = region_query(i)

    if len(neighbors) < min_points:
        mark as noise candidate
    else:
        grow cluster from i and neighbors

after DBSCAN:
    if keep_singletons:
        convert unassigned points into singleton clusters
```

### Cluster Centroid

For each cluster:

```python
cx = mean(p["x"] for p in cluster_points)
cy = mean(p["y"] for p in cluster_points)
cz = mean(p["z"] for p in cluster_points)
mean_doppler = mean(p["doppler"] for p in cluster_points)
mean_snr_raw = mean(p["snr_raw"] for p in cluster_points if exists)
```

---

## Stage 3: Cluster Confidence

Implement:

```python
def compute_cluster_confidence(cluster_points: list[dict], cfg: NavConfig, is_singleton: bool) -> float:
    ...
```

Suggested v1 formula:

```python
distance_weight = 1.0 / max(cy, 0.3)
distance_weight = clamp(distance_weight, 0.0, 1.0)

count_weight = min(count / 3.0, 1.0)

if mean_snr_raw is available:
    snr_weight = (mean_snr_raw - 80) / 220
    snr_weight = clamp(snr_weight, 0.2, 1.0)
else:
    snr_weight = 0.6

base_weight = cfg.singleton_weight if is_singleton else cfg.cluster_weight

confidence = base_weight * distance_weight * count_weight * snr_weight
confidence = clamp(confidence, 0.0, 1.0)
```

Notes:
- Closer clusters should matter more.
- Multi-point clusters should matter more.
- Singletons should still count, but weakly.
- Stronger SNR should help, but not dominate.

This scoring will require tuning.

---

## Stage 4: Assign Cluster to Zone

Implement:

```python
def cluster_zone(cx: float, cy: float, cfg: NavConfig) -> Zone:
    ...
```

Basic rule:

```python
if abs(cx) <= cfg.front_half_width:
    return "front"
elif cx < -cfg.left_right_deadband:
    return "left"
elif cx > cfg.left_right_deadband:
    return "right"
else:
    return "unknown"
```

Potential issue:
`front_half_width` and `left_right_deadband` overlap conceptually.

For v1, front gets priority:
- if within front corridor, it is front
- otherwise left/right

This makes sense for obstacle avoidance because anything near the robot centerline is most dangerous.

---

## Stage 5: Convert Clusters to Frame Evidence

Implement:

```python
def clusters_to_evidence(clusters: list[RadarCluster], cfg: NavConfig) -> tuple[float, float, float]:
    ...
```

Initialize:

```python
current_left = 0.0
current_front = 0.0
current_right = 0.0
```

For each cluster:

```python
if cluster.zone == "front":
    current_front += cluster.confidence
elif cluster.zone == "left":
    current_left += cluster.confidence
elif cluster.zone == "right":
    current_right += cluster.confidence
```

Clamp:

```python
current_left = clamp(current_left, 0.0, 1.0)
current_front = clamp(current_front, 0.0, 1.0)
current_right = clamp(current_right, 0.0, 1.0)
```

This gives per-frame evidence.

---

## Stage 6: Smoothing

Implement exponential moving average.

```python
def update_scores(state: NavState, current_left: float, current_front: float, current_right: float, cfg: NavConfig) -> None:
    a = cfg.alpha
    state.left_score = (1 - a) * state.left_score + a * current_left
    state.front_score = (1 - a) * state.front_score + a * current_front
    state.right_score = (1 - a) * state.right_score + a * current_right
```

Meaning:
- `alpha = 0.15` means 15% new evidence, 85% previous belief
- lower alpha = smoother but slower
- higher alpha = faster but more jittery

Start with:
- `alpha = 0.15`

Tune later.

---

## Stage 7: Hysteresis

Implement:

```python
def update_front_blocked(state: NavState, cfg: NavConfig) -> None:
    if state.front_score > cfg.front_on_thresh:
        state.front_blocked = True
    elif state.front_score < cfg.front_off_thresh:
        state.front_blocked = False
```

Initial values:

```text
front_on_thresh = 0.70
front_off_thresh = 0.40
```

Meaning:
- score must rise above 0.70 to declare blocked
- score must fall below 0.40 to declare clear
- between 0.40 and 0.70, keep previous state

This prevents rapid toggling near one threshold.

---

## Stage 8: Command Logic

Implement:

```python
def choose_desired_command(state: NavState, cfg: NavConfig) -> Command:
    ...
```

Logic:

```python
if state.front_score > cfg.emergency_stop_thresh:
    return "stop"

if state.front_blocked:
    if state.left_score < state.right_score - cfg.side_margin:
        return "turn_left"
    elif state.right_score < state.left_score - cfg.side_margin:
        return "turn_right"
    else:
        if state.command in ("turn_left", "turn_right"):
            return state.command
        return "stop"

return "forward"
```

Interpretation:
- If front is blocked, turn toward the side with lower danger.
- If sides are almost equal, do not twitch.
- If already turning, continue that turn.
- If no clear turn preference, stop.

---

## Stage 9: Command Lock

Implement:

```python
def apply_command_lock(state: NavState, desired: Command, now: float, cfg: NavConfig) -> Command:
    ...
```

Logic:

```python
if desired == "stop" and state.front_score > cfg.emergency_stop_thresh:
    accept immediately

elif now - state.last_command_time < cfg.command_lock_s:
    keep previous command

else:
    accept desired
```

If accepted command differs from current:

```python
state.command = desired
state.last_command_time = now
```

Start with:

```text
command_lock_s = 0.35
```

This avoids:

```text
turn_left, turn_right, turn_left, turn_right
```

---

## Stage 10: Pipeline Class

Create a class:

```python
class RadarNavPipeline:
    def __init__(self, cfg: NavConfig):
        self.cfg = cfg
        self.state = NavState()

    def process_frame(self, decoded_frame: dict, now: float | None = None) -> NavOutput:
        ...
```

Steps inside `process_frame`:

```python
now = time.time() if now is None else now

raw_points = decoded_frame["combined_points"]
frame_number = decoded_frame["header"].get("frame_number")

filtered_points = filter_points(raw_points, cfg)
clusters = cluster_points(filtered_points, cfg)

current_left, current_front, current_right = clusters_to_evidence(clusters, cfg)

update_scores(state, current_left, current_front, current_right, cfg)
update_front_blocked(state, cfg)

desired = choose_desired_command(state, cfg)
command = apply_command_lock(state, desired, now, cfg)

return NavOutput(...)
```

---

## Stage 11: Pygame Visualization

Use pygame for live visualization.

Create:

```python
class RadarPygameViz:
    def __init__(self, cfg: NavConfig, width=1000, height=700):
        ...

    def handle_events(self) -> bool:
        # returns False when user closes window
        ...

    def draw(self, output: NavOutput) -> None:
        ...
```

### Window Layout

Suggested layout:

```text
+------------------------------------------------------+
| Top-down radar plot                    Score panel   |
|                                                      |
|                                                      |
|                                                      |
|                                                      |
|                                                      |
|                                                      |
| Command/status line                                  |
+------------------------------------------------------+
```

Use left ~70% for radar plot, right ~30% for text/scores.

### Plot Coordinate Mapping

World:
- `x` from `cfg.viz_x_min` to `cfg.viz_x_max`
- `y` from `cfg.viz_y_min` to `cfg.viz_y_max`

Screen:
- x increases right
- y increases down

Mapping:

```python
screen_x = plot_left + (x - x_min) / (x_max - x_min) * plot_width
screen_y = plot_bottom - (y - y_min) / (y_max - y_min) * plot_height
```

Radar origin at:

```text
world x = 0
world y = 0
```

Draw it as a triangle or small circle at bottom center.

### Draw Elements

Draw in this order:

1. background
2. grid lines
3. zone boundaries
4. filtered points
5. raw points maybe faint/gray if desired
6. clusters
7. cluster centroids
8. score bars/text
9. command/status

### Zone Boundaries

Draw vertical lines for:

```text
x = -front_half_width
x = +front_half_width
x = -left_right_deadband
x = +left_right_deadband
```

But avoid clutter. Recommended:
- draw front corridor boundary using two stronger lines
- label left/front/right regions at the top

### Points

Raw points:
- optional, small gray dots

Filtered points:
- small white dots

Cluster centroids:
- larger circles

Singleton clusters:
- small outlined circles

Normal clusters:
- circle around centroid
- radius maybe based on max point distance from centroid, or fixed 0.15–0.25 m

### Cluster Confidence

Represent confidence visually:
- higher confidence = larger centroid circle
- or stronger color/intensity

Possible colors:
- front clusters: red-ish
- left/right clusters: yellow-ish
- filtered points: white
- raw ignored points: dim gray

Exact colors do not matter, but keep high contrast.

### Score Panel

Display:

```text
Frame: 12345
Raw points: 8
Filtered points: 5
Clusters: 2

Left score:  0.12
Front score: 0.78
Right score: 0.34

Front blocked: TRUE
Command: TURN_LEFT
```

Add horizontal bars:

```text
LEFT   [####------] 0.42
FRONT  [########--] 0.81
RIGHT  [##--------] 0.23
```

### Command Display

At bottom:
- large command text
- color-coded background or border

Examples:
- `FORWARD`
- `TURN_LEFT`
- `TURN_RIGHT`
- `STOP`

### FPS / Timing

Display:
- FPS
- frame number
- time since last frame
- maybe packet drop warning if frame number skips

---

## Stage 12: Keyboard Controls in Pygame

Add useful live tuning keys.

```text
Q / ESC: quit

S: toggle showing singleton clusters
R: reset NavState scores
P: pause/unpause
L: toggle logging

Up/Down: increase/decrease front_on_thresh
Left/Right: adjust alpha
[ / ]: adjust cluster_eps_m
- / =: adjust min_snr_raw
```

Print current config values to terminal when changed.

Optional:
- save config to JSON when pressing `K`
- load config from JSON at startup

---

## Stage 13: Logging

Create JSONL logging.

Each line is one frame.

```json
{
  "timestamp": 1710000000.123,
  "frame_number": 42,
  "raw_points": [
    {"x": 0.1, "y": 0.8, "z": 0.0, "doppler": 0.0, "snr_raw": 180, "noise_raw": 900}
  ],
  "filtered_points": [...],
  "clusters": [
    {
      "cx": 0.1,
      "cy": 0.8,
      "cz": 0.0,
      "count": 2,
      "confidence": 0.65,
      "zone": "front",
      "is_singleton": false
    }
  ],
  "scores": {
    "left": 0.1,
    "front": 0.72,
    "right": 0.2
  },
  "front_blocked": true,
  "command": "turn_left"
}
```

Recommended filename:

```text
logs/radar_nav_YYYYMMDD_HHMMSS.jsonl
```

Important:
- log enough data to replay
- do not log raw binary packets at first unless needed

---

## Stage 14: Replay Mode

Create:

```text
run_nav_replay.py
```

It should:
- read a JSONL log
- reconstruct frames or directly feed stored raw points into the pipeline
- show the same pygame visualization
- optionally run slower/faster than real time

Command example:

```bash
python run_nav_replay.py --log logs/radar_nav_20260426_143000.jsonl
```

Useful controls:
- Space: pause
- Right arrow: step one frame
- 1x / 2x / 0.5x playback speed
- R: reset pipeline state and replay from beginning

Why replay matters:
- tune `eps`, `alpha`, thresholds, and SNR without needing the radar connected
- compare algorithms on the same captured data

---

## Stage 15: Synthetic Tests

Before using live radar, implement synthetic tests.

### Test 1: No Points

Input:
```text
100 frames with no points
```

Expected:
```text
scores remain low or decay low
front_blocked = False
command = forward or stop depending chosen startup behavior
```

### Test 2: Persistent Front Obstacle

Input:
```text
30 frames with a point or cluster at x=0.0, y=0.8
```

Expected:
```text
front_score rises gradually
front_blocked becomes True only after several frames
command becomes stop/turn after threshold
```

### Test 3: Random One-Frame Noise

Input:
```text
random singleton in front every 10 frames
```

Expected:
```text
front_score should not cross front_on_thresh
command should not twitch
```

### Test 4: Object Disappears

Input:
```text
front object for 30 frames, then no points for 30 frames
```

Expected:
```text
front_score decays
front_blocked remains True until score < front_off_thresh
then front_blocked becomes False
```

### Test 5: Left/Right Tie

Input:
```text
similar danger left and right
```

Expected:
```text
command should not alternate rapidly
side_margin should prevent twitching
```

### Test 6: Stronger Right Obstacle

Input:
```text
front blocked
right_score high
left_score low
```

Expected:
```text
turn_left
```

### Test 7: Stronger Left Obstacle

Input:
```text
front blocked
left_score high
right_score low
```

Expected:
```text
turn_right
```

---

## Stage 16: Live Runner

Create:

```text
run_nav_live.py
```

Command example:

```bash
python run_nav_live.py --data-port COM5 --baud 921600
```

Optional if config sending is needed:

```bash
python run_nav_live.py --cfg-port COM6 --cfg-file profile_2d.cfg --data-port COM5
```

Flow:

```python
if cfg_port and cfg_file:
    send_cfg(cfg_port, cfg_file)

parser = MmwaveUartParser(data_port)
pipeline = RadarNavPipeline(nav_cfg)
viz = RadarPygameViz(nav_cfg)

while running:
    decoded = parser.read_decoded_frame()
    if decoded is None:
        continue

    output = pipeline.process_frame(decoded)

    if logging:
        logger.write(output)

    running = viz.handle_events()
    viz.draw(output)
```

Add graceful cleanup:

```python
parser.close()
pygame.quit()
```

---

## Stage 17: Parameter Tuning Procedure

Use this physical test process.

### Setup

1. Mount the radar rigidly.
2. Do not hand-hold it.
3. Put robot/radar at fixed height.
4. Use a simple scene first:
   - empty area
   - one box/chair/person
   - obstacle directly ahead
   - obstacle left
   - obstacle right

### Tune in This Order

1. **Point filtering**
   - Does the radar see points in expected range?
   - Are obvious garbage points removed?

2. **Cluster epsilon**
   - Too small: one object splits into many clusters
   - Too large: separate objects merge
   - Start around `0.35 m`

3. **Singleton weight**
   - Too low: real sparse obstacles ignored
   - Too high: noise causes jitter
   - Start around `0.25`

4. **Alpha**
   - Too low: slow response
   - Too high: jitter
   - Start around `0.15`

5. **Hysteresis thresholds**
   - If front_blocked never triggers, lower `front_on_thresh`
   - If front_blocked stays true forever, raise `front_off_thresh` or reduce alpha
   - Start `on=0.70`, `off=0.40`

6. **Command lock**
   - If commands twitch, increase lock time
   - If robot reacts too slowly, decrease lock time
   - Start `0.35 s`

---

## Stage 18: Acceptance Criteria

v1 is successful if:

```text
1. Live pygame window shows radar points and clusters.
2. The score panel updates in real time.
3. Persistent front obstacle causes front_score to rise.
4. One-frame noise does not immediately flip command.
5. Hysteresis prevents front_blocked from toggling rapidly.
6. Left/right commands do not alternate rapidly.
7. Logs can be replayed without radar hardware.
8. Parameters can be adjusted and tested from replay.
```

---

## Stage 19: What Not to Build Yet

Do not build these in v1:

```text
- motor control
- full SLAM
- full occupancy grid mapping
- 3D map
- neural network classifier
- camera fusion
- ultrasonic fusion
- complex multi-object tracker
- aggressive free-space clearing
```

Those can come later.

The first milestone is:

```text
stable radar perception visualization
```

The second milestone is:

```text
stable symbolic command estimate
```

Only then connect to robot motors.

---

## Stage 20: Future Upgrades

After v1 works, consider:

### 1. Track Persistence

Instead of only smoothing zone scores, track cluster centroids over time.

```text
cluster at frame t
→ match nearest cluster at frame t+1
→ maintain track ID
→ delete after missing N frames
```

This would help distinguish persistent obstacles from flicker.

### 2. Occupancy-Style Local Danger Grid

Use a small top-down grid, not full mapping.

```text
5 columns × 6 rows
```

Each cell holds a danger value.

This may be useful if the zone method is still too coarse.

### 3. Doppler-Aware Logic

Use Doppler to detect approaching objects.

But be careful:
- static obstacles matter
- robot ego-motion affects Doppler
- sideways motion may not show strong radial velocity

### 4. Close-Range Safety Sensor

Eventually add ultrasonic/ToF/bumper for emergency close obstacle detection.

Radar is useful, but it should not be the only safety layer.

---

## Summary

Build this first:

```text
existing UART parser
→ point filter
→ simple DBSCAN + singleton weak clusters
→ cluster confidence
→ left/front/right evidence
→ exponential smoothing
→ hysteresis
→ command lock
→ pygame top-down visualization
→ JSONL logs
→ replay mode
```

This is the best first architecture because it directly targets the observed issue:

```text
raw radar point decisions are jittery
```

The fix is:

```text
do not react to one frame
react to persistent clustered evidence
```
