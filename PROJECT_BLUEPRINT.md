# IHAILOS — AI Context & Architecture Blueprint

> **Purpose:** Single-source context for any AI coding agent (Cursor, Copilot, Claude, Grok, etc.).  
> Read this document once before changing code. Do not reinvent module boundaries, packet names, or safety rules.

| Field | Value |
|--------|--------|
| Project codename | **IHAILOS** |
| Competition domain | TEKNOFEST Sabit Kanat — otonom hedef tespiti + balistik yük bırakma |
| Primary rules file | `docs/architecture.md` |
| Competition spec (Sabit Kanat) | `docs/SARTNAME_SABIT_KANAT_2026.md` |
| Data contracts | `common/data_contracts.py` |
| Entry point | `main.py` (multithreaded) |
| Language / target | Python **3.10** (dev may run newer; CI/target = 3.10) |
| Last blueprint sync | 2026-10-26 |

---

## 1. Executive Summary & Mission

### 1.1 What this project is

IHAILOS is a **fixed-wing autonomous mission stack** for TEKNOFEST-style fixed-wing UAV tasks:

1. Detect a ground target (YOLO / TensorRT path).
2. Project the detection to **WGS84** (geo-referencing).
3. Stabilize the track (**2D Kalman**).
4. Estimate **wind** and compute **ballistic release** + Final Approach Waypoint (FAW).
5. Fly a **finite-state mission** under hard **flight-safety gates**.
6. Actuate release via **MAVLink DO_SET_SERVO** (AUX 9, PWM 2000).
7. Stream JSON telemetry / event photos to a **referee server**, with offline **SQLite/JSON outbox**.

Scenarios and tests use **synthetic or open data only** — no classified real weapon-system claims.

### 1.2 Target hardware

| Layer | Target |
|--------|--------|
| Companion computer | NVIDIA Jetson (YOLO / TensorRT inference) |
| Autopilot | Pixhawk-class + **ArduPlane** (MAVLink) |
| Sensors | Down/forward-oblique camera (~**30°** pitch mount), LiDAR/rangefinder **AGL**, GPS/IMU via FC |
| Actuation | Servo on **AUX / SERVO instance 9**, release PWM **2000 µs** |

### 1.3 Target software stack

| Component | Version / note |
|-----------|----------------|
| OS | **Ubuntu 22.04 LTS** |
| Middleware | **ROS 2 Humble** (system packages; Python app is MAVLink-centric today) |
| Language | **Python 3.10** |
| Autopilot protocol | **MAVLink** via `pymavlink` |
| Core libs | NumPy, SciPy, OpenCV, Ultralytics YOLO, filterpy, Shapely, requests |

Local Windows/Python 3.14 may be used for unit tests; production target remains Ubuntu 22.04 + Python 3.10.

### 1.4 One-line dataflow

```
Camera+FC → vision_engine → estimator → mission_coordinator → MAVLink + comms_gateway
                 ↑______________ TelemetryPacket (ring buffer) ______________↑
```

---

## 2. System Architecture & Module Responsibilities

### 2.1 Four core modules

| # | Package | Responsibility | Key files |
|---|---------|----------------|-----------|
| 1 | `vision_engine/` | OpenCV capture, **UTC timestamp** at grab, Ultralytics YOLO inference, **8-frame debounce**, `VisionPacket` | `detector.py` |
| 2 | `estimator/` | Pinhole + mount → WGS84, wind/ballistics, 2D Kalman → `EstimatorPacket` | `geo_referencing.py`, `ballistics.py`, `kalman_tracker.py` |
| 3 | `mission_coordinator/` | FSM, Shapely geofence, safety gates, GUIDED WP, AUX9 servo | `state_machine.py`, `geofence.py`, `mavlink_interface.py` |
| 4 | `comms_gateway/` | Referee HTTP JSON + annotated photo POST; offline outbox | `referee_client.py` |

Shared types live in **`common/`**. Orchestration: **`main.py`** (5 threads: telemetry, vision, estimator, mission, comms).

### 2.2 Data contracts (`common/data_contracts.py`)

**Do not rename these packets or their fields casually.** All inter-module I/O must use them (frozen dataclasses + `to_dict` / `from_dict` where provided).

| Packet | Producer → Consumer | Role |
|--------|---------------------|------|
| `TelemetryPacket` | FC / `MavlinkInterface` → vision, estimator, mission, referee | Time-synced pose, attitude, speeds, `FlightMode`, battery |
| `VisionPacket` | `VisionDetector` → estimator | Frame id, UTC `timestamp_ns`, detections, camera intrinsics, latency |
| `EstimatorPacket` | estimator fuse → mission FSM | Target WGS84, NED offset, covariance, wind, ballistic solution, confidence |
| `MissionPacket` | FSM → MAVLink intent + referee | `MissionState`, `SafetyGate`, setpoint, `engage_authorized` |

Important nested types: `GeoPointWGS84`, `NEDVector`, `AttitudeRad`, `BoundingBoxXYXY`, `Detection2D`, `CameraIntrinsics`, `WindEstimate`, `BallisticSolution`, `SafetyGate`, `MavlinkSetpoint`.

Enums (authoritative):

- `FlightMode`: includes `AUTO`, `GUIDED`, …  
- `MissionState`: `SEARCH → TRACK_LOCK → APPROACH_PLAN → FINAL_RUN → RELEASE → VERIFY → RTL` + `GO_AROUND`  
- `TargetClass`: e.g. `HSS`, `HOSTILE`, `VEHICLE`, …

### 2.3 Data-flow diagram

```
┌─────────────┐   TelemetryPacket    ┌──────────────────┐
│ ArduPlane   │ ───────────────────► │ mavlink_interface│── ring-buffer
│ (MAVLink)   │ ◄── GUIDED / SERVO ─ │                  │
└─────────────┘                      └────────┬─────────┘
                                              │
┌─────────────┐   VisionPacket               ▼
│ Camera+YOLO │ ──────────────► ┌─────────────────────┐
│ vision_eng. │                 │ estimator (geo+KF+  │
└─────────────┘                 │ wind+ballistics)    │
                                └──────────┬──────────┘
                                           │ EstimatorPacket
                                           ▼
                                ┌─────────────────────┐
                                │ mission_coordinator │
                                │ FSM + geofence+safe │
                                └──────────┬──────────┘
                                           │ MissionPacket
                     ┌─────────────────────┼─────────────────────┐
                     ▼                     ▼                     ▼
              MAVLink cmds          referee_client         local outbox
              (WP / AUX9)           JSON + photo POST      SQLite+JSON
```

### 2.4 Multithreaded runtime (`main.py`)

| Thread | Rate (typical) | Action |
|--------|----------------|--------|
| `telemetry` | ~10 Hz | Synthetic or `spin_once` → bus |
| `vision` | ~5 Hz | `VisionDetector.step` → bus |
| `estimator` | ~10 Hz | `fuse_estimator` (bbox→WGS84→Kalman→wind/ballistics) |
| `mission` | ~10 Hz | `MissionStateMachine.tick` |
| `comms` | ~5 Hz | `push_telemetry` + edge photos + `flush_outbox` |

Flags: `--synthetic` (no camera/FC), `--mavlink`, `--model`, `--camera`, `--referee-url`, `--duration`, …

---

## 3. Features & Capabilities (Implemented)

### 3.1 Mathematical models (working)

| Feature | Location | Notes |
|---------|----------|--------|
| Pinhole \(K^{-1}[u,v,1]\) | `estimator/geo_referencing.py` | OpenCV camera frame → body FRD |
| **30° forward pitch mount** | `DEFAULT_PITCH_MOUNT_DEG = 30` | `Ry(-mount)` depression |
| Body→NED `Rz(yaw)Ry(pitch)Rx(roll)` | same | |
| Ground intersect with **LiDAR AGL** | `intersect_ray_with_ground_ned` | `t = AGL / d_down` |
| NED→WGS84 local tangent | `ned_offset_to_wgs84` | WGS84 metres-per-degree |
| Wind \(V_w = V_g - V_a\) | `estimator/ballistics.py` | Pitot airspeed × heading vs GPS NED |
| Free-fall \(t=\sqrt{2h/g}\) | `freefall_time_s` | \(g=9.80665\) |
| Release Point + **FAW 300 m** behind | `plan_ballistic_engagement` | `FAW_STANDOFF_M = 300` |
| 2D Kalman `[n,e,vn,ve]` | `estimator/kalman_tracker.py` | filterpy; Mahalanobis gate |

**Sanity example (verified):** UAV `(39.920000, 32.850000)`, AGL 100 m, mount 30°, principal pixel, yaw North → target `(39.921560, 32.850000)`, \(N=173.21\) m \(=100/\tan 30°\), \(E=0\).

### 3.2 Safety & flight control

| Rule | Enforcement |
|------|-------------|
| **RC Override** if `flight_mode ∉ {AUTO, GUIDED}` | `MavlinkInterface.is_rc_override_active` / `SafetyGate.rc_override_active` |
| **\|Roll\| < 5°** and **\|Pitch\| < 5°** | `ATTITUDE_LIMIT_DEG = 5`; every FSM tick |
| **AGL ≤ 120 m + fresh AGL measurement** | `TelemetryPacket.agl_altitude_m` from a downward `DISTANCE_SENSOR`; missing/stale/out-of-limit AGL blocks engagement |
| **Fresh telemetry** | `MissionConfig.telemetry_max_age_s`; stale or future-dated packet, position, attitude, or heartbeat timestamps block engagement |
| **Geofence** ops polygon + approach corridor | `MissionGeofence` (Shapely) |
| **`engage_authorized` only if `safety.all_clear`** | `MissionPacket.__post_init__` invariant |
| No servo if unsafe | `RELEASE` → GO_AROUND without AUX9 |
| **GO_AROUND** | Clears plan/corridor; optional re-enter `SEARCH` |

FSM states (canonical):

`SEARCH → TRACK_LOCK → APPROACH_PLAN → FINAL_RUN → RELEASE → VERIFY → RTL`  
(+ `GO_AROUND` from unsafe / fence / track loss)

### 3.3 Hardware & network protocols

| Feature | Detail |
|---------|--------|
| Frame UTC stamp | `utc_timestamp_ns()` at OpenCV `read()` |
| Telemetry sync | `VisionPacket.sync_telemetry_seq` ↔ `TelemetryPacket.sequence` |
| Debounce | **8 consecutive** same-class + IoU frames (`DEFAULT_DEBOUNCE_FRAMES`) |
| GUIDED waypoint | `set_position_target_global_int` |
| Release | Existing prototype path: `MAV_CMD_DO_SET_SERVO`, instance **9**, PWM **2000**; do not use for competition flight until the approved passive-load mechanism and two-load configuration are validated |
| Referee | Continuous JSON telemetry; lock/release **JPEG + bbox** multipart POST |
| Offline buffer | SQLite `outbox` + JSON sidecars + image blobs; `flush_outbox` / background flusher |

---

## 4. Repository Tree (Codebase Map)

```
IHAILOS/
├── PROJECT_BLUEPRINT.md          ← this file (AI context)
├── docs/
│   └── architecture.md           ← short hard rules (source of safety policy)
├── requirements.txt
├── main.py                       ← multithreaded system runner
├── common/
│   ├── __init__.py               ← re-exports contracts
│   └── data_contracts.py         ← ALL inter-module dataclasses / enums
├── vision_engine/
│   ├── __init__.py
│   └── detector.py               ← capture, YOLO, debounce, VisionPacket
├── estimator/
│   ├── __init__.py
│   ├── geo_referencing.py        ← pixel → WGS84
│   ├── ballistics.py             ← wind, release, FAW
│   └── kalman_tracker.py         ← 2D KF track smoother
├── mission_coordinator/
│   ├── __init__.py
│   ├── geofence.py               ← Shapely ops + corridor
│   ├── mavlink_interface.py      ← ring buffer, safety, WP, servo
│   └── state_machine.py          ← FSM + MissionPacket
├── comms_gateway/
│   ├── __init__.py
│   └── referee_client.py         ← HTTP + LocalOutboxQueue
├── tests/
│   ├── test_data_contracts.py
│   ├── test_estimator.py
│   ├── test_mission_coordinator.py
│   ├── test_vision_and_comms.py
│   └── test_math_and_fsm.py      ← geo + wind + geofence + FSM edge
└── data/                         ← runtime outbox (created at run; git-ignore recommended)
    └── outbox/                   ← SQLite + json/ + blobs/
```

---

## 5. Rules of Engagement for AI Agents

These rules are **non-negotiable**. Prefer refusing an unsafe change over shipping a bypass.

### 5.1 Code completeness

1. **Never** leave `TODO`, `pass`, `...`, or stub implementations in production paths.
2. Implement math (geo, ballistic, Kalman) fully with NumPy/SciPy/filterpy — no fake numbers in real functions.
3. Prefer small, verified diffs; keep module boundaries intact.

### 5.2 Data contracts discipline

1. Treat `common/data_contracts.py` as the **API schema**.
2. Do **not** rename packets (`VisionPacket`, `EstimatorPacket`, `MissionPacket`, `TelemetryPacket`) or casually rename fields (`latitude_deg`, `timestamp_ns`, …).
3. New cross-module fields → extend contracts first, then producers/consumers, then tests.
4. Use type hints everywhere; dataclasses (or Pydantic) for structured I/O.

### 5.3 Defensive programming

1. Any function that can fail to produce a result must return `Optional[...]` **and** document when it is `None` (e.g. no detection, ray misses ground).
2. Validate finite numbers (no NaN/Inf) at contract boundaries (see `_require_finite`).
3. Never assume camera, MAVLink, or referee are connected — support injection / synthetic paths (as in `main.py --synthetic`).

### 5.4 Flight safety (never bypass)
1. Every control-cycle decision must re-evaluate:
   - `flight_mode in {AUTO, GUIDED}`
   - `|roll_deg| < 5` and `|pitch_deg| < 5`
   - fresh telemetry and a fresh downward rangefinder AGL measurement at or below 120 m
   - geofence OK when engaging
2. **Never** set `engage_authorized=True` unless `SafetyGate.all_clear`.
3. **Never** fire AUX9 / release servo when RC override is active or attitude/geofence fails.
4. Prefer `GO_AROUND` / abort over “best effort” engage.
5. Relative altitude is not an AGL substitute. Missing, stale, future-dated, or out-of-range AGL data must fail closed.
6. The current mission stack is a prototype, not an approved competition flight system. Hardware failsafe, encrypted RC/telemetry, pilot override, and power-cut checks require separate documented bench/field validation.

### 5.5 Domain & ethics

1. Keep examples **synthetic / open**.
2. Do not invent classified datasets, real combat employment claims, or weaponized guidance beyond this competition-style sim stack.

### 5.6 How to extend safely (checklist)

1. Read this blueprint + `docs/architecture.md`.
2. Touch `data_contracts.py` only if the wire format must change.
3. Add/adjust unit tests under `tests/` with synthetic numbers and explicit tolerances.
4. Run `pytest` before claiming done.
5. If both a fix and an exploit/PoC are requested for any system: provide the fix only.

---

## 6. Development & Test Procedure

### 6.1 Setup

```bash
# Preferred target: Ubuntu 22.04, Python 3.10 venv
python3.10 -m venv .venv
source .venv/bin/activate   # Windows: .venv\Scripts\activate
pip install -r requirements.txt
```

Shapely note: on some newer Python wheels, `shapely>=2.1` may be required; `requirements.txt` allows `<2.3`.

### 6.2 Unit tests

```bash
# Full suite
python -m pytest tests/ -q

# Math / geo / wind / geofence / FSM edge
python -m pytest tests/test_math_and_fsm.py -v

# Module-focused
python -m pytest tests/test_estimator.py tests/test_mission_coordinator.py -q
```

**Expected:** all tests **PASS** (currently on the order of **23** tests). Treat any failure as a blocker for merge.

### 6.3 Synthetic end-to-end (no hardware)

```bash
python main.py --synthetic --duration 8 --debounce 3 --track-lock 2
```

Logs should show UAV/TGT lat/lon and FSM progression. Referee URL default may be offline → outbox grows (expected).

### 6.4 Hardware / SITL outline

1. **SITL ArduPlane** (example): start ArduPilot SITL, expose MAVLink UDP (often `udp:127.0.0.1:14550`).
2. Confirm HEARTBEAT / `GLOBAL_POSITION_INT` / `ATTITUDE` in QGC or `mavlink_interface.connect()`.
3. Run:
   ```bash
   python main.py --mavlink udp:127.0.0.1:14550 --model /path/to/yolov8n.pt --camera 0
   ```
4. Keep aircraft in **GUIDED** or **AUTO** for autonomy; switching to MANUAL/FBWA must trip RC-override path (no release).
5. Geofence vertices must contain the planned FAW → Release → Target corridor.

### 6.5 Test thresholds (acceptance hints)

| Check | Threshold / expectation |
|-------|-------------------------|
| Geo mount 30°, AGL \(h\), principal pixel, level, yaw N | \(N \approx h/\tan 30°\), \(E \approx 0\) |
| FAW–Release separation | ≈ **300 m** (±~2%) |
| Attitude gate | \|roll\|, \|pitch\| **&lt; 5°** |
| Debounce lock | **8** consecutive consistent frames |
| Release servo | Instance **9**, PWM **2000** |
| `engage_authorized` | Requires `safety.all_clear` |
| pytest | **0 failures** |

### 6.6 Suggested next work (not yet required)

- Dedicated `estimator/pipeline.py` if fusion in `main.py` grows.
- ROS 2 message bridges mirroring `data_contracts`.
- TensorRT `.engine` load path on Jetson (Ultralytics already flags `.engine`).
- Persist geofence polygons from mission config files.

---

## 7. Quick Reference for Agents

**When the user says…** → **You should…**

| Intent | Action |
|--------|--------|
| Change a packet field | Edit `common/data_contracts.py` + all producers/consumers + tests |
| Fix geo / LatLon | `estimator/geo_referencing.py` + `tests/test_math_and_fsm.py` |
| Fix wind / release / FAW | `estimator/ballistics.py` |
| Fix track jitter | `estimator/kalman_tracker.py` |
| Fix FSM / safety | `mission_coordinator/state_machine.py` — **do not weaken gates** |
| Fix fence | `mission_coordinator/geofence.py` |
| Fix MAVLink / servo | `mission_coordinator/mavlink_interface.py` |
| Fix camera / YOLO / debounce | `vision_engine/detector.py` |
| Fix referee / offline queue | `comms_gateway/referee_client.py` |
| Wire threads / CLI | `main.py` |

---

## 8. Document maintenance

Update this blueprint when you:

- Add/remove a module or packet,
- Change FSM states or safety limits,
- Change FAW distance, mount angle, debounce count, or servo mapping,
- Change how tests or SITL are run.

**If code and this file disagree, fix the discrepancy immediately** — prefer making code match `docs/architecture.md` safety rules.

---

*End of IHAILOS AI Context & Architecture Blueprint.*
