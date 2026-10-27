# Autonomous Valet Parking Agent: Technical Reference Manual

CSAI4014 Algorithms for Intelligent Systems and Robotics | B.Tech CSE (AI & ML) | Group G14

This document is the exhaustive engineering reference for the Autonomous Valet Parking (AVP) system. It details the mathematical foundations, kinematics, control laws, sensor topologies, software interfaces, parameter schemas, and failure recovery procedures.

---

## 1. System Architecture & Inter-Layer Contracts

The valet parking agent is structured according to Gat's three-layer hybrid architecture (Declarative, Procedural, Reflexive), running across a synchronous control period of $\Delta t = 50\text{ ms}$ (Webots: $3 \times 16\text{ ms} = 48\text{ ms}$).

```
+---------------------------------------------------------------------------------+
|                                 INFRASTRUCTURE                                  |
|         ISO 23374 Facility Message: lot_config.json (Slot Occupancy, Obstacles)  |
+---------------------------------------+-----------------------------------------+
                                        |
+---------------------------------------v-----------------------------------------+
| DECLARATIVE LAYER (valet/agent.py, valet/slot_selection.py, valet/planning/)    |
| - Frequency: On mission start (~0.2 Hz) or on dynamic events                    |
| - State evaluation: Utility-based slot selector min J(s)                        |
| - Trajectory synthesis: Hybrid A* with continuous Ackermann primitives + funnel |
| - Output: Continuous geometric Path (waypoints, headings, motion directions)    |
+---------------------------------------+-----------------------------------------+
                                        | Path (waypoints, curvature, direction)
+---------------------------------------v-----------------------------------------+
| PROCEDURAL LAYER (valet/control/tracker.py, steering.py, pid.py)                |
| - Frequency: 20 Hz (dt = 0.05 s)                                                |
| - Path tracking: Direction-aware Pure Pursuit or Stanley controller             |
| - Profile generation: Trapezoidal deceleration & lag-compensated braking profile|
| - Output: Proactive steering delta_cmd and reference velocity v_ref             |
+---------------------------------------+-----------------------------------------+
                                        | (delta_cmd, v_ref)
+---------------------------------------v-----------------------------------------+
| REFLEXIVE SUBSUMPTION LAYER (valet/reflex.py)                                   |
| - Frequency: 20 Hz (overrides procedural commands within the same cycle)        |
| - R1: Swept-path Emergency Stop (d_obs < 0.45 m -> v_ref = 0.0)                 |
| - R2: Swept-Footprint Time-To-Collision (TTC) speed regulation                   |
| - R3: Slot Docking centering and rear bumper distance stop                      |
| - R4: Stalled deadlock detection (blocked_time > 4.0 s -> triggers re-plan)    |
| - Output: Safe command (delta_exec, v_exec) -> closed-loop PID acceleration     |
+---------------------------------------+-----------------------------------------+
                                        | Command(steer, accel, v_ref, state, info)
+---------------------------------------v-----------------------------------------+
| ACTUATORS & HARDWARE ABSTRACTION (controllers/valet_parking/valet_parking.py)   |
| - Webots Driver API / Headless Simulator CarModel                               |
+---------------------------------------------------------------------------------+
```

### 1.1 Finite State Machine (FSM) Transition Matrix

The mission executive in `valet/agent.py` transitions across six discrete states:

| Current State | Trigger / Event | Guard Condition | Next State | Executed Action |
|---|---|---|---|---|
| `SELECT` | Mission start or re-selection | Free slots exist; `tries < max_slot_tries` | `PLAN` | Utility ranking computed; target slot assigned. |
| `SELECT` | No viable slots remaining | `len(tried) >= max_slot_tries` or unranked | `FAILED` | Vehicle halts; emergency hazard state. |
| `PLAN` | Hybrid A* search finishes | Path found (`info["ok"] == True`) | `DRIVE` | Tracker initialized with segmented path. |
| `PLAN` | Hybrid A* search fails | Node expansions exhausted (`exp >= budget`) | `SELECT` | Target slot blacklisted in `tried[]`; next slot chosen. |
| `DRIVE` | Waypoint traversal | Final segment; reverse direction; $d_{end} < 5.0\text{ m}$ | `DOCK` | Activates R3 side centering and rear docking rules. |
| `DRIVE` | R4 blocked timeout | Obstacle motionless for $t_{blocked} > 4.0\text{ s}$ | `PLAN` | Injects virtual obstacle OBB; triggers local re-plan. |
| `DRIVE` | Infrastructure update | Target slot occupied by foreign car | `SELECT` | Global costmap rebuilt; target slot re-selected. |
| `DOCK` | R3 rear stop / end of path | Rear sensor $d \le 0.25\text{ m}$ OR path finished | `PARKED` | Speed set to zero; parking brake applied. |
| `DRIVE`/`DOCK` | Mission completion | Tracker done and $\|v\| < 0.05\text{ m/s}$ | `PARKED` | Mission validated against tolerance gates. |

### 1.2 Data Class Definitions & API Contracts

#### Actuator Command Contract (`valet/agent.py`)
```python
@dataclass
class Command:
    steer: float        # Commanded steering angle [rad] (positive = left turn)
    accel: float        # Commanded longitudinal acceleration [m/s^2]
    v_ref: float        # Target speed setpoint for velocity profile [m/s]
    state: str          # Current mission FSM state (SELECT, PLAN, DRIVE, DOCK, PARKED, FAILED)
    info: dict          # Diagnostics telemetry (active rule, d_obs, cross-track error)
```

#### Infrastructure Exchange Format (`worlds/lot_config.json`)
The facility server delivers lot occupancy as structured JSON:
```json
{
  "seed": 3,
  "occupancy": 0.7,
  "start": [3.0, 1.5, 1.5707963267948966],
  "exit_point": [3.0, 34.5],
  "occupied": ["A0", "A2", "A3", "B1", "B4", "C2", "C3", "D0", "D5"],
  "parked": [
    [8.4, 2.75, 1.570796, 4.945, 1.95],
    [14.0, 2.75, 1.570796, 4.945, 1.95]
  ],
  "walls": [
    [19.8, 0.05, 0.0, 39.6, 0.1],
    [19.8, 35.95, 0.0, 39.6, 0.1]
  ]
}
```

---

## 2. Vehicle Kinematics & Actuator Dynamics

### 2.1 Kinematic Bicycle Model
The vehicle pose is parameterized at the center of the rear axle:
$$\mathbf{q} = \begin{bmatrix} x \\ y \\ \psi \end{bmatrix} \in \mathbb{R}^2 \times \mathbb{S}^1$$

The non-holonomic continuous-time equations of motion are:
$$\dot{x} = v \cos \psi$$
$$\dot{y} = v \sin \psi$$
$$\dot{\psi} = \frac{v}{L} \tan \delta$$

where:
- $L = 2.995\text{ m}$ is the vehicle wheelbase.
- $\psi$ is the heading angle relative to the Global East axis ($x$).
- $\delta$ is the front-wheel steering angle ($|\delta| \le \delta_{max} = 0.55\text{ rad} \approx 31.5^\circ$).
- $v$ is the longitudinal speed at the rear axle.

Path curvature $\kappa$ is related to steering angle $\delta$ via:
$$\kappa = \frac{\tan \delta}{L} \implies R_{min} = \frac{L}{\tan(\delta_{max})} = \frac{2.995}{\tan(0.55)} = 4.873\text{ m}$$

### 2.2 First-Order Actuator & Drivetrain Dynamics
Actuation delays are modeled to ensure realistic response between planner commands and physical motion:

1. **Steering Slew Rate Limit:**
   $$\frac{d\delta}{dt} = \text{clip}\left(\frac{\delta_{cmd} - \delta}{\Delta t}, -\dot{\delta}_{max}, \dot{\delta}_{max}\right), \quad \dot{\delta}_{max} = 1.2\text{ rad/s}$$

2. **Drivetrain Time Constant:**
   $$\dot{a} = \frac{a_{cmd} - a}{\tau_{speed}}, \quad \tau_{speed} = 0.45\text{ s}$$
   $$v_{k+1} = \text{clip}\left(v_k + a \Delta t, -v_{rev,max}, v_{fwd,max}\right)$$

3. **Geometric Center Transformation:**
   Because the rear axle is the origin, the geometric center of the body $(x_c, y_c)$ requires an offset:
   $$d_{offset} = \frac{L + d_{front} - d_{rear}}{2} = \frac{2.995 + 0.95 - 1.00}{2} = 1.4725\text{ m}$$
   $$x_c = x + d_{offset} \cos \psi, \quad y_c = y + d_{offset} \sin \psi$$

---

## 3. Deliberative Planning Layer

### 3.1 Hybrid A* Search Formulation
The planner operates in a 3D state space $(x, y, \psi)$ with continuous state transitions and discrete graph exploration.

- **Spatial Discretization:**
  - Position resolution: $\Delta x = \Delta y = 0.5\text{ m}$
  - Heading resolution: $\Delta \psi = \frac{2\pi}{72} = 5^\circ$ ($72$ discrete orientation bins)
- **Motion Primitives ($10$ total):**
  - Steering samples: $5$ linearly spaced angles in $[-\delta_{plan}, \delta_{plan}]$ where $\delta_{plan} = 0.88 \cdot \delta_{max} = 0.484\text{ rad}$ ($88\%$ limit leaves $12\%$ feedback headroom).
  - Directions: $\{-1, +1\}$ (reverse and forward).
  - Arc length: $\Delta s = 1.0\text{ m}$, integrated with sub-steps of $0.25\text{ m}$ via forward Euler.

### 3.2 Trajectory Cost Function
The cumulative path cost $g(n)$ is penalized to discourage erratic maneuvers and excessive gear shifts:
$$g(n) = \sum_{k=1}^N \left( \Delta s_k \cdot C_{dir} + C_{\delta} |\delta_k| + C_{\Delta \delta} |\delta_k - \delta_{k-1}| + C_{gear} \cdot \mathbf{1}_{\{dir_k \ne dir_{k-1}\}} \right)$$

| Cost Term | Symbol | Value | Rationale |
|---|---|---|---|
| Forward distance multiplier | $C_{fwd}$ | $1.0$ | Baseline distance penalty. |
| Reverse distance multiplier | $C_{rev}$ | $1.6$ | Penalizes reversing to prioritize forward aisle navigation. |
| Steering magnitude penalty | $C_{\delta}$ | $0.4$ | Penalizes driving on high tire lock angles. |
| Steering variation penalty | $C_{\Delta \delta}$ | $0.8$ | Prevents chattering/wiggling across consecutive steps. |
| Gear reversal penalty | $C_{gear}$ | $6.0$ | Strongly penalizes multi-point turns (shunts) in aisles. |

### 3.3 Obstacle Heuristic & Goal Funnel
To ensure informed graph expansion around parked vehicle rows:
$$h(x, y) = \max \Big( \text{Euclidean}(x, y, x_g, y_g), \; h_{Dijkstra}(x, y) \Big)$$
where $h_{Dijkstra}$ is computed once per mission by running an 8-connected 2D Dijkstra wave backwards from the goal cell over a 2D occupancy grid inflated by $(w_{vehicle}/2 - 0.1\text{ m})$.

#### Goal Funnel Acceptance Geometry
Instead of requiring the Ackermann lattice to terminate precisely at the final millimetre-level parking node, the expansion connects to a **Goal Funnel** when the state satisfies:
$$\begin{bmatrix} \Delta s_{along} \\ \Delta s_{lat} \end{bmatrix} = \begin{bmatrix} \cos \psi_g & \sin \psi_g \\ -\sin \psi_g & \cos \psi_g \end{bmatrix} \begin{bmatrix} x - x_g \\ y - y_g \end{bmatrix}$$

Acceptance criteria:
1. Longitudinal distance: $2.5\text{ m} \le \Delta s_{along} \le 7.0\text{ m}$ (ensures sufficient straight travel to align wheels).
2. Lateral offset: $|\Delta s_{lat}| \le 0.20\text{ m}$.
3. Heading deviation: $|\text{wrap}(\psi - \psi_g)| \le 6.0^\circ$ ($0.1047\text{ rad}$).
4. Collision-free corridor: Linear interpolation along the reverse axis into the slot is verified against the CostMap.

---

## 4. Procedural Control Layer

### 4.1 Path Tracker & Cusp Sequencing
Paths are split into continuous-direction segments at cusp points where $\text{sgn}(v)$ flips. When approaching a cusp or the end of a segment, the tracker commands deceleration to zero, verifies vehicle standstill ($|v| < 0.05\text{ m/s}$), and increments the segment pointer.

### 4.2 Steering Control Laws

#### Pure Pursuit Formulation (Forward & Reverse)
For a lookahead distance $L_d = \max(L_{d,min}, k_{lookahead} |v|)$:
- In reverse, $L_d \leftarrow \max(1.2\text{ m}, 0.8 L_d)$.
- Heading relative to direction: $\psi_{dir} = \psi$ if forward, $\psi + \pi$ if reverse.
- Lookahead vector angle: $\alpha = \text{wrap}\left(\text{atan2}(y_{target} - y, x_{target} - x) - \psi_{dir}\right)$.
- Commanded steering angle:
  $$\kappa = \frac{2 \sin \alpha}{L_d}, \quad \delta = d \cdot \arctan(L \kappa) \quad (d \in \{+1, -1\})$$

#### Stanley Controller (with Reverse Curvature Feedforward)
- **Forward Mode:** Cross-track error $e$ is measured at the front axle $(x_f, y_f) = (x + L \cos \psi, y + L \sin \psi)$:
  $$\delta = \psi_e + \arctan\left(\frac{-k_e \cdot e}{k_s + |v|}\right)$$
- **Reverse Mode:** Because the rear axle leads in reverse, lateral error is evaluated at the rear axle, and curvature feedforward $\delta_{ff}$ is mandatory to prevent massive steady-state tracking errors on arcs:
  $$\delta_{ff} = -\arctan(L \kappa)$$
  $$\delta = \delta_{ff} - \left( \psi_e + \arctan\left(\frac{-k_e \cdot e}{k_s + |v|}\right) \right)$$

### 4.3 Longitudinal Speed Profile & PID Loop
The target speed $v_{ref}$ combines cruise limits, curvature restrictions, and deceleration constraints:
1. Curvature speed limit: $v_{lat} = \sqrt{\frac{a_{lat,max}}{\kappa_{max}}} = \sqrt{\frac{1.0}{\kappa_{max}}}$.
2. Drivetrain lag compensation:
   $$d_{brake} = \max(d_{end} - 0.03 - 0.5 |v|, 0.0)$$
   $$v_{stop} = \sqrt{2 \cdot a_{decel} \cdot d_{brake}}, \quad a_{decel} = 0.6\text{ m/s}^2$$
   $$v_{ref} = \min(v_{cruise}, v_{lat}, v_{stop})$$

3. **Discrete PID with Derivative-on-Measurement & Anti-Windup Clamping:**
   $$e_k = v_{ref,k} - v_{meas,k}$$
   $$\dot{v}_{meas} = \frac{v_{meas,k} - v_{meas,k-1}}{\Delta t}$$
   $$u_{unsat} = K_p e_k + K_i \left( I_{k-1} + e_k \Delta t \right) - K_d \dot{v}_{meas}$$
   $$u_{sat} = \text{clip}(u_{unsat}, a_{min}, a_{max}) \quad (-3.0\text{ to }+1.5\text{ m/s}^2)$$
   $$\text{Conditional Integration: } I_k = I_{k-1} + e_k \Delta t \iff u_{sat} == u_{unsat} \lor (u_{unsat} > a_{max} \land e_k < 0) \lor (u_{unsat} < a_{min} \land e_k > 0)$$

---

## 5. Reflexive Subsumption Layer

The reflexive layer sits between the procedural tracker and the low-level actuator drivers, inspecting sensor ranges every $50\text{ ms}$.

```
Procedural Command (steer_proc, v_proc)
          |
          v
+-------------------+      Yes
| R3: Docking Active?|------------> Adjust delta via Side Ultrasonic Balance
+---------+---------+               Stop if Rear Gap < 0.25 m
          | No
          v
+-------------------+      Yes
| R1: E-Stop Range? |------------> Force v_exec = 0.0 (Immediate Stop)
|   d < 0.45 m      |
+---------+---------+
          | No
          v
+-------------------+      Yes
| R2: TTC Exceeded? |------------> Throttle speed: v_exec = (s_c - d_stop)/T_safe - v_obs
+---------+---------+
          | No
          v
Pass procedural command unmodified (v_exec = v_proc)
```

### 5.1 Swept-Footprint Contact Distance ($s_c$)
Standard autonomous vehicles cast a straight bounding box ahead of the bumper. During turns, this misses obstacles swept by the vehicle flanks and corners.

The algorithm transforms all 26 range measurements into Cartesian hit points $\mathbf{p}_i = [p_{x,i}, p_{y,i}]^T$ in the rear-axle vehicle frame. For current steering angle $\delta$, the instantaneous path curvature is $\kappa = \frac{\tan \delta}{L}$.
For arc displacement $s \in [0, s_{horizon}]$ sampled at $0.1\text{ m}$:
$$\theta(s) = d \cdot s \cdot \kappa$$
$$\mathbf{p}_{origin}(s) = \begin{cases} \left[ \frac{\sin \theta(s)}{\kappa}, \; \frac{1 - \cos \theta(s)}{\kappa} \right]^T & |\kappa| \ge 10^{-4} \\ [d \cdot s, \; 0]^T & |\kappa| < 10^{-4} \end{cases}$$

Hit points are transformed into the frame of the vehicle swept body at displacement $s$:
$$\begin{bmatrix} l_x \\ l_y \end{bmatrix} = \begin{bmatrix} \cos \theta(s) & \sin \theta(s) \\ -\sin \theta(s) & \cos \theta(s) \end{bmatrix} \Big( \mathbf{p}_i - \mathbf{p}_{origin}(s) \Big)$$

An intrusion occurs if:
$$-d_{rear} - m_{swept} \le l_x \le L + d_{front} + m_{swept} \quad \text{and} \quad |l_y| \le \frac{w}{2} + m_{swept}$$
The contact distance $s_c$ is the smallest arc displacement $s$ that produces an intrusion.

### 5.2 Dynamic Velocity Limitation (R2 Formula)
When an obstacle is detected on the swept path, the permissible approach speed is restricted:
$$v_{allow} = \max\left(0.0, \; \frac{s_c - d_{stop}}{T_{safe}} - v_{obs}\right)$$
where $d_{stop} = 0.45\text{ m}$, $T_{safe} = 2.5\text{ s}$, and $v_{obs}$ is the closing velocity of dynamic obstacles estimated via range rate differentiation:
$$v_{obs} = \text{clip}\left(\frac{s_{prev} - s_c}{\Delta t} - |v|, \; 0.0, \; 2.0\right)\text{ m/s}$$

---

## 6. Sensor Topology & Spatial Mounting Reference

The ego car utilizes 26 Webots `DistanceSensor` devices mounted at an elevation of $z = 0.50\text{ m}$. Coordinates are referenced to the center of the rear axle ($x$ forward, $y$ left, $\theta$ counter-clockwise from vehicle longitudinal centerline).

| Sensor ID | Mounting $x$ [m] | Mounting $y$ [m] | Yaw Angle [rad] | Yaw Angle [deg] | Primary Functional Purpose |
|---|---|---|---|---|---|
| `front_center` | $+3.94$ | $0.00$ | $0.0000$ | $0.0^\circ$ | Front bumper centerline forward collision check |
| `front_left_in` | $+3.92$ | $+0.50$ | $0.0000$ | $0.0^\circ$ | Left headlight corridor clearance |
| `front_right_in` | $+3.92$ | $-0.50$ | $0.0000$ | $0.0^\circ$ | Right headlight corridor clearance |
| `front_left` | $+3.80$ | $+0.85$ | $+0.5236$ | $+30.0^\circ$ | Front-left bumper corner oblique scan |
| `front_right` | $+3.80$ | $-0.85$ | $-0.5236$ | $-30.0^\circ$ | Front-right bumper corner oblique scan |
| `front_left_wide` | $+3.70$ | $+0.92$ | $+1.0472$ | $+60.0^\circ$ | Front-left wing pedestrian sweeping check |
| `front_right_wide` | $+3.70$ | $-0.92$ | $-1.0472$ | $-60.0^\circ$ | Front-right wing pedestrian sweeping check |
| `rear_center` | $-1.00$ | $0.00$ | $+3.1416$ | $+180.0^\circ$ | R3 docking back-wall distance trigger |
| `rear_left_in` | $-0.98$ | $+0.50$ | $+3.1416$ | $+180.0^\circ$ | Rear bumper left corridor clearance |
| `rear_right_in` | $-0.98$ | $-0.50$ | $+3.1416$ | $+180.0^\circ$ | Rear bumper right corridor clearance |
| `rear_left` | $-0.90$ | $+0.85$ | $+2.6180$ | $+150.0^\circ$ | Rear-left corner reverse swing check |
| `rear_right` | $-0.90$ | $-0.85$ | $-2.6180$ | $-150.0^\circ$ | Rear-right corner reverse swing check |
| `rear_left_wide` | $-0.80$ | $+0.92$ | $+2.0944$ | $+120.0^\circ$ | Rear-left flank clearance on reversing turn |
| `rear_right_wide` | $-0.80$ | $-0.92$ | $-2.0944$ | $-120.0^\circ$ | Rear-right flank clearance on reversing turn |
| `side_left_0` | $-0.40$ | $+0.97$ | $+1.5708$ | $+90.0^\circ$ | Left rear overhang flank coverage |
| `side_left_1` | $+0.30$ | $+0.97$ | $+1.5708$ | $+90.0^\circ$ | Left rear passenger door (R3 docking) |
| `side_left_2` | $+1.00$ | $+0.97$ | $+1.5708$ | $+90.0^\circ$ | Left B-pillar lateral distance (R3 docking) |
| `side_left_3` | $+1.70$ | $+0.97$ | $+1.5708$ | $+90.0^\circ$ | Left front door lateral distance (R3 docking) |
| `side_left_4` | $+2.40$ | $+0.97$ | $+1.5708$ | $+90.0^\circ$ | Left front wheel-arch clearance (R3 docking) |
| `side_left_5` | $+3.10$ | $+0.97$ | $+1.5708$ | $+90.0^\circ$ | Left front fender swing collision check |
| `side_right_0` | $-0.40$ | $-0.97$ | $-1.5708$ | $-90.0^\circ$ | Right rear overhang flank coverage |
| `side_right_1` | $+0.30$ | $-0.97$ | $-1.5708$ | $-90.0^\circ$ | Right rear passenger door (R3 docking) |
| `side_right_2` | $+1.00$ | $-0.97$ | $-1.5708$ | $-90.0^\circ$ | Right B-pillar lateral distance (R3 docking) |
| `side_right_3` | $+1.70$ | $-0.97$ | $-1.5708$ | $-90.0^\circ$ | Right front door lateral distance (R3 docking) |
| `side_right_4` | $+2.40$ | $-0.97$ | $-1.5708$ | $-90.0^\circ$ | Right front wheel-arch clearance (R3 docking) |
| `side_right_5` | $+3.10$ | $-0.97$ | $-1.5708$ | $-90.0^\circ$ | Right front fender swing collision check |

---

## 7. State Estimation & Sensor Fusion

The `PoseFilter` fuses wheel odometry and IMU heading with noisy GPS updates. Because heading $\psi$ is directly observed by the IMU, the planar position estimates $x$ and $y$ decouple into identical scalar filters sharing variance $P$:

1. **Prediction Step (Dead Reckoning):**
   $$x_k^- = x_{k-1} + v_{meas} \cdot \Delta t \cos \psi_{imu}$$
   $$y_k^- = y_{k-1} + v_{meas} \cdot \Delta t \sin \psi_{imu}$$
   $$P_k^- = P_{k-1} + Q \cdot \Delta t \quad (Q = \sigma_{drift}^2 = 0.03^2 = 9 \times 10^{-4}\text{ m}^2/\text{s})$$

2. **Innovation & 5-Sigma Mahalanobis Gate:**
   $$\mathbf{i}_k = \begin{bmatrix} z_{gps,x} - x_k^- \\ z_{gps,y} - y_k^- \end{bmatrix}, \quad S_k = P_k^- + R \quad (R = \sigma_{gps}^2 = 0.15^2 = 0.0225\text{ m}^2)$$
   $$\text{Accept update if } \|\mathbf{i}_k\|^2 \le 5.0^2 \cdot S_k = 25 \cdot S_k$$

3. **Correction Step:**
   $$K = \frac{P_k^-}{S_k}$$
   $$\begin{bmatrix} x_k \\ y_k \end{bmatrix} = \begin{bmatrix} x_k^- \\ y_k^- \end{bmatrix} + K \mathbf{i}_k, \quad P_k = (1 - K) P_k^-$$

---

## 8. Declarative Slot Selection Utility Function

The declarative selector optimizes the total round-trip valet mission time and risk:
$$J(s) = D_{route}(s) + w_{exit} D_{exit}(s) + w_{man} M(s)$$
where:
- $D_{route}(s)$: Dijkstra travel distance from the ego entrance pose to the slot mouth.
- $D_{exit}(s)$: Dijkstra travel distance from the slot mouth to the drop-off exit point $[3.0, 34.5]$.
- $w_{exit} = 1.0$: Weight balancing retrieval time against parking time.
- $w_{man} = 4.0$: Penalty equivalent to 4 metres of extra driving per maneuver complication.
- $M(s) \in [0, 4]$: Complexity index:
  $$M(s) = N_{adj} + N_{opp} + N_{wall}$$
  - $N_{adj} \in \{0, 1, 2\}$: Number of occupied immediately adjacent parking bays.
  - $N_{opp} \in \{0, 1\}$: Occupancy status of slot directly across the aisle.
  - $N_{wall} \in \{0, 1\}$: Equals $1$ if slot is the terminal bay in the aisle (restricted turning radius).

---

## 9. Comprehensive Configuration Parameter Dictionary

Every parameter in `valet/config.py` is documented below with its physical dimension and operational tuning bounds:

### 9.1 `VehicleParams`
| Parameter Name | Field Type | Default Value | SI Unit | Description / Tuning Bounds |
|---|---|---|---|---|
| `wheelbase` | `float` | `2.995` | m | Distance between front and rear axle centers ($L$). |
| `width` | `float` | `1.95` | m | Overall body width across mirrors. |
| `front_overhang`| `float` | `0.95` | m | Front axle to tip of front bumper. |
| `rear_overhang` | `float` | `1.00` | m | Rear axle to tip of rear bumper. |
| `max_steer` | `float` | `0.55` | rad | Maximum front wheel articulation angle ($\approx 31.5^\circ$). |
| `max_steer_rate`| `float` | `1.2` | rad/s | Maximum slew rate of steering actuator. |
| `max_speed` | `float` | `3.0` | m/s | Forward speed ceiling ($\approx 10.8\text{ km/h}$). |
| `max_reverse_speed`| `float` | `1.2` | m/s | Reversing speed ceiling ($\approx 4.3\text{ km/h}$). |
| `max_accel` | `float` | `1.5` | m/s² | Maximum commanded forward acceleration. |
| `speed_tau` | `float` | `0.45` | s | Drivetrain first-order lag time constant. |

### 9.2 `PlannerParams`
| Parameter Name | Field Type | Default Value | SI Unit | Description / Tuning Bounds |
|---|---|---|---|---|
| `xy_resolution` | `float` | `0.5` | m | Grid cell size for closed-set hashing. |
| `yaw_bins` | `int` | `72` | count | Number of heading discretization bins ($5.0^\circ$ each). |
| `step` | `float` | `1.0` | m | Primitive arc displacement length. |
| `n_steer` | `int` | `5` | count | Steering angles evaluated per node expansion. |
| `steer_fraction`| `float` | `0.88` | scalar | Primitives use $88\%$ of max lock ($0.484\text{ rad}$) for feedback margin. |
| `reverse_cost` | `float` | `1.6` | scalar | Multiplier on distance traveled in reverse gear. |
| `gear_switch_cost`| `float`| `6.0` | scalar | Additive penalty incurred per forward/reverse cusp. |
| `steer_cost` | `float` | `0.4` | scalar | Penalty on absolute steering lock $|\delta|$. |
| `steer_change_cost`| `float`| `0.8` | scalar | Penalty on steering rate change $|\delta_k - \delta_{k-1}|$. |
| `safety_margin` | `float` | `0.20` | m | Minimum clearance between vehicle footprint and obstacles. |
| `max_expansions`| `int` | `40000`| count | Search budget before declaring planning failure. |
| `funnel_lateral_tol`| `float` | `0.20` | m | Funnel entry maximum cross-track deviation. |
| `funnel_heading_tol`| `float` | `0.1047` | rad | Funnel entry maximum heading deviation ($6.0^\circ$). |
| `funnel_max_dist`| `float` | `7.0` | m | Maximum distance from slot mouth to enter goal funnel. |
| `funnel_min_dist`| `float` | `2.5` | m | Minimum straight reverse run to guarantee vehicle alignment. |

### 9.3 `ControlParams`
| Parameter Name | Field Type | Default Value | SI Unit | Description / Tuning Bounds |
|---|---|---|---|---|
| `dt` | `float` | `0.05` | s | Procedural control loop interval ($20\text{ Hz}$). |
| `lookahead_min` | `float` | `1.6` | m | Pure Pursuit minimum lookahead threshold. |
| `lookahead_gain`| `float` | `0.8` | s | Proportional velocity gain: $L_d = \max(L_{d,min}, k \cdot |v|)$. |
| `stanley_k` | `float` | `1.2` | 1/s | Stanley cross-track error gain. |
| `stanley_soft` | `float` | `0.5` | m/s | Low-speed softening constant ($k_s$). |
| `speed_kp` | `float` | `1.8` | 1/s | Velocity PID proportional gain. |
| `speed_ki` | `float` | `0.6` | 1/s² | Velocity PID integral gain. |
| `speed_kd` | `float` | `0.05`| scalar | Velocity PID derivative gain. |
| `cruise_speed` | `float` | `2.2` | m/s | Nominal driving velocity in main lot aisles ($\approx 7.9\text{ km/h}$). |
| `manoeuvre_speed`| `float`| `0.9` | m/s | Speed cap inside slots, near cusps, and while docking. |
| `goal_pos_tol` | `float` | `0.30` | m | Maximum terminal lateral/longitudinal error for success. |
| `goal_yaw_tol` | `float` | `0.0873` | rad | Maximum terminal heading error ($5.0^\circ$). |

### 9.4 `ReflexParams`
| Parameter Name | Field Type | Default Value | SI Unit | Description / Tuning Bounds |
|---|---|---|---|---|
| `stop_distance` | `float` | `0.45` | m | R1 hard emergency stop safety bubble ($d_{stop}$). |
| `swept_margin` | `float` | `0.20` | m | Footprint expansion margin along the circular steering arc. |
| `ttc_safe` | `float` | `2.5` | s | Time-to-collision horizon for speed attenuation ($T_{safe}$). |
| `fixed_slow_distance`| `float`| `2.5` | m | Baseline corridor safety threshold (for comparison in E3). |
| `dock_rear_gap` | `float` | `0.25` | m | R3 hard docking stop distance to back wall. |
| `centring_gain` | `float` | `0.12` | rad/m | Proportional steering correction from side range difference. |
| `blocked_replan_time`| `float`| `4.0` | s | Stalled duration before invoking R4 dynamic local re-plan. |

---

## 10. CLI Command Reference & Reproduction Matrix

### 10.1 `tools/make_world.py`
Generates the Webots `.wbt` world file and accompanying `lot_config.json`:
```bash
python tools/make_world.py [OPTIONS]
```
| Argument | Type | Default | Choices / Allowed Values | Description |
|---|---|---|---|---|
| `--seed` | `int` | `3` | Any integer | Random seed for slot occupancy and pedestrian placement. |
| `--occupancy` | `float` | `0.7` | `0.0` to `1.0` | Proportion of slots populated with parked vehicles. |
| `--pedestrians` | `int` | `1` | $\ge 0$ | Number of dynamic pedestrians spawned in aisles. |
| `--planner` | `str` | `hybrid` | `hybrid`, `grid` | Path planner passed to vehicle controller args. |
| `--law` | `str` | `pure_pursuit` | `pure_pursuit`, `stanley` | Steering law configured in vehicle controller. |
| `--reflex` | `str` | `ttc` | `none`, `fixed`, `ttc` | Safety reflex layer mode. |
| `--selector` | `str` | `utility` | `utility`, `nearest` | Slot ranking algorithm. |
| `--out` | `str` | `worlds/valet_parking.wbt` | Filepath | Output destination for generated Webots world. |

### 10.2 `sim/run_demo.py`
Executes an interactive single-mission visualization using the headless kinematic twin:
```bash
python sim/run_demo.py [OPTIONS]
```
| Argument | Type | Default | Choices / Allowed Values | Description |
|---|---|---|---|---|
| `--seed` | `int` | `3` | Any integer | Random scenario seed. |
| `--occupancy` | `float` | `0.7` | `0.0` to `1.0` | Initial lot occupancy ratio. |
| `--pedestrians` | `int` | `2` | $\ge 0$ | Number of walking pedestrians. |
| `--steal` | flag | `False` | Presence flag | Simulates target bay being taken by an intruder car. |
| `--planner` | `str` | `hybrid` | `hybrid`, `grid` | Planner mode. |
| `--law` | `str` | `pure_pursuit` | `pure_pursuit`, `stanley` | Steering tracker. |
| `--reflex` | `str` | `ttc` | `none`, `fixed`, `ttc` | Safety rule set. |
| `--out` | `str` | `results/demo.gif` | Filepath | Destination for animated GIF render. |
| `--every` | `int` | `4` | $\ge 1$ | Subsampling frequency (frames per step). |

### 10.3 `experiments/run_experiments.py`
Runs the Monte Carlo benchmark suite across parallel worker pools:
```bash
python experiments/run_experiments.py [--quick] [--n SEEDS]
```
- Benchmark experiments evaluated:
  - **E1:** Planner comparison (`grid` vs `hybrid_nofunnel` vs `hybrid`).
  - **E2:** Steering control (`pure_pursuit` vs `stanley`).
  - **E3:** Reflex layer (`none` vs `fixed` vs `ttc`).
  - **E4:** Slot selection policy (`nearest` vs `utility`).
  - **E5:** Noise resilience ($\sigma_{gps} \in [0.0, 0.30\text{ m}]$, Raw vs Kalman Filter).
  - **E6:** Density stress test (Occupancies: $50\%, 70\%, 85\%, 92\%$).
  - **E7:** Dynamic event resilience (Target bay stolen during approach).

---

## 11. Diagnostic & Failure Recovery Runbook

| Failure Symptom | Probable Root Cause | Verification Command | Permanent Remediation |
|---|---|---|---|
| Vehicle steers opposite to path at first turn in Webots | Angular convention mismatch: Webots Driver uses $+1 = \text{Right}$; ISO 8855 uses $+1 = \text{Left}$. | Inspect console: steering commands are negated. | Add `"--steer-sign=1"` to `controllerArgs` in world file or run controller with `--steer-sign=1`. |
| Immediate R1 E-stop trigger upon spawning | DistanceSensor beam touches vehicle bumper mesh or wheel arch. | Run `sim/run_demo.py` and print raw range dictionary. Look for sensor reading $\le 0.05\text{ m}$. | In `valet/config.py: _sensor_layout()`, shift affected sensor mounting coordinate $3\text{--}5\text{ cm}$ outwards. Re-run `tools/make_world.py`. |
| Vehicle fails to reverse into bay during cruise speed mode | Webots automatic gearbox refuses negative cruise velocities when engaged in Drive. | Check Webots console for `setCruisingSpeed(-v)` warnings. | Launch controller with `"--speed-mode=throttle"`. This enables manual gear selection (`-1` reverse) and PID torque control. |
| Planner exceeds 40,000 expansions in dense lots ($>90\%$) | Goal funnel disabled or search trapped in local minima behind long parked rows. | Check `path.info["expansions"]` and verify `use_funnel=True`. | Ensure `funnel_max_dist=7.0` and `safety_margin=0.20`. Verify that the 2D Dijkstra heuristic grid is populated without isolated disconnects. |
| Severe cross-track oscillations when reversing with Stanley | Curvature feedforward missing or rear-axle direction flip incorrect. | Plot cross-track error $e(t)$ over reverse segment. | Confirm `delta_ff = -atan(L * kappa)` is subtracted in `valet/control/steering.py:108`. Use Pure Pursuit for reverse docking ($L_d \ge 1.2\text{ m}$). |
| Persistent R4 blocked timeouts on moving pedestrians | Pedestrian stopping in front of car without clearing aisle. | Inspect pedestrian coordinates: verify $s$ progress along path loop. | Ensure pedestrian speed $> 0.8\text{ m/s}$. The agent will hold for $4.0\text{ s}$ before routing around the stalled person. |
| Inaccurate Kalman position under heavy multipath noise | Multipath outliers corrupting measurement update. | Inspect raw vs filtered residuals in `results/experiments.csv` (E5). | Verify that the 5-sigma innovation gate in `valet/localisation.py:35` is active (`ix*ix + iy*iy <= 25*S`). |
