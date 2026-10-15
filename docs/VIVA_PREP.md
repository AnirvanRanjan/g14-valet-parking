# Viva preparation: G14 Autonomous Valet Parking Agent

Rubric B3 (10 marks, individual): *"Answers on own part AND team-mates' parts correctly."*
**Everyone should be able to answer every question below**, not only the questions for their own module.
Practise in pairs: one asks, the other answers without notes.

---

## A. Whole-project questions (everyone)

**Q1. Explain the project in 30 seconds.**
A car-like robot in Webots parks itself in a crowded lot. A deliberative layer chooses a slot with
a utility function and plans a drivable route with Hybrid A*. A procedural layer follows the route
forward and in reverse with Pure Pursuit and a PID speed loop. A reflexive layer, which runs every
48 ms, overrides both to stop for pedestrians and to dock in the slot. Over hundreds of random lots
it parks successfully in most missions, while grid A* almost always fails, because a car cannot
follow a grid path.

**Q2. Which syllabus topics did you use?**
The declarative-procedural-reflexive hierarchy, intelligent agents (utility-based), path planning
(A*, Hybrid A*), moving (the Ackermann bicycle model), open/closed-loop control (PID, Pure Pursuit),
time and frequency domain (step response, Bode plot, phase margin), perception (range sensors,
Kalman filter), planning under uncertainty (noise, Monte Carlo) and software architecture.

**Q3. What is new in your work (novelty)?**
1. A goal funnel for Hybrid A*: about 5.5x fewer node expansions, and 100% success against 80% without it.
2. A swept-footprint TTC reflex that catches side-swipes while turning.
3. A utility-based slot choice that optimises the whole valet cycle (drive in + drive out).
4. One agent shared by Webots and a headless twin, which made Monte Carlo evaluation possible,
   with event handling (slot taken) and a Kalman filter.

**Q4. Why reverse in (nose-out)?**
Retrieval is faster and safer, because the car drives out forward and sees the aisle. Also, the
steered front wheels can swing into a narrow slot more easily when reversing. Valet operators and
many car parks require it.

**Q5. Why not just use A* on a grid?**
A* treats the car as a point or disc and gives sharp corners. The car has a minimum turning radius
of 4.87 m and cannot turn 90 degrees into a 2.8 m slot from a 7 m aisle. In E1, grid A* almost never
succeeded: the car either touched a parked car or the reflex stopped it before contact.

**Q6. What is the difference between the Webots run and the "twin"?**
Both run the *same* `valet/` agent code. Webots adds full physics (tyres, suspension) and is used
for the live demo. The twin is a fast kinematic simulator with the same lot, sensors and noise, used
for hundreds of reproducible missions. The Webots world is generated from the same Python lot model,
so the two cannot differ in geometry.

**Q7. How do you define success?**
State PARKED, no contact with anything, all four corners inside the slot (+10 cm), lateral error
within 0.3 m and heading error within 5 degrees.

**Q8. What would you change for a real car?**
Indoor localisation instead of GPS, the car's ultrasonic sensors or a lidar, the planner rewritten
in C++, a dynamic model with tyre slip, and testing on a 1:10 scale car first.

---

## B. Saurabh: lot model, costmap, Webots

* **How does the collision check work?** We rasterise all obstacles into a 0.1 m occupancy grid and
  compute a Euclidean distance transform (distance from each cell to the nearest obstacle). A pose is
  free if every point on the car outline (sampled every 0.2 m) is more than the margin (0.2 m) away.
  That is a few array look-ups per pose.
* **Why not circles (disc decomposition)?** We tried it first. The end discs stick out about 0.7 m past
  the bumpers, so the correct parked pose next to the back wall was rejected.
* **Webots car model?** BmwX5 PROTO from the vehicle library. Its origin is the rear-axle centre, the
  same reference as our bicycle model. Sensors go in `sensorsSlotCenter`. The `Driver` API gives
  `setSteeringAngle` (positive = right in Webots, so we multiply by -1) and `setCruisingSpeed`
  (km/h, so we multiply m/s by 3.6).
* **Why generate the world with a script?** So that the Webots world and the planner's map always use
  the same random scenario (seed). Changing the layout means one command, not hours of editing.
* **What does `lot_config.json` represent?** The message from the parking facility (as in ISO 23374):
  which slots are occupied and the outlines of the parked cars.

## C. Naman: planners

* **Hybrid A* in one line?** A* over (x, y, heading) cells, where each node stores a continuous pose
  reached by driving the car model, so every path can be driven.
* **What are the motion primitives?** 5 steering angles (up to 88% of max) x forward/reverse, each
  1 m long, integrated in 0.25 m steps with the bicycle model.
* **Cost function?** Length (reverse x1.6) + 0.4|delta| + 0.8|delta change| + 6 per gear change.
  It penalises reversing, wiggling and many shunts.
* **Heuristic? Is it admissible?** max(Euclidean, 2-D Dijkstra on the inflated grid). Dijkstra
  accounts for obstacles but ignores heading, so it never overestimates much. With the funnel and
  inflation it is near-admissible, not strictly, which is fine because we want fast and good
  paths, not proven optimal ones.
* **Goal funnel?** We stop when a node is on the slot axis, 2.5-7 m out, within 0.2 m and 6 degrees,
  and a straight reverse into the slot is collision-free. The goal region becomes long and thin, so
  about 5.5x fewer expansions are needed (54 470 -> 9 968 on average). The minimum of 2.5 m guarantees a straight run to settle the heading.
* **Why 88% of the steering limit?** With full lock in the plan, the controller has no spare steering
  to correct errors, and the car ended up 10 degrees crooked.
* **Grid A* details?** 8-connected, octile heuristic (admissible and consistent), no corner cutting,
  obstacles inflated by half the car width plus the margin, then line-of-sight smoothing.

## D. Anirvan: agent, slot selection, localisation

* **Agent type?** A utility-based, model-based agent (Russell & Norvig). It keeps a model (the costmap)
  and chooses the action (slot) with the best utility.
* **PEAS?** Performance: parked, accurate, fast, no contact. Environment: lot, cars, pedestrians.
  Actuators: steering and speed. Sensors: GPS, IMU, wheel speed, 26 range sensors, facility messages.
* **Utility function?** J = D_in + 1.0 x D_out + 4 m x M, where M = occupied neighbours (0-2)
  + occupied opposite slot + end-wall slot. D_in and D_out come from one Dijkstra run each, for all
  slots at once.
* **Why include D_out?** The valet must bring the car back to the pick-up zone, so a slot that is
  slightly further in but much nearer the exit is better for the whole cycle.
* **State machine?** SELECT -> PLAN -> DRIVE -> DOCK -> PARKED. If planning fails, try the next slot.
  If the slot is taken, re-select. If R4 says blocked, re-plan. After 4 failed slots, FAILED.
* **How are the layers connected?** In `agent.step()`: the procedural layer proposes (steer, v_ref),
  then the reflex layer may modify it, then the PID turns v_ref into acceleration. Events (blocked,
  slot taken) go back up to the declarative layer.
* **Kalman filter?** Predict with odometry (v, IMU yaw), P += Q dt. Update with GPS, K = P/(P+R).
  x and y decouple because yaw comes from the IMU. At 0.3 m GPS noise it keeps most missions
  successful, while raw GPS fails most of them.

## E. Aditya: control

* **Pure Pursuit formula?** kappa = 2 sin(alpha) / L_d, delta = atan(L kappa), where alpha is the
  angle to the look-ahead point measured from the direction of motion.
* **What changes in reverse?** The direction of motion is yaw + pi, and d(yaw)/dt = v tan(delta)/L
  changes sign with v, so the steering sign flips.
* **Look-ahead tuning?** Too short makes the steering oscillate; too long cuts corners.
  L_d = max(1.6, 0.8|v|), with a 1.2 m minimum in reverse.
* **Stanley?** delta = heading error + atan(k e / (k_s + v)) at the front axle. Our planner gives a
  rear-axle path, so we shift the path to the front axle. In reverse we use the rear axle plus a
  curvature feed-forward. It tracks well forward, but its final accuracy in reverse is worse than
  Pure Pursuit's.
* **PID speed loop?** Kp 1.8, Ki 0.6, Kd 0.05, output clamped (anti-windup by conditional
  integration), derivative on the measurement (avoids a kick when v_ref steps). Plant
  G(s) = 1/(s(tau s+1)), tau = 0.45 s.
* **Stability?** Bode plot: phase margin around 45 degrees, see the report for the exact value.
  Step: rise under 1 s, with some overshoot, which is acceptable because v_ref is ramped.
* **Why did the car overshoot the stop and how did you fix it?** The drivetrain lag: braking started
  too late. The speed profile now subtracts the distance covered during the lag (0.5|v|).

## F. Priyal: reflex, simulator, experiments

* **Reflex rules in priority order?** R1 emergency stop, R2 TTC speed limit, R3 docking (centring and
  a hard stop on the rear gap), R4 blocked -> re-plan.
* **Swept-footprint contact distance?** Convert all range readings to points in the car frame. Move
  the inflated car outline along the arc of the current steering angle, and find the first arc
  length where a point is inside the outline.
* **TTC limit formula?** v_allow = (s_c - d_stop) / T_safe - v_obstacle, with d_stop 0.45 m and
  T_safe 2.5 s. The speed falls smoothly; there is no stop-go.
* **Why is it better than a fixed distance?** A fixed corridor ahead of the bumper misses what the
  side of a turning car sweeps, and stops for parked cars beside the path that are not in the way.
* **Why 26 sensors?** With only bumper sensors, pedestrians beside the front wing were invisible
  during turns. 7 per bumper (including 30 and 60 degree corners) and 6 per side, spaced 0.7 m, so a
  0.7 m wide person cannot hide between rays.
* **Simulator twin?** Kinematic bicycle model with a steering-rate limit and drivetrain lag,
  vectorised ray casting (slab method) for the sensors, and SAT for exact collision tests. Noise on
  GPS, heading, speed and range.
* **Why Monte Carlo with fixed seeds?** One demo run proves nothing. Random lots show the success
  rate, and fixed seeds make every number reproducible and the comparisons fair (the same lots for
  every method).

---

## G. Tricky questions the faculty may ask

* *"Is your planner optimal?"* Not strictly. Hybrid A* is resolution-limited and our heuristic is
  not strictly admissible. It is complete up to the resolution and gives near-shortest drivable paths.
* *"What if the GPS fails completely?"* The filter keeps predicting with odometry, so the error grows
  slowly; the reflex layer still prevents collisions. A real system would use facility localisation.
* *"Why is planning 2-4 s acceptable?"* The car is stopped at drop-off while planning, and re-plans
  have a 12 000-node budget. In C++ it would take under 0.1 s.
* *"Where does the car's knowledge of parked cars come from?"* The facility message (infrastructure
  sensors), as in Bosch/Mercedes AVP and ISO 23374. The car's own sensors are used for safety and
  docking.
* *"Show me in the code where X happens."* Know your file names and the main function in each, e.g.
  `HybridAStarPlanner.plan`, `_funnel`, `PurePursuit.__call__`, `ReflexLayer.apply`,
  `swept_contact_distance`, `SlotSelector.rank`, `ValetAgent.step`.
