const pptxgen = require("pptxgenjs");
const path = require("path");

// Run from the repo:  npm install pptxgenjs && node slides/build_deck.js
const ROOT = path.join(__dirname, "..");
const FIG = (f) => path.join(ROOT, "results", "figures", f);
const OUT = path.join(ROOT, "slides", "G14_Valet_Parking_Slides.pptx");

const THEME = {
  name: "G14 Asphalt",
  headFontFace: "Calibri",
  bodyFontFace: "Calibri",
  colors: {
    dk1: "1B1D21", lt1: "FFFFFF", dk2: "2B2F36", lt2: "F2F3F5",
    accent1: "F5B700", accent2: "2A78D6", accent3: "EB6834", accent4: "1BAF7A",
    accent5: "6B6A66", accent6: "9AA0A6", hlink: "2A78D6", folHlink: "6250D6",
  },
};
const HEX = { ink: "1B1D21", ink2: "4A4F57", yellow: "F5B700", blue: "2A78D6", orange: "EB6834",
  green: "1BAF7A", light: "F2F3F5", asphalt: "2B2F36", white: "FFFFFF", line: "D5D8DD" };

const pres = new pptxgen();
pres.layout = "LAYOUT_WIDE"; // 13.333 x 7.5
pres.title = "Autonomous Valet Parking Agent (P13, G14)";
pres.author = "Group G14";
pres.theme = { headFontFace: THEME.headFontFace, bodyFontFace: THEME.bodyFontFace };
const C = pres.SchemeColor;
const W = 13.333, H = 7.5;

// ------------------------------------------------------------------ layouts
pres.defineSlideMaster({
  title: "CONTENT",
  background: { color: HEX.white },
  objects: [
    { placeholder: { options: { name: "title", type: "title", x: 0.6, y: 0.35, w: 9.6, h: 0.85,
      fontSize: 30, bold: true, color: HEX.ink, valign: "middle", align: "left", margin: 0 }, text: "" } },
  ],
  slideNumber: { x: 12.45, y: 7.0, w: 0.6, h: 0.3, fontSize: 11, color: HEX.ink2, align: "right" },
});
pres.defineSlideMaster({
  title: "DARK",
  background: { color: HEX.asphalt },
  objects: [],
});

// ------------------------------------------------------------------ helpers
function presenter(slide, name) {
  slide.addShape(pres.shapes.ROUNDED_RECTANGLE, { x: 10.45, y: 0.45, w: 2.45, h: 0.55, rectRadius: 0.12,
    fill: { color: HEX.yellow }, line: { color: HEX.yellow }, objectName: "presenter-tag" });
  slide.addText(name, { x: 10.45, y: 0.45, w: 2.45, h: 0.55, fontSize: 14, bold: true, color: HEX.ink,
    align: "center", valign: "middle", isTextBox: true, margin: 0, objectName: "presenter-name" });
}
function img(slide, file, ratio, x, y, maxW, maxH, align = "center") {
  let w = maxW, h = maxW / ratio;
  if (h > maxH) { h = maxH; w = maxH * ratio; }
  const ox = align === "center" ? x + (maxW - w) / 2 : x;
  slide.addImage({ path: FIG(file), x: ox, y: y + (maxH - h) / 2, w, h, objectName: file });
}
function bullets(slide, items, x, y, w, h, size = 20) {
  slide.addText(items.map((t, i) => {
    const runs = Array.isArray(t) ? t : [{ text: t }];
    return runs.map((r, j) => ({ text: r.text, options: Object.assign({
      bullet: j === 0 ? { indent: 22 } : undefined, breakLine: j === runs.length - 1 && i < items.length - 1,
      paraSpaceAfter: 10, color: HEX.ink }, r.options || {}) }));
  }).flat(), { x, y, w, h, fontSize: size, valign: "top", isTextBox: true, margin: 2 });
}
function content(title, who, section) {
  const s = pres.addSlide({ masterName: "CONTENT", sectionTitle: section });
  s.addText(title, { placeholder: "title" });
  presenter(s, who);
  return s;
}
function statCard(slide, x, y, w, h, big, label, color) {
  slide.addShape(pres.shapes.ROUNDED_RECTANGLE, { x, y, w, h, rectRadius: 0.1, fill: { color: HEX.light },
    line: { color: HEX.light } });
  slide.addText(big, { x: x + 0.2, y: y + 0.15, w: w - 0.4, h: h * 0.45, fontSize: 36, bold: true, color,
    valign: "middle", isTextBox: true, margin: 0 });
  slide.addText(label, { x: x + 0.2, y: y + 0.15 + h * 0.45, w: w - 0.4, h: h * 0.5 - 0.2, fontSize: 18,
    color: HEX.ink, valign: "top", isTextBox: true, margin: 0 });
}
function table(slide, rows, x, y, w, colW, size = 18, rowH = 0.6) {
  const head = rows[0].map((t) => ({ text: t, options: { bold: true, color: HEX.white, fill: { color: HEX.asphalt } } }));
  const body = rows.slice(1).map((r, i) => r.map((t) => ({ text: t, options: {
    color: HEX.ink, fill: { color: i % 2 ? HEX.white : HEX.light } } })));
  slide.addTable([head, ...body], { x, y, w, colW, fontSize: size, border: { type: "solid", pt: 0.75, color: HEX.line },
    margin: [4, 6, 4, 6], valign: "middle", rowH });
}

// ================================================================== 1 TITLE
pres.addSection({ title: "Introduction" });
let s = pres.addSlide({ masterName: "DARK", sectionTitle: "Introduction" });
s.addText("CSAI4014 Algorithms for Intelligent Systems and Robotics  |  B.Tech CSE (AI & ML), Sem VII", {
  x: 0.7, y: 0.5, w: 11.5, h: 0.4, fontSize: 16, color: "C9CDD3", isTextBox: true, margin: 0 });
s.addText("Autonomous Valet Parking Agent", { x: 0.7, y: 1.15, w: 11.5, h: 1.0, fontSize: 44, bold: true,
  color: HEX.white, isTextBox: true, margin: 0 });
s.addText("Hybrid A* planning  ·  Pure Pursuit + PID  ·  reflexive safety layer  ·  Webots", {
  x: 0.7, y: 2.1, w: 11.5, h: 0.5, fontSize: 22, color: HEX.yellow, isTextBox: true, margin: 0 });
s.addText([
  { text: "Project P13   |   Group G14   |   Batch B2", options: { breakLine: true, bold: true } },
  { text: "Faculty: Course Faculty, CSAI4014, School of Computer Science, UPES" },
], { x: 0.7, y: 2.85, w: 6.0, h: 0.9, fontSize: 18, color: HEX.white, isTextBox: true, margin: 0 });
const members = [["Saurabh Kumar", "500124288"], ["Naman Chanana", "500119485"], ["Anirvan Ranjan", "500119330"],
  ["Aditya Pratap Singh Sisodia", "500121985"], ["Priyal Sharma", "500122041"]];
s.addTable(members.map(([n, id]) => [
  { text: n, options: { color: HEX.white, bold: true } }, { text: id, options: { color: "C9CDD3" } }]),
  { x: 0.7, y: 4.05, w: 6.6, colW: [4.4, 2.2], fontSize: 18, border: { type: "solid", pt: 0.5, color: "4A4F57" },
    margin: [3, 6, 3, 6] });
s.addImage({ path: path.join(ROOT, "results", "demo_final.png"), x: 8.15, y: 2.85, w: 4.45, h: 4.07, objectName: "demo" });
presenter(s, "Saurabh Kumar");
s.addNotes("SAURABH (about 1 min). Good morning. We are group G14 and our project is P13, the Autonomous Valet Parking Agent. " +
  "Each member now says name and SAP ID on camera. Then: our car finds a free slot in a crowded parking lot, plans a drivable route, " +
  "reverses into the slot and stays safe around pedestrians. Everything runs in Webots with Python.");

// ================================================================== 2 PROBLEM
s = content("Problem and motivation", "Saurabh Kumar", "Introduction");
bullets(s, [
  [{ text: "Parking: ", options: { bold: true } }, { text: "slow, stressful, and where most low-speed scrapes happen" }],
  [{ text: "Automated valet parking (AVP): ", options: { bold: true } }, { text: "leave the car at drop-off, it parks itself and comes back to pick-up" }],
  [{ text: "Industry: ", options: { bold: true } }, { text: "Bosch + Mercedes Level-4 AVP, Stuttgart airport (2022); ISO 23374 standard" }],
  [{ text: "Hard because: ", options: { bold: true } }, { text: "a car can't turn on the spot (R_min = 4.87 m), must reverse, and must react to people" }],
], 0.6, 1.5, 6.6, 5.2, 20);
img(s, "fig_lot_layout.png", 1.048, 7.4, 1.3, 5.5, 5.6);
s.addText("Our lot: 48 slots, 2 aisles, 50-92 % full, crossing pedestrians", { x: 7.4, y: 6.9, w: 5.5, h: 0.35,
  fontSize: 12, color: HEX.ink2, align: "center", isTextBox: true, margin: 0 });
s.addNotes("SAURABH (1 min). Explain the real problem and the industry context. Point at the lot: start bottom-left, pick-up top-left, grey boxes are parked cars, orange dots are pedestrians.");

// ================================================================== 3 OBJECTIVES
s = content("Objectives: target vs achieved", "Saurabh Kumar", "Introduction");
table(s, [
  ["#", "Measurable objective", "Target", "Achieved"],
  ["O1", "Success rate, 70 % full lot, zero collisions", "≥ 95 %", "100 %  (30/30)"],
  ["O2", "Final parking accuracy", "< 10 cm, < 2°", "4.2 cm, 1.2°"],
  ["O3", "Planning time in Python", "< 5 s", "1.65 s mean"],
  ["O4", "Pedestrian contacts (4 crossing people)", "0 %", "0 %"],
  ["O5", "Methods compared per layer", "≥ 2", "2 planners, 2 steering laws, 3 reflexes, 2 slot policies"],
], 0.6, 1.5, 12.1, [0.7, 5.2, 2.2, 4.0], 20, 0.8);
s.addNotes("SAURABH (1 min). Read each objective and show that every target was met. Numbers come from 525 simulated missions.");

// ================================================================== 4 EXISTING
pres.addSection({ title: "Background and novelty" });
s = content("Existing solutions and the gap", "Naman Chanana", "Background and novelty");
table(s, [
  ["Approach", "Strength", "Gap"],
  ["Grid A* (Hart et al. 1968)", "Fast, optimal on grid", "Ignores heading: path not drivable"],
  ["Hybrid A* (Dolgov et al. 2010)", "Drivable, obstacle-aware", "Slow to hit a narrow slot pose"],
  ["Pure Pursuit / Stanley", "Simple, proven", "Stanley weak in reverse"],
  ["Park-assist rules", "Cheap ultrasonic stop", "Fixed distance misses side-swipes"],
  ["Industry AVP (Bosch, ISO 23374)", "Certified, Level 4", "Closed source, costly infrastructure"],
], 0.6, 1.5, 12.1, [3.9, 3.6, 4.6], 20, 0.72);
s.addText("Gap we target: fast slot planning + turning-aware safety + choosing the slot for the whole valet cycle",
  { x: 0.6, y: 6.0, w: 12.1, h: 0.75, fontSize: 20, bold: true, color: HEX.ink, fill: { color: "FFF3C4" },
    isTextBox: true, margin: 8, valign: "middle" });
s.addNotes("NAMAN (1.5 min). Walk through the table quickly; stress the three gaps in the yellow box, which our novelty fills.");

// ================================================================== 5 NOVELTY
s = content("Novelty: what is new, with evidence", "Naman Chanana", "Background and novelty");
statCard(s, 0.6, 1.5, 5.9, 2.4, "5.5× fewer", "node expansions with our slot \"goal funnel\" in Hybrid A* (success 80 % → 100 %)", HEX.blue);
statCard(s, 6.8, 1.5, 5.9, 2.4, "0 % contacts", "with the swept-footprint TTC reflex (17 % fixed-distance, 20 % none)", HEX.green);
statCard(s, 0.6, 4.2, 5.9, 2.4, "−7.6 %", "valet cycle length (drive in + out) with the utility-based slot choice", HEX.orange);
statCard(s, 6.8, 4.2, 5.9, 2.4, "100 % vs 20 %", "success at 0.3 m GPS noise: Kalman filter vs raw GPS", HEX.asphalt);
s.addNotes("NAMAN (1.5 min). Four novelty points, each backed by a number from our experiments. Also mention: one agent code base runs both Webots and the headless twin.");

// ================================================================== 6 SYLLABUS
s = content("AISR syllabus mapping", "Anirvan Ranjan", "Background and novelty");
table(s, [
  ["Syllabus topic", "Where we use it"],
  ["Declarative–procedural–reflexive hierarchy", "The three layers of the agent"],
  ["Intelligent agents", "Utility-based slot choice, mission state machine"],
  ["Path planning, planning to move", "Grid A*, Hybrid A* + goal funnel"],
  ["Moving (ground robots)", "Ackermann bicycle model, reversing"],
  ["Closed-loop control, time/frequency domain", "PID speed loop, step response, Bode, phase margin"],
  ["Perception, uncertain movement", "26 range sensors, Kalman filter, Monte Carlo tests"],
], 0.6, 1.5, 12.1, [5.4, 6.7], 20, 0.75);
s.addNotes("ANIRVAN (1 min). Map each syllabus topic to the module that uses it.");

// ================================================================== 7 ARCHITECTURE
pres.addSection({ title: "Design and algorithms" });
s = content("System architecture: three-layer agent", "Anirvan Ranjan", "Design and algorithms");
const layers = [
  ["DECLARATIVE", "slot utility J(s)  →  Hybrid A* + goal funnel", "once per mission and on events", "E3F1E8"],
  ["PROCEDURAL", "Pure Pursuit steering  →  speed profile  →  PID", "every 48 ms", "E3ECF8"],
  ["REFLEXIVE", "e-stop · swept TTC · docking · re-plan", "every 48 ms, overrides", "FCE9DF"],
];
layers.forEach(([n, d, f, col], i) => {
  const y = 1.55 + i * 1.55;
  s.addShape(pres.shapes.ROUNDED_RECTANGLE, { x: 3.2, y, w: 6.9, h: 1.2, rectRadius: 0.08, fill: { color: col }, line: { color: HEX.ink2, width: 1 } });
  s.addText([{ text: n, options: { bold: true, breakLine: true } }, { text: d }], { x: 3.35, y, w: 5.2, h: 1.2,
    fontSize: 20, color: HEX.ink, valign: "middle", isTextBox: true, margin: 0 });
  s.addText(f, { x: 8.55, y, w: 1.45, h: 1.2, fontSize: 14, italic: true, color: HEX.ink2, valign: "middle", align: "right", isTextBox: true, margin: 0 });
  if (i < 2) s.addShape(pres.shapes.DOWN_ARROW, { x: 6.45, y: y + 1.22, w: 0.4, h: 0.3, fill: { color: HEX.ink2 }, line: { color: HEX.ink2 } });
});
const ins = [["Facility: free slots,\nparked cars", 1.55], ["GPS + IMU + wheels\n→ Kalman filter", 3.1], ["26 range sensors", 4.65]];
ins.forEach(([t, y]) => {
  s.addShape(pres.shapes.ROUNDED_RECTANGLE, { x: 0.6, y: y + 0.1, w: 2.2, h: 1.0, rectRadius: 0.08, fill: { color: HEX.light }, line: { color: HEX.line } });
  s.addText(t, { x: 0.6, y: y + 0.1, w: 2.2, h: 1.0, fontSize: 15, color: HEX.ink, align: "center", valign: "middle", isTextBox: true, margin: 2 });
  s.addShape(pres.shapes.RIGHT_ARROW, { x: 2.85, y: y + 0.45, w: 0.3, h: 0.3, fill: { color: HEX.ink2 }, line: { color: HEX.ink2 } });
});
s.addShape(pres.shapes.RIGHT_ARROW, { x: 10.2, y: 4.95, w: 0.4, h: 0.4, fill: { color: HEX.ink2 }, line: { color: HEX.ink2 } });
s.addShape(pres.shapes.ROUNDED_RECTANGLE, { x: 10.7, y: 4.55, w: 2.1, h: 1.2, rectRadius: 0.08, fill: { color: HEX.asphalt }, line: { color: HEX.asphalt } });
s.addText("Webots Driver\nsteer + speed", { x: 10.7, y: 4.55, w: 2.1, h: 1.2, fontSize: 16, bold: true, color: HEX.white, align: "center", valign: "middle", isTextBox: true, margin: 2 });
s.addText("Mission FSM:  SELECT → PLAN → DRIVE → DOCK → PARKED   (re-select if slot taken, re-plan if blocked)",
  { x: 0.6, y: 6.35, w: 12.1, h: 0.55, fontSize: 18, bold: true, color: HEX.ink, isTextBox: true, margin: 0 });
s.addNotes("ANIRVAN (1.5 min). Slow layers plan, fast layers react; lower layers can override. Explain the state machine and the two events: slot taken by another car, and blocked by a pedestrian.");

// ================================================================== 8 ROBOT + ENV
s = content("Robot and environment model", "Saurabh Kumar", "Design and algorithms");
img(s, "fig_sensor_layout.png", 1.332, 0.6, 1.4, 6.6, 5.0);
bullets(s, [
  [{ text: "Car: ", options: { bold: true } }, { text: "Webots BMW X5, Ackermann" }],
  [{ text: "Bicycle model: ", options: { bold: true } }, { text: "θ̇ = v·tan δ / L,  L = 2.995 m" }],
  [{ text: "R_min = L / tan δ_max = 4.87 m", options: { bold: true } }],
  [{ text: "26 range sensors, GPS, IMU, wheel speed", options: {} }],
  [{ text: "World generated from the same Python lot model (no drift between Webots and twin)", options: {} }],
], 7.5, 1.5, 5.3, 5.2, 20);
s.addNotes("SAURABH (1 min). Explain the reference point at the rear axle, the turning radius and why we needed side sensors. Mention make_world.py generates the Webots world.");

// ================================================================== 9 ALGO 1
s = content("Algorithm 1: Hybrid A* with goal funnel", "Naman Chanana", "Design and algorithms");
img(s, "fig_planner_paths.png", 2.267, 0.6, 1.35, 12.1, 3.8);
const steps = ["Expand Ackermann primitives (5 steers × fwd/rev, 1 m)", "Cost: length, reverse ×1.6, steering, gear changes",
  "Funnel: stop on the slot axis (±0.2 m, ±6°), finish with a straight reverse"];
steps.forEach((t, i) => {
  const x = 0.6 + i * 4.1;
  s.addShape(pres.shapes.OVAL, { x, y: 5.45, w: 0.6, h: 0.6, fill: { color: HEX.yellow }, line: { color: HEX.yellow } });
  s.addText(String(i + 1), { x, y: 5.45, w: 0.6, h: 0.6, fontSize: 20, bold: true, color: HEX.ink, align: "center", valign: "middle", isTextBox: true, margin: 0 });
  s.addText(t, { x: x + 0.7, y: 5.3, w: 3.25, h: 1.1, fontSize: 17, color: HEX.ink, valign: "middle", isTextBox: true, margin: 0 });
});
s.addNotes("NAMAN (1.5 min). Left: the grid path cuts a corner the car cannot drive; Hybrid A* drives past and reverses in. Right: search tree without funnel (orange) vs with funnel (blue): 5.5 times fewer nodes.");

// ================================================================== 10 ALGO 2
s = content("Algorithm 2: path tracking and speed control", "Aditya P. S. Sisodia", "Design and algorithms");
bullets(s, [
  [{ text: "Pure Pursuit: ", options: { bold: true } }, { text: "κ = 2 sin α / L_d,  δ = ±atan(L κ)  (sign flips in reverse)" }],
  [{ text: "Look-ahead tuned: ", options: { bold: true } }, { text: "L_d = max(1.6, 0.8|v|)" }],
  [{ text: "PID speed: ", options: { bold: true } }, { text: "Kp 1.8, Ki 0.6, Kd 0.05, anti-windup" }],
  [{ text: "Lag-aware braking ", options: { bold: true } }, { text: "removed a 0.35 m stop overshoot" }],
  [{ text: "Plan with 88 % steering ", options: { bold: true } }, { text: "→ reserve for feedback" }],
], 0.6, 1.5, 5.6, 5.4, 20);
img(s, "fig_speed_loop.png", 2.157, 6.4, 1.5, 6.4, 3.0);
s.addText("Phase margin 46° · rise 0.85 s · overshoot 20 % (with saturation)", { x: 6.4, y: 4.6, w: 6.4, h: 0.5,
  fontSize: 18, bold: true, color: HEX.ink, align: "center", isTextBox: true, margin: 0 });
s.addText("Stanley compared: tracks forward well, but only 57 % of missions park within tolerance (Pure Pursuit 100 %)",
  { x: 6.4, y: 5.3, w: 6.4, h: 1.2, fontSize: 18, color: HEX.ink, fill: { color: HEX.light }, isTextBox: true, margin: 8, valign: "middle" });
s.addNotes("ADITYA (1.5 min). Explain the pure pursuit geometry and reversing, the PID, the Bode plot and the two bugs we fixed (overshoot at stop, full-lock planning).");

// ================================================================== 11 ALGO 3
s = content("Algorithm 3: reflexive layer (swept TTC)", "Priyal Sharma", "Design and algorithms");
bullets(s, [
  [{ text: "R1 ", options: { bold: true } }, { text: "e-stop if contact distance < 0.45 m" }],
  [{ text: "R2 ", options: { bold: true } }, { text: "v ≤ (s_c − 0.45) / 2.5 s − v_obstacle" }],
  [{ text: "R3 ", options: { bold: true } }, { text: "docking: centre in slot, rear gap stop" }],
  [{ text: "R4 ", options: { bold: true } }, { text: "blocked 4 s → re-plan around it" }],
  [{ text: "s_c: ", options: { bold: true } }, { text: "move the car outline along the current steering arc; first sensed point it touches" }],
], 0.6, 1.5, 5.6, 5.4, 20);
img(s, "fig_reflex_timeline.png", 1.415, 6.4, 1.35, 6.4, 4.6);
s.addText("Fixed-distance rule hits the pedestrian (×); swept TTC slows smoothly and parks", { x: 6.4, y: 6.0, w: 6.4, h: 0.5,
  fontSize: 16, color: HEX.ink2, align: "center", isTextBox: true, margin: 0 });
s.addNotes("PRIYAL (1.5 min). Why a straight corridor is not enough: the side of a turning car sweeps through people. Show the timeline: same lot, same pedestrians.");

// ================================================================== 12 SETUP + DEMO
pres.addSection({ title: "Demo and results" });
s = content("Simulation setup and live demo", "Saurabh + Aditya", "Demo and results");
s.addText([
  { text: "python tools/make_world.py", options: { breakLine: true } },
  { text: "# Webots: open valet_parking.wbt, Run", options: { breakLine: true, color: "9AA0A6" } },
  { text: "python sim/run_demo.py", options: { breakLine: true } },
  { text: "python experiments/run_experiments.py", options: {} },
], { x: 0.6, y: 1.5, w: 6.6, h: 2.2, fontSize: 18, fontFace: "Courier New", color: HEX.white, fill: { color: HEX.asphalt },
  isTextBox: true, margin: 12, valign: "middle" });
bullets(s, [
  "Webots R2025a, BmwX5 PROTO, Python 3.11 controller",
  "Headless twin: same agent code, 525 Monte Carlo missions",
  "Control period 48 ms; 30 random lots per setting",
], 0.6, 4.0, 6.6, 2.4, 20);
s.addImage({ path: path.join(ROOT, "results", "demo_final.png"), x: 7.6, y: 1.35, w: 5.2, h: 4.75, objectName: "demo2" });
s.addText("▶ Switch to Webots now (≈ 6 min live demo)", { x: 7.6, y: 6.25, w: 5.2, h: 0.55, fontSize: 18, bold: true,
  color: HEX.ink, fill: { color: "FFF3C4" }, align: "center", valign: "middle", isTextBox: true, margin: 4 });
s.addNotes("SAURABH + ADITYA (6 min LIVE). Share the Webots window: car starts, console prints SELECT/PLAN/DRIVE/DOCK/PARKED, pedestrian crosses and the car slows. Then show run_demo.py in the twin and, if time, the 'slot taken' run: python sim/run_demo.py --seed 3 --steal.");

// ================================================================== 13 RESULTS 1
s = content("Results 1: planning and tracking", "Aditya P. S. Sisodia", "Demo and results");
img(s, "fig_e1_planners.png", 2.865, 0.6, 1.35, 12.1, 4.2);
statCard(s, 0.6, 5.55, 3.85, 1.55, "0 %", "grid A* success (cannot be driven)", HEX.orange);
statCard(s, 4.72, 5.55, 3.85, 1.55, "100 %", "Hybrid A* + funnel success", HEX.blue);
statCard(s, 8.84, 5.55, 3.86, 1.55, "4.2 cm · 1.2°", "final lateral / heading error", HEX.green);
s.addNotes("ADITYA (1.5 min). E1: grid A* never parks; Hybrid A* without funnel 80 %, with funnel 100 % and 5.5x fewer nodes. Tracking error 3.3 cm mean.");

// ================================================================== 14 RESULTS 2
s = content("Results 2: safety, robustness, comparison", "Anirvan Ranjan", "Demo and results");
table(s, [
  ["Experiment", "Baseline", "Ours"],
  ["E3 pedestrian contacts", "17 % fixed-distance / 20 % none", "0 % (swept TTC)"],
  ["E4 valet cycle length", "54.9 m (nearest slot)", "50.7 m (utility agent)"],
  ["E5 success at 0.3 m GPS noise", "20 % (raw GPS)", "100 % (Kalman filter)"],
  ["E6 / E7 92 % full / slot taken", "–", "93 % / 100 %"],
], 0.6, 1.4, 12.1, [4.3, 4.4, 3.4], 20, 0.55);
img(s, "fig_e5_noise.png", 2.676, 0.6, 4.4, 6.0, 2.5);
img(s, "fig_e4_e6.png", 2.671, 6.7, 4.4, 6.0, 2.5);
s.addNotes("ANIRVAN (1.5 min). Pedestrian safety, slot choice, noise robustness, full lots and the dynamic event. Be honest about limits: kinematic twin for statistics, Python planning 1-3 s.");

// ================================================================== 15 CONTRIBUTION
s = content("Team contribution (traceable to GitHub)", "Priyal Sharma", "Demo and results");
table(s, [
  ["Member", "Module and files"],
  ["Saurabh Kumar (lead)", "Lot model, costmap, Webots world generator and controller"],
  ["Naman Chanana", "Grid A*, Hybrid A* + goal funnel, path, unit tests"],
  ["Anirvan Ranjan", "Agent + state machine, utility slot selection, Kalman filter"],
  ["Aditya P. S. Sisodia", "PID, Pure Pursuit, Stanley, path tracker, tuning"],
  ["Priyal Sharma", "Reflex layer (swept TTC), simulator twin, experiments"],
], 0.6, 1.5, 12.1, [3.6, 8.5], 20, 0.8);
s.addText("Report written equally: about 5 pages each, plus own contribution row and reflection", { x: 0.6, y: 6.6, w: 12.1, h: 0.5,
  fontSize: 18, color: HEX.ink2, isTextBox: true, margin: 0 });
s.addNotes("PRIYAL (1 min). Each member owns a module and committed it from their own GitHub account.");

// ================================================================== 16 CONCLUSION
pres.addSection({ title: "Close" });
s = content("Conclusion and industry use", "All members", "Close");
const cols = [
  ["Findings", ["Kinematic planning is essential", "Funnel: 5.5× faster search", "Swept TTC: 0 % contacts", "Utility + KF: robust agent"], HEX.blue],
  ["Industry use", ["Infrastructure-supported AVP (ISO 23374)", "Factory yards and ports", "Robotaxi parking and charging"], HEX.green],
  ["Future scope", ["Real car: lidar or ultrasonic sensors, C++", "Parallel and angled slots", "Pedestrian prediction", "Many valet cars sharing a lot"], HEX.orange],
];
cols.forEach(([t, items, col], i) => {
  const x = 0.6 + i * 4.1;
  s.addShape(pres.shapes.ROUNDED_RECTANGLE, { x, y: 1.5, w: 3.85, h: 5.2, rectRadius: 0.1, fill: { color: HEX.light }, line: { color: HEX.light } });
  s.addShape(pres.shapes.OVAL, { x: x + 0.25, y: 1.7, w: 0.45, h: 0.45, fill: { color: col }, line: { color: col } });
  s.addText(t, { x: x + 0.85, y: 1.65, w: 2.9, h: 0.55, fontSize: 22, bold: true, color: HEX.ink, valign: "middle", isTextBox: true, margin: 0 });
  bullets(s, items, x + 0.2, 2.45, 3.5, 4.1, 20);
});
s.addNotes("ALL (1.5 min). Each member says one thing they learned as a team. Then summarise the findings and industrial use.");

// ================================================================== 17 END
s = pres.addSlide({ masterName: "DARK", sectionTitle: "Close" });
s.addText("Thank you", { x: 0.7, y: 1.6, w: 11.9, h: 1.2, fontSize: 54, bold: true, color: HEX.white, isTextBox: true, margin: 0 });
s.addText("Questions?", { x: 0.7, y: 2.75, w: 11.9, h: 0.8, fontSize: 32, color: HEX.yellow, isTextBox: true, margin: 0 });
s.addText([
  { text: "GitHub: ", options: { bold: true } }, { text: "github.com/<team-lead>/g14-valet-parking", options: { breakLine: true } },
  { text: "Video: ", options: { bold: true } }, { text: "youtu.be/<video-id> (unlisted)", options: { breakLine: true } },
  { text: "Contact: ", options: { bold: true } }, { text: "team e-mails in the repository README" },
], { x: 0.7, y: 4.2, w: 11.9, h: 1.6, fontSize: 20, color: HEX.white, isTextBox: true, margin: 0 });
s.addText("Saurabh Kumar · Naman Chanana · Anirvan Ranjan · Aditya Pratap Singh Sisodia · Priyal Sharma", {
  x: 0.7, y: 6.3, w: 11.9, h: 0.5, fontSize: 16, color: "C9CDD3", isTextBox: true, margin: 0 });
presenter(s, "All members");
s.addNotes("ALL. Thank the faculty; take questions.");

(async () => {
  await pres.writeFile({ fileName: OUT });
  console.log("wrote", OUT);
})();
