# Demonstration video: plan and script (target 20:00, allowed 18-22 min)

Rules recap: an online meeting on Google Meet, Teams or Zoom; **record the whole meeting**;
everyone **camera ON in grid view** for the entire video; everyone speaks about 4 minutes;
**live** Webots demo (not screenshots); start with name + SAP ID; no cuts in the middle;
upload as **Unlisted** on YouTube.

## Before recording (day before)
1. On the demo laptop (Saurabh): Webots R2025a installed, Python with numpy and scipy selected in
   Webots > Preferences > Python command.
2. `python tools/make_world.py --seed 3 --occupancy 0.7 --pedestrians 1` and open
   `worlds/valet_parking.wbt`. Run it once **fully** to check that the car parks. If it steers the
   wrong way, add `"--steer-sign=1"` to `controllerArgs` (see README).
3. Take the Webots screenshot for the report (`report/figures/webots_screenshot.png`), then
   run `report/build.sh` again.
4. Keep a backup in case Webots crashes during recording: `python sim/run_demo.py --seed 3 --pedestrians 2`
   (shows the same agent in the twin) and `results/demo.gif`.
5. Do one timed rehearsal and adjust using the timing column below.

## Run sheet

| Time | Slide | Who | What to say / do |
|---|---|---|---|
| 0:00 | (cameras) | All | Each person says: "I am <name>, SAP ID <id>." (about 10 s each) |
| 0:50 | 1 | Saurabh | Project, group, course; one-sentence summary |
| 1:30 | 2 | Saurabh | Real problem, AVP in industry (Bosch/Mercedes, ISO 23374), why it is hard (turning radius, reversing, pedestrians) |
| 2:45 | 3 | Saurabh | Five objectives: target vs achieved |
| 3:45 | 4 | Naman | Existing solutions and the three gaps |
| 5:00 | 5 | Naman | Novelty: four points with numbers |
| 6:15 | 6 | Anirvan | Syllabus mapping |
| 7:00 | 7 | Anirvan | Three-layer architecture + mission FSM; events (slot taken, blocked) |
| 8:15 | 8 | Saurabh | Car model, R_min, 26 sensors, world generated from Python |
| 9:00 | 9 | Naman | Hybrid A*: primitives, cost, goal funnel; point at the search trees |
| 10:15 | 10 | Aditya | Pure Pursuit (reverse), PID, Bode/phase margin, the two bugs fixed |
| 11:30 | 11 | Priyal | Reflex rules R1-R4, swept-footprint TTC, timeline figure |
| 12:45 | 12 | Saurabh + Aditya | **Switch screen-share to Webots.** Press Run. Saurabh narrates the console states (SELECT -> PLAN -> DRIVE -> DOCK -> PARKED); Aditya explains what the controller does at the gear change and while docking; point out the pedestrian and the car slowing. Then show `python sim/run_demo.py --seed 3 --steal` (slot taken -> re-select). |
| 18:45 | 13 | Aditya | Results 1: planners and tracking |
| 19:30* | 14 | Anirvan | Results 2: safety, slot policy, noise, occupancy |
| 20:15* | 15 | Priyal | Team contribution |
| 20:45* | 16 | All | Each says one sentence: what they learned as a team |
| 21:30* | 17 | All | Thank you / Q&A |

\* If the rehearsal runs over 21:30, shorten the live demo to 5 min; the video must stay under
22:00. Each member's total speaking time is about 4 minutes: Saurabh (1, 2, 3, 8, demo),
Naman (4, 5, 9), Anirvan (6, 7, 14), Aditya (10, demo, 13), Priyal (11, 15, 16).

## After recording
* Trim only the very start and end. Upload to YouTube as **Unlisted** (not Private, not Public).
* Put the link into the report Appendix and slide 17, rebuild the report and slides, and export
  the slides PDF again.
