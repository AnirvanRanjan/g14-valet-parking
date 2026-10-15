# Meeting log: G14 Autonomous Valet Parking Agent

Fill in the real dates, durations and attendees **as the meetings happen**. The evaluator may
check them against the Meet/Teams recordings and the commit dates. The rows below are the
**planned** agenda for each meeting; edit them to match what you actually decided.

| No. | Date | Duration | Mode | Attendees | Agenda / decisions | Action items |
|---|---|---|---|---|---|---|
| 1 | 15/10/2026 | | Online | | Read P13; chose team lead; split work into 5 modules (lot + Webots, planners, agent, control, reflex + experiments); created the GitHub repo | All: install Webots R2025a and Python 3.11 |
| 2 | 17/10/2026 | | | | Chose Hybrid A* over plain A* (turning radius); agreed on the 3-layer architecture and interfaces (`agent.step()`); chose reverse-in parking | Naman: planner; Aditya: tracker |
| 3 | 20/10/2026 | | | | First full mission in the Python twin; found the tracker overshoot at the end of the path and fixed it with lag-aware braking | Priyal: pedestrians + reflex |
| 4 | 22/10/2026 | | | | Pedestrian collisions found: side-swipes while turning -> designed the swept-footprint TTC rule; added side sensors | Anirvan: Kalman filter; Saurabh: Webots world |
| 5 | 24/10/2026 | | | | Ran experiments E1-E7; decided on report figures; split the report writing (Writing Allocation table) | All: write own sections |
| 6 | 26/10/2026 | | | | Webots run and screenshots; slide review; rehearsal (timed) | Record the video on 27/10 |
| 7 | 27/10/2026 | 20 min | Online (recorded) | All 5 | Final recorded presentation + live demo | Team lead uploads to YouTube (Unlisted) |
| 8 | 28/10/2026 | | | | Print, bind, peer-assessment sheets, Google Form | Submit by 29/10 |
