# ARC – Adaptive Retail Companion

**Design and simulation-based evaluation of a retrofit autonomous shopping cart**

ARC is a modular kit that retrofits an ordinary supermarket cart into an assistive robot for elderly and mobility-impaired shoppers. The cart leads the shopper to the items on a list, follows them, or waits.

This repository holds everything from the capstone project:
- the ROS 2 navigation stack;
- the simulation test kit and its results;
- the hardware-in-the-loop bench code, with perception on a Raspberry Pi;
- the web shopping interface;
- the CAD and structural analysis of the drive mount;
- the report, the paper and the presentation material.

Course BMHA497J, Capstone Project 1, School of Mechanical Engineering, VIT Chennai.
Team: Akshada Raghuvanshi (23BMH1110) and Manav Harish Kriplani (23BMH1084).
Guide: Dr. T. Christo Michael.

---

## Key results

| Test | Result |
|---|---|
| Localisation (A1) | Median error 0.018–0.038 m; 91–98% of samples within 0.15 m and 5° |
| Missions, original stack (A2) | 29/30 lists, 143/144 stops, no contact. But only 67% of stops reached at the first attempt, and 97 recoveries were needed |
| Cause of aborts | All 62 were Nav2's default progress check (0.5 m in 10 s); a 180° turn at 0.10 m/s leaves only 0.32 m |
| Controller (A3) | TEB-like 19/20 goals vs stock RPP 13/20 |
| Route ordering (C1, C2) | Typed order is 23–94% longer than the optimum; exact ordering was faster on 30/30 lists |
| **Phase 2: relaxed check + exact ordering** | **30/30 lists, 26% less time, 38% less distance (Wilcoxon p = 5.1 × 10⁻⁷)** |
| Bench HIL | Pi 5 perception drove real motors; PWM was 0 in the first sample after every WAIT; 17 660 serial commands, 0 errors |
| Mechanical | Cart is stable (15.8° sideways tipping slope); the bracket at 246 MPa must be stiffened. A 6 mm plate plus a 20 mm wheel offset brings it to 23 MPa |

The full method, calculations and discussion are in [`docs/report/ARC_Capstone_Report.pdf`](docs/report/ARC_Capstone_Report.pdf).

---

## Repository layout

```
ros2_ws/src/                 ROS 2 Jazzy packages (colcon workspace)
  arc_planner/               Dijkstra planner, shopping manager, path hand-over (planned_path_to_teb)
  teb_local_planner/         ARC's own TEB-like Nav2 controller plugin (not the upstream TEB package)
  arc_behavior_tree/         Behaviour tree (FOLLOW / WAIT / LEAD / E-stop), motion gate, person inputs
  arc_store_sim/             Gazebo store worlds, maps, launch files
  arc_person_follower/       Person path follower
  autonomous_tb3/            Exploration helper for the stand-in robot
  arc_ui_bridge/             Web shopping interface + bridge to the stack (Held–Karp ordering, arrival guard)
  arc_hw_bridge/             Motor bridge from /cmd_vel to the Arduino (inside the workspace)
perception/                  YOLOv8n + ByteTrack on the Raspberry Pi, target lock, UDP relay to the laptop
  offline_tools/             Offline tracking and movement-analysis scripts used on recorded video
hardware/
  arc_hw/                    Arduino Uno firmware (M,/ENC, protocol), bench and calibration tools
  raspberry_pi/              Pi set-up, deployment, HIL launch and logging scripts
evaluation/
  arc_test_kit/              Test kit that ran tests A1–A4, B1, C1, C2 and phase 2
  arc_compact_store_kit/     Compact-store build, mapping drive and run scripts
  patches/phase2_fixes/      The progress-check and exact-ordering changes tested in phase 2
  results/                   Raw results: missions, legs, events, samples (gzip), run logs, HIL log
mechanical/
  cad/                       STEP files: cart, drive modules, sleeve, springs, motor, wheels
  analysis/                  Geometry and scikit-fem FEA scripts with their results
  simscale/                  SimScale assembly, results and re-plot scripts
docs/
  report/                    Capstone report (.docx and .pdf)
  paper/                     IEEE paper draft (.docx, .pdf, LaTeX source and figures)
  presentation/              Slide content for the review deck
  figures/                   All report and deck figures
  runbook/                   Run sheets for the simulation
research paper/              Reference papers collected during the literature review
```

---

## Quick start (simulation)

Tested on Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic.

```bash
# 1. Build the workspace
cd ros2_ws
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
source install/setup.bash

# 2. Bring up the compact store, mapping drive and navigation
#    (see evaluation/arc_compact_store_kit/RUN.md for the full order)
cd ../evaluation/arc_compact_store_kit
./install.sh
./arc_up.sh

# 3. Run a test from the evaluation kit
cd ../arc_test_kit
cat README.md        # test names and options, then ./run.sh <test>
```

**Hardware-in-the-loop:** see `hardware/raspberry_pi/README.md` and `hardware/arc_hw/README.md`. The laptop and the Pi both join the same phone hotspot. The YOLOv8n weights (`yolov8n.pt`) are not stored here; download them from Ultralytics into `perception/pi/arc-perception/`.

---

## Status and limitations

- All navigation results are from simulation, using a stand-in differential-drive robot. The cart itself has not yet driven a shopping list.
- The bench HIL used two real motors on a bench, not motors mounted on the cart. The Pi's frame rate was not measured.
- Stopping-distance and drive-torque figures are illustrative estimates.
- The phase-2 fixes were tested on copies of the configuration and are not yet merged into the packages.
- `ros2_ws/src` is the workspace as of 12 Oct 2026, plus `arc_ui_bridge` and `arc_hw_bridge`, which were added later. Changes made on the lab laptop after that date should be committed from there.

## Licence

Apache 2.0, see [LICENSE](LICENSE).
