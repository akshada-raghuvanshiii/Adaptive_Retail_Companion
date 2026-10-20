# ARC – Adaptive Retail Companion

## Design and Simulation-Based Evaluation of a Retrofit Autonomous Shopping Cart

Course BMHA497J · Capstone Project 1

Akshada Raghuvanshi (23BMH1110) · Manav Harish Kriplani (23BMH1084)

Guide: Dr. T. Christo Michael · School of Mechanical Engineering, VIT Chennai


![VIT logo](https://raw.githubusercontent.com/akshada-raghuvanshiii/pothole_detector/arc-review-figures/vit_logo.png)

---

# Presentation Outline

| S. No. | Main Heading |
|---|---|
| 01 | Introduction & Motivation |
| 02 | Literature Review & Research Gap |
| 03 | Problem Definition & Objectives |
| 04 | Methodology |
| 05 | System Architecture & Test Set-up |
| 06 | Results: Navigation Evaluation |
| 07 | Results: Route Ordering & Phase-2 Fix |
| 08 | Results: Bench Hardware-in-the-Loop & Shopping Interface |
| 09 | Results: Mechanical Design & Structural Check |
| 10 | Key Findings, Conclusion & Future Work |
| 11 | Sustainability Impact & References |

---

# Introduction — What is ARC?

ARC (Adaptive Retail Companion) is a modular kit that retrofits a conventional supermarket cart into an assistive robot for elderly and mobility-impaired shoppers. The cart leads the user to the items on a shopping list, follows them, or waits.

**Why retrofit?**

- Reuses the cart fleet a store already owns
- Drive, sensing and computing serviced as a module
- No new vehicle for the shopper to learn

**What this project delivered**

- Baseline-first evaluation of ARC's ROS 2 navigation stack in a Gazebo store
- Engineering models checked against the measurements
- Bench hardware-in-the-loop test with real drive motors and on-board person detection
- Web shopping interface linked to the robot
- CAD retrofit with a structural check of the drive mount

---

# Literature Review — Landscape

- **Shopping robots:** RoboCart (Kulyukin 2005), CompaRob in real stores (Sales 2016), Yao et al. 2026 shopper tracking under occlusion
- **Retrofit carts:** Nishimura 2007 detachable drive module; Kohtsuka 2011 laser following — neither navigates a shopping list
- **Mapping & localisation:** RTAB-Map (Labbé 2019), ICP scan matching, LIO-SAM, ORB-SLAM2
- **Planning & control:** Dijkstra, Nav2, timed elastic band, Regulated Pure Pursuit, DWA
- **Route ordering:** order picking (Ratliff 1983), nearest-neighbour bounds, exact Held–Karp; Choi et al. 2025 genetic ordering outside ROS
- **Perception & behaviour:** YOLOv8 & ByteTrack, OSNet re-identification, behaviour trees

70 references cited in the report: 43 journal papers, 18 conference papers, 4 books and 5 websites.

---

# Literature Review — Technology Choices

| Subsystem | Alternatives | ARC choice | Reason |
|---|---|---|---|
| Localisation | Gmapping & AMCL; ORB-SLAM2; LIO-SAM | RTAB-Map & lidar ICP | Map locking and re-localisation |
| Global planning | A*; sampling planners | Dijkstra, 8-connected | Exact least-cost path |
| Local control | DWA; RPP; MPC | TEB-like controller | Narrow aisles; 19/20 goals vs 13/20 for RPP |
| Route ordering | Typed; nearest-next; genetic | Exact Held–Karp | Optimal; ≤ 4.8 ms for 12 stops |
| Perception | Colour or RGB-D tracking | YOLOv8n & ByteTrack | Runs on a Raspberry Pi 5 |
| Mission logic | Finite-state machine | Behaviour tree | LEAD, FOLLOW, WAIT modes |

---

# Research Gap & Problem Definition

**Gap:** No reviewed study measures, inside a ROS 2 stack, how reliably a retrofit cart completes shopping lists, how far simple ordering rules fall from the optimum, and whether planned savings survive execution. Integration faults, which decide whether a trip finishes, are rarely reported.

**Problem:** Measure, repeatably and before deployment, how accurately ARC localises, how reliably it completes shopping lists, how its controller and route ordering perform, and how its commands act on real motors — then trace each failure and test fixes on the same tasks.

---

# Objectives

1. **Evaluate** ARC's ROS 2 navigation stack in simulation with a baseline-first protocol: localisation, mission reliability, controller, obstacles, odometry and route ordering.
2. **Diagnose** integration-level faults with engineering calculations, and test fixes on the same tasks and random seeds.
3. **Validate** the command chain on a bench with real motors and on-board person detection, and link the shopping interface to the stack.
4. **Design** a quick-release drive mount, verify its strength and fit, and check the cart footprint against the store aisles.

---

# Methodology — Overview

- **Phase 1 – Baseline:** the unchanged stack measured in seven tests (A1–A4, B1, C1, C2)
- **Phase 2 – Fixes:** candidate fixes run on the same tasks and seeds, from copies of the configuration
- **Phase 3 – Paired comparison:** Wilcoxon signed-rank and sign tests, list by list
- **In parallel:** perception on the Raspberry Pi 5, bench hardware-in-the-loop test, UI–ROS bridge, CAD & structural analysis

Integrity checks in the test chain: verified obstacle removal, goal-matched arrivals and automatic re-checks of every leg.


![Figure 4.1: Flow chart of methodology](https://raw.githubusercontent.com/akshada-raghuvanshiii/pothole_detector/arc-review-figures/fig4_1_method_flowchart.png)

---

# Methodology — Test Set-up

| Component | Configuration |
|---|---|
| Software | Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic |
| Store | 3 gondola units, checkout, tea shelf, fridge, produce; 29 items in 9 stops |
| Map | 0.05 m grid, about 180 × 160 cells |
| Localisation | RTAB-Map in localisation mode, lidar ICP odometry |
| Global planner | Dijkstra, 8-connected, 0.20 m inflation, replan every 2 s |
| Local controller | TEB-like: 0.15 m/s, 0.5 rad/s, 1.0 rad/s² |
| Behaviour | LEAD mode, motion gate caps speed at 0.10 m/s |

| Test | Task | Trials |
|---|---|---|
| A1 | Localisation error | All samples |
| A2 | Lists of 3–12 items, typed order | 30 |
| A3 | TEB-like vs stock RPP | 20 each |
| A4 | 1–3 boxes placed on the path | 20 |
| B1 | Wheel odometry only | 10 |
| C1 / C2 | Route ordering, offline / driven | 50 per size / 30 × 3 |
| A2-pc / A2-final | Relaxed progress check / plus exact order | 30 each |

---

# System Architecture

Lidar → ICP odometry → RTAB-Map → Dijkstra planner → path hand-over → TEB-like controller (Nav2) → motion gate → motors.

- The **behaviour tree** sets the mode: FOLLOW, WAIT, LEAD or emergency stop
- The **motion gate** scales the velocity command by that mode
- The **shopping manager** sends one goal per item and waits for "goal reached"
- **Safety:** 300 ms command watchdog on the Arduino Uno, E-stop input, and a 1.5 s data-loss hold in the relay


![Figure 4.2: ARC navigation pipeline](https://raw.githubusercontent.com/akshada-raghuvanshiii/pothole_detector/arc-review-figures/fig4_2_nav_pipeline.png)

---

# Methodology — Key Engineering Models

- **Differential drive:** v = (vᵣ + vₗ)/2, ω = (vᵣ − vₗ)/B
- **Encoder resolution:** s = πD/P = π × 80/210 = 1.197 mm per pulse
- **Route cost:** J = Σ d(Pᵢ, Pᵢ₊₁), solved exactly by Held–Karp in O(n²2ⁿ): 589 824 steps for 12 stops vs 479 million orders by brute force
- **Progress check:** a leg aborts unless the robot moves r within T; turn time t = ω/α + Δψ/ω
- **Stopping distance:** d = v·t_delay + v²/(2a_b)
- **Statistics:** sign test p = Σ C(n,i)·0.5ⁿ; Wilcoxon signed-rank with exact p-values

---

# Results — Localisation & Mission Reliability

- **A1:** median localisation error **0.018–0.038 m** in every configuration; 91–98% of samples within 0.15 m and 5°
- **A2:** **29 of 30** missions and **143 of 144** stops completed with no contact — but only **67%** of stops at the first attempt and **97 recoveries**
- **All 62 controller aborts** were Nav2's default progress check (0.5 m in 10 s), which ARC never overrides

**Why it aborts — worked calculation (0.10 m/s, ω = 0.5 rad/s, α = 1.0 rad/s²):**

| In-place turn | Turn time | Distance left in 10 s | Result |
|---|---|---|---|
| 90° | 3.64 s | 0.64 m | Passes, 0.14 m spare |
| 180° | 6.78 s | 0.32 m | **Abort certain** |
| 180° with 0.25 m / 20 s | 6.78 s | 1.32 m | Passes easily |


![Figure 5.2: Displacement achievable after an in-place turn vs the progress check](https://raw.githubusercontent.com/akshada-raghuvanshiii/pothole_detector/arc-review-figures/fig5_2_progress_check.png)

---

# Results — Controller, Obstacles & Odometry

| Measure | TEB-like | RPP (stock) |
|---|---|---|
| Goals reached | 19/20 | 13/20 |
| Recoveries | 17 | 24 |
| Mean angular acceleration (rad/s²) | 0.220 | 0.083 |
| Steering reversals per m | 1.88 | 0.80 |

- **A4 – unmapped boxes:** only **3 of 18** goals reached; a box was touched in 17 legs. The planner never sees the boxes and the hand-over ignores new paths, so avoidance is left to the local controller.
- **B1 – wheel odometry only:** localisation lost twice, after about 73 m and 48 m; lidar odometry protects re-localisation.


![Figure 5.3: First localisation loss with wheel-only odometry (B1)](https://raw.githubusercontent.com/akshada-raghuvanshiii/pothole_detector/arc-review-figures/fig5_3_wheel_odom_loss.png)

---

# Results — Shopping-List Ordering (C1, C2)

- **Offline:** typed order is **23–94%** longer than the optimum; nearest-next on path cost stays within 1–7%
- **Driven:** exact order was faster than typed order on **30 of 30** lists (sign test p = 0.5³⁰ = 9.3 × 10⁻¹⁰)
- **But** completion did not improve: short hops between shelves trip the same progress-check abort

| Items on the list | Route length change | Time change |
|---|---|---|
| 5 | −32% | −25% |
| 8 | −42% | −34% |
| 12 | −51% | −41% |


![Figure 5.4: Offline route length relative to the exact optimum](https://raw.githubusercontent.com/akshada-raghuvanshiii/pothole_detector/arc-review-figures/fig5_4_route_length.png)


![Figure 5.5: Median mission time by ordering strategy and list size](https://raw.githubusercontent.com/akshada-raghuvanshiii/pothole_detector/arc-review-figures/fig5_5_mission_time.png)

---

# Results — Phase 2: Progress Check & Exact Ordering

| Measure | A2 (original) | A2-pc | A2-final |
|---|---|---|---|
| Progress check | 0.5 m / 10 s | 0.25 m / 20 s | 0.25 m / 20 s |
| Visiting order | Typed | Typed | Exact |
| Missions completed | 29/30 | 29/30 | **30/30** |
| Reached at first attempt | 67% | 96% | 92% |
| Recoveries (aborts) | 97 (62) | 4 (0) | 13 (2) |
| Median mission time (s) | 255 | 259 | **218** |
| Mean driven distance (m) | 18.6 | 18.9 | **11.6** |

On the 29 lists both completed, A2-final took **26% less time** (281.0 → 207.4 s) and drove **38% less** (18.6 → 11.5 m). Wilcoxon W = 15, n = 29, **p = 5.1 × 10⁻⁷**. The two changes are complementary: neither alone gives both reliability and speed.


![Figure 5.6: Original stack and phase-2 variants on the same 30 lists](https://raw.githubusercontent.com/akshada-raghuvanshiii/pothole_detector/arc-review-figures/fig5_6_phase2_variants.png)

---

# Results — Bench Hardware-in-the-Loop Test

Raspberry Pi 5 (YOLOv8n & ByteTrack) → UDP relay → behaviour tree → velocity command → Arduino Uno → two DC motors with Hall encoders.

| Measurement | Result |
|---|---|
| Session | 15 min: LEAD 240 s, FOLLOW 29 s, WAIT 616 s |
| Person flag → BT mode | Median 0.12 s, max 0.20 s (22 changes) |
| Stop on WAIT | PWM 0 in first sample; ≤ 1 encoder pulse (1.2 mm) after |
| Right-wheel speed | 104 mm/s at 100 mm/s command |
| Serial link | 17 660 commands, 0 errors |
| Goal-reached race | 2 of 4 goals skipped within 2 ms |

Stopping-distance estimate: d = 0.10 × 0.25 + 0.10²/(2 × 0.5) = **35 mm** at 0.10 m/s, plus 10 mm per 0.1 s of camera delay.


![Figure 5.8: Bench set-up — laptop, Raspberry Pi 5 with webcam, Arduino Uno, motor drivers and DC motors](https://raw.githubusercontent.com/akshada-raghuvanshiii/pothole_detector/arc-review-figures/fig5_8_bench_setup.jpg)


![Figure 5.9: Bench session logged by the motor bridge](https://raw.githubusercontent.com/akshada-raghuvanshiii/pothole_detector/arc-review-figures/fig5_9_bench_session.png)

---

# Results — Shopping Interface & ROS Link

- New ROS 2 package **arc_ui_bridge** serves the web UI and drives the stack — no ARC package modified
- 60 products → 22 store items → up to 9 stops, ordered by Held–Karp on the live map
- **Arrival guard** fixes the goal-reached race: accept only if within 1.0 m of the goal **and** at least 3 s after it was sent
- Pause and stop hold the robot at its pose and cancel the controller's path
- **9/9** offline tests passed; browser end-to-end test completed a 5-product trip in 4 stops with every stale arrival ignored


![Figure 5.10: Shopping interface linked to the robot](https://raw.githubusercontent.com/akshada-raghuvanshiii/pothole_detector/arc-review-figures/fig5_10_shopping_ui.png)

---

# Mechanical Design — CAD Retrofit & Quick-Release Mount

- Two quick-release drive modules at the front wheels of a standard cart
- Slider plate with rails slides into a sleeve and is held by two flat-spring latches; wheel bracket carries the motor
- Cart measured from the CAD assembly: **904 × 667 mm** (over motors), body 440 mm wide, 1275 mm high
- Wheelbase 680 mm, drive track 340 mm, Ø150 mm wheels, ground clearance 42.5 mm
- CAD check found the drive wheels drawn **36 mm below** the rear wheels (3.0° pitch) and the motor screws on the bending axis


![Figure 4.3: CAD assembly of the ARC retrofit cart](https://raw.githubusercontent.com/akshada-raghuvanshiii/pothole_detector/arc-review-figures/fig4_3_cad_cart.png)


![Figure 4.4: Quick-release drive mount — sleeve, spring latches, slider plate and wheel bracket](https://raw.githubusercontent.com/akshada-raghuvanshiii/pothole_detector/arc-review-figures/fig4_4_quick_release_mount.png)

---

# Mechanical Design — Loads, Stability, Footprint & Drive Sizing

| Check | Calculation / result |
|---|---|
| Wheel loads (50 kg at the cart centre of mass) | Drive axle carries 73.8% → **181 N** per drive wheel, 64 N per rear wheel |
| Sideways tipping | Tipping slope **15.8°**; a_lat to tip = 2.78 m/s² (≈25× the 0.11 m/s² at 0.15 m/s on a 0.2 m radius) |
| Longitudinal tipping | 16.5° over the drive axle, 39.9° over the rear axle; braking to tip 2.91 m/s² |
| Footprint re-run in the store | Straight 0.384 m: 9/9 stops; centre turn 0.562 m: 9/9; spin about drive axle 0.826 m: 3/9 |
| Drive torque per wheel | 0.84 N·m flat, 1.75 N·m on a ramp; 12.7 / 19.1 rpm |
| Drive shaft | 38 MPa |


![Figure 5.11: Plan view of the retrofitted cart with measured dimensions and centre of mass](https://raw.githubusercontent.com/akshada-raghuvanshiii/pothole_detector/arc-review-figures/fig5_11_cart_plan.png)

---

# Structural Analysis — FEA Validation

SimScale static analysis of the slider and wheel bracket, checked against an independent scikit-fem model (P2 tetrahedra) and beam theory.

| Method | Root stress | Tip deflection |
|---|---|---|
| SimScale | 2.4–2.8 MPa | 0.020 mm |
| scikit-fem (independent) | 2.1–2.5 MPa | 0.017 mm |
| Beam theory | 2.45 MPa | 0.023 mm |

All three agree within about 15%, so the model set-up is trusted for the design-load case.


![Figure 5.12: SimScale von Mises stress in the slider and wheel bracket](https://raw.githubusercontent.com/akshada-raghuvanshiii/pothole_detector/arc-review-figures/fig5_12_simscale.png)


![Figure 5.13: SimScale result re-plotted with the colour scale capped at 4.5 MPa](https://raw.githubusercontent.com/akshada-raghuvanshiii/pothole_detector/arc-review-figures/fig5_13_simscale_replot.png)

---

# Structural Analysis — Design Load & Fixes

Design load: **181 N** at the measured **71.7 mm** wheel offset (mesh converged, 96k → 167k DOF).

- Peak **246 MPa** at the motor-hole ligament; 139 MPa at the bracket root
- Safety factor about **1.1 (Al 6061-T6) / 1.0 (steel)** — fails under a bump factor of 2; bracket tilt 1.59° (Al)
- Guide clearance 1.1 mm per side allows **2.5°** wheel misalignment

| Fix | Peak stress |
|---|---|
| 6 mm plate | 84 MPa |
| Wheel offset e = 20 mm | 69 MPa |
| Both | 23 MPa |
| Fork (two-sided support) | about 2.2 MPa |

**Result:** the bracket must be stiffened before fabrication — the combined fix gives a safety factor above 10.


![Figure 5.14: Independent FE model of the wheel bracket under the design wheel load](https://raw.githubusercontent.com/akshada-raghuvanshiii/pothole_detector/arc-review-figures/fig5_14_fea_design.png)

---

# Key Integration-Level Findings

1. **A default parameter causes most aborts** — Nav2's progress check is too strict at 0.10–0.15 m/s
2. **The path hand-over has no replanning or recovery** — unmapped obstacles are left to the local controller
3. **A goal-reached race** lets a stale arrival skip goals — observed on the bench, fixed in the UI bridge
4. **Lidar odometry does not recover** from a failed match after contact
5. **Lidar odometry protects re-localisation** — wheel odometry alone loses the map

**Recommendation:** adopt a 0.25 m / 20 s progress check together with exact path-cost ordering, paired with contact detection on the cart.

---

# Conclusion

- ARC localises to **2–3 cm** and completes 29 of 30 missions in simulation; a kinematic calculation explains all 62 aborts
- Exact ordering with a relaxed progress check completed **all 30 lists** in **26% less time** and **38% less distance** (p = 5.1 × 10⁻⁷)
- On the bench, Pi-based person detection drove real motors, which stopped within one telemetry period of every WAIT
- The quick-release mount was designed and checked: the cart is stable, but the wheel bracket must be stiffened before fabrication

**Technical hurdles:** goal-reached race (fixed by the arrival guard) and an intermittent left encoder (traced by calibration comparison).

---

# Future Work

- Adopt the progress-check & ordering fixes in ARC itself
- Add replanning to the path hand-over and odometry recovery after contact
- Restore left-wheel feedback; measure camera-to-stop delay and Pi frame rate
- Mount a 2D lidar on the cart and repeat the same 30 test lists on it
- Stiffen the wheel bracket, rotate the motor-screw pattern and tighten the guide fit
- Run the UI bridge on the real stack; two-person re-identification; trials with older users

---

# Sustainability & Societal Impact

- **SDG 3 – Good health & well-being:** less physical effort for elderly and mobility-impaired shoppers
- **SDG 9 – Industry, innovation & infrastructure:** a retrofit upgrades existing carts instead of replacing the fleet
- **SDG 10 – Reduced inequalities:** supports independent access to stores
- **SDG 12 – Responsible consumption & production:** optimised routes cut driven distance by 38% per trip in simulation

These are intended outcomes; only the route-distance saving has been measured.

---

# Selected References

1. Sales, J. et al. (2016), CompaRob: The shopping cart assistance robot, Int. J. Distributed Sensor Networks 12(2).
2. Nishimura, S. et al. (2007), Attachable modules for robotizing daily items — person-following shopping cart, IEEE ROBIO.
3. Labbé, M. and Michaud, F. (2019), RTAB-Map, Journal of Field Robotics 36(2), 416–446.
4. Macenski, S. et al. (2020), The Marathon 2: A navigation system, IEEE/RSJ IROS.
5. Rösmann, C. et al. (2017), Integrated online trajectory planning and optimization, Robotics and Autonomous Systems 88.
6. Held, M. and Karp, R. M. (1962), A dynamic programming approach to sequencing problems, J. SIAM 10(1).
7. Choi, D. et al. (2025), Path planning for robots in supermarket environments, Electronics 14(11).
8. Zhang, Y. et al. (2022), ByteTrack, ECCV.
9. Yao, H. et al. (2026), Shopping service robot with intersection-aware following, IEEE RA-L 11(4).
10. Wilcoxon, F. (1945), Individual comparisons by ranking methods, Biometrics Bulletin 1(6).

---

# Thank You

ARC – Adaptive Retail Companion

Akshada Raghuvanshi (23BMH1110) · Manav Harish Kriplani (23BMH1084)

Guide: Dr. T. Christo Michael · School of Mechanical Engineering, VIT Chennai
