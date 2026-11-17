# Sparrow V7E - handoff for continuing the flight-feasibility study (15 Nov 2026)

Workspace: `D:\Downloads\Sparrow_Locomotion_Engineering_V1` (everything here is a development copy).
Python: `D:\Downloads\project-sparrow\.venv\Scripts\python.exe` (MuJoCo 4.0.0, Python 3.12), PowerShell 5.1.

## Rules from the owner (keep these)

- Do NOT modify the V7E master Blender file, the original MuJoCo export, the scenario launcher, earlier controllers or
  checkpoints. Work only in this workspace. No CAD change and no purchases without the owner's approval.
- Preserve the working controllers: WingWalk-A, QuickStep-A (137 mm/s), the low-hop controller and its hop objectives.
- At most 2 simulation processes at once (a third light job is fine); keep CPU and RAM under 90%. Set
  `OMP_NUM_THREADS=1`. Never stop Python processes you did not start (a Blender bridge may be running).
- After any work run `launchers\Verify-Originals.ps1`. Expected output: 2 changed (the two `.blend` files, saved in
  Blender by the owner on 12 Nov), 0 missing, 0 added. Anything else means an original was touched.
- Keep both goals: full companion features AND self take-off. Do not propose stripping features until Concept B
  (below) has been tested properly. Distinguish simulated results from physically demonstrated ones.

## Where things are

- `REPORT.md` - everything so far, sections 1-10 and A-W. Hopping detail is in O-U, parts in V, take-off in W.
- `parts/BOM.md`, `parts/LEG_SERVO_SHORTLIST.md`, `parts/bom.py` - parts list and mass roll-up (about 529 g).
- `flight_research/F0_FEASIBILITY_AUDIT.md`, `F1_TAKEOFF_STUDY.md` - flight studies.
- `flight_research/f1_wing_model.py` - blade-element wing model with prescribed twist (functions `wing_forces`, `cycle`).
- `flight_research/f2_redesign_sizing.py` - hover-class sizing of design variants.
- `flight_research/f3_jump_to_flight.py` - jump-to-flight transient (`fly`, `best_case`, `set_wing`, `jump`); modes `paper`, `sparrow`.
- `flight_research/f4_concept_b.py` - Concept B study; modes `grid <area_scale>`, `timing`, `launchmap <max Hz>`.
- Results and logs: `flight_research/results/`.

Run from `flight_research`, for example:
`$env:OMP_NUM_THREADS='1'; & D:\Downloads\project-sparrow\.venv\Scripts\python.exe f4_concept_b.py grid 2.0`

## State of the evidence (all simulation or calculation, assumed coefficients)

Hopping (MuJoCo, v7e_bispring variant):
- Hip-and-knee spring gives 60/80/100 mm hops. With realistic assumed servo back-drive they need 2.4-3.3 J.
- Compliant servo couplings cut touchdown force: 52-59 mm hop passes 9/9 and 5/5 at 53 N (6 Nm/rad, 1.92 J);
  100 mm passes 9/9 and 5/5 only with a 30 Nm/rad coupling (137 N). Hip gear train still sees 1.2-1.7 Nm in the push.
- Servo values are assumed, not measured. A bench test of real servos is the missing step.

Flight:
- Wing model check: zero twist gives zero thrust; level cruise 11-13 W aerodynamic at 9-10 m/s, 3.5-4 Hz.
- Check against a real robot (Li et al., npj Robotics 4, 4, 2026: 180 g, 0.9 m span, 0.27 m chord, ~3 J spring, jump
  to ~1 m at 60 deg with 2.5 m/s forward, flies at 8 Hz, cannot hold height below 6 Hz, 32/40 take-offs): with an
  assumed +-40 deg flap amplitude the model sinks at <=6 Hz and flies from 7 Hz; with +-30 deg it needs 9 Hz. So the
  model is usable and 1-2 Hz pessimistic; its power looks about 2x high. Amplitude, twist and wing mass are not in the paper.
- That robot's wing loading is about 9 N/m2. Sparrow's is about 31 N/m2. This, with a 1 m jump instead of 0.1 m, is the
  main difference.
- Sparrow as designed: from any jump of 3-10 J it only stays up by hovering (10 Hz, ~110 W, ~11 Nm per wing). Not feasible.
- First Concept B numbers (529 g kept, wing area doubled through the chord, stroke +-45 deg, twist <= 50 deg):
  stays up from 5 Hz at about 82 W, 5.9 Nm (coarse grid, hover-like). At 450 g with a 10 J jump, or 400 g with 7 J,
  it accelerates into forward flight on about 37 W at 4 Hz.

## Jobs running when this was written (results will be in `flight_research/results/`)

1. `f4_concept_b.py grid 2.0` then `timing`  -> `f4_grid_area20.log`, `f4_timing.log`
2. `f4_concept_b.py grid 1.5` then `launchmap 4.5` -> `f4_grid_area15.log`, `f4_launchmap.log`
If the logs are incomplete, re-run that mode; each takes roughly 10-30 minutes.

## What to do next, in order

1. Read the four f4 logs. Answer for Concept B (529 g, bigger wing): least power and torque for a successful
   take-off; whether starting to flap earlier helps; minimum jump height and forward speed.
2. Extend `f4_concept_b.py grid` to area x2.5 and x3, and to a heavier jump mechanism (the 180 g robot spends a third of
   its mass on it; a 7-10 J spring for Sparrow is not in the 529 g budget). Add 30-60 g and re-check.
3. Sensitivity of the conclusion to the uncertain coefficients: stalled-plate coefficient 1.2 / 2.0 (`CN`), leading-edge
   suction 0.5 / 0.9 (env `SPARROW_ETA_S`), twist limit 35 / 50 / 65 deg. Report which results survive all of them.
4. More model checks against the paper where it gives numbers: body pitch (30 -> 80 -> 30-50 deg), about 1 m height when
   flapping becomes evident, 12 min flight on a 7.4 V 300 mAh battery (about 11 W electrical average).
5. Mass budgets for three concepts with every missing item (fasteners, wiring, bearings, springs, couplings,
   regulators, latch): A existing Sparrow, B same features with bigger lighter wing and stronger jump, C lighter
   Sparrow (400-450 g) only if B fails. Start from `parts/bom.py`.
6. Design (on paper only) a wing-and-drive test rig from purchasable parts: one wing pair, 1806-size brushless motor and
   ESC, reduction of about 50:1 with a crank, two load cells with HX711 amplifiers for thrust and lift, a current/voltage
   sensor, a flap-rate sensor, a microcontroller logger. Give a parts list with prices and what each measurement would
   confirm or refute in the model.
7. Recommendation: the smallest set of changes that gives credible self take-off, and what must be measured first.

## Known limits to state with every result

Point-mass body with a prescribed pitch angle (stability in the air is untested); prescribed wing twist (no twist
mechanism exists in CAD); assumed aerodynamic coefficients; no wing flexibility or unsteady effects; bigger-wing
variants assume today's wing inertia; hop results use assumed servo properties; nothing has been built or measured.

## Update: first two f4 jobs finished (read from `f4_grid_area20.log`, `f4_timing.log`)

Concept B at full mass (529 g), wing area doubled, twist <= 50 deg:
- Every jump of 5-13 J reaches sustained flight, but at 54-96 W aerodynamic and 6.2-8.0 Nm peak per wing.
  That is above the working limits assumed so far (about 50 W and 4 Nm), so it is not yet a feasible design.
- Best case: 13 J jump at 45 deg (4.4 m/s forward, 1.0 m apex) -> 54 W at 4.5 Hz, 6.4 Nm, accelerating into
  forward flight (10.9 m/s). A 45 deg launch is better than 60 deg: forward speed matters more than height.
- Below about 10 J the robot only stays up in a hover-like state at about 80 W (+-45 deg stroke) or 96 W (+-60 deg).
- Timing (10 J, 60 deg, +-60 deg stroke): already flapping at release needs 84 W and transitions to forward flight;
  starting 0.1-0.5 s after release needs 96-97 W and stays hover-like. Starting early helps, modestly.
- The torque is mostly wing inertia (bigger-wing variants assume today's 0.0049 kg m2 per wing). A lighter wing or a
  spring at the flap hinge is the obvious next variable, along with area x2.5-3 and the model's ~2x power pessimism.
Still to read: `f4_grid_area15.log`, `f4_launchmap.log`.
