# AGENTS.md - working rules for every bot on the Sparrow repo

Read this before changing anything. The owner is building a real robot bird from this work; wrong numbers cost money.

## Hard rules (owner's standing instructions)

1. **Originals are untouchable**: the master Blender file, the original MuJoCo export, the scenario launcher, earlier
   controllers and checkpoints. `src/vendor/` holds byte copies of the owner's code - never edit them. New model
   behaviour = a new model copy under `model/<name>/` with a manifest listing every change (see `src/make_variants.py`,
   `game/make_world.py`, `game/make_hipyaw.py` for the pattern).
2. **Preserve working skills**: WingWalk-A (walk), QuickStep-A (fast walk), the low-hop controller and its five hop
   objectives, the validated spring jumps (`results/h2_opt/v7e_bispring/library.json` entries bi_E080/E100/E110), and
   `game/skills.json`. Improvements go in as new entries or candidates and are promoted only after the checks below pass.
3. **No CAD changes and no purchases without the owner's approval.** Propose with evidence; test in a model copy.
4. **Evidence over impressions**: every claim needs the script, the conditions and the numbers. Separate "simulated",
   "calculated" and "physically demonstrated" (nothing is physically demonstrated yet). Say what was NOT tested.
5. **Don't loosen a test to make something pass.** If a gate is wrong, say why and propose a new one.
6. **Trusted files only**: trained brains (`model.zip`, `vecnormalize.pkl`) are pickle-based and can run code when
   loaded. Only load brains produced by this project's own runs.
7. **Resource use**: on the owner's PC keep CPU and RAM under 90% and use at most 2 training workers. In the cloud,
   use what the machine has (`--envs` / `SPARROW_WORKERS`).

## How results are judged

- Development conditions vs held-out conditions: tune on one set, report both. Condition sets live in the scripts
  (friction 0.5-1.6, mass +-7-15%, centre-of-mass shifts +-3-4 mm, small leans).
- `game/selftest.py robust`: the 45 s game routine on 5 conditions. Currently 4/5 complete. Never make it worse.
- `game/brain/evaluate.py <task>`: a brain must beat the scripted controller **with full physics (air forces on)**
  on 20 unseen episodes before it is used in the game. Training may run with air forces off for speed.
- Reports: append a lettered section to `REPORT.md` (next letter after X) or a dated Markdown file next to the work.

## Work streams and who owns them

| Bot | Owns | First jobs |
|---|---|---|
| **Sparrow RL** | `game/brain/` | Scale training of `recover`, `getup`, `step_up` on many cores; tune rewards; add tasks: `land` (touch down from flight and stay up), `moving_jump`, `turn`; curriculum; domain randomisation; report learning curves and evaluate.py tables |
| **Sparrow Locomotion** | `src/`, `game/pilot.py` | Gait and stepping on uneven ground; foot length vs stride problem; hip-twist prototype gaits (`model/v7e_game_hipyaw`); make the scripted fallbacks robust (routine 5/5) |
| **Sparrow Jump** | hop/jump controllers, spring design | Forward jump >= 130 mm with >= 25 mm clearance (needed for steps); jump from a walk; landing loads with compliant couplings (REPORT sections S-U) |
| **Sparrow Aero** | `flight_research/`, `game/flight.py` | Replace prescribed wing twist with a buildable passive/active twist; validate the wing model against published ornithopters (Li et al., npj Robotics 2026); pitch authority for a landing flare; landing |
| **Sparrow CAD** | proposals only | Turn accepted proposals into CAD changes FOR OWNER REVIEW: hip-twist servo mounts, wing chord/area change, tail area, compliant couplings, spring mounts. Never edit the master file |
| **Sparrow Systems** | `parts/` | Real components: leg servo bench-test plan, flap motor + gearbox, battery, electronics, mass budget (`parts/bom.py`), power and wiring |
| **Sparrow Perception** | new `perception/` | Camera/IMU/distance sensing in simulation (MuJoCo cameras, noise models), safety filter, state estimation the brain can use instead of perfect simulator state |
| **Sparrow QA** | tests, CI | Turn selftest, evaluate.py and the robust checks into an automated test suite; run it on every change; guard the "never make it worse" rules |

Suggested new specialists if the team grows: **Sparrow Sim2Real** (servo back-drive models, latency, sensor noise,
randomisation ranges from bench data), **Sparrow Game/UX** (the playable game, controller mapping, HUD, world
content), **Sparrow Hardware Test** (bench rigs: servo characterisation, wing-and-drive thrust rig, drop tests).

## Facts that trip people up

- Forward is **-Y** in the model; left foot is +X; positive pitch is **nose down**.
- Leg joints only pitch (no roll/yaw) on the current design; that is why turning on the ground is weak.
- Air forces cost ~90% of simulation time; ground training runs 10x faster with them off (`SparrowEnv(aero=False)`).
- Flight in the game uses ASSUMED changes (wing x3 chord, tail x8 area, wing 15 mm aft, prescribed twist, 6 Nm flap
  drive) listed in `game/skills.json` -> `flight`. They are not the current design.
- Servo properties (back-drive drag, rotor inertia) are assumed ranges until bench-measured (REPORT section S).
