# Sparrow - bird-inspired walking, hopping and (eventually) flying robot

Simulation, control and design-research workspace for **Sparrow V7E**: a 0.52-0.56 kg robot bird (1.42 m wingspan)
modelled in MuJoCo from the owner's Blender CAD. Goal: a real, 3D-printed bird that walks, runs, hops, jumps, takes
off, flies and lands, and is also a companion robot (camera, microphone, speaker, sensors, game-controller driving,
later onboard autonomy). Everything here is **simulation or desk study**; nothing has been built or measured yet.

New here (human or bot)? Read **AGENTS.md** first: it has the rules, the work streams and how results are judged.

## Quick start

```bash
python -m venv .venv && . .venv/bin/activate          # Windows: .venv\Scripts\activate
pip install -r requirements.txt
cd game && python selftest.py                          # headless: walk, hop, turn, jump, fast walk, high jump (45 s routine)
cd brain && python train_brain.py recover --steps 2e5 --envs 4      # train a brain (headless, any machine)
python evaluate.py recover                             # judge it against the scripted controller, full physics
```

On Windows with a screen: double-click `Sparrow Studio.bat` (play, train, check) or `Play Sparrow.bat`.
Headless servers need no display: nothing in training or testing renders.

## Map

| Folder | What is in it |
|---|---|
| `game/` | the playable game, its pilot (`pilot.py`), shared simulation loop (`world.py`), flight (`flight.py`), get-up (`getup.py`), parameter trainer (`train.py`), Studio window, `skills.json` (skills in use) |
| `game/brain/` | **the learned brain**: gymnasium environments (`sparrow_env.py`), PPO trainer, evaluator, in-game loader. Trained runs in `runs/` |
| `src/` | locomotion research code: walking gaits, hop controllers (`hop.py`, `hop2.py`, `hop3.py`), experiments `v*.py`, `h*.py`, `t*.py`, `lab.py` (instrumented simulation) |
| `src/vendor/` | byte copies of the owner's original controller and simulator helpers - do not edit |
| `model/<variant>/` | MuJoCo model copies; each has a manifest of its exact changes. `v7e_game` = game world; `v7e_game_hipyaw` = proposed hip-twist servos |
| `flight_research/` | flight feasibility, wing model, take-off studies (F0, F1 reports and scripts) |
| `parts/` | parts shortlist, bill of materials and mass roll-up |
| `results/` | experiment summaries (JSON/Markdown). Raw traces are not in git (regenerate with the scripts) |
| `REPORT.md` | full engineering log, sections 1-10 and A-W, with numbers |
| `HANDOFF_FLIGHT_STUDY.md` | flight study handoff |
| `codex_continuation_*` | work by an earlier assistant (Codex) on flight, kept as delivered |

## Where things stand (17 Nov 2026, all simulation)

| Capability | State |
|---|---|
| Walking | 137-160 mm/s, validated on development and held-out conditions |
| Fast walk | ~200 mm/s; not a true run (no flight phase) |
| Hops / jumps | 30-60 mm low hops; 60 / 80 / 100 mm jumps with a proposed hip-and-knee spring, validated |
| Hop while walking | works mechanically, mostly falls; untrained |
| Climbing a 20 mm step | fails: 128 mm foot vs ~30 mm stride. Needs a forward jump >= 130 mm |
| Turning on the ground | weak on current legs (pitch-only joints). Strong (~50-60 deg/s) with proposed hip-twist servos |
| Get-up after a fall | scripted attempt reaches ~34 deg then stalls; brain training started |
| Flight | free-body flight with course hold and ~40 deg/s turns, only with ASSUMED wing/drive changes |
| Take-off | from a 12 m tower only; ground take-off not feasible with this wing (see flight_research) |
| Landing | fails (5-7 m/s touchdown, tips over) |
| Brain (RL) | environments for get-up, walking recovery under shoves, step-up; first runs trained locally |

## Rules that matter most

1. Never modify the owner's master Blender file, the original MuJoCo export, or `src/vendor/`. Make model copies.
2. No CAD change and no purchases without the owner's approval. Mechanical ideas are tested as model copies first.
3. Report measured results only, with the conditions, and say what was not tested.
