# Sparrow V7E — Locomotion Engineering V1, Phase 0 + Phase 1 report (12 Nov 2026)

Everything here was measured in MuJoCo 4.0.0 from **copies** inside this folder. No original file was modified
(1,237 originals hashed before work and re-verified after: 0 changed, 0 missing, 0 added — `manifests/`).
The master `.blend` was inspected read-only through the live Blender session and was **not saved or edited**.

## 1. Headline

**The unmodified V7E mechanism can walk** — slowly, quasi-statically — using two things nobody had tried:

1. **Wing as lateral counterweight.** Fully un-sweeping the stance-side wing moves the whole-robot COM 24.6 mm sideways,
   which puts it 13–16 mm *inside* the stance foot pad. (Tail: 0.0 mm. Neck yaw: ±2.6 mm. They cannot do this job.)
2. **Lean back over the ankle before lifting a foot.** In the standing pose the centre of pressure is ~43 mm ahead of
   the ankle axis: 5.1 N × 0.043 m = 0.22 Nm, more than **one** 0.2 Nm ankle servo can hold. Moving both ankles 30 mm
   forward under the body removes that load. Without it every single-support attempt falls in pitch within 0.5 s.

With both, 10 planned steps give 10 valid alternating steps under the strict detector (Section 4).

**No mechanical redesign is requested at this point.** Two things do need your decision (Section 8).

## 2. Tool access actually verified

| Tool | Evidence |
|---|---|
| Blender 4.2.3 LTS (live, MCP) | `Sparrow_V7E_HeadWings.blend`, scene "SPARROW V7 \| Lightweight flyer", 4,107 objects (3,790 mesh, 231 empty, 63 camera), 131 collections, 13 actions, frames 1–1020. Joint frames read: hips at x = ±30 mm, all leg joint X-axes = world X (pitch only). Read-only; claim released. |
| PowerShell 5.1 / filesystem | all seven project locations found and hashed |
| Python 3.12.16 + MuJoCo 4.0.0 | model loads: 27 bodies, 26 joints, 17 actuators, 110 meshes, 0.5199 kg |

Note: the Blender file was already flagged **unsaved-changes (dirty)** when I connected. I did not touch that state.

## 3. Phase 0 — corrected baselines (results/p0_controls)

| Case | Result | Archived V3/V4 value |
|---|---|---|
| **True quiet stand, feedback only, 20 s** | **upright, max tilt 3.4°, knee target exactly 0** | (never run in V4) |
| Hold pose, no feedback, 20 s | upright, max tilt 5.3° | — |
| V4 `stationary_control` reproduced (gait amp 0) | knee target oscillates 0…0.10 rad, **falls 2.535 s** | 2.54 s |
| V4 shuffle (amp −0.085, 1.2 Hz) 12 s | upright, +131 mm, **feet never unload: 0 swing intervals** | +127.6 mm, 0 steps |
| Original gait | falls 1.165 s | 1.17 s |
| 12° lift L / R | falls 1.654 / 1.761 s | 1.66 / 1.77 s |

So: the quiet-standing controller has **not** regressed; the V4 "stationary" fall is entirely the hidden knee wiggle.
The new instrumentation reproduces every archived fall time to within 10 ms.

## 4. What counts as a step here (src/lab.py `detect_steps`)

A step is valid only if, in order: the foot's ground force is zero; the **lowest corner** of its pad rises > 5 mm;
meanwhile the other foot carries ≥ 50 % body weight and no non-foot part touches the ground; the pad centre advances
≥ 10 mm in the world; it is re-loaded to ≥ 20 % body weight for ≥ 30 ms; no fall for 1 s afterwards.
"Strict" also requires the stance foot to move < 5 mm during the swing. Forces come from `mj_contactForce`, 1 kHz.

## 5. Phase 1 — lateral authority of existing actuators (results/p1_authority)

| Command (ramped, held, feedback on) | COM lateral shift | COM position vs stance pad |
|---|---:|---|
| tail roll ±30° / tail pitch 35° | 0.0 mm | 9 mm outside |
| neck yaw ±60° | ±2.6 mm | 6 mm outside |
| left wing 25 / 50 / 75 / 100 % deployed | +9.4 / +17.4 / +22.8 / **+24.6 mm** | +0.8 / +8.7 / +14.1 / **+16.0 mm inside** |
| wing 100 % + neck + tail | +27.5 mm | +18.8 mm inside |
| leg-length difference 4 / 8 mm | −4.0 / −22.3 mm (body rolls 1.2° / 6.8°, feet on edges) | −5 / +12.5 mm |
| leg-length difference 12 mm | topples sideways at 2.2 s | — |

All commanded joints reached their targets (commanded vs achieved logged per test).

## 6. One step, then walking

Single step (results/p1_one_step), right foot, left wing out:

| Variant of the attempt | Outcome |
|---|---|
| no wing | falls 2.3 s |
| wing out, no back-lean, level foot | falls 2.3 s in pitch, stance ankle pinned at 0.2 Nm |
| wing out, back-lean 10 / 20 mm | falls |
| **wing out, back-lean 30 mm** | **valid strict step: 0.80 s airborne, 10.3 mm clearance, +28.6 mm, stance load 1.00 BW, stance slip 0.4 mm, max tilt 2.2°, 0 % saturation** |
| same, slow (1.6 s airborne) | valid strict step, 11.2 mm clearance |
| back-lean 50 mm | step valid, then falls backward at 4.0 s (too far) |

Walking gait "WingWalk-A" (`src/gait.py`; frozen parameters: step 40 mm, lift 20 mm, back-lean 30 mm, 0.7 s double support):
single support with stance wing out → double support where the wings swap and the body advances.

Gate: ≥ 5 valid alternating replants, ≥ 0.25 m forward COM, ≥ 10 s upright. Each condition run starting on either foot.

| Model copy | Nominal (10 steps planned) | Development set (12 conditions × 2) | Untouched holdout (8 × 2) |
|---|---|---:|---:|
| `v7e_baseline` (original contact) | 10/10 valid, 347 mm, 25.3 s, tilt 2.2° | **22/24** | **14/16** |
| `v7e_stiffcontact` | 10/10 valid, 13 mm lateral margin | **24/24** | **16/16** |
| `v7e_stiffcontact_elliptic` | 10/10 valid, 13 mm lateral margin | **24/24** | **16/16** |

Conditions: friction ×0.4–1.5, mass ×0.88–1.15, torso COM ±2–4 mm, initial pitch/roll lean ±2–3°.
Passing runs: 10/10 valid strict steps, 340–357 mm, stance slip ≤ 0.4 mm, torque saturation ≤ 5 % of samples.
**All four failures on the original-contact model are the friction ≥ 1.3 cases** (see Section 7).
Parameter sweep (16 settings): fails with only 50 % wing deployment, 20 mm back-lean, or the fastest timing tried.

Speed is 9–16 mm/s. That is a real limitation: this is a careful quasi-static walk, not a brisk gait.

## 7. Ground-contact fidelity finding (results/p1_contact)

The original export uses MuJoCo's default contact softness. Measured on the original-contact copy:

| Friction × | Pad sink, quiet stand | Single-support result |
|---:|---:|---|
| 0.6 | 0.48 mm | holds, 12.4 mm margin |
| 1.0 | 0.67 mm | holds, margin dips to 6.5 mm |
| 1.3 | 1.16 mm | **rolls over the pad's inner edge** |
| 1.6 | 2.19 mm | rolls over |
| 2.0 | 4.59 mm | rolls over |

Pad sink grows with the friction coefficient, which is a solver artefact, not physics. A 48 mm pad on that soft a
cushion has roll stiffness close to m·g·h (0.95 Nm/rad for a 187 mm COM height), so one-foot stance is nearly unstable.
With contact 25× stiffer (`solref 0.004`, 3 changed XML lines, nothing mechanical) the pad sinks 0.02–0.4 mm and single
support holds at every friction level with 10–13 mm margin. This also means earlier V2–V4 conclusions about one-leg
balance were partly measuring the cushion.

## 8. Decision log

**PROVEN (in simulation, measured here)**
- Quiet-standing feedback is intact; V4 `stationary_control` was a knee-wiggle test.
- Tail and neck have no useful lateral authority; one deployed wing has 24.6 mm.
- One ankle servo at 0.2 Nm cannot hold pitch in single support at the standard stance; a 30 mm back-lean fixes it.
- The existing joints complete genuine single steps and ≥ 10 alternating steps with ≥ 0.25 m progress.
- The original model's default contact softness makes single support fail at friction ≥ 1.3.

**HYPOTHESIS (not yet shown)**
- A real wing fold mechanism can sweep a 53 g wing fully in ~0.7–1 s, repeatedly, and hold it level (the model's sweep
  servo is 0.3 Nm with ideal fold coupling; flap servo holds the wing horizontal).
- Stiff contact represents the real sole better than the default. The CAD sole pad is 20 × 44 mm but the MuJoCo collider
  is 48 × 128 mm — the real support width needs confirming.
- A dedicated hip-roll or ankle-roll joint would allow a faster dynamic gait. Not needed for a first walk; untested.

**NOT TESTED**
- Hopping and jumping (Phases 2–3) — not started. Rough ground, slopes, pushes during walking, turning.
- Servo speed limits, backlash, sensor noise, self-collision of a deployed wing with the legs/body.
- Any change to the Blender model.

## 9. Requests for your approval

1. **Adopt `v7e_stiffcontact` as the working simulation baseline** for the next phases (original stays untouched).
2. **Confirm the real foot contact footprint** (20 × 44 mm sole pad vs the 48 × 128 mm collider). If the true pad is
   20 mm wide, the lateral margin shrinks and I would re-run everything on a corrected copy before going further.

## 10. Reproduce

```
& "D:\Downloads\Sparrow_Locomotion_Engineering_V1\launchers\View-Sparrow.ps1"            # live 10-step walk
& "D:\Downloads\Sparrow_Locomotion_Engineering_V1\launchers\View-Sparrow.ps1" --list     # all demos incl. negative controls
& "D:\Downloads\Sparrow_Locomotion_Engineering_V1\launchers\Run-All-Experiments.ps1"     # every experiment + hash check
& "D:\Downloads\Sparrow_Locomotion_Engineering_V1\launchers\Verify-Originals.ps1"
```

Layout: `src/` code · `src/vendor/` byte-identical copies of your controller · `model/` copied models + variant manifests ·
`results/` CSV traces, per-step event JSON, summaries · `manifests/` hashes and copy provenance.

---

# Phase 2 addendum — stowed-wing walking and low hops (13 Nov 2026)

All on the `v7e_stiffcontact` copy (owner-approved working baseline), full physics with aerodynamics on, one simulation at a time.
Originals re-verified: 1,235 of 1,237 unchanged. The two that differ are `Sparrow_V7E_HeadWings.blend` / `.blend1`: the master
was saved from Blender at 23:01:31 on 12 Nov (not by this workspace); the earlier version survives byte-identical as `.blend1`.

## A. Stowed-wing walk "QuickStep-A" (src/run_gait.py, results/r2_fast, r3_turn)

Wings stay folded. Quick alternating steps (3.5 Hz per leg, 30 mm stride, 12 mm lift, 20 mm back-lean): the body tips
sideways only slowly when a foot lifts, so short swings land before it matters. No lateral actuator is used.

| Gait | Wings | Speed | Development | Holdout |
|---|---|---:|---:|---:|
| WingWalk-A (preserved) | one fully out per step | 14 mm/s | 24/24 | 16/16 |
| QuickStep-A | stowed | 137 mm/s | 12/12 | 8/8 |

Same gate as before plus stopping and standing at the end. Frequency/stride sweep: upright from 68 to 249 mm/s; falls at
2 Hz with 20 or 50 mm stride and at 4.5 Hz with 50 mm stride. There is no aerial phase, so this is a fast walk, not a run.
Steering by stride asymmetry: about 10 deg in 6 s at asymmetry 0.4, 36 deg at 0.6, falls at 0.8 and above.
Leg command slew is 10 rad/s here (3 rad/s in WingWalk-A); real servo speed is NOT yet confirmed.

## B. Hop controller (src/hop.py) and spring model

Existing leg joints only. The knee spring is modelled as a finite-energy one-sided torsion spring (E per knee set per hop,
0.24-0.32 J total), applied through the placeholder `jump_spring_*` actuators; it stays attached through landing and
cushions it. The first version of the model let the spring detach before landing in some conditions; those results are
archived as `library_v1_springdisengages_SUPERSEDED.json` and not used. Parameters were found by simulation search
(src/h2_opt.py) against three conditions at once, then validated separately (src/h3_validate.py).

Reliable forward range: about 0-60 mm per hop, 25-37 mm two-foot clearance, ~0.15 s flight. Targets inside the range
landed within 2 mm nominally except the 10 mm target (16 mm). Single-hop library validation: 35/36.

## C. The five hop objectives (src/h4_scenarios.py, results/h4_scenarios)

PASS = no fall anywhere in the episode, every planned hop >= 15 mm two-foot clearance with a single flight phase, no
body contact, upright afterwards, no unplanned flights.

| Objective | Scenario | Development (7) | Holdout (5) |
|---|---|---:|---:|
| 1 standstill to target | 25 mm | 7/7 | 4/5 |
| 1 standstill to target | 55 mm | 7/7 | 5/5 |
| 2 walk then hop | WingWalk-A 5 steps, 50 mm hop | 7/7 | 5/5 |
| 3 "run" then hop | QuickStep-A ~630 mm, stop, 50 mm hop | 7/7 | 5/5 |
| 4 three in a row | 3 x 45 mm | 7/7 | 5/5 |
| 4 three in a row | 20 / 58 / 40 mm | 6/7 | 4/5 |
| 5 left / right | steer left with QuickStep-A, stop, hop | 7/7 | 4/5 |
| 5 left / right | steer right, stop, hop | 7/7 | 5/5 |

Failures: three are the shortest hop taking an extra small bounce on low friction (no fall). One is a real fall: steering
left with mass x1.10 and COM -2 mm fell during the curved gait at 4.98 s, before the hop.
Steer-then-hop ends 62-144 mm to the chosen side. It needed a 1.4 s settle before the hop (0.6 s fell once on low friction).

**Not achieved:** a single hop that travels sideways. The robust tuning moves only 5-10 mm laterally (hop_left 3/7,
hop_right 3/7, zigzag 1/7 against an 8 mm minimum); an earlier tuning reached 19 mm but fell once. A turning hop has no
yaw authority at all with pitch-only legs.

## D. Jump feasibility, arithmetic only (src/j0_feasibility.py)

> **Correction, 13 Nov 2026:** the 2x / 3x standing-height figures below came from a misread request and are **not design
> requirements**. The owner's targets are: normal hops 30-60 mm clearance, higher hops 60-100 mm, larger jumps undecided.
> The table is kept only as reference arithmetic. Nothing is to be redesigned around 678 or 1,017 mm.

Standing height 339 mm, mass 0.520 kg, maximum leg stroke 95 mm (hip-ankle 74 to 169 mm).

| Target | Foot clearance | Energy | Take-off speed | Mean leg force, full stroke | Mean power |
|---|---:|---:|---:|---:|---:|
| low hop today | 30 mm | 0.15 J | 0.8 m/s | 7 N (1 BW) | 3 W |
| 2x height | 678 mm | 3.5 J | 3.7 m/s | 41 N (8 BW) | 75 W |
| 3x height | 1,017 mm | 5.2 J | 4.5 m/s | 60 N (12 BW) | 133 W |

Servos alone: under 1.9 J even at full torque over full travel with no speed limit, so not feasible. A spring is required;
with 50 % margin a 2x jump needs ~52 g steel, ~10 g composite leaf or ~3 g rubber (assumed typical energy densities) and
~10 s winding at 1 W shaft power. Leg strength at 41-60 N mean load and 3.7-4.5 m/s landings is unchecked.
NOT TESTED in simulation; added hardware mass not included.

## E. Decision log additions

**PROVEN (simulation):** stowed-wing walking at 68-249 mm/s; low hops to forward targets within 0-60 mm; walk-then-hop,
fast-gait-then-hop, three hops in a row; left/right via steering then hopping.
**HYPOTHESIS:** servos follow 10 rad/s commands; a real one-sided spring + latch + winder behaves like the model.
**NOT ACHIEVED / NOT TESTED:** sideways single hop; true running with an aerial phase; any jump above ~37 mm clearance;
structural loads.

New viewer demos: `quickstep`, `quickstep_fast`, `quickstep_left`, `hop_25`, `hop_55`, `hop_triple`, `hop_mixed`,
`walk_hop`, `run_hop` (press N to cycle).

---

# Phase 2b addendum — higher hops (60-100 mm) and faster walking (13 Nov 2026)

Targets as corrected by the owner: normal hops 30-60 mm clearance, higher hops 60-100 mm, larger jumps undecided.
Everything on `v7e_stiffcontact`, aerodynamics on, at most two simulations at a time. The preserved low-hop controller
(`src/hop.py`) and its library entries were not modified; the new work is in `src/hop2.py` and separate library entries.

## F. Higher hops: height is reachable, controlled landing is not (yet)

| Experiment | Result |
|---|---|
| Raise only spring energy on the robust hop (`h5_diag`) | clearance 47 / 62 / 80 / 99.5 mm at 0.20 / 0.40 / 0.50 / 0.60 J per knee; landing force 34-43 N throughout; nose-up spin grows to 80-120 deg/s, arrives tilted 18-37 deg, falls |
| Search for 60-75 mm (`high40_60_75`) | settles at **47 mm, 9/9 conditions upright**, 0.40 J stored, 0.72 Nm peak spring torque, 35-43 N landing |
| Energy ladder, geometry re-tuned per rung (`ladder_E025..E060`) | 9/9, 8/9, 7/9 at 0.25 / 0.30 / 0.36 J per knee but clearance stays ~44 mm; 3/9, 0/9, 0/9 above |
| Aligned crouch (ankle-hip-COM collinear), hip passive (`h6_aligned`) | worse: 170-210 deg/s spin (torso is an inverted pendulum on a free hip) |
| Aligned crouch + hip pitch PD (`h7_hip_pd`) | at 0.60 J per knee the result is identical for every gain: hips saturated, ~150 deg/s spin, 70-90 mm clearance, falls |
| Push trace at 1 kHz (`h8_pushtrace`) | whole push lasts ~16 ms at 54 N with the centre of pressure at the heel, 70 mm behind the COM; hip and ankle servos pinned at their limits |
| Long-stroke soft spring, same energy (`h9_softspring`) | spin drops to 40-70 deg/s in several settings, clearance 40-57 mm, but all tip over after landing |
| Combined search: soft spring + aligned crouch + hip damping + landing (`aligned_E045`, `aligned_E060`) | **0/18**: 52-59 mm and 71-95 mm clearance reached, arrival tilt as low as 6 deg at 0.45 J, but none stays up |

**PROVEN (simulation):** the existing finite-energy spring approach supplies enough energy for 60-100 mm clearance
(0.8-1.2 J stored, 0.7-1.3 Nm peak knee torque). The robust, repeatable ceiling with a controlled landing is **47 mm**.
**Cause, measured:** pitch control during the push. The only strong actuator is the knee spring; the hip (0.35 Nm) and
ankle (0.2 Nm) servos saturate, so take-off spin cannot be removed, and the landing controller was not able to recover it.
**NOT TESTED:** releasing the spring from the leg after take-off so landing is on servos alone; correcting attitude in
flight by swinging the legs; any mechanical counterfactual (stronger hip servo, ankle nearer mid-foot, spring acting
across hip and knee). No CAD change is proposed on this evidence yet.

## G. Faster stowed-wing walking (`src/r4_speed.py`)

| Gait | Speed | Development | Holdout | Peak joint speed |
|---|---:|---:|---:|---:|
| QuickStep-A (preserved baseline) | 137 mm/s | 12/12 | 8/8 | 8.4 rad/s |
| Candidate M (3.5 Hz, 40 mm stride, 50 % stance) | 207 mm/s | 12/12 | 6/8 | 8.4 rad/s |
| Candidate B (3.5 Hz, 60 mm stride) | 262 mm/s | 12/12 | 5/8 | 12.2 rad/s |

40-setting grid: nothing above ~290 mm/s stays upright; aerial phases appear only in runs that fall (no true run).
Holdout failures accelerate past the set pace (320-395 mm/s) before falling: there is no speed or pitch regulator.
The model does not enforce a joint speed limit; a hobby servo manages roughly 7.5-10.5 rad/s unloaded, so QuickStep-A is
already near that and Candidate B is beyond it. Real servo speed is unconfirmed.

## H. Flight

First-pass feasibility audit (arithmetic only, nothing simulated): `flight_research/F0_FEASIBILITY_AUDIT.md`.
Its lift, power and take-off figures are preliminary calculations and assumed ranges, not demonstrated capabilities.

---

# Phase 2c addendum — controlled landing above 47 mm: software-only study and first hardware variants (14 Nov 2026)

Sign convention used below (corrects earlier wording): **positive pitch = nose down**. The falls described in section F as
"nose-up / backward" are forward tumbles.

## I. Software-only study on the unmodified mechanism (`v7e_stiffcontact`)

| Question | Experiment | Measured result |
|---|---|---|
| Can the legs absorb the drop? | `t1_touchdown.py grid`, 180 imposed touchdown states | Yes. Drop height is not the limit: 30 mm and 100 mm drops recover over the same states; peak force 30 N to 52 N |
| Over what arrival states? | same | Narrow: level to ~5 deg nose-down with little forward speed; at 0.3 m/s forward only when 5-10 deg nose-up. 56/180 overall |
| Can foot placement widen it? | `t7_landing_window.py` | No. Best 5/16; feet 30 mm or more further forward: 0/16 |
| Where does take-off spin come from? | `t2_spin_sources.py` (angular momentum at lift-off) | Spring alone: about +15 to +16 mNms nose-down at every energy. Hip servos chasing a pose cancel most of it; a saturated ankle servo adds to it |
| Can the hips cancel it? | `t3_hip_bias.py` | L = 16 - 33 x (hip torque in Nm). At the 0.35 Nm limit +4.7 to +6.1 mNms remains; about 0.49 Nm would be needed |
| Does a longer, softer spring help? | `t4_soft_bias.py` | No: same residual, less lift-off speed |
| In-flight correction? | `t5_flight_authority.py` | Legs: 9 deg per 0.6 rad swing, 14 deg per 1.0 rad, within 0.15 s. Tail, head, wings: about 1 deg each |
| Why did the 0.45 J attempt fall at 6 deg? | `t1`, `t6` | 6 deg nose-down with forward speed is outside the landing window above |
| 60 mm by tuning the crouch? | `t6_milestone60.py` | No: arrival 9-23 deg nose-down across the sweep |

**Conclusion:** with the existing actuators and foot, software as tested does not give a controlled landing at 60 mm.
The robust ceiling stays at 47 mm (9/9). The preserved controllers were not changed.

## J. Foot geometry check (`g1_foot_geometry.py`)

Measured from the CAD-exported meshes in the model copy (the live Blender session was not reachable for this check).
Lowest part of the foot is the claw set: 49 mm wide x 142 mm long, at z = -23.3 mm below the ankle. The 20 x 44 mm sole pad
sits 0.3 mm higher. The 48 x 128 mm box collider is therefore a fair, slightly short stand-in **if the claws are stiff
enough to carry load** (not verified). The ankle joint is 21-28 mm from the rear end and 107-114 mm from the front.

## K. Hardware counterfactuals in copied models (`make_variants.py`, `v1_variant_compare.py`)

Identical tests, same hop settings on every model, only the hip-torque knob swept to each model's own limit.

| | Original | Hip 0.55 Nm | Hip 0.70 Nm | Ankle +30 mm | Ankle +45 mm | Hip 0.55 + ankle +30 |
|---|---:|---:|---:|---:|---:|---:|
| Lowest take-off spin at 0.50 J per knee (mNms) | +5.1 | +0.3 | -3.0 | +5.3 | +5.0 | -0.6 |
| Arrival tilt at that setting (deg nose-down) | 18.5 | 5.0 | 5.3 | 21.7 | 21.7 | 4.9 |
| Touchdown states recovered at 60 / 80 / 100 mm (of 20) | 4 / 5 / 4 | 4 / 5 / 4 | 4 / 4 / 4 | 10 / 11 / 11 | 9 / 11 / 10 | 9 / 10 / 11 |
| Peak landing force, 100 mm drop | 54 N | 55 N | 55 N | 71 N | 79 N | 71 N |
| Added mass (my estimate, not a selected part) | - | +12 g | +24 g | 0 | 0 | +12 g |

- Hip torque fixes take-off spin and nothing else. 0.55 Nm is enough.
- Ankle position fixes the landing window and nothing else. 30 mm is enough; 45 mm adds nothing and raises impact force.
- Fair A/B search at 0.50 J per knee (same seed, budget, conditions, objective; `h2_opt.py fair`):
  original 0/9, falls at touchdown (tilt to 64-72 deg within 0.1 s); combined variant 0/9, absorbs the touchdown and
  topples forward about 0.8 s later (tilt 47-57 deg; two conditions stayed under 20 deg but bounced). Clearance 49-62 mm.
- Holding the crouch longer, standing up slower or standing with the feet forward made no difference on the combined
  variant (`v2_recovery_check.py`, 0/72).

**Status:** each hardware change does what the physics says, measured in isolation, but **no variant has yet produced a
complete stable 60 mm hop**. The remaining problem is forward speed: cancelling the spin with hip torque also drives the
body forward at 0.45-0.5 m/s, and no landing tested so far arrests that. No CAD change is recommended on this evidence.

**NOT TESTED:** spring attachment geometry (force nearer the centre of mass), added landing compliance, a landing
controller that uses the hips or takes a step, servo speed limits on the stronger hip servos, claw stiffness.
Walking speed/pitch regulator: not started.

---

# Phase 2d addendum — hip-and-knee spring variant: 60 / 80 / 100 mm hops with controlled landing (14 Nov 2026)

## L. What the variant is (`model/v7e_bispring`, `src/hop3.py`, `src/v3_bispring.py`)

A **copied model only**; nothing in the Blender master or the original export is changed and no CAD change is made.
The jump spring spans hip and knee: its stretch follows `q_knee + rho * q_hip` with a fixed `rho = -0.6` (hip pulley 60 %
of the knee pulley). It is still a finite-energy, one-sided spring, slack at the standing pose. Servos stay at the original
0.35 Nm (hip, knee) and 0.2 Nm (ankle); the ankle position is the original. +4 g added to the torso for cable, pulleys and
anchors (estimate, not a designed part). During the push the leg servos follow instead of chasing a pose.

Why it works, measured (`v3_bispring.py sweep / esweep`):
- knee-only spring: +15 mNms nose-down spin at lift-off whatever the energy; with the hip share the same spring also
  extends the hip, which cancels it (0 to +2.5 mNms at 0.80 J per leg in every condition);
- crouching with the feet 35-41 mm further forward puts the centre of mass over the ankle, so the push no longer drives
  the body forward: 0.15 m/s at lift-off instead of 0.45-0.5 m/s when hip servos did the cancelling.

## M. Milestones (full physics, aerodynamics on; `results/v3_bispring/check_v7e_bispring_*.json`)

PASS = one flight phase, two-foot clearance >= 15 mm, no body contact, tilt < 20 deg and COM > 90 mm for 2 s after touchdown.

| Milestone | Energy stored (total) | Clearance | Development | Holdout | Tilt at touchdown (deg, + = nose down) | Max tilt after | Peak landing force | Peak spring torque knee / hip |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| 60 mm (`bi_E080`) | 1.60 J | 59-67 mm | **9/9** | **5/5** | -4.6 .. +3.6 | 14.5 | 50-57 N | 1.54 / 0.92 Nm |
| 80 mm (`bi_E100`) | 2.00 J | 83-93 mm | **9/9** | **5/5** | -9.2 .. +1.4 | 15.6 | 60-78 N | 1.86 / 1.12 Nm |
| 100 mm (`bi_E110`) | 2.20 J | 94-105 mm | **9/9** | **5/5** | -11.0 .. -0.1 | 18.5 | 70-80 N | 2.05 / 1.23 Nm |

Forward travel per hop 15-87 mm. Same spring hardware for all three; between heights only the stored energy, the crouch
trim, the landing pose and a hip-servo trim (0, -0.06, -0.14 Nm) change. Each was tuned by one small search on four
conditions (nominal, friction x0.6, friction x1.4, mass x1.07 with COM -2 mm) and validated separately; the five holdout
conditions were never used for tuning. History: 80 mm first pass 6/9 and 4/5, 100 mm first two passes 0/14 then 6/9 and 2/5,
before the hip-servo trim was added and the search continued.

Comparison on the same milestones: unmodified mechanism 47 mm ceiling (0/9 at 0.50 J per knee); hip 0.55 Nm + ankle +30 mm
variant 0/9; hip-and-knee spring on the forward-ankle copy 1/36 untuned (set aside).

## N. What this does and does not establish

**PROVEN (simulation):** with a spring acting across hip and knee, the existing servos and foot land 60, 80 and 100 mm hops
upright in all tested development and holdout conditions. Stronger hip servos or a moved ankle alone did not.
**HYPOTHESIS / assumptions:** an ideal lossless spring releasing 1.6-2.2 J; the +4 g allowance; a latch and winder that can
store and release it (about 4-6 times the energy of the 47 mm knee-only hop); leg joints and links carrying 2 Nm at the
knee and 1.2 Nm at the hip from the spring and 80 N at landing; claws stiff enough to be the contact surface.
**NOT TESTED:** servo speed limits, friction and losses in the spring path, repeated hops and walk/run-then-hop with this
spring, left/right, the three milestones on rough ground, any structural check. No CAD change has been made or approved.

Viewer: `View-Sparrow.ps1 bi_hop_60 --variant v7e_bispring` (also `bi_hop_80`, `bi_hop_100`).

---

# Phase 2e addendum — detailing the hip-and-knee spring: realism stress tests and mechanism numbers (14 Nov 2026)

All on the `v7e_bispring` copy with the three validated hops (`bi_E080`, `bi_E100`, `bi_E110`), **no re-tuning** unless stated.
Five conditions per cell (nominal, friction x0.6, x1.4, mass x1.07, COM -3 mm). Scripts `v4_detail.py`, `v5_calibration.py`,
`v6_mechanism_numbers.py`, `v7_drag.py`, `v8_drag_calibration.py`; results in `results/v4_detail/`.

## O. Stress tests (good hops out of 5: 60 mm / 80 mm / 100 mm settings)

| Change, nothing re-tuned | 60 mm | 80 mm | 100 mm | Note |
|---|---:|---:|---:|---|
| as validated | 5 | 5 | 5 | |
| spring efficiency 0.90 | 3 | 5 | 5 | clearance drops ~12-15 % |
| spring efficiency 0.80 | 0 | 3 | 3 | |
| spring efficiency 0.70 | 0 | 0 | 0 | |
| efficiency 0.80 with stored energy raised 25 % | 5 | 5 | 5 | losses can be made up with energy |
| hip pulley ratio built -0.55 / -0.65 (8 % off) | 3 / 5 | 3 / 3 | 3 / 2 | |
| hip pulley ratio built -0.50 / -0.70 (17 % off) | 0 / 3 | 0 / 1 | 0 / 0 | |
| stored energy -10 % / +10 % | 3 / 5 | 4 / 3 | 5 / 2 | |
| servo torque falling to zero at 10.5 rad/s | 5 | 5 | 3 | clearance drops ~10-13 mm |
| servo torque falling to zero at 7.5 rad/s | 5 | 5 | 4 | |
| leg-joint drag x0.5 | 0 | - | 0 | overshoots: 81-99 mm and 128-136 mm |
| leg-joint drag x1.5 / x2 / x3 | 1 / 0 / 0 | - | 4 / 1 / 0 | clearance falls to 28-76 mm |

**The result is finely balanced.** In particular it depends on the leg-joint drag, which in the model is a placeholder
(0.02 Nm s/rad) and which dissipates a large share of the spring energy while the push back-drives the joints at
20-30 rad/s. Real back-driven servo drag is unknown.

## P. Can calibration absorb that? (one or two numbers per robot)

| Fault | Re-calibrated with | 60 mm | 100 mm |
|---|---|---|---|
| pulley ratio -0.50 | hip-servo trim only | 5/5 at +0.21 Nm | 5/5 at +0.21 Nm |
| pulley ratio -0.70 | hip-servo trim only | 5/5 at -0.35 Nm (at the servo limit) | **0/5** |
| energy -10 % / +10 % | hip-servo trim only | 5/5 / 5/5 | 5/5 / 5/5 at the servo limit |
| drag x2 | energy x1.30-1.60 + trim | 5/5 (2.1-2.6 J, clearance 51-79 mm) | not run |
| drag x3 | energy x1.6-1.8 + trim | 5/5 (2.6-2.9 J, clearance 45-65 mm) | not run |
| drag x0.5 | energy x0.6-0.8 + trim | **at best 2/5** | not run |

So: a hip pulley share on the low side (-0.50 to -0.60) is trimmable at every height; -0.70 is not at 100 mm. Higher drag
is recoverable with more stored energy; lower drag is not recovered by these two numbers and would need the crouch and
landing re-tuned.

## Q. What a mechanism would have to deliver (from the simulated spring, nominal run)

| | 60 mm | 80 mm | 100 mm |
|---|---:|---:|---:|
| Stored energy, total (lossless) | 1.60 J | 2.00 J | 2.20 J |
| Stored energy if 80 % efficient | 2.00 J | 2.50 J | 2.75 J |
| Spring stretch (knee + 0.6 x hip) | 1.05 rad | 1.08 rad | 1.09 rad |
| Stiffness at the knee | 1.45 Nm/rad | 1.70 Nm/rad | 1.87 Nm/rad |
| Peak spring torque, knee / hip | 1.52 / 0.91 Nm | 1.84 / 1.11 Nm | 2.03 / 1.22 Nm |
| Push duration | 19 ms | 19 ms | 18 ms |
| Peak ground force in the push | 73 N | 86 N | 93 N |
| Peak joint speed in the push, knee / hip | 23 / 21 rad/s | 27 / 23 rad/s | 29 / 25 rad/s |
| Crouch: knee 0 -> 0.65 rad, hip +0.07 -> -0.60..-0.67 rad | | | |

Cable version, 100 mm hop, by knee pulley radius: 6 mm -> 3.6 mm hip pulley, 6.5 mm travel, 338 N peak tension, 52 N/mm;
8 mm -> 4.8 mm, 8.7 mm, 253 N, 29 N/mm; 10 mm -> 6.0 mm, 10.9 mm, 203 N, 19 N/mm; 12 mm -> 7.2 mm, 13.0 mm, 169 N, 13 N/mm.
Spring mass for 2.75 J at assumed usable energy densities: about 28 g steel, 6 g composite leaf, 1.4 g rubber.
Winding 2.75 J at 50 % efficiency: 5.5 s at 1 W shaft power, 2.8 s at 2 W. These are bounded scenarios, not selected parts.

## R. Open risks before any CAD work

1. **Back-driving the leg servos at 20-30 rad/s** during the push. Typical hobby servos run 7.5-10.5 rad/s unloaded and resist
   being driven faster. The result is very sensitive to this drag. Options to investigate: a release that decouples the
   servos during the push, or measuring real drag and designing to it.
2. **Peak loads**: 170-340 N cable tension, 2.0 Nm at the knee, 1.2 Nm at the hip, 93 N on the feet in the push and
   80 N on landing, all in under 20 ms. Leg links, joints and claws are not checked against these.
3. **Latch and winder** are not modelled at all.
4. **Tolerance**: pulley ratio and stored energy need to be right to within a few percent, or calibrated per robot.
5. **Not yet run with this spring**: repeated hops, walk/run-then-hop, left/right, low hops.


## Phase 2f - S. Servo back-drive during the push (13 Nov 2026, simulation only)

Question: the spring turns the knee at ~30 rad/s, about three times a hobby servo's no-load speed, so the leg must drive
the servo gear trains. The validated model had only a placeholder viscous drag (0.02 Nm s/rad) and almost no rotor
inertia (2e-5 kg m^2). Does a realistic servo invalidate the hip-and-knee spring result?

Added to `src/hop3.py` (all default off, validated results unchanged): `arm` reflected rotor inertia, `fric` dry gear
friction, `emf_w0` back-EMF drag = stall torque / no-load speed, `knee_free` knee servo decoupled during the push.
Scripts: `src/v9_backdrive.py`, `src/v10_backdrive_validate.py`. Data: `results/v9_backdrive/`.
Servo values are ranges for small 0.2-0.35 Nm hobby servos, NOT measurements of Sparrow's servos:

| case | rotor inertia at joint | dry friction | no-load speed (back-EMF drag) |
|---|---|---|---|
| light | 1e-4 kg m^2 | 0.02 Nm | 14 rad/s (0.025 Nm s/rad) |
| typical | 3e-4 kg m^2 | 0.035 Nm | 10.5 rad/s (0.033 Nm s/rad) |
| heavy | 1e-3 kg m^2 | 0.07 Nm | 7.5 rad/s (0.047 Nm s/rad) |

**S1. One factor at a time, no re-tuning (5 conditions each).**
- Dry friction up to 0.07 Nm: 5/5 for both the 60 mm and 100 mm hops, 3-8 mm lower. Not a concern.
- Rotor inertia: 1e-4 gives 2/5 (60 mm) and 5/5 (100 mm); 3e-4 and 1e-3 give 0/5.
- Back-EMF drag: 14 rad/s gives 2/5 and 4/5; 10.5 rad/s 0/5 and 2/5; 7.5 rad/s 0/5.
- All three together: 0/5 except light at 100 mm (1/5). Clearance falls to 49-53 / 35-40 / 19-22 mm (60 mm hop).
- Failure is mostly the return of nose-down spin at take-off (+5 to +7 mNms against about 0 validated), not lost height.
- Even the validated model loses 0.66 J of the 2.0 J the spring delivers to the placeholder drag.

**S2. Re-calibrated with two numbers (stored energy, hip-servo trim), same hardware geometry.**

| servo case | 60 mm hop | 100 mm hop |
|---|---|---|
| light | 1.84 J (x1.15), trim +0.07 Nm: 5/5, 57-62 mm | 2.5-2.9 J (x1.15-1.3): 5/5, 91-119 mm |
| typical | 2.40 J (x1.5), trim +0.21..+0.35 Nm: 5/5, 58-63 mm | 3.30 J (x1.5), trim +0.21..+0.35 Nm: 5/5, 95-102 mm |
| heavy | not recovered at 60 mm; first 5/5 is 4.8 J giving 77-81 mm | 5.7 J: 5/5 at 87-91 mm; 6.6 J: 4/5 at 92-95 mm |

Full check of the typical case at trim +0.28 Nm: 60 mm hop 9/9 development, 5/5 holdout (57-62 mm);
100 mm hop 9/9 development, 5/5 holdout (95-103 mm).

**S3. New costs that the re-calibration exposes.**
- Landing force rises from 57-80 N to 165-229 N (typical servo): the rotor inertia makes the legs stiff at touchdown.
  That is 32-45x body weight through the servo gears. Not checked against any gear or link strength.
- Peak knee spring torque reaches 2.3 Nm (60 mm) and 3.0 Nm (100 mm); 3.0 Nm is the limit of the model's spring channel,
  so heavier cases are partly clipped by it.
- Hip trim sits at +0.21..+0.35 Nm, i.e. close to the 0.35 Nm servo limit, so little trim margin remains.
- Energy rises 15% (light), 50% (typical), 150-200% (heavy).

**S4. Knee servo decoupled during the push.** Without re-tuning the hop overshoots (120-160 mm from the 60 mm settings),
and the two-number calibration found 0/5 at every energy (0.45-0.9x) and trim. Clearance is there at half the energy
(94-105 mm from 1.04 J), but the landing fails every time. The spring's hip/knee split and crouch position were tuned
with knee drag present and would need re-optimising for a decoupled knee; that was not done.

**Limits.** Servo numbers are assumed ranges. The back-EMF model assumes the motor circuit stays closed for the whole
hop. Results show a small run-order sensitivity (0.3 mm clearance) in the simulation harness.


## Phase 2g - T. Decoupled knee against coupled knee at equal stored energy (14 Nov 2026, simulation only)

Typical assumed servo back-drive on every leg servo (rotor 3e-4 kg m^2, dry friction 0.035 Nm, no-load 10.5 rad/s).
Decoupled = one-way coupling (`knee_free=2` in `src/hop3.py`): the knee runs ahead of its servo from spring release, a
weak 0.05 Nm return acts on it, and the servo and rotor load the joint again only once the knee has stopped extending,
so no energy is created at re-engagement. `knee_free=3` keeps the knee off the servo through touchdown as well.
Scripts: `src/v11_kneefree.py`, `src/v12_compare.py`. Data: `results/v11_kneefree/`, `results/v12_compare/`.
Gear-train torque = motor torque + back-EMF drag + dry friction + rotor inertia reaction. Servo stall: hip/knee 0.35,
ankle 0.2 Nm. 5 conditions per row, best hip-servo trim.

**T1. Why the first decoupled attempt failed.** With the knee free, the validated spring split (-0.6) leaves +13 to
+15 mNms nose-down spin, even with ideal hips and ankles. In the coupled robot the knee servo's drag was absorbing
roughly 1 Nm of knee torque late in the push, so the effective split was already hip-heavy. A decoupled knee needs
a split of about -1.2 to -1.8 (hip pulley larger than knee pulley), and zero spin then comes with 0.35-0.45 m/s forward
speed: every decoupled hop found travels 80-150 mm forward.

**T2. Like for like.**

| stored | knee | good | clearance | forward | take-off spin | lost in gear trains during push | spring peak knee / hip | gear-train peak in push hip / knee / ankle | gear-train peak at landing | touchdown force |
|---|---|---|---|---|---|---|---|---|---|---|
| 1.6 J | coupled | 3/5 | 32-36 mm | 3-9 mm | +4.7 | 0.84 J | 1.5 / 0.9 Nm | 0.95 / 1.25 / 0.43 Nm | 0.34 / 0.71 / 0.43 Nm | 106 N |
| 1.6 J | decoupled (-1.8) | 5/5 | 44-53 mm | 79-96 mm | -2.8 | 0.69 J | 1.0 / 1.8 Nm | 1.87 / 0.11 / 0.41 Nm | 0.58 / 1.32 / 1.84 Nm | 82 N |
| 2.0 J | coupled | 5/5 | 45-49 mm | 21-31 mm | +3.5 | 1.10 J | 1.9 / 1.2 Nm | 1.10 / 1.57 / 0.59 Nm | 0.38 / 0.83 / 0.49 Nm | 125 N |
| 2.0 J | decoupled (-1.4) | 3/5 | 72-82 mm | 111-137 mm | -1.5 | 0.95 J | 1.5 / 2.0 Nm | 2.19 / 0.18 / 0.62 Nm | 0.67 / 0.55 / 1.21 Nm | 102 N |
| 2.4 J | coupled | 5/5 | 58-63 mm | 37-49 mm | +2.6 | 1.33 J | 2.3 / 1.4 Nm | 1.24 / 1.87 / 0.65 Nm | 0.81 / 1.66 / 0.53 Nm | 159 N |
| 2.4 J | decoupled (-1.2) | 3/5 | 104-118 mm | 91-120 mm | +1.3 | 1.14 J | 1.7 / 2.1 Nm | 2.20 / 0.20 / 0.68 Nm | 0.72 / 2.46 / 2.36 Nm | 190 N |
| 2.4 J | decoupled, knee off servo through touchdown | 5/5 | 93-106 mm | 92-148 mm | +3.6 | 1.06 J | 1.7 / 2.1 Nm | 2.27 / 0.19 / 0.58 Nm | 2.02 / 2.30 / 3.32 Nm | 218 N |
| 3.3 J | coupled | 5/5 | 95-102 mm | 23-47 mm | +2.2 | 1.88 J | 3.1 / 1.8 Nm | 1.62 / 2.47 / 0.84 Nm | 0.72 / 1.38 / 0.67 Nm | 223 N |

Full check of the 2.4 J "knee off servo through touchdown" setting: 8/9 development, 3/5 holdout (93-109 mm).
The coupled 3.3 J hop remains the only 100 mm setting that passes 9/9 and 5/5.

**T3. What it shows.**
1. Height per joule: decoupling gives the same height for about 27% less stored energy (100 mm from 2.4 J against
   3.3 J; 45-50 mm from 1.6 J against 2.0 J).
2. Gear loss: about half the delivered spring energy goes into servo gear trains in BOTH arrangements (0.7-1.9 J).
   Decoupling the knee only moves the loss to the hip, because the hip-heavy split drives the hip harder.
3. Push loads: decoupling takes the knee gear train from 1.3-2.5 Nm down to 0.1-0.2 Nm, but the hip gear train rises
   to 1.9-2.3 Nm. Either way one gear train sees 4-7x its servo's stall torque.
4. Touchdown: force follows hop height, not the knee arrangement: about 80-125 N at 45-50 mm, 160 N at 60 mm,
   190-240 N at 100 mm. Decoupled landings put 1.2-3.3 Nm through the ankle gear train (stall 0.2 Nm).
5. Reliability: the decoupled knee has narrow working settings and always travels forward; it is not yet as robust as
   the coupled, re-calibrated hop.

**T4. Reading.** The decoupled knee alone does not deliver a 100 mm hop with plausible loads. The loads come from two
things it does not address: rotor inertia and back-EMF in the HIP and ANKLE servos during the push, and a stiff leg at
touchdown. Candidates that address those (not tested): a one-way coupling or series spring on the hip as well; a
compliant element between each servo and its joint (series elastic) so the rotor is not jerked; a softer, longer
landing stroke or compliant foot. All servo numbers remain assumed ranges; no CAD change has been made.


## Phase 2h - U. Compliant (series-elastic) servo couplings (14 Nov 2026, simulation only)

A spring between each leg servo's output and its joint. The servo (motor, gears, rotor inertia 3e-4 kg m^2, back-EMF,
dry friction: the assumed "typical" case) is its own state and reaches the joint only through the coupling; its
position loop closes on its own shaft. Knee coupled, validated spring split (-0.6). Gear-train torque = coupling
torque. Code: `sea_k` option in `src/hop3.py` (off by default; baseline reproduces exactly), `src/v13_compliance.py`.
Data: `results/v13_compliance/`. Only stored energy and hip-servo trim were re-tuned; the controller was not.

| coupling | stored | development | holdout | clearance | touchdown force | gear-train peak in push hip/knee/ankle | at landing |
|---|---|---|---|---|---|---|---|
| rigid | 2.40 J | 9/9 | 5/5 | 57-62 mm | 167 N | 1.24 / 1.87 / 0.65 Nm | 0.81 / 1.66 / 0.53 Nm |
| 6 Nm/rad | 1.92 J | 9/9 | 5/5 | 52-59 mm | 53 N | 1.16 / 0.67 / 0.16 Nm | 0.45 / 0.48 / 0.28 Nm |
| 6 Nm/rad | 2.40 J | 9/9 | 4/5 | 70-81 mm | 69 N | 1.44 / 0.81 / 0.19 Nm | 0.52 / 0.59 / 0.34 Nm |
| rigid | 3.30 J | 9/9 | 5/5 | 95-103 mm | 229 N | 1.62 / 2.47 / 0.84 Nm | 0.72 / 1.38 / 0.67 Nm |
| 30 Nm/rad | 3.30 J | 9/9 | 5/5 | 92-109 mm | 137 N | 1.70 / 1.43 / 0.45 Nm | 0.90 / 1.79 / 0.99 Nm |
| 12 Nm/rad | 3.30 J | 7/9 | 4/5 | 95-106 mm | 94 N | 1.73 / 1.36 / 0.26 Nm | 0.61 / 1.16 / 0.67 Nm |
| 6 Nm/rad | 2.86 J | 7/9 | 2/5 | 97-110 mm | 83 N | 1.57 / 0.96 / 0.22 Nm | 0.46 / 0.83 / 0.41 Nm |

Five-condition sweeps (not full checks): 3 Nm/rad gives 50-56 mm from 1.6 J at 46 N (5/5) but 0/5 at 2.2 J and above.

**Reading.**
1. Touchdown force falls with softer couplings: 100 mm hop 229 -> 137 -> 94 -> 83 N; 55-60 mm hop 167 -> 53 N.
2. Softer couplings waste less spring energy: 55-60 mm from 1.92 J instead of 2.40 J; 100 mm from 2.86 J instead of 3.30 J.
3. Reliability at 100 mm falls as the coupling softens (5/5 holdout at 30, 4/5 at 12, 2/5 at 6 Nm/rad). The hop
   controller was written for rigid servos; this may be recoverable by re-tuning it, which was not attempted.
4. Up to about 75 mm, the 6 Nm/rad coupling is as reliable as rigid with a third of the touchdown force.
5. Not fixed: the hip gear train still carries 1.2-1.7 Nm in the push (stall 0.35 Nm) in every case, because the
   hip spring cable acts on the joint and drags the hip servo through the coupling faster than it can turn.
6. Landing gear-train peaks stay above stall (0.45-1.8 Nm against 0.35) but are transient coupling torques.

**Limits.** Coupling deflection reaches 0.06 rad (30), 0.14 (12), 0.25 rad (6 Nm/rad): 3-14 degrees of joint give that
walking has not been tested with. Coupling damping is an assumed 0.01 Nm s/rad. Servo values are assumed, not measured.

## V. Parts and mass (14 Nov 2026, desk study)

`parts/LEG_SERVO_SHORTLIST.md`, `parts/BOM.md` (built by `parts/bom.py` from the CAD mass register). Candidate parts
give about 529 g including 36 g of items missing from the CAD register, against 519 g used in simulation (563 g with
Dynamixel hips and knees). Flight numbers barely move with that mass; the blocking items are unchanged: no thrust
mechanism, undefined flap gearbox. A 100 mm hop gives 0.30 s of air time and 1.5 m/s; wing-borne flight needs ~6.5 m/s.


## W. Take-off study, first pass (15 Nov 2026, calculation only)

Full write-up: `flight_research/F1_TAKEOFF_STUDY.md`; model `flight_research/f1_wing_model.py`. A blade-element wing
model with prescribed twist (no twist mechanism exists in CAD). Checked first: zero twist gives zero thrust; level cruise
costs 11-13 W aerodynamic at 9-10 m/s and 3.5-4 Hz. A first version without leading-edge suction was wrong and discarded.
- From a 100 mm hop with wings already open: back on the ground in 0.3-0.7 s unless hovering (10 Hz, 110 W, 10.6 Nm/wing).
- Hovering needs 65-190 W aerodynamic and 6.5-10.5 Nm per wing at any flap amplitude: out of reach for this wing.
- Thrown at 8-10 m/s it flies from 4-5 Hz on 13-16 W. Drops of 1-3 m were not enough.
- So the jump cannot be the launch; flight needs 8-10 m/s from a throw, launcher or long drop, or a wing redesign.
- Not modelled: pitch stability, tail, flexibility, unsteady effects. All coefficients assumed.
- Redesign sizing (F1 study section 7): jumping higher cannot close the gap (9 m/s needs ~21 J and ~43 g over 95 mm of stroke). Self take-off needs a combination: about 400 g or less, about +-60 deg flap stroke, half-inertia wings and large wing twist; with twist limited to 45 deg only a ~250 g variant fits. Model is least reliable at large twist.


## Phase 3 - X. Game, brain and repository (15-17 Nov 2026, simulation only)

Code: `game/` (game, Studio, pilot, flight, get-up, parameter trainer), `game/brain/` (reinforcement learning).
Repository: https://github.com/azwoodd/Sparrow. Rules for contributors: `AGENTS.md`.

**X1. Flight in the game (free body, nothing holds the attitude).** Uses assumed changes (wing x3 chord, tail x8 area,
wing 15 mm aft, prescribed twist, 6 Nm flap drive). Findings: (a) the player's turn input had never reached the flight
controller (my bug; the "lean right" was the model's own drift); (b) roll authority measured per effector in 1 s of
steady flight: sweeping one wing back 0.3 rad rolls ~110 deg, +-5 deg differential incidence ~73 deg, 30% flap-size
difference ~1 deg, tail roll ~0 - so roll now uses the existing wing-sweep servos; (c) steering on body yaw rate fails
because the body yaws +-70-140 deg/s with each wingbeat; steering on the flight path works. Result: course held to
~2 deg/s with no input, turns ~43 deg/s left / ~36 deg/s right (radius ~13 m), on mass +-7% and balance +-3 mm.
Course hold and sweep roll act only above 7 m/s: below that they upset the dive off the launch tower.
Tower take-off (12 m): 5/5 conditions with the walking brain on, 4/5 without. Landing: fails (5-7 m/s touchdown).

**X2. Ground turning needs a hip-twist joint.** Pitch-only legs: 5 s of full steering bends the path a few degrees;
training turn-on-the-spot ended at -6 deg. Model copy `v7e_game_hipyaw` (+2 hip-yaw servos, +19 g, PROPOSED): at
0.12 rad twist per step, 300-340 deg in 5 s walking, slow or on the spot, 18/18 upright (heading tracked continuously;
an earlier wrapped reading had suggested the opposite direction).

**X3. Steps and get-up (scripted).** The 20 mm step cannot be walked up: 128 mm foot, ~30 mm stride, the toe meets the
riser at any foot lift tried (12-30 mm). Parameter search of a jump-onto-step while walking: 0/5. Scripted get-up
(wing push, crouch found by search, slow rise) reaches ~34 deg from side and backward falls, then stalls; it cannot
right itself from upside down. Parameter search did not improve it.

**X4. Brain (PPO, stable-baselines3).** Environments `getup`, `recover`, `step_up` in `game/brain/sparrow_env.py`.
Ground training runs with air forces off (they cost ~90% of run time; 2,770 vs 283 control steps/s). Every brain is
judged with air forces ON on unseen seeds (`evaluate.py`).
- `recover` (corrections on the walking controller, random commands and 1-4 N shoves): 250k steps, 5 min-scale run on
  2 workers. Unseen runs, full physics: brain 20/20 survive vs scripted 16/20. In the game routine: used while walking
  and standing 3/5 (worse - it never saw landings); used while walking only 5/5 (scripted 4/5). Pinned in skills.json.
- Brains see torso height above the ground they stand on (on the 12 m tower the absolute height was 60x anything seen
  in training and the brain misbehaved).
- `getup` first run (1.2M steps): 0% success and a reward exploit - the brain bounced in the air for height points
  (end state: torso 154-171 mm, tilt 30-94 deg, no ground contact in 4/5). Reward fixed (height counts only with
  weight on the feet; airborne penalised). Retrained 1.5M steps (`runs/getup_20261117_231647`): return rose -219 -> +800
  but success is still 0%: it now ends on its side (79-91 deg) propped on one leg with ~0.5 body weight on that foot -
  a partial-credit position, not standing. Evaluation, full physics, 12 unseen falls: brain 0% (return 809) vs scripted
  0% (return 43). Not enabled in the game. Next: curriculum from near-upright starts, success-weighted reward, far more
  steps (cloud).
