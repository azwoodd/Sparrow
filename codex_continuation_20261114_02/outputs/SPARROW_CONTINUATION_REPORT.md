# Sparrow flight continuation — 14 November 2026

## Finding

Full-feature self-takeoff remains a plausible research direction, not an established capability. Larger wings reduce calculated power at 529 g. No candidate tested here was robust across the tested aerodynamic/twist envelope and reduced launch efficiency. No master CAD, controller, actuator limit or original flight code was changed. Nothing has been physically tested.

## Recovered state

Read the complete HANDOFF_FLIGHT_STUDY.md, the latest flight section of REPORT.md, BOM mass roll-up and original f1/f3/f4 implementation. The handoff is internally dated 15 November although the client date and file timestamps are 14 November; this report uses 14 November.

- Both f4 grids and the timing study are complete, with JSON outputs. At 529 g, area x1.5 reaches its best recorded 68.7 W aerodynamic / 7.0 Nm peak per wing; area x2 reaches 54.1 W / 6.4 Nm. These are five-second, prescribed-motion model outcomes, not motor-qualified flight.
- Launch-map log is incomplete: rows through 1.00 m exist; the final 1.50 m row and expected result JSON are absent. No simulation process was detected before work. The partial table must not be labelled a completed experiment.
- Existing BOM total is 529 g INCLUDING 36 g of missing-item allowances. The +30/+60 g cases here represent additional provisional jump-mechanism allowances, not a second addition of those 36 g. They are not engineered mass budgets.

## What was executed

286 deterministic trajectory evaluations: baseline: 1, screen: 108, sensitivity: 27, dt_check: 1, duration_check: 1, launch_efficiency_check: 1, extended_convergence: 27, extended_uncertainty: 96, extended_launch: 8, symmetry_original: 8, symmetry_corrected: 8.

Screen: area x2/x2.5/x3; mass 529/559/589 g; stroke ±45/±60 degrees; spring energy 10/13 J; frequency 3.5/4/4.5 Hz. Fixed 45-degree launch, 50-degree tip twist, prescribed pitch 40→15 degrees, 0.15 s flap delay, 80% jump efficiency. This is a bounded screen, not an exhaustive optimum search. The original baseline reproduced 54.0717 W and 6.3546 Nm per wing.

Extended checks use 15 s trajectories and time steps 2, 1 and 0.5 ms on the three lowest-power forward-flight candidates per mass. Eight survived all three nominal checks. Their uncertainty tests use CN 1.2/2.0, suction efficiency 0.5/0.9, and prescribed twist 35/50/65 degrees: twelve combinations per candidate. These are deterministic sensitivity cases, not probabilistic reliability estimates.

Success retains the legacy criterion: no ground return and last-second mean climb rate greater than −0.15 m/s. An additional forward-flight flag requires final forward speed above the model’s CL=1 stall-speed estimate. Neither test establishes controlled, stable flight or landing. A 15-second pass is not indefinite endurance.

## Extended nominal results (original model, 1 ms)

All rows use a 13 J spring, 80% launch efficiency, 50-degree twist, existing wing inertia and prescribed pitch. Aerodynamic power is positive aerodynamic work averaged over active flapping; mechanical power is positive combined inertial-plus-aerodynamic hinge work, without regeneration or an energy-return spring. Peak torque is at ONE wing hinge, not at the motor shaft. All power columns refer to BOTH wings.

| Mass g | Area scale | Stroke ±deg | Hz | 15 s result | Aero W | Positive mech W | Peak mech W | Peak hinge Nm | Uncertainty passes |
|---:|---:|---:|---:|:---|---:|---:|---:|---:|:---|
| 529 | 3.0 | 45 | 3.5 | airborne | 34.0 | 37.4 | 120.3 | 4.10 | 2/12 |
| 529 | 2.5 | 45 | 4.0 | airborne | 43.3 | 49.7 | 173.2 | 5.53 | 5/12 |
| 529 | 3.0 | 45 | 4.0 | airborne | 48.0 | 54.2 | 188.5 | 5.94 | 6/12 |
| 559 | 3.0 | 45 | 3.5 | fell | 40.4 | 43.4 | 142.9 | 4.63 | not advanced |
| 559 | 2.5 | 45 | 4.0 | airborne | 44.8 | 50.7 | 175.3 | 5.55 | 4/12 |
| 559 | 3.0 | 45 | 4.0 | airborne | 49.3 | 55.3 | 190.0 | 5.96 | 5/12 |
| 589 | 2.5 | 60 | 3.5 | airborne | 67.2 | 73.2 | 263.9 | 7.00 | 4/12 |
| 589 | 3.0 | 45 | 4.5 | airborne | 67.8 | 75.8 | 267.6 | 7.51 | 8/12 |
| 589 | 3.0 | 60 | 3.5 | airborne | 74.7 | 80.3 | 289.3 | 7.59 | 7/12 |

The 559 g, x3-area, 3.5 Hz case passed the five-second screen but returned to ground at 8.284 s with the original 2 ms step. It also failed with the smaller time steps. This is why the earlier short-horizon label “sustained flight” is insufficient. The 529 g counterpart remained airborne for 15 s at all three steps, with final speed about 6.5 m/s and aerodynamic power about 34 W.

All eight extended candidates failed when jump efficiency was reduced to 50%, retaining their selected settings. This does not prove no 50%-efficient design can work: it demonstrates that these candidates rely on the optimistic launch assumption. Their 13 J jump mechanism has not been designed or demonstrated in the leg model.

## Model audit and isolated symmetry experiment

The source doubles the left-wing force and comments that lateral force cancels, but it leaves the doubled lateral component in the force vector used for induced-drag magnitude. An isolated variant zeroes that component before this calculation; the original stays untouched. This enforces the stated mirrored-pair resultant, but it does not validate the induced-drag closure for flapping wings. See symmetry_experiment.diff.

Eight paired original/variant trajectories were run for 15 s at 1 ms. At 559 g, area x3, 3.5 Hz and CN=1.6, the original falls while the variant passes at approximately 34.7 W and 4.22 Nm. At CN=1.2 the same configuration still falls. The qualitative outcome depends on model formulation as well as uncertain coefficients. Do not merge this one-line variant and declare the robot flight-ready.

Other unresolved model limits: prescribed pitch has no attitude dynamics or control torque; prescribed twist has no designed mechanism or work budget; wings are already deployed; no ground/wingtip clearance, launch reaction torque or realistic flapping spin-up; no flexible-wing effects; no motor torque-speed envelope, transmission kinematics, heating, saturation or battery sag. The area changes also scale BODY_CDA in set_wing, coupling body drag to wing area. No correction factor was applied to power to force agreement with earlier reference-robot claims.

## Mechanical implications

At x3 area the pair has 0.498216 m² area on the same 1.42 m span, about 351 mm mean chord. Holding inertia at 0.0049 kg m² per wing is an unverified construction constraint. Keeping mass fixed while tripling area does not supply a feasible structural design.

For the 529 g, x3, 3.5 Hz nominal 15-second trajectory: positive mechanical input is about 37.4 W average and 120 W instantaneous peak; hinge torque is about 4.10 Nm peak and 2.27 Nm RMS per wing. At an ASSUMED 65% aggregate motor/transmission efficiency, average electrical input would be about 57.5 W (~7.8 A at nominal 7.4 V), excluding electronics and launch charging. This is a scenario calculation, not a motor/battery rating or purchase recommendation. Do not convert instantaneous peak power into a continuous rating.

Inertia scaling was evaluated by recomputing loads along the SAME prescribed trajectory at 100%, 50% and 25% inertia. It does not change the trajectory because no actuator saturation exists. It is not evidence that lighter wings have been designed. In the reproduced x2-area baseline, reducing inertia to one quarter lowers peak hinge torque only from 6.35 to 4.61 Nm: aerodynamic torque remains substantial.

An ideal resonance estimate at 3.5 Hz is k=I(2πf)²≈2.37 Nm/rad per hinge. At ±45 degrees that is about 0.73 J maximum stored energy per wing. This is not a designed spring or proof of reliable energy recovery: linkage, off-resonance operation, losses, root loads and mass remain unresolved.

A 13 J spring at 80% conversion supplies 10.4 J translational energy. At 529 g and 45 degrees, both forward and vertical launch speeds are 4.43 m/s. If useful acceleration travel were 95 mm, the kinetic-energy term alone requires about 109 N average net force, before gravity/losses; geometry and peak forces could be worse. Existing ~100 mm-hop simulations do not validate this launch. The stronger spring must be mechanically isolated from fragile servo gear trains and its loads validated in a new model before incorporation.

## Next physical validation plan — no purchases or CAD edits

1. Instrument a restrained, guarded paired-wing rig with independent lift/thrust channels. Use rigid mounts, calibrated multi-axis load measurement, stroke encoder, synchronized voltage/current logging and high-speed video of twist and deflection. Select load ranges from inertial and aerodynamic peaks, not robot weight alone. The 529 g target weight is 5.19 N, but instantaneous fixture loads can be much larger.
2. Measure several complete wingbeats after settling; subtract fixture tare, characterize cross-axis response, compare inertial reference tests, and synchronize force with stroke phase. A blank or surrogate wing can help estimate inertial contamination but must match mass distribution. Cycle means suppress ideal periodic inertia; vibration, drift and cross-coupling still need characterization. Select sampling and anti-alias filtering to resolve wingbeat harmonics; a slow scale readout alone cannot resolve peaks.
3. Measure both useful force components, actual twist, wing-root strain/load, motor/gear temperatures, input power, and loaded frequency. Check torque-speed operation and transmission crank geometry before choosing a motor. A nominal motor wattage or a fixed guessed gear ratio is insufficient.
4. Measure real launch efficiency and achievable impulse using a separate guarded leg/spring rig: stored energy, force-displacement curve, takeoff speed and angle, rotational impulse, servo back-drive, latch release, and landing loads. Test modest energy first. A 13 J mechanism is a proposal, not an approved CAD change.
5. Feed measured force, loss, inertia and launch data into a coupled attitude/actuator-limited model. Then repeat uncertainty tests and extend to controlled landing. Preserve walking, hopping and launcher baselines. Do not strip companion features on the basis of these simplified tests.

## Resources and preservation

Maximum sampled CPU: 20.25%; maximum RAM: 60%. Sampling interval: one second. Each batch had an 85% termination threshold and a ten-minute limit; no guard tripped. One simulation worker per batch; the end of the extended batch briefly overlapped the symmetry batch (two simulation workers total). Numerical library thread limits were set to one, with below-normal process priority. One-second sampling cannot guarantee the absence of shorter system-wide spikes.

Source hashes for the inspected flight code and model inputs matched before/after every batch. The project’s Verify-Originals.ps1 checked 1,237 original files: exactly the two documented Blender files changed, zero missing, zero added. No simulation remains running from these batches. The verification script updated its normal verification report; no design/controller originals were edited.

All runs are reproducible from guarded_study.py and the original D: workspace. Detailed parameters, termination state, power/load metrics and elapsed time are in flight_runs.jsonl. There is no random seed because these runs are deterministic. Resource logs, guard summaries and source hashes are included. The copied runner remains experimental and defaults to preserving original files; it is not integrated into the robot launcher.

## Recommended next engineering gate

Keep full companion features. Treat the 529 g, x3-area wing as a low-power aerodynamic reference and the 559 g x2.5/x3 candidates as mass-allowance references, not selected build designs. First reconcile the force model and measure achievable launch efficiency and wing forces. We have improved the evidence and exposed false-positive short tests; we have not yet made Sparrow jump and fly correctly in a physically validated system.
