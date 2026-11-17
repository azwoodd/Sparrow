# Sparrow V7E — flight feasibility audit, first pass (13 Nov 2026)

Measurement and bounded arithmetic only (`f0_audit.py`, numbers in `results/f0_audit.json`). **Nothing here is a flight
simulation and nothing here shows that Sparrow flies.** Walking and hopping work stays the priority; this audit ran as a
light second job. Tags: **[M]** measured from the copied model / CAD export · **[S]** component spec · **[C]** calculation ·
**[A]** assumption · **[U]** unknown.

## 1. What the model says

| Quantity | Value | Tag |
|---|---:|---|
| Mass / weight | 0.520 kg / 5.10 N | M (masses are CAD-volume estimates and target weights; fasteners, glue, wiring not counted) |
| Wing span | 1.42 m | M |
| Wing area | 0.166 m² (tail 0.012 m²) | M |
| Aspect ratio / mean chord | 12.1 / 117 mm (106 mm inboard, up to 162 mm on the hand) | M |
| Wing loading | 30.7 N/m² (31 g/dm²) | C |
| One wing: mass, inertia about flap hinge | 55.8 g, 0.0049 kg·m² | M |
| Flap stroke | ±31° | M |
| Flap drive in the model | ideal position servo, 1.5 Nm, no speed limit — a placeholder, not the crank gearbox | M |
| Flap motor, gearbox ratio, battery cell, ESC | not selected | **U** |
| Battery mass budget | 45 g | M (target) |

There are no manufacturer specifications anywhere in the project yet, so the **[S]** column is empty.

## 2. Wing-borne flight (fixed-wing relations)

| Assumed max lift coefficient [A] | Stall speed [C] | Cruise at 1.2× stall |
|---:|---:|---:|
| 0.8 | 7.9 m/s | 9.5 m/s |
| 1.0 | 7.1 m/s | 8.5 m/s |
| 1.2 | 6.5 m/s | 7.8 m/s |
| 1.4 | 6.0 m/s | 7.2 m/s |

This agrees with the owner's earlier held-in-airflow test (lift equals weight at 7 m/s, not at 6 m/s).
Minimum-power level flight, drag polar with Oswald factor 0.8 [A] and zero-lift drag 0.03 / 0.05 / 0.08 [A]:
drag 0.33–0.53 N, aerodynamic power **2.1–3.3 W** at about 6.3 m/s. These lift-to-drag ratios (10–15) are optimistic for a
slotted, feathered wing at Reynolds number ~55,000; halving them doubles the power.
Electrical power for cruise, across propulsive efficiency 0.25–0.6 and drive efficiency 0.5–0.7 [A]: **5–27 W**.
A 45 g battery at 120–200 Wh/kg [A] holds 5.4–9.0 Wh: roughly **10–17 minutes at 25 W**, 5–9 minutes at 50 W.

**Reading:** on weight, wing area and power, sustained cruise is not ruled out. Sparrow is in the same class as existing
half-kilogram ornithopters.

## 3. Flapping

Tip travel is 0.73 m peak to peak. For a Strouhal number of 0.2–0.4 (where efficient flapping flight sits) [A]:

| Airspeed | Wingbeat frequency | Peak torque just to reverse one wing |
|---:|---:|---:|
| 5 m/s | 1.4–2.7 Hz | 0.19–0.78 Nm |
| 7 m/s | 1.9–3.8 Hz | 0.38–1.53 Nm |
| 9 m/s | 2.5–4.9 Hz | 0.63–2.53 Nm |

Aerodynamic torque comes on top of that. The model's 1.5 Nm placeholder is already at the edge at 7 m/s.

## 4. What blocks flight today

1. **No thrust mechanism.** The handoff states wing twist for thrust is not designed, and the simulation's air model is a
   normal force on rigid panels that cannot produce thrust. Rigid panels flapped ±31° without twist make little or none.
   This must be designed and modelled before any flight claim.
2. **Launch gap.** Minimum flying speed is about 6.5 m/s (10.9 J of kinetic energy, or a 2.1 m drop). The fastest tested
   gait is 0.25 m/s and a hop leaves the ground at 0.8 m/s. Legs and springs cannot supply the speed.
3. **Ground take-off needs near-hover power.** Actuator-disc bound over the area the wings actually sweep (0.50 m²): 10 W
   ideal, **20–40 W aerodynamic** at a figure of merit of 0.25–0.5 [A], so roughly 40–130 W electrical for a few seconds.
4. **Drive train is undefined.** Motor, gearbox, battery and their real masses are unknown; every power figure above is a
   scenario, not a capability.

## 5. Answers so far

| Question | Answer | Confidence |
|---|---|---|
| Can the wings carry the weight? | Yes above ~6.5 m/s | calculation, matches earlier airflow test |
| Can the current design produce thrust? | Not shown; no mechanism designed | — |
| Minimum practical flying speed | 6–8 m/s | bounded by lift coefficient |
| Can a hop or run reach it? | No: 0.25–0.8 m/s available | measured |
| Is a running start worth much? | Not at today's speeds (0.25 of 6.5 m/s) | calculation |
| Flight duration | 5–17 min if cruise needs 25–50 W | scenario |

## 6. Next steps, when walking and hopping are done

1. A flapping-wing force model with passive or driven twist, checked against published ornithopter data, in a copied model.
2. Bounded motor/gearbox/battery candidates with real masses, then re-run this audit.
3. Airborne tests from set initial conditions (speed, height) before any take-off attempt.
4. Only then: jump-assisted launch, flight control, landing.

No CAD change is proposed yet.
