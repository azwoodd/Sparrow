# Sparrow V7E - take-off study, first pass (14-15 Nov 2026)

`f1_wing_model.py`, logs and numbers in `flight_research/results/f1_*`. Calculation only. **This is not a flight
simulation of Sparrow and does not show that Sparrow flies.** It is a bounded model built to find out what a take-off
would demand, so the mechanical design can be aimed at something achievable.

## 1. The wing force model

- Blade elements on the real wing strips of the model (12 per side, tip 0.63 m from the flap hinge, 0.166 m2, AR 12).
- Flapping +-31 deg (the model's range) at a chosen frequency.
- **Twist is prescribed**: leading edge down on the downstroke, up on the upstroke, growing linearly to the tip.
  No such mechanism exists in the CAD; the model asks how much of it would be needed.
- Section forces: attached-flow lift with 70% leading-edge suction below ~10 deg, blending to a stalled flat plate above
  ~18 deg; skin friction; finite-wing induced drag; body drag area 5% of wing area. All assumed.
- Body is a point mass (0.529 kg, candidate-parts mass) with a prescribed pitch angle. Pitch stability, the tail,
  wing flexibility and unsteady effects are not modelled.

Checks on the model itself:
- Zero twist gives exactly zero thrust at rest (agrees with the F0 audit: rigid panels cannot propel).
- My first version left out leading-edge suction and could not hold level flight on less than 40-85 W. That was a model
  error (drag about six times too high in normal flight), not a finding, and its take-off numbers were discarded.
- Corrected model, level cruise: **11-13 W aerodynamic at 9-10 m/s, 3.5-4 Hz, 30-40 deg tip twist**, peak motor torque
  2.7-2.9 Nm per wing. That is in the range of real half-kilogram ornithopters (Festo SmartBird: 450 g, 12 W in flight).
- The model cannot hold level flight below about 9 m/s. Its maximum lift coefficient is about 1.0, so this is more
  pessimistic than the 6.5-7 m/s stall speed in the F0 audit. The true figure is likely between the two.

## 2. Held still: how much flapping makes thrust equal to weight

| Flap amplitude | Tip twist | Frequency | Aerodynamic power | Peak motor torque per wing |
|---|---|---|---|---|
| +-31 deg (as designed) | 60-75 deg | 10 Hz | 64-92 W | 10.5 Nm |
| +-45 deg | 45-75 deg | 8 Hz | 100-190 W | 10 Nm |
| +-60 deg | 45-75 deg | 6 Hz | 100-190 W | 8 Nm |
| +-70 deg | 45-75 deg | 5 Hz | 92-174 W | 6.5-7 Nm |

(Frequency stepped coarsely, so the higher-amplitude rows overshoot weight by 20-40%.) The torque is mostly the effort
of reversing the wing itself: each wing is 56 g with 0.0049 kg m2 about its hinge.

## 3. Take-off trajectories (best twist and body-angle schedule found for each case)

| Start | Result |
|---|---|
| 100 mm hop, wings already open and flapping at lift-off | Back on the ground in 0.3-0.7 s at 3-8 Hz. Stays up only at 10 Hz: 110 W, 10.6 Nm per wing, i.e. hovering. |
| Dropped from a 1, 2 or 3 m perch (dives allowed) | Reaches the ground at 3-8 Hz. Stays up only by hovering at 10 Hz. |
| Thrown level at 6 m/s from 1.5 m | Flies only at 10 Hz (96 W). |
| Thrown level at 8 m/s from 1.5 m | Flies from 5 Hz: 16 W, 3.9 Nm per wing. |
| Thrown level at 10 m/s from 1.5 m | Flies from 4 Hz: 13 W, 3.0 Nm per wing. |

## 4. What this says about "jump, open wings, flap, fly"

1. **The jump cannot be the launch.** It supplies 1.5 m/s and 0.3 s. Flight in this model needs about 9 m/s. The gap can
   only be closed by hovering, which costs roughly ten times cruise power and three to four times cruise torque.
2. **Hover-class take-off is out of reach for this wing**: 65-190 W aerodynamic and 6.5-10.5 Nm per wing, whatever the
   flap amplitude. A wing with 1.42 m span and 56 g per side is a cruising wing.
3. **Cruise looks achievable** once at speed: 11-13 W aerodynamic, about 20-45 W electrical with drive losses, inside
   what an 1806-size motor and the 43 g battery can supply.
4. **So the realistic sequence needs speed from somewhere else**: a throw or launcher at 8-10 m/s, or a drop of more than
   about 4-5 m (9 m/s is 4.1 m of fall before losses; 3 m was not enough here). A hop helps only by clearing the feet.
5. For a ground take-off to become possible the design itself would have to change: far lighter wings, lower wing
   loading, or a different high-thrust mode. That is a redesign decision, not a controller problem.

## 5. What the mechanical design now needs

| Item | Requirement from this study |
|---|---|
| Wing twist | 30-40 deg at the tip, leading edge down on the downstroke, for cruise. Without it there is no thrust. |
| Flap drive | 3.5-5 Hz, about 3-4 Nm peak per wing, 11-16 W mean aerodynamic. Mostly inertial torque, so a spring at the hinge or lighter outer wing would cut it. |
| Gearbox | An 1806 motor turning at roughly 200 rev/s under load needs about 50:1 to flap at 4 Hz. Ratio still undefined in CAD. |
| Launch | 8-10 m/s. Not available from legs or springs. |

## 6. Limits

Assumed coefficients throughout; prescribed twist and body angle; no pitch dynamics, so "does not fall over in the air"
is not tested at all; coarse search grids; no measured wing data. Next step for confidence would be to put the twist
into a copy of the MuJoCo model and check the thrown-launch case with full body dynamics.

## 7. How far the design would have to move for a self take-off (`f2_redesign_sizing.py`)

Self take-off from a hop means thrust about equal to weight for a second or two. For each variant: the cheapest
flapping that does it. Limits used for the verdict (assumed): 50 W mean aerodynamic, 4 Nm peak per wing.
"Hinge spring" = a spring at the flap hinge that cancels the wing's own inertia, leaving the aerodynamic torque.

| Variant | Tip twist limited to 45 deg | Tip twist up to 85 deg |
|---|---|---|
| As designed: 529 g, +-31 deg stroke, 56 g wings | 10.5 Hz, 140 W, 11.6 Nm | 10 Hz, 48 W, 10.5 Nm (1.3 Nm with hinge spring) |
| Stroke +-60 deg | 5.5 Hz, 146 W, 7.0 Nm | 5.25 Hz, 51 W, 5.9 Nm |
| Stroke +-75 deg | 4.25 Hz, 132 W, 5.8 Nm | 4.25 Hz, 53 W, 5.0 Nm |
| Wings with half the inertia | 10.5 Hz, 140 W, 6.7 Nm | 10 Hz, 48 W, 5.5 Nm |
| Robot 400 g | 9 Hz, 88 W, 8.5 Nm | 8.75 Hz, 32 W, 8.0 Nm |
| Robot 300 g | 7.75 Hz, 56 W, 6.3 Nm | 7.5 Hz, 20 W, 5.9 Nm |
| Chord +50% | 8.5 Hz, 112 W, 8.0 Nm | 8.25 Hz, 41 W, 7.3 Nm |
| 400 g, +-60 deg, half-inertia wings | 4.75 Hz, 94 W, 3.9 Nm | **4.5 Hz, 32 W, 2.6 Nm - within limits** |
| 300 g, +-60 deg, half-inertia wings | 4.25 Hz, 67 W, 3.1 Nm | **4 Hz, 22 W, 2.0 Nm - within limits** |
| 300 g, +-75 deg, half-inertia wings, chord +50% | 2.75 Hz, 54 W, 2.7 Nm | **2.5 Hz, 18 W, 1.4 Nm - within limits** |
| 250 g, +-75 deg, third-inertia wings, chord +50% | **2.5 Hz, 40 W, 2.2 Nm - within limits** | **2.5 Hz, 16 W, 1.1 Nm - within limits** |

Reading:
1. **Twist is the biggest lever.** Letting the outer wing feather far more than 45 deg cuts hover power roughly threefold.
   Whether a real wing can twist that far and still behave as the model assumes is unverified; the model is at its
   least trustworthy there (attached-flow assumptions at extreme angles).
2. **No single change is enough.** Stroke, wing inertia, mass and area each help one of power or torque, not both.
3. **A combination works on paper**: about 400 g or less, about +-60 deg stroke, wings with half the inertia, and large
   twist. With twist limited to 45 deg the robot would have to be about 250 g with a bigger, much lighter wing.
4. Wider stroke lowers frequency and torque but not power; lower mass lowers power; lighter wings or a hinge spring
   lower torque.
