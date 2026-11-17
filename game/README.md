# Sparrow game and Studio (updated 17 Nov 2026)

Double-click **`Sparrow Studio.bat`** in the workspace folder. `Play Sparrow.bat` goes straight to the game;
`Play Sparrow (hip-twist prototype).bat` plays the version that can really turn (see below).

## Controls

Click the 3D window first.

| Keyboard | Controller (Xbox-style) | On the ground | In the air |
|---|---|---|---|
| UP (hold) | left stick up | walk, about 160 mm/s | flap harder, climb |
| SHIFT + UP | stick fully up | fast walk, about 200 mm/s | |
| (nothing) | | stand | cruise, holds its course |
| DOWN (hold) | stick down | | land: descend with feet forward |
| LEFT / RIGHT | stick left / right | steer (weak, see below) | bank and turn, about 40 deg/s |
| SPACE | A | hop (works while walking too) | |
| SHIFT + SPACE | X | far hop | |
| ENTER | B | jump (works while walking too) | |
| SHIFT + ENTER | Y | high jump | |
| HOME | LB | go to the top of the 12 m launch tower | |
| END | RB | assisted launch: put the bird in the air at flying speed | |
| R | START | reset | |
| C | | camera: chase / side | |

To fly: press HOME, then walk forward off the front of the tower. The wings open by themselves when the bird
leaves the edge, it dives to pick up speed, then flies. If it falls, it now tries to get up by itself for about
8 seconds before the game resets it.

## The world

Stairs (three 20 mm steps and a plateau) straight ahead, two hurdles (15 and 30 mm) to the right, a 0.30 m ledge,
three balls to push, a 12 m launch tower behind the start, posts as landmarks, four gates in the air to fly through,
and distance lines every metre. Built by `make_world.py` into the model copy `model/v7e_game`.

## The hip-twist prototype (a proposed hardware change)

Every leg joint on the current design swings forward and back only, so the bird cannot rotate its body with its
legs: five seconds of full steering bends its path a few degrees. The prototype adds one small servo per hip that
twists the leg about the vertical (`make_hipyaw.py` -> `model/v7e_game_hipyaw`, +19 g, about $80-110 for two
KST X08H-class servos). The standing leg twists during each step and turns the body over the planted foot:

- about 60 deg/s at 0.12 rad of twist (300-340 deg in 5 s), walking, slow or on the spot; 18 of 18 runs upright
- the game uses 0.10 rad (about 50 deg/s); walking, hops and jumps still work on the heavier model

It is in a model copy only. It is not in the CAD and is not approved.

## Training (Studio, Train tab)

| Task | What is scored |
|---|---|
| walk, fast_walk | speed without falling |
| steer, pivot, reverse | turning / backwards on the current legs (weak by design) |
| jump_height, hop_distance | jump to a target clearance / hop to a target distance |
| moving_jump | jump while walking, then keep walking |
| step_up | jump up the first step while walking and stay on top |
| getup | get back up after falling backwards, left or right |

Every candidate is checked on 5 robot conditions and the full routine before you decide to promote it.

## What is true today (all simulation)

- **Flight**: free-body flight with nothing holding the attitude. Holds a course on its own (about 2 deg/s of drift;
  it used to pull to the right), turns about 40 deg/s each way (circle of about 13 m), on bird weight +-7% and
  balance +-3 mm. It flies only with assumed changes that are NOT the current design: wing acting as 3x its chord,
  tail as 8x its area, wing mounted 15 mm further back, a twisting wing and a 6 Nm flap drive. Roll comes from the
  existing wing-sweep servos, which turned out to be by far the strongest roll control.
- **Take-off**: from the 12 m tower. A take-off from the ground is not possible with this bird (see the flight studies).
- **Landing**: does not work yet. The bird comes down at 5-7 m/s and tips over; a flare makes it stall and tumble
  because the tail cannot pitch it up hard enough. After a crash it tries to get up.
- **Get-up**: tries with wings, crouch and a slow rise. From sideways and backward falls it gets from lying flat to
  about 34 deg, then stalls. From a forward fall it ends upside down and cannot right itself. Being trained.
- **Steps**: walking cannot climb the 20 mm step: the foot is 128 mm long and the stride about 30 mm, so the toe
  always meets the riser. Getting up needs a forward jump of at least 130 mm; hopping while walking now works but
  mostly falls with the current settings. Being trained.
- **Turning on the ground**: weak on the current legs; strong with the hip-twist prototype.
