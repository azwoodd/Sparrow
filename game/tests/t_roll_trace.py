"""Trace roll command vs actual roll in flight, and the roll moment each effector can produce."""
import math, sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import numpy as np
from world import World

w = World(); w.reset(None, None, 'start'); w.launch(8.0, 7.0)
fp = w.fpilot; lab = w.lab; d = lab.d
while w.t < 9:
    w.pilot.speed = 0.3; w.pilot.turn = 1.0 if w.t > 4 else 0.0; w.tick()
    if int(round(w.t * 100)) % 50 == 0:
        p, r, v = lab.e.state()
        print(f't={w.t:4.1f} roll {math.degrees(r):+6.1f} rollrate {math.degrees(v[1]):+6.1f} pitch(up) {-math.degrees(p):+5.1f} heading {w.heading_deg:+6.1f} '
              f'tail_roll cmd {math.degrees(d.ctrl[w.rig.tail_r]):+5.1f} flapL ctrl {math.degrees(d.ctrl[w.rig.flap_a[0]]):+5.1f} flapR ctrl {math.degrees(d.ctrl[w.rig.flap_a[1]]):+5.1f}', flush=True)
