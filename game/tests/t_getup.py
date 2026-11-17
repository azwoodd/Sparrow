"""Knock the bird over in each direction, then run the get-up controller. Reports whether it ends on its feet."""
import math, sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import numpy as np
from world import World
from getup import GetUp, lying

w = World(); lab = w.lab
extra = {a.split('=')[0]: float(a.split('=')[1]) for a in sys.argv[1:]}
for direction in ('forward', 'backward', 'left', 'right'):
    w.reset(None, {'direction': direction, 'force': 12.0}); w.pilot.speed = 0
    while w.t < 3.0:
        lab.e.step(); w.i += 1
        if w.i % 10 == 0: w.pilot(lab, w.t)
    kind, lower = lying(lab)
    g = GetUp(lab, extra); g.start(lab, w.t); stage = ''; t_up = None
    while w.t < 13.0 and stage != 'stood':
        if w.i % 10 == 0: stage = g(lab, w.t)
        lab.e.step(); w.i += 1
    # then stand for 2 s with the normal controller
    if stage == 'stood':
        w.pilot.start(lab); t_up = w.t
        while w.t < t_up + 2.0:
            if w.i % 10 == 0: w.pilot(lab, w.t)
            w.pilot.substep(lab); lab.e.step(); w.i += 1
    R = lab.d.xmat[1].reshape(3, 3); tilt = math.degrees(math.acos(float(np.clip(R[2, 2], -1, 1))))
    ok = stage == 'stood' and tilt < 20
    print(f'fell {direction:8} -> lying {kind}{" on its " + lower if lower else ""}: {"STOOD UP" if ok else "still down"} '
          f'(stage {stage}, {(t_up - 3.0) if t_up else w.t - 3.0:.1f} s, final tilt {tilt:.0f} deg)', flush=True)
