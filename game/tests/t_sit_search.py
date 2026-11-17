"""From the sitting-back fall, try leg/head/wing postures and see which bring the bird up onto its feet."""
import itertools, math, sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import numpy as np, mujoco
from world import World

w = World(); lab = w.lab; m = lab.m; s = lab.s
w.reset(None, {'direction': 'backward', 'force': 12.0}); w.pilot.speed = 0
while w.t < 3.0:
    lab.e.step(); w.i += 1
    if w.i % 10 == 0: w.pilot(lab, w.t)
snap = mujoco.MjData(m); mujoco.mj_copyData(snap, m, lab.d)
base = lab.e.base[lab.ids].copy()
tilt = lambda: math.degrees(math.acos(float(np.clip(lab.d.xmat[1].reshape(3, 3)[2, 2], -1, 1))))
print(f'sitting start tilt {tilt():.0f} deg')
res = []
for a, b, hd, fl in itertools.product((-0.8, -0.4, 0.0, 0.4, 0.8), (0.0, 0.5, 1.0), (-0.35, 0.5), (-0.54, 0.0, 0.54)):
    mujoco.mj_copyData(lab.d, m, snap)
    q = base + np.array([a, b, -(a + b), a, b, -(a + b)])
    for k in range(2000):
        if k % 10 == 0:
            lab.d.ctrl[lab.ids] = np.clip(q, m.actuator_ctrlrange[lab.ids, 0], m.actuator_ctrlrange[lab.ids, 1])
            lab.d.ctrl[s.act['act_head_pitch']] = hd
            for S in 'LR': lab.d.ctrl[s.act[f'act_wing_{S}_sweep']] = 0.0; lab.d.ctrl[s.act[f'act_wing_{S}_flap']] = fl
        lab.e.step()
    res.append((tilt(), a, b, hd, fl))
res.sort()
for r in res[:8]: print(f'tilt {r[0]:5.1f} deg  thigh {r[1]:+.1f} knee {r[2]:+.1f} head {r[3]:+.2f} flap {r[4]:+.2f}')
