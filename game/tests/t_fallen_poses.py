"""Knock the standing bird over in each direction and report how it comes to rest: which way up, what touches the floor."""
import math, sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import numpy as np, mujoco
from world import World

w = World(); lab = w.lab; m = lab.m
names = lambda g: mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g) or f'geom{g}'
for direction in ('forward', 'backward', 'left', 'right'):
    w.reset(None, {'direction': direction, 'force': 12.0})
    w.pilot.speed = 0
    while w.t < 4.0:
        lab.e.step(); w.i += 1
        if w.i % 10 == 0: w.pilot(lab, w.t)
    R = lab.d.xmat[1].reshape(3, 3)
    up = R[:, 2]; fwd = -R[:, 1]
    touching = sorted({names(c.geom1 if c.geom2 == lab.floor else c.geom2) for c in lab.d.contact[:lab.d.ncon] if lab.floor in (c.geom1, c.geom2)})
    print(f'pushed {direction:8}: body up-axis points ({up[0]:+.2f},{up[1]:+.2f},{up[2]:+.2f}), nose points ({fwd[0]:+.2f},{fwd[1]:+.2f},{fwd[2]:+.2f}), '
          f'torso height {lab.d.qpos[2] * 1000:.0f} mm\n    touching the floor: {", ".join(touching)}', flush=True)
