"""Trace a commanded flight turn: commanded vs measured turn rate, bank, roll, heading."""
import math, sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import numpy as np, mujoco
from world import World

tn = float(sys.argv[1]) if len(sys.argv) > 1 else -1.0
w = World(); w.reset(None, None, 'start'); w.launch(8.0, 7.0); lab = w.lab; d = lab.d
while w.t < 9 and w.fallen is None:
    w.pilot.speed = 0.3; w.pilot.turn = tn if w.t > 4 else 0.0; w.tick()
    if int(round(w.t * 100)) % 10 == 0 and w.t > 3.9:
        wv = np.zeros(6); mujoco.mj_objectVelocity(lab.m, d, mujoco.mjtObj.mjOBJ_BODY, 1, wv, 0)
        p, r, v = lab.e.state()
        vx, vy = float(d.qvel[0]), float(d.qvel[1]); track = math.degrees(math.atan2(vx, -vy))       # + = travelling to its left of the start direction
        print(f't={w.t:4.1f} turn cmd {w.pilot.turn:+.0f}  world yaw rate {math.degrees(wv[2]):+6.1f} deg/s  heading {w.heading_deg:+7.1f}  track {track:+7.1f}  roll {math.degrees(r):+6.1f}  course error {math.degrees(getattr(w.fpilot, "course_err", 0)):+6.1f} height {float(d.qpos[2]):.2f} sweepL {d.ctrl[w.rig.sweep_a[0]]:.2f} sweepR {d.ctrl[w.rig.sweep_a[1]]:.2f} mode {w.mode}', flush=True)
