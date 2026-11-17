"""Use trained brains inside the game (world.py). Observations and action mapping are identical to sparrow_env.py.
Enabled per task in skills.json:  "brain": {"recover": "latest" | "<run folder>" | null, "getup": ...}"""
import math
from pathlib import Path
import numpy as np
from sparrow_env import brain_obs, brain_joints
from policy import Brain, latest_run

HERE = Path(__file__).resolve().parent


def load(cfg, task):
    v = (cfg or {}).get(task)
    if not v: return None
    run = latest_run(task) if v == 'latest' else (Path(v) if Path(v).is_absolute() else HERE / v)
    return Brain(run) if run and (run / 'model.zip').exists() else None


class WalkCorrector:
    """Adds the recover brain's leg corrections on top of the walking controller (same 0.15 rad scale as training)."""
    def __init__(self, brain, lab):
        self.b = brain; self.acts = brain_joints(lab); self.last = np.zeros(6, np.float32)

    def apply(self, lab, pilot):
        ph = [math.sin(2 * math.pi * pilot.cyc), math.cos(2 * math.pi * pilot.cyc)] if pilot.mode == 'walk' else [0.0, 0.0]
        o = brain_obs(lab, self.acts, self.last, [pilot.speed, pilot.turn] + ph, getattr(lab, 'ground_z', 0.0))
        a = np.clip(self.b.act(o), -1, 1); self.last = a
        d = lab.d; d.ctrl[lab.ids] = np.clip(d.ctrl[lab.ids] + 0.15 * a[:6], lab.m.actuator_ctrlrange[lab.ids, 0], lab.m.actuator_ctrlrange[lab.ids, 1])


class BrainGetUp:
    """Same interface as getup.GetUp, driven by the getup brain."""
    def __init__(self, brain, lab, give_up_s=8.0):
        self.b = brain; self.acts = brain_joints(lab); self.p = {'give_up_s': give_up_s}
        self.lo = lab.m.actuator_ctrlrange[self.acts, 0]; self.hi = lab.m.actuator_ctrlrange[self.acts, 1]
        self.last = np.zeros(len(self.acts), np.float32); self.held = 0

    def start(self, lab, t):
        self.t0 = t; self.target = lab.d.ctrl[self.acts].copy(); self.held = 0

    def __call__(self, lab, t):
        o = brain_obs(lab, self.acts, self.last, [0.0, 0.0])
        a = np.clip(self.b.act(o), -1, 1); self.last = a
        want = self.lo + (a + 1) * 0.5 * (self.hi - self.lo)
        self.target += np.clip(want - self.target, -0.15, 0.15); lab.d.ctrl[self.acts] = self.target
        R = lab.d.xmat[1].reshape(3, 3); tilt = math.degrees(math.acos(float(np.clip(R[2, 2], -1, 1))))
        cs = lab.contact_state(); fl = [cs.get(lab.foot_geom[S], {'fn': 0.0})['fn'] / lab.weight for S in 'LR']
        self.held = self.held + 1 if (tilt < 15 and min(fl) > 0.2) else 0
        return 'stood' if self.held >= 100 else 'brain'
