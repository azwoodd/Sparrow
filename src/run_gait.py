"""Fast dynamic alternating gait ("run") for the pitch-only V7E legs - existing actuators only, wings optional.

Idea: the COM is only ~9 mm outside each pad's inner edge, so when one foot is lifted the body tips sideways SLOWLY
(a few degrees in 0.2 s). If each swing is short, the swing foot lands before the tip matters and the next step tips it
back. No lateral actuator is needed - this is a timed dynamic gait rather than a balanced one.

Parameters (cyclic, per leg, half a period apart):
  f      step frequency per leg (Hz)         duty   fraction of the cycle a leg is commanded down (stance)
  s      stride: stance foot travels from -s/2 (front) to +s/2 (back) relative to the body
  lift   swing height                         back   static backward lean (ankles forward) to unload the ankle
  wing   0..1 symmetric wing deployment (roll inertia)     ramp   seconds to ramp stride/lift in from standing
  fp     swing foot toe-up pitch              kroll  lateral gain: roll feedback into leg-length difference
"""
import math
import numpy as np
from lab import GAINS
import legs
from script_ctl import STOW

RUN_DEFAULT = dict(f=2.5, duty=0.6, s=0.03, lift=0.012, back=0.02, wing=0.0, ramp=1.0, fp=0.0, kroll=0.0, slew=10.0, dz0=0.0,
                   stop_t=None, stop_ramp=0.6)


class Runner:
    def __init__(self, **kw):
        self.p = {**RUN_DEFAULT, **kw}

    def start(self, lab):
        b = lab.e.base[lab.ids]
        self.p0 = legs.fk(b[0], b[1]); self.last = lab.d.ctrl[lab.ids].copy()
        self.w0 = STOW; self.wing_cur = STOW; self.phase0 = None

    def amp(self, t):
        p = self.p
        a = min(1.0, max(0.0, (t - 0.3) / p['ramp']))
        if p['stop_t'] is not None and t > p['stop_t']:
            a *= max(0.0, 1.0 - (t - p['stop_t']) / p['stop_ramp'])
        return a

    def leg_pose(self, ph, a):
        """ph in [0,1): stance for ph < duty (foot moves back under body), swing otherwise (lift, return forward)."""
        p = self.p; duty = p['duty']
        if ph < duty:
            u = ph / duty
            return a * p['s'] * (u - 0.5), 0.0, False
        u = (ph - duty) / (1 - duty)
        dy = a * p['s'] * (0.5 - (u * u * (3 - 2 * u)))
        dz = a * p['lift'] * math.sin(math.pi * u)
        return dy, dz, True

    def __call__(self, lab, t):
        d, m = lab.d, lab.m; p = self.p
        a = self.amp(t)
        back = p['back'] * min(1.0, t / 0.3)
        if p['stop_t'] is not None and t > p['stop_t'] + p['stop_ramp']:
            back *= max(0.0, 1.0 - (t - p['stop_t'] - p['stop_ramp']) / 0.4)
        pitch, roll, v = lab.e.state()
        kp, kd = (GAINS[5], GAINS[6]) if pitch < 0 else (GAINS[0], GAINS[1])
        u = float(np.clip(kp * pitch + kd * v[0], -0.45, 0.45))
        tgt = np.zeros(6)
        cyc = max(0.0, t - 0.3) * p['f']
        dzr = float(np.clip(p['kroll'] * roll, -0.008, 0.008))
        for k in range(2):
            ph = (cyc + 0.5 * k) % 1.0
            dy, dz, swing = self.leg_pose(ph, a) if a > 0 else (0.0, 0.0, False)
            sg = 1.0 if k == 0 else -1.0
            dy *= 1.0 - sg * p.get('turn', 0.0)          # turn > 0: shorter left stride, longer right -> curves LEFT
            pos = self.p0 + np.array([dy - back, dz + p['dz0'] * min(1.0, t / 0.3) + sg * dzr])
            q = legs.leg_targets(pos[0], pos[1])
            if swing:
                q1, q2 = d.qpos[lab.qi[3 * k]], d.qpos[lab.qi[3 * k + 1]]
                q[2] = -(q1 + q2) - pitch + p['fp']
            else:
                q[2] -= u
            tgt[3 * k:3 * k + 3] = q
        tgt = np.clip(tgt, m.actuator_ctrlrange[lab.ids, 0], m.actuator_ctrlrange[lab.ids, 1])
        sl = p['slew'] * 0.01
        tgt = np.clip(tgt, self.last - sl, self.last + sl)
        d.ctrl[lab.ids] = tgt; self.last = tgt.copy()
        if p['wing'] > 0:
            want = STOW * (1 - p['wing'] * min(1.0, t / 0.3))
            if p['stop_t'] is not None and t > p['stop_t']:
                want = STOW * (1 - p['wing'] * max(0.0, 1.0 - (t - p['stop_t']) / (p['stop_ramp'] + 0.4)))
            self.wing_cur = float(np.clip(want, self.wing_cur - 0.06, self.wing_cur + 0.06))
            for S in 'LR':
                d.ctrl[lab.act[f'act_wing_{S}_sweep']] = self.wing_cur
