"""Live pilot for Sparrow: turns player commands (speed, turn, hop, jump) into the same joint targets the validated
controllers produce. It reuses the workspace controllers without changing them:
  * walking / running  - the quick-step gait law of src/run_gait.py (Runner), re-written here so that speed and turn can
                         change continuously while it runs
  * standing, hopping, jumping - src/hop3.py BiHopper (hip-and-knee spring), scheduled on demand
Everything is real simulated physics through the model's own actuators. Skills (parameter sets) come from game/skills.json.
"""
import json, math, sys
from pathlib import Path
import numpy as np

GAME = Path(__file__).resolve().parent
WS = GAME.parent
sys.path.insert(0, str(WS / 'src'))
from lab import GAINS                # noqa: E402
import legs                          # noqa: E402
from hop3 import BiHopper            # noqa: E402
from hop import DEFAULT as HOP_DEFAULT   # noqa: E402

SKILLS_FILE = GAME / 'skills.json'


def load_skills(path=SKILLS_FILE):
    return json.loads(Path(path).read_text())


class Pilot:
    """Commands (set any time):  speed in [-1, 1] (0 stand, 0.5 walk, 1 run, negative = backwards), turn in [-1, 1]
    (+ = left), request('hop') / request('jump') / any other key of skills['hops']."""

    def __init__(self, skills=None):
        self.sk = skills or load_skills()
        self.speed = 0.0; self.turn = 0.0; self.pending = None
        self.events = []                       # (time, text) for the HUD / log

    # ------------------------------------------------------------------ setup
    def start(self, lab):
        b = lab.e.base[lab.ids]
        self.p0 = legs.fk(b[0], b[1])
        # placeholder entry while standing: neutral settings only (no energy scale or trim carried into real hops)
        self.idle = {k: v for k, v in self.sk['hops'][self.sk['hop_order'][0]].items() if k not in ('E_scale', 'hip_bias')}
        self.hopper = BiHopper([(1e9, self.idle)])
        self.hopper.start(lab)
        self.mode = 'stand'; self.amp = 0.0; self.back = 0.0; self.cyc = 0.0; self.f = self.sk['walk']['f']; self.s = 0.0; self.trn = 0.0; self.pv = 0.0; self.rest = 0.0
        self.yaw_a = [lab.s.act.get(f'act_leg_{S}_yaw') for S in 'LR']; self.yaw = [0.0, 0.0]
        self.last = lab.d.ctrl[lab.ids].copy(); self.t_mode = 0.0; self.hops_done = 0; self.hop_name = None

    def request(self, name):
        if name in self.sk['hops']:
            self.pending = name

    # ------------------------------------------------------------------ gait
    def _gait_params(self):
        w, r = self.sk['walk'], self.sk['run']
        c = abs(self.speed)
        x = 0.0 if c <= 0.5 else (c - 0.5) / 0.5
        g = {k: w[k] + (r[k] - w[k]) * x for k in ('f', 'duty', 's', 'lift', 'back')}
        g['s'] *= min(1.0, c / 0.5) * (1.0 if self.speed >= 0 else -self.sk.get('reverse_scale', 0.6))
        if c <= 0.08: g['s'] = 0.0
        return g

    def _walk(self, lab, t, moving):
        d, m = lab.d, lab.m; dt = 0.01
        g = self._gait_params() if moving else {'f': self.f, 'duty': self.sk['walk']['duty'], 's': self.s, 'lift': self.sk['walk']['lift'], 'back': self.sk['walk']['back']}
        self.f += float(np.clip(g['f'] - self.f, -2.0 * dt, 2.0 * dt))
        self.s += float(np.clip(g['s'] - self.s, -0.06 * dt, 0.06 * dt))
        self.trn += float(np.clip(self.turn * self.sk.get('turn_gain', 0.4) - self.trn, -1.5 * dt, 1.5 * dt))
        pv_t = self.sk.get('pivot_stride', 0.0) * (1.0 if self.turn > 0 else -1.0) if (moving and abs(self.speed) <= 0.08) else 0.0
        self.pv += float(np.clip(pv_t - self.pv, -0.04 * dt, 0.04 * dt))
        self.amp = float(np.clip(self.amp + (dt / 1.0 if moving else -dt / 0.6), 0.0, 1.0))
        if moving or self.amp > 0:
            self.back = min(g['back'], self.back + g['back'] * dt / 0.3)
        else:
            self.back = max(0.0, self.back - self.sk['walk']['back'] * dt / 0.4)
        self.cyc += self.f * dt
        pitch, roll, v = lab.e.state()
        kp, kd = (GAINS[5], GAINS[6]) if pitch < 0 else (GAINS[0], GAINS[1])
        u = float(np.clip(kp * pitch + kd * v[0], -0.45, 0.45))
        tgt = np.zeros(6); duty = g['duty']
        for k in range(2):
            ph = (self.cyc + 0.5 * k) % 1.0; swing = False; dy = dz = 0.0
            if self.amp > 0:
                if ph < duty:
                    dy = self.amp * (ph / duty - 0.5)
                else:
                    q = (ph - duty) / (1 - duty); swing = True
                    dy = self.amp * (0.5 - q * q * (3 - 2 * q)); dz = self.amp * g['lift'] * math.sin(math.pi * q)
            sg = 1.0 if k == 0 else -1.0
            dy *= self.s * (1.0 - sg * self.trn) - sg * self.pv        # per-leg stride: forward part, steering part, pivot part
            q3 = legs.leg_targets(self.p0[0] + dy - self.back, self.p0[1] + dz)
            if swing:
                q3[2] = -(d.qpos[lab.qi[3 * k]] + d.qpos[lab.qi[3 * k + 1]]) - pitch
            else:
                q3[2] -= u
            tgt[3 * k:3 * k + 3] = q3
        tgt = np.clip(tgt, m.actuator_ctrlrange[lab.ids, 0], m.actuator_ctrlrange[lab.ids, 1])
        tgt = np.clip(tgt, self.last - 0.10, self.last + 0.10)
        d.ctrl[lab.ids] = tgt; self.last = tgt.copy()
        # hip twist (only on the hip-yaw model copy): the standing leg sweeps its hip from -A to +A during stance, which
        # rotates the body over the planted foot; the swinging leg sweeps back to -A ready for its next stance.
        if self.yaw_a[0] is not None:
            A = self.sk.get('hip_yaw_amp', 0.25) * self.sk.get('hip_yaw_sign', 1.0) * self.turn * self.amp
            for k in range(2):
                ph = (self.cyc + 0.5 * k) % 1.0
                s_ = (ph / duty - 0.5) * 2 if ph < duty else 1 - 2 * ((lambda q: q * q * (3 - 2 * q))((ph - duty) / (1 - duty)))
                want = A * s_
                self.yaw[k] += float(np.clip(want - self.yaw[k], -0.08, 0.08))
                d.ctrl[self.yaw_a[k]] = self.yaw[k]

    def _yaw_home(self, lab):
        if self.yaw_a[0] is not None:
            for k in range(2):
                self.yaw[k] += float(np.clip(-self.yaw[k], -0.03, 0.03)); lab.d.ctrl[self.yaw_a[k]] = self.yaw[k]

    # ------------------------------------------------------------------ main tick (100 Hz)
    def __call__(self, lab, t):
        if self.speed < 0 and self.sk.get('reverse_scale', 0.0) <= 0: self.speed = 0.0      # backwards is off unless a skill enables it
        if abs(self.turn) > 0.5 and abs(self.speed) <= 0.08 and self.sk.get('pivot_stride', 0.0) <= 0: pass
        pivoting = self.sk.get('pivot_stride', 0.0) > 0 and abs(self.speed) <= 0.08 and abs(self.turn) > 0.5 and self.pending is None
        want_move = (abs(self.speed) > 0.08 or pivoting) and (self.pending is None or (self.mode == 'walk' and self.sk.get('moving_hop', True)))
        if self.mode == 'stand':
            if self.pending is not None and t - self.t_mode > self.sk.get('settle_s', 0.4):
                h = self.hopper
                if h.i >= len(h.hops): h.hops.append((1e9, self.idle))
                h.hops[h.i] = (t, {**HOP_DEFAULT, **self.sk['hops'][self.pending]})
                self.hop_name = self.pending; self.pending = None; self.mode = 'hop'; self.t_mode = t
                self.events.append((t, f'{self.hop_name}!'))
            elif want_move and t - self.t_mode >= self.rest:
                self.mode = 'walk'; self.t_mode = t; self.cyc = 0.0; self.f = self.sk['walk']['f']; self.s = 0.0; self.last = lab.d.ctrl[lab.ids].copy()
        if self.mode == 'walk' and self.pending is not None and self.sk.get('moving_hop', True):
            # hop straight out of the stride, no stopping first: the hopper starts its crouch from the current leg targets
            h = self.hopper
            if h.i >= len(h.hops): h.hops.append((1e9, self.idle))
            h.last = lab.d.ctrl[lab.ids].copy(); h.stand_pose = np.zeros(3); h.phase = 'stand'; h.t_phase = t
            name = self.pending; hp = self.sk['hops'].get(name + '_moving', self.sk['hops'][name])
            h.hops[h.i] = (t, {**HOP_DEFAULT, **hp})
            self.hop_name = name; self.pending = None; self.mode = 'hop'; self.t_mode = t; self.amp = 0.0; self.back = 0.0
            self.events.append((t, f'{name} (moving)!'))
        if self.mode == 'walk':
            self._walk(lab, t, want_move)
            if not want_move and self.amp <= 0 and self.back <= 0:
                self.mode = 'stand'; self.t_mode = t; self.rest = 0.0
                self.hopper.last = lab.d.ctrl[lab.ids].copy(); self.hopper.stand_pose = np.zeros(3)
            return
        self._yaw_home(lab)
        h = self.hopper
        if h.i >= len(h.hops): h.hops.append((1e9, self.idle))
        was = h.i
        h(lab, t)
        if self.mode == 'hop' and h.i > was:
            self.mode = 'stand'; self.t_mode = t; self.hops_done += 1; self.rest = self.sk.get('post_hop_settle_s', 0.8)
        self.last = lab.d.ctrl[lab.ids].copy()

    def substep(self, lab):
        self.hopper.substep(lab)

    @property
    def phase(self):
        return self.hopper.phase if self.mode == 'hop' else self.mode
