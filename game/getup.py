"""Get-up controller: when the bird is lying down, use the wings (and legs, neck, tail) to roll or push itself upright,
then hand back to the normal standing controller. Uses only the model's own actuators at their own limits.

Strategies (chosen from which way the body is lying):
  side      open both wings, press the LOWER wing tip down into the floor and lift the upper one, legs tucked;
            the lower wing levers the body up onto its feet
  back      (lying upside down) open both wings and beat them alternately to roll onto a side, then 'side'
  sitting   (on the tail, feet down) lean the head and neck forward, push the tail down, crouch and stand
Parameters live in skills.json -> "getup" so they can be trained."""
import math
import numpy as np

GETUP_DEFAULT = dict(push_flap=-0.54, lift_flap=0.54, open_time=0.5, push_time=1.2, roll_flap_hz=1.5, done_tilt_deg=30.0, give_up_s=8.0,
                     sit_neck=-0.6, sit_head=-0.35, sit_tail=0.0, sit_flap=0.0, crouch_thigh=0.8, crouch_knee=1.0, rise_time=1.5, rise_kp=1.0, rise_kd=0.1)


def lying(lab):
    """Return ('upright'|'side'|'back'|'sitting', lower_side 'L'|'R'|None) from the torso orientation."""
    R = lab.d.xmat[1].reshape(3, 3); up = R[:, 2]
    if up[2] > 0.82: return 'upright', None
    if up[2] < -0.5: return 'back', None
    left_down = R[2, 0] > 0                     # measured: this sign means lying on its LEFT side (first guess was inverted)
    if abs(R[2, 0]) > abs(R[2, 1]): return 'side', ('L' if left_down else 'R')
    return 'sitting', None


class GetUp:
    def __init__(self, lab, params=None):
        self.p = {**GETUP_DEFAULT, **(params or {})}; s = lab.s
        self.flap = {S: s.act[f'act_wing_{S}_flap'] for S in 'LR'}; self.sweep = {S: s.act[f'act_wing_{S}_sweep'] for S in 'LR'}
        self.neck, self.head, self.tail = s.act['act_neck_yaw'], s.act['act_head_pitch'], s.act['act_tail_pitch']
        self.stow = s.info['poses']['stand_wings_stowed']['q']
        self.t0 = None; self.kind = None; self.lower = None; self.stage = 'start'; self.t_stage = 0.0

    def start(self, lab, t):
        self.t0 = t; self.kind, self.lower = lying(lab); self.stage = 'open'; self.t_stage = t

    def done(self, lab):
        R = lab.d.xmat[1].reshape(3, 3)
        return math.degrees(math.acos(float(np.clip(R[2, 2], -1, 1)))) < self.p['done_tilt_deg']

    def __call__(self, lab, t):
        d = lab.d; p = self.p; dt = t - self.t_stage
        kind, lower = lying(lab)
        # legs tucked: knees bent, feet under the body (helps both rolling and standing)
        base = lab.e.base[lab.ids]
        c = np.array([p['crouch_thigh'], p['crouch_knee'], -(p['crouch_thigh'] + p['crouch_knee'])])     # deep crouch (found by search from the sitting fall)
        tuck = base + np.concatenate([c, c])
        d.ctrl[lab.ids] = np.clip(tuck, lab.m.actuator_ctrlrange[lab.ids, 0], lab.m.actuator_ctrlrange[lab.ids, 1])
        if kind == 'side' and lower is not None: self.kind, self.lower = kind, lower
        if self.stage == 'open':
            for S in 'LR': d.ctrl[self.sweep[S]] = 0.0; d.ctrl[self.flap[S]] = 0.0
            if dt > p['open_time']: self.stage = 'act'; self.t_stage = t
        elif self.stage == 'act':
            if kind == 'back':
                ph = math.sin(2 * math.pi * p['roll_flap_hz'] * dt)
                d.ctrl[self.flap['L']] = p['lift_flap'] * ph; d.ctrl[self.flap['R']] = -p['lift_flap'] * ph
            elif kind == 'side':
                lo, hi = (self.lower, 'R' if self.lower == 'L' else 'L')
                d.ctrl[self.flap[lo]] = p['push_flap']; d.ctrl[self.flap[hi]] = p['lift_flap']
            else:                                   # sitting back on the tail
                d.ctrl[self.head] = p['sit_head']; d.ctrl[self.tail] = p['sit_tail']
                for S in 'LR': d.ctrl[self.flap[S]] = p['sit_flap']
            if self.done(lab): self.stage = 'rise'; self.t_stage = t
            elif dt > p['push_time'] * 3: self.stage = 'open'; self.t_stage = t      # try again from wings open
        elif self.stage == 'rise':                      # wings folding, stand slowly out of the crouch, ankles correct pitch
            for S in 'LR':
                d.ctrl[self.flap[S]] = 0.0; d.ctrl[self.sweep[S]] = self.stow.get(f'wing_{S}_sweep', 1.5)
            d.ctrl[self.head] = 0.0; d.ctrl[self.tail] = 0.0
            a = min(1.0, dt / p['rise_time']); a = a * a * (3 - 2 * a)
            pitch, roll, v = lab.e.state()
            u = float(np.clip(p['rise_kp'] * pitch + p['rise_kd'] * v[0], -0.45, 0.45))
            q = tuck * (1 - a) + base * a; q[2] -= u; q[5] -= u
            d.ctrl[lab.ids] = np.clip(q, lab.m.actuator_ctrlrange[lab.ids, 0], lab.m.actuator_ctrlrange[lab.ids, 1])
            if dt > p['rise_time'] + 0.3: self.stage = 'stood'
            elif not self.done(lab) and dt > 0.3: self.stage = 'act'; self.t_stage = t     # tipped back down: try again
        elif self.stage == 'fold':
            for S in 'LR':
                d.ctrl[self.flap[S]] = 0.0; d.ctrl[self.sweep[S]] = self.stow.get(f'wing_{S}_sweep', 1.5)
            d.ctrl[self.head] = 0.0; d.ctrl[self.tail] = 0.0
            d.ctrl[lab.ids] = base
            if dt > 0.8: self.stage = 'stood'
        return self.stage
