"""Keyframed command scripts on top of the owner's feedback controller. Commands go ONLY to existing actuators.

Tracks (all optional): any actuator joint name (absolute target, e.g. 'wing_L_sweep'), and per leg S in L/R:
  S_dy, S_dz : ankle displacement in the torso frame relative to the standing pose (m). dy<0 = forward, dz>0 = leg shorter
  S_pitch    : extra foot pitch (rad)
Keyframes are (time, value) pairs joined by smoothstep; before the first key the reset value is held.
"""
import numpy as np
from lab import FeedbackBase
import legs

STOW = 1.51844


def smooth(x):
    x = min(max(x, 0.0), 1.0)
    return x * x * (3 - 2 * x)


def track_value(keys, t, v0):
    if not keys:
        return v0
    if t <= keys[0][0]:
        return v0 if keys[0][0] > 0 else keys[0][1]
    prev_t, prev_v = None, v0
    for i, (kt, kv) in enumerate(keys):
        if t < kt:
            # ramp occupies [start, kt] where start = previous key time (or a 'hold' key)
            return prev_v + (kv - prev_v) * smooth((t - prev_t) / (kt - prev_t)) if prev_t is not None else v0
        prev_t, prev_v = kt, kv
    return prev_v


class Script(FeedbackBase):
    """tracks: {name: [(t, value), ...]}. A ramp runs between consecutive keys; repeat a value to hold it."""
    def __init__(self, tracks, feedback=True, act_slew=0.06):
        self.tracks = {k: sorted(v) for k, v in tracks.items()}
        self.feedback = feedback
        self.act_slew = act_slew        # rad per 10 ms tick for non-leg position servos (6 rad/s)

    def start(self, lab):
        super().start(lab)
        b = lab.e.base[lab.ids]
        self.base_leg = b.copy()
        self.p0 = {'L': legs.fk(b[0], b[1]), 'R': legs.fk(b[3], b[4])}
        self.act_names = [k for k in self.tracks if 'act_' + k in lab.act]
        self.v0 = {k: float(lab.d.ctrl[lab.act['act_' + k]]) for k in self.act_names}
        self.cur = dict(self.v0)

    def val(self, name, t, default=0.0):
        return track_value(self.tracks.get(name), t, default)

    def delta(self, lab, t):
        d = np.zeros(6)
        for k, S in enumerate('LR'):
            dy, dz, dp = self.val(S + '_dy', t), self.val(S + '_dz', t), self.val(S + '_pitch', t)
            if dy or dz or dp:
                p = self.p0[S] + np.array([dy, dz])
                d[3 * k:3 * k + 3] = legs.leg_targets(p[0], p[1], dp) - self.base_leg[3 * k:3 * k + 3]
        return d

    def __call__(self, lab, t):
        """As FeedbackBase, but a leg whose 'S_swing' track is > 0.5 gets a world-level foot (ankle target from measured
        hip/knee angles and IMU pitch) instead of the stance pitch-feedback ankle term, so the pad lifts flat."""
        e, d, m = lab.e, lab.d, lab.m
        e.command('feedback' if self.feedback else 'baseline', self.gains if hasattr(self, 'gains') else __import__('lab').GAINS)
        base = d.ctrl[lab.ids].copy()
        tgt = base + self.delta(lab, t)
        pitch = lab.e.state()[0]
        for k, S in enumerate('LR'):
            if self.val(S + '_swing', t) > 0.5:
                q1, q2 = d.qpos[lab.qi[3 * k]], d.qpos[lab.qi[3 * k + 1]]
                tgt[3 * k + 2] = -(q1 + q2) - pitch + self.val(S + '_pitch', t)
        tgt = np.clip(tgt, m.actuator_ctrlrange[lab.ids, 0], m.actuator_ctrlrange[lab.ids, 1])
        tgt = np.clip(tgt, self.last - 0.03, self.last + 0.03)
        d.ctrl[lab.ids] = tgt
        self.last = tgt.copy()
        self.extra(lab, t)

    def extra(self, lab, t):
        for k in self.act_names:
            want = track_value(self.tracks[k], t, self.v0[k])
            self.cur[k] = float(np.clip(want, self.cur[k] - self.act_slew, self.cur[k] + self.act_slew))
            lab.d.ctrl[lab.act['act_' + k]] = self.cur[k]
