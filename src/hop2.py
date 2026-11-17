"""Aligned-thrust hop controller (new, separate from hop.Hopper which stays as the preserved low-hop controller).

Why: the knee spring is the only strong actuator. A torque at the knee alone makes the ground force act along the line
ankle -> hip. If the whole-robot centre of mass is not on that line the push spins the body in pitch, and during a strong
push (30-40 N) the 0.2 Nm ankle and 0.35 Nm hip servos can only move that line by a few millimetres. So:
  1. choose the crouch fore-aft offset so that ankle, hip and centre of mass are collinear (computed from the model),
  2. during thrust the hip servo does NOT drive to a pose; it follows the joint and only adds a pitch-rate damping term,
  3. everything else (spring model, flight, landing) is inherited unchanged from hop.Hopper.
"""
import numpy as np
import mujoco
import legs
from hop import Hopper


def aligned_dy(lab, dz, p0=None):
    """Fore-aft ankle offset (m, relative to the standing pose) that puts the COM on the ankle->hip line for crouch depth dz.
    Pure kinematics on the model (feet flat, torso level). Returns (dy, residual_mm, lean_deg)."""
    m, d = lab.m, lab.d
    q_save, v_save = d.qpos.copy(), d.qvel.copy()
    b = lab.e.base[lab.ids]
    p0 = legs.fk(b[0], b[1]) if p0 is None else p0
    hipb = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, 'leg_L_thigh'); ankb = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, 'leg_L_ankle')

    def miss(dy):
        q = legs.leg_targets(p0[0] + dy, p0[1] + dz)
        for k in range(2):
            d.qpos[lab.qi[3 * k:3 * k + 3]] = q
        mujoco.mj_forward(m, d)
        hip, ank, com = d.xpos[hipb], d.xpos[ankb], d.subtree_com[1]
        a = np.array([hip[1] - ank[1], hip[2] - ank[2]]); c = np.array([com[1] - ank[1], com[2] - ank[2]])
        return float(a[0] * c[1] - a[1] * c[0]) / float(np.linalg.norm(a)), float(np.degrees(np.arctan2(-(hip[1] - ank[1]), hip[2] - ank[2])))

    lo, hi = -0.06, 0.08
    flo = miss(lo)[0]
    for _ in range(40):
        mid = 0.5 * (lo + hi); fm = miss(mid)[0]
        if (fm > 0) == (flo > 0):
            lo, flo = mid, fm
        else:
            hi = mid
    dy = 0.5 * (lo + hi); res, lean = miss(dy)
    d.qpos[:] = q_save; d.qvel[:] = v_save; mujoco.mj_forward(m, d)
    return dy, res * 1000, lean


class AlignedHopper(Hopper):
    """Extra hop parameters: kh (hip pitch-rate damping gain, rad per rad/s), kp_h (hip pitch-angle gain), align (0/1: replace
    dy_c by the collinear value for the given dz_c), dy_trim (m, added to the aligned offset)."""

    def start(self, lab):
        super().start(lab)
        for i, (t, H) in enumerate(self.hops):
            if H.get('align', 1):
                dy, res, lean = aligned_dy(lab, H['dz_c'], self.p0)
                H['dy_c'] = dy + H.get('dy_trim', 0.0); H['_lean_deg'] = lean
        self.pitch0 = 0.0
        self.hip_lim = float(lab.m.actuator_forcerange[lab.ids[0], 1] / lab.m.actuator_gainprm[lab.ids[0], 0])     # rad of target lead = servo torque limit

    def __call__(self, lab, t):
        was = self.phase
        super().__call__(lab, t)
        d, m = lab.d, lab.m
        if self.phase == 'thrust':
            H = self.hops[self.i][1]
            pitch, roll, v = lab.e.state()
            if was != 'thrust':
                self.pitch0 = pitch
            # hip torque during the push = steady bias (hip_bias, Nm; tunes take-off angular momentum) + pitch damping, <= 0.35 Nm
            u = float(np.clip(H.get('hip_bias', 0.0) / 4.0 + H.get('kh', 0.0) * v[0] + H.get('kp_h', 0.0) * (pitch - self.pitch0), -self.hip_lim, self.hip_lim))
            for k in range(2):
                if H.get('hip_follow', 1):
                    tgt = float(np.clip(d.qpos[lab.qi[3 * k]] + u, m.actuator_ctrlrange[lab.ids[3 * k], 0], m.actuator_ctrlrange[lab.ids[3 * k], 1]))
                    d.ctrl[lab.ids[3 * k]] = tgt; self.last[3 * k] = tgt
                if H.get('ank_follow', 0):
                    # ankle servo commanded to its own angle (+ optional bias torque ank_bias Nm) instead of chasing a pose:
                    # a saturated 0.2 Nm ankle during a 16 ms push is enough to spin the body by itself
                    j = 3 * k + 2
                    tgt = float(np.clip(d.qpos[lab.qi[j]] + H.get('ank_bias', 0.0) / 2.5, m.actuator_ctrlrange[lab.ids[j], 0], m.actuator_ctrlrange[lab.ids[j], 1]))
                    d.ctrl[lab.ids[j]] = tgt; self.last[j] = tgt

