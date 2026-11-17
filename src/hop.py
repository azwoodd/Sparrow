"""Hop controller for the V7E legs (existing joints only) with an ENERGY-LIMITED knee spring model.

Spring model (stands in for the 'Knee jump spring' + 'Jump winder gearmotor' placeholders in the CAD):
  * A torsion spring on each knee is wound and latched while the leg is in stance; the latch isolates it from the leg.
  * On release it acts as a one-sided torsion spring tau = -k * max(0, q_knee - q_free): it pushes the knee straight,
    goes slack at q_free, and pushes again if the knee flexes past q_free on landing (a cushion). It is slackened by
    the winder once the robot is standing again.
  * k is set from the stored energy E (J per knee) and the knee angle at release: E = 1/2 k (q_rel - q_free)^2.
  So the energy available is exactly E per knee per hop (elastic, no added power), unlike the ideal torque pulse of V3/V4.
  The torque is applied through the existing 'jump_spring_*' motor actuators (ctrl range +-3 Nm) at 1 kHz.

Leg servos: same position actuators/limits as the model. Command slew limit SLEW rad/s (default 10 rad/s ~ 0.10 s/60 deg
hobby servo) - the owner's 3 rad/s walking slew would make the servos fight the spring during the 80 ms thrust.
Pitch feedback in stance: same law and gains as the owner's controller (gains.json), applied to the ankles.

A hop is a dict of parameters (metres / radians / seconds / joules):
  crouch (dy_c, dz_c)          ankle offset from the standing pose while crouched (dy>0 = feet back = body leans forward)
  thrust (dy_e, dz_e, fp_e)    leg extension target + foot pitch (plantar-flexion) during thrust
  E, dE                        spring energy per knee; dE = left-right difference fraction (for turning/lateral hops)
  flight (dy_l, dz_l, fp_l)    landing leg pose, foot held level in the world + fp_l
  sc                           scissor: +sc on left ankle dy, -sc on right during thrust (yaw)
  times: t_c crouch, t_h hold, t_a absorb, t_r recover
"""
import math
import numpy as np
from lab import GAINS
import legs

DEFAULT = dict(dy_c=0.0, dz_c=0.035, dy_e=0.0, dz_e=-0.020, fp_e=0.0, E=0.12, dE=0.0, dy_l=0.0, dz_l=0.010, fp_l=0.0, sc=0.0,
               dzd=0.0, t_c=0.5, t_h=0.25, t_a=0.25, t_r=0.6, dy_s=0.0, kq=0.0)


def smooth(x):
    x = min(max(x, 0.0), 1.0)
    return x * x * (3 - 2 * x)


class Hopper:
    """Sequence of hops. `hops` = list of (start_time, params). Between hops the robot stands under pitch feedback."""
    SLEW = 10.0

    def __init__(self, hops, pre=None, pitch_gain=1.0):
        self.hops = [(t, {**DEFAULT, **p}) for t, p in hops]
        self.pre = pre                      # optional controller to run before the first hop (e.g. walking Script)
        self.pitch_gain = pitch_gain
        self.log = []

    def start(self, lab):
        if self.pre is not None and hasattr(self.pre, 'start'):
            self.pre.start(lab)
        b = lab.e.base[lab.ids]
        self.base = b.copy()
        self.p0 = legs.fk(b[0], b[1])
        self.last = lab.d.ctrl[lab.ids].copy()
        self.i = 0; self.phase = 'stand'; self.t_phase = 0.0
        self.spring_on = [False, False]; self.k = [0.0, 0.0]; self.q_free = 0.0
        self.stand_pose = np.zeros(3)       # (dy, dz, fp) currently regarded as 'standing'
        self.pose_from = np.zeros(3)
        self.spring_work = 0.0; self.peak_tau = 0.0; self.stored = 0.0
        self.handover = False

    # --- helpers
    def leg_cmd(self, pose, extra_dy=(0.0, 0.0), dz_diff=0.0):
        out = np.zeros(6)
        for k in range(2):
            sgn = 1.0 if k == 0 else -1.0
            p = self.p0 + np.array([pose[0] + extra_dy[k], pose[1] + sgn * dz_diff])
            out[3 * k:3 * k + 3] = legs.leg_targets(p[0], p[1], pose[2])
        return out

    def pitch_fb(self, lab):
        p, r, v = lab.e.state()
        kp, kd = GAINS[0], GAINS[1]
        if p < 0:
            kp, kd = GAINS[5], GAINS[6]
        return float(np.clip(kp * p + kd * v[0], -0.45, 0.45)) * self.pitch_gain

    def set_phase(self, ph, t):
        self.phase = ph; self.t_phase = t
        self.log.append((round(t, 3), self.i, ph))

    def substep(self, lab):
        d = lab.d
        for k, S in enumerate('LR'):
            tau = 0.0
            if self.spring_on[k]:
                # one-sided torsion spring: pushes only while the knee is flexed past its free angle. It stays attached
                # through flight and landing (so it also cushions touchdown) and is slackened once standing again.
                q = d.qpos[lab.qi[3 * k + 1]]
                if q > self.q_free:
                    tau = -self.k[k] * (q - self.q_free)
                    self.spring_work += tau * d.qvel[lab.vi[3 * k + 1]] * lab.dt      # signed net work on the leg
                    self.peak_tau = max(self.peak_tau, -tau)
            d.ctrl[lab.act['jump_spring_' + S]] = max(-3.0, tau)

    def __call__(self, lab, t):
        d, m = lab.d, lab.m
        if self.pre is not None and (not self.hops or t < self.hops[0][0]) and self.i == 0 and self.phase == 'stand':
            self.pre(lab, t); self.last = d.ctrl[lab.ids].copy()
            # keep track of where the walking controller left the legs so the hop starts from there
            return
        if self.pre is not None and not self.handover:
            self.handover = True
            q = [d.ctrl[lab.ids[0]], d.ctrl[lab.ids[1]]]
        fzL, fzR = self._loads(lab)
        H = self.hops[self.i][1] if self.i < len(self.hops) else None
        ts = self.hops[self.i][0] if H else 1e9
        ph = self.phase; fb = True; level = False; extra = (0.0, 0.0); dzd = 0.0
        if ph == 'stand':
            pose = self.stand_pose
            if H and t >= ts:
                self.pose_from = self.stand_pose.copy(); self.set_phase('crouch', t)
        if self.phase == 'crouch':
            a = smooth((t - self.t_phase) / H['t_c'])
            tgt = np.array([H['dy_c'], H['dz_c'], 0.0])
            pose = self.pose_from + a * (tgt - self.pose_from); dzd = a * H['dzd']
            if t >= self.t_phase + H['t_c'] + H['t_h']:
                self.set_phase('thrust', t)
                q_rel = [d.qpos[lab.qi[1]], d.qpos[lab.qi[4]]]
                self.q_free = min(q_rel) - H['kq'] if H['kq'] else self.base[1]
                for k in range(2):
                    E = H['E'] * (1 + (H['dE'] if k == 0 else -H['dE']))
                    dq = max(q_rel[k] - self.q_free, 1e-3)
                    self.k[k] = 2 * E / dq ** 2 if E > 0 else 0.0
                    self.spring_on[k] = E > 0
                    self.stored += E
        if self.phase == 'thrust':
            pose = np.array([H['dy_e'], H['dz_e'], H['fp_e']]); fb = False; extra = (H['sc'], -H['sc']); dzd = H['dzd']
            if (fzL <= 0 and fzR <= 0 and t > self.t_phase + 0.02) or t > self.t_phase + 0.35:
                self.set_phase('flight', t)
        if self.phase == 'flight':
            pose = np.array([H['dy_l'], H['dz_l'], H['fp_l']]); fb = False; level = True
            if (fzL > 0 or fzR > 0) and t > self.t_phase + 0.03:
                self.set_phase('absorb', t)
        if self.phase == 'absorb':
            pose = np.array([H['dy_l'], H['dz_l'], 0.0])
            if t >= self.t_phase + H['t_a']:
                self.pose_from = pose.copy(); self.set_phase('recover', t)
        if self.phase == 'recover':
            a = smooth((t - self.t_phase) / H['t_r'])
            tgt = np.array([H['dy_s'], 0.0, 0.0])
            pose = self.pose_from + a * (tgt - self.pose_from)
            if a >= 1.0:
                self.stand_pose = tgt; self.i += 1; self.set_phase('stand', t)
                self.spring_on = [False, False]          # winder slackens the springs once standing
        tgt = self.leg_cmd(pose, extra, dzd)
        if level:
            pitch = lab.e.state()[0]
            for k in range(2):
                q1, q2 = d.qpos[lab.qi[3 * k]], d.qpos[lab.qi[3 * k + 1]]
                tgt[3 * k + 2] = -(q1 + q2) - pitch + pose[2]
        if fb:
            u = self.pitch_fb(lab)
            tgt[2] -= u; tgt[5] -= u
        tgt = np.clip(tgt, m.actuator_ctrlrange[lab.ids, 0], m.actuator_ctrlrange[lab.ids, 1])
        s = self.SLEW * 0.01
        tgt = np.clip(tgt, self.last - s, self.last + s)
        d.ctrl[lab.ids] = tgt; self.last = tgt.copy()

    def _loads(self, lab):
        cs = lab.contact_state()
        return [cs.get(lab.foot_geom[S], {'fn': 0.0})['fn'] for S in 'LR']


# ---------------------------------------------------------------------------------------------- hop analysis
def analyse_hops(rows, weight, settle_s=2.0):
    """Segment true flight phases (both feet unloaded AND no other part touching) and judge the landing after each.
    Returns list of hops: takeoff/touchdown time, peak clearance of the LOWER foot, displacement of the feet midpoint,
    and whether the robot stayed up (tilt < 20 deg, COM z > 0.09 m, no non-foot ground contact) until the next take-off
    or for `settle_s` after the final touchdown."""
    t = np.array([r['t'] for r in rows]); n = len(t)
    fzL = np.array([r['fz_L'] for r in rows]); fzR = np.array([r['fz_R'] for r in rows]); oth = np.array([r['fz_other'] for r in rows])
    clr = np.minimum([r['clear_min_L'] for r in rows], [r['clear_min_R'] for r in rows])
    tilt = np.array([r['tilt_deg'] for r in rows]); cz = np.array([r['com_z'] for r in rows])
    mx = (np.array([r['foot_x_L'] for r in rows]) + np.array([r['foot_x_R'] for r in rows])) / 2
    my = (np.array([r['foot_y_L'] for r in rows]) + np.array([r['foot_y_R'] for r in rows])) / 2
    yaw = np.unwrap(np.radians((np.array([r['foot_yaw_deg_L'] for r in rows]) + np.array([r['foot_yaw_deg_R'] for r in rows])) / 2))
    air = (fzL <= 0) & (fzR <= 0) & (oth <= 0) & (t > 0.2)
    dt = float(np.median(np.diff(t)))
    segs = []; i = 0
    while i < n:
        if air[i]:
            j = i
            while j < n and air[j]:
                j += 1
            if (j - i) * dt >= 0.02 and clr[i:j].max() > 0.002:
                segs.append((i, j))
            i = j
        else:
            i += 1
    # merge flights separated by a very brief touch (< 40 ms) = same hop bouncing
    hops = []
    for a, b in segs:
        if hops and (t[a] - hops[-1]['t_touchdown']) < 0.04:
            hops[-1]['_b'] = b; hops[-1]['t_touchdown'] = float(t[min(b, n - 1)]); hops[-1]['bounces'] += 1
            hops[-1]['peak_clear_mm'] = max(hops[-1]['peak_clear_mm'], float(clr[a:b].max() * 1000))
        else:
            hops.append({'_a': a, '_b': b, 't_takeoff': float(t[a]), 't_touchdown': float(t[min(b, n - 1)]), 'bounces': 0,
                         'peak_clear_mm': float(clr[a:b].max() * 1000)})
    for k, h in enumerate(hops):
        a, b = h.pop('_a'), min(h.pop('_b'), n - 1)
        pre = max(0, a - int(0.6 / dt))                     # feet position before the crouch/thrust
        end = (np.searchsorted(t, hops[k + 1]['t_takeoff']) if k + 1 < len(hops) else min(n, np.searchsorted(t, t[b] + settle_s) + 1))
        end = max(end, b + 1)
        last = slice(max(b, end - int(0.2 / dt)), end)
        h['flight_s'] = h['t_touchdown'] - h['t_takeoff']
        h['forward_mm'] = float(-(my[last].mean() - my[pre]) * 1000)
        h['left_mm'] = float((mx[last].mean() - mx[pre]) * 1000)
        h['yaw_deg'] = float(np.degrees(yaw[last].mean() - yaw[pre]))
        seg = slice(b, end)
        h['observed_after_s'] = float(t[end - 1] - t[b])
        h['max_tilt_after_deg'] = float(tilt[seg].max()); h['min_comz_after'] = float(cz[seg].min())
        h['other_contact_after'] = bool((oth[seg] > 0).any())
        h['tilt_at_touchdown_deg'] = float(tilt[b])
        final = (k + 1 == len(hops))
        h['stayed_up'] = bool(h['max_tilt_after_deg'] < 20 and h['min_comz_after'] > 0.09 and not h['other_contact_after']
                              and (not final or h['observed_after_s'] >= settle_s - 0.02))
        h['clear_ge_15mm'] = h['peak_clear_mm'] >= 15.0
        h['good'] = bool(h['stayed_up'] and h['clear_ge_15mm'])
    return hops

