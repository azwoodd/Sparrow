"""Phase 1a: lateral weight-shift authority of every EXISTING actuator (no model change, no root manipulation).

Each test ramps one command 0.5-1.5 s, holds to 5 s under the owner's feedback controller and reports the steady-state
(last 1 s) COM shift, per-foot load share, centre of pressure and body roll, plus whether the joint actually moved.
"""
import json, math, sys
import numpy as np
from lab import Lab, FeedbackBase, WS, arr, mujoco
import legs

OUT = WS / 'results' / 'p1_authority'
OUT.mkdir(parents=True, exist_ok=True)
STOW = 1.51844


def ramp(t, t0=0.5, T=1.0):
    x = min(max((t - t0) / T, 0.0), 1.0)
    return x * x * (3 - 2 * x)


class Shift(FeedbackBase):
    """targets: {actuator joint name: final target}. leg_dz: (dz_L, dz_R) ankle height change in m (+ = leg shorter)."""
    def __init__(self, targets=None, leg_dz=(0.0, 0.0), feedback=True):
        self.targets = targets or {}; self.leg_dz = leg_dz; self.feedback = feedback

    def start(self, lab):
        super().start(lab)
        self.q0 = {j: float(lab.d.ctrl[lab.act['act_' + j]]) for j in self.targets}
        b = lab.e.base[lab.ids]
        self.base_leg = b.copy()
        self.p0 = [legs.fk(b[0], b[1]), legs.fk(b[3], b[4])]

    def delta(self, lab, t):
        a = ramp(t); d = np.zeros(6)
        for k, dz in enumerate(self.leg_dz):
            if dz:
                p = self.p0[k] + np.array([0.0, dz * a])
                d[3 * k:3 * k + 3] = legs.leg_targets(p[0], p[1]) - self.base_leg[3 * k:3 * k + 3]
        return d

    def extra(self, lab, t):
        a = ramp(t)
        for j, v in self.targets.items():
            lab.d.ctrl[lab.act['act_' + j]] = self.q0[j] + a * (v - self.q0[j])


def tests():
    T = []
    T.append(('00_control_none', {}, (0, 0)))
    for v in (0.5236, -0.5236):
        T.append((f'tail_roll_{v:+.2f}', {'tail_roll': v}, (0, 0)))
    T.append(('tail_pitch_+0.61', {'tail_pitch': 0.61}, (0, 0)))
    for v in (1.0472, -1.0472):
        T.append((f'neck_yaw_{v:+.2f}', {'neck_yaw': v}, (0, 0)))
    for frac in (0.25, 0.5, 0.75, 1.0):
        T.append((f'wingL_deploy_{int(frac * 100)}pct', {'wing_L_sweep': STOW * (1 - frac)}, (0, 0)))
    T.append(('wingR_deploy_100pct', {'wing_R_sweep': 0.0}, (0, 0)))
    T.append(('wingL_deploy_100_flap_up', {'wing_L_sweep': 0.0, 'wing_L_flap': -0.54}, (0, 0)))
    T.append(('wingL_deploy_100_flap_down', {'wing_L_sweep': 0.0, 'wing_L_flap': 0.54}, (0, 0)))
    T.append(('wingL_stowed_flap_up', {'wing_L_flap': -0.54}, (0, 0)))
    T.append(('wingL_stowed_flap_down', {'wing_L_flap': 0.54}, (0, 0)))
    T.append(('wingL_deploy100_neckL_tailroll', {'wing_L_sweep': 0.0, 'neck_yaw': 1.0472, 'tail_roll': 0.5236}, (0, 0)))
    T.append(('wingL_deploy100_neckR_tailroll-', {'wing_L_sweep': 0.0, 'neck_yaw': -1.0472, 'tail_roll': -0.5236}, (0, 0)))
    for mm in (2, 4, 6, 8, 12, 16):
        T.append((f'legdiff_L_long_R_short_{mm}mm', {}, (-mm / 2000.0, mm / 2000.0)))
    for mm in (4, 8, 16, 24):
        T.append((f'leg_R_short_only_{mm}mm', {}, (0.0, mm / 1000.0)))
    for mm in (4, 8):
        T.append((f'leg_L_long_only_{mm}mm', {}, (-mm / 1000.0, 0.0)))
    return T


if __name__ == '__main__':
    lab = Lab(); res = []
    jadr = lambda j: lab.s.jq[j]
    for name, targets, dz in tests():
        ctl = Shift(targets, dz)
        fall = lab.run(ctl, 5.0, stop_after_fall=0.3)
        rows = lab.rows; ss = [r for r in rows if r['t'] >= rows[-1]['t'] - 1.0]
        r0 = [r for r in rows if 0.3 <= r['t'] < 0.5]
        mean = lambda rr, k: float(np.nanmean(arr(rr, k)))
        achieved = {j: [round(float(v), 3), round(float(lab.d.qpos[jadr(j)]), 3)] for j, v in targets.items()}
        rec = {'name': name, 'fell': fall, 'commanded_vs_achieved': achieved,
               'com_x_shift_mm': (mean(ss, 'com_x') - mean(r0, 'com_x')) * 1000, 'com_x_abs_mm': mean(ss, 'com_x') * 1000,
               'cop_x_mm': mean(ss, 'cop_x') * 1000, 'load_L': mean(ss, 'fz_L') / lab.weight, 'load_R': mean(ss, 'fz_R') / lab.weight,
               'roll_deg': mean(ss, 'roll_deg'), 'pitch_deg': mean(ss, 'pitch_deg'), 'tilt_max_deg': float(arr(rows, 'tilt_deg').max()),
               'com_margin_L_mm': mean(ss, 'com_margin_L') * 1000, 'com_margin_R_mm': mean(ss, 'com_margin_R') * 1000,
               'foot_tilt_L_deg': mean(ss, 'foot_tilt_deg_L'), 'foot_tilt_R_deg': mean(ss, 'foot_tilt_deg_R'),
               'sat_frac': float((arr(ss, 'n_saturated') > 0).mean()), 'other_contacts': sorted({r['other_contacts'] for r in ss} - {''})}
        res.append(rec)
        lab.save_trace(OUT / f'{name}.csv', decimate=10)
        print(f"{name:34} fall={str(fall)[:5]:>5} dCOMx={rec['com_x_shift_mm']:+6.1f}mm CoPx={rec['cop_x_mm']:+6.1f} loadL/R={rec['load_L']:.2f}/{rec['load_R']:.2f} "
              f"roll={rec['roll_deg']:+5.2f} marginL/R={rec['com_margin_L_mm']:+5.1f}/{rec['com_margin_R_mm']:+5.1f}mm sat={rec['sat_frac']:.2f} ach={achieved}", flush=True)
    (OUT / 'summary.json').write_text(json.dumps(res, indent=1))
