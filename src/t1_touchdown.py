"""Touchdown-only test: can the existing legs and servos absorb a landing and recover, separately from take-off?

Sparrow is INITIALISED 20 mm above the ground in its landing pose with a chosen touchdown speed, forward speed, pitch angle
and pitch rate (an imposed initial condition - flagged as such; nothing is pushed or pinned afterwards). The landing part
of the preserved hop controller then runs: level feet in flight, absorb, recover, stand. No spring is attached.
Drop height h is quoted as the foot clearance that gives that touchdown speed: v = sqrt(2 g h).
  python t1_touchdown.py grid            envelope with the robust-hop landing settings
  python t1_touchdown.py variants        same states, a few landing-pose variants
PASS = upright (tilt < 20 deg, COM > 90 mm, no body contact) for 2 s after first foot contact.
"""
import json, math, os, sys
os.environ.setdefault('SPARROW_VARIANT', 'v7e_stiffcontact')
import numpy as np, mujoco
from lab import Lab, WS, VARIANT, arr, G
from hop import Hopper, DEFAULT
import legs
from h2_opt import Library

OUT = WS / 'results' / 't1_touchdown' / VARIANT; OUT.mkdir(parents=True, exist_ok=True)


class Lander(Hopper):
    """Preserved hop controller started directly in its flight phase (spring never engaged)."""
    def start(self, lab):
        super().start(lab)
        self.phase = 'flight'; self.t_phase = -1.0; self.i = 0


def make_state(p, h_mm, vfwd, pitch_deg, rate_deg_s):
    def pre(lab):
        m, d = lab.m, lab.d
        b = lab.e.base[lab.ids]; p0 = legs.fk(b[0], b[1])
        q = legs.leg_targets(p0[0] + p['dy_l'], p0[1] + p['dz_l'], p['fp_l'])
        a = math.radians(pitch_deg)
        for k in range(2):
            q3 = q.copy(); q3[2] = -(q[0] + q[1]) - a + p['fp_l']          # feet level in the world, as the controller holds them
            d.qpos[lab.qi[3 * k:3 * k + 3]] = q3; d.ctrl[lab.ids[3 * k:3 * k + 3]] = q3
        d.qpos[3:7] = [math.cos(a / 2), math.sin(a / 2), 0, 0]
        mujoco.mj_forward(m, d)
        low = min(lab.foot_pose(S)[1][:, 2].min() for S in 'LR')
        d.qpos[2] += 0.020 - low
        v = math.sqrt(2 * G * h_mm / 1000.0); v0 = math.sqrt(max(v * v - 2 * G * 0.020, 0.0))
        d.qvel[:] = 0; d.qvel[1] = -vfwd; d.qvel[2] = -v0; d.qvel[3] = math.radians(rate_deg_s)
        mujoco.mj_forward(m, d)
    return pre


def run(lab, p, h_mm, vfwd, pitch, rate, case=None):
    ctl = Lander([(0.0, p)])
    fall = lab.run(ctl, 3.2, case=case, stop_after_fall=0.2, pre_reset=make_state(p, h_mm, vfwd, pitch, rate), every=2)
    rows = lab.rows; t = arr(rows, 't')
    F = arr(rows, 'fz_L') + arr(rows, 'fz_R')
    k = int(np.argmax(F > 0)) if (F > 0).any() else len(rows) - 1
    after = rows[k:]
    ok = bool(fall is None and after and rows[-1]['t'] - rows[k]['t'] >= 2.0 and max(r['tilt_deg'] for r in after) < 20 and min(r['com_z'] for r in after) > 0.09
              and all(r['fz_other'] <= 0 for r in after))
    win = rows[k:k + 150]
    return {'h_mm': h_mm, 'v_touch': math.sqrt(2 * G * h_mm / 1000), 'vfwd': vfwd, 'pitch0': pitch, 'rate0': rate, 'PASS': ok, 'fall': fall,
            'peak_force_N': float(F[k:k + 150].max()) if len(F) > k else 0.0, 'peak_force_BW': float(F[k:k + 150].max() / lab.weight) if len(F) > k else 0.0,
            'max_tilt_after': float(max(r['tilt_deg'] for r in after)), 'pitch_peak': float(max(r['pitch_deg'] for r in after)), 'pitch_min': float(min(r['pitch_deg'] for r in after)),
            'knee_max_rad': float(max(r['leg_L_shin_q'] for r in win)), 'sat_frac_0.3s': float(np.mean([r['n_saturated'] > 0 for r in win])),
            'bounce': bool(any(F[k + 5:k + 200] <= 0)) if len(F) > k + 200 else False, 'fwd_slide_mm': float(-(after[-1]['com_y'] - after[0]['com_y']) * 1000)}


if __name__ == '__main__':
    mode = sys.argv[1] if len(sys.argv) > 1 else 'grid'
    lab = Lab(); lib = Library(); base = {**DEFAULT, **lib.d['ladder_E025']['params']}
    res = []
    if mode == 'grid':
        print('landing settings:', {k: round(base[k], 4) for k in ('dy_l', 'dz_l', 'fp_l', 't_a', 't_r')})
        for h in (30, 47, 60, 80, 100):
            for vf in (0.0, 0.3):
                line = f"h={h:3d}mm v={math.sqrt(2 * G * h / 1000):.2f}m/s vfwd={vf:.1f} | "
                for rate in (0, 60, 120):
                    cells = []
                    for pitch in (-10, -5, 0, 5, 10, 15):
                        r = run(lab, base, h, vf, pitch, rate); res.append(r); cells.append('P' if r['PASS'] else '.')
                    line += f"rate {rate:3d}: {''.join(cells)}   "
                print(line + f"| peakF nominal {next(x['peak_force_N'] for x in res if x['h_mm'] == h and x['vfwd'] == vf and x['rate0'] == 0 and x['pitch0'] == 0):.0f}N", flush=True)
        print('columns within each rate block: pitch at touchdown = -10 -5 0 +5 +10 +15 deg (positive = nose up); P = upright 2 s')
    elif mode == 'variants':
        V = {'robust_hop_landing': {}, 'deeper_absorb_dz25': {'dz_l': 0.025}, 'deeper_dz35': {'dz_l': 0.035}, 'feet_fwd_+15': {'dy_l': base['dy_l'] - 0.015},
             'feet_back_+15': {'dy_l': base['dy_l'] + 0.015}, 'long_absorb_0.6s': {'t_a': 0.6}, 'slow_recover_1.2s': {'t_r': 1.2},
             'deeper+feet_fwd': {'dz_l': 0.030, 'dy_l': base['dy_l'] - 0.015}}
        states = [(h, 0.3, pitch, rate) for h in (60, 80, 100) for pitch in (-5, 0, 5, 10) for rate in (0, 60)]
        for name, ch in V.items():
            p = dict(base, **ch); n = 0; per = {60: 0, 80: 0, 100: 0}
            for (h, vf, pitch, rate) in states:
                r = run(lab, p, h, vf, pitch, rate); r['variant'] = name; res.append(r); n += r['PASS']; per[h] += r['PASS']
            print(f"{name:22} {n:2d}/{len(states)} pass   by height 60/80/100 mm: {per[60]}/8 {per[80]}/8 {per[100]}/8", flush=True)
    (OUT / f'{mode}.json').write_text(json.dumps(res, indent=1, default=float))
    print(f"{mode}: {sum(r['PASS'] for r in res)}/{len(res)} pass")
