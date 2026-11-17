"""One-knob take-off tuning: a steady hip torque during the push (hip_bias, within the 0.35 Nm servo limit) to null the
take-off angular momentum. Ankles follow (no saturation). Sweep the bias at three spring energies; landing settings are
passed in so the same script can re-test with a better landing pose. Positive pitch = nose down.
  python t3_hip_bias.py [landing-variant]      variants: robust | deep_fwd"""
import json, os, sys
os.environ.setdefault('SPARROW_VARIANT', 'v7e_stiffcontact')
import numpy as np, mujoco
from lab import Lab, WS, VARIANT, arr
from hop import analyse_hops
from hop2 import AlignedHopper
from h2_opt import Library

OUT = WS / 'results' / 't3_hip_bias' / VARIANT; OUT.mkdir(parents=True, exist_ok=True)
lab = Lab(); lib = Library(); base = dict(lib.d['ladder_E025']['params'], dz_c=0.030, dz_e=-0.020, align=1, hip_follow=1, ank_follow=1)
LAND = {'robust': {}, 'deep_fwd': {'dz_l': 0.030, 'dy_l': base['dy_l'] - 0.015}}
orig = lab.measure
def measure():
    r = orig(); mujoco.mj_subtreeVel(lab.m, lab.d)
    r['Lx'] = float(lab.d.subtree_angmom[1][0]); r['vz'] = float(lab.d.subtree_linvel[1][2]); r['vfwd'] = float(-lab.d.subtree_linvel[1][1]); return r
lab.measure = measure


def run(p, case=None):
    ctl = AlignedHopper([(0.5, p)])
    fall = lab.run(ctl, 4.5, case=case, stop_after_fall=0.2, every=1)
    hops = analyse_hops(lab.rows, lab.weight); h = hops[0] if hops else None
    r = {'fall': fall, 'flights': len(hops), 'peak_tau': ctl.peak_tau, 'stored': ctl.stored, 'good': False, **{k: p.get(k) for k in ('E', 'hip_bias', 'dz_c', 'dz_l', 'dy_l')}}
    if h:
        t = arr(lab.rows, 't'); k0 = int(np.searchsorted(t, h['t_takeoff'])); k1 = min(int(np.searchsorted(t, h['t_touchdown'])), len(t) - 1)
        a, b = lab.rows[k0], lab.rows[k1]
        r.update(clear=h['peak_clear_mm'], fwd=h['forward_mm'], L=a['Lx'] * 1000, vz=a['vz'], vfwd=a['vfwd'], pitch_TO=a['pitch_deg'], pitch_TD=b['pitch_deg'],
                 tilt_after=h['max_tilt_after_deg'], landF=max((x['fz_L'] + x['fz_R'] for x in lab.rows[k1:k1 + 300]), default=0.0),
                 good=bool(h['good'] and fall is None and len(hops) == 1))
    return r


if __name__ == '__main__':
    land = sys.argv[1] if len(sys.argv) > 1 else 'robust'; res = []
    for E in (0.40, 0.50, 0.60):
        for hb in (0.35, 0.2, 0.1, 0.0, -0.1, -0.2, -0.35):
            r = run(dict(base, E=E, hip_bias=hb, **LAND[land])); r['landing'] = land; res.append(r)
            if 'clear' in r:
                print(f"E={E:.2f} hip_bias={hb:+.2f}Nm  L={r['L']:+6.2f}mNms vz={r['vz']:.2f} vfwd={r['vfwd']:+.2f} clr={r['clear']:5.1f} fwd={r['fwd']:6.1f} pitchTO={r['pitch_TO']:+5.1f} "
                      f"pitchTD={r['pitch_TD']:+6.1f} landF={r['landF']:5.1f}N tiltAfter={r['tilt_after']:5.1f} flights={r['flights']} good={r['good']} fall={r['fall']}", flush=True)
            else:
                print(f"E={E:.2f} hip_bias={hb:+.2f} no flight fall={r['fall']}", flush=True)
    (OUT / f'bias_sweep_{land}.json').write_text(json.dumps(res, indent=1, default=float))
