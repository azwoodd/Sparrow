"""Aligned thrust + ACTIVE hip pitch stabilisation during the push (hip servos, 0.35 Nm each, are the only pitch authority
while the springs fire). Sweep the PD gains and spring energy; no optimisation. Full physics, one simulation at a time."""
import json, os, sys
os.environ.setdefault('SPARROW_VARIANT', 'v7e_stiffcontact')
import numpy as np
from lab import Lab, WS, VARIANT, arr
from hop import analyse_hops
from hop2 import AlignedHopper
from h2_opt import Library

OUT = WS / 'results' / 'h7_hip_pd' / VARIANT; OUT.mkdir(parents=True, exist_ok=True)
lab = Lab(); lib = Library(); base = dict(lib.d['ladder_E025']['params']); res = []


def run(p, case=None, tag='', quiet=False):
    ctl = AlignedHopper([(0.5, p)])
    fall = lab.run(ctl, 4.5, case=case, stop_after_fall=0.2, every=2)
    hops = analyse_hops(lab.rows, lab.weight); h = hops[0] if hops else None
    r = {'tag': tag, **{k: p.get(k) for k in ('E', 'dz_c', 'kh', 'kp_h', 'dy_trim')}, 'fall': fall, 'flights': len(hops), 'stored': ctl.stored, 'peak_tau': ctl.peak_tau, 'good': False}
    if h:
        t = arr(lab.rows, 't'); k0 = int(np.searchsorted(t, h['t_takeoff'])); k1 = min(int(np.searchsorted(t, h['t_touchdown'])), len(t) - 1)
        pit = arr(lab.rows, 'pitch_deg')
        r.update(clear=h['peak_clear_mm'], fwd=h['forward_mm'], good=bool(h['good'] and len(hops) == 1 and fall is None), pitchTO=pit[k0], pitchTD=pit[k1],
                 spin=(pit[k1] - pit[k0]) / max(h['flight_s'], 1e-6), tilt_after=h['max_tilt_after_deg'],
                 landF=max((x['fz_L'] + x['fz_R'] for x in lab.rows[k1:k1 + 150]), default=0.0))
        if not quiet:
            print(f"{tag:8} E={p['E']:.2f} kp_h={p.get('kp_h', 0):.1f} kh={p.get('kh', 0):.2f} trim={p.get('dy_trim', 0) * 1000:+3.0f} clr={r['clear']:5.1f} fwd={r['fwd']:6.1f} pitchTO={r['pitchTO']:+5.1f} "
                  f"pitchTD={r['pitchTD']:+6.1f} spin={r['spin']:+6.0f}deg/s landF={r['landF']:5.1f}N tiltAfter={r['tilt_after']:5.1f} flights={len(hops)} good={r['good']} fall={fall}", flush=True)
    elif not quiet:
        print(f"{tag:8} E={p['E']:.2f} no flight fall={fall}", flush=True)
    res.append(r); return r


if __name__ == '__main__':
    for E in (0.40, 0.60):
        for kp_h in (0.0, 1.0, 3.0):
            for kh in (0.05, 0.15):
                for trim in (0.0, 0.010, 0.020):
                    run(dict(base, dz_c=0.030, E=E, align=1, kh=kh, kp_h=kp_h, dy_trim=trim), tag='pd')
    (OUT / 'pd_sweep.json').write_text(json.dumps(res, indent=1, default=float))
