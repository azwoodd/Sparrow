"""Aligned-thrust hop: first experiment, no optimisation. Fix a crouch depth, compute the collinear crouch offset, then
raise the spring energy and watch take-off spin, clearance and landing (full physics, aero on, one sim at a time).
  python h6_aligned.py [dz_c ...]"""
import json, os, sys
os.environ.setdefault('SPARROW_VARIANT', 'v7e_stiffcontact')
import numpy as np
from lab import Lab, WS, VARIANT, arr
from hop import analyse_hops
from hop2 import AlignedHopper, aligned_dy
from h2_opt import Library

OUT = WS / 'results' / 'h6_aligned' / VARIANT; OUT.mkdir(parents=True, exist_ok=True)
lab = Lab(); lab.reset({})
lib = Library(); base = dict(lib.d['ladder_E025']['params'])       # landing/flight settings of the robust 44 mm hop
res = []


def run(p, case=None, tag=''):
    ctl = AlignedHopper([(0.5, p)])
    fall = lab.run(ctl, 4.5, case=case, stop_after_fall=0.2, every=2)
    hops = analyse_hops(lab.rows, lab.weight); h = hops[0] if hops else None
    r = {'tag': tag, 'E': p['E'], 'dz_c': p['dz_c'], 'dy_c': ctl.hops[0][1]['dy_c'], 'kh': p.get('kh', 0), 'fall': fall, 'flights': len(hops), 'stored': ctl.stored, 'peak_tau': ctl.peak_tau}
    if h:
        t = arr(lab.rows, 't'); k0 = int(np.searchsorted(t, h['t_takeoff'])); k1 = min(int(np.searchsorted(t, h['t_touchdown'])), len(t) - 1)
        pit = arr(lab.rows, 'pitch_deg')
        r.update(clear=h['peak_clear_mm'], fwd=h['forward_mm'], good=h['good'], pitchTO=pit[k0], pitchTD=pit[k1], rate=(pit[k1] - pit[k0]) / max(h['flight_s'], 1e-6),
                 tilt_after=h['max_tilt_after_deg'], landF=max((x['fz_L'] + x['fz_R'] for x in lab.rows[k1:k1 + 150]), default=0.0), flight_s=h['flight_s'])
        print(f"{tag:10} E={p['E']:.2f} dz_c={p['dz_c']:.3f} dy_c={r['dy_c'] * 1000:+5.1f}mm kh={r['kh']:+.3f} clr={r['clear']:5.1f} fwd={r['fwd']:6.1f} fl={r['flight_s']:.3f}s pitchTO={r['pitchTO']:+5.1f} "
              f"pitchTD={r['pitchTD']:+6.1f} spin={r['rate']:+6.0f}deg/s landF={r['landF']:5.1f}N tiltAfter={r['tilt_after']:5.1f} flights={len(hops)} good={r['good']} fall={fall}", flush=True)
    else:
        print(f"{tag:10} E={p['E']:.2f} dz_c={p['dz_c']:.3f} no flight fall={fall}", flush=True)
    res.append(r); return r


for dz in [float(a) for a in sys.argv[1:]] or (0.030, 0.045):
    dy, resid, lean = aligned_dy(lab, dz)
    print(f"--- crouch depth {dz * 1000:.0f} mm: collinear ankle offset {dy * 1000:+.1f} mm (residual {resid:.3f} mm), push line leans {lean:.1f} deg forward")
    for E in (0.20, 0.30, 0.40, 0.50, 0.60):
        run(dict(base, dz_c=dz, E=E, align=1, kh=0.0), tag='aligned')
    for E in (0.40, 0.60):
        for kh in (-0.03, 0.03):
            run(dict(base, dz_c=dz, E=E, align=1, kh=kh), tag='hipdamp')
        for trim in (-0.010, 0.010):
            run(dict(base, dz_c=dz, E=E, align=1, dy_trim=trim), tag=f'trim{int(trim * 1000):+d}')
(OUT / 'aligned_energy_sweep.json').write_text(json.dumps(res, indent=1, default=float))
