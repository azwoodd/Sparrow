"""Can the landing window be widened toward nose-down arrivals (what the take-off actually delivers) by placing the feet
further forward and landing deeper? Touchdown-only test at the 60 mm drop speed; states: pitch 0..+15 deg nose-down,
pitch rate 0 / 60 deg/s, forward speed 0.1 / 0.3 m/s. One simulation at a time."""
import json, os
os.environ.setdefault('SPARROW_VARIANT', 'v7e_stiffcontact')
from lab import Lab, WS, VARIANT
from hop import DEFAULT
from h2_opt import Library
import t1_touchdown as t1

lab = Lab(); lib = Library(); base = {**DEFAULT, **lib.d['ladder_E025']['params']}
states = [(60, vf, pitch, rate) for vf in (0.1, 0.3) for pitch in (0, 5, 10, 15) for rate in (0, 60)]
res = []
print('each row: 16 states = [vfwd 0.1: pitch 0,5,10,15 x rate 0,60] [vfwd 0.3: same]')
for dz in (0.030, 0.045):
    for fwd in (0.015, 0.030, 0.045, 0.060):
        for fp in (0.0, -0.25):
            p = dict(base, dz_l=dz, dy_l=base['dy_l'] - fwd, fp_l=fp)
            cells = []
            for (h, vf, pitch, rate) in states:
                r = t1.run(lab, p, h, vf, pitch, rate); r.update(dz_l=dz, feet_fwd=fwd, fp_l=fp); res.append(r); cells.append('P' if r['PASS'] else '.')
            s = ''.join(cells)
            print(f"land depth {dz * 1000:.0f}mm feet +{fwd * 1000:.0f}mm fwd toe {'up 14deg' if fp else 'level  '} : {s[:8]} {s[8:]}  {s.count('P')}/16", flush=True)
(WS / 'results' / 't1_touchdown' / VARIANT / 'landing_window.json').write_text(json.dumps(res, indent=1, default=float))
