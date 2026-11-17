"""Hip authority over take-off spin = hip torque x push duration. The stiff spring pushes for ~20 ms, so 0.35 Nm hips can
remove only ~70 % of the +16 mNms. A softer, longer-stroke spring (free angle = release angle - kq) lengthens the push.
Sweep kq x energy x hip bias with ankles following; report push duration, take-off angular momentum and the landing.
  python t4_soft_bias.py [landing: robust|deep_fwd] [dz_c]"""
import json, os, sys
os.environ.setdefault('SPARROW_VARIANT', 'v7e_stiffcontact')
import numpy as np
from lab import WS, VARIANT, arr
import t3_hip_bias as t3

OUT = WS / 'results' / 't4_soft_bias' / VARIANT; OUT.mkdir(parents=True, exist_ok=True)
land = sys.argv[1] if len(sys.argv) > 1 else 'robust'; dz = float(sys.argv[2]) if len(sys.argv) > 2 else 0.045
res = []
for kq in (0.9, 1.3, 1.7):
    for E in (0.40, 0.50, 0.60):
        for hb in (0.35, 0.25, 0.15, 0.05, -0.05):
            p = dict(t3.base, dz_c=dz, dz_e=-0.026, kq=kq, E=E, hip_bias=hb, **t3.LAND[land])
            r = t3.run(p); r.update(kq=kq, landing=land); res.append(r)
            rows = t3.lab.rows
            if 'clear' in r:
                print(f"kq={kq:.1f} E={E:.2f} hip_bias={hb:+.2f}  peakTau={r['peak_tau']:.2f}Nm L={r['L']:+6.2f}mNms vz={r['vz']:.2f} vfwd={r['vfwd']:+.2f} clr={r['clear']:5.1f} fwd={r['fwd']:6.1f} "
                      f"pitchTO={r['pitch_TO']:+5.1f} pitchTD={r['pitch_TD']:+6.1f} landF={r['landF']:5.1f}N tiltAfter={r['tilt_after']:5.1f} flights={r['flights']} good={r['good']} fall={r['fall']}", flush=True)
            else:
                print(f"kq={kq:.1f} E={E:.2f} hip_bias={hb:+.2f} no flight fall={r['fall']}", flush=True)
(OUT / f'soft_bias_{land}_dz{int(dz * 1000)}.json').write_text(json.dumps(res, indent=1, default=float))
print('good:', sum(r['good'] for r in res), '/', len(res))
