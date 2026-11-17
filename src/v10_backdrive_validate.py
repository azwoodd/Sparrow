"""Full check (9 development + 5 holdout conditions) of the hops re-calibrated for realistic servo back-drive.
python v10_backdrive_validate.py <case> <entry> <E_scale> <hip_bias> [...more triples of entry E_scale hip_bias]"""
import json, os, sys
os.environ.setdefault('SPARROW_VARIANT', 'v7e_bispring')
import v3_bispring as v3
import v9_backdrive as v9
from lab import WS

case = sys.argv[1]; a = sys.argv[2:]; res = []
for i in range(0, len(a), 3):
    name, es, hb = a[i], float(a[i + 1]), float(a[i + 2])
    p = dict(v9.lib[name]['params'], **v9.CASES[case], E_scale=es, hip_bias=hb)
    for label, conds in (('development', v3.DEV), ('holdout', v3.HOLD)):
        n = 0; cl = []; lf = []; bad = []
        for cn, c in conds:
            r = v3.hop(p, c); r.update(cond=cn, set=label, entry=name, case=case, E_scale=es, hip_bias=hb); res.append(r)
            n += r['good']; cl.append(r.get('clear', 0)); lf.append(r.get('landF', 0))
            if not r['good']: bad.append(cn)
        print(f"{case} {name} energy x{es} ({2 * p['E'] * es:.2f} J) trim {hb:+.2f}: {label} {n}/{len(conds)} good, clearance {min(cl):.0f}-{max(cl):.0f} mm, landing force up to {max(lf):.0f} N, peak knee spring torque {r['peak_knee_tau']:.2f} Nm" + (f"  failed: {bad}" if bad else ''), flush=True)
(v9.OUT / f'validate_{case}.json').write_text(json.dumps(res, indent=1, default=float))
