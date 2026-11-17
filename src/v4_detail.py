"""Stress tests of the three validated hip-and-knee spring hops, with NO re-tuning (set SPARROW_VARIANT=v7e_bispring).
One factor at a time on 5 conditions each: spring efficiency, pulley-ratio error, stored-energy error, and a servo
torque-speed line. Also reports what the simulated spring demands (stretch, peak torques) for the mechanism numbers."""
import json, os, sys
import numpy as np
import v3_bispring as v3
from lab import WS, VARIANT

OUT = WS / 'results' / 'v4_detail'; OUT.mkdir(parents=True, exist_ok=True)
lib = json.loads((WS / 'results' / 'h2_opt' / VARIANT / 'library.json').read_text())
CONDS = [('nominal', {}), ('friction0.6', {'friction': 0.6}), ('friction1.4', {'friction': 1.4}), ('mass1.07', {'mass': 1.07}), ('comY-3', {'com_shift': -0.003})]
TESTS = [('as validated', {}),
         ('spring efficiency 0.90', {'eta': 0.90}), ('spring efficiency 0.80', {'eta': 0.80}), ('spring efficiency 0.70', {'eta': 0.70}),
         ('efficiency 0.80, energy raised 1/0.8', {'eta': 0.80, 'E_scale': 1.25}),
         ('pulley ratio built -0.55', {'rho_hw': -0.55}), ('pulley ratio built -0.65', {'rho_hw': -0.65}),
         ('pulley ratio built -0.50', {'rho_hw': -0.50}), ('pulley ratio built -0.70', {'rho_hw': -0.70}),
         ('stored energy -10%', {'E_scale': 0.90}), ('stored energy +10%', {'E_scale': 1.10}),
         ('servo no-load speed 10.5 rad/s', {'w0': 10.5}), ('servo no-load speed 7.5 rad/s', {'w0': 7.5}),
         ('eff 0.85 + servo 10.5 rad/s', {'eta': 0.85, 'w0': 10.5})]
res = []
names = sys.argv[1:] or ['bi_E080', 'bi_E100', 'bi_E110']
for name in names:
    base = lib[name]['params']
    print(f'=== {name}  (E={base["E"]} J per leg, rho={base["rho"]})', flush=True)
    for label, ch in TESTS:
        n = 0; cl = []; tl = []; lf = []; sp = 0.0
        for cn, case in CONDS:
            r = v3.hop(dict(base, **ch), case); r.update(entry=name, test=label, cond=cn); res.append(r)
            n += r['good']; cl.append(r.get('clear', 0)); tl.append(r.get('tilt_after', 99)); lf.append(r.get('landF', 0))
        print(f"  {label:38} {n}/{len(CONDS)} good   clearance {min(cl):5.1f}-{max(cl):5.1f} mm   max tilt after {max(tl):5.1f}   landing force up to {max(lf):3.0f} N", flush=True)
(OUT / f'stress_{VARIANT}.json').write_text(json.dumps(res, indent=1, default=float))
