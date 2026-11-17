"""Can a one-number calibration absorb build tolerance? The spring hardware is built with the wrong hip/knee pulley ratio
(or stores the wrong energy); everything else stays as validated. Sweep only the hip-servo trim torque (within +-0.35 Nm)
and report the best count over 5 conditions. Set SPARROW_VARIANT=v7e_bispring."""
import json, os, sys
import numpy as np
import v3_bispring as v3
from lab import WS, VARIANT

OUT = WS / 'results' / 'v4_detail'; OUT.mkdir(parents=True, exist_ok=True)
lib = json.loads((WS / 'results' / 'h2_opt' / VARIANT / 'library.json').read_text())
CONDS = [('nominal', {}), ('friction0.6', {'friction': 0.6}), ('friction1.4', {'friction': 1.4}), ('mass1.07', {'mass': 1.07}), ('comY-3', {'com_shift': -0.003})]
FAULTS = [('pulley ratio -0.50', {'rho_hw': -0.50}), ('pulley ratio -0.70', {'rho_hw': -0.70}), ('energy -10%', {'E_scale': 0.90}), ('energy +10%', {'E_scale': 1.10})]
res = []
for name in sys.argv[1:] or ['bi_E080', 'bi_E110']:
    base = lib[name]['params']; hb0 = base.get('hip_bias', 0.0)
    print(f'=== {name}: validated hip trim {hb0:+.2f} Nm', flush=True)
    for label, ch in FAULTS:
        row = []
        for hb in np.arange(-0.35, 0.351, 0.07):
            n = 0; cl = []
            for cn, case in CONDS:
                r = v3.hop(dict(base, hip_bias=float(hb), **ch), case); n += r['good']; cl.append(r.get('clear', 0))
            row.append((float(hb), n, min(cl), max(cl))); res.append({'entry': name, 'fault': label, 'hip_bias': float(hb), 'good': n, 'clear_min': min(cl), 'clear_max': max(cl)})
        best = max(row, key=lambda x: x[1])
        print(f"  {label:20} good/5 by trim " + ' '.join(f'{hb:+.2f}:{n}' for hb, n, _, _ in row) + f"   -> best {best[1]}/5 at {best[0]:+.2f} Nm, clearance {best[2]:.0f}-{best[3]:.0f} mm", flush=True)
(OUT / f'calibration_{VARIANT}.json').write_text(json.dumps(res, indent=1, default=float))
