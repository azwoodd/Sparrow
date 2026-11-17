"""The hop outcome depends strongly on leg-joint drag (a placeholder 0.02 Nm s/rad in the model; real back-driven servo drag
is unknown). If the real drag is half or double, can the same hardware be re-calibrated with two numbers - stored energy
and hip-servo trim - to give the 60 mm hop again? 5 conditions per cell. Set SPARROW_VARIANT=v7e_bispring."""
import json, os
os.environ.setdefault('SPARROW_VARIANT', 'v7e_bispring')
import numpy as np
import v3_bispring as v3
from lab import WS

lib = json.loads((WS / 'results' / 'h2_opt' / 'v7e_bispring' / 'library.json').read_text()); base = lib['bi_E080']['params']
CONDS = [('nominal', {}), ('friction0.6', {'friction': 0.6}), ('friction1.4', {'friction': 1.4}), ('mass1.07', {'mass': 1.07}), ('comY-3', {'com_shift': -0.003})]
res = []
for drag, scales in ((0.5, (0.60, 0.70, 0.80)), (2.0, (1.30, 1.45, 1.60)), (3.0, (1.6, 1.8, 2.0))):
    print(f'=== leg-joint drag x{drag} (60 mm hop settings)', flush=True)
    for es in scales:
        row = []
        for hb in (-0.35, -0.21, -0.07, 0.07, 0.21, 0.35):
            n = 0; cl = []
            for cn, case in CONDS:
                r = v3.hop(dict(base, damp_scale=drag, E_scale=es, hip_bias=hb), case); n += r['good']; cl.append(r.get('clear', 0))
            row.append((hb, n, min(cl), max(cl))); res.append({'drag': drag, 'E_scale': es, 'hip_bias': hb, 'good': n, 'clear_min': min(cl), 'clear_max': max(cl)})
        b = max(row, key=lambda x: x[1])
        print(f"  energy x{es:.2f} ({2 * base['E'] * es:.2f} J): good/5 by hip trim " + ' '.join(f'{hb:+.2f}:{n}' for hb, n, _, _ in row) + f"   best {b[1]}/5 at {b[0]:+.2f} Nm, clearance {b[2]:.0f}-{b[3]:.0f} mm", flush=True)
(WS / 'results' / 'v4_detail' / 'drag_calibration_v7e_bispring.json').write_text(json.dumps(res, indent=1, default=float))
