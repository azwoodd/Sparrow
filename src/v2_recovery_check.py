"""Targeted check on one variant's 'fair_E050' hop: it now survives touchdown but topples ~0.8 s later while returning to
the standing pose. Does holding the absorbed crouch longer, standing up more slowly, or standing with the feet a little
forward fix that? Nominal + 3 conditions per setting. (Set SPARROW_VARIANT.)"""
import json, os, sys
import numpy as np
import h2_opt
from h2_opt import simulate, Library
from lab import WS, VARIANT

h2_opt.AERO = True
lib = Library(); base = lib.d['fair_E050']['params']
CONDS = [('nominal', {}), ('friction1.4', {'friction': 1.4}), ('mass1.07', {'mass': 1.07}), ('comY-3', {'com_shift': -0.003})]
res = []
print(f'=== {VARIANT}: recovery variations of fair_E050')
for t_a in (base['t_a'], 0.8):
    for t_r in (0.6, 1.2, 2.0):
        for dy_s in (0.0, -0.015, -0.030):
            p = dict(base, t_a=t_a, t_r=t_r, dy_s=dy_s); cells = []; clr = []; tm = []
            for cn, case in CONDS:
                fall, hops, rows, ctl = simulate(p, case, settle=2.2 + t_a + t_r)
                ok = bool(fall is None and len(hops) == 1 and hops[0]['good'])
                cells.append('P' if ok else ('b' if fall is None else '.'))
                if hops: clr.append(hops[0]['peak_clear_mm']); tm.append(hops[0]['max_tilt_after_deg'])
                res.append({'t_a': t_a, 't_r': t_r, 'dy_s': dy_s, 'cond': cn, 'ok': ok, 'fall': fall, 'flights': len(hops),
                            'clear': hops[0]['peak_clear_mm'] if hops else None, 'tilt_after': hops[0]['max_tilt_after_deg'] if hops else None})
            print(f"hold {t_a:.2f}s  stand-up {t_r:.1f}s  feet {dy_s * 1000:+4.0f}mm : {''.join(cells)}  clearance {min(clr):.0f}-{max(clr):.0f}mm  max tilt after {min(tm):.0f}-{max(tm):.0f} deg", flush=True)
(WS / 'results' / 'v1_variant_compare' / f'recovery_{VARIANT}.json').write_text(json.dumps(res, indent=1, default=float))
print('P = good hop, b = upright but extra bounce/flight, . = fell')
