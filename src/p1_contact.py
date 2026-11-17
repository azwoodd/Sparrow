"""Phase 1d: ground-contact fidelity check for the current SPARROW_VARIANT.

Quiet stand, then full left-wing deploy + 30 mm back shift, then a slow right-foot lift held for 3 s (single support).
Reports pad penetration, static roll, COM margin in the stance pad and whether single support holds, per friction level.
"""
import json, sys
import numpy as np
from lab import Lab, WS, VARIANT, arr, summarize
from script_ctl import Script, STOW

OUT = WS / 'results' / 'p1_contact'
OUT.mkdir(parents=True, exist_ok=True)
tr = {'wing_L_sweep': [(0.5, STOW), (1.5, 0.0)], 'L_dy': [(0.5, 0.0), (1.5, -0.03)], 'R_dy': [(0.5, 0.0), (1.5, -0.03)],
      'R_dz': [(2.0, 0.0), (2.6, 0.015)], 'R_swing': [(2.0, 0.0), (2.001, 1.0)]}
lab = Lab(); res = []
for mu in (0.6, 1.0, 1.3, 1.6, 2.0):
    fall = lab.run(Script(tr), 6.0, case={'friction': mu}, stop_after_fall=0.3)
    rows = lab.rows
    win = lambda a, b: [r for r in rows if a <= r['t'] < b]
    q, ds, ss = win(0.3, 0.5), win(1.8, 2.0), win(4.5, 6.0)
    m = lambda rr, k: float(np.mean(arr(rr, k))) if rr else None
    rec = {'variant': VARIANT, 'friction': mu, 'fell': fall,
           'quiet_pad_penetration_mm': -m(q, 'clear_min_L') * 1000,
           'ds_wing_out_roll_deg': m(ds, 'roll_deg'), 'ds_com_margin_L_mm': m(ds, 'com_margin_L') * 1000,
           'ss_roll_deg': m(ss, 'roll_deg'), 'ss_com_margin_L_mm': (m(ss, 'com_margin_L') or 0) * 1000 if ss else None,
           'ss_pad_penetration_mm': -m(ss, 'clear_min_L') * 1000 if ss else None, 'ss_load_L': m(ss, 'fz_L') / lab.weight if ss else None,
           'ss_min_margin_mm': float(arr(win(2.0, 6.0), 'com_margin_L').min() * 1000)}
    res.append(rec)
    f = lambda v: 'None' if v is None else f'{v:6.2f}'
    print(f"{VARIANT:26} mu={mu:.1f} fall={str(fall)[:5]:>5} quiet_sink={f(rec['quiet_pad_penetration_mm'])}mm DSroll={f(rec['ds_wing_out_roll_deg'])} "
          f"DSmargin={f(rec['ds_com_margin_L_mm'])} | SSroll={f(rec['ss_roll_deg'])} SSmargin={f(rec['ss_com_margin_L_mm'])}mm SSsink={f(rec['ss_pad_penetration_mm'])}mm minMargin={f(rec['ss_min_margin_mm'])}", flush=True)
(OUT / f'{VARIANT}.json').write_text(json.dumps(res, indent=1))
