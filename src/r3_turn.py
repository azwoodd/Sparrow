"""Can the stowed-wing quick-step gait steer? Left/right stride asymmetry sweep (one simulation at a time)."""
import os, sys, json, math
os.environ.setdefault('SPARROW_VARIANT', 'v7e_stiffcontact')
from lab import Lab, WS, VARIANT
from r1_probe import run_metrics
from r2_fast import QUICKSTEP

OUT = WS / 'results' / 'r3_turn' / VARIANT; OUT.mkdir(parents=True, exist_ok=True)
lab = Lab(); res = []
for turn in (0.0, 0.2, 0.4, 0.6, 0.8, 1.0, -0.4, -0.8):
    kw = dict(QUICKSTEP, turn=turn, stop_t=6.0)
    o, ev = run_metrics(lab, kw, 8.5)
    r = lab.rows[-1]; r0 = lab.rows[0]
    o.update(turn=turn, end_tilt=r['tilt_deg'], dx_left_mm=(r['com_x'] - r0['com_x']) * 1000, dy_fwd_mm=-(r['com_y'] - r0['com_y']) * 1000)
    o['heading_change_deg'] = o['yaw_deg']
    o['PASS'] = bool(o['fall'] is None and o['end_tilt'] < 10 and o['other'] == 0)
    res.append(o)
    print(f"turn={turn:+.1f} PASS={o['PASS']!s:5} fall={str(o['fall'])[:5]:>5} heading={o['yaw_deg']:+7.1f}deg fwd={o['dy_fwd_mm']:7.0f}mm left={o['dx_left_mm']:+7.0f}mm "
          f"alt_steps={o['alt_steps']:3d} tilt={o['max_tilt']:5.1f} sat={o['sat']:.2f}", flush=True)
(OUT / 'turn_sweep.json').write_text(json.dumps(res, indent=1))
