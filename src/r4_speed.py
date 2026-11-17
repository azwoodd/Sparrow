"""Faster stowed-wing walking: how fast can the quick-step go while staying upright AND within realistic servo speed?

QuickStep-A (preserved) is the reference. Each candidate runs 9 s of gait, stops, stands 3 s (same gate as r2_fast).
Reported per run: speed, peak joint speed actually reached (from the joint trace), torque saturation, aerial fraction.
Servo speed reference: a 0.10 s/60 deg hobby servo = 10.5 rad/s, 0.14 s/60 deg = 7.5 rad/s (no-load figures).
  python r4_speed.py grid            coarse grid at command slew 10 rad/s
  python r4_speed.py slew            best candidates re-run at command slew 6 / 8 / 10 / 14 rad/s
"""
import os, sys, json
os.environ.setdefault('SPARROW_VARIANT', 'v7e_stiffcontact')
import numpy as np
from lab import Lab, WS, VARIANT, arr
from r1_probe import run_metrics
from r2_fast import QUICKSTEP

OUT = WS / 'results' / 'r4_speed' / VARIANT; OUT.mkdir(parents=True, exist_ok=True)


def go(lab, name, kw, case=None):
    o, ev = run_metrics(lab, kw, 12.0, case)
    rows = [r for r in lab.rows if 1.5 < r['t'] < kw.get('stop_t', 9.0)]
    pk = 0.0
    if len(rows) > 10:
        t = arr(rows, 't')
        for j in lab.leg_names:
            q = arr(rows, j + '_q'); v = np.abs(np.gradient(q, t))
            pk = max(pk, float(np.percentile(v, 99.5)))
    o.update(name=name, params=kw, peak_joint_speed_rad_s=pk, end_tilt=lab.rows[-1]['tilt_deg'])
    o['PASS'] = bool(o['fall'] is None and o['T'] >= 10 and o['alt_steps'] >= 5 and o['forward_mm'] >= 250 and o['end_tilt'] < 10 and o['other'] == 0)
    print(f"{name:26} PASS={o['PASS']!s:5} fall={str(o['fall'])[:5]:>5} v={o['speed_mm_s']:6.1f}mm/s tilt={o['max_tilt']:5.1f} roll={o['max_roll']:4.1f} "
          f"jointSpeed={pk:5.1f}rad/s sat={o['sat']:.2f} aerial={o['flight_frac']:.3f} single={o['single_frac']:.2f} lat={o['lateral_mm']:+5.0f} yaw={o['yaw_deg']:+5.1f}", flush=True)
    return o


if __name__ == '__main__':
    mode = sys.argv[1]; lab = Lab(); res = []
    if mode == 'grid':
        for f in (3.5, 4.0, 4.5, 5.0, 6.0):
            for s in (0.04, 0.05, 0.06, 0.07):
                for duty in (0.6, 0.5):
                    res.append(go(lab, f'f{f}_s{int(s * 1000)}_d{duty}', dict(QUICKSTEP, f=f, s=s, duty=duty)))
    elif mode == 'slew':
        picks = json.loads((OUT / 'summary_grid.json').read_text())
        best = sorted([r for r in picks if r['PASS']], key=lambda r: -r['speed_mm_s'])[:4]
        for b in best + [{'params': dict(QUICKSTEP), 'name': 'QuickStep-A'}]:
            for sl in (6.0, 8.0, 10.0, 14.0):
                res.append(go(lab, f"{b['name']}_slew{sl:g}", dict(b['params'], slew=sl)))
    (OUT / f'summary_{mode}.json').write_text(json.dumps(res, indent=1, default=float))
    print(f"{mode}: {sum(r['PASS'] for r in res)}/{len(res)} pass; fastest passing {max([r['speed_mm_s'] for r in res if r['PASS']], default=0):.0f} mm/s")
