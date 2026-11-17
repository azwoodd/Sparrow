"""Stowed-wing fast gait ("QuickStep-A"): same gate and same condition sets as the wing-counterweight walk, plus a stop.

  python r2_fast.py robust | holdout | sweep
Run 9 s of gait, ramp to a stop, stand 3 s. Gate: >=5 valid alternating steps, >=0.25 m forward, >=10 s upright,
and still standing (tilt < 10 deg) at the end.
"""
import os, sys, json
os.environ.setdefault('SPARROW_VARIANT', 'v7e_stiffcontact')
import numpy as np
from lab import Lab, WS, VARIANT
from r1_probe import run_metrics

OUT = WS / 'results' / 'r2_fast' / VARIANT; OUT.mkdir(parents=True, exist_ok=True)
QUICKSTEP = dict(f=3.5, duty=0.6, s=0.03, lift=0.012, back=0.02, stop_t=9.0)      # frozen before robust/holdout were run

DEV = [('nominal', {}), ('friction0.6', {'friction': 0.6}), ('friction0.8', {'friction': 0.8}), ('friction1.3', {'friction': 1.3}),
       ('mass0.92', {'mass': 0.92}), ('mass1.08', {'mass': 1.08}), ('comY+3mm', {'com_shift': 0.003}), ('comY-3mm', {'com_shift': -0.003}),
       ('lean_pitch+2', {'lean': 2.0, 'lean_axis': 'pitch'}), ('lean_pitch-2', {'lean': -2.0, 'lean_axis': 'pitch'}),
       ('lean_roll+2', {'lean': 2.0, 'lean_axis': 'roll'}), ('lean_roll-2', {'lean': -2.0, 'lean_axis': 'roll'})]
HOLD = [('H_friction0.5_mass1.05', {'friction': 0.5, 'mass': 1.05}), ('H_friction0.7_mass0.95_comY+2', {'friction': 0.7, 'mass': 0.95, 'com_shift': 0.002}),
        ('H_friction1.5_mass1.1', {'friction': 1.5, 'mass': 1.1}), ('H_friction0.9_comY-4', {'friction': 0.9, 'com_shift': -0.004}),
        ('H_mass0.88_roll+3', {'mass': 0.88, 'lean': 3.0, 'lean_axis': 'roll'}), ('H_friction0.65_pitch-3', {'friction': 0.65, 'lean': -3.0, 'lean_axis': 'pitch'}),
        ('H_friction0.4', {'friction': 0.4}), ('H_mass1.15_comY+4', {'mass': 1.15, 'com_shift': 0.004})]


def go(lab, name, kw, case, dur=12.0):
    o, ev = run_metrics(lab, kw, dur, case)
    o['name'] = name; o['case'] = case; o['params'] = kw
    o['end_tilt'] = lab.rows[-1]['tilt_deg']
    o['PASS'] = bool(o['fall'] is None and o['T'] >= 10 and o['alt_steps'] >= 5 and o['forward_mm'] >= 250 and o['end_tilt'] < 10 and o['other'] == 0)
    print(f"{name:34} PASS={o['PASS']!s:5} fall={str(o['fall'])[:5]:>5} fwd={o['forward_mm']:7.1f}mm v={o['speed_mm_s']:6.1f}mm/s alt={o['alt_steps']:3d} "
          f"tilt={o['max_tilt']:5.1f} roll={o['max_roll']:4.1f} lat={o['lateral_mm']:+6.1f} yaw={o['yaw_deg']:+5.1f} sat={o['sat']:.2f} single={o['single_frac']:.2f} flight={o['flight_frac']:.3f}", flush=True)
    return o


if __name__ == '__main__':
    mode = sys.argv[1]; lab = Lab(); res = []
    # optional candidate:  python r2_fast.py robust <tag> f=3.5 s=0.04 duty=0.5   (QuickStep-A itself is never changed)
    tag = ''
    if len(sys.argv) > 2:
        tag = '_' + sys.argv[2]; QUICKSTEP = dict(QUICKSTEP, **{a.split('=')[0]: float(a.split('=')[1]) for a in sys.argv[3:]})
    if mode in ('robust', 'holdout'):
        for n, c in (DEV if mode == 'robust' else HOLD):
            res.append(go(lab, f'{mode}_{n}', QUICKSTEP, c))
        print(f"{mode}: {sum(r['PASS'] for r in res)}/{len(res)} pass")
    elif mode == 'sweep':
        for f in (2.0, 2.5, 3.0, 3.5, 4.0, 4.5):
            for s in (0.02, 0.03, 0.04, 0.05):
                res.append(go(lab, f'sweep_f{f}_s{int(s * 1000)}', dict(QUICKSTEP, f=f, s=s), {}))
    (OUT / f'summary_{mode}{tag}.json').write_text(json.dumps(res, indent=1))

