"""60 mm milestone, targeted: preserved push style (hips and ankles chase the extension pose, which already cancels most of
the spring's nose-down spin at 0.40 J per knee) + small sweep of the crouch fore-aft position to steer the arrival tilt
into the landing window found in t1 (level to ~10 deg nose-up), with the deeper / feet-forward landing pose.
  python t6_milestone60.py sweep [E]         2-D sweep, nominal only
  python t6_milestone60.py check <name> E dy_c dz_l dy_l     validate one setting on development + holdout conditions"""
import json, os, sys
os.environ.setdefault('SPARROW_VARIANT', 'v7e_stiffcontact')
import numpy as np
from lab import WS, VARIANT
import t3_hip_bias as t3

OUT = WS / 'results' / 't6_milestones' / VARIANT; OUT.mkdir(parents=True, exist_ok=True)
B = dict(t3.base, align=0, hip_follow=0, ank_follow=0)          # = preserved Hopper behaviour, via the same code path
DEV = [('nominal', {}), ('friction0.6', {'friction': 0.6}), ('friction1.4', {'friction': 1.4}), ('mass0.93', {'mass': 0.93}), ('mass1.07', {'mass': 1.07}),
       ('comY+3', {'com_shift': 0.003}), ('comY-3', {'com_shift': -0.003}), ('pitch+2', {'lean': 2.0, 'lean_axis': 'pitch'}), ('roll+2', {'lean': 2.0, 'lean_axis': 'roll'})]
HOLD = [('H_fr0.5_m1.05', {'friction': 0.5, 'mass': 1.05}), ('H_fr0.8_m0.95_c+2', {'friction': 0.8, 'mass': 0.95, 'com_shift': 0.002}), ('H_fr1.6', {'friction': 1.6}),
        ('H_m1.1_c-2', {'mass': 1.1, 'com_shift': -0.002}), ('H_fr0.7_pitch+1.5', {'friction': 0.7, 'lean': 1.5, 'lean_axis': 'pitch'})]


def show(tag, r):
    if 'clear' in r:
        print(f"{tag:34} L={r['L']:+6.2f}mNms vfwd={r['vfwd']:+.2f} clr={r['clear']:5.1f} fwd={r['fwd']:6.1f} pitchTD={r['pitch_TD']:+6.1f} landF={r['landF']:5.1f}N "
              f"tiltAfter={r['tilt_after']:5.1f} flights={r['flights']} good={r['good']} fall={r['fall']}", flush=True)
    else:
        print(f"{tag:34} no flight fall={r['fall']}", flush=True)


if __name__ == '__main__':
    mode = sys.argv[1]; res = []
    if mode == 'sweep':
        E = float(sys.argv[2]) if len(sys.argv) > 2 else 0.40
        for dz_l, dy_l in ((0.030, B['dy_l'] - 0.015), (0.0077, B['dy_l'])):
            for dyc in (-0.045, -0.038, -0.031, -0.025, -0.019, -0.012, -0.005):
                p = dict(B, E=E, dy_c=dyc, dz_l=dz_l, dy_l=dy_l); r = t3.run(p); res.append(r)
                show(f"E={E:.2f} dy_c={dyc * 1000:+4.0f}mm land dz={dz_l * 1000:.0f} dy={dy_l * 1000:+.0f}", r)
        (OUT / f'sweep_E{int(E * 100)}.json').write_text(json.dumps(res, indent=1, default=float))
    elif mode == 'check':
        name, E, dyc, dz_l, dy_l = sys.argv[2], float(sys.argv[3]), float(sys.argv[4]), float(sys.argv[5]), float(sys.argv[6])
        p = dict(B, E=E, dy_c=dyc, dz_l=dz_l, dy_l=dy_l)
        for label, conds in (('development', DEV), ('holdout', HOLD)):
            n = 0
            for cn, case in conds:
                r = t3.run(p, case); r.update(cond=cn, set=label); res.append(r); n += r['good']; show(f'{label[:4]} {cn}', r)
            print(f'== {name} {label}: {n}/{len(conds)} good', flush=True)
        (OUT / f'check_{name}.json').write_text(json.dumps({'params': p, 'results': res}, indent=1, default=float))
