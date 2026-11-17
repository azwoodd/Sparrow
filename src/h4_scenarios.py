"""Hop scenarios required by the owner (full physics, aerodynamics ON, one simulation at a time).

  1 standstill -> hop to a forward target      2 walk -> hop      3 fast gait ("run") -> hop
  4 three hops in a row                         5 hop to the left / right
  python h4_scenarios.py <scenario|all> [dev|holdout]
Every scenario must finish with NO fall anywhere in the episode, every planned hop 'good' (>=15 mm two-foot clearance,
single flight, no body contact, upright 2 s / until the next take-off) and no unplanned flight phases.
"""
import json, os, sys
os.environ.setdefault('SPARROW_VARIANT', 'v7e_stiffcontact')
import numpy as np
from lab import Lab, WS, VARIANT, summarize
from hop import Hopper, analyse_hops
from h2_opt import Library
from h3_validate import interp_params
from script_ctl import Script
from gait import walk_tracks
from run_gait import Runner
from p2_walk import CANDIDATE
from r2_fast import QUICKSTEP

OUT = WS / 'results' / 'h4_scenarios' / VARIANT; OUT.mkdir(parents=True, exist_ok=True)
DEV = [('nominal', {}), ('friction0.6', {'friction': 0.6}), ('friction1.4', {'friction': 1.4}), ('mass0.93', {'mass': 0.93}),
       ('mass1.07', {'mass': 1.07}), ('comY+3', {'com_shift': 0.003}), ('comY-3', {'com_shift': -0.003})]
HOLD = [('H_fr0.5_m1.05', {'friction': 0.5, 'mass': 1.05}), ('H_fr0.8_m0.95_c+2', {'friction': 0.8, 'mass': 0.95, 'com_shift': 0.002}),
        ('H_fr1.6', {'friction': 1.6}), ('H_m1.1_c-2', {'mass': 1.1, 'com_shift': -0.002}), ('H_fr0.7_pitch+1.5', {'friction': 0.7, 'lean': 1.5, 'lean_axis': 'pitch'})]
lib = None


def P(D, prefix='fwd'):
    return interp_params(lib, D, prefix)


def build(name):
    """-> (controller, duration, planned_hops, description)"""
    if name.startswith('standstill_'):
        D = int(name.split('_')[1]); return Hopper([(0.5, P(D))]), 4.5, 1, f'standstill hop to {D} mm'
    if name == 'triple':
        gap = 1.7; return Hopper([(0.5 + k * gap, P(45)) for k in range(3)]), 0.5 + 3 * gap + 2.5, 3, 'three 45 mm hops in a row'
    if name == 'triple_mixed':
        gap = 1.7; return Hopper([(0.5 + k * gap, P(D)) for k, D in enumerate((20, 58, 40))]), 0.5 + 3 * gap + 2.5, 3, 'hops of 20, 58, 40 mm in a row'
    if name == 'walk_hop':
        tr, tend, _ = walk_tracks(**dict(CANDIDATE, n_steps=5, close=True))
        return Hopper([(tend + 0.6, P(50))], pre=Script(tr)), tend + 0.6 + 4.2, 1, 'wing-counterweight walk (4 steps + closing step) then 50 mm hop'
    if name == 'quickstep_hop':
        q = dict(QUICKSTEP, stop_t=4.0); th = 4.0 + q.get('stop_ramp', 0.6) + 0.4 + 0.6
        return Hopper([(th, P(50))], pre=Runner(**q)), th + 4.2, 1, 'stowed-wing fast gait 3.7 s, stop, then 50 mm hop'
    if name in ('hop_left', 'hop_right'):
        p = dict(lib.d['diag40_20']['params'])
        if name == 'hop_right':                       # mirror image: swap the left/right asymmetries
            for k in ('dE', 'sc', 'dzd'):
                p[k] = -p[k]
        return Hopper([(0.5, p)]), 4.5, 1, f'single diagonal hop, forward and to the {name[4:]}'
    if name in ('zigzag',):
        pl = dict(lib.d['diag40_20']['params']); pr = dict(pl)
        for k in ('dE', 'sc', 'dzd'):
            pr[k] = -pr[k]
        gap = 1.9; return Hopper([(0.5, pl), (0.5 + gap, pr), (0.5 + 2 * gap, pl)]), 0.5 + 3 * gap + 2.5, 3, 'three hops: left, right, left'
    if name in ('steer_left_hop', 'steer_right_hop'):
        sgn = 1.0 if 'left' in name else -1.0
        q = dict(QUICKSTEP, stop_t=5.0, turn=0.4 * sgn); th = 5.0 + q.get('stop_ramp', 0.6) + 0.4 + 1.4      # 1.4 s settle (0.6 s fell once at low friction)
        return Hopper([(th, P(50))], pre=Runner(**q)), th + 4.2, 1, f"fast gait curving {name.split('_')[1]} for 4.7 s, stop, then 50 mm hop along the new heading"
    raise KeyError(name)


def run(lab, name, cname, case):
    ctl, dur, n, desc = build(name)
    fall = lab.run(ctl, dur, case=case, stop_after_fall=0.3, every=2)
    hops = analyse_hops(lab.rows, lab.weight)
    s, ev = summarize(lab.rows, lab.weight, fall)
    t_first = ctl.hops[0][0]
    steps = [e for e in ev if e['valid'] and e['t_liftoff'] < t_first]
    pre_rows = [r for r in lab.rows if r['t'] <= t_first]
    pre_fwd = -(pre_rows[-1]['com_y'] - lab.rows[0]['com_y']) * 1000
    planned = [h for h in hops if h['t_takeoff'] >= t_first]
    ok = bool(fall is None and len(planned) == n and all(h['good'] and h['bounces'] == 0 for h in planned)
              and s['other_contact_frac'] == 0 and lab.rows[-1]['tilt_deg'] < 10)
    r = {'scenario': name, 'desc': desc, 'cond': cname, 'case': case, 'PASS': ok, 'fall': fall, 'duration_s': lab.rows[-1]['t'],
         'pre_hop_valid_steps': len(steps), 'pre_hop_forward_mm': pre_fwd, 'pre_hop_speed_mm_s': pre_fwd / max(t_first - 1.0, 1e-6),
         'total_forward_mm': s['forward_com_mm'], 'lateral_mm': s['lateral_com_mm'], 'max_tilt_deg': s['max_tilt_deg'],
         'spring_J_total': ctl.spring_work, 'hops': planned, 'unplanned_flights': len(hops) - len(planned), 'end_tilt_deg': lab.rows[-1]['tilt_deg']}
    if name in ('hop_left', 'hop_right', 'zigzag') and planned:
        want = [1, -1, 1] if name == 'zigzag' else ([1] if name == 'hop_left' else [-1])
        r['lateral_ok'] = bool(len(planned) == len(want) and all(h['left_mm'] * w >= 8.0 for h, w in zip(planned, want)))
        r['PASS'] = ok = bool(ok and r['lateral_ok'])
    if name.startswith('steer') and planned:
        sgn = 1.0 if 'left' in name else -1.0
        r['lateral_ok'] = bool(s['lateral_com_mm'] * sgn >= 30.0); r['PASS'] = ok = bool(ok and r['lateral_ok'])
    hs = ' | '.join(f"{h['forward_mm']:.0f}mm fwd {h['left_mm']:+.0f}mm left/{h['peak_clear_mm']:.0f}clr/{'ok' if h['good'] else 'BAD'}" for h in planned)
    hs += f" || total lateral {s['lateral_com_mm']:+.0f}mm"
    print(f"{name:16} {cname:18} PASS={ok!s:5} fall={str(fall)[:5]:>5} steps={len(steps):2d} preFwd={pre_fwd:6.0f}mm total={s['forward_com_mm']:6.0f}mm tilt={s['max_tilt_deg']:4.1f} hops[{hs}]", flush=True)
    return r


if __name__ == '__main__':
    lib = Library()
    which = sys.argv[1]; mode = sys.argv[2] if len(sys.argv) > 2 else 'dev'
    names = ['standstill_25', 'standstill_55', 'triple', 'triple_mixed', 'walk_hop', 'quickstep_hop'] if which == 'all' else which.split(',')
    conds = {'dev': DEV, 'holdout': HOLD, 'nominal': DEV[:1]}[mode]
    lab = Lab(); res = []
    for nm in names:
        for cname, case in conds:
            res.append(run(lab, nm, cname, case))
        if mode == 'nominal' or True:
            lab_trace = OUT / f'{nm}_{conds[-1][0]}.csv'
        print(f"== {nm}: {sum(r['PASS'] for r in res if r['scenario'] == nm)}/{len(conds)} pass", flush=True)
    tag = which.replace(',', '+')
    (OUT / f'{tag}_{mode}.json').write_text(json.dumps(res, indent=1))


