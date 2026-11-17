"""Phase 1c/2: sustained alternating walking on the UNMODIFIED V7E using the wing-counterweight gait.

  python p2_walk.py nominal            one nominal run, full trace + events
  python p2_walk.py sweep              parameter sweep (step length, timing, back-shift, deploy)
Gate (handoff): >=5 valid alternating loaded replants, >=0.25 m forward COM, >=10 s without falling.
"""
import json, sys
import numpy as np
from lab import Lab, WS, VARIANT, summarize
from script_ctl import Script
from gait import walk_tracks

OUT = WS / 'results' / 'p2_walk' / VARIANT
OUT.mkdir(parents=True, exist_ok=True)


CANDIDATE = dict(n_steps=10, s=0.040, lift=0.020, back=0.030, t_ds=0.7)      # "WingWalk-A", frozen 12 Nov 2026


def gate(s):
    return {'valid_alternating_ge_5': s['valid_alternating_steps'] >= 5, 'forward_ge_250mm': s['forward_com_mm'] >= 250.0,
            'upright_ge_10s': (not s['fell']) and s['duration_s'] >= 10.0}


def run(lab, name, kw, case=None, save=False, extra=2.0):
    tr, tend, plan = walk_tracks(**kw)
    fall = lab.run(Script(tr), tend + extra, case=case, stop_after_fall=0.5)
    s, ev = summarize(lab.rows, lab.weight, fall)
    ev = [e for e in ev if e['t_liftoff'] > 0.2]
    s.update(name=name, params=kw, case=case or {}, planned_steps=len(plan), gate=gate(s))
    s['PASS'] = all(s['gate'].values())
    s['speed_mm_s'] = s['forward_com_mm'] / s['duration_s']
    v = [e for e in ev if e['valid']]
    # lateral safety margin: smallest distance of the COM ground projection inside the stance pad during any real swing
    t = np.array([r['t'] for r in lab.rows]); mm = []
    for e in ev:
        if e['swing_s'] > 0.05 and e['t_touchdown']:
            st = 'L' if e['foot'] == 'R' else 'R'
            k0, k1 = np.searchsorted(t, e['t_liftoff']), np.searchsorted(t, e['t_touchdown'])
            mm.append(min(r['com_margin_' + st] for r in lab.rows[k0:k1]) * 1000)
    s['min_ss_com_margin_mm'] = float(min(mm)) if mm else None
    if v:
        s['mean_clear_mm'] = float(np.mean([e['max_clear_mm'] for e in v])); s['mean_advance_mm'] = float(np.mean([e['advance_mm'] for e in v]))
        s['max_stance_slip_mm'] = float(max(e['stance_slip_mm'] for e in v)); s['min_stance_load'] = float(min(e['stance_mean_load_frac'] for e in v))
    if save:
        lab.save_trace(OUT / f'{name}.csv')
        (OUT / f'{name}_events.json').write_text(json.dumps({'summary': s, 'plan': plan, 'events': ev}, indent=1))
    return s, ev


def line(s):
    return (f"{s['name']:40} PASS={s['PASS']!s:5} fall={str(s['first_fall_s'])[:5]:>5} T={s['duration_s']:5.1f}s fwd={s['forward_com_mm']:6.1f}mm "
            f"valid={s['valid_steps']:2d}/{s['planned_steps']:2d} alt={s['valid_alternating_steps']:2d} strict={s['strict_steps']:2d} "
            f"v={s['speed_mm_s']:5.1f}mm/s tilt={s['max_tilt_deg']:4.1f} sat={s['sat_fraction']:.2f} "
            f"clr={s.get('mean_clear_mm', 0):4.1f} adv={s.get('mean_advance_mm', 0):5.1f} slip={s.get('max_stance_slip_mm', 0):4.1f} lat={s['lateral_com_mm']:+.0f}mm margin={s.get('min_ss_com_margin_mm') if s.get('min_ss_com_margin_mm') is None else round(s['min_ss_com_margin_mm'], 1)}")


if __name__ == '__main__':
    mode = sys.argv[1] if len(sys.argv) > 1 else 'nominal'
    lab = Lab(); res = []
    if mode == 'nominal':
        s, ev = run(lab, 'walk_nominal_10steps', dict(n_steps=10), save=True)
        print(line(s))
        for e in ev:
            print(f"  {e['foot']} lift={e['t_liftoff']:6.2f} swing={e['swing_s']:.2f}s clear={e['max_clear_mm']:5.1f} adv={e['advance_mm']:+6.1f} "
                  f"stance={e['stance_mean_load_frac']:.2f} slip={e['stance_slip_mm']:4.1f} valid={e['valid']} fail={[k for k, v in e['criteria'].items() if not v]}")
        res.append(s)
    elif mode == 'sweep':
        grid = []
        for sl in (0.02, 0.03, 0.04, 0.05):
            grid.append((f's{int(sl * 1000)}', dict(n_steps=12, s=sl)))
        for back in (0.02, 0.025, 0.035, 0.04):
            grid.append((f'back{int(back * 1000)}', dict(n_steps=12, back=back)))
        for dep in (0.5, 0.75):
            grid.append((f'deploy{int(dep * 100)}', dict(n_steps=12, deploy=dep)))
        for tds in (0.5, 0.7):
            grid.append((f'tds{tds}', dict(n_steps=12, t_ds=tds)))
        for f in (0.75, 0.5):
            grid.append((f'swingx{f}', dict(n_steps=12, t_lift=0.3 * f, t_adv=0.4 * f, t_down=0.3 * f)))
        grid.append(('fast_combo', dict(n_steps=14, s=0.04, t_ds=0.6, t_lift=0.2, t_adv=0.3, t_down=0.2, hold=0.15)))
        grid.append(('lift25', dict(n_steps=12, lift=0.025)))
        for name, kw in grid:
            s, _ = run(lab, 'sweep_' + name, kw)
            res.append(s); print(line(s), flush=True)
    elif mode == 'margin':
        # what makes single support laterally safe? (un-weighting time, neck assist) x (friction)
        for nm, extra in (('A_candidate', {}), ('unw0.3', dict(t_unw=0.3)), ('unw0.6', dict(t_unw=0.6)), ('neck1.0', dict(neck=1.0)),
                          ('unw0.4_neck1.0', dict(t_unw=0.4, neck=1.0)), ('unw0.6_neck1.0', dict(t_unw=0.6, neck=1.0)),
                          ('unw0.6_unw5mm_neck1.0', dict(t_unw=0.6, unw=0.005, neck=1.0))):
            for mu in (0.6, 1.0, 1.3, 1.6, 2.0):
                s, _ = run(lab, f'margin_{nm}_mu{mu}', dict(CANDIDATE, n_steps=6, **extra), case={'friction': mu})
                res.append(s); print(line(s), flush=True)
    elif mode in ('robust', 'holdout'):
        # CANDIDATE gait frozen before either set was run. 'robust' = development conditions, 'holdout' = untouched set.
        if mode == 'robust':
            conds = [('nominal', {}), ('friction0.6', {'friction': 0.6}), ('friction0.8', {'friction': 0.8}), ('friction1.3', {'friction': 1.3}),
                     ('mass0.92', {'mass': 0.92}), ('mass1.08', {'mass': 1.08}), ('comY+3mm', {'com_shift': 0.003}), ('comY-3mm', {'com_shift': -0.003}),
                     ('lean_pitch+2', {'lean': 2.0, 'lean_axis': 'pitch'}), ('lean_pitch-2', {'lean': -2.0, 'lean_axis': 'pitch'}),
                     ('lean_roll+2', {'lean': 2.0, 'lean_axis': 'roll'}), ('lean_roll-2', {'lean': -2.0, 'lean_axis': 'roll'})]
        else:
            conds = [('H_friction0.5_mass1.05', {'friction': 0.5, 'mass': 1.05}), ('H_friction0.7_mass0.95_comY+2', {'friction': 0.7, 'mass': 0.95, 'com_shift': 0.002}),
                     ('H_friction1.5_mass1.1', {'friction': 1.5, 'mass': 1.1}), ('H_friction0.9_comY-4', {'friction': 0.9, 'com_shift': -0.004}),
                     ('H_mass0.88_roll+3', {'mass': 0.88, 'lean': 3.0, 'lean_axis': 'roll'}), ('H_friction0.65_pitch-3', {'friction': 0.65, 'lean': -3.0, 'lean_axis': 'pitch'}),
                     ('H_friction0.4', {'friction': 0.4}), ('H_mass1.15_comY+4', {'mass': 1.15, 'com_shift': 0.004})]
        for first in ('R', 'L'):
            for cname, case in conds:
                s, _ = run(lab, f'{mode}_{cname}_first{first}', dict(CANDIDATE, first=first), case=case)
                res.append(s); print(line(s), flush=True)
        print(f"{mode}: {sum(r['PASS'] for r in res)}/{len(res)} pass the walking gate")
    (OUT / f'summary_{mode}.json').write_text(json.dumps(res, indent=1))
