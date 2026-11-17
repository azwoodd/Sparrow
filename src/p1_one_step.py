"""Phase 1b: can the UNMODIFIED V7E complete one genuine step with its existing actuators?

Strategy W ("wing counterweight"): deploy the wing on the stance side so the COM moves over the stance foot,
lift / advance / replant the other foot with the pitch-only leg, then re-stow the wing.
Scored by lab.detect_steps (strict criteria); no root manipulation, no external forces, no model edits.
"""
import json, sys
import numpy as np
from lab import Lab, WS, summarize, arr
from script_ctl import Script, STOW

OUT = WS / 'results' / 'p1_one_step'
OUT.mkdir(parents=True, exist_ok=True)


def wing_step(swing='R', lift=0.015, adv=0.030, deploy=1.0, t_dep=1.0, t_lift=0.3, t_adv=0.4, t_down=0.3, hold=0.3, restow=True, t0=0.5,
              level=True, toe_up=0.0, back=0.0):
    """back: before lifting, move both ankles `back` metres forward under the body (= body COM moves back toward the
    ankle axis) so ONE 0.2 Nm ankle servo can hold pitch. Standing CoP is ~43 mm ahead of the ankle: 5.1 N x 0.043 m
    = 0.22 Nm, more than a single ankle can supply."""
    stance = 'L' if swing == 'R' else 'R'
    a = t0 + t_dep + hold                 # wing out, settled
    b = a + t_lift; c = b + t_adv; d = c + t_down; e = d + hold
    tr = {f'wing_{stance}_sweep': [(t0, STOW), (t0 + t_dep, STOW * (1 - deploy))] + ([(e, STOW * (1 - deploy)), (e + t_dep, STOW)] if restow else []),
          f'{swing}_dz': [(a, 0.0), (b, lift), (c, lift), (d, 0.0)],
          f'{swing}_dy': [(t0, 0.0), (t0 + t_dep, -back), (b, -back), (c, -back - adv)],
          f'{stance}_dy': [(t0, 0.0), (t0 + t_dep, -back)]}
    if level:
        tr[f'{swing}_swing'] = [(a, 0.0), (a + 0.001, 1.0), (d + 0.1, 1.0), (d + 0.101, 0.0)]
        if toe_up:
            tr[f'{swing}_pitch'] = [(a, 0.0), (b, toe_up), (c, toe_up), (d, 0.0)]
    return tr, e + (t_dep if restow else 0)


def run_case(lab, name, tracks, dur, save=True, case=None):
    fall = lab.run(Script(tracks), dur, case=case, stop_after_fall=0.5)
    summ, ev = summarize(lab.rows, lab.weight, fall)
    summ['name'] = name
    real = [e for e in ev if e['t_liftoff'] > 0.2]
    summ['events'] = real
    if save:
        lab.save_trace(OUT / f'{name}.csv')
    return summ


def line(s):
    ev = s['events']
    best = max(ev, key=lambda e: (e['valid'], e['max_clear_mm'])) if ev else None
    txt = (f"{s['name']:46} fall={str(s['first_fall_s'])[:5]:>5} valid={s['valid_steps']} strict={s['strict_steps']} swings={len(ev)} "
           f"maxtilt={s['max_tilt_deg']:5.1f} sat={s['sat_fraction']:.2f}")
    if best:
        txt += (f" | best: foot={best['foot']} clear={best['max_clear_mm']:.1f}mm adv={best['advance_mm']:+.1f}mm swing={best['swing_s']:.2f}s "
                f"stanceLoad={best['stance_mean_load_frac']:.2f} slip={best['stance_slip_mm']:.1f}mm fail={[k for k, v in best['criteria'].items() if not v]}")
    return txt


if __name__ == '__main__':
    lab = Lab(); res = []
    grid = []
    grid.append(('W0_ctrl_no_wing_R_lift15_adv30', dict(deploy=0.0)))
    grid.append(('W1_legacyankle_deploy100_lift15_adv30', dict(level=False)))
    grid.append(('W1_legacyankle_deploy100_slow', dict(level=False, t_lift=0.6, t_adv=0.8, t_down=0.6)))
    grid.append(('W2_levelfoot_noback_deploy100', dict()))
    for back in (0.010, 0.020, 0.030, 0.040, 0.050):
        grid.append((f'W3_back{int(back * 1000)}_deploy100_lift15_adv30', dict(back=back)))
    for back in (0.020, 0.030, 0.040):
        grid.append((f'W3_back{int(back * 1000)}_deploy100_lift15_adv30_slow', dict(back=back, t_lift=0.6, t_adv=0.8, t_down=0.6)))
    if len(sys.argv) > 1:
        grid = [g for g in grid if any(a in g[0] for a in sys.argv[1:])]
    for name, kw in grid:
        tr, tend = wing_step(**kw)
        s = run_case(lab, name, tr, tend + 2.5)
        res.append(s); print(line(s), flush=True)
    (OUT / 'summary.json').write_text(json.dumps(res, indent=1))
