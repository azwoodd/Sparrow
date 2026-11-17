"""Validate optimised hops with full physics (aerodynamics ON, 2 s settle) - one simulation at a time.

  python h3_validate.py library            every library entry, nominal + development conditions
  python h3_validate.py targets            hop-to-target test at in-between distances (parameters interpolated)
A hop PASSES when: both feet airborne with >= 15 mm clearance, exactly one flight phase, no non-foot ground contact,
tilt < 20 deg and COM > 90 mm for 2 s after touchdown.
"""
import json, os, sys
os.environ.setdefault('SPARROW_VARIANT', 'v7e_stiffcontact')
import numpy as np
import h2_opt
from h2_opt import simulate, Library, FWD
from lab import WS, VARIANT

OUT = WS / 'results' / 'h3_validate' / VARIANT; OUT.mkdir(parents=True, exist_ok=True)
LAST_ROWS = None
CONDS = [('nominal', {}), ('friction0.6', {'friction': 0.6}), ('friction1.4', {'friction': 1.4}), ('mass0.93', {'mass': 0.93}),
         ('mass1.07', {'mass': 1.07}), ('comY+3', {'com_shift': 0.003}), ('comY-3', {'com_shift': -0.003}),
         ('pitch+2', {'lean': 2.0, 'lean_axis': 'pitch'}), ('roll+2', {'lean': 2.0, 'lean_axis': 'roll'})]


def interp_params(lib, D, prefix='fwd'):
    """Hop-to-target lookup: linear interpolation between library entries, indexed by the forward distance each entry
    actually MEASURED in nominal validation (not by the label it was optimised for). Clamps to the reachable range."""
    ent = sorted((v['nominal_fwd_mm'], k) for k, v in lib.d.items() if k.startswith(prefix) and v.get('nominal_fwd_mm') is not None and v.get('use', True))
    ds = [e[0] for e in ent]
    D = min(max(D, ds[0]), ds[-1])
    hi = next(i for i, d in enumerate(ds) if d >= D); lo = max(hi - 1, 0)
    a, b = lib.d[ent[lo][1]]['params'], lib.d[ent[hi][1]]['params']
    w = 0.0 if hi == lo else (D - ds[lo]) / (ds[hi] - ds[lo])
    return {k: (1 - w) * a[k] + w * b[k] for k in a}


def reach(lib, prefix='fwd'):
    ds = [v['nominal_fwd_mm'] for k, v in lib.d.items() if k.startswith(prefix) and v.get('nominal_fwd_mm') is not None and v.get('use', True)]
    return min(ds), max(ds)

def check(p, case, n=1, **kw):
    fall, hops, rows, ctl = simulate(p, case, n=n, **kw)
    ok = fall is None and len(hops) == n and all(h['good'] and h['bounces'] == 0 for h in hops)
    global LAST_ROWS
    LAST_ROWS = rows
    return ok, fall, hops, ctl


def row(name, cname, ok, fall, hops, ctl, tgt=None):
    h = hops[0] if hops else {}
    r = {'name': name, 'cond': cname, 'PASS': bool(ok), 'fall': fall, 'n_flights': len(hops), 'spring_J': ctl.spring_work, 'target_mm': tgt,
         **{k: h.get(k) for k in ('forward_mm', 'left_mm', 'yaw_deg', 'peak_clear_mm', 'flight_s', 'tilt_at_touchdown_deg', 'max_tilt_after_deg')}}
    if h and LAST_ROWS:
        from lab import G
        W = LAST_ROWS[0]['fz_L'] * 0 + 1.0
        after = [x for x in LAST_ROWS if h['t_touchdown'] <= x['t'] <= h['t_touchdown'] + 0.3]
        r['peak_landing_force_N'] = max((x['fz_L'] + x['fz_R'] for x in after), default=None)
        r['peak_spring_torque_Nm'] = ctl.peak_tau; r['spring_stored_J'] = ctl.stored
        r['max_leg_servo_saturated'] = max((x['n_saturated'] for x in after), default=None)
    f = lambda v: '   n/a' if v is None else f'{v:6.1f}'
    print(f"{name:10} {cname:12} PASS={ok!s:5} fwd={f(r['forward_mm'])}mm left={f(r['left_mm'])} clr={f(r['peak_clear_mm'])}mm flight={0 if not h else h['flight_s']:.3f}s "
          f"tiltTD={f(r['tilt_at_touchdown_deg'])} tiltMax={f(r['max_tilt_after_deg'])} stored={ctl.stored:.2f}J peakTau={ctl.peak_tau:.2f}Nm landF={f(r.get('peak_landing_force_N'))}N flights={len(hops)} fall={fall}", flush=True)
    return r


if __name__ == '__main__':
    h2_opt.AERO = True
    mode = sys.argv[1]; lib = Library(); res = []
    if mode == 'library':
        names = sys.argv[2:] or sorted(lib.d, key=lambda k: (k.rstrip('-0123456789'), int(''.join(c for c in k if c.isdigit() or c == '-') or 0)))
        for name in names:
            for cname, case in CONDS:
                ok, fall, hops, ctl = check(lib.d[name]['params'], case)
                res.append(row(name, cname, ok, fall, hops, ctl, lib.d[name]['target'][0]))
            n_ok = sum(r['PASS'] for r in res if r['name'] == name)
            lib.d[name]['validated'] = f'{n_ok}/{len(CONDS)}'
            lib.d[name]['nominal_fwd_mm'] = next(r['forward_mm'] for r in res if r['name'] == name and r['cond'] == 'nominal')
            lib.path.write_text(json.dumps(lib.d, indent=1))
            print(f'== {name}: {n_ok}/{len(CONDS)} conditions pass', flush=True)
    elif mode == 'targets':
        for D in [int(a) for a in sys.argv[2:]] or (10, 20, 30, 40, 50, 60):
            ok, fall, hops, ctl = check(interp_params(lib, D), {})
            r = row(f'tgt{D}', 'nominal', ok, fall, hops, ctl, D)
            r['error_mm'] = None if r['forward_mm'] is None else r['forward_mm'] - D; res.append(r)
    (OUT / (f'{mode}.json' if len(sys.argv) < 3 else f"{mode}_{'+'.join(sys.argv[2:])}.json")).write_text(json.dumps(res, indent=1))
    print(f"{mode}: {sum(r['PASS'] for r in res)}/{len(res)} pass")


