"""Decoupled knee (one-way coupling, knee_free=2) with realistic servo back-drive on the hips and ankles: re-find the
spring's hip/knee split (rho) and the crouch foot position (dy_c). Set SPARROW_VARIANT=v7e_bispring.
  python v11_kneefree.py map <case> <E per leg>        rho x dy_c on the nominal condition (prints take-off spin etc.)
  python v11_kneefree.py grid <case> <E> <rho> <dy_c>  hip trim x landing pose on 5 conditions around one point
  python v11_kneefree.py check <case> <E> <rho> <dy_c> <hip_bias> <dy_l> <dz_l> [knee_free]   9 development + 5 holdout
Positive pitch / positive L = nose down."""
import json, os, sys
os.environ.setdefault('SPARROW_VARIANT', 'v7e_bispring')
import v3_bispring as v3
import v9_backdrive as v9

OUT = v9.WS / 'results' / 'v11_kneefree'; OUT.mkdir(parents=True, exist_ok=True)
mode, case = sys.argv[1], sys.argv[2]; a = [float(x) for x in sys.argv[3:]]
base = dict(v9.lib['bi_E080']['params'], **v9.CASES[case], knee_free=2); base.pop('hip_bias', None)
res = []
if mode == 'map':
    E = a[0]
    for rho in (0.0, -0.3, -0.6, -0.9, -1.2):
        for dyc in (-0.050, -0.040, -0.030, -0.020, -0.010, 0.0):
            r = v3.hop(dict(base, E=E, rho=rho, dy_c=dyc)); res.append(r); v3.show(f'E={E:.2f} rho={rho:+.1f} dy_c={dyc * 1000:+4.0f}mm', r)
    (OUT / f'map_{case}_E{int(E * 100)}.json').write_text(json.dumps(res, indent=1, default=float))
elif mode == 'grid':
    E, rho, dyc = a[:3]
    for dyl, dzl in ((-0.0234, 0.0105), (-0.010, 0.0105), (-0.035, 0.0105), (-0.0234, 0.020)):
        row = []
        for hb in (-0.35, -0.21, -0.07, 0.07, 0.21, 0.35):
            n = 0; cl = []
            for cn, c in v9.CONDS:
                r = v3.hop(dict(base, E=E, rho=rho, dy_c=dyc, hip_bias=hb, dy_l=dyl, dz_l=dzl), c); n += r['good']; cl.append(r.get('clear', 0))
            row.append((hb, n, min(cl), max(cl))); res.append({'E': E, 'rho': rho, 'dy_c': dyc, 'dy_l': dyl, 'dz_l': dzl, 'hip_bias': hb, 'good': n, 'clear_min': min(cl), 'clear_max': max(cl)})
        b = max(row, key=lambda x: x[1])
        print(f"E={E} rho={rho} dy_c={dyc * 1000:.0f}mm landing dy_l={dyl * 1000:.0f} dz_l={dzl * 1000:.0f}mm: good/5 by trim " + ' '.join(f'{hb:+.2f}:{n}' for hb, n, _, _ in row) + f"  best {b[1]}/5 at {b[0]:+.2f}, clearance {b[2]:.0f}-{b[3]:.0f} mm", flush=True)
    (OUT / f'grid_{case}_E{int(E * 100)}_rho{int(-rho * 10)}_dy{int(-dyc * 1000)}.json').write_text(json.dumps(res, indent=1, default=float))
elif mode == 'scan':
    # python v11_kneefree.py scan <case> <dy_l>   : rho x dy_c x E on 5 conditions, for one landing foot position
    dyl = a[0]
    for rho in (-1.4, -1.6, -1.8):
        for dyc in (-0.020, -0.010, 0.0):
            for E in (0.8, 1.0, 1.2):
                n = 0; cl = []; fw = []; lf = []; L = []
                for cn, c in v9.CONDS:
                    r = v3.hop(dict(base, E=E, rho=rho, dy_c=dyc, dy_l=dyl), c); n += r['good']; cl.append(r.get('clear', 0)); fw.append(r.get('fwd', 0)); lf.append(r.get('landF', 0)); L.append(r.get('L', 0))
                res.append({'E': E, 'rho': rho, 'dy_c': dyc, 'dy_l': dyl, 'good': n, 'clear_min': min(cl), 'clear_max': max(cl), 'fwd_min': min(fw), 'fwd_max': max(fw), 'landF_max': max(lf), 'L_nom': L[0]})
                print(f"rho={rho} dy_c={dyc * 1000:+4.0f}mm E={2 * E:.1f}J dy_l={dyl * 1000:.0f}mm: {n}/5 good, clearance {min(cl):.0f}-{max(cl):.0f} mm, forward {min(fw):.0f}-{max(fw):.0f} mm, landing force up to {max(lf):.0f} N, L {L[0]:+.1f}", flush=True)
    (OUT / f'scan_{case}_dyl{int(-dyl * 1000)}.json').write_text(json.dumps(res, indent=1, default=float))
elif mode == 'trim':
    # python v11_kneefree.py trim <case> <E> <rho> <dy_c> [<E> <rho> <dy_c> ...]   hip-servo trim sweep on 5 conditions
    for i in range(0, len(a), 3):
        E, rho, dyc = a[i:i + 3]; row = []
        for hb in (-0.35, -0.21, -0.07, 0.07, 0.21, 0.35):
            n = 0; cl = []; fw = []; lf = []
            for cn, c in v9.CONDS:
                r = v3.hop(dict(base, E=E, rho=rho, dy_c=dyc, hip_bias=hb), c); n += r['good']; cl.append(r.get('clear', 0)); fw.append(r.get('fwd', 0)); lf.append(r.get('landF', 0))
            row.append((hb, n, min(cl), max(cl), max(fw), max(lf))); res.append({'E': E, 'rho': rho, 'dy_c': dyc, 'hip_bias': hb, 'good': n, 'clear_min': min(cl), 'clear_max': max(cl), 'fwd_max': max(fw), 'landF_max': max(lf)})
        b = max(row, key=lambda x: x[1])
        print(f"E={2 * E:.1f}J rho={rho} dy_c={dyc * 1000:.0f}mm: good/5 by trim " + ' '.join(f'{x[0]:+.2f}:{x[1]}' for x in row) + f"  best {b[1]}/5 at {b[0]:+.2f}: clearance {b[2]:.0f}-{b[3]:.0f} mm, forward up to {b[4]:.0f} mm, landing force up to {b[5]:.0f} N", flush=True)
    (OUT / f'trim_{case}_{int(a[0] * 100)}_{int(-a[1] * 10)}_{int(-a[2] * 1000)}.json').write_text(json.dumps(res, indent=1, default=float))
elif mode == 'check':
    E, rho, dyc, hb, dyl, dzl = a[:6]
    p = dict(base, E=E, rho=rho, dy_c=dyc, hip_bias=hb, dy_l=dyl, dz_l=dzl)
    for label, conds in (('development', v3.DEV), ('holdout', v3.HOLD)):
        n = 0; cl = []; lf = []; bad = []
        for cn, c in conds:
            r = v3.hop(p, c); r.update(cond=cn, set=label); res.append(r); n += r['good']; cl.append(r.get('clear', 0)); lf.append(r.get('landF', 0))
            if not r['good']: bad.append(cn)
        print(f"{case} knee decoupled E={2 * E:.2f} J rho={rho} dy_c={dyc * 1000:.0f}mm trim {hb:+.2f}: {label} {n}/{len(conds)} good, clearance {min(cl):.0f}-{max(cl):.0f} mm, landing force up to {max(lf):.0f} N, peak spring torque knee {r['peak_knee_tau']:.2f} hip {r['peak_hip_tau']:.2f} Nm" + (f"  failed: {bad}" if bad else ''), flush=True)
    (OUT / f'check_{case}_E{int(E * 100)}.json').write_text(json.dumps({'params': p, 'results': res}, indent=1, default=float))
