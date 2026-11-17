"""Series-elastic coupling between every leg servo and its joint (typical assumed servo back-drive), knee coupled,
validated spring geometry. For each coupling stiffness: stored energy x hip-servo trim on 5 conditions.
  python v13_compliance.py calib <bi_E080|bi_E110> [k ...]
  python v13_compliance.py check <entry> <k> <E_scale> <hip_bias> [key=val ...]     9 development + 5 holdout
Gear-train torque with a coupling = the coupling's own torque. Set SPARROW_VARIANT=v7e_bispring."""
import json, os, sys
os.environ.setdefault('SPARROW_VARIANT', 'v7e_bispring')
import numpy as np
import v3_bispring as v3
import v9_backdrive as v9

OUT = v9.WS / 'results' / 'v13_compliance'; OUT.mkdir(parents=True, exist_ok=True)
case = 'typical'


def cell(p, conds):
    rows = []
    for cn, c in conds:
        r = v3.hop(p, c); ctl = v9._last['ctl']; gp, gl = ctl.gear_peak['push'], ctl.gear_peak['land']
        r.update(cond=cn, drag_J=ctl.drag_loss, delivered_J=ctl.delivered, defl=getattr(ctl, 'sea_defl', 0.0),
                 gear_push=[float(max(gp[j], gp[j + 3])) for j in range(3)], gear_land=[float(max(gl[j], gl[j + 3])) for j in range(3)])
        rows.append(r)
    return rows


def line(rows):
    g = lambda k: [r.get(k, 0) for r in rows]
    gp = np.max([r['gear_push'] for r in rows], axis=0); gl = np.max([r['gear_land'] for r in rows], axis=0)
    return (f"clearance {min(g('clear')):.0f}-{max(g('clear')):.0f} mm, forward {min(g('fwd')):.0f}-{max(g('fwd')):.0f} mm, touchdown up to {max(g('landF')):.0f} N, "
            f"gear-train peak push {gp[0]:.2f}/{gp[1]:.2f}/{gp[2]:.2f} landing {gl[0]:.2f}/{gl[1]:.2f}/{gl[2]:.2f} Nm (hip/knee/ankle), gear loss {rows[0]['drag_J']:.2f} J, coupling deflection up to {max(g('defl')):.2f} rad")


mode, name = sys.argv[1], sys.argv[2]
base = dict(v9.lib[name]['params'], **v9.CASES[case]); res = []
if mode == 'calib':
    ks = [float(x) for x in sys.argv[3:]] or [30.0, 12.0, 6.0, 3.0]
    for k in ks:
        print(f'=== {name}, coupling {k} Nm/rad', flush=True)
        for es in (0.8, 1.0, 1.2, 1.5):
            best = None; counts = []
            for hb in (-0.35, -0.21, -0.07, 0.07, 0.21, 0.35):
                rows = cell(dict(base, sea_k=k, E_scale=es, hip_bias=hb), v9.CONDS); n = sum(r['good'] for r in rows); counts.append((hb, n))
                res.append({'entry': name, 'k': k, 'E_scale': es, 'hip_bias': hb, 'good': n, 'rows': rows})
                if best is None or n > best[0]: best = (n, hb, rows)
            print(f"  energy x{es} ({2 * base['E'] * es:.2f} J): good/5 by trim " + ' '.join(f'{hb:+.2f}:{n}' for hb, n in counts) + f"\n      best {best[0]}/5 at {best[1]:+.2f}: " + line(best[2]), flush=True)
    (OUT / f'calib_{name}_{"_".join(str(int(k)) for k in ks)}.json').write_text(json.dumps(res, indent=1, default=float))
elif mode == 'check':
    k, es, hb = [float(x) for x in sys.argv[3:6]]
    extra = {a.split('=')[0]: float(a.split('=')[1]) for a in sys.argv[6:]}
    p = dict(base, sea_k=k, E_scale=es, hip_bias=hb, **extra)
    for label, conds in (('development', v3.DEV), ('holdout', v3.HOLD)):
        rows = cell(p, conds); res += [dict(r, set=label) for r in rows]; bad = [r['cond'] for r in rows if not r['good']]
        print(f"{name} k={k} energy {2 * base['E'] * es:.2f} J trim {hb:+.2f} {extra or ''}: {label} {sum(r['good'] for r in rows)}/{len(rows)} good, " + line(rows) + (f"  failed: {bad}" if bad else ''), flush=True)
    (OUT / f'check_{name}_k{int(k)}_E{int(es * 100)}.json').write_text(json.dumps({'params': p, 'results': res}, indent=1, default=float))
