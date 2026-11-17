"""Like-for-like: knee servo COUPLED (as validated geometry, spring split -0.6) against knee DECOUPLED during the push
(one-way coupling, best split/crouch found per energy) at the same stored spring energy, typical servo back-drive on every
leg servo. For each: hip-servo trim swept, best trim reported on 5 conditions. Set SPARROW_VARIANT=v7e_bispring.
  python v12_compare.py coupled | decoupled | cushion     (cushion = decoupled and the knee stays off the servo through touchdown)
Gear-train torque = motor torque + back-EMF drag + dry friction + rotor inertia reaction, per servo."""
import json, os, sys
os.environ.setdefault('SPARROW_VARIANT', 'v7e_bispring')
import numpy as np
import v3_bispring as v3
import v9_backdrive as v9

OUT = v9.WS / 'results' / 'v12_compare'; OUT.mkdir(parents=True, exist_ok=True)
which = sys.argv[1]; case = 'typical'
b0 = dict(v9.lib['bi_E080']['params'], **v9.CASES[case]); b0.pop('hip_bias', None)
GEO = {'coupled': {1.6: {}, 2.0: {}, 2.4: {}, 3.3: {'dy_c': v9.lib['bi_E110']['params']['dy_c']}},
       'decoupled': {1.6: {'rho': -1.8, 'dy_c': -0.020}, 2.0: {'rho': -1.4, 'dy_c': -0.020}, 2.4: {'rho': -1.2, 'dy_c': -0.030}}}
GEO['cushion'] = GEO['decoupled']
extra = {'coupled': {}, 'decoupled': {'knee_free': 2}, 'cushion': {'knee_free': 3}}[which]
res = []
for Etot, geo in GEO[which].items():
    best = None
    for hb in (-0.35, -0.21, -0.07, 0.07, 0.21, 0.35):
        rows = []
        for cn, c in v9.CONDS:
            r = v3.hop(dict(b0, **geo, **extra, E=Etot / 2, hip_bias=hb), c); ctl = v9._last['ctl']
            gp, gl = ctl.gear_peak['push'], ctl.gear_peak['land']
            r.update(cond=cn, drag_J=ctl.drag_loss, rotor_J=ctl.rotor_ke_takeoff, delivered_J=ctl.delivered,
                     gear_push=[float(max(gp[j], gp[j + 3])) for j in range(3)], gear_land=[float(max(gl[j], gl[j + 3])) for j in range(3)])
            rows.append(r)
        n = sum(r['good'] for r in rows)
        res.append({'mode': which, 'E_total': Etot, 'hip_bias': hb, 'good': n, 'rows': rows})
        if best is None or n > best[0]: best = (n, hb, rows)
    n, hb, rows = best; nom = rows[0]; g = lambda k: [r.get(k, 0) for r in rows]
    gp = np.max([r['gear_push'] for r in rows], axis=0); gl = np.max([r['gear_land'] for r in rows], axis=0)
    print(f"{which:9} {Etot:.1f} J  split {geo.get('rho', -0.6)}  trim {hb:+.2f}: {n}/5 good | clearance {min(g('clear')):.0f}-{max(g('clear')):.0f} mm, forward {min(g('fwd')):.0f}-{max(g('fwd')):.0f} mm, "
          f"take-off spin {nom.get('L', 0):+.1f} mNms | gear loss in push {nom['drag_J']:.2f} J + rotors {nom['rotor_J']:.2f} J of {nom['delivered_J']:.2f} J delivered\n"
          f"          spring peak knee {max(g('peak_knee_tau')):.2f} hip {max(g('peak_hip_tau')):.2f} Nm | gear-train peak in push hip/knee/ankle {gp[0]:.2f}/{gp[1]:.2f}/{gp[2]:.2f} Nm, "
          f"at landing {gl[0]:.2f}/{gl[1]:.2f}/{gl[2]:.2f} Nm | touchdown force up to {max(g('landF')):.0f} N, max tilt after {max(g('tilt_after')):.0f} deg", flush=True)
(OUT / f'{which}.json').write_text(json.dumps(res, indent=1, default=float))
