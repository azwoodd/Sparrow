"""Why do higher hops fail? Take the robust 47 mm hop and raise only the spring energy; log what changes at take-off,
in flight and at landing (full physics, aero on, one simulation at a time)."""
import json, os, sys
os.environ.setdefault('SPARROW_VARIANT', 'v7e_stiffcontact')
import numpy as np
import h2_opt
from h2_opt import simulate, Library
from lab import WS, VARIANT, arr

h2_opt.AERO = True
OUT = WS / 'results' / 'h5_diag' / VARIANT; OUT.mkdir(parents=True, exist_ok=True)
lib = Library(); base = lib.d[sys.argv[1] if len(sys.argv) > 1 else 'high40_60_75']['params']
res = []
for E in (0.20, 0.25, 0.30, 0.35, 0.40, 0.50, 0.60):
    for dzc in (None, 0.045):
        p = dict(base, E=E)
        if dzc: p['dz_c'] = dzc
        fall, hops, rows, ctl = simulate(p, {})
        h = hops[0] if hops else None
        r = {'E_per_knee': E, 'dz_c': p['dz_c'], 'fall': fall, 'flights': len(hops), 'stored_J': ctl.stored, 'peak_tau': ctl.peak_tau}
        if h:
            t = arr(rows, 't'); k0 = int(np.searchsorted(t, h['t_takeoff'])); k1 = int(np.searchsorted(t, h['t_touchdown']))
            pit = arr(rows, 'pitch_deg'); vz = arr(rows, 'com_vz'); vy = arr(rows, 'com_vy')
            dt = t[1] - t[0]
            r.update(clear=h['peak_clear_mm'], fwd=h['forward_mm'], flight_s=h['flight_s'], good=h['good'], bounces=h['bounces'],
                     pitch_takeoff=pit[k0], pitch_touchdown=pit[min(k1, len(pit) - 1)],
                     pitch_rate_flight_deg_s=(pit[min(k1, len(pit) - 1)] - pit[k0]) / max(h['flight_s'], 1e-6),
                     vz_takeoff=vz[k0], vfwd_takeoff=-vy[k0], max_tilt_after=h['max_tilt_after_deg'],
                     land_force=max((x['fz_L'] + x['fz_R'] for x in rows[k1:k1 + 150]), default=0.0),
                     sat_after=max((x['n_saturated'] for x in rows[k1:k1 + 150]), default=0),
                     knee_max_after=max((x['leg_L_shin_q'] for x in rows[k1:k1 + 200]), default=0.0))
            print(f"E={E:.2f} dz_c={p['dz_c']:.3f} clr={r['clear']:5.1f} fwd={r['fwd']:6.1f} fl={r['flight_s']:.3f}s vz0={r['vz_takeoff']:.2f} vf0={r['vfwd_takeoff']:.2f} "
                  f"pitchTO={r['pitch_takeoff']:+5.1f} pitchTD={r['pitch_touchdown']:+6.1f} rate={r['pitch_rate_flight_deg_s']:+6.0f}deg/s landF={r['land_force']:5.1f}N sat={r['sat_after']} "
                  f"kneeMax={r['knee_max_after']:.2f} tiltAfter={r['max_tilt_after']:5.1f} bounces={r['bounces']} flights={len(hops)} good={r['good']} fall={fall}", flush=True)
        else:
            print(f"E={E:.2f} no flight, fall={fall}", flush=True)
        res.append(r)
(OUT / 'energy_sweep.json').write_text(json.dumps(res, indent=1, default=float))
