"""Identical take-off and landing tests on one model variant (set SPARROW_VARIANT). Hop parameters are the same robust set
(from the v7e_stiffcontact library) for every variant; nothing is re-tuned per variant except the single hip-bias knob,
which is swept up to that variant's own hip torque limit.
  A  take-off: spring-only angular momentum, and the hip bias sweep (ankles following) at 0.40 / 0.50 / 0.60 J per knee
  B  landing window: touchdown-only drops (60 / 80 / 100 mm speeds; pitch -5..+15 deg; rate 0 / 60 deg/s; forward 0.1 / 0.3 m/s)
  C  best complete hop found by the sweep at each energy, re-run on development conditions
Positive pitch = nose down."""
import json, os, sys
import numpy as np, mujoco
from lab import Lab, WS, VARIANT, arr
from hop import analyse_hops, DEFAULT
from hop2 import AlignedHopper
import t1_touchdown as t1

OUT = WS / 'results' / 'v1_variant_compare'; OUT.mkdir(parents=True, exist_ok=True)
ref = json.loads((WS / 'results' / 'h2_opt' / 'v7e_stiffcontact' / 'library.json').read_text())['ladder_E025']['params']
base = dict(ref, dz_c=0.030, dz_e=-0.020, align=1, hip_follow=1, ank_follow=1)
lab = Lab()
lim = float(lab.m.actuator_forcerange[lab.ids[0], 1])
orig = lab.measure
def measure():
    r = orig(); mujoco.mj_subtreeVel(lab.m, lab.d)
    r['Lx'] = float(lab.d.subtree_angmom[1][0]); r['vz'] = float(lab.d.subtree_linvel[1][2]); r['vfwd'] = float(-lab.d.subtree_linvel[1][1]); return r
lab.measure = measure
DEV = [('nominal', {}), ('friction0.6', {'friction': 0.6}), ('friction1.4', {'friction': 1.4}), ('mass0.93', {'mass': 0.93}), ('mass1.07', {'mass': 1.07}),
       ('comY+3', {'com_shift': 0.003}), ('comY-3', {'com_shift': -0.003})]


def hop(p, case=None):
    ctl = AlignedHopper([(0.5, p)])
    fall = lab.run(ctl, 4.5, case=case, stop_after_fall=0.2, every=1)
    hops = analyse_hops(lab.rows, lab.weight); h = hops[0] if hops else None
    r = {'fall': fall, 'flights': len(hops), 'good': False, 'E': p['E'], 'hip_bias': p.get('hip_bias', 0.0), 'peak_tau': ctl.peak_tau}
    if h:
        t = arr(lab.rows, 't'); k0 = int(np.searchsorted(t, h['t_takeoff'])); k1 = min(int(np.searchsorted(t, h['t_touchdown'])), len(t) - 1)
        a, b = lab.rows[k0], lab.rows[k1]
        r.update(clear=h['peak_clear_mm'], fwd=h['forward_mm'], L=a['Lx'] * 1000, vz=a['vz'], vfwd=a['vfwd'], pitch_TD=b['pitch_deg'], tilt_after=h['max_tilt_after_deg'],
                 landF=max((x['fz_L'] + x['fz_R'] for x in lab.rows[k1:k1 + 300]), default=0.0), good=bool(h['good'] and fall is None and len(hops) == 1))
    return r


out = {'variant': VARIANT, 'hip_limit_Nm': lim, 'mass_kg': float(lab.m.body_subtreemass[1]), 'takeoff': [], 'landing': [], 'dev': []}
print(f"=== {VARIANT}: mass {out['mass_kg']:.3f} kg, hip limit {lim:.2f} Nm")
best = {}
for E in (0.40, 0.50, 0.60):
    rows = []
    for frac in (0.0, 0.25, 0.5, 0.75, 1.0):
        r = hop(dict(base, E=E, hip_bias=frac * lim)); rows.append(r); out['takeoff'].append(r)
    Ls = [(r['hip_bias'], r.get('L')) for r in rows]
    print(f"A E={E:.2f}: L(mNms) vs hip bias " + ', '.join(f"{b:+.2f}Nm:{l:+5.1f}" if l is not None else f"{b:+.2f}:n/a" for b, l in Ls), flush=True)
    cand = [r for r in rows if 'L' in r]
    b = min(cand, key=lambda r: (not r['good'], abs(r['pitch_TD']))) if cand else None
    if b:
        best[E] = b
        print(f"   best: bias {b['hip_bias']:+.2f}Nm L={b['L']:+5.1f} clr={b['clear']:5.1f}mm vfwd={b['vfwd']:+.2f} pitchTD={b['pitch_TD']:+5.1f} landF={b['landF']:.0f}N tiltAfter={b['tilt_after']:.0f} good={b['good']}", flush=True)
land = {**DEFAULT, **ref}
for h in (60, 80, 100):
    cells = []
    for vf in (0.1, 0.3):
        for pitch in (-5, 0, 5, 10, 15):
            for rate in (0, 60):
                r = t1.run(lab, land, h, vf, pitch, rate); out['landing'].append(r); cells.append('P' if r['PASS'] else '.')
    s = ''.join(cells)
    print(f"B drop {h:3d}mm: vfwd0.1 [{s[:10]}] vfwd0.3 [{s[10:]}]  {s.count('P')}/20   (pairs = pitch -5,0,+5,+10,+15 at rate 0,60)   peakF {max(x['peak_force_N'] for x in out['landing'][-20:]):.0f}N", flush=True)
for E, b in best.items():
    n = 0; cl = []
    for cn, case in DEV:
        r = hop(dict(base, E=E, hip_bias=b['hip_bias']), case); r['cond'] = cn; out['dev'].append(r); n += r['good']; cl.append(r.get('clear', 0))
    print(f"C E={E:.2f} bias {b['hip_bias']:+.2f}Nm on development conditions: {n}/{len(DEV)} good, clearance {min(cl):.0f}-{max(cl):.0f} mm", flush=True)
(OUT / f'{VARIANT}.json').write_text(json.dumps(out, indent=1, default=float))
