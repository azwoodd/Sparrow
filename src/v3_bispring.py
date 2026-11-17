"""Hip-and-knee spring variant: sweep the hip/knee split (rho) and where the feet sit in the crouch (dy_c), at fixed energy.
Servos only follow during the push (no hip bias), so any spin cancellation here comes from the spring geometry itself.
Set SPARROW_VARIANT to a v7e_bispring* copy.   python v3_bispring.py sweep [E]   |   python v3_bispring.py check E rho dy_c [dz_l dy_l]
Positive pitch = nose down."""
import json, os, sys
import numpy as np, mujoco
from lab import Lab, WS, VARIANT, arr
from hop import analyse_hops
from hop3 import BiHopper

OUT = WS / 'results' / 'v3_bispring'; OUT.mkdir(parents=True, exist_ok=True)
ref = json.loads((WS / 'results' / 'h2_opt' / 'v7e_stiffcontact' / 'library.json').read_text())['ladder_E025']['params']
base = dict(ref, dz_c=0.030, dz_e=-0.020, align=0, hip_follow=1, ank_follow=1, kh=0.10, kp_h=1.0)
lab = Lab()
orig = lab.measure
def measure():
    r = orig(); mujoco.mj_subtreeVel(lab.m, lab.d)
    r['Lx'] = float(lab.d.subtree_angmom[1][0]); r['vz'] = float(lab.d.subtree_linvel[1][2]); r['vfwd'] = float(-lab.d.subtree_linvel[1][1]); return r
lab.measure = measure
DEV = [('nominal', {}), ('friction0.6', {'friction': 0.6}), ('friction1.4', {'friction': 1.4}), ('mass0.93', {'mass': 0.93}), ('mass1.07', {'mass': 1.07}),
       ('comY+3', {'com_shift': 0.003}), ('comY-3', {'com_shift': -0.003}), ('pitch+2', {'lean': 2.0, 'lean_axis': 'pitch'}), ('roll+2', {'lean': 2.0, 'lean_axis': 'roll'})]
HOLD = [('H_fr0.5_m1.05', {'friction': 0.5, 'mass': 1.05}), ('H_fr0.8_m0.95_c+2', {'friction': 0.8, 'mass': 0.95, 'com_shift': 0.002}), ('H_fr1.6', {'friction': 1.6}),
        ('H_m1.1_c-2', {'mass': 1.1, 'com_shift': -0.002}), ('H_fr0.7_pitch+1.5', {'friction': 0.7, 'lean': 1.5, 'lean_axis': 'pitch'})]


def hop(p, case=None):
    ctl = BiHopper([(0.5, p)])
    fall = lab.run(ctl, 4.5, case=case, stop_after_fall=0.2, every=1)
    hops = analyse_hops(lab.rows, lab.weight); h = hops[0] if hops else None
    r = {'fall': fall, 'flights': len(hops), 'good': False, 'peak_knee_tau': ctl.peak_tau, 'peak_hip_tau': ctl.peak_hip_tau, 'stored': ctl.stored,
         **{k: p.get(k) for k in ('E', 'rho', 'dy_c', 'dz_l', 'dy_l')}}
    if h:
        t = arr(lab.rows, 't'); k0 = int(np.searchsorted(t, h['t_takeoff'])); k1 = min(int(np.searchsorted(t, h['t_touchdown'])), len(t) - 1)
        a, b = lab.rows[k0], lab.rows[k1]
        r.update(clear=h['peak_clear_mm'], fwd=h['forward_mm'], L=a['Lx'] * 1000, vz=a['vz'], vfwd=a['vfwd'], pitch_TD=b['pitch_deg'], tilt_after=h['max_tilt_after_deg'],
                 landF=max((x['fz_L'] + x['fz_R'] for x in lab.rows[k1:k1 + 300]), default=0.0), good=bool(h['good'] and fall is None and len(hops) == 1))
    return r


def show(tag, r):
    if 'clear' in r:
        print(f"{tag:30} L={r['L']:+6.1f}mNms vz={r['vz']:.2f} vfwd={r['vfwd']:+.2f} clr={r['clear']:5.1f} fwd={r['fwd']:6.1f} pitchTD={r['pitch_TD']:+6.1f} "
              f"kneeTau={r['peak_knee_tau']:.2f} hipTau={r['peak_hip_tau']:.2f}Nm landF={r['landF']:4.0f}N tiltAfter={r['tilt_after']:5.1f} flights={r['flights']} good={r['good']} fall={r['fall']}", flush=True)
    else:
        print(f"{tag:30} no flight fall={r['fall']}", flush=True)


if __name__ == '__main__':
    mode = sys.argv[1]; res = []
    print(f'=== {VARIANT}  mass {float(lab.m.body_subtreemass[1]):.3f} kg')
    if mode == 'sweep':
        E = float(sys.argv[2]) if len(sys.argv) > 2 else 0.50
        for rho in (0.0, -0.2, -0.4, -0.6, -0.8):
            for dyc in (-0.045, -0.035, -0.025, -0.015, -0.005):
                r = hop(dict(base, E=E, rho=rho, dy_c=dyc)); res.append(r); show(f'E={E:.2f} rho={rho:+.1f} dy_c={dyc * 1000:+4.0f}mm', r)
        (OUT / f'sweep_{VARIANT}_E{int(E * 100)}.json').write_text(json.dumps(res, indent=1, default=float))
        print('good:', sum(r['good'] for r in res), '/', len(res))
    elif mode == 'esweep':
        # raise the energy at the spring splits that cancelled the spin, with the feet placed so forward speed is near zero
        for rho in (-0.6, -0.8, -1.0):
            off = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0          # shift of the crouch position for models whose foot sits differently
            for dyc in (-0.045 + off, -0.035 + off, -0.025 + off, -0.015 + off):
                for E in (0.65, 0.80, 0.95):
                    r = hop(dict(base, E=E, rho=rho, dy_c=dyc)); res.append(r); show(f'E={E:.2f} rho={rho:+.1f} dy_c={dyc * 1000:+4.0f}mm', r)
        (OUT / f'esweep_{VARIANT}{sys.argv[2] if len(sys.argv) > 2 else str()}.json').write_text(json.dumps(res, indent=1, default=float))
        print('good:', sum(r['good'] for r in res), '/', len(res))
    elif mode == 'check_lib':
        # validate a library entry of THIS variant on development + holdout conditions:  python v3_bispring.py check_lib <entry>
        name = sys.argv[2]; p = json.loads((WS / 'results' / 'h2_opt' / VARIANT / 'library.json').read_text())[name]['params']
        tot = {}
        for label, conds in (('development', DEV), ('holdout', HOLD)):
            n = 0; cl = []; lf = []
            for cn, case in conds:
                r = hop(p, case); r.update(cond=cn, set=label); res.append(r); n += r['good']; cl.append(r.get('clear', 0)); lf.append(r.get('landF', 0)); show(f'{label[:4]} {cn}', r)
            print(f'== {VARIANT} {name}: {label} {n}/{len(conds)} good, clearance {min(cl):.0f}-{max(cl):.0f} mm, peak landing force {max(lf):.0f} N', flush=True)
        (OUT / f'check_{VARIANT}_{name}.json').write_text(json.dumps({'params': p, 'results': res}, indent=1, default=float))
    elif mode == 'check':
        E, rho, dyc = float(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4])
        p = dict(base, E=E, rho=rho, dy_c=dyc)
        if len(sys.argv) > 6: p.update(dz_l=float(sys.argv[5]), dy_l=float(sys.argv[6]))
        for label, conds in (('development', DEV), ('holdout', HOLD)):
            n = 0; cl = []
            for cn, case in conds:
                r = hop(p, case); r.update(cond=cn, set=label); res.append(r); n += r['good']; cl.append(r.get('clear', 0)); show(f'{label[:4]} {cn}', r)
            print(f'== {VARIANT} E={E} rho={rho} dy_c={dyc}: {label} {n}/{len(conds)} good, clearance {min(cl):.0f}-{max(cl):.0f} mm', flush=True)
        (OUT / f'check_{VARIANT}_E{int(E * 100)}_rho{int(-rho * 10)}_dy{int(-dyc * 1000)}.json').write_text(json.dumps({'params': p, 'results': res}, indent=1, default=float))



