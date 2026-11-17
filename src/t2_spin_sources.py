"""Which actuator makes the take-off spin? Same stiff-spring push (0.60 J per knee, crouch 30 mm), four servo policies
during the ~16 ms push: (a) preserved behaviour: hips and ankles chase the extension pose; (b) hips follow; (c) ankles
follow; (d) both follow (only the spring acts). Also with a small ankle bias torque. Angular momentum is read from the
simulator at the first airborne sample. Positive pitch = nose DOWN (top of body moves forward)."""
import json, os, sys
os.environ.setdefault('SPARROW_VARIANT', 'v7e_stiffcontact')
import numpy as np, mujoco
from lab import Lab, WS, VARIANT, arr
from hop import analyse_hops
from hop2 import AlignedHopper
from h2_opt import Library

OUT = WS / 'results' / 't2_spin_sources' / VARIANT; OUT.mkdir(parents=True, exist_ok=True)
lab = Lab(); lib = Library(); base = dict(lib.d['ladder_E025']['params'], dz_c=0.030, dz_e=-0.020)
orig = lab.measure
def measure():
    r = orig()
    mujoco.mj_subtreeVel(lab.m, lab.d)
    r['Lx'] = float(lab.d.subtree_angmom[1][0]); r['pitch_rate'] = float(np.degrees(lab.e.state()[2][0]))
    r['vz'] = float(lab.d.subtree_linvel[1][2]); r['vfwd'] = float(-lab.d.subtree_linvel[1][1]); return r
lab.measure = measure
res = []
cases = [('a_chase_pose', dict(align=0, dy_c=base['dy_c'], hip_follow=0, ank_follow=0)), ('b_hip_follow', dict(align=1, hip_follow=1, ank_follow=0)),
         ('c_ankle_follow', dict(align=1, hip_follow=0, ank_follow=1)), ('d_both_follow', dict(align=1, hip_follow=1, ank_follow=1)),
         ('e_both+hipdamp', dict(align=1, hip_follow=1, ank_follow=1, kh=0.15, kp_h=1.0)),
         ('f_both+ank-0.1Nm', dict(align=1, hip_follow=1, ank_follow=1, ank_bias=-0.1)), ('g_both+ank+0.1Nm', dict(align=1, hip_follow=1, ank_follow=1, ank_bias=0.1))]
for E in (0.25, 0.40, 0.60):
    for name, ch in cases:
        p = dict(base, E=E, **ch)
        ctl = AlignedHopper([(0.5, p)])
        fall = lab.run(ctl, 4.5, stop_after_fall=0.2, every=1)
        hops = analyse_hops(lab.rows, lab.weight); h = hops[0] if hops else None
        r = {'E': E, 'case': name, 'fall': fall, 'flights': len(hops), 'peak_tau': ctl.peak_tau}
        if h:
            t = arr(lab.rows, 't'); k0 = int(np.searchsorted(t, h['t_takeoff'])); k1 = min(int(np.searchsorted(t, h['t_touchdown'])), len(t) - 1)
            a = lab.rows[k0]; b = lab.rows[k1]
            t_th = next(tt for tt, i, ph in ctl.log if ph == 'thrust')
            push = [x for x in lab.rows if t_th <= x['t'] <= h['t_takeoff'] and x['fz_L'] + x['fz_R'] > 1.0]
            r.update(clear=h['peak_clear_mm'], push_ms=len(push), peak_push_N=max((x['fz_L'] + x['fz_R'] for x in push), default=0), Lx_takeoff_mNms=a['Lx'] * 1000,
                     spin_takeoff=a['pitch_rate'], vz=a['vz'], vfwd=a['vfwd'], pitch_TO=a['pitch_deg'], pitch_TD=b['pitch_deg'], good=bool(h['good'] and fall is None and len(hops) == 1),
                     tilt_after=h['max_tilt_after_deg'])
            print(f"E={E:.2f} {name:18} push={r['push_ms']:3d}ms peak={r['peak_push_N']:5.1f}N  L={r['Lx_takeoff_mNms']:+6.2f}mNms spin={r['spin_takeoff']:+6.0f}deg/s vz={r['vz']:.2f} vfwd={r['vfwd']:+.2f} "
                  f"clr={r['clear']:5.1f} pitchTO={r['pitch_TO']:+5.1f} pitchTD={r['pitch_TD']:+6.1f} tiltAfter={r['tilt_after']:5.1f} good={r['good']} fall={fall}", flush=True)
        else:
            print(f"E={E:.2f} {name:18} no flight fall={fall}", flush=True)
        res.append(r)
(OUT / 'spin_sources.json').write_text(json.dumps(res, indent=1, default=float))
