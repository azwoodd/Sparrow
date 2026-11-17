"""Where does the take-off spin come from? Trace one strong aligned push at 1 kHz: centre of pressure relative to the ankle
and to the COM, foot tilt (heel lift), ground force, body pitch and pitch rate, hip/ankle servo torque."""
import os, sys, json
os.environ.setdefault('SPARROW_VARIANT', 'v7e_stiffcontact')
import numpy as np, mujoco
from lab import Lab, WS, VARIANT
from hop2 import AlignedHopper
from h2_opt import Library

E = float(sys.argv[1]) if len(sys.argv) > 1 else 0.60
lab = Lab(); lib = Library(); p = dict(lib.d['ladder_E025']['params'], dz_c=0.030, E=E, align=1, kh=0.15, kp_h=1.0)
ctl = AlignedHopper([(0.5, p)])
ank = mujoco.mj_name2id(lab.m, mujoco.mjtObj.mjOBJ_BODY, 'leg_L_ankle')
extra = []
orig = lab.measure
def measure():
    r = orig(); r['ankle_y'] = float(lab.d.xpos[ank][1]); r['pitch_rate_deg_s'] = float(np.degrees(lab.e.state()[2][0])); return r
lab.measure = measure
lab.run(ctl, 1.7, stop_after_fall=0.05, every=1)
t_thrust = next(t for t, i, ph in ctl.log if ph == 'thrust'); t_flight = next((t for t, i, ph in ctl.log if ph == 'flight'), 9)
print(f"E={E} J/knee  thrust starts {t_thrust:.3f}s  flight {t_flight:.3f}s   (forward is -y: 'ahead' columns are positive toward the toes)")
print("  t     F_total  CoP_ahead_of_ankle  CoP_ahead_of_COM  foot_tilt  pitch   pitch_rate  hip_Nm  ankle_Nm  knee_q")
out = []
for r in lab.rows:
    if t_thrust - 0.01 <= r['t'] <= t_flight + 0.01 and round(r['t'] * 1000) % 8 == 0:
        F = r['fz_L'] + r['fz_R']
        cop = r['cop_y_L'] if r['fz_L'] > 0 else float('nan')
        row = dict(t=r['t'], F=F, cop_ahead_ankle_mm=(r['ankle_y'] - cop) * 1000, cop_ahead_com_mm=(r['com_y'] - cop) * 1000, foot_tilt=r['foot_tilt_deg_L'],
                   pitch=r['pitch_deg'], rate=r['pitch_rate_deg_s'], hip=r['leg_L_thigh_Nm'], ankle=r['leg_L_ankle_Nm'], knee=r['leg_L_shin_q'])
        out.append(row)
        print(f"{row['t']:6.3f} {F:7.1f}N {row['cop_ahead_ankle_mm']:12.1f}mm {row['cop_ahead_com_mm']:14.1f}mm {row['foot_tilt']:9.1f} {row['pitch']:+6.1f} {row['rate']:+9.0f} {row['hip']:+7.2f} {row['ankle']:+8.2f} {row['knee']:7.3f}")
(WS / 'results' / 'h7_hip_pd' / VARIANT / f'push_trace_E{int(E * 100)}.json').write_text(json.dumps(out, indent=1, default=float))
