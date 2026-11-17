"""Turn the simulated hip-and-knee spring into numbers a mechanism can be designed to (set SPARROW_VARIANT=v7e_bispring).
For each validated hop: spring stretch, stiffness, peak torques, push duration, joint speeds, ground force; then, for a few
candidate knee pulley radii, the cable travel, peak cable tension and the equivalent linear spring. Spring mass and winding
time are bounded scenarios, not selected parts. Nothing here is a CAD change."""
import json, os, sys, math
import numpy as np
from lab import Lab, WS, VARIANT, arr
from hop import analyse_hops
from hop3 import BiHopper

OUT = WS / 'results' / 'v4_detail'; OUT.mkdir(parents=True, exist_ok=True)
lib = json.loads((WS / 'results' / 'h2_opt' / VARIANT / 'library.json').read_text())
lab = Lab(); out = {}
for name, label in (('bi_E080', '60 mm'), ('bi_E100', '80 mm'), ('bi_E110', '100 mm')):
    p = lib[name]['params']; ctl = BiHopper([(0.5, p)])
    lab.run(ctl, 4.5, stop_after_fall=0.2, every=1)
    hops = analyse_hops(lab.rows, lab.weight); h = hops[0]
    t_th = next(t for t, i, ph in ctl.log if ph == 'thrust')
    push = [r for r in lab.rows if t_th <= r['t'] <= h['t_takeoff']]
    loaded = [r for r in push if r['fz_L'] + r['fz_R'] > 1.0]
    t = arr(push, 't'); qk = arr(push, 'leg_L_shin_q'); qh = arr(push, 'leg_L_thigh_q'); qa = arr(push, 'leg_L_ankle_q')
    wk = np.abs(np.gradient(qk, t)).max(); wh = np.abs(np.gradient(qh, t)).max(); wa = np.abs(np.gradient(qa, t)).max()
    crouch = next(r for r in lab.rows if r['t'] >= t_th - 0.01)
    du = ctl.max_stretch; ku = ctl.ku[0]; E = p['E']; rho = p['rho']
    rec = {'target': label, 'energy_per_leg_J': E, 'energy_total_J': 2 * E, 'rho': rho, 'spring_stretch_rad': du, 'stiffness_Nm_per_rad': ku,
           'peak_knee_torque_Nm': ctl.peak_tau, 'peak_hip_torque_Nm': ctl.peak_hip_tau, 'push_duration_ms': len(loaded),
           'peak_ground_force_N': max(r['fz_L'] + r['fz_R'] for r in push), 'peak_knee_speed_rad_s': float(wk), 'peak_hip_speed_rad_s': float(wh), 'peak_ankle_speed_rad_s': float(wa),
           'crouch_knee_rad': crouch['leg_L_shin_q'], 'crouch_hip_rad': crouch['leg_L_thigh_q'], 'standing_knee_rad': float(ctl.base[1]), 'standing_hip_rad': float(ctl.base[0]),
           'clearance_mm': h['peak_clear_mm'], 'hip_servo_trim_Nm': p.get('hip_bias', 0.0), 'pulleys': {}}
    for rk in (0.006, 0.008, 0.010, 0.012):
        travel = rk * du; Fpk = ctl.peak_tau / rk
        rec['pulleys'][f'knee_r_{int(rk * 1000)}mm'] = {'hip_radius_mm': abs(rho) * rk * 1000, 'cable_travel_mm': travel * 1000, 'peak_cable_tension_N': Fpk,
                                                       'linear_spring_N_per_mm': ku / rk ** 2 / 1000, 'check_energy_J': 0.5 * (ku / rk ** 2) * travel ** 2}
    for eff in (1.0, 0.8):
        Es = 2 * E / eff
        rec[f'stored_total_at_eff_{eff}'] = {'J': Es, 'steel_spring_g_at_100J_kg': Es / 100 * 1000, 'composite_leaf_g_at_500J_kg': Es / 500 * 1000,
                                             'rubber_g_at_2000J_kg': Es / 2000 * 1000, 'winding_s_at_1W_shaft_50pct': Es / 0.5, 'winding_s_at_2W_shaft_50pct': Es / 1.0}
    out[name] = rec
    print(f"--- {label} hop ({name}): E {E:.2f} J per leg, clearance {h['peak_clear_mm']:.0f} mm")
    print(f"    spring stretch {du:.2f} rad (knee {rec['standing_knee_rad']:.2f} -> {rec['crouch_knee_rad']:.2f} rad, hip {rec['standing_hip_rad']:.2f} -> {rec['crouch_hip_rad']:.2f} rad), stiffness {ku:.2f} Nm/rad")
    print(f"    peak spring torque knee {ctl.peak_tau:.2f} Nm, hip {ctl.peak_hip_tau:.2f} Nm; push lasts {len(loaded)} ms, peak ground force {rec['peak_ground_force_N']:.0f} N")
    print(f"    peak joint speeds during push: knee {wk:.0f}, hip {wh:.0f}, ankle {wa:.0f} rad/s")
    for k, v in rec['pulleys'].items():
        print(f"    {k}: hip pulley {v['hip_radius_mm']:.1f} mm, cable travel {v['cable_travel_mm']:.1f} mm, peak tension {v['peak_cable_tension_N']:.0f} N, linear spring {v['linear_spring_N_per_mm']:.1f} N/mm")
    for eff in (1.0, 0.8):
        s = rec[f'stored_total_at_eff_{eff}']
        print(f"    stored at efficiency {eff}: {s['J']:.2f} J total -> steel {s['steel_spring_g_at_100J_kg']:.0f} g, composite {s['composite_leaf_g_at_500J_kg']:.0f} g, rubber {s['rubber_g_at_2000J_kg']:.1f} g; winding {s['winding_s_at_1W_shaft_50pct']:.1f} s at 1 W, {s['winding_s_at_2W_shaft_50pct']:.1f} s at 2 W")
(OUT / 'mechanism_numbers.json').write_text(json.dumps(out, indent=1, default=float))
