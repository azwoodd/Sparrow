"""Flight research, step 0: physical feasibility audit of the CURRENT V7E design. Measurement + bounded arithmetic only.

Reads the copied MuJoCo model (never the originals), measures geometry/mass/inertia, and computes bounded scenarios for
wing-borne flight, flapping, power and launch. No dynamics are simulated and nothing here shows that Sparrow flies.
Every number is tagged:  [M] measured from the copied model/CAD export   [S] component spec (none exist yet)
                         [C] engineering calculation from M + stated formula   [A] assumption needing validation
Output: flight_research/results/f0_audit.json and F0_FEASIBILITY_AUDIT.md
"""
import json, math, os, sys
from pathlib import Path
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / 'src'))
os.environ.setdefault('SPARROW_VARIANT', 'v7e_stiffcontact')
import numpy as np
from lab import Lab, MODEL_DIR, mujoco

RHO, G = 1.225, 9.81
lab = Lab(); m, d, s = lab.m, lab.d, lab.s
info = json.load(open(MODEL_DIR / 'sparrow_v7e_model.json'))
out = {}

# ---------------------------------------------------------------- geometry and mass [M]
s.reset('stand_wings_deployed'); mujoco.mj_forward(m, d)
mass = float(m.body_subtreemass[1]); W = mass * G
strips = info['aero_strips']
wing = [x for x in strips if x['body'].startswith('wing')]; tail = [x for x in strips if x['body'].startswith('tail')]
S_wing = sum(x['area_m2'] for x in wing); S_tail = sum(x['area_m2'] for x in tail)
# world position of every wing strip in the deployed pose -> span stations and chord distribution
pts = []
for x in wing:
    b = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, x['body'])
    pts.append((d.xpos[b] + d.xmat[b].reshape(3, 3) @ np.array(x['pos']), x['area_m2']))
ys = np.array([abs(p[0][0]) for p in pts]); areas = np.array([p[1] for p in pts])
span_strips = 2 * float(ys.max())
tipsL = d.geom_xpos[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, 'col_tip_L')]; tipsR = d.geom_xpos[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, 'col_tip_R')]
span = info['wing_span_mm'] / 1000
semi = span / 2
order = np.argsort(ys[:len(ys) // 2]) if False else np.argsort(ys)
stations = sorted(set(np.round(ys, 3)))
dy = semi / (len(stations))                                  # mean strip width
chords = {float(st): float(areas[np.isclose(ys, st, atol=1e-3)].sum() / 2 / dy) for st in stations}
AR = span ** 2 / S_wing; cbar = S_wing / span
com = s.com()
hinge = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, 'wing_L_flap')
hb = m.jnt_bodyid[hinge]; hpos = d.xanchor[hinge]; hax = d.xaxis[hinge]
# wing inertia about the flap hinge (deployed): sum over the subtree of body inertia + parallel axis
I_flap = 0.0; m_wing = 0.0; r_cg = np.zeros(3)
for b in range(m.nbody):
    k = b
    while k != 0 and k != hb:
        k = m.body_parentid[k]
    if k != hb:
        continue
    R = d.ximat[b].reshape(3, 3); Ib = R @ np.diag(m.body_inertia[b]) @ R.T
    r = d.xipos[b] - hpos; rp = r - (r @ hax) * hax
    I_flap += float(hax @ Ib @ hax) + m.body_mass[b] * float(rp @ rp); m_wing += float(m.body_mass[b]); r_cg += m.body_mass[b] * r
r_cg /= m_wing
flap_range = m.jnt_range[hinge]
flap_act = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, 'act_wing_L_flap')
out['measured'] = {
    'mass_kg': mass, 'weight_N': W, 'mass_by_group_g': info['mass_by_group_g'], 'com_deployed_m': com.round(4).tolist(),
    'wing_span_m_export': span, 'wing_tip_collider_span_m': float(abs(tipsL[0] - tipsR[0])), 'outermost_aero_strip_span_m': span_strips,
    'wing_area_m2_strips': S_wing, 'wing_area_m2_export_reference': info['wing_area_cm2'] / 1e4, 'tail_area_m2': S_tail,
    'aspect_ratio': AR, 'mean_chord_m': cbar, 'chord_by_span_station_m': {f'{k:.3f}': round(v, 4) for k, v in chords.items()},
    'wing_loading_N_m2': W / S_wing, 'wing_loading_g_dm2': mass * 1000 / (S_wing * 100),
    'one_wing_mass_kg': m_wing, 'one_wing_inertia_about_flap_hinge_kgm2': I_flap, 'one_wing_cg_from_hinge_m': float(np.linalg.norm(r_cg)),
    'flap_range_rad': flap_range.tolist(), 'flap_range_deg': np.degrees(flap_range).round(1).tolist(),
    'flap_actuator_model': {'type': 'ideal position servo (placeholder, NOT the crank gearbox)', 'kp': float(m.actuator_gainprm[flap_act, 0]),
                            'torque_limit_Nm': float(m.actuator_forcerange[flap_act, 1]), 'speed_limit': 'none modelled'},
    'tail_actuators': {'pitch_range_deg': 35, 'roll_range_deg': 30, 'torque_limit_Nm': 0.1, 'tail_mass_g': 5.6},
    'battery_mass_g_target': info['mass_by_group_g']['battery'], 'wings_and_flap_drive_g': info['mass_by_group_g']['wings and flap drive'],
}

# ---------------------------------------------------------------- fixed-wing limits [C] with [A] coefficient ranges
def vstall(cl): return math.sqrt(2 * W / (RHO * S_wing * cl))
fw = {}
for cl in (0.8, 1.0, 1.2, 1.4):
    fw[f'CLmax_{cl}'] = {'stall_speed_m_s': vstall(cl), 'cruise_1.2Vs_m_s': 1.2 * vstall(cl)}
out['fixed_wing_speed'] = fw
e_osw = 0.8
power = {}
for cd0 in (0.03, 0.05, 0.08):                                    # zero-lift drag incl. body, legs, feathers at Re ~ 5e4 [A]
    k = 1 / (math.pi * AR * e_osw)
    best = None
    for V in np.arange(4.0, 14.01, 0.25):
        cl = 2 * W / (RHO * S_wing * V * V)
        if cl > 1.3:
            continue
        D = 0.5 * RHO * V * V * S_wing * (cd0 + k * cl * cl); P = D * V
        if best is None or P < best['aero_power_W']:
            best = {'V_m_s': float(V), 'CL': cl, 'drag_N': D, 'L_over_D': W / D, 'aero_power_W': P}
    power[f'CD0_{cd0}'] = best
out['level_flight_min_power'] = power
Re = RHO * 7.0 * cbar / 1.81e-5
out['reynolds_at_7m_s'] = Re

# ---------------------------------------------------------------- flapping kinematics [C]
amp = float(flap_range[1])                                       # +-0.54 rad
tip_pp = 2 * semi * math.sin(amp)                                # peak-to-peak tip excursion
flap = {'stroke_amplitude_deg': math.degrees(amp), 'tip_excursion_peak_to_peak_m': tip_pp, 'cases': {}}
for U in (5.0, 7.0, 9.0):
    for St in (0.2, 0.3, 0.4):                                   # efficient cruising animals/ornithopters sit at St 0.2-0.4
        f = St * U / tip_pp; om = 2 * math.pi * f
        tau_inertia = I_flap * amp * om * om                     # peak torque to reverse one wing (sinusoidal stroke)
        flap['cases'][f'U{U:g}_St{St}'] = {'freq_Hz': f, 'peak_hinge_speed_rad_s': amp * om, 'peak_tip_speed_m_s': semi * amp * om,
                                           'peak_inertial_torque_per_wing_Nm': tau_inertia, 'peak_inertial_power_per_wing_W': 0.5 * tau_inertia * amp * om}
out['flapping'] = flap
out['flap_placeholder_servo_check'] = {'model_torque_limit_Nm': 1.5, 'note': 'compare with peak inertial torque per wing above; aerodynamic torque comes on top'}

# ---------------------------------------------------------------- slow-speed / take-off bound (actuator disc) [C]
swept = 2 * (0.5 * semi ** 2 * 2 * amp)                          # area swept by both wings through +-amp
P_ideal_swept = W ** 1.5 / math.sqrt(2 * RHO * swept); P_ideal_full = W ** 1.5 / math.sqrt(2 * RHO * math.pi * semi ** 2)
out['hover_bound'] = {'swept_area_m2': swept, 'ideal_induced_power_W_swept_area': P_ideal_swept, 'ideal_induced_power_W_full_disc': P_ideal_full,
                      'realistic_aero_power_W_at_figure_of_merit_0.25_0.5': [P_ideal_swept / 0.5, P_ideal_swept / 0.25]}

# ---------------------------------------------------------------- power train and battery scenarios [A]
sc = {}
for name, eta_prop, eta_drive in (('optimistic', 0.6, 0.7), ('middle', 0.4, 0.6), ('pessimistic', 0.25, 0.5)):
    for cd0 in (0.03, 0.05, 0.08):
        Pa = power[f'CD0_{cd0}']['aero_power_W']
        sc[f'{name}_CD0_{cd0}'] = {'electrical_power_cruise_W': Pa / (eta_prop * eta_drive)}
out['cruise_electrical_power_scenarios'] = sc
bat = {}
for wh_kg in (120, 160, 200):
    E = wh_kg * info['mass_by_group_g']['battery'] / 1000
    bat[f'{wh_kg}_Wh_per_kg'] = {'energy_Wh': E, 'minutes_at_10W': 60 * E / 10 * 0.8, 'minutes_at_25W': 60 * E / 25 * 0.8, 'minutes_at_50W': 60 * E / 50 * 0.8}
out['battery_45g_scenarios_80pct_usable'] = bat

# ---------------------------------------------------------------- launch gap [C]
vs = vstall(1.2)
out['launch_gap'] = {'min_flying_speed_m_s_CL1.2': vs, 'fastest_tested_gait_m_s': 0.249, 'low_hop_takeoff_speed_m_s': 0.77,
                     'kinetic_energy_at_min_flying_speed_J': 0.5 * mass * vs * vs,
                     'height_to_dive_for_that_speed_m': vs * vs / (2 * G),
                     'note': 'legs + springs supply < 1 m/s today; the remaining speed must come from flapping thrust or a drop'}
(HERE / 'results').mkdir(exist_ok=True)
(HERE / 'results' / 'f0_audit.json').write_text(json.dumps(out, indent=1))

M = out['measured']
print(f"mass {mass:.3f} kg  weight {W:.2f} N | span {span:.2f} m (tip colliders {M['wing_tip_collider_span_m']:.2f} m) | wing area {S_wing:.4f} m2 (export ref {M['wing_area_m2_export_reference']:.4f}) | AR {AR:.1f} | mean chord {cbar * 1000:.0f} mm")
print(f"wing loading {M['wing_loading_N_m2']:.1f} N/m2 ({M['wing_loading_g_dm2']:.1f} g/dm2) | tail area {S_tail:.4f} m2 | Re at 7 m/s {Re:.0f}")
print(f"one wing: {m_wing * 1000:.1f} g, inertia about flap hinge {I_flap:.5f} kg m2, cg {M['one_wing_cg_from_hinge_m'] * 1000:.0f} mm out | flap range {M['flap_range_deg']} deg")
print('chord by station (m):', M['chord_by_span_station_m'])
for k, v in fw.items(): print(f"  {k}: stall {v['stall_speed_m_s']:.1f} m/s, cruise {v['cruise_1.2Vs_m_s']:.1f} m/s")
for k, v in power.items(): print(f"  min-power level flight {k}: V={v['V_m_s']:.2f} m/s CL={v['CL']:.2f} L/D={v['L_over_D']:.1f} drag={v['drag_N']:.2f} N aero power={v['aero_power_W']:.1f} W")
for k, v in flap['cases'].items(): print(f"  flap {k}: f={v['freq_Hz']:.1f} Hz tip {v['peak_tip_speed_m_s']:.1f} m/s inertial torque {v['peak_inertial_torque_per_wing_Nm']:.2f} Nm/wing")
print('hover bound:', {k: (round(v, 1) if not isinstance(v, list) else [round(x, 1) for x in v]) for k, v in out['hover_bound'].items()})
print('cruise electrical W:', {k: round(v['electrical_power_cruise_W'], 1) for k, v in sc.items()})
print('battery:', {k: {a: round(b, 1) for a, b in v.items()} for k, v in bat.items()})
print('launch gap:', {k: (round(v, 2) if isinstance(v, float) else v) for k, v in out['launch_gap'].items() if k != 'note'})
