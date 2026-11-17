"""Jump feasibility, step 0: measure the model and do the energy / force / stroke arithmetic (no dynamics run).

Standing height is measured from the model geometry (highest point of any visual mesh above the ground in the standing
pose). Targets: feet clear the ground by 2x and 3x that height. All numbers come from the copied model + stated assumptions.
"""
import json, math, os
os.environ.setdefault('SPARROW_VARIANT', 'v7e_stiffcontact')
import numpy as np
from lab import Lab, WS, VARIANT, mujoco, G
import legs

lab = Lab(); m, d = lab.m, lab.d
lab.reset({}); mujoco.mj_forward(m, d)
top = -1e9; top_name = None; lo = 1e9
for g in range(m.ngeom):
    if m.geom_type[g] != mujoco.mjtGeom.mjGEOM_MESH:
        continue
    mid = m.geom_dataid[g]
    v = m.mesh_vert[m.mesh_vertadr[mid]:m.mesh_vertadr[mid] + m.mesh_vertnum[mid]]
    w = d.geom_xpos[g] + v @ d.geom_xmat[g].reshape(3, 3).T
    if w[:, 2].max() > top:
        top = float(w[:, 2].max()); top_name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g) or f'mesh geom {g} on body {mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, m.geom_bodyid[g])}'
    lo = min(lo, float(w[:, 2].min()))
ground = min(lab.foot_pose(S)[1][:, 2].min() for S in 'LR')
H = top - ground
mass = float(m.body_subtreemass[1]); com_z = float(lab.s.com()[2]) - ground
b = lab.e.base[lab.ids]; p0 = legs.fk(b[0], b[1])
L_stand = float(np.hypot(*p0)); L_max = legs.LA + legs.LB
q2max = legs.Q2_RANGE[1]; gmin = q2max + legs.G0
L_min = float(math.sqrt(legs.LA ** 2 + legs.LB ** 2 + 2 * legs.LA * legs.LB * math.cos(gmin)))
out = {'variant': VARIANT, 'standing_height_m': H, 'highest_part': top_name, 'mass_kg': mass, 'com_height_standing_m': com_z,
       'leg_hip_to_ankle_m': {'standing': L_stand, 'max_straight': L_max, 'min_at_knee_limit': L_min},
       'stroke_available_m': {'from_standing_crouch_only_as_low_hop': 0.035 + 0.020, 'full_knee_range': L_max - L_min}}
rows = []
for label, h in (('low hop today (30 mm)', 0.030), ('1x height', H), ('2x height', 2 * H), ('3x height', 3 * H)):
    E = mass * G * h; v = math.sqrt(2 * G * h)
    r = {'target': label, 'foot_clearance_m': h, 'energy_at_takeoff_J': E, 'takeoff_speed_m_s': v, 'flight_time_s': 2 * v / G,
         'landing_speed_m_s': v}
    for name, s in (('stroke_55mm', 0.055), ('stroke_full_%dmm' % round((L_max - L_min) * 1000), L_max - L_min)):
        Fav = E / s + mass * G                          # mean leg force over the stroke incl. weight
        r[name] = {'mean_force_N': Fav, 'mean_force_bodyweights': Fav / (mass * G), 'push_time_s_approx': 2 * s / v,
                   'mean_power_W': (E + mass * G * s) / (2 * s / v)}
    rows.append(r)
out['targets'] = rows
# what the existing actuators can do: two knee servos (0.35 Nm) + two hip servos (0.35 Nm) + two ankles (0.2 Nm)
dq_knee = q2max - 0.0
out['servo_only_energy_upper_bound_J'] = {'knees_2x0.35Nm_over_%.2frad' % dq_knee: 2 * 0.35 * dq_knee, 'hips_2x0.35Nm_over_1.0rad': 2 * 0.35 * 1.0,
                                          'ankles_2x0.2Nm_over_1.0rad': 2 * 0.2 * 1.0, 'note': 'upper bound: full torque over full travel, no speed limit, no losses'}
# spring mass for the 2x / 3x energy at typical usable specific energies (J per kg of spring material, incl. ~50% margin on E)
dens = {'steel torsion/compression spring (~100 J/kg usable)': 100.0, 'CFRP/glass leaf or bow spring (~500 J/kg usable)': 500.0,
        'natural rubber / latex in tension (~2000 J/kg usable)': 2000.0}
out['spring_mass_g'] = {r['target']: {k: 1000 * 1.5 * r['energy_at_takeoff_J'] / v for k, v in dens.items()} for r in rows}
out['winding_time_s_at_shaft_power'] = {r['target']: {f'{P} W': 1.5 * r['energy_at_takeoff_J'] / (P * 0.5) for P in (0.5, 1.0, 2.0)} for r in rows}
out['assumptions'] = ['jump height = foot clearance above ground', 'spring sized for 1.5x take-off energy (losses in legs, foot mass, friction)',
                      'winder efficiency 50% (small gearmotor + cord/cam)', 'no aerodynamic help', 'mass fixed at model mass; extra hardware NOT yet added']
(WS / 'results' / 'j0_feasibility').mkdir(parents=True, exist_ok=True)
(WS / 'results' / 'j0_feasibility' / 'feasibility_numbers.json').write_text(json.dumps(out, indent=1))
print(f"standing height {H * 1000:.0f} mm (top: {top_name}); mass {mass:.3f} kg; COM {com_z * 1000:.0f} mm above ground")
print(f"leg hip-ankle: standing {L_stand * 1000:.0f} mm, straight {L_max * 1000:.0f} mm, fully flexed {L_min * 1000:.0f} mm -> max stroke {(L_max - L_min) * 1000:.0f} mm")
for r in rows:
    a, c = r['stroke_55mm'], [v for k, v in r.items() if k.startswith('stroke_full')][0]
    print(f"{r['target']:22} clear {r['foot_clearance_m'] * 1000:5.0f} mm  E={r['energy_at_takeoff_J']:.2f} J  v0={r['takeoff_speed_m_s']:.2f} m/s  flight {r['flight_time_s']:.2f} s | "
          f"55mm stroke: {a['mean_force_N']:.0f} N ({a['mean_force_bodyweights']:.0f} BW) {a['mean_power_W']:.0f} W | full stroke: {c['mean_force_N']:.0f} N ({c['mean_force_bodyweights']:.0f} BW) {c['mean_power_W']:.0f} W in {c['push_time_s_approx'] * 1000:.0f} ms")
print('servo-only energy upper bound (J):', {k: round(v, 2) for k, v in out['servo_only_energy_upper_bound_J'].items() if k != 'note'})
for t, dct in out['spring_mass_g'].items():
    print(f"spring mass for {t}: " + ', '.join(f"{k.split('(')[0].strip()} {v:.0f} g" for k, v in dct.items()))
for t, dct in out['winding_time_s_at_shaft_power'].items():
    print(f"winding time for {t}: " + ', '.join(f"{k}: {v:.1f} s" for k, v in dct.items()))
