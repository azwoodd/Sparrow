"""Phase 0a: model audit from the isolated copy (mass, joints, actuators, standing geometry, appendage mass)."""
import json, math
import numpy as np
from lab import Lab, WS, mujoco, MODEL_DIR

lab = Lab(); m, d, s = lab.m, lab.d, lab.s
lab.reset({})
out = {'model_dir': str(MODEL_DIR), 'mujoco': mujoco.__version__, 'nbody': int(m.nbody), 'njnt': int(m.njnt), 'nu': int(m.nu), 'ngeom': int(m.ngeom),
       'nmesh': int(m.nmesh), 'timestep': lab.dt, 'total_mass_kg': float(s.total_mass), 'weight_N': lab.weight}
out['bodies'] = [{'name': mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, b), 'mass_g': round(float(m.body_mass[b]) * 1000, 2),
                  'subtree_g': round(float(m.body_subtreemass[b]) * 1000, 2)} for b in range(1, m.nbody)]
out['joints'] = [{'name': mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, j), 'axis': m.jnt_axis[j].tolist(), 'range': m.jnt_range[j].round(4).tolist()}
                 for j in range(m.njnt)]
out['actuators'] = [{'name': mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_ACTUATOR, i), 'kp': float(m.actuator_gainprm[i, 0]),
                     'ctrlrange': m.actuator_ctrlrange[i].round(4).tolist(), 'forcerange': m.actuator_forcerange[i].tolist()} for i in range(m.nu)]
com = s.com()
st = {'com_m': com.round(5).tolist(), 'root_qpos': d.qpos[:7].round(5).tolist(), 'leg_q': {j: round(float(d.qpos[lab.qi[k]]), 5) for k, j in enumerate(lab.leg_names)},
      'balance_error_mm': s.balance_error_mm}
for S in 'LR':
    pos, corners, yaw, tilt, R = lab.foot_pose(S)
    st['foot_' + S] = {'centre_m': pos.round(5).tolist(), 'x_range_mm': [round(corners[:, 0].min() * 1000, 2), round(corners[:, 0].max() * 1000, 2)],
                       'y_range_mm': [round(corners[:, 1].min() * 1000, 2), round(corners[:, 1].max() * 1000, 2)], 'lowest_mm': round(corners[:, 2].min() * 1000, 3)}
st['com_x_to_inner_edge_L_mm'] = round((st['foot_L']['x_range_mm'][0] - com[0] * 1000), 2)
st['com_x_to_inner_edge_R_mm'] = round((com[0] * 1000 - st['foot_R']['x_range_mm'][1]), 2)
out['standing_reset'] = st
(WS / 'results' / 'p0_audit').mkdir(parents=True, exist_ok=True)
(WS / 'results' / 'p0_audit' / 'model_audit.json').write_text(json.dumps(out, indent=1))
print(json.dumps({k: v for k, v in out.items() if k not in ('joints', 'actuators')}, indent=1))
