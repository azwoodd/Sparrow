"""Foot contact geometry check from the CAD-exported visual meshes in the copied MuJoCo model (the meshes come from the
Blender export; the live Blender session was not reachable for this check). Compares the real underside of the foot with
the simplified box collider used for all contact so far. Coordinates are in the ankle-joint frame: x lateral, forward = -y."""
import json, os
os.environ.setdefault('SPARROW_VARIANT', 'v7e_stiffcontact')
import numpy as np, mujoco
from lab import Lab, WS

lab = Lab(); m = lab.m
b = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, 'leg_L_ankle')
V = []; parts = []
for g in range(m.ngeom):
    if m.geom_bodyid[g] != b or m.geom_type[g] != mujoco.mjtGeom.mjGEOM_MESH:
        continue
    mid = m.geom_dataid[g]
    v = m.mesh_vert[m.mesh_vertadr[mid]:m.mesh_vertadr[mid] + m.mesh_vertnum[mid]]
    q = m.geom_quat[g]; R = np.zeros(9); mujoco.mju_quat2Mat(R, q); w = m.geom_pos[g] + v @ R.reshape(3, 3).T
    V.append(w); parts.append({'mesh': mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_MESH, mid), 'verts': int(len(w)),
                               'x_mm': [round(float(w[:, 0].min() * 1000), 1), round(float(w[:, 0].max() * 1000), 1)],
                               'y_mm': [round(float(w[:, 1].min() * 1000), 1), round(float(w[:, 1].max() * 1000), 1)],
                               'z_mm': [round(float(w[:, 2].min() * 1000), 1), round(float(w[:, 2].max() * 1000), 1)]})
V = np.vstack(V); zmin = float(V[:, 2].min())
cg = lab.foot_geom['L']; cp = m.geom_pos[cg]; cs = m.geom_size[cg]
out = {'parts_on_ankle_body': parts, 'lowest_point_z_mm': zmin * 1000,
       'collider_box': {'x_mm': [(cp[0] - cs[0]) * 1000, (cp[0] + cs[0]) * 1000], 'y_mm': [(cp[1] - cs[1]) * 1000, (cp[1] + cs[1]) * 1000],
                        'bottom_z_mm': (cp[2] - cs[2]) * 1000, 'size_mm': [2 * cs[0] * 1000, 2 * cs[1] * 1000]}, 'underside_bands': {}}
print('visual meshes on the ankle (foot) body:')
for p in parts: print('  ', p)
print(f"collider box: x {out['collider_box']['x_mm'][0]:.0f}..{out['collider_box']['x_mm'][1]:.0f}  y {out['collider_box']['y_mm'][0]:.0f}..{out['collider_box']['y_mm'][1]:.0f}  bottom z {out['collider_box']['bottom_z_mm']:.1f} mm (ankle joint at 0,0,0)")
print(f"lowest CAD vertex: z = {zmin * 1000:.1f} mm")
for band in (0.3, 1.0, 2.0, 4.0):
    s = V[V[:, 2] <= zmin + band / 1000]
    rec = {'n_vertices': int(len(s)), 'x_mm': [float(s[:, 0].min() * 1000), float(s[:, 0].max() * 1000)], 'y_mm': [float(s[:, 1].min() * 1000), float(s[:, 1].max() * 1000)]}
    out['underside_bands'][f'within_{band}mm'] = rec
    print(f"CAD surface within {band:.1f} mm of the lowest point: x {rec['x_mm'][0]:6.1f}..{rec['x_mm'][1]:6.1f} (width {rec['x_mm'][1] - rec['x_mm'][0]:5.1f})  "
          f"y {rec['y_mm'][0]:6.1f}..{rec['y_mm'][1]:6.1f} (length {rec['y_mm'][1] - rec['y_mm'][0]:5.1f}), {rec['n_vertices']} vertices")
(WS / 'results' / 'g1_foot_geometry').mkdir(parents=True, exist_ok=True)
(WS / 'results' / 'g1_foot_geometry' / 'foot_geometry.json').write_text(json.dumps(out, indent=1, default=float))
