"""Sparrow V7E (lightweight flyer, three-section wings) MuJoCo harness.

Model: sparrow_v7e.xml (exported from Blender by scripts/v7e_export_mujoco.py).
Joint coordinates are the Blender control angles in radians; the neutral pose (all zero) is standing with wings spread.

Honest limits of this model
- Masses are weight targets for UNSELECTED parts plus CAD-volume estimates for structure.
- Actuator gains and torque limits are placeholders, not selected hardware.
- No self-collision: robot parts only collide with the floor.
- Flap is a position servo on each wing hinge, not the crank gearbox. Fold coupling (elbow, panel rolls) is by equality constraints.
- Aerodynamics: quasi-steady strips with a normal force only. Attached flow CN = CN_SLOPE*sin(alpha), capped at CN_MAX, then
  the flat-plate value 2*sin(alpha) at high angles. No leading-edge suction, no wing twist, no unsteady effects, no body drag.
  That means it can estimate lift and the drag due to lift, but it CANNOT show flapping thrust. Do not read flight from it.
"""
import json, math, os
import numpy as np
import mujoco

HERE = os.path.dirname(os.path.abspath(__file__))
RHO = 1.225
CN_SLOPE = 5.5      # per unit sin(alpha): 2*pi corrected for aspect ratio 14
CN_MAX = 1.2


class SparrowSim:
    def __init__(self, xml='sparrow_v7e.xml', wind=(0.0, 0.0, 0.0)):
        self.m = mujoco.MjModel.from_xml_path(os.path.join(HERE, xml))
        self.d = mujoco.MjData(self.m)
        self.info = json.load(open(os.path.join(HERE, 'sparrow_v7e_model.json')))
        self.wind = np.array(wind, float)          # air velocity in the world frame (m/s)
        self.aero_on = True
        self.strips = []
        for s in self.info['aero_strips']:
            bid = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_BODY, s['body'])
            self.strips.append((bid, np.array(s['pos']), s['area_m2'], np.array(s['normal'], float)))
        self.act = {mujoco.mj_id2name(self.m, mujoco.mjtObj.mjOBJ_ACTUATOR, i): i for i in range(self.m.nu)}
        self.jq = {}; self.jv = {}; self.jr = {}
        for j in range(self.m.njnt):
            n = mujoco.mj_id2name(self.m, mujoco.mjtObj.mjOBJ_JOINT, j)
            self.jq[n] = self.m.jnt_qposadr[j]; self.jv[n] = self.m.jnt_dofadr[j]; self.jr[n] = self.m.jnt_range[j].copy()
        self.aero_force = np.zeros(3)
        self.total_mass = float(self.m.body_subtreemass[1])
        self.ov = {}

    # ---------------------------------------------------------------- poses
    def reset(self, key='stand_wings_stowed', lift=0.0):
        k = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_KEY, key)
        mujoco.mj_resetDataKeyframe(self.m, self.d, k)
        self.d.qpos[2] += lift
        self.hold_pose(key)
        mujoco.mj_forward(self.m, self.d)

    def hold_pose(self, key, **override):
        """Set every position actuator target to the named pose."""
        q = dict(self.info['poses'][key]['q']); q.update(override)
        for name, i in self.act.items():
            if name.startswith('act_'):
                lo, hi = self.m.actuator_ctrlrange[i]
                self.d.ctrl[i] = min(max(q[name[4:]], lo), hi)
            else:
                self.d.ctrl[i] = 0.0

    def balance_legs(self, key, knee=None):
        """Keep the knee angle and flat feet; solve the hip angle that puts the whole-robot COM over mid-foot."""
        q = self.info['poses'][key]['q']
        kn = q['leg_L_shin'] if knee is None else knee
        g = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_GEOM, 'col_foot_L')

        def err(h):
            for S in 'LR':
                self.d.qpos[self.jq['leg_%s_thigh' % S]] = h
                self.d.qpos[self.jq['leg_%s_shin' % S]] = kn
                self.d.qpos[self.jq['leg_%s_ankle' % S]] = -(h + kn)
            mujoco.mj_forward(self.m, self.d)
            return self.d.subtree_com[1][1] - self.d.geom_xpos[g][1]
        lo, hi = self.jr['leg_L_thigh']
        for _ in range(40):
            mid = (lo + hi) / 2
            if err(mid) > 0:
                lo = mid
            else:
                hi = mid
        h = (lo + hi) / 2
        self.balance_error_mm = float(err(h)) * 1000
        self.d.qpos[2] -= self.d.geom_xpos[g][2] - self.m.geom_size[g][2] - 0.001
        mujoco.mj_forward(self.m, self.d)
        ov = {}
        for S in 'LR':
            ov['leg_%s_thigh' % S] = h; ov['leg_%s_shin' % S] = kn; ov['leg_%s_ankle' % S] = -(h + kn)
        return ov

    def reset_balanced(self, key='stand_wings_stowed', knee=None):
        self.reset(key)
        self.ov = self.balance_legs(key, knee)
        self.d.qvel[:] = 0
        self.hold_pose(key, **self.ov)
        return self.ov

    def set_target(self, joint, value):
        self.d.ctrl[self.act['act_' + joint]] = value

    def flap_targets(self, phase, amp_rad=math.radians(30)):
        """Both wings together; positive is wing tip up."""
        for S in 'LR':
            self.d.ctrl[self.act['act_wing_%s_flap' % S]] = amp_rad * math.sin(phase)

    # ---------------------------------------------------------------- aerodynamics
    def apply_aero(self):
        self.d.qfrc_applied[:] = 0
        self.aero_force[:] = 0
        if not self.aero_on:
            return
        vel = np.zeros(6)
        for bid, pos, area, nrm in self.strips:
            R = self.d.xmat[bid].reshape(3, 3)
            r = R @ pos
            p = self.d.xpos[bid] + r
            mujoco.mj_objectVelocity(self.m, self.d, mujoco.mjtObj.mjOBJ_BODY, bid, vel, 0)
            v = vel[3:] + np.cross(vel[:3], r) - self.wind
            sp = float(np.linalg.norm(v))
            if sp < 1e-6:
                continue
            n = R @ nrm
            vn = float(v @ n)
            sa = abs(vn) / sp
            cn = min(CN_SLOPE * sa, max(CN_MAX, 2.0 * sa))
            f = -0.5 * RHO * area * sp * sp * cn * math.copysign(1.0, vn) * n
            mujoco.mj_applyFT(self.m, self.d, f, np.zeros(3), p, bid, self.d.qfrc_applied)
            self.aero_force += f

    def step(self, n=1):
        for _ in range(n):
            self.apply_aero()
            mujoco.mj_step(self.m, self.d)

    # ---------------------------------------------------------------- measurements
    def torso_up(self):
        return float(self.d.xmat[1].reshape(3, 3)[2, 2])

    def com(self):
        return self.d.subtree_com[1].copy()

    def foot_clearance(self):
        z = []
        for S in 'LR':
            g = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_GEOM, 'col_foot_%s' % S)
            z.append(self.d.geom_xpos[g][2] - self.m.geom_size[g][2])
        return min(z)

    def upright(self):
        return bool(self.torso_up() > 0.9 and self.com()[2] > 0.09)

    def render(self, path, cam_pos=(0.9, -1.1, 0.45), look=(0, 0, 0.12), w=1280, h=800):
        try:
            r = mujoco.Renderer(self.m, h, w)
            cam = mujoco.MjvCamera()
            cam.type = mujoco.mjtCamera.mjCAMERA_FREE
            look = np.array(look) + np.array([self.d.qpos[0], self.d.qpos[1], 0])
            d = look - (np.array(cam_pos) + np.array([self.d.qpos[0], self.d.qpos[1], 0]))
            cam.lookat[:] = look; cam.distance = float(np.linalg.norm(d))
            cam.azimuth = math.degrees(math.atan2(d[1], d[0])); cam.elevation = math.degrees(math.asin(d[2] / np.linalg.norm(d)))
            opt = mujoco.MjvOption(); opt.geomgroup[3] = 0
            r.update_scene(self.d, cam, opt)
            img = r.render(); r.close()
            import struct, zlib
            raw = b''.join(b'\x00' + img[y].tobytes() for y in range(h))

            def ch(t, dta):
                c = struct.pack('>I', len(dta)) + t + dta
                return c + struct.pack('>I', zlib.crc32(t + dta) & 0xffffffff)
            open(path, 'wb').write(b'\x89PNG\r\n\x1a\n' + ch(b'IHDR', struct.pack('>IIBBBBB', w, h, 8, 2, 0, 0, 0)) + ch(b'IDAT', zlib.compress(raw, 6)) + ch(b'IEND', b''))
            return path
        except Exception as e:      # headless machines without OpenGL
            return 'render failed: %s' % e
