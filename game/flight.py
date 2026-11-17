"""Flight physics and flight pilot for the Sparrow game - EXPERIMENTAL.

What is real here: the bird is a free rigid-body system in MuJoCo. Nothing holds its attitude: wing forces, tail forces,
gravity and inertia decide pitch, roll and yaw, and the wings are driven through their hinge actuators with a torque limit.
What is NOT the current design (all set in skills.json -> "flight", and all still unproven hardware):
  * wing twist: each wing strip feathers in proportion to how fast it is being flapped (leading edge down on the
    downstroke), up to twist_tip_deg at the tip. No twist mechanism exists in the CAD.
  * area_scale: the wing strips act as if their chord were this many times larger (the drawn wing does not change).
  * stroke_deg and flap_torque: wider flap stroke and a stronger flap drive than the model's +-31 deg / 1.5 Nm.
Aerodynamics: the strip model of flight_research/f1_wing_model.py (attached lift with leading-edge suction, stalled
flat plate above ~18 deg, skin friction, body drag). Assumed coefficients; a guide, not a prediction.
The owner's simulator files are not modified: this replaces the simulator object's aero function at run time only."""
import math
import numpy as np
import mujoco

RHO = 1.225
FLIGHT_DEFAULT = dict(area_scale=3.0, stroke_deg=30.0, twist_tip_deg=20.0, twist_vref=1.5, incidence_deg=4.0, cn_stall=1.6, eta_s=0.7, cf=0.02,
                      body_cda=0.008, flap_torque=6.0, flap_kp=60.0, flap_kv=0.6, f_min=1.5, f_max=3.2, mass_scale=1.0,
                      k_pitch=1.2, k_q=0.12, k_roll=0.8, k_p=0.08, pitch_trim_deg=6.0, sweep_gain=0.0, diff_gain=0.5, sweep_trim=0.0, com_shift=0.0, tail_scale=8.0, v_safe=5.5, dive_deg=-25.0, wing_aft=0.015, bank_deg=50.0, turn_rate_deg=60.0, k_course=1.5, k_sw=0.2, k_swd=0.04, sw_max=0.12, flare_h=0.8, flare_deg=40.0, flare_f=2.0, land_dy=-0.03, glide_land_throttle=0.15, v_course=7.0)


class FlightRig:
    def __init__(self, lab, params=None):
        self.lab = lab; self.p = {**FLIGHT_DEFAULT, **(params or {})}; s = lab.s; m = s.m
        name = lambda b: mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, b)
        self.bid = np.array([x[0] for x in s.strips]); self.pos = np.array([x[1] for x in s.strips]); self.area0 = np.array([x[2] for x in s.strips]); self.nrm = np.array([x[3] for x in s.strips])
        self.is_wing = np.array([name(b).startswith('wing_') for b in self.bid]); self.side = np.array([1.0 if '_L_' in name(b) else -1.0 for b in self.bid])
        self.unique = np.unique(self.bid); self.index = np.searchsorted(self.unique, self.bid); self.vel = np.zeros((len(self.unique), 6))
        # spanwise distance of each wing strip from its flap hinge, measured with the wings deployed
        d2 = mujoco.MjData(m)
        for j, q in s.info['poses']['stand_wings_deployed']['q'].items(): d2.qpos[s.jq[j]] = q
        mujoco.mj_forward(m, d2)
        self.r = np.zeros(len(self.bid))
        for S in 'LR':
            h = d2.xanchor[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, f'wing_{S}_flap')]
            for i, b in enumerate(self.bid):
                if self.is_wing[i] and f'_{S}_' in name(b):
                    self.r[i] = abs((d2.xpos[b] + d2.xmat[b].reshape(3, 3) @ self.pos[i])[0] - h[0])
        self.R = float(self.r.max())
        self.flap_j = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, f'wing_{S}_flap') for S in 'LR']
        self.flap_a = [s.act[f'act_wing_{S}_flap'] for S in 'LR']; self.sweep_a = [s.act[f'act_wing_{S}_sweep'] for S in 'LR']
        self.tail_p, self.tail_r = s.act['act_tail_pitch'], s.act['act_tail_roll']
        ids = self.flap_a + self.sweep_a
        self.keep = dict(jr=m.jnt_range[self.flap_j].copy(), cr=m.actuator_ctrlrange[ids].copy(), fr=m.actuator_forcerange[ids].copy(), gp=m.actuator_gainprm[ids].copy(), bp=m.actuator_biasprm[ids].copy(), aero=s.apply_aero)
        self.on = False; self.lift = self.thrust = self.power = 0.0

    # ------------------------------------------------------------------ switch the flight hardware on / off (run time only)
    def enable(self, on=True):
        s = self.lab.s; m = s.m; p = self.p; ids = self.flap_a + self.sweep_a
        if on and not self.on:
            a = math.radians(p['stroke_deg'])
            m.jnt_range[self.flap_j] = (-a - 0.05, a + 0.05)
            for i in self.flap_a:
                m.actuator_ctrlrange[i] = (-a, a); m.actuator_forcerange[i] = (-p['flap_torque'], p['flap_torque'])
                m.actuator_gainprm[i, 0] = p['flap_kp']; m.actuator_biasprm[i, 1] = -p['flap_kp']; m.actuator_biasprm[i, 2] = -p['flap_kv']
            for i in self.sweep_a: m.actuator_forcerange[i] = (-2.0, 2.0)
            s.apply_aero = self.apply
        elif not on and self.on:
            m.jnt_range[self.flap_j] = self.keep['jr']; m.actuator_ctrlrange[ids] = self.keep['cr']; m.actuator_forcerange[ids] = self.keep['fr']
            m.actuator_gainprm[ids] = self.keep['gp']; m.actuator_biasprm[ids] = self.keep['bp']; s.apply_aero = self.keep['aero']
        self.on = on

    # ------------------------------------------------------------------ aerodynamic forces (called by the simulator every step)
    def apply(self):
        s = self.lab.s; m, d = s.m, s.d; p = self.p
        d.qfrc_applied[:] = 0
        k = p['area_scale']; area = np.where(self.is_wing, self.area0 * k, self.area0 * p['tail_scale'])
        S = float(area[self.is_wing].sum()); span = s.info['wing_span_mm'] / 1000; AR = span * span / S; a0 = 2 * math.pi * AR / (AR + 2)
        rot = d.xmat[self.bid].reshape(-1, 3, 3)
        r = np.einsum('nij,nj->ni', rot, self.pos); n = np.einsum('nij,nj->ni', rot, self.nrm)
        for i, b in enumerate(self.unique): mujoco.mj_objectVelocity(m, d, mujoco.mjtObj.mjOBJ_BODY, int(b), self.vel[i], 0)
        Rt0 = d.xmat[1].reshape(3, 3)
        r = r + np.where(self.is_wing, p['wing_aft'], 0.0)[:, None] * Rt0[:, 1][None, :]      # wing mounted further back (body +Y is aft)
        pt = d.xpos[self.bid] + r
        v = self.vel[self.index, 3:] + np.cross(self.vel[self.index, :3], r) - s.wind
        vt = np.zeros(6); mujoco.mj_objectVelocity(m, d, mujoco.mjtObj.mjOBJ_BODY, 1, vt, 0)
        Rt = d.xmat[1].reshape(3, 3); fwd = -Rt[:, 1]
        vbody = vt[3:] + np.cross(vt[:3], pt - d.xpos[1]) - s.wind
        vfl = np.einsum('ij,ij->i', v - vbody, n)                                   # how fast each strip is being flapped along its normal
        th = np.where(self.is_wing, self.side * math.radians(p.get('incidence_diff_deg', 0.0)) - math.radians(p['twist_tip_deg']) * (self.r / self.R) * np.tanh(vfl / p['twist_vref']) - math.radians(p['incidence_deg']), 0.0)
        c = fwd[None, :] - (n @ fwd)[:, None] * n; c /= np.maximum(np.linalg.norm(c, axis=1), 1e-9)[:, None]
        npn = np.cos(th)[:, None] * n + np.sin(th)[:, None] * c; cpn = np.cos(th)[:, None] * c - np.sin(th)[:, None] * n
        u = -v; sp = np.linalg.norm(u, axis=1) + 1e-9; un = np.einsum('ij,ij->i', u, npn); sa = np.clip(np.abs(un) / sp, 0, 1)
        al = np.arcsin(sa); sg = np.clip((al - math.radians(10)) / math.radians(8), 0, 1); sg = sg * sg * (3 - 2 * sg)
        cn = ((1 - sg) * a0 + sg * p['cn_stall']) * sa
        N = 0.5 * RHO * area * cn * sp * sp
        F = (N * np.sign(un))[:, None] * npn + (p['eta_s'] * (1 - sg) * N * np.tan(np.minimum(al, 0.5)))[:, None] * cpn + (0.5 * RHO * area * p['cf'] * sp)[:, None] * (u - un[:, None] * npn)
        for b in self.unique:
            mk = self.bid == b
            mujoco.mj_applyFT(m, d, F[mk].sum(axis=0), np.cross(r[mk], F[mk]).sum(axis=0), d.xpos[b], int(b), d.qfrc_applied)
        vb = vt[3:] - s.wind; D = -0.5 * RHO * p['body_cda'] * np.linalg.norm(vb) * vb
        mujoco.mj_applyFT(m, d, D, np.zeros(3), d.xpos[1], 1, d.qfrc_applied)
        tot = F.sum(axis=0) + D; s.aero_force[:] = tot
        self.lift = float(tot[2]); self.thrust = float(tot @ np.array([fwd[0], fwd[1], 0.0]) / max(math.hypot(fwd[0], fwd[1]), 1e-6))
        self.power = float(sum(abs(d.actuator_force[a] * d.qvel[m.jnt_dofadr[j]]) for a, j in zip(self.flap_a, self.flap_j)))


class FlightPilot:
    """Player commands: throttle 0..1 (flap rate; 0 = glide), pitch -1..1 (+ = nose up), turn -1..1 (+ = left)."""

    def __init__(self, rig):
        self.rig = rig; self.throttle = 0.0; self.pitch_cmd = 0.0; self.turn = 0.0; self.phase = 0.0; self.f = 0.0; self.landing = False; self.flaring = False

    def start(self, lab):
        self.phase = 0.0; self.f = 0.0; self.base = lab.d.ctrl.copy(); self.target = None

    def __call__(self, lab, t):
        p = self.rig.p; d = lab.d; dt = 0.01
        ft = p['f_min'] + self.throttle * (p['f_max'] - p['f_min']) if self.throttle > 0.02 else 0.0
        hag = float(d.qpos[2]) - 0.16                         # height of the torso above standing height (flat floor assumed)
        self.flaring = self.landing and hag < p['flare_h']
        if self.flaring: ft = p['flare_f']
        elif float(np.linalg.norm(d.qvel[:3])) < p['v_safe'] and ft > 0: ft = min(ft, p['f_min'])      # below flying speed: flap gently, let the dive build speed
        self.f += float(np.clip(ft - self.f, -6 * dt, 6 * dt))
        self.phase += 2 * math.pi * self.f * dt
        a = math.radians(p['stroke_deg']) * min(1.0, self.f / max(p['f_min'], 1e-6))
        m = lab.m
        pitch, roll, v = lab.e.state()                       # pitch + = nose DOWN, roll about the forward axis
        V = float(np.linalg.norm(d.qvel[:3]))
        up_deg = p['pitch_trim_deg'] + 6.0 * self.pitch_cmd
        if self.flaring:                                      # landing flare: nose up hard so the wings act as an airbrake
            up_deg = p['flare_deg']
        elif V < p['v_safe']:                                 # too slow to fly: put the nose down and trade height for speed
            up_deg += (p['dive_deg'] - up_deg) * min(1.0, (p['v_safe'] - V) / 2.5)
        want = -math.radians(up_deg)                         # nose-up target as a negative pitch
        up = p['k_pitch'] * (pitch - want) + p['k_q'] * v[0]
        # course hold: steer on the direction of travel (smooth), not the body's yaw (it swings with every wingbeat).
        # No input = keep the current course, which also cancels the model's own pull to the right. The stick moves the
        # target course at turn_rate_deg; the bird banks towards it.
        vx, vy = float(d.qvel[0]), float(d.qvel[1]); course = math.atan2(vx, -vy)          # + = to the bird's start-left
        if math.hypot(vx, vy) < p['v_course']:              # too slow for the course to mean anything: wings level
            self.target = None
        elif not hasattr(self, 'target') or self.target is None: self.target = course
        if abs(self.turn) > 0.05 and self.target is not None:
            self.target += math.radians(p['turn_rate_deg']) * self.turn * dt
        err = 0.0 if self.target is None else (self.target - course + math.pi) % (2 * math.pi) - math.pi
        if abs(err) > math.radians(70): self.target = course + math.copysign(math.radians(70), err); err = math.copysign(math.radians(70), err)
        self.course_err = err
        bank = float(np.clip(p['k_course'] * math.degrees(err), -p['bank_deg'], p['bank_deg']))
        roll_want = math.radians(bank)                                                # positive roll = bank to the left (measured)
        self.droll = (roll - getattr(self, 'prev_roll', roll)) / dt; self.prev_roll = roll
        ur = p['k_roll'] * (roll - roll_want) + p['k_p'] * v[1]
        m = lab.m
        for k, S in enumerate('LR'):
            sg = 1.0 if S == 'L' else -1.0
            amp = a * (1.0 + sg * float(np.clip(p['diff_gain'] * ur, -0.3, 0.3)))
            d.ctrl[self.rig.flap_a[k]] = amp * math.sin(self.phase)
            # roll with the wing-sweep (fold) servos: measured as by far the strongest roll control the model has
            # (sweeping the LEFT wing back 0.3 rad rolls the bird ~110 deg right in 1 s; flap-size difference and tail roll
            # do almost nothing). Bank left = sweep the right wing back, and vice versa.
            us = 0.0 if self.target is None else float(np.clip(p['k_sw'] * (roll_want - roll) - p['k_swd'] * self.droll, -p['sw_max'], p['sw_max']))
            back = max(0.0, -us) if S == 'L' else max(0.0, us)
            d.ctrl[self.rig.sweep_a[k]] = float(np.clip(p['sweep_trim'] + back, 0.0, 0.8))
        d.ctrl[self.rig.tail_p] = float(np.clip(up, *m.actuator_ctrlrange[self.rig.tail_p]))
        d.ctrl[self.rig.tail_r] = float(np.clip(ur, *m.actuator_ctrlrange[self.rig.tail_r]))

    def substep(self, lab):
        pass


def air_start(lab, rig, height=3.0, speed=7.0, pitch_up_deg=5.0, case=None):
    """Put the bird in the air with wings open, flying forward (an assisted launch)."""
    lab.reset(dict(case or {}, pose='stand_wings_deployed', com_shift=(case or {}).get('com_shift', 0.0) + rig.p['com_shift']))
    d = lab.d; d.qpos[2] += height
    a = math.radians(pitch_up_deg) / 2                       # nose up = rotation about +X for a bird facing -Y
    q0 = d.qpos[3:7].copy(); dq = np.array([math.cos(a), math.sin(a), 0.0, 0.0]); out = np.zeros(4); mujoco.mju_mulQuat(out, dq, q0); d.qpos[3:7] = out
    d.qvel[:] = 0; d.qvel[1] = -speed
    mujoco.mj_forward(lab.m, d); rig.enable(True)
