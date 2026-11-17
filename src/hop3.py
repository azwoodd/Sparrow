"""Hop controller for the hip-and-knee (biarticular) spring variants. Separate file; hop.py and hop2.py are unchanged.

Spring model: one finite-energy, one-sided spring per leg whose stretch follows  u = q_knee + rho * q_hip  (rho = hip pulley
radius / knee pulley radius, signed). It is slack at the standing pose. Torque on the knee: tau_k = -k * max(0, u - u_free);
torque on the hip: rho * tau_k. Energy per leg is exactly E: k = 2E / (u_release - u_free)^2. rho = 0 reproduces the
knee-only spring. With rho < 0 the same spring also extends the hip, which is the torque that cancels take-off spin.
Needs the 'jump_spring_hip_L/R' channels of the v7e_bispring* model copies (ignored if absent).

Realism options (all default OFF so the validated results reproduce exactly):
  eta   spring/cable efficiency: while the spring is shortening (doing work on the leg) its torque is multiplied by eta;
        while it is being stretched (landing) it is not, so 1 - eta of the stored energy is lost as hysteresis/friction.
  w0    servo no-load speed in rad/s. Each leg servo's available torque in its direction of motion falls linearly from the
        stall value at rest to zero at w0 (a DC servo torque-speed line); braking torque is not reduced.
  damp_scale  leg-joint viscous drag multiplier (the model has 0.02 Nm s/rad; the push back-drives the servos at 20-30 rad/s).
  rho_hw, E_scale   the hardware actually built: pulley ratio and stored energy differing from what the controller assumed.
"""
import numpy as np
from hop2 import AlignedHopper


class BiHopper(AlignedHopper):
    def start(self, lab):
        super().start(lab)
        self.armed = [False, False]; self.ku = [0.0, 0.0]; self.u_free = [0.0, 0.0]
        self.hip_act = [lab.act.get('jump_spring_hip_' + S) for S in 'LR']
        self.peak_hip_tau = 0.0; self.max_stretch = 0.0; self.delivered = 0.0
        if not hasattr(lab, '_stall'):
            lab._stall = lab.m.actuator_forcerange[lab.ids].copy()
        lab.m.actuator_forcerange[lab.ids] = lab._stall            # always start from the model's own limits
        self.peak_servo_speed = 0.0
        if not hasattr(lab, '_damp'):
            lab._damp = lab.m.dof_damping[lab.vi].copy()
        # damp_scale: leg-joint viscous drag relative to the model's 0.02 Nm s/rad (geared servos being back-driven fast)
        H0 = self.hops[0][1]
        lab.m.dof_damping[lab.vi] = lab._damp * H0.get('damp_scale', 1.0)
        # --- servo back-drive model (all OFF by default). The push turns the knee at ~30 rad/s, about 3x a hobby servo's
        # no-load speed, so the gear train and motor are driven by the leg:
        #   arm     rotor inertia reflected through the gears, kg m^2 at the joint (model placeholder is 2e-5)
        #   fric    Coulomb (dry) gear friction at the joint, Nm, for a 0.35 Nm servo; scaled by stall torque for the ankle
        #   emf_w0  no-load speed in rad/s: a DC motor's back-EMF gives a drag of stall/w0 per rad/s whenever the motor
        #           circuit is closed (driven or braked). Replaces the placeholder viscous drag.
        #   knee_free  1 = the knee servo is mechanically decoupled during the push (one-way coupling): no drag, inertia or
        #           servo torque on the knee from spring release until take-off.
        if not hasattr(lab, '_arm'):
            lab._arm = lab.m.dof_armature[lab.vi].copy(); lab._fric = lab.m.dof_frictionloss[lab.vi].copy()
        stall = lab._stall[:, 1]
        self.j_arm = np.full(len(lab.vi), H0['arm']) if 'arm' in H0 else lab._arm.copy()
        self.j_fric = H0['fric'] * stall / 0.35 if 'fric' in H0 else lab._fric.copy()
        self.j_damp = stall / H0['emf_w0'] if H0.get('emf_w0') else lab.m.dof_damping[lab.vi].copy()
        lab.m.dof_armature[lab.vi] = self.j_arm; lab.m.dof_frictionloss[lab.vi] = self.j_fric; lab.m.dof_damping[lab.vi] = self.j_damp
        self.knee_freed = False; self.drag_loss = 0.0; self.rotor_ke_takeoff = 0.0
        self.gear_peak = {'push': np.zeros(len(lab.vi)), 'land': np.zeros(len(lab.vi))}
        # --- series-elastic coupling between each leg servo and its joint (sea_k Nm/rad; OFF unless given).
        # The servo (motor, gears, rotor inertia, back-EMF, dry friction) becomes its own state `th` (output-shaft angle);
        # it drives the joint only through a spring sea_k with damping sea_b (Nm s/rad). The servo's position loop closes
        # on its own shaft angle, as a hobby servo's potentiometer does. The torque through the gear train is the spring
        # torque. sea_stop (rad): a hard stop that limits the coupling's deflection (stiff contact beyond it).
        # The coupling torque is sent through the existing leg-servo actuator channels, switched at run time from position
        # servos to plain torque sources (force = 10 * ctrl); the model's own settings are restored at every start.
        if not hasattr(lab, '_gain'):
            lab._gain = lab.m.actuator_gainprm[lab.ids].copy(); lab._bias = lab.m.actuator_biasprm[lab.ids].copy()
        lab.m.actuator_gainprm[lab.ids] = lab._gain; lab.m.actuator_biasprm[lab.ids] = lab._bias
        self.sea_k = H0.get('sea_k', 0.0)
        if self.sea_k:
            self.sea_tgt = lab.d.ctrl[lab.ids].copy()
            self.th = lab.d.qpos[lab.qi].copy(); self.om = np.zeros(len(lab.vi))
            self.sea_b = H0.get('sea_b', 0.01); self.sea_stop = H0.get('sea_stop', 0.0)
            self.sea_kp = lab.m.actuator_gainprm[lab.ids, 0].copy()
            lab.m.dof_armature[lab.vi] = lab._arm; lab.m.dof_frictionloss[lab.vi] = 0.0; lab.m.dof_damping[lab.vi] = 0.002
            lab.m.actuator_gainprm[lab.ids, 0] = 10.0; lab.m.actuator_biasprm[lab.ids, 1] = 0.0
            lab.m.actuator_forcerange[lab.ids, 0] = -10.0; lab.m.actuator_forcerange[lab.ids, 1] = 10.0
            self.sea_defl = 0.0

    def __call__(self, lab, t):
        if self.sea_k:
            lab.d.ctrl[lab.ids] = self.sea_tgt          # the controller sees and writes position targets as usual
        super().__call__(lab, t)
        if self.sea_k:
            self.sea_tgt = lab.d.ctrl[lab.ids].copy()

    def sea_step(self, lab):
        d, m = lab.d, lab.m
        q, v = d.qpos[lab.qi], d.qvel[lab.vi]
        x = self.th - q
        tc = self.sea_k * x + self.sea_b * (self.om - v)
        if self.sea_stop:
            tc = tc + 60.0 * (np.clip(x, -self.sea_stop, self.sea_stop) - x) * -1.0     # stiff stop (60 Nm/rad) beyond the travel
        tm = np.clip(self.sea_kp * (self.sea_tgt - self.th), lab._stall[:, 0], lab._stall[:, 1])
        net = tm - self.j_damp * self.om - tc
        stuck = (np.abs(self.om) < 1e-3) & (np.abs(net) <= self.j_fric)
        acc = (net - self.j_fric * np.where(np.abs(self.om) > 1e-3, np.sign(self.om), np.sign(net))) / self.j_arm
        om_new = np.where(stuck, 0.0, self.om + acc * lab.dt)
        om_new = np.where((np.abs(self.om) > 1e-3) & (om_new * self.om < 0) & (np.abs(net) <= self.j_fric), 0.0, om_new)   # friction stops it
        self.om = om_new; self.th = self.th + self.om * lab.dt
        d.ctrl[lab.ids] = np.clip(tc, -10.0, 10.0) / 10.0
        self.sea_defl = max(self.sea_defl, float(np.abs(x).max()))
        if self.phase in ('thrust', 'absorb', 'recover'):
            key = 'push' if self.phase == 'thrust' else 'land'
            self.gear_peak[key] = np.maximum(self.gear_peak[key], np.abs(tc))
        if self.phase == 'thrust':
            self.drag_loss += float((self.j_damp * self.om ** 2 + self.j_fric * np.abs(self.om)).sum()) * lab.dt
            self.rotor_ke_takeoff = float((0.5 * self.j_arm * self.om ** 2).sum())

    def substep(self, lab):
        d, m = lab.d, lab.m
        H = self.hops[min(self.i, len(self.hops) - 1)][1]
        if self.sea_k:
            self.sea_step(lab)
        if H.get('knee_free') and not self.sea_k:
            # knee_free = 1: (first version) the servo and its rotor come back the instant the feet leave the ground. That adds
            #   the rotor's kinetic energy from nowhere while the knee is still extending at ~30 rad/s - kept only so the
            #   13 Nov results reproduce.
            # knee_free = 2: one-way coupling. The knee runs ahead of the servo horn; it stays free (a weak return spring,
            #   `knee_ret` Nm, default 0.05, pulls it back towards the horn) until it has stopped extending, and only then
            #   does the servo and its rotor load the joint again. No energy is created at re-engagement.
            mode = H['knee_free']; ret = H.get('knee_ret', 0.05)
            if mode == 1:
                free = self.phase == 'thrust'
            elif mode == 3:
                # knee stays off the servo through touchdown: the one-sided spring takes the landing instead of the gear
                # train, and the servo picks the knee up again once the landing has been absorbed and the knee is slow
                slow = all(abs(d.qvel[lab.vi[k]]) < 1.0 for k in (1, 4))
                free = self.phase in ('thrust', 'flight') or (self.knee_freed and self.phase == 'absorb' and not slow)
            else:
                still_ext = any(d.qvel[lab.vi[k]] < -0.5 for k in (1, 4))
                free = self.phase == 'thrust' or (self.knee_freed and self.phase == 'flight' and still_ext)
            if free != self.knee_freed:
                for k in (1, 4):
                    v = lab.vi[k]
                    m.dof_armature[v] = lab._arm[k] if free else self.j_arm[k]
                    m.dof_frictionloss[v] = 0.0 if free else self.j_fric[k]
                    m.dof_damping[v] = 0.0 if free else self.j_damp[k]
                    m.actuator_forcerange[lab.ids[k]] = ((0.0, 0.0) if mode == 1 else (-ret, ret)) if free else lab._stall[k]
                self.knee_freed = free
        # torque carried by each servo's gear train = motor torque + back-EMF/viscous drag + dry friction + rotor inertia
        # reaction (order: hip, knee, ankle, left then right). Peaks kept separately for the push and for the landing.
        if self.phase in ('thrust', 'absorb', 'recover') and not self.sea_k:
            v = d.qvel[lab.vi]
            tg = np.abs(d.actuator_force[lab.ids] - m.dof_damping[lab.vi] * v - m.dof_frictionloss[lab.vi] * np.sign(v) - m.dof_armature[lab.vi] * d.qacc[lab.vi])
            key = 'push' if self.phase == 'thrust' else 'land'
            self.gear_peak[key] = np.maximum(self.gear_peak[key], tg)
        if self.phase == 'thrust' and not self.sea_k:
            v = d.qvel[lab.vi]
            self.drag_loss += float(((m.dof_damping[lab.vi] * v * v) + m.dof_frictionloss[lab.vi] * np.abs(v)).sum()) * lab.dt
            self.rotor_ke_takeoff = float((0.5 * m.dof_armature[lab.vi] * v * v).sum())
        rho = H.get('rho_hw', H.get('rho', 0.0)); eta = H.get('eta', 1.0)
        for k, S in enumerate('LR'):
            tk = th = 0.0
            if self.spring_on[k]:
                qk, qh = d.qpos[lab.qi[3 * k + 1]], d.qpos[lab.qi[3 * k]]
                u = qk + rho * qh
                if not self.armed[k]:
                    self.u_free[k] = self.base[3 * k + 1] + rho * self.base[3 * k]
                    E = H['E'] * H.get('E_scale', 1.0) * (1 + (H['dE'] if k == 0 else -H['dE']))
                    self.ku[k] = 2 * E / max(u - self.u_free[k], 1e-3) ** 2
                    self.max_stretch = max(self.max_stretch, u - self.u_free[k])
                    self.armed[k] = True
                if u > self.u_free[k]:
                    udot = d.qvel[lab.vi[3 * k + 1]] + rho * d.qvel[lab.vi[3 * k]]
                    tk = -self.ku[k] * (u - self.u_free[k]) * (eta if udot < 0 else 1.0); th = rho * tk
                    w = (tk * d.qvel[lab.vi[3 * k + 1]] + th * d.qvel[lab.vi[3 * k]]) * lab.dt
                    self.spring_work += w; self.delivered += max(w, 0.0)
                    self.peak_tau = max(self.peak_tau, -tk); self.peak_hip_tau = max(self.peak_hip_tau, abs(th))
            else:
                self.armed[k] = False
            d.ctrl[lab.act['jump_spring_' + S]] = max(-3.0, tk)
            if self.hip_act[k] is not None:
                d.ctrl[self.hip_act[k]] = float(np.clip(th, -3.0, 3.0))
        w0 = H.get('w0', 0.0)
        if w0:
            v = d.qvel[lab.vi]
            self.peak_servo_speed = max(self.peak_servo_speed, float(np.abs(v).max()))
            f = np.clip(1.0 - np.abs(v) / w0, 0.0, 1.0)
            hi = np.where(v > 0, lab._stall[:, 1] * f, lab._stall[:, 1])       # torque in the direction of motion is speed-limited
            lo = np.where(v < 0, lab._stall[:, 0] * f, lab._stall[:, 0])
            m.actuator_forcerange[lab.ids, 0] = lo; m.actuator_forcerange[lab.ids, 1] = hi
            if self.knee_freed:
                m.actuator_forcerange[lab.ids[1]] = 0.0; m.actuator_forcerange[lab.ids[4]] = 0.0

