"""Sparrow 'brain': reinforcement-learning environments (gymnasium) on the same model, controllers and skills as the game.

Tasks
  getup    the bird starts lying where a random shove left it (any direction, random weight/grip/balance). The policy
           drives legs, wings (flap + sweep), tail and head directly as position targets. Goal: on its feet, still.
  recover  the existing walking controller walks to random speed/turn commands while the bird is shoved at random. The
           policy adds corrections (residuals) to the six leg joints. Goal: keep walking, never fall.
  step_up  starts in front of the first 20 mm step; walking controller walks forward; the policy adds leg corrections
           and may trigger the jump (once). Goal: get up the steps and keep going.

Physics: 1 kHz MuJoCo, 100 Hz control - identical to the game. Air forces are OFF by default for these ground tasks
(they cost 90% of run time and are negligible below 0.3 m/s with folded wings); every trained brain must be checked
with them ON (evaluate.py does this). Observations are in brain_obs() and are shared with the in-game loader.
"""
import math, os, sys
from pathlib import Path
import numpy as np
import gymnasium as gym
from gymnasium import spaces

GAME = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(GAME))
import mujoco                                          # noqa: E402
from world import World, load_skills                    # noqa: E402

TASKS = ('getup', 'recover', 'step_up')


def brain_joints(lab):
    """Actuators the get-up brain drives directly (order matters - it is the action layout)."""
    s = lab.s; names = ['act_leg_L_thigh', 'act_leg_L_shin', 'act_leg_L_ankle', 'act_leg_R_thigh', 'act_leg_R_shin', 'act_leg_R_ankle',
                        'act_wing_L_flap', 'act_wing_R_flap', 'act_wing_L_sweep', 'act_wing_R_sweep', 'act_tail_pitch', 'act_head_pitch']
    names += [n for n in ('act_leg_L_yaw', 'act_leg_R_yaw') if n in s.act]
    return [s.act[n] for n in names]


def brain_obs(lab, acts, last_action, extra=(), z0=0.0):
    """Body-frame state: gravity direction, angular and linear velocity, torso height, actuated joint angles and speeds,
    foot loads, last action, task extras. z0 = height of the ground the bird stands on (training is at 0)."""
    m, d = lab.m, lab.d
    R = d.xmat[1].reshape(3, 3)
    g = R.T @ np.array([0.0, 0.0, -1.0])
    v = np.zeros(6); mujoco.mj_objectVelocity(m, d, mujoco.mjtObj.mjOBJ_BODY, 1, v, 1)
    jid = m.actuator_trnid[acts, 0]
    q = d.qpos[m.jnt_qposadr[jid]]; qd = d.qvel[m.jnt_dofadr[jid]]
    cs = lab.contact_state()
    fl = [cs.get(lab.foot_geom[S], {'fn': 0.0})['fn'] / max(lab.weight, 1e-6) for S in 'LR']
    return np.concatenate([g, v[:3] * 0.2, v[3:] * 0.5, [(d.qpos[2] - z0) * 5.0], q, qd * 0.1, np.clip(fl, 0, 3), last_action, np.asarray(extra, float)]).astype(np.float32)


class SparrowEnv(gym.Env):
    metadata = {'render_modes': []}

    def __init__(self, task='getup', aero=False, skills=None, seed=None, randomize=True, max_s=None):
        assert task in TASKS
        self.task = task; self.aero = aero; self.randomize = randomize
        self.w = World(); self.lab = self.w.lab; self.sk = skills or load_skills()
        super().reset(seed=seed)
        self.acts = brain_joints(self.lab)
        self.max_steps = int(100 * (max_s or {'getup': 8.0, 'recover': 12.0, 'step_up': 12.0}[task]))
        if task == 'getup':
            na = len(self.acts)
        else:
            na = 6 + (1 if task == 'step_up' else 0)
        self.action_space = spaces.Box(-1.0, 1.0, (na,), np.float32)
        nx = {'getup': 0, 'recover': 2, 'step_up': 3}[task] + 2      # + walking phase sin/cos (zero when not walking)
        nobs = len(brain_obs(self.lab, self.acts, np.zeros(na), np.zeros(nx)))
        self.observation_space = spaces.Box(-np.inf, np.inf, (nobs,), np.float32)
        lo = self.lab.m.actuator_ctrlrange[self.acts, 0]; hi = self.lab.m.actuator_ctrlrange[self.acts, 1]
        self.lo, self.hi = lo, hi

    # ------------------------------------------------------------------ physics
    def _physics(self, n=10):
        lab = self.lab; s = lab.s; d = lab.d
        for _ in range(n):
            if self.task != 'getup': self.w.pilot.substep(lab)
            if self.aero: s.apply_aero()
            else: d.qfrc_applied[:] = 0
            d.xfrc_applied[:] = 0
            if self.push_left > 0:
                d.xfrc_applied[1, :3] = self.push; self.push_left -= 1
            mujoco.mj_step(lab.m, d)

    def _case(self):
        if not self.randomize: return {}
        r = self.np_random
        return {'mass': float(r.uniform(0.92, 1.08)), 'friction': float(r.uniform(0.5, 1.5)), 'com_shift': float(r.uniform(-0.004, 0.004))}

    def _tilt(self):
        return math.degrees(math.acos(float(np.clip(self.lab.d.xmat[1].reshape(3, 3)[2, 2], -1, 1))))

    def _extra(self):
        p = self.w.pilot
        ph = [math.sin(2 * math.pi * p.cyc), math.cos(2 * math.pi * p.cyc)] if (self.task != 'getup' and p.mode == 'walk') else [0.0, 0.0]
        if self.task == 'recover': return [p.speed, p.turn] + ph
        if self.task == 'step_up':
            y = -float(self.lab.d.qpos[1]); return [(0.9 - y) * 5.0, float(self.jumped), p.speed] + ph
        return ph

    # ------------------------------------------------------------------ gym API
    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        w, lab = self.w, self.lab; r = self.np_random
        spawn = 'start' if self.task == 'step_up' else 'open'
        w.reset(self.sk, self._case(), spawn); lab.s.aero_on = self.aero
        self.push = np.zeros(3); self.push_left = 0; self.steps = 0; self.jumped = False; self.held = 0
        self.last = np.zeros(self.action_space.shape[0], np.float32)
        if self.task == 'getup':
            # knock it over: a random horizontal shove, then let it come to rest under the standing controller
            ang = r.uniform(0, 2 * math.pi); F = r.uniform(10.0, 18.0)
            for k in range(300):
                if k % 10 == 0: w.pilot(lab, w.t)
                if k == 100: self.push = np.array([F * math.cos(ang), F * math.sin(ang), 0.0]); self.push_left = 150
                self._physics(1)
            self.target = lab.d.ctrl[self.acts].copy()
        elif self.task == 'recover':
            w.pilot.speed = float(r.choice([0.0, 0.5, 0.5, 1.0])); w.pilot.turn = float(r.uniform(-1, 1)) * (r.random() < 0.5)
            self.next_push = int(r.integers(100, 300)); self.next_cmd = int(r.integers(200, 500))
        else:
            w.pilot.speed = 0.0; self.trigger_at = None
        return brain_obs(lab, self.acts, self.last, self._extra()), {}

    def step(self, action):
        a = np.clip(np.asarray(action, np.float32), -1, 1); w, lab, d = self.w, self.lab, self.lab.d; r = self.np_random
        smooth = float(np.sum((a - self.last) ** 2)); self.last = a; self.steps += 1
        if self.task == 'getup':
            want = self.lo + (a + 1) * 0.5 * (self.hi - self.lo)
            self.target += np.clip(want - self.target, -0.15, 0.15)           # servo speed limit ~15 rad/s
            d.ctrl[self.acts] = self.target
            self._physics(10)
        else:
            if self.task == 'recover':
                if self.steps == self.next_push:
                    ang = r.uniform(0, 2 * math.pi); F = r.uniform(1.0, 4.0)
                    self.push = np.array([F * math.cos(ang), F * math.sin(ang), 0.0]); self.push_left = int(r.integers(100, 300)) // 10
                    self.next_push += int(r.integers(150, 400))
                if self.steps == self.next_cmd:
                    w.pilot.speed = float(r.choice([0.0, 0.5, 0.5, 1.0])); w.pilot.turn = float(r.uniform(-1, 1)) * (r.random() < 0.5)
                    self.next_cmd += int(r.integers(200, 500))
            else:
                w.pilot.speed = 0.5 if self.steps > 100 and -float(d.qpos[1]) < 2.6 else 0.0
                if a[6] > 0.5 and not self.jumped and w.pilot.mode in ('walk', 'stand'):
                    w.pilot.request('jump'); self.jumped = True
            w.pilot(lab, w.t)
            d.ctrl[lab.ids] = np.clip(d.ctrl[lab.ids] + 0.15 * a[:6], lab.m.actuator_ctrlrange[lab.ids, 0], lab.m.actuator_ctrlrange[lab.ids, 1])
            self._physics(10)
        obs = brain_obs(lab, self.acts, self.last, self._extra())
        tilt = self._tilt(); up = float(d.xmat[1].reshape(3, 3)[2, 2]); z = float(d.qpos[2])
        cs = lab.contact_state(); fl = [cs.get(lab.foot_geom[S], {'fn': 0.0})['fn'] / lab.weight for S in 'LR']
        terminated = False; info = {}
        if self.task == 'getup':
            standing = tilt < 15 and min(fl) > 0.2 and np.linalg.norm(d.qvel[:3]) < 0.15
            self.held = self.held + 1 if standing else 0
            # height only counts with weight on the feet: the first version paid for height alone and the brain learned
            # to bounce in the air instead of standing (reward exploit found 17 Nov 2026)
            load = float(sum(fl)); grounded = min(1.0, load / 0.8)
            airborne = not any(rec['fn'] > 0 for rec in cs.values())
            rew = (up + 2.0 * float(np.clip((z - 0.06) / 0.10, 0, 1))) * grounded + (1.0 if standing else 0.0) - (0.5 if airborne else 0.0) - 0.1 * float(np.linalg.norm(d.qvel[:3])) - 0.02 * smooth
            if self.held >= 100: rew += 50.0; terminated = True; info['success'] = True
        else:
            other = any(g not in (lab.foot_geom['L'], lab.foot_geom['R']) and rec['fn'] > 0 for g, rec in cs.items())
            fallen = tilt > 45 or other
            if self.task == 'recover':
                vy = -float(d.qvel[1]); want = 0.16 * w.pilot.speed / 0.5 if w.pilot.speed <= 0.5 else 0.2
                rew = 1.0 - 2.0 * abs(vy - want) - 0.01 * tilt - 0.05 * float(np.sum(a ** 2)) - 0.02 * smooth
            else:
                y = -float(d.qpos[1])
                rew = 5.0 * max(0.0, y - getattr(self, 'best_y', 0.0)) * 10 + (0.5 if (y > 0.95 and z > 0.175) else 0.0) - 0.01 * tilt - 0.02 * smooth
                self.best_y = max(getattr(self, 'best_y', 0.0), y)
                if y > 2.3 and z > 0.2: info['success'] = True
            if fallen: rew -= 20.0; terminated = True; info['fell'] = True
        truncated = self.steps >= self.max_steps
        if truncated and self.task == 'recover' and not terminated: info['success'] = True
        if self.task == 'step_up' and (terminated or truncated): self.best_y = 0.0
        info.setdefault('success', False); info.setdefault('fell', False)
        return obs, float(rew), terminated, truncated, info


def make_env(task, rank=0, aero=False, seed=0):
    def _f():
        os.environ.setdefault('OMP_NUM_THREADS', '1')
        return SparrowEnv(task, aero=aero, seed=seed + 1000 * rank)
    return _f
