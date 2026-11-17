"""Shared simulation loop for the Sparrow game, the self-test and the trainer, so that what is trained is exactly what
is played. Loads only the model copies in this workspace (via src/lab.py). Forward is -Y in the model.
Two modes: 'ground' (pilot.py: walk, hop, jump) and 'air' (flight.py: free-body flapping flight, experimental)."""
import json, math, os, sys
from pathlib import Path

GAME = Path(__file__).resolve().parent
WS = GAME.parent
_sk = json.loads((GAME / 'skills.json').read_text())
os.environ.setdefault('SPARROW_VARIANT', _sk.get('variant', 'v7e_bispring'))
os.environ.setdefault('OMP_NUM_THREADS', '1')
sys.path.insert(0, str(WS / 'src'))
import numpy as np                                     # noqa: E402
import mujoco                                          # noqa: E402
from lab import Lab, VARIANT, FALL_TILT_DEG, FALL_COM_Z   # noqa: E402
from pilot import Pilot, load_skills                   # noqa: E402
from flight import FlightRig, FlightPilot              # noqa: E402

_man = WS / 'model' / VARIANT / 'GAME_WORLD_MANIFEST.json'
SPAWNS = json.loads(_man.read_text())['spawns'] if _man.exists() else {'start': (0.0, 0.0, 0.0)}


class World:
    def __init__(self):
        self.lab = Lab(); self.pilot = None; self.i = 0
        lab = self.lab; m = lab.m
        # everything named env_* that does not move is ground the bird can stand on; env_ball* are loose objects
        self.ground = {lab.floor}; self.loose = set()
        for g in range(m.ngeom):
            n = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g) or ''
            if n.startswith('env_ball'): self.loose.add(g)
            elif n.startswith('env_') and m.geom_conaffinity[g]: self.ground.add(g)
        if len(self.ground) > 1 or self.loose:
            lab.contact_state = self._contact_state
        self.rig = None

    def _contact_state(self):
        """Same as Lab.contact_state but any static world object counts as ground, and loose balls are ignored."""
        lab = self.lab; d, m = lab.d, lab.m; out = {}; f6 = np.zeros(6)
        for i in range(d.ncon):
            c = d.contact[i]
            if c.geom1 in self.ground: g = c.geom2
            elif c.geom2 in self.ground: g = c.geom1
            else: continue
            if g in self.loose or g in self.ground: continue
            mujoco.mj_contactForce(m, d, i, f6); fn = float(f6[0])
            rec = out.setdefault(g, {'fn': 0.0, 'ft': np.zeros(2), 'mom': np.zeros(3), 'n': 0, 'pen': 0.0})
            rec['n'] += 1; rec['pen'] = min(rec['pen'], float(c.dist))
            if fn > 0:
                fr = np.array(c.frame).reshape(3, 3); rec['fn'] += fn; rec['ft'] += (f6[1] * fr[1] + f6[2] * fr[2])[:2]; rec['mom'] += fn * np.array(c.pos)
        return out

    # ------------------------------------------------------------------ reset / spawn
    def reset(self, skills=None, case=None, spawn='start'):
        lab = self.lab
        if self.rig is not None: self.rig.enable(False)
        self.skills = skills or load_skills(); self.case = dict(case or {})
        lab.reset(dict(self.case))
        sx, sy, sz = SPAWNS.get(spawn, (0.0, 0.0, 0.0)) if isinstance(spawn, str) else spawn
        lab.d.qpos[0] += sx; lab.d.qpos[1] += sy; lab.d.qpos[2] += sz; mujoco.mj_forward(lab.m, lab.d)
        lab.ground_z = float(sz)                          # ground height under the spawn (brains see height above it)
        self.pilot = Pilot(self.skills); self.pilot.start(lab)
        self.rig = FlightRig(lab, self.skills.get('flight')); self.fpilot = FlightPilot(self.rig)
        self.corrector = self.brain_getup = None
        if self.skills.get('brain'):                       # trained brains, only if switched on in skills.json
            sys.path.insert(0, str(GAME / 'brain')); import ingame
            b = ingame.load(self.skills['brain'], 'recover'); self.corrector = ingame.WalkCorrector(b, lab) if b else None
            self.brain_getup = ingame.load(self.skills['brain'], 'getup')
        self.mode = 'ground'; self.air_ticks = 0; self.flight_time = 0.0; self.flights = 0; self.max_height = 0.0
        self.i = 0; self.fallen = None; self.air = False; self.air_t0 = 0.0; self.best_clear = 0.0; self.last_clear = 0.0
        self.x0 = float(lab.d.qpos[0]); self.y0 = float(lab.d.qpos[1]); self.speed = 0.0; self._py = self.y0; self._px = self.x0; self.top_speed = 0.0
        self.landed = 0; self.row = None
        return self

    def launch(self, height=6.0, speed=7.0):
        """Assisted launch: put the bird in the air at flying speed (as if thrown)."""
        from flight import air_start
        heading = self.heading_deg; x, y = float(self.lab.d.qpos[0]), float(self.lab.d.qpos[1])
        air_start(self.lab, self.rig, height, speed, 5.0, self.case)
        self.lab.d.qpos[0] += x; self.lab.d.qpos[1] += y; mujoco.mj_forward(self.lab.m, self.lab.d)
        self.fpilot.start(self.lab); self.mode = 'air'; self.fallen = None; self.flights += 1; self.air_ticks = 0

    def _to_air(self):
        s = self.lab.s
        for S in 'LR': self.lab.d.ctrl[s.act[f'act_wing_{S}_sweep']] = 0.0          # open the wings
        self.rig.enable(True); self.fpilot.start(self.lab); self.mode = 'air'; self.flights += 1

    def _to_ground(self):
        lab = self.lab; self.rig.enable(False)
        q = lab.s.info['poses']['stand_wings_stowed']['q']
        for name, i in lab.s.act.items():
            if name.startswith('act_wing') or name.startswith('act_tail'): lab.d.ctrl[i] = q.get(name[4:], 0.0)
        self.pilot = Pilot(self.skills); self.pilot.start(lab); self.pilot.t_mode = self.t; self.pilot.rest = 1.0; self.mode = 'ground'

    @property
    def t(self):
        return float(self.lab.d.time)

    # ------------------------------------------------------------------ one 100 Hz tick
    def start_getup(self):
        """The bird is down: try to get up with the get-up controller instead of ending the run."""
        from getup import GetUp
        if self.mode == 'air': self.rig.enable(False)
        if self.brain_getup is not None:
            import ingame; self.gu = ingame.BrainGetUp(self.brain_getup, self.lab, self.skills.get('getup', {}).get('give_up_s', 8.0))
        else:
            self.gu = GetUp(self.lab, self.skills.get('getup'))
        self.gu.start(self.lab, self.t)
        self.mode = 'getup'; self.fallen = None; self.getups = getattr(self, 'getups', 0) + 1; self.gu_stage = 'start'

    def _getup_tick(self):
        lab = self.lab
        for _ in range(10):
            if self.i % 10 == 0: self.gu_stage = self.gu(lab, self.t)
            lab.e.step(); self.i += 1
        r = lab.measure(); self.row = r
        if self.gu_stage == 'stood':
            self.pilot = Pilot(self.skills); self.pilot.start(lab); self.pilot.t_mode = self.t; self.pilot.rest = 0.8; self.mode = 'ground'
            self.got_up = getattr(self, 'got_up', 0) + 1
        elif self.t - self.gu.t0 > self.gu.p['give_up_s']:
            self.fallen = self.t; self.getup_failed = True
        return r

    def tick(self):
        lab = self.lab
        if self.mode == 'getup':
            return self._getup_tick()
        if self.mode == 'air':
            p = self.pilot; fp = self.fpilot
            fp.throttle = self.rig.p['glide_land_throttle'] if p.speed < -0.1 else 0.5 + 0.5 * max(0.0, min(1.0, p.speed))      # never fully stop flapping: the glide is not stable
            fp.turn = p.turn; fp.landing = p.speed < -0.1
            if fp.landing:                                # feet forward, ready to take the touchdown
                import legs as _legs
                q = _legs.leg_targets(self.pilot.p0[0] + self.rig.p['land_dy'], self.pilot.p0[1])
                lab.d.ctrl[lab.ids] = np.clip(np.concatenate([q, q]), lab.m.actuator_ctrlrange[lab.ids, 0], lab.m.actuator_ctrlrange[lab.ids, 1])
            z = float(lab.d.qpos[2])
            # assists: more power while banking (a turn costs height) and when low and sinking
            fp.throttle = min(1.0, fp.throttle + 0.35 * abs(p.turn) + (0.4 if (z < 2.5 and lab.d.qvel[2] < 0 and p.speed >= -0.1) else 0.0))
            fp.pitch_cmd = 1.0 if (z < 0.9 and lab.d.qvel[2] < 0 and self.max_height < 3.0 + 99 * (z < 0.9)) else 0.0      # flare close to the floor
            for _ in range(10):
                if self.i % 10 == 0: fp(lab, self.t)
                lab.e.step(); self.i += 1
            self.flight_time += 0.01
        else:
            p = self.pilot
            for _ in range(10):
                if self.i % 10 == 0:
                    p(lab, self.t)
                    if self.corrector is not None and p.mode in self.skills.get('brain', {}).get('recover_modes', ['walk', 'stand']): self.corrector.apply(lab, p)
                p.substep(lab); lab.e.step(); self.i += 1
        r = lab.measure(); self.row = r
        fz = r['fz_L'] + r['fz_R']; touching = fz > 0 or r['fz_other'] > 0
        clear = min(r['clear_min_L'], r['clear_min_R']) * 1000
        x, y, z = (float(v) for v in lab.d.qpos[:3])
        v = math.hypot(x - self._px, y - self._py) / 0.01; self.speed += 0.05 * (v - self.speed); self._px, self._py = x, y
        self.top_speed = max(self.top_speed, self.speed); self.max_height = max(self.max_height, z)
        if self.mode == 'air':
            self.air_ticks += 1
            if touching and self.air_ticks > 30:
                if r['tilt_deg'] > FALL_TILT_DEG or r['fz_other'] > 0: self.fallen = r['t']; self.rig.enable(False)
                else: self._to_ground()
            return r
        if not touching and clear > 2:
            if not self.air: self.air = True; self.air_t0 = r['t']; self.last_clear = 0.0
            self.last_clear = max(self.last_clear, clear)
            # fell off an edge (not a hop): open the wings and fly
            if r['t'] - self.air_t0 > 0.12 and self.pilot.mode != 'hop' and self.skills.get('flight', {}).get('auto', True):
                self._to_air(); self.air = False; return r
        elif self.air:
            self.air = False
            if self.last_clear >= 15: self.landed += 1; self.best_clear = max(self.best_clear, self.last_clear)
        if z > 1.5 and r['tilt_deg'] > 30 and self.skills.get('flight', {}).get('auto', True):
            self._to_air(); self.air = False; return r            # tipping off a high edge: open the wings and fly
        if self.fallen is None and (r['tilt_deg'] > FALL_TILT_DEG or r['fz_other'] > 0 or (r['com_z'] < FALL_COM_Z and touching)):
            self.fallen = r['t']
        return r

    @property
    def heading_deg(self):
        """Which way the body points, degrees, positive = turned to its left."""
        R = self.lab.d.xmat[1].reshape(3, 3)
        return math.degrees(math.atan2(-R[0, 1], R[1, 1]))

    @property
    def forward_mm(self):
        return -(float(self.lab.d.qpos[1]) - self.y0) * 1000

    @property
    def left_mm(self):
        return (float(self.lab.d.qpos[0]) - self.x0) * 1000

    def run_script(self, script, skills=None, case=None, stop_on_fall=True, spawn='open'):
        """script = list of (duration_s, {'speed':..,'turn':..,'do': 'hop'|'jump'|...,'launch': (height, speed)}). Returns a summary dict."""
        self.reset(skills, case, spawn); segs = []
        for dur, cmd in script:
            self.pilot.speed = cmd.get('speed', 0.0); self.pilot.turn = cmd.get('turn', 0.0)
            if cmd.get('do'): self.pilot.request(cmd['do'])
            if cmd.get('launch'): self.launch(*cmd['launch']); self.pilot.speed = cmd.get('speed', 0.0); self.pilot.turn = cmd.get('turn', 0.0)
            f0, l0, t0, n0 = self.forward_mm, self.left_mm, self.t, self.landed; self.last_clear = 0.0; peak = 0.0; tilt = 0.0
            for _ in range(int(round(dur * 100))):
                self.pilot.speed = cmd.get('speed', 0.0); self.pilot.turn = cmd.get('turn', 0.0)
                r = self.tick(); peak = max(peak, self.last_clear); tilt = max(tilt, r['tilt_deg'])
                if self.fallen is not None and stop_on_fall: break
            segs.append({'cmd': cmd, 'dur': self.t - t0, 'forward_mm': self.forward_mm - f0, 'left_mm': self.left_mm - l0, 'mm_per_s': (self.forward_mm - f0) / max(self.t - t0, 1e-6),
                         'landed': self.landed - n0, 'clear_mm': peak, 'max_tilt': tilt, 'mode_end': self.pilot.mode if self.mode == 'ground' else 'flying',
                         'height_m': float(self.lab.d.qpos[2]), 'heading': self.heading_deg})
            if self.fallen is not None and stop_on_fall: break
        return {'fallen': self.fallen, 't': self.t, 'forward_mm': self.forward_mm, 'left_mm': self.left_mm, 'landed': self.landed, 'segments': segs,
                'flight_time': self.flight_time, 'max_height': self.max_height}
