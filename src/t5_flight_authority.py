"""In-flight pitch authority of the EXISTING mechanisms. Sparrow is released 0.45 m above the ground at rest (imposed initial
condition, about 0.30 s of free fall) and one mechanism is commanded at t = 0.02 s through its real actuator (torque limits
as modelled, command slew 10 rad/s for legs, 6 rad/s for wings/tail/neck). We read how far the body pitch has changed
0.15 s and 0.25 s later, compared with doing nothing. Positive pitch = nose down. Angular momentum is conserved in the air,
so these are reaction rotations: they shift attitude, they do not remove spin unless the limb keeps moving."""
import json, math, os
os.environ.setdefault('SPARROW_VARIANT', 'v7e_stiffcontact')
import numpy as np, mujoco
from lab import Lab, WS, VARIANT, arr
import legs

OUT = WS / 'results' / 't5_flight_authority' / VARIANT; OUT.mkdir(parents=True, exist_ok=True)
lab = Lab()


class Cmd:
    def __init__(self, leg=None, act=None, rate0=0.0):
        self.leg, self.act = leg, act or {}
    def start(self, lab):
        self.last = lab.d.ctrl[lab.ids].copy(); self.cur = {k: float(lab.d.ctrl[lab.act['act_' + k]]) for k in self.act}
        self.base = self.last.copy()
    def __call__(self, lab, t):
        d = lab.d
        if t >= 0.02:
            tgt = self.base.copy()
            if self.leg:
                for k in range(2):
                    tgt[3 * k:3 * k + 3] += np.array(self.leg)
            tgt = np.clip(tgt, lab.m.actuator_ctrlrange[lab.ids, 0], lab.m.actuator_ctrlrange[lab.ids, 1])
            tgt = np.clip(tgt, self.last - 0.10, self.last + 0.10)
            d.ctrl[lab.ids] = tgt; self.last = tgt
            for k, v in self.act.items():
                self.cur[k] = float(np.clip(v, self.cur[k] - 0.06, self.cur[k] + 0.06)); d.ctrl[lab.act['act_' + k]] = self.cur[k]


def pre(rate):
    def f(lab):
        lab.d.qpos[2] += 0.45; lab.d.qvel[:] = 0; lab.d.qvel[3] = math.radians(rate); mujoco.mj_forward(lab.m, lab.d)
    return f


def run(name, cmd, rate=0.0):
    lab.run(cmd, 0.29, pre_reset=pre(rate), every=1)
    t = arr(lab.rows, 't'); p = arr(lab.rows, 'pitch_deg')
    return {'name': name, 'p15': float(np.interp(0.17, t, p)), 'p25': float(np.interp(0.27, t, p)), 'contact': bool(any(r['fz_L'] + r['fz_R'] + r['fz_other'] > 0 for r in lab.rows))}


ref = run('nothing', Cmd()); res = []
tests = [('hips swing legs forward 0.6 rad', Cmd(leg=[-0.6, 0, 0])), ('hips swing legs back 0.6 rad', Cmd(leg=[0.6, 0, 0])),
         ('hips forward 1.0 rad', Cmd(leg=[-1.0, 0, 0])), ('hips back 1.0 rad', Cmd(leg=[1.0, 0, 0])),
         ('knees flex 0.8 rad (tuck)', Cmd(leg=[0, 0.8, 0])), ('ankles 0.8 rad', Cmd(leg=[0, 0, 0.8])),
         ('tail pitch +35 deg', Cmd(act={'tail_pitch': 0.61})), ('tail pitch -35 deg', Cmd(act={'tail_pitch': -0.61})),
         ('head pitch +30 deg', Cmd(act={'head_pitch': 0.52})),
         ('both wings unfold fully (sweep)', Cmd(act={'wing_L_sweep': 0.0, 'wing_R_sweep': 0.0})),
         ('both wings flap up 31 deg (stowed)', Cmd(act={'wing_L_flap': -0.54, 'wing_R_flap': -0.54})),
         ('both wings flap down 31 deg (stowed)', Cmd(act={'wing_L_flap': 0.54, 'wing_R_flap': 0.54}))]
print(f"reference (no command): pitch {ref['p15']:+.2f} deg at 0.15 s, {ref['p25']:+.2f} deg at 0.25 s")
for name, c in tests:
    r = run(name, c); r['d15'] = r['p15'] - ref['p15']; r['d25'] = r['p25'] - ref['p25']; res.append(r)
    print(f"{name:38} body pitch change {r['d15']:+6.2f} deg after 0.15 s, {r['d25']:+6.2f} deg after 0.25 s" + ('  (touched ground)' if r['contact'] else ''), flush=True)
# what a take-off spin looks like for scale: 5 and 15 mNms of whole-body angular momentum ~ this many deg/s
mujoco.mj_subtreeVel(lab.m, lab.d)
for rate in (60.0, 120.0):
    lab.run(Cmd(), 0.05, pre_reset=pre(rate), every=1); mujoco.mj_subtreeVel(lab.m, lab.d)
    print(f"body spinning at {rate:.0f} deg/s carries {lab.d.subtree_angmom[1][0] * 1000:+.2f} mNms")
    res.append({'name': f'spin_{rate}', 'L_mNms': float(lab.d.subtree_angmom[1][0] * 1000)})
(OUT / 'authority.json').write_text(json.dumps({'reference': ref, 'tests': res}, indent=1, default=float))
