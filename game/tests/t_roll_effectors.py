"""Open-loop roll authority: in steady flight, apply one roll effector for 1 s on top of the course-hold controller (whose
roll correction is switched off for that second) and measure roll and course change. Tells sign and strength of each."""
import math, sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import numpy as np
from world import World

EFF = [('nothing', {}), ('sweep LEFT wing back 0.3 rad', {'sweepL': 0.3}), ('sweep RIGHT wing back 0.3 rad', {'sweepR': 0.3}),
       ('LEFT wing flaps 30% bigger', {'ampL': 1.3, 'ampR': 0.7}), ('tail roll +0.4 rad', {'tail': 0.4}),
       ('LEFT wing +5 deg incidence, right -5', {'inc': 5.0}), ('LEFT wing +10 deg incidence, right -10', {'inc': 10.0})]
w = World()
for name, e in EFF:
    w.reset(None, None, 'start'); w.launch(8.0, 8.0); fp = w.fpilot; lab = w.lab; d = lab.d
    class P(type(fp)):
        def __call__(self, lab, t):
            super().__call__(lab, t)
            if self.e is not None:
                e2 = self.e; dd = lab.d
                if 'sweepL' in e2: dd.ctrl[w.rig.sweep_a[0]] = e2['sweepL']
                if 'sweepR' in e2: dd.ctrl[w.rig.sweep_a[1]] = e2['sweepR']
                if 'ampL' in e2: dd.ctrl[w.rig.flap_a[0]] *= e2['ampL']; dd.ctrl[w.rig.flap_a[1]] *= e2['ampR']
                if 'tail' in e2: dd.ctrl[w.rig.tail_r] = e2['tail']
    fp.__class__ = P; fp.e = None
    roll0 = course0 = None; rolls = []
    while w.t < 5.0:
        w.pilot.speed = 0.3; w.pilot.turn = 0.0
        on = w.t >= 4.0
        if on and roll0 is None:
            roll0 = math.degrees(lab.e.state()[1]); course0 = math.degrees(math.atan2(float(d.qvel[0]), -float(d.qvel[1])))
            w.rig.p['incidence_diff_deg'] = e.get('inc', 0.0); fp.target = None; fp.e = e
        w.tick()
        if on:
            fp.target = math.atan2(float(d.qvel[0]), -float(d.qvel[1]))            # course hold off: no correction this second
            k = w.i
            if 'sweepL' in e: d.ctrl[w.rig.sweep_a[0]] = e['sweepL']
            if 'sweepR' in e: d.ctrl[w.rig.sweep_a[1]] = e['sweepR']
            if 'ampL' in e:
                d.ctrl[w.rig.flap_a[0]] *= e['ampL']; d.ctrl[w.rig.flap_a[1]] *= e['ampR']
            if 'tail' in e: d.ctrl[w.rig.tail_r] = e['tail']
            rolls.append(math.degrees(lab.e.state()[1]))
    w.rig.p['incidence_diff_deg'] = 0.0
    course1 = math.degrees(math.atan2(float(d.qvel[0]), -float(d.qvel[1])))
    print(f'{name:40}: roll change {rolls[-1] - roll0:+6.1f} deg (+ = left wing down per the course test), course change {course1 - course0:+6.1f} deg (+ = left) in 1 s', flush=True)
