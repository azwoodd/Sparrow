"""Walk at the first 20 mm step (1.05 m ahead) and see if the bird gets up it and keeps going.
   python t_step.py "walk.lift=0.02 | walk.lift=0.025 walk.f=3"   (skill overrides, | separates tries)"""
import copy, sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from world import World, load_skills
from train import put

C = [({}, 'nominal'), ({'friction': 0.6}, 'friction 0.6'), ({'mass': 1.07}, 'mass +7%')]


def try_step(w, sk, case, extra_script=None):
    r = w.run_script(extra_script or [(1, {}), (12, {'speed': 0.5}), (2, {})], skills=sk, case=case)
    y = -float(w.lab.d.qpos[1]); z = float(w.lab.d.qpos[2])
    on = 'on the plateau' if y > 2.0 else 'on the steps' if y > 0.95 else 'before the step'
    return r['fallen'], y, z, on


if __name__ == '__main__':
    w = World(); base = load_skills()
    for grp in (' '.join(sys.argv[1:]).split('|') if len(sys.argv) > 1 else ['']):
        sk = copy.deepcopy(base)
        for a in grp.split(): put(sk, a.split('=')[0], float(a.split('=')[1]))
        out = []
        for case, name in C:
            fell, y, z, on = try_step(w, sk, case)
            out.append(f'{name}: {"FELL at %.1f s" % fell if fell else "upright"}, {y:.2f} m along, {on}')
        print(f'{grp.strip() or "current":36} | ' + ' | '.join(out), flush=True)
