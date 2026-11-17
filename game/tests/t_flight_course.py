"""Score a flight controller setting: 6 s straight, 6 s full left, 6 s full right after an 8 m / 8 m/s launch.
Prints course held, turn rates achieved in each direction, height kept.   python t_flight_course.py key=val ...  [| key=val ...]"""
import copy, math, sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import numpy as np
from world import World, load_skills


def score(w, sk, case=None):
    w.reset(sk, case, 'start'); w.launch(8.0, 8.0); d = w.lab.d; tr = []; low = 99.0
    plan = [(6.0, 0.0), (6.0, 1.0), (6.0, -1.0)]; t0 = w.t; seg = []
    for dur, tn in plan:
        cs = []; tend = w.t + dur
        while w.t < tend and w.fallen is None:
            w.pilot.speed = 0.3; w.pilot.turn = tn; w.tick()
            cs.append(math.atan2(float(d.qvel[0]), -float(d.qvel[1]))); low = min(low, float(d.qpos[2]))
        if len(cs) < 150: seg.append(None); break
        c = np.unwrap(np.array(cs)); half = len(c) // 3
        seg.append(math.degrees((c[-1] - c[half]) / ((len(c) - half) * 0.01)))
    ok = w.fallen is None and len(seg) == 3 and None not in seg
    return ok, seg, low, w.t - t0


if __name__ == '__main__':
    w = World(); base = load_skills()
    groups = ' '.join(sys.argv[1:]).split('|') if len(sys.argv) > 1 else ['']
    for g in groups:
        sk = copy.deepcopy(base); kv = {a.split('=')[0]: float(a.split('=')[1]) for a in g.split()}
        case = {k: kv.pop(k) for k in ('mass', 'friction', 'com_shift') if k in kv}
        sk['flight'].update(kv); ok, seg, low, T = score(w, sk, case)
        s = ['--' if x is None else f'{x:+6.1f}' for x in seg] + ['--'] * (3 - len(seg))
        print(f'{g.strip() or "current":50} {"OK  " if ok else "FAIL"} after {T:4.1f} s | course rate straight {s[0]}, left {s[1]}, right {s[2]} deg/s | lowest {low:.1f} m', flush=True)
