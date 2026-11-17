"""Landing: launch, cruise, then hold DOWN (land) until touchdown. Reports speed at touchdown and whether it ends standing.
   python t_landing.py "key=val key=val | key=val ..."     each group is one flight setting to try"""
import copy, math, sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from world import World, load_skills


def land(w, sk, h0, case=None):
    w.reset(sk, case, 'start'); w.launch(h0, 8.0); d = w.lab.d; td = None; prev = None
    while w.t < 25 and w.fallen is None:
        w.pilot.speed = 0.3 if w.t < 3 else -1.0; w.pilot.turn = 0.0
        if w.mode == 'air': prev = (math.hypot(float(d.qvel[0]), float(d.qvel[1])), float(d.qvel[2]), -math.degrees(w.lab.e.state()[0]))
        w.tick()
        if w.mode == 'ground' and td is None: td = (w.t, prev)
        if td and w.t > td[0] + 2.5: break
    return td, w.fallen is None and td is not None


if __name__ == '__main__':
    w = World(); base = load_skills()
    for grp in (' '.join(sys.argv[1:]).split('|') if len(sys.argv) > 1 else ['']):
        sk = copy.deepcopy(base); sk['flight'].update({a.split('=')[0]: float(a.split('=')[1]) for a in grp.split()})
        out = []
        for h0 in (3.0, 6.0):
            td, ok = land(w, sk, h0)
            out.append(f'from {h0:.0f} m: ' + (f'touchdown {td[1][0]:.1f} m/s fwd {td[1][1]:+.1f} m/s vert nose {td[1][2]:+.0f} -> {"STANDING" if ok else "FELL"}' if td else 'crashed in the air'))
        print(f'{grp.strip() or "current":62} | ' + ' | '.join(out), flush=True)
