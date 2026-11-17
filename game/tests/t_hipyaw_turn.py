"""Hip-twist turning on the v7e_game_hipyaw copy, heading tracked every tick (no wrap-around ambiguity).
   python t_hipyaw_turn.py "hip_yaw_amp=0.12 | hip_yaw_amp=0.18" """
import copy, math, os, sys
from pathlib import Path
os.environ['SPARROW_VARIANT'] = 'v7e_game_hipyaw'
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import numpy as np
from world import World, VARIANT
from train import put
import json

C = [{}, {'friction': 0.6}, {'mass': 1.07}]
w = World(); base = json.loads((Path(__file__).resolve().parents[1] / 'skills_hipyaw.json').read_text()); print('model', VARIANT)


def turn_run(sk, case, speed, tn, dur=5.0):
    w.reset(sk, case); hs = []
    while w.t < 1.0 + dur and w.fallen is None:
        w.pilot.speed = speed if w.t > 1 else 0.0; w.pilot.turn = tn if w.t > 1 else 0.0; w.tick(); hs.append(math.radians(w.heading_deg))
    return w.fallen is None, math.degrees(np.unwrap(hs)[-1] - np.unwrap(hs)[0])


for grp in (' '.join(sys.argv[1:]).split('|') if len(sys.argv) > 1 else ['']):
    sk = copy.deepcopy(base)
    for a in grp.split(): put(sk, a.split('=')[0], float(a.split('=')[1]))
    out = []
    for label, speed in (('walking', 0.5), ('slow walk', 0.2), ('LEFT/RIGHT only', 0.0)):
        for tn in (1.0, -1.0):
            r = [turn_run(sk, c, speed, tn) for c in C]
            out.append(f"{label:16} {'left ' if tn > 0 else 'right'}: turned {[round(x[1]) for x in r]} deg in 5 s (+ = left), upright {sum(x[0] for x in r)}/3")
    print(f'--- {grp.strip() or "current"}\n    ' + '\n    '.join(out), flush=True)
