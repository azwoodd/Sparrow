"""Tower take-off: stand 1 s on the 12 m tower, walk off the front edge, then fly (cruise input) until 20 s.
   python t_tower.py [brain] [flight.key=val ...]"""
import sys, copy
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from world import World, load_skills
base = load_skills(); w = World()
use_brain = 'brain' in sys.argv[1:]
over = {a.split('=')[0]: float(a.split('=')[1]) for a in sys.argv[1:] if '=' in a}
C = [({}, 'nominal'), ({'mass': 1.07}, 'mass +7%'), ({'mass': 0.93}, 'mass -7%'), ({'friction': 0.6}, 'friction 0.6'), ({'com_shift': -0.003}, 'balance 3 mm back')]
sk = copy.deepcopy(base); sk['flight'].update(over)
if not use_brain: sk['brain'] = None
out = []; ok = 0
for c, n in C:
    w.reset(sk, c, 'tower'); low = 99
    while w.t < 20 and w.fallen is None:
        w.pilot.speed = (0.5 if w.t > 1.0 else 0.0) if w.mode != 'air' else 0.3; w.pilot.turn = 0.0; w.tick()
        if w.mode == 'air': low = min(low, float(w.lab.d.qpos[2]))
    ok += w.fallen is None
    out.append(f'{n}: {"flying" if w.fallen is None else "crashed %.1fs" % w.fallen} (lowest {low:.1f} m)')
print(f'{"brain" if use_brain else "no brain"} {over or ""} {ok}/5 | ' + ' | '.join(out), flush=True)
