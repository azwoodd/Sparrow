"""Hop/jump pressed while walking (no stop): distance gained, clearance, and whether it lands standing. 5 conditions."""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from world import World

C = [{}, {'friction': 0.6}, {'friction': 1.4}, {'mass': 1.07}, {'com_shift': -0.003}]
w = World()
for slot in (sys.argv[1:] or ['hop', 'hop_far', 'jump', 'high_jump']):
    res = []
    for c in C:
        r = w.run_script([(1, {}), (3, {'speed': 0.5}), (0.01, {'speed': 0.5, 'do': slot}), (3.5, {'speed': 0.5}), (1.5, {})], case=c)
        s = r['segments']; hop = s[2:4] if len(s) > 3 else s[2:]
        res.append((r['fallen'] is None, sum(x['forward_mm'] for x in hop), max((x['clear_mm'] for x in hop), default=0), r['landed']))
    print(f'{slot:10} while walking: {sum(x[0] for x in res)}/5 stayed up; clearance {[round(x[2]) for x in res]} mm; landed {[x[3] for x in res]}', flush=True)
