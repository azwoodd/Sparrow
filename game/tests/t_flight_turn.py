"""Flight turn radius against bank angle and wing-differential gain."""
import copy, math, sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import numpy as np
from world import World, load_skills

w = World(); sk = load_skills()
CASES = [(b, dg, tn) for b in (20, 35, 50) for dg in (0.25, 0.5) for tn in (1.0,)] if len(sys.argv) < 2 else [tuple(float(x) for x in a.split(',')) for a in sys.argv[1:]]
for bank, dg, tn in CASES:
    s = copy.deepcopy(sk); s['flight'].update(bank_deg=bank, diff_gain=dg)
    w.reset(s, None, 'start'); w.launch(8.0, 7.0); hs = []; xs = []; low = 99.0; x4 = None
    while w.t < 14 and w.fallen is None:
        w.pilot.speed = 0.3; w.pilot.turn = tn if w.t > 4 else 0.0; w.tick()
        if x4 is None and w.t >= 4: x4 = (float(w.lab.d.qpos[0]), float(w.lab.d.qpos[1]))
        if x4 is not None and w.t < 5.5: side = float(w.lab.d.qpos[0]) - x4[0]
        if w.t > 6:
            hs.append(w.heading_deg); xs.append((float(w.lab.d.qpos[0]), float(w.lab.d.qpos[1]))); low = min(low, float(w.lab.d.qpos[2]))
    if len(hs) < 100:
        print(f'bank {bank:.0f} diff {dg} turn {tn:+.0f}: FELL at {w.t:.1f} s', flush=True); continue
    h = np.unwrap(np.radians(hs)); rate = (h[-1] - h[0]) / (len(h) * 0.01)
    v = float(np.mean(np.linalg.norm(np.diff(np.array(xs), axis=0), axis=1)) / 0.01)
    state = 'ok' if w.fallen is None else 'FELL'
    print(f'bank {bank:.0f} diff {dg} turn {tn:+.0f}: {state:4} turn rate {math.degrees(rate):5.1f} deg/s, speed {v:.1f} m/s, radius {v / max(abs(rate), 1e-3):5.1f} m, lowest {low:.1f} m, first 1.5 s of input moved it {side:+.1f} m sideways (+ = to its left)', flush=True)
