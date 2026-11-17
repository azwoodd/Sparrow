"""Headless check that every move the game offers works through the live pilot, with no window.
  python selftest.py            one full routine on the nominal robot
  python selftest.py robust     the routine on 5 different conditions (friction, mass, balance)"""
import json, sys
from world import World, VARIANT

ROUTINE = [(1.0, {}), (5.0, {'speed': 0.5}), (2.0, {}), (2.5, {'do': 'hop'}), (4.0, {'speed': 0.5, 'turn': 1.0}), (2.0, {}),
           (4.0, {'speed': 0.5, 'turn': -1.0}), (2.0, {}), (2.8, {'do': 'jump'}), (4.0, {'speed': 1.0}), (2.5, {}), (3.0, {'do': 'high_jump'}),
           (3.0, {'speed': -0.5}), (2.0, {}), (3.0, {'speed': 0.5, 'do': 'hop'}), (2.0, {})]
CONDS = [('nominal', {}), ('friction 0.6', {'friction': 0.6}), ('friction 1.4', {'friction': 1.4}), ('mass +7%', {'mass': 1.07}), ('balance 3 mm back', {'com_shift': -0.003})]


def label(c):
    s = []
    if c.get('do'): s.append(c['do'])
    if c.get('speed'): s.append({0.5: 'walk', 1.0: 'run', -0.5: 'walk backwards'}.get(c['speed'], f"speed {c['speed']}"))
    if c.get('turn'): s.append('left' if c['turn'] > 0 else 'right')
    return ' + '.join(s) or 'stand'


if __name__ == '__main__':
    w = World(); robust = len(sys.argv) > 1 and sys.argv[1] == 'robust'
    print(f'model: {VARIANT}')
    out = {}
    for name, case in (CONDS if robust else CONDS[:1]):
        r = w.run_script(ROUTINE, case=case); out[name] = r
        print(f"=== {name}: {'FELL at %.1f s' % r['fallen'] if r['fallen'] else 'stayed up for the whole routine'} ({r['t']:.0f} s), forward {r['forward_mm']:.0f} mm, hops/jumps landed {r['landed']}")
        if not robust or r['fallen']:
            for s in r['segments']:
                print(f"    {label(s['cmd']):22} {s['dur']:4.1f} s  forward {s['forward_mm']:7.0f} mm ({s['mm_per_s']:5.0f} mm/s)  sideways {s['left_mm']:6.0f} mm  clearance {s['clear_mm']:4.0f} mm  max tilt {s['max_tilt']:4.1f}  -> {s['mode_end']}")
    (w.lab.__class__ and __import__('pathlib').Path(__file__).resolve().parent / ('selftest_robust.json' if robust else 'selftest.json')).write_text(json.dumps(out, indent=1, default=float))
