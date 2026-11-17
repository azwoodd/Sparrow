"""Live MuJoCo viewer: real simulated physics driven by the controllers in this workspace (nothing is keyframed).

  python view.py [demo] [--variant NAME] [--friction F] [--mass M] [--speed S]
Keys (click the 3D window first):
  SPACE pause/resume   R restart demo   N next demo   P previous demo   - / = slower / faster   F follow camera on/off
  C show/hide red collision shapes      ESC or close window to quit
A status line (time, tilt, per-foot load, valid-step count from the strict detector) prints in the console once a second.
"""
import argparse, os, sys, time

ap = argparse.ArgumentParser()
ap.add_argument('demo', nargs='?', default='walk')
ap.add_argument('--variant', default=os.environ.get('SPARROW_VARIANT', 'v7e_stiffcontact'))
ap.add_argument('--friction', type=float, default=1.0)
ap.add_argument('--mass', type=float, default=1.0)
ap.add_argument('--speed', type=float, default=1.0)
ap.add_argument('--list', action='store_true')
args = ap.parse_args()
os.environ['SPARROW_VARIANT'] = args.variant

import numpy as np                      # noqa: E402
import mujoco, mujoco.viewer            # noqa: E402
from lab import Lab, VARIANT, detect_steps, FALL_TILT_DEG, FALL_COM_Z   # noqa: E402
from demos import DEMOS, ORDER          # noqa: E402
from hop import analyse_hops            # noqa: E402

if args.demo.startswith('bi_') and 'bispring' not in args.variant:
    print('this demo needs the hip-and-knee spring model copy; re-run with:  --variant v7e_bispring'); sys.exit(1)
if args.list or args.demo not in DEMOS:
    print('Available demos:'); [print(f'  {k:14} {v[0]}') for k, v in DEMOS.items()]; sys.exit(0 if args.list else 1)

lab = Lab()
st = {'demo': args.demo, 'paused': False, 'restart': True, 'speed': args.speed, 'follow': True, 'col': False, 'quit': False}
K = mujoco.glfw.glfw if hasattr(mujoco, 'glfw') else None


def on_key(code):
    ch = chr(code) if 0 < code < 256 else ''
    if code == 32: st['paused'] = not st['paused']
    elif ch == 'R': st['restart'] = True
    elif ch == 'N': st['demo'] = ORDER[(ORDER.index(st['demo']) + 1) % len(ORDER)]; st['restart'] = True
    elif ch == 'P': st['demo'] = ORDER[(ORDER.index(st['demo']) - 1) % len(ORDER)]; st['restart'] = True
    elif ch == '-': st['speed'] = max(0.05, st['speed'] / 2)
    elif ch == '=': st['speed'] = min(8.0, st['speed'] * 2)
    elif ch == 'F': st['follow'] = not st['follow']
    elif ch == 'C': st['col'] = not st['col']


print(__doc__)
with mujoco.viewer.launch_passive(lab.m, lab.d, key_callback=on_key, show_left_ui=False, show_right_ui=False) as v:
    v.cam.distance, v.cam.azimuth, v.cam.elevation = 1.3, 135, -15
    ctl = dur = None; i = 0; fall = None; next_print = 0.0
    while v.is_running():
        if st['restart']:
            desc, make = DEMOS[st['demo']]
            ctl, dur = make()
            lab.reset({'friction': args.friction, 'mass': args.mass})
            if hasattr(ctl, 'start'): ctl.start(lab)
            i = 0; fall = None; next_print = 0.0; st['restart'] = False
            print(f"\n=== {st['demo']}: {desc}\n    model variant: {VARIANT} | friction x{args.friction} | mass x{args.mass} | planned {dur:.1f} s")
        t0 = time.perf_counter()
        if not st['paused'] and lab.d.time < dur + 3.0:
            for _ in range(10):                       # one 100 Hz control tick = 10 physics steps
                if i % 10 == 0: ctl(lab, float(lab.d.time))
                if hasattr(ctl, 'substep'): ctl.substep(lab)      # 1 kHz knee-spring law for the hop controller
                lab.e.step(); i += 1
                if i % 5 == 0:
                    row = lab.measure(); lab.rows.append(row)
                    if fall is None and (row['tilt_deg'] > FALL_TILT_DEG or row['com_z'] < FALL_COM_Z):
                        fall = row['t']; print(f'    ** FALL threshold crossed at t={fall:.2f} s **')
            if lab.d.time >= next_print and lab.rows:
                r = lab.rows[-1]; _, s = detect_steps(lab.rows, lab.weight)
                print(f"    t={r['t']:6.2f}s tilt={r['tilt_deg']:5.1f} loadL/R={r['fz_L'] / lab.weight:4.2f}/{r['fz_R'] / lab.weight:4.2f} "
                      f"forward={-(r['com_y'] - lab.rows[0]['com_y']) * 1000:7.1f}mm valid steps={s['valid_steps']} (alternating {s['valid_alternating_steps']})"
                      + (f" hops={sum(h['good'] for h in analyse_hops(lab.rows, lab.weight, settle_s=0.0))}" if hasattr(ctl, 'substep') else '')
                      + ('  FALLEN' if fall else ''))
                next_print += 1.0
        if st['follow']:
            v.cam.lookat[:] = [lab.d.qpos[0], lab.d.qpos[1], 0.12]
        v.opt.geomgroup[3] = 1 if st['col'] else 0
        v.sync()
        time.sleep(max(0.0, 0.01 / st['speed'] - (time.perf_counter() - t0)))

