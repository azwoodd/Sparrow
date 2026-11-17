"""Sparrow - play it like a game. Real simulated physics (MuJoCo) driven through the same pilot the trainer uses.

Keyboard (click the 3D window first)            Gamepad (Xbox-style, optional)
  UP          walk forward (hold)                 left stick up     walk ... push fully = fast walk
  SHIFT + UP  fast walk                           left stick left/right  steer
  LEFT/RIGHT  steer (hold)                        A  hop      X  far hop      B  jump      Y  high jump
  SPACE       hop        SHIFT + SPACE  far hop   START  reset
  ENTER       jump       SHIFT + ENTER  high jump
  R reset     C camera (chase / side)   ESC or close the window to quit
  HOME  go to the top of the launch tower (LB)      END  assisted launch into the air (RB)
  In the air: UP = flap harder (climb), no key = cruise, DOWN = descend, LEFT/RIGHT = bank and turn

  python play.py [--friction 1.0] [--mass 1.0] [--skills other_skills.json]
Backwards walking and turning on the spot exist as experimental skills (off by default, see skills.json)."""
import argparse, ctypes, json, math, sys, time
from pathlib import Path

ap = argparse.ArgumentParser()
ap.add_argument('--friction', type=float, default=1.0); ap.add_argument('--mass', type=float, default=1.0)
ap.add_argument('--skills', default=None); ap.add_argument('--autoquit', type=float, default=0.0); ap.add_argument('--bot', action='store_true')
args = ap.parse_args()
if args.skills:                                  # a skills file can choose its own model copy (e.g. the hip-twist prototype)
    import os as _os
    _v = json.loads(Path(args.skills).read_text()).get('variant')
    if _v: _os.environ['SPARROW_VARIANT'] = _v

import mujoco, mujoco.viewer                # noqa: E402
from world import World, VARIANT, load_skills   # noqa: E402

user32 = ctypes.windll.user32 if sys.platform == 'win32' else None
VK = {'up': 0x26, 'down': 0x28, 'left': 0x25, 'right': 0x27, 'shift': 0x10, 'space': 0x20, 'enter': 0x0D, 'r': 0x52, 'c': 0x43, 'home': 0x24, 'end': 0x23}


def window_focused():
    if not user32: return False
    h = user32.GetForegroundWindow(); buf = ctypes.create_unicode_buffer(256); user32.GetWindowTextW(h, buf, 256)
    return 'mujoco' in buf.value.lower() or 'sparrow' in buf.value.lower()


def key(name):
    return bool(user32 and user32.GetAsyncKeyState(VK[name]) & 0x8000)


class Pad:
    """Xbox-style controller through Windows XInput (no extra packages). Silent if none is plugged in."""
    class _State(ctypes.Structure):
        _fields_ = [('packet', ctypes.c_ulong), ('buttons', ctypes.c_ushort), ('lt', ctypes.c_ubyte), ('rt', ctypes.c_ubyte),
                    ('lx', ctypes.c_short), ('ly', ctypes.c_short), ('rx', ctypes.c_short), ('ry', ctypes.c_short)]

    def __init__(self):
        self.dll = None
        for n in ('xinput1_4.dll', 'xinput1_3.dll', 'xinput9_1_0.dll'):
            try: self.dll = ctypes.windll.LoadLibrary(n); break
            except OSError: pass
        self.prev = 0

    def read(self):
        if not self.dll: return None
        s = Pad._State()
        if self.dll.XInputGetState(0, ctypes.byref(s)) != 0: return None
        dz = lambda v: 0.0 if abs(v) < 0.2 else (abs(v) - 0.2) / 0.8 * (1 if v > 0 else -1)
        pressed = s.buttons & ~self.prev; self.prev = s.buttons
        return {'speed': dz(s.ly / 32767.0), 'turn': -dz(s.lx / 32767.0), 'a': pressed & 0x1000, 'b': pressed & 0x2000, 'x': pressed & 0x4000, 'y': pressed & 0x8000, 'start': pressed & 0x0010, 'lb': pressed & 0x0100, 'rb': pressed & 0x0200}


def main():
    skills = load_skills(args.skills) if args.skills else load_skills()
    case = {'friction': args.friction, 'mass': args.mass}
    w = World().reset(skills, case); pad = Pad(); prev = {}; chase = True; botted = False; msg = ''; msg_t = 0.0; fell_wall = None; resets = 0
    print(__doc__); print(f'model: {VARIANT}   friction x{args.friction}   mass x{args.mass}   controller: {"found" if pad.read() else "none (keyboard)"}')

    def edge(name, down):
        e = down and not prev.get(name, False); prev[name] = down; return e

    with mujoco.viewer.launch_passive(w.lab.m, w.lab.d, show_left_ui=False, show_right_ui=False) as v:
        v.cam.distance, v.cam.elevation, v.cam.azimuth = 1.1, -18, -90
        t_start = time.perf_counter(); cam_az = -90.0
        while v.is_running():
            t0 = time.perf_counter()
            if args.autoquit and t0 - t_start > args.autoquit: break
            p = w.pilot; speed = turn = 0.0; do = None; reset = False; go_tower = go_launch = False
            if args.bot:                                   # scripted input, used only to test that the window loop runs
                T = w.t; speed = (1.0 if w.mode == 'air' else 0.5) if T > 1 else 0.0; turn = 1.0 if (w.mode == 'air' and T > 9) else 0.0
                if not botted: go_tower = True; botted = True
            elif window_focused():
                sh = key('shift')
                if key('up'): speed = 1.0 if sh else 0.5
                if key('down'): speed = -0.5
                turn = (1.0 if key('left') else 0.0) - (1.0 if key('right') else 0.0)
                if edge('space', key('space')): do = 'hop_far' if sh else 'hop'
                if edge('enter', key('enter')): do = 'high_jump' if sh else 'jump'
                if edge('r', key('r')): reset = True
                if edge('c', key('c')): chase = not chase
                if edge('home', key('home')): go_tower = True
                if edge('end', key('end')): go_launch = True
            g = pad.read()
            if g and not args.bot:
                if abs(g['speed']) > 0.05: speed = max(g['speed'], -0.5) if g['speed'] < 0 else 0.5 * min(1.0, g['speed'] / 0.6) if g['speed'] < 0.6 else 0.5 + 0.5 * (g['speed'] - 0.6) / 0.4
                if abs(g['turn']) > 0.05: turn = g['turn']
                do = do or ('hop' if g['a'] else 'hop_far' if g['x'] else 'jump' if g['b'] else 'high_jump' if g['y'] else None)
                reset = reset or bool(g['start']); go_tower = go_tower or bool(g['lb']); go_launch = go_launch or bool(g['rb'])
            if reset or (fell_wall and time.perf_counter() - fell_wall > 3.0):
                w.reset(skills, case); fell_wall = None; resets += 1; msg = 'reset'; msg_t = w.t; continue
            if go_tower:
                w.reset(skills, case, spawn='tower'); fell_wall = None; msg = 'on the tower: walk forward off the edge to fly'; msg_t = w.t; continue
            if go_launch:
                w.launch(); fell_wall = None; msg = 'launched'; msg_t = w.t
            p = w.pilot; p.speed, p.turn = speed, turn
            if do: p.request(do); msg = do.replace('_', ' '); msg_t = w.t
            if w.fallen is not None and not getattr(w, 'getup_failed', False) and w.mode != 'getup':
                print(f'  fell at {w.t:.1f} s ({w.mode}) - trying to get up')
                w.start_getup(); msg = 'down - trying to get up (R = reset)'; msg_t = w.t
            if w.fallen is None:
                was = w.mode; w.tick()
                if was == 'getup' and w.mode == 'ground': msg = 'got back up!'; msg_t = w.t
            elif fell_wall is None:
                fell_wall = time.perf_counter(); msg = 'could not get up - resetting in 3 s (R = now)'; msg_t = w.t
                print(f'  fell at {w.t:.1f} s ({w.mode}, height {float(w.lab.d.qpos[2]):.2f} m, flight time so far {w.flight_time:.1f} s)')
            # camera
            d = w.lab.d
            v.cam.lookat[:] = [d.qpos[0], d.qpos[1], max(0.14, float(d.qpos[2]) - 0.02)]; v.cam.distance += 0.05 * ((3.0 if w.mode == 'air' else 1.1) - v.cam.distance)
            want = (-90.0 - w.heading_deg) if chase else 180.0
            cam_az += 0.05 * (((want - cam_az + 180) % 360) - 180); v.cam.azimuth = cam_az
            state = {'stand': 'standing', 'walk': 'fast walk' if abs(p.speed) > 0.75 else 'walking', 'hop': f'{p.hop_name}: {p.phase}'}.get(p.mode, p.mode)
            if w.mode == 'getup':
                state = f'GETTING UP ({w.gu_stage})'
            if w.mode == 'air':
                state = f'FLYING (experimental)\nheight {float(d.qpos[2]):5.1f} m\nflap {w.fpilot.f:3.1f} Hz   {w.rig.power:3.0f} W'
            left = f"{state}\nspeed {w.speed * 1000:4.0f} mm/s\ndistance {w.forward_mm / 1000:5.2f} m\nhops landed {w.landed}\nbest clearance {w.best_clear:3.0f} mm\ntop speed {w.top_speed * 1000:4.0f} mm/s"
            right = msg if w.t - msg_t < 2.0 or fell_wall else ''
            try:
                v.set_texts([(None, mujoco.mjtGridPos.mjGRID_TOPLEFT, left, ''),
                             (None, mujoco.mjtGridPos.mjGRID_TOPRIGHT, right, '')])
            except Exception:
                pass
            v.sync()
            time.sleep(max(0.0, 0.01 - (time.perf_counter() - t0)))
    print(f'session: {w.t:.0f} s simulated, distance {w.forward_mm / 1000:.2f} m, hops landed {w.landed}, resets {resets}')


if __name__ == '__main__':
    main()
