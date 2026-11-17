"""Offscreen contact sheet of a demo (real simulation frames):  python render_frames.py demo out.png t0 t1 n [az el dist]"""
import os, sys, struct, zlib
import numpy as np
os.environ.setdefault('SPARROW_VARIANT', 'v7e_stiffcontact')
import mujoco
from lab import Lab
from demos import DEMOS

demo, out, t0, t1, n = sys.argv[1], sys.argv[2], float(sys.argv[3]), float(sys.argv[4]), int(sys.argv[5])
az, el, dist = (float(x) for x in sys.argv[6:9]) if len(sys.argv) > 8 else (150.0, -12.0, 1.1)
lab = Lab(); ctl, dur = DEMOS[demo][1]()
lab.reset({}); ctl.start(lab) if hasattr(ctl, 'start') else None
W, H = 480, 360
r = mujoco.Renderer(lab.m, H, W)
cam = mujoco.MjvCamera(); cam.type = mujoco.mjtCamera.mjCAMERA_FREE; cam.azimuth, cam.elevation, cam.distance = az, el, dist
opt = mujoco.MjvOption(); opt.geomgroup[3] = 0
times = np.linspace(t0, t1, n); frames = []; i = 0
for tt in times:
    while lab.d.time < tt:
        if i % 10 == 0: ctl(lab, float(lab.d.time))
        lab.e.step(); i += 1
    cam.lookat[:] = [lab.d.qpos[0], lab.d.qpos[1], 0.12]
    r.update_scene(lab.d, cam, opt); frames.append(r.render().copy())
cols = min(n, 5); rows = (n + cols - 1) // cols
sheet = np.zeros((rows * H, cols * W, 3), np.uint8)
for k, f in enumerate(frames):
    sheet[(k // cols) * H:(k // cols + 1) * H, (k % cols) * W:(k % cols + 1) * W] = f
raw = b''.join(b'\x00' + sheet[y].tobytes() for y in range(sheet.shape[0]))
ch = lambda t, d: struct.pack('>I', len(d)) + t + d + struct.pack('>I', zlib.crc32(t + d) & 0xffffffff)
open(out, 'wb').write(b'\x89PNG\r\n\x1a\n' + ch(b'IHDR', struct.pack('>IIBBBBB', sheet.shape[1], sheet.shape[0], 8, 2, 0, 0, 0)) + ch(b'IDAT', zlib.compress(raw, 6)) + ch(b'IEND', b''))
print('wrote', out, 'frames at', np.round(times, 2).tolist())
