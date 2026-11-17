"""Long-stroke soft spring test. The push trace showed the short stiff spring (slack at the standing knee angle) dumps its
energy in ~16 ms at 54 N - too fast and too strong for the hip/ankle servos to steer. Same energy, softer spring that
keeps pushing until the leg is nearly straight (free angle = release angle - kq) gives a longer, gentler push.
Sweep kq, crouch depth and energy; aligned crouch + hip pitch damping; no optimisation. One simulation at a time."""
import json, os, sys
os.environ.setdefault('SPARROW_VARIANT', 'v7e_stiffcontact')
from lab import WS, VARIANT
import h7_hip_pd as h7

OUT = WS / 'results' / 'h9_softspring' / VARIANT; OUT.mkdir(parents=True, exist_ok=True)
base = dict(h7.base, align=1, kh=0.15, kp_h=1.0, dz_e=-0.026)
for dz in (0.030, 0.045, 0.058):
    for kq in (0.9, 1.3, 1.7):
        for E in (0.30, 0.45, 0.60):
            p = dict(base, dz_c=dz, kq=kq, E=E)
            r = h7.run(p, tag=f'dz{int(dz * 1000)}kq{kq}', quiet=True)
            f = lambda k: f"{r[k]:6.1f}" if r.get(k) is not None else '   n/a'
            print(f"dz_c={dz * 1000:2.0f}mm kq={kq:.1f} E={E:.2f} peakTau={r['peak_tau']:.2f}Nm clr={f('clear')} fwd={f('fwd')} pitchTO={f('pitchTO')} pitchTD={f('pitchTD')} "
                  f"spin={f('spin')}deg/s landF={f('landF')}N tiltAfter={f('tilt_after')} flights={r['flights']} good={r['good']} fall={r['fall']}", flush=True)
(OUT / 'soft_spring_sweep.json').write_text(json.dumps(h7.res, indent=1, default=float))
