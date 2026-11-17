"""Concept B: keep Sparrow's mass and features, change wing area / flap stroke / jump.  Full jump-to-flight transients
(f1 wing model, f3 trajectory), finer frequency steps, least-power successful take-off reported for each case.
  python f4_concept_b.py grid <area_scale>     stroke x jump energy x jump angle
  python f4_concept_b.py timing                when flapping starts relative to release (incl. already flapping at release)
  python f4_concept_b.py launchmap             minimum launch height / forward speed for one wing at a fixed power-limited flap rate
Wing area is scaled through the chord on the existing 1.42 m span; wing inertia is kept at today's value (i.e. the bigger
wing must be a lighter construction such as film on spars). Twist limited to 50 deg. Same limits as f1/f3."""
import json, math, sys
import numpy as np
import f1_wing_model as w
import f3_jump_to_flight as f3

MASS = 0.529


def least_power(vx0, vz0, vs, t_delay=0.15, freqs=np.arange(3.0, 7.01, 0.5)):
    best = None
    for f in freqs:
        for tw in (35, 50):
            for b0 in (40, 80):
                for be in (15, 30, 45):
                    ok, hmin, end, P, tq = f3.fly(vx0, vz0, float(f), tw, b0, be, 1.5 * vs, t_delay=t_delay)
                    if ok and (best is None or P < best[0]):
                        best = (P, float(f), tw, b0, be, end, tq, hmin)
    return best


def setup(area, phi, mass=MASS):
    f3.set_wing(f3.R_SP, f3.A_SP * area, f3.SPAN_SP, mass, phi, f3.I_SP)
    W = mass * w.G; return math.sqrt(2 * W / (1.225 * w.S_WING * 1.0)), W / w.S_WING


if __name__ == '__main__':
    mode = sys.argv[1]; res = []
    if mode == 'grid':
        area = float(sys.argv[2])
        for phi in (45, 60):
            vs, wl = setup(area, phi)
            print(f'--- 529 g, wing area x{area} (mean chord {117 * area:.0f} mm), stroke +-{phi} deg: wing loading {wl:.1f} N/m2, model stall speed {vs:.1f} m/s', flush=True)
            for E in (5.0, 7.0, 10.0, 13.0):
                for ang in (45, 60):
                    vx0, vz0 = f3.jump(E, MASS, ang); b = least_power(vx0, vz0, vs)
                    rec = {'area': area, 'phi': phi, 'E_J': E, 'angle': ang, 'vx0': vx0, 'vz0': vz0, 'apex_m': vz0 ** 2 / (2 * w.G)}
                    if b:
                        P, f, tw, b0, be, end, tq, hmin = b; rec.update(ok=True, aero_W=P, f=f, twist=tw, torque_Nm=tq, end_vx=end[1], end_h=end[3])
                        print(f'  jump {E:4.1f} J at {ang} deg ({vx0:.1f} fwd, {vz0:.1f} up m/s, apex {rec["apex_m"]:.2f} m): least power {P:5.1f} W at {f:.1f} Hz, twist {tw} deg, pitch {b0}->{be}; peak torque {tq:.1f} Nm per wing; ends {end[1]:.1f} m/s at {end[3]:.1f} m', flush=True)
                    else:
                        rec.update(ok=False); print(f'  jump {E:4.1f} J at {ang} deg ({vx0:.1f} fwd, {vz0:.1f} up m/s, apex {rec["apex_m"]:.2f} m): no sustained flight up to 7 Hz', flush=True)
                    res.append(rec)
        (w.OUT / f'f4_grid_area{int(area * 10)}.json').write_text(json.dumps(res, indent=1, default=float))
    elif mode == 'timing':
        vs, wl = setup(2.0, 60)
        print(f'--- 529 g, wing area x2, stroke +-60 deg, jump 10 J at 60 deg: effect of when flapping starts')
        vx0, vz0 = f3.jump(10.0, MASS, 60)
        for td, label in ((0.0, 'already flapping at release'), (0.1, '0.10 s after release'), (0.2, '0.20 s after release'), (0.3, '0.30 s after release'), (0.5, '0.50 s after release (near the top)')):
            b = least_power(vx0, vz0, vs, t_delay=td)
            if b:
                P, f, tw, b0, be, end, tq, hmin = b; print(f'  {label:38}: least power {P:5.1f} W at {f:.1f} Hz; lowest point after 0.3 s {hmin:+.2f} m; ends {end[1]:.1f} m/s at {end[3]:.1f} m', flush=True)
            else:
                print(f'  {label:38}: no sustained flight up to 7 Hz', flush=True)
            res.append({'t_delay': td, 'ok': bool(b), 'aero_W': b[0] if b else None, 'f': b[1] if b else None})
        (w.OUT / 'f4_timing.json').write_text(json.dumps(res, indent=1, default=float))
    elif mode == 'launchmap':
        vs, wl = setup(2.0, 60); fmax = float(sys.argv[2]) if len(sys.argv) > 2 else 4.5
        print(f'--- 529 g, wing area x2, stroke +-60 deg, flap rate limited to {fmax} Hz: which launch conditions reach sustained flight')
        print('  rows = apex height of the jump, columns = forward speed at release;  number = least aero power W, -- = comes down')
        fw = (0.5, 1.5, 2.5, 3.5, 4.5); print('  apex   ' + ''.join(f'{v:7.1f}' for v in fw) + '   m/s')
        for apex in (0.1, 0.3, 0.5, 0.75, 1.0, 1.5):
            row = []
            for vx0 in fw:
                b = least_power(vx0, math.sqrt(2 * w.G * apex), vs, freqs=np.arange(3.0, fmax + 0.01, 0.5))
                row.append(f'{b[0]:7.0f}' if b else '     --'); res.append({'apex': apex, 'vx0': vx0, 'ok': bool(b), 'aero_W': b[0] if b else None})
            print(f'  {apex:4.2f} m ' + ''.join(row), flush=True)
        (w.OUT / f'f4_launchmap_f{int(fmax * 10)}.json').write_text(json.dumps(res, indent=1, default=float))
