"""Full jump-to-flight transients with the f1 wing model, (1) checked against a robot that has physically done it and
(2) applied to Sparrow and to variants of Sparrow.

Reference robot: Li et al., 'Large-sized ornithopters achieve continuous flight through jump-assisted takeoff',
npj Robotics 4, 4 (2026).  Stated in the paper: 180 g (120 g aircraft + 60 g jump mechanism); kite-fabric elliptical wing,
span 0.9 m, chord 0.27 m; ~3 J carbon-fibre bow spring; jump at ~60 deg to ~1 m height with ~2.5 m/s horizontal speed;
flapping becomes evident 0.1-0.2 s after the jump; 8 Hz at take-off, cannot hold height below 6 Hz, 9 Hz at full power;
body pitch 30-50 deg in flight; 32 of 40 outdoor take-offs succeeded.  NOT stated: flap amplitude, wing twist, wing mass.

  python f3_jump_to_flight.py paper      does the model reproduce 'flies at 8 Hz, sinks below 6 Hz'?
  python f3_jump_to_flight.py sparrow    Sparrow and variants, jump energies 3-10 J
Same assumptions and limits as f1_wing_model.py (point mass, prescribed body pitch, prescribed twist, assumed coefficients)."""
import json, math, sys
import numpy as np
import f1_wing_model as w

A_SP, R_SP, SPAN_SP, I_SP = w.A.copy(), w.r.copy(), w.SPAN, w.I_WING
CN = 1.6


def set_wing(r, A, span, mass, phi_deg, inertia):
    w.r = np.asarray(r); w.A = np.asarray(A); w.R = float(w.r.max()); w.SPAN = span; w.MASS = mass
    w.S_WING = 2 * float(w.A.sum()); AR = span ** 2 / w.S_WING; w.A0 = 2 * math.pi * AR / (AR + 2)
    w.BODY_CDA = 0.05 * w.S_WING; w.PHI = math.radians(phi_deg); w.I_WING = inertia


def fly(vx0, vz0, f, tw, b0, b_end, v_ref, t_delay=0.15, T=5.0, dt=0.002):
    """Released from the ground (feet clearance h = 0) at (vx0, vz0). Wings open; flapping starts t_delay after release.
    Body pitch goes from b0 (slow) to b_end at v_ref. Returns (stayed up, lowest point after 0.3 s, end state, mean aero W, peak torque)."""
    h, vx, vz, t = 0.0, vx0, vz0, 0.0; hmin = 1e9; E = 0.0; tq = 0.0; hs = []
    while t < T:
        V = math.hypot(vx, vz)
        beta = math.radians(b_end + (b0 - b_end) * max(0.0, 1 - V / v_ref))
        if t >= t_delay:
            F, p, ta, dphi, ddphi = w.wing_forces(t - t_delay, f, math.radians(tw), math.radians(4), beta, (vx, vz), CN)
            tq = max(tq, abs(w.I_WING * ddphi - ta)); E += max(p, 0) * dt
        else:
            F, p, ta, dphi, ddphi = w.wing_forces(0.0, 1e-6, 0.0, math.radians(4), beta, (vx, vz), CN)      # wings held still
        Dg = w.body_drag((vx, vz))
        vx += (F[0] + Dg[0]) / w.MASS * dt; vz += ((F[1] + Dg[1]) / w.MASS - w.G) * dt; h += vz * dt; t += dt
        hs.append(h)
        if h < 0 and t > 0.1:
            return False, h, (t, vx, vz, h), E / max(t - t_delay, 1e-6), tq
    n1 = int(1.0 / dt); sink = (hs[-1] - hs[-n1]) / 1.0                       # climb rate over the last second
    return sink > -0.15, min(hs[int(0.3 / dt):]), (t, vx, vz, h, sink), E / (T - t_delay), tq


def best_case(vx0, vz0, f, twists, v_ref):
    best = None
    for tw in twists:
        for b0 in (20, 40, 60, 80):
            for be in (3, 15, 30, 45):
                ok, hmin, end, P, tq = fly(vx0, vz0, f, tw, b0, be, v_ref)
                score = (ok, end[0], hmin)
                if best is None or score > best[0]:
                    best = (score, tw, b0, be, hmin, end, P, tq)
    return best


def jump(E, mass, angle_deg, eff=0.8):
    v = math.sqrt(2 * eff * E / mass); a = math.radians(angle_deg)
    return v * math.cos(a), v * math.sin(a)


if __name__ == '__main__':
    mode = sys.argv[1]; res = []
    if mode == 'paper':
        y = (np.arange(12) + 0.5) / 12 * 0.45; c = 0.27 * np.sqrt(1 - (y / 0.45) ** 2); A = c * (0.45 / 12)
        for phi in (30, 40):
            set_wing(y, A, 0.9, 0.18, phi, 0.0004)
            W = 0.18 * w.G; vs = math.sqrt(2 * W / (1.225 * w.S_WING * 1.0))
            print(f'--- reference robot, assumed flap amplitude +-{phi} deg: wing area {w.S_WING:.3f} m2, wing loading {W / w.S_WING:.1f} N/m2, model stall speed {vs:.1f} m/s')
            vx0, vz0 = 2.5, 2.5 * math.sqrt(3)
            for f in (4, 5, 6, 7, 8, 9):
                (ok, tend, _), tw, b0, be, hmin, end, P, tq = best_case(vx0, vz0, f, (20, 30, 40, 50, 60), 1.5 * vs)
                res.append({'phi': phi, 'f': f, 'stays_up': ok, 'twist': tw, 'b0': b0, 'b_end': be, 'end': end, 'aero_W': P})
                print(f'  {f} Hz: {"SUSTAINS FLIGHT" if ok else f"comes down at {end[0]:.1f} s"}  (best twist {tw} deg, pitch {b0}->{be} deg; ends {end[1]:.1f} m/s fwd at {end[3]:.1f} m; aero power {P:.1f} W)', flush=True)
        (w.OUT / 'f3_paper_check.json').write_text(json.dumps(res, indent=1, default=float))
    elif mode == 'sparrow':
        twmax = int(sys.argv[2]) if len(sys.argv) > 2 else 50
        VAR = [('Sparrow as designed, 529 g', dict(mass=0.529, area=1.0, phi=31, inertia=1.0)),
               ('529 g, stroke +-60 deg', dict(mass=0.529, area=1.0, phi=60, inertia=1.0)),
               ('529 g, wing area x2 (chord), stroke +-45 deg', dict(mass=0.529, area=2.0, phi=45, inertia=1.0)),
               ('529 g, wing area x3 (chord), stroke +-45 deg', dict(mass=0.529, area=3.0, phi=45, inertia=1.0)),
               ('450 g, wing area x2, stroke +-45 deg', dict(mass=0.450, area=2.0, phi=45, inertia=1.0)),
               ('400 g, wing area x2, stroke +-45 deg', dict(mass=0.400, area=2.0, phi=45, inertia=1.0))]
        print(f'tip twist allowed up to {twmax} deg; jump at 60 deg, 80% of spring energy becomes speed; flapping starts 0.15 s after release')
        for name, v in VAR:
            set_wing(R_SP, A_SP * v['area'], SPAN_SP, v['mass'], v['phi'], I_SP * v['inertia'])
            W = v['mass'] * w.G; vs = math.sqrt(2 * W / (1.225 * w.S_WING * 1.0))
            print(f'--- {name}: wing loading {W / w.S_WING:.1f} N/m2, model stall speed {vs:.1f} m/s', flush=True)
            for E in (3.0, 7.0, 10.0):
                vx0, vz0 = jump(E, v['mass'], 60); found = None
                for f in (3, 4, 5, 6, 7, 8, 10):
                    (ok, tend, _), tw, b0, be, hmin, end, P, tq = best_case(vx0, vz0, f, tuple(range(20, twmax + 1, 15)), 1.5 * vs)
                    if ok:
                        found = (f, tw, b0, be, end, P, tq); break
                rec = {'variant': name, **v, 'E_J': E, 'vx0': vx0, 'vz0': vz0}
                if found:
                    f, tw, b0, be, end, P, tq = found; rec.update(f=f, twist=tw, aero_W=P, torque_Nm=tq, end=end)
                    print(f'  jump {E:.0f} J ({vx0:.1f} m/s fwd, {vz0:.1f} m/s up, apex {vz0 ** 2 / (2 * w.G):.2f} m): sustains flight from {f} Hz, twist {tw} deg, pitch {b0}->{be} deg; aero power {P:.0f} W, peak motor torque {tq:.1f} Nm per wing; ends {end[1]:.1f} m/s at {end[3]:.1f} m', flush=True)
                else:
                    rec.update(f=None)
                    print(f'  jump {E:.0f} J ({vx0:.1f} m/s fwd, {vz0:.1f} m/s up, apex {vz0 ** 2 / (2 * w.G):.2f} m): does not sustain flight up to 10 Hz', flush=True)
                res.append(rec)
        (w.OUT / f'f3_sparrow_twist{twmax}.json').write_text(json.dumps(res, indent=1, default=float))
