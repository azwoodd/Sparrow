"""Sparrow flapping-wing force model with twist (first version) and take-off trajectories.  Standalone: reads the copied
model for wing geometry only; nothing in MuJoCo is run and no file of the owner's is touched.

Model (quasi-steady blade elements, one wing computed and doubled):
  * strips = the wing 'aero_strips' of the model export, at their real distance from the flap hinge (wings deployed)
  * flapping  phi(t) = PHI sin(2 pi f t)  about the fore-aft hinge, PHI = 31 deg (the model's flap range)
  * twist     theta(r,t) = -theta_tip (r/R) cos(2 pi f t): leading edge down on the downstroke, up on the upstroke,
              growing linearly to the tip.  THIS MECHANISM DOES NOT EXIST IN THE CAD; it is prescribed here to find out
              how much of it would be needed.  i0 = fixed wing incidence (leading edge up).
  * force on a strip = normal force only (plus skin friction): attached flow  CN = a0 sin(alpha) below ~10 deg,
              blending to a stalled flat plate  CN = CN_STALL sin(alpha) above ~18 deg.  No leading-edge suction, no
              unsteady or added-mass terms, no wing-wing or wing-body interference, no flexibility.
  * body = point mass with a prescribed pitch angle (nose-up angle beta); pitch dynamics and the tail are NOT modelled.
All coefficients are assumptions [A]; results are bounds to steer design, not predictions.
  python f1_wing_model.py static | cruise | takeoff
"""
import json, math, sys
from pathlib import Path
import numpy as np
import mujoco

HERE = Path(__file__).resolve().parent; MODEL = HERE.parent / 'model' / 'v7e_stiffcontact'
OUT = HERE / 'results'; OUT.mkdir(exist_ok=True)
info = json.load(open(MODEL / 'sparrow_v7e_model.json'))
m = mujoco.MjModel.from_xml_path(str(MODEL / 'sparrow_v7e.xml')); d = mujoco.MjData(m)
for j, q in info['poses']['stand_wings_deployed']['q'].items():
    d.qpos[m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, j)]] = q
mujoco.mj_forward(m, d)
hinge = d.xanchor[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, 'wing_L_flap')]
R_, A_ = [], []
for s in info['aero_strips']:
    if s['body'].startswith('wing_L'):
        b = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, s['body'])
        p = d.xpos[b] + d.xmat[b].reshape(3, 3) @ np.array(s['pos'])
        R_.append(abs(p[0] - hinge[0])); A_.append(s['area_m2'])
r = np.array(R_); A = np.array(A_); R = r.max()
RHO, G = 1.225, 9.81
MASS = 0.529                       # kg, candidate-parts mass from parts/BOM.md
PHI = math.radians(31.0)           # flap amplitude
I_WING = 0.0049                    # kg m^2 about the flap hinge, one wing (measured from the model in the F0 audit)
SPAN = info['wing_span_mm'] / 1000; S_WING = 2 * A.sum(); AR = SPAN ** 2 / S_WING
A0 = 2 * math.pi * AR / (AR + 2)   # finite-wing lift slope
CF = 0.02                          # skin friction on the panels
BODY_CDA = 0.05 * S_WING           # body + legs + tail drag area [A]
import os
ETA_S = float(os.environ.get('SPARROW_ETA_S', 0.7))   # leading-edge suction efficiency [A]; 0 reproduces the first version


def wing_forces(t, f, theta_tip, i0, beta, vel, cn_stall):
    """Both wings. Returns force (fwd, up) in world axes [N], motor power against air [W], aero hinge torque of one wing [Nm]."""
    w = 2 * math.pi * f
    phi = PHI * math.sin(w * t); dphi = PHI * w * math.cos(w * t)
    fh = np.array([0.0, math.cos(beta), math.sin(beta)]); uh = np.array([0.0, -math.sin(beta), math.cos(beta)]); lh = np.array([1.0, 0.0, 0.0])
    nf = -math.sin(phi) * lh + math.cos(phi) * uh                      # direction the wing moves when phi increases
    th = -theta_tip * (r / R) * math.cos(w * t) - i0                    # effective panel pitch, per strip
    npan = np.cos(th)[:, None] * nf[None, :] + np.sin(th)[:, None] * fh[None, :]      # panel normal (-sin(th) * aft = +sin(th) * fwd)
    vb = np.array([0.0, vel[0], vel[1]])
    vrel = -(vb[None, :] + (r * dphi)[:, None] * nf[None, :])           # air relative to each strip
    sp = np.linalg.norm(vrel, axis=1) + 1e-9
    vn = np.einsum('ij,ij->i', vrel, npan); sa = np.abs(vn) / sp
    al = np.arcsin(np.clip(sa, 0, 1)); sg = np.clip((al - math.radians(10)) / math.radians(8), 0, 1); sg = sg * sg * (3 - 2 * sg)
    cn = ((1 - sg) * A0 + sg * cn_stall) * sa
    N = 0.5 * RHO * A * cn * sp ** 2
    F = (N * np.sign(vn))[:, None] * npan
    # leading-edge suction, attached flow only: a force along the panel chord towards the leading edge, ETA_S of the ideal
    # value N tan(alpha). Without it a wing in normal flight has several times too much drag (first version of this model).
    cp = np.cos(th)[:, None] * fh[None, :] - np.sin(th)[:, None] * nf[None, :]
    F += (ETA_S * (1 - sg) * N * np.tan(np.minimum(al, 0.5)))[:, None] * cp
    F += (0.5 * RHO * A * CF * sp)[:, None] * (vrel - vn[:, None] * npan)
    Fn = F @ nf
    power = -2 * float((Fn * r * dphi).sum())                           # both wings
    tot = 2 * F.sum(axis=0)                                             # lateral cancels between the wings
    # induced drag of the finite wing, from the force across the flight path (capped at low speed where the formula fails)
    V = math.hypot(vel[0], vel[1])
    if V > 1.0:
        vh = np.array([0.0, vel[0], vel[1]]) / V
        perp = tot - (tot @ vh) * vh; Lp = float(np.linalg.norm(perp))
        Di = min(Lp ** 2 / (0.5 * RHO * V * V * math.pi * 0.8 * SPAN ** 2), 0.5 * Lp)
        tot = tot - Di * vh
    return np.array([tot[1], tot[2]]), power, float((Fn * r).sum()), dphi, -PHI * w * w * math.sin(w * t)


def cycle(f, theta_tip, i0, beta, vel, cn_stall, n=80):
    Fs = np.zeros(2); P = 0.0; tq = 0.0; pmech_peak = 0.0
    for k in range(n):
        t = k / (n * f)
        F, p, ta, dphi, ddphi = wing_forces(t, f, theta_tip, i0, beta, vel, cn_stall)
        Fs += F; P += p
        tm = I_WING * ddphi - ta                                         # motor torque on one wing
        tq = max(tq, abs(tm)); pmech_peak = max(pmech_peak, 2 * abs(tm * dphi))
    return Fs / n, P / n, tq, pmech_peak


def body_drag(vel):
    v = math.hypot(*vel)
    return -0.5 * RHO * BODY_CDA * v * np.array(vel)


if __name__ == '__main__':
    mode = sys.argv[1] if len(sys.argv) > 1 else 'static'
    W = MASS * G
    print(f'wing: {len(r)} strips per side, tip radius {R:.3f} m, area {S_WING:.4f} m2, AR {AR:.1f}, lift slope {A0:.2f}; mass {MASS} kg, weight {W:.2f} N')
    res = []
    if mode == 'static':
        # held still in still air, body level: thrust along the body, and what it costs
        for cn in (1.2, 2.0):
            print(f'--- static (no airspeed), stalled-plate coefficient {cn}')
            print('  f Hz  twist  thrust N  thrust/weight  vertical N  aero power W  peak motor torque per wing Nm  peak mech power W')
            for f in (2, 3, 4, 5, 6, 8, 10):
                for tw in (0, 15, 30, 45):
                    F, P, tq, pk = cycle(f, math.radians(tw), 0.0, 0.0, (0.0, 0.0), cn)
                    res.append({'cn': cn, 'f': f, 'twist_deg': tw, 'thrust_N': F[0], 'vertical_N': F[1], 'aero_power_W': P, 'peak_torque_Nm': tq, 'peak_mech_W': pk})
                    print(f'  {f:4.0f}  {tw:4.0f}   {F[0]:7.2f}   {F[0] / W:7.2f}       {F[1]:7.2f}     {P:8.1f}        {tq:6.2f}                    {pk:7.0f}')
        (OUT / 'f1_static.json').write_text(json.dumps(res, indent=1, default=float))
    elif mode == 'cruise':
        # level flight: find flap rate, twist, body angle so that mean lift = weight and mean thrust = drag, at least power
        for cn in (1.2, 2.0):
            print(f'--- level cruise, stalled-plate coefficient {cn}, wing incidence 4 deg')
            for U in (6.0, 7.0, 8.0, 9.0, 10.0):
                best = None
                for f in np.arange(1.5, 8.01, 0.5):
                    for tw in range(0, 41, 5):
                        for bd in range(0, 15, 2):
                            F, P, tq, pk = cycle(f, math.radians(tw), math.radians(4), math.radians(bd), (U, 0.0), cn, n=40)
                            net = F + body_drag((U, 0.0)); ez = (net[1] - W) / W; ex = net[0] / W
                            if ez >= 0 and ex >= 0:
                                if best is None or P < best[0]: best = (P, f, tw, bd, ez, ex, tq)
                if best:
                    P, f, tw, bd, ez, ex, tq = best
                    res.append({'cn': cn, 'U': U, 'power_W': P, 'f': f, 'twist_deg': tw, 'body_deg': bd, 'peak_torque_Nm': tq})
                    print(f'  {U:4.1f} m/s: least aero power {P:5.1f} W at {f:.1f} Hz, tip twist {tw} deg, body {bd} deg nose-up (surplus lift {ez * 100:.0f}%, thrust {ex * 100:.0f}% of weight), peak motor torque {tq:.2f} Nm per wing')
                else:
                    print(f'  {U:4.1f} m/s: no combination holds level flight')
        (OUT / 'f1_cruise.json').write_text(json.dumps(res, indent=1, default=float))
    elif mode == 'takeoff':
        # point-mass trajectories. Body pitch is scheduled from beta0 (slow) down to 8 deg at 7 m/s. Wings already open.
        STARTS = {'100 mm hop (1.47 m/s up, 0.3 m/s fwd), wings open and flapping at lift-off': (0.0, 1.47, 0.3),
                  'dropped from a 1.0 m perch': (1.0, 0.0, 0.0), 'dropped from a 2.0 m perch': (2.0, 0.0, 0.0), 'dropped from a 3.0 m perch': (3.0, 0.0, 0.0),
                  'thrown level at 6 m/s from 1.5 m': (1.5, 0.0, 6.0), 'thrown level at 8 m/s from 1.5 m': (1.5, 0.0, 8.0),
                  'thrown level at 10 m/s from 1.5 m': (1.5, 0.0, 10.0)}
        cn = float(sys.argv[2]) if len(sys.argv) > 2 else 1.6
        print(f'--- take-off trajectories, stalled-plate coefficient {cn}; success = never below ground level in 4 s and flying at >= 8.5 m/s without sinking')
        for name, (h0, vz0, vx0) in STARTS.items():
            print(name)
            for f in (3, 4, 5, 6, 8, 10):
                best = None
                for tw in (20, 30, 40, 50):
                    for b0 in (-70, -45, -20, 3, 10, 30, 50, 70, 85):          # negative = nose-down dive to trade height for speed
                        h, vx, vz, t = h0, vx0, vz0, 0.0; hmin = 1e9; E = 0.0; dt = 0.002; ok = True; tq = 0.0
                        while t < 4.0:
                            V = math.hypot(vx, vz)
                            beta = math.radians(3 + (b0 - 3) * max(0.0, 1 - V / 9.0))
                            F, p, ta, dphi, ddphi = wing_forces(t, f, math.radians(tw), math.radians(4), beta, (vx, vz), cn)
                            Dg = body_drag((vx, vz)); E += max(p, 0) * dt; tq = max(tq, abs(I_WING * ddphi - ta))
                            vx += (F[0] + Dg[0]) / MASS * dt; vz += ((F[1] + Dg[1]) / MASS - G) * dt; h += vz * dt; t += dt
                            if t > 0.05: hmin = min(hmin, h)
                            if h < 0 and t > 0.05: ok = False; break
                        fly = ok and vx >= 8.5 and vz > -0.3
                        score = (fly, ok, hmin if ok else t)
                        if best is None or score > best[0]:
                            best = (score, tw, b0, hmin, vx, vz, h, t, E / max(t, 1e-6), tq)
                (fly, ok, _), tw, b0, hmin, vx, vz, h, t, pw, tq = best
                res.append({'start': name, 'f': f, 'twist_deg': tw, 'beta0_deg': b0, 'flying': fly, 'stayed_up': ok, 'lowest_m': hmin, 'end_vx': vx, 'end_vz': vz, 'end_h': h, 't_end': t, 'mean_aero_W': pw, 'peak_torque_Nm': tq})
                verdict = 'FLYING' if fly else ('airborne but not at flying speed' if ok else f'hits the ground at {t:.2f} s')
                print(f'  {f:3.0f} Hz: {verdict:34} best twist {tw} deg, start pitch {b0} deg; lowest point {hmin:+.2f} m, ends {vx:.1f} m/s fwd {vz:+.1f} m/s up at {h:.1f} m; mean aero power {pw:.0f} W, peak motor torque {tq:.1f} Nm per wing')
        (OUT / f'f1_takeoff_cn{int(cn * 10)}.json').write_text(json.dumps(res, indent=1, default=float))
