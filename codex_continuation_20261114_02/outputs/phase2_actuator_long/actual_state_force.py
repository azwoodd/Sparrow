def wing_forces(t, f, theta_tip, i0, beta, vel, cn_stall):
    """Both wings. Returns force (fwd, up) in world axes [N], motor power against air [W], aero hinge torque of one wing [Nm]."""
    phi, dphi, phase = ACTUAL_STATE
    fh = np.array([0.0, math.cos(beta), math.sin(beta)]); uh = np.array([0.0, -math.sin(beta), math.cos(beta)]); lh = np.array([1.0, 0.0, 0.0])
    nf = -math.sin(phi) * lh + math.cos(phi) * uh                      # direction the wing moves when phi increases
    th = -theta_tip * (r / R) * math.cos(phase) - i0                    # effective panel pitch, per strip
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
    return np.array([tot[1], tot[2]]), power, float((Fn * r).sum()), dphi, 0.0
