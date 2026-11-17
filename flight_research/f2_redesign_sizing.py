"""How far would the design have to move for a self take-off?  Uses the f1 wing model (same assumptions and limits).
A self take-off from a hop needs thrust about equal to weight for a second or two (hover-class), with the body pitched up.
For each design variant: the cheapest flapping (frequency, tip twist) that makes thrust >= weight at rest, its
aerodynamic power, the peak motor torque per wing, and the same torque if a spring at the hinge cancels the wing's own
inertia (aero torque only).  Variants scale the existing wing; nothing here is a CAD change.
Reference limits used for the verdict [A]: 50 W mean aerodynamic (about 100-120 W electrical from an 1806-size motor for
a few seconds) and 4 Nm peak per wing (cruise already needs about 3 Nm)."""
import json, math
import numpy as np
import f1_wing_model as w

import sys
A_BASE = w.A.copy(); I_BASE = w.I_WING; CN = 1.6
TWMAX = int(sys.argv[1]) if len(sys.argv) > 1 else 85      # largest tip twist allowed (deg). 85 = wing feathered like a propeller blade
print(f'largest tip twist allowed: {TWMAX} deg')
VARIANTS = [
 ('as designed: 529 g, +-31 deg, 56 g wings', dict(mass=0.529, phi=31, inertia=1.0, area=1.0)),
 ('flap stroke +-60 deg', dict(mass=0.529, phi=60, inertia=1.0, area=1.0)),
 ('flap stroke +-75 deg', dict(mass=0.529, phi=75, inertia=1.0, area=1.0)),
 ('wings half the inertia', dict(mass=0.529, phi=31, inertia=0.5, area=1.0)),
 ('robot 400 g', dict(mass=0.400, phi=31, inertia=1.0, area=1.0)),
 ('robot 300 g', dict(mass=0.300, phi=31, inertia=1.0, area=1.0)),
 ('wing chord +50% (area x1.5), same inertia', dict(mass=0.529, phi=31, inertia=1.0, area=1.5)),
 ('+-60 deg and half-inertia wings', dict(mass=0.529, phi=60, inertia=0.5, area=1.0)),
 ('400 g, +-60 deg, half-inertia wings', dict(mass=0.400, phi=60, inertia=0.5, area=1.0)),
 ('300 g, +-60 deg, half-inertia wings', dict(mass=0.300, phi=60, inertia=0.5, area=1.0)),
 ('300 g, +-75 deg, half-inertia wings, chord +50%', dict(mass=0.300, phi=75, inertia=0.5, area=1.5)),
 ('250 g, +-75 deg, third-inertia wings, chord +50%', dict(mass=0.250, phi=75, inertia=0.33, area=1.5)),
]
res = []
print('variant | flapping needed | aero power | peak torque per wing | with hinge spring | verdict')
for name, v in VARIANTS:
    w.MASS = v['mass']; w.PHI = math.radians(v['phi']); w.I_WING = I_BASE * v['inertia']; w.A = A_BASE * v['area']
    W = v['mass'] * w.G; best = None
    for tw in range(20, TWMAX + 1, 5):
        for f in np.arange(1.5, 12.01, 0.25):
            F, P, tq, pk = w.cycle(float(f), math.radians(tw), 0.0, 0.0, (0.0, 0.0), CN, n=60)
            if F[0] >= W:
                if best is None or P < best[0]:
                    # aero-only torque: repeat with zero wing inertia
                    keep = w.I_WING; w.I_WING = 0.0; _, _, tq_aero, _ = w.cycle(float(f), math.radians(tw), 0.0, 0.0, (0.0, 0.0), CN, n=60); w.I_WING = keep
                    best = (P, float(f), tw, tq, tq_aero)
                break
    if best:
        P, f, tw, tq, tqa = best
        ok_p = P <= 50; ok_t = tq <= 4.0; ok_ts = tqa <= 4.0
        verdict = 'within both limits' if ok_p and ok_t else ('within limits only with a hinge spring' if ok_p and ok_ts else ('power ok, torque too high' if ok_p else 'power too high'))
        res.append({'variant': name, **v, 'f_Hz': f, 'twist_deg': tw, 'aero_W': P, 'peak_torque_Nm': tq, 'peak_torque_aero_only_Nm': tqa, 'verdict': verdict})
        print(f'{name:50} | {f:5.2f} Hz, twist {tw} deg | {P:6.0f} W | {tq:5.1f} Nm | {tqa:5.1f} Nm | {verdict}')
    else:
        res.append({'variant': name, **v, 'verdict': 'thrust never reaches weight up to 12 Hz'})
        print(f'{name:50} | thrust never reaches weight up to 12 Hz')
(w.OUT / f'f2_redesign_sizing_twist{TWMAX}.json').write_text(json.dumps(res, indent=1, default=float))
