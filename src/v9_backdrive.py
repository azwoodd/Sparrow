"""Servo back-drive during the push (set SPARROW_VARIANT=v7e_bispring). The spring turns the knee at ~30 rad/s, about three
times a hobby servo's no-load speed, so the leg has to drive the servo gear trains. The validated model only has a
placeholder viscous drag (0.02 Nm s/rad) and almost no rotor inertia. Here the three physical parts are added:
  arm     reflected rotor inertia (kg m^2 at the joint)           fric   dry gear friction (Nm)
  emf_w0  back-EMF drag = stall torque / no-load speed            knee_free   knee servo decoupled during the push
  python v9_backdrive.py single            one factor at a time and three combined servo cases, NO re-tuning
  python v9_backdrive.py calib <case>      re-calibrate a combined case with stored energy and hip-servo trim
  python v9_backdrive.py free              the combined cases with the knee servo decoupled during the push
5 conditions per cell. Values are ranges for small 0.2-0.35 Nm hobby servos, not measurements of Sparrow's servos."""
import json, os, sys
os.environ.setdefault('SPARROW_VARIANT', 'v7e_bispring')
import numpy as np
import v3_bispring as v3
from hop3 import BiHopper
from lab import WS, VARIANT

OUT = WS / 'results' / 'v9_backdrive'; OUT.mkdir(parents=True, exist_ok=True)
lib = json.loads((WS / 'results' / 'h2_opt' / VARIANT / 'library.json').read_text())
CONDS = [('nominal', {}), ('friction0.6', {'friction': 0.6}), ('friction1.4', {'friction': 1.4}), ('mass1.07', {'mass': 1.07}), ('comY-3', {'com_shift': -0.003})]
CASES = {'light': {'arm': 1e-4, 'fric': 0.02, 'emf_w0': 14.0},          # fast coreless servo, low-friction gears
         'typical': {'arm': 3e-4, 'fric': 0.035, 'emf_w0': 10.5},       # 0.10 s/60 deg servo, friction 10% of stall
         'heavy': {'arm': 1e-3, 'fric': 0.07, 'emf_w0': 7.5}}           # slow high-ratio servo, friction 20% of stall
SINGLE = [('as validated', {}),
          ('rotor inertia 1e-4', {'arm': 1e-4}), ('rotor inertia 3e-4', {'arm': 3e-4}), ('rotor inertia 1e-3', {'arm': 1e-3}),
          ('dry friction 0.02 Nm', {'fric': 0.02}), ('dry friction 0.035 Nm', {'fric': 0.035}), ('dry friction 0.07 Nm', {'fric': 0.07}),
          ('back-EMF, no-load 14 rad/s', {'emf_w0': 14.0}), ('back-EMF, no-load 10.5 rad/s', {'emf_w0': 10.5}), ('back-EMF, no-load 7.5 rad/s', {'emf_w0': 7.5}),
          ('LIGHT servo (all three)', CASES['light']), ('TYPICAL servo (all three)', CASES['typical']), ('HEAVY servo (all three)', CASES['heavy'])]

# keep the controller of the last hop so the energy bookkeeping can be read back
_last = {}
_orig_init = BiHopper.__init__
def _init(self, *a, **k):
    _orig_init(self, *a, **k); _last['ctl'] = self
BiHopper.__init__ = _init


def cell(p):
    n = 0; cl = []; tl = []; rows = []
    for cn, case in CONDS:
        r = v3.hop(p, case); c = _last['ctl']
        r.update(cond=cn, drag_loss_J=c.drag_loss, rotor_ke_J=c.rotor_ke_takeoff, delivered_J=c.delivered)
        n += r['good']; cl.append(r.get('clear', 0)); tl.append(r.get('tilt_after', 99)); rows.append(r)
    return n, cl, tl, rows


if __name__ == '__main__':
    mode = sys.argv[1]; res = []
    if mode == 'single':
        for name in sys.argv[2:] or ['bi_E080', 'bi_E110']:
            base = lib[name]['params']; print(f'=== {name} (E {base["E"]} J per leg), no re-tuning', flush=True)
            for label, ch in SINGLE:
                n, cl, tl, rows = cell(dict(base, **ch)); nom = rows[0]
                for r in rows: res.append(dict(r, entry=name, test=label))
                print(f"  {label:30} {n}/5 good  clearance {min(cl):5.1f}-{max(cl):5.1f} mm  max tilt after {max(tl):5.1f}  | nominal: spring gave {nom['delivered_J']:.2f} J, "
                      f"drag took {nom['drag_loss_J']:.2f} J, rotors hold {nom['rotor_ke_J']:.2f} J at take-off, vz {nom.get('vz', 0):.2f} m/s, L {nom.get('L', 0):+.1f} mNms", flush=True)
        (OUT / 'single.json').write_text(json.dumps(res, indent=1, default=float))
    elif mode == 'calib':
        cname = sys.argv[2]; extra = {'knee_free': 1} if 'free' in sys.argv[3:] else {}
        names = [a for a in sys.argv[3:] if a.startswith('bi_')] or ['bi_E080', 'bi_E110']
        scales = {'light': (1.0, 1.15, 1.3), 'typical': (1.3, 1.5, 1.7, 1.9), 'heavy': (1.8, 2.2, 2.6, 3.0)}[cname]
        if extra: scales = (0.45, 0.55, 0.65, 0.75, 0.9)
        for name in names:
            base = lib[name]['params']; print(f'=== {name} with {cname.upper()} servo back-drive{" + knee decoupled" if extra else ""}: stored energy x hip-servo trim', flush=True)
            for es in scales:
                row = []
                for hb in (-0.35, -0.21, -0.07, 0.07, 0.21, 0.35):
                    n, cl, tl, rows = cell(dict(base, **CASES[cname], **extra, E_scale=es, hip_bias=hb))
                    row.append((hb, n, min(cl), max(cl))); res.append({'entry': name, 'case': cname, 'knee_free': bool(extra), 'E_scale': es, 'hip_bias': hb, 'good': n, 'clear_min': min(cl), 'clear_max': max(cl), 'peak_knee_tau': rows[0]['peak_knee_tau']})
                b = max(row, key=lambda x: x[1])
                print(f"  energy x{es:.2f} ({2 * base['E'] * es:.2f} J): good/5 by trim " + ' '.join(f'{hb:+.2f}:{n}' for hb, n, _, _ in row) + f"   best {b[1]}/5 at {b[0]:+.2f} Nm, clearance {b[2]:.0f}-{b[3]:.0f} mm", flush=True)
        (OUT / f'calib_{cname}{"_free" if extra else ""}_{"_".join(names)}.json').write_text(json.dumps(res, indent=1, default=float))
    elif mode == 'free':
        for name in sys.argv[2:] or ['bi_E080', 'bi_E110']:
            base = lib[name]['params']; print(f'=== {name}: knee servo decoupled during the push, no re-tuning', flush=True)
            for cname, ch in CASES.items():
                for label, ex in ((cname, {}), (cname + ' + knee decoupled', {'knee_free': 1})):
                    n, cl, tl, rows = cell(dict(base, **ch, **ex)); nom = rows[0]
                    for r in rows: res.append(dict(r, entry=name, test=label))
                    print(f"  {label:28} {n}/5 good  clearance {min(cl):5.1f}-{max(cl):5.1f} mm  max tilt after {max(tl):5.1f}  | drag took {nom['drag_loss_J']:.2f} J, rotors {nom['rotor_ke_J']:.2f} J", flush=True)
        (OUT / 'free.json').write_text(json.dumps(res, indent=1, default=float))
