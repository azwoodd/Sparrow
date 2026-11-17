"""Simulation-in-the-loop optimiser for hop parameters (parallel cross-entropy search, deterministic physics).

  python h2_opt.py <task> [gens] [pop]      results -> results/h2_opt/<variant>/<task>.json
Tasks are defined in TASKS: free parameters + bounds, target landing displacement (forward, left, yaw), test conditions.
"""
import json, os, sys, time
os.environ.setdefault('SPARROW_VARIANT', 'v7e_stiffcontact')
for _v in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS', 'NUMEXPR_NUM_THREADS'):
    os.environ[_v] = '1'                 # one thread per worker process (14 workers on 16 logical cores)
import numpy as np

B = {'dy_c': (-0.035, 0.035), 'dz_c': (0.012, 0.055), 'dy_e': (-0.045, 0.045), 'dz_e': (-0.027, 0.012), 'fp_e': (-0.6, 0.6),
     'E': (0.0, 0.30), 'E_hi': (0.0, 0.70), 'kq': (0.3, 1.9), 'dy_trim': (-0.02, 0.04), 'kh': (0.0, 0.3), 'kp_h': (0.0, 4.0), 'hip_bias': (0.0, 0.35), 'dy_l': (-0.06, 0.04), 'dz_l': (-0.022, 0.035), 'fp_l': (-0.5, 0.5), 't_a': (0.04, 0.5), 't_h': (0.05, 0.4),
     'dE': (-0.5, 0.5), 'sc': (-0.03, 0.03), 'dzd': (-0.006, 0.006), 'dy_s': (-0.03, 0.03)}
FWD = ['dy_c', 'dz_c', 'dy_e', 'dz_e', 'fp_e', 'E', 'dy_l', 'dz_l', 'fp_l', 't_a']
LAT = FWD + ['dE', 'sc', 'dzd']
NOM = [{}]
ROB = [{}, {'friction': 0.7}, {'friction': 1.3, 'mass': 1.05}, {'mass': 0.95, 'com_shift': 0.003}]

_lab = None
AERO = True          # the optimiser switches the strip aerodynamics off for speed (wings stowed); validation always runs with it on
T0 = time.time()


def simulate(p, case, n=1, gap=1.5, settle=2.2, pre=None, t0=0.5):
    global _lab
    from lab import Lab
    from hop import Hopper, analyse_hops
    if _lab is None:
        _lab = Lab()
    _lab.s.aero_on = AERO
    if p.get('align') is not None:
        from hop2 import AlignedHopper as Hopper          # aligned-thrust controller for the higher hops (hop.Hopper untouched)
    if p.get('rho') is not None:
        from hop3 import BiHopper as Hopper               # hip-and-knee spring variants
    ctl = Hopper([(t0 + k * gap, p) for k in range(n)], pre=pre)
    dur = t0 + n * gap + settle
    fall = _lab.run(ctl, dur, case=case, stop_after_fall=0.2, every=2)
    return fall, analyse_hops(_lab.rows, _lab.weight, settle_s=2.0), _lab.rows, ctl


def hop_cost(h, tgt):
    """tgt = (forward_mm, left_mm, yaw_deg or None)."""
    if h is None:
        return 400.0
    up = 0.0 if h['stayed_up'] else 120.0 + 2.0 * min(h['tilt_at_touchdown_deg'], 60) - 20.0 * min(h['observed_after_s'], 2.0)
    c = up + abs(h['forward_mm'] - tgt[0]) / 4.0 + abs(h['left_mm'] - tgt[1]) / 4.0
    if tgt[2] is not None:
        c += abs(h['yaw_deg'] - tgt[2]) / 2.0
    lo, hi = (tgt[3], tgt[4]) if len(tgt) > 4 else (18.0, 60.0)        # wanted two-foot clearance band (mm)
    c += 3.0 * max(0.0, lo - h['peak_clear_mm']) + 1.0 * max(0.0, h['peak_clear_mm'] - hi)
    c += 0.5 * h['max_tilt_after_deg'] + 0.5 * h['tilt_at_touchdown_deg'] + 30.0 * h['bounces']
    return c


def evaluate(args):
    x, names, fixed, tgt, cases, n = args
    p = dict(fixed); p.update({k: float(v) for k, v in zip(names, x)})
    tot = 0.0; worst = 0.0
    for case in cases:
        try:
            fall, hops, rows, ctl = simulate(p, case, n=n)
        except Exception:
            return 2000.0
        c = 0.0
        for k in range(n):
            c += hop_cost(hops[k] if k < len(hops) else None, tgt)
        c += 150.0 * max(0, len(hops) - n)          # extra unplanned flights (bouncing/tumbling)
        tot += c; worst = max(worst, c)
    return tot / len(cases) + 0.5 * worst


def optimise(names, tgt, cases=NOM, fixed=None, x0=None, sigma0=0.25, gens=14, pop=20, n=1, label='', seed=0, save=None, evalf=None, bounds=None):
    """Single-process cross-entropy search (one simulation at a time). `save(cost, params, gen)` is called after every
    generation so that stopping the job loses nothing."""
    rng = np.random.default_rng(seed)
    bounds = dict(B, **(bounds or {}))
    lo = np.array([bounds[k][0] for k in names]); hi = np.array([bounds[k][1] for k in names]); span = hi - lo
    from hop import DEFAULT
    mean = np.array([(x0 or {}).get(k, DEFAULT.get(k, 0.0)) for k in names], float)
    mean = np.clip(mean, lo, hi); std = sigma0 * span
    best = (1e9, mean.copy()); fixed = fixed or {}
    evalf = evalf or evaluate
    for g in range(gens):
        X = np.clip(mean + std * rng.standard_normal((pop, len(names))), lo, hi)
        X[0] = best[1] if g else mean
        costs = np.array([evalf((x, names, fixed, tgt, cases, n)) for x in X])
        order = np.argsort(costs); el = X[order[:max(4, pop // 4)]]
        if costs[order[0]] < best[0]:
            best = (float(costs[order[0]]), X[order[0]].copy())
        mean = 0.6 * el.mean(0) + 0.4 * mean
        std = np.maximum(0.6 * el.std(0) + 0.4 * std * 0.9, 0.004 * span)
        params = {**fixed, **{k: float(v) for k, v in zip(names, best[1])}}
        if save:
            save(best[0], params, g)
        print(f'  {label} gen {g:2d} best={best[0]:8.2f} median={np.median(costs):8.1f}  t={time.time() - T0:.0f}s', flush=True)
        if best[0] < 3.0:
            break
    return best[0], params


class Library:
    """results/h2_opt/<variant>/library.json : {name: {cost, params, target, gen}} - rewritten after every generation."""
    def __init__(self):
        from lab import WS, VARIANT
        self.path = WS / 'results' / 'h2_opt' / VARIANT / 'library.json'
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.d = json.loads(self.path.read_text()) if self.path.exists() else {}

    def saver(self, name, tgt):
        def f(cost, params, gen):
            if self.path.exists():                      # merge with the file so two workers never erase each other's entries
                self.d = {**json.loads(self.path.read_text()), **{name: self.d.get(name)}} if name in self.d else json.loads(self.path.read_text())
            self.d[name] = {'cost': cost, 'params': params, 'target': list(tgt), 'gen': gen, 'validated': None}
            self.path.write_text(json.dumps(self.d, indent=1))
        return f

    def get(self, name):
        return self.d.get(name, {}).get('params')


if __name__ == '__main__':
    AERO = False
    lib = Library(); task = sys.argv[1]
    if task == 'forward':
        targets = [int(a) for a in sys.argv[2:]] or [0, 40, 80, 120, 160, 200]
        OPTC = [{}, {'friction': 1.4}, {'mass': 1.07, 'com_shift': 0.003}]       # optimise against three conditions at once
        x0 = None
        for D in targets:
            name = f'fwd{D}'
            prev = lib.get(name)
            x0 = prev or x0
            cold = x0 is None
            c, p = optimise(FWD, (D, 0, None), OPTC, x0=x0, gens=14 if cold else 10, pop=20 if cold else 16, label=name,
                            sigma0=0.25 if cold else 0.08, save=lib.saver(name, (D, 0, None)))
            x0 = p
            print(f'== {name}: cost {c:.2f}  [{time.time() - T0:.0f}s]', flush=True)
    elif task == 'diag':
        # single hop landing forward AND to the left:  python h2_opt.py diag <fwd_mm> <left_mm> [seed-entry]
        F, L = int(sys.argv[2]), int(sys.argv[3]); name = f'diag{F}_{L}'
        x0 = lib.get(name) or lib.get(sys.argv[4] if len(sys.argv) > 4 else f'fwd{F}')
        RC = [{}, {'friction': 1.4}, {'friction': 0.6}, {'mass': 1.07, 'com_shift': 0.003}]
        c, p = optimise(LAT, (F, L, None), RC if 'rob' in sys.argv else NOM, x0=x0, gens=18, pop=24, label=name, sigma0=0.15, save=lib.saver(name, (F, L, None)))
        print(f'== {name}: cost {c:.2f}  [{time.time() - T0:.0f}s]', flush=True)
    elif task == 'turn':
        # hop that rotates the body about the vertical:  python h2_opt.py turn <fwd_mm> <yaw_deg>
        F, Y = int(sys.argv[2]), int(sys.argv[3]); name = f'turn{F}_{Y}'
        x0 = lib.get(name) or lib.get(f'fwd{F}')
        c, p = optimise(LAT, (F, 0, Y), NOM, x0=x0, gens=18, pop=24, label=name, sigma0=0.15, save=lib.saver(name, (F, 0, Y)))
        print(f'== {name}: cost {c:.2f}  [{time.time() - T0:.0f}s]', flush=True)


    elif task == 'high':
        # higher hop with a wanted clearance band:  python h2_opt.py high <fwd_mm> <clear_lo> <clear_hi> [seed-entry]
        # Spring energy bound raised to 0.70 J per knee for this task only; everything else unchanged.
        F, lo, hi = int(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4]); name = f'high{F}_{int(lo)}_{int(hi)}'
        x0 = lib.get(name) or lib.get(sys.argv[5] if len(sys.argv) > 5 else 'fwd50')
        RC = [{}, {'friction': 1.4}, {'friction': 0.6}, {'mass': 1.07, 'com_shift': 0.003}]
        c, p = optimise(FWD, (F, 0, None, lo, hi), RC, x0=x0, gens=16, pop=20, label=name, sigma0=0.15,
                        save=lib.saver(name, (F, 0, None, lo, hi)), bounds={'E': B['E_hi'], 'dz_c': (0.012, 0.062)})
        print(f'== {name}: cost {c:.2f}  [{time.time() - T0:.0f}s]', flush=True)
    elif task == 'ladder':
        # Energy continuation: fix the spring energy, re-tune only the push/landing geometry so the body leaves the ground
        # without pitch spin, then step the energy up and warm-start from the previous rung.
        #   python h2_opt.py ladder <seed-entry> <E1> <E2> ...     (E = joules per knee)
        seed = sys.argv[2]; x0 = lib.get(seed)
        GEO = [k for k in FWD if k != 'E']
        RC = [{}, {'friction': 1.4}, {'mass': 1.07, 'com_shift': 0.003}]
        wide = {'dy_c': (-0.045, 0.045), 'dy_e': (-0.055, 0.055), 'dz_c': (0.012, 0.062), 'dy_l': (-0.07, 0.05), 'fp_e': (-0.8, 0.8)}
        for E in [float(a) for a in sys.argv[3:]]:
            name = f'ladder_E{int(round(E * 100)):03d}'
            x0 = lib.get(name) or x0
            c, p = optimise(GEO, (40, 0, None, 18.0, 400.0), RC, fixed={'E': E}, x0=x0, gens=10, pop=18, label=name, sigma0=0.06,
                            save=lib.saver(name, (40, 0, None, 18.0, 400.0)), bounds=wide)
            x0 = p
            print(f'== {name}: cost {c:.2f}  [{time.time() - T0:.0f}s]', flush=True)
    elif task == 'aligned':
        # aligned crouch + long-stroke soft spring + hip pitch damping + landing, tuned together at FIXED spring energy
        #   python h2_opt.py aligned <E per knee> <clear_lo> <clear_hi> [seed-entry]
        E, lo, hi = float(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4]); name = f'aligned_E{int(round(E * 100)):03d}'
        seed = lib.get(name) or lib.get(sys.argv[5] if len(sys.argv) > 5 else 'ladder_E025')
        x0 = dict({'kq': 1.3, 'dy_trim': 0.0, 'kh': 0.15, 'kp_h': 1.0, 'dz_c': 0.050, 'dz_e': -0.026}, **{k: v for k, v in seed.items() if k in ('fp_e', 'dy_l', 'dz_l', 'fp_l', 't_a')})
        if lib.get(name): x0 = lib.get(name)
        NAMES = ['dz_c', 'kq', 'dy_trim', 'kh', 'kp_h', 'fp_e', 'dz_e', 'dy_l', 'dz_l', 'fp_l', 't_a']
        RC = [{}, {'friction': 1.4}, {'mass': 1.07, 'com_shift': 0.003}]
        c, p = optimise(NAMES, (60, 0, None, lo, hi), RC, fixed={'E': E, 'align': 1}, x0=x0, gens=14, pop=24, label=name, sigma0=0.2,
                        save=lib.saver(name, (60, 0, None, lo, hi)), bounds={'dz_c': (0.02, 0.062), 'dy_l': (-0.07, 0.05), 'fp_e': (-0.8, 0.8)})
        print(f'== {name}: cost {c:.2f}  [{time.time() - T0:.0f}s]', flush=True)
    elif task == 'fair':
        # FAIR A/B between model variants: identical small search (same seed, budget, conditions, objective) on whichever
        # SPARROW_VARIANT is set. Free: crouch trim, hip bias (up to that variant's own hip limit) and the landing pose.
        #   python h2_opt.py fair <E per knee> <clear_lo> <clear_hi>
        import mujoco as _mj
        from lab import WS as _WS, Lab as _Lab
        E, lo, hi = float(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4]); name = f'fair_E{int(round(E * 100)):03d}'
        ref = json.loads((_WS / 'results' / 'h2_opt' / 'v7e_stiffcontact' / 'library.json').read_text())['ladder_E025']['params']
        _lab = _Lab(); lim = float(_lab.m.actuator_forcerange[_lab.ids[0], 1])
        fixed = dict({k: v for k, v in ref.items() if k in ('fp_e',)}, E=E, align=1, hip_follow=1, ank_follow=1, dz_c=0.030, dz_e=-0.020)
        x0 = lib.get(name) or {'dy_trim': 0.0, 'hip_bias': 0.8 * lim, 'dy_l': ref['dy_l'], 'dz_l': 0.02, 'fp_l': ref['fp_l'], 't_a': ref['t_a']}
        RC = [{}, {'friction': 1.4}, {'mass': 1.07, 'com_shift': 0.003}]
        c, p = optimise(['dy_trim', 'hip_bias', 'dy_l', 'dz_l', 'fp_l', 't_a'], (60, 0, None, lo, hi), RC, fixed=fixed, x0=x0, gens=9, pop=16, label=name, sigma0=0.2,
                        save=lib.saver(name, (60, 0, None, lo, hi)), bounds={'hip_bias': (0.0, lim), 'dy_l': (-0.07, 0.05), 'dz_l': (-0.02, 0.045)})
        print(f'== {name}: cost {c:.2f}  [{time.time() - T0:.0f}s]', flush=True)
    elif task == 'bituned':
        # hip-and-knee spring variant: small landing-focused tune at a FIXED spring split and energy (take-off untouched
        # except a fine crouch trim).   python h2_opt.py bituned <E per leg> <rho> <dy_c> <clear_lo> <clear_hi>
        from lab import WS as _WS
        E, rho, dyc, lo, hi = (float(a) for a in sys.argv[2:7]); name = f'bi_E{int(round(E * 100)):03d}'
        ref = json.loads((_WS / 'results' / 'h2_opt' / 'v7e_stiffcontact' / 'library.json').read_text())['ladder_E025']['params']
        fixed = dict(fp_e=ref['fp_e'], E=E, rho=rho, align=0, hip_follow=1, ank_follow=1, kh=0.10, kp_h=1.0, dz_c=0.030, dz_e=-0.020)
        prev = lib.get('bi_E080')           # start higher hops from the validated 60 mm landing settings
        x0 = lib.get(name) or (dict({k: prev[k] for k in ('dy_l', 'dz_l', 'fp_l', 't_a', 'dy_s')}, dy_c=dyc) if prev else None) or {'dy_c': dyc, 'dy_l': ref['dy_l'], 'dz_l': ref['dz_l'], 'fp_l': ref['fp_l'], 't_a': ref['t_a'], 'dy_s': 0.0}
        RC = [{}, {'friction': 0.6}, {'friction': 1.4}, {'mass': 1.07, 'com_shift': -0.002}]
        names = ['dy_c', 'dy_l', 'dz_l', 'fp_l', 't_a', 'dy_s']
        if 'hipbias' in sys.argv:           # same hardware split (rho fixed); let the 0.35 Nm hip servos trim the take-off spin either way
            names.append('hip_bias'); x0.setdefault('hip_bias', 0.0)
        c, p = optimise(names, (50, 0, None, lo, hi), RC, fixed=fixed, x0=x0, gens=8, pop=16, label=name, sigma0=0.10,
                        save=lib.saver(name, (50, 0, None, lo, hi)), bounds={'dy_c': (dyc - 0.012, dyc + 0.012), 'dy_l': (-0.07, 0.05), 'dz_l': (-0.02, 0.045), 'hip_bias': (-0.35, 0.35)})
        print(f'== {name}: cost {c:.2f}  [{time.time() - T0:.0f}s]', flush=True)



