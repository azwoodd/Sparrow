"""Sparrow trainer: improve one skill at a time with the same pilot and physics the game uses. One process, saves after
every generation, never touches skills.json until you promote a candidate.

  python train.py list
  python train.py run <task> [--gens 6] [--pop 8] [--target N] [--seed 0]
  python train.py validate [candidate.json]        full 45 s routine on 5 conditions, current skills or a candidate
  python train.py promote <candidate.json>         copy a candidate into skills.json (old file is backed up first)
  python train.py restore                          put the most recent backup of skills.json back
Method: cross-entropy search (sample settings around the current best, keep the best quarter, narrow, repeat). The score is
measured, not predicted: each sample is simulated on several robot conditions and falls are penalised heavily."""
import argparse, copy, json, shutil, sys, time
from pathlib import Path
import numpy as np

GAME = Path(__file__).resolve().parent
CAND = GAME / 'candidates'; BACK = GAME / 'skills_backups'
SEARCH = [{}, {'friction': 0.6}, {'mass': 1.07}]
CHECK = [('nominal', {}), ('friction 0.6', {'friction': 0.6}), ('friction 1.4', {'friction': 1.4}), ('mass +7%', {'mass': 1.07}), ('balance 3 mm back', {'com_shift': -0.003})]


def _walk_score(speed):
    def f(w, sk, case, target):
        r = w.run_script([(1, {}), (6, {'speed': speed}), (2.5, {})], skills=sk, case=case)
        return (0.0, True) if r['fallen'] else (abs(r['segments'][1]['mm_per_s']), False)
    return f


def _steer(w, sk, case, target):
    tot = 0.0; fell = False
    for turn in (1.0, -1.0):
        r = w.run_script([(1, {}), (2, {'speed': 0.5}), (5, {'speed': 0.5, 'turn': turn}), (2, {})], skills=sk, case=case)
        fell = fell or bool(r['fallen']); tot += 0.0 if r['fallen'] else turn * w.heading_deg
    return tot / 2, fell


def _pivot(w, sk, case, target):
    tot = 0.0; fell = False
    for turn in (1.0, -1.0):
        r = w.run_script([(1, {}), (5, {'turn': turn}), (2, {})], skills=sk, case=case)
        fell = fell or bool(r['fallen']); tot += 0.0 if r['fallen'] else turn * w.heading_deg - 0.05 * abs(r['forward_mm'])
    return tot / 2, fell


def _reverse(w, sk, case, target):
    r = w.run_script([(1, {}), (3, {'speed': 0.5}), (2, {}), (4, {'speed': -0.5}), (2, {})], skills=sk, case=case)
    return (0.0, True) if r['fallen'] else (-r['segments'][3]['mm_per_s'], False)


def _hop(slot, what):
    def f(w, sk, case, target):
        r = w.run_script([(1, {}), (3.2, {'do': slot}), (1.0, {})], skills=sk, case=case)
        if r['fallen'] or r['landed'] < 1: return -200.0, True
        s = r['segments'][1]; return -abs((s['clear_mm'] if what == 'clear' else s['forward_mm']) - target), False
    return f


def _moving_jump(w, sk, case, target):
    r = w.run_script([(1, {}), (3, {'speed': 0.5}), (0.01, {'speed': 0.5, 'do': 'jump'}), (3.5, {'speed': 0.5}), (1.5, {})], skills=sk, case=case)
    if r['fallen'] or r['landed'] < 1: return -100.0, True
    s = r['segments'][2:4]; clear = max(x['clear_mm'] for x in s)
    return min(clear, 60.0) + 0.2 * sum(x['forward_mm'] for x in s), False


def _step_up(w, sk, case, target):
    """Walk at the first 20 mm step; jump (while walking) when the body is 0.70 m along; keep walking; must stay up on top."""
    w.reset(sk, case); w.pilot.speed = 0.0; jumped = False
    while w.t < 12.0 and w.fallen is None:
        y = -float(w.lab.d.qpos[1])
        w.pilot.speed = 0.5 if w.t > 1.0 and y < 1.6 else 0.0
        if not jumped and y > sk.get('step_trigger', 0.70): w.pilot.request('jump'); jumped = True
        w.tick()
    y = -float(w.lab.d.qpos[1]); z = float(w.lab.d.qpos[2])
    if w.fallen: return -100.0 + 50.0 * min(1.0, max(0.0, y - 0.7) / 0.3), True
    return (100.0 if y > 0.95 and z > 0.17 else 0.0) + 50.0 * min(1.0, max(0.0, y - 0.7) / 0.3), False


def _getup(w, sk, case, target):
    from getup import GetUp
    lab = w.lab; tot = 0.0; fails = 0
    for direction in ('backward', 'left', 'right'):
        w.reset(sk, dict(case, direction=direction, force=12.0)); w.pilot.speed = 0
        while w.t < 3.0:
            lab.e.step(); w.i += 1
            if w.i % 10 == 0: w.pilot(lab, w.t)
        g = GetUp(lab, sk.get('getup')); g.start(lab, w.t); stage = ''
        while w.t < 11.0 and stage != 'stood':
            if w.i % 10 == 0: stage = g(lab, w.t)
            lab.e.step(); w.i += 1
        if stage == 'stood':
            w.pilot.start(lab); t1 = w.t
            while w.t < t1 + 2.0:
                if w.i % 10 == 0: w.pilot(lab, w.t)
                w.pilot.substep(lab); lab.e.step(); w.i += 1
        R = lab.d.xmat[1].reshape(3, 3); tilt = float(np.degrees(np.arccos(np.clip(R[2, 2], -1, 1))))
        ok = stage == 'stood' and tilt < 20
        tot += (100.0 if ok else 0.0) - tilt; fails += not ok
    return tot / 3, fails > 0


# task -> (description, unit of the score, [(skill path, low, high)], scorer, default target)
TASKS = {
 'walk':      ('Walking speed (the normal walk, UP key)', 'mm/s', [('walk.f', 2.5, 4.5), ('walk.s', 0.02, 0.045), ('walk.lift', 0.008, 0.02), ('walk.back', 0.01, 0.03), ('walk.duty', 0.5, 0.7)], _walk_score(0.5), None),
 'fast_walk': ('Fast walk speed (SHIFT+UP)', 'mm/s', [('run.f', 3.0, 5.0), ('run.s', 0.03, 0.06), ('run.lift', 0.008, 0.022), ('run.back', 0.01, 0.035), ('run.duty', 0.5, 0.7)], _walk_score(1.0), None),
 'steer':     ('Steering while walking: degrees turned in 5 s', 'deg', [('turn_gain', 0.05, 0.5), ('walk.lift', 0.008, 0.02), ('walk.back', 0.01, 0.03)], _steer, None),
 'pivot':     ('Turning on the spot (experimental, off by default)', 'deg', [('pivot_stride', 0.004, 0.03), ('walk.lift', 0.008, 0.02)], _pivot, None),
 'reverse':   ('Walking backwards (experimental, off by default)', 'mm/s', [('reverse_scale', 0.1, 0.8), ('walk.lift', 0.008, 0.02)], _reverse, None),
 'jump_height': ('Jump to a target clearance in mm (--target, default 80)', 'mm error', [('hops.jump.E_scale', 0.8, 1.5), ('hops.jump.hip_bias', -0.35, 0.35), ('hops.jump.dy_c', -0.05, -0.02), ('hops.jump.dy_l', -0.04, -0.005)], _hop('jump', 'clear'), 80.0),
 'moving_jump': ('Jump while walking without stopping, then keep walking (ENTER while UP is held)', 'points', [('hops.jump_moving.E_scale', 0.7, 1.4), ('hops.jump_moving.hip_bias', -0.35, 0.35), ('hops.jump_moving.dy_c', -0.05, 0.0), ('hops.jump_moving.dy_l', -0.04, 0.0), ('hops.jump_moving.t_c', 0.15, 0.5), ('hops.jump_moving.t_h', 0.05, 0.3), ('post_hop_settle_s', 0.3, 1.2)], _moving_jump, None),
 'step_up':   ('Jump up the first 20 mm step while walking and stay up on top', 'points', [('hops.jump_moving.E_scale', 0.7, 1.4), ('hops.jump_moving.hip_bias', -0.35, 0.35), ('hops.jump_moving.dy_c', -0.05, 0.0), ('hops.jump_moving.dy_l', -0.04, 0.0), ('hops.jump_moving.t_c', 0.15, 0.5), ('step_trigger', 0.6, 0.8)], _step_up, None),
 'getup':     ('Get back up after falling backwards, left or right', 'points', [('getup.crouch_thigh', 0.0, 1.0), ('getup.crouch_knee', 0.3, 1.13), ('getup.done_tilt_deg', 25.0, 45.0), ('getup.rise_time', 0.8, 3.5), ('getup.rise_kp', 0.3, 3.0), ('getup.rise_kd', 0.0, 0.4), ('getup.push_flap', -0.54, 0.0), ('getup.sit_head', -0.35, 0.5)], _getup, None),
 'hop_distance': ('Far hop to a target distance in mm (--target, default 80)', 'mm error', [('hops.hop_far.E', 0.2, 0.45), ('hops.hop_far.dy_c', -0.02, 0.02), ('hops.hop_far.dy_e', -0.02, 0.02), ('hops.hop_far.dy_l', -0.03, 0.01)], _hop('hop_far', 'forward'), 80.0),
}


def get(sk, path):
    o = sk
    for k in path.split('.'): o = o.get(k, 1.0 if k == 'E_scale' else 0.0) if isinstance(o, dict) else o
    return float(o)


def put(sk, path, v):
    ks = path.split('.'); o = sk
    for k in ks[:-1]: o = o[k]
    o[ks[-1]] = float(v)


def evaluate(w, sk, task, target, conds):
    scorer = TASKS[task][3]; vals = []; falls = 0
    for c in conds:
        s, fell = scorer(w, sk, c, target); vals.append(s); falls += fell
    return float(np.mean(vals)) - 60.0 * falls, falls, vals


def routine_check(w, sk):
    from selftest import ROUTINE
    out = []
    for name, c in CHECK:
        r = w.run_script(ROUTINE, skills=sk, case=c); out.append((name, r['fallen'], r['landed'], r['forward_mm']))
    return out


def main():
    ap = argparse.ArgumentParser(); ap.add_argument('cmd'); ap.add_argument('arg', nargs='?')
    ap.add_argument('--gens', type=int, default=6); ap.add_argument('--pop', type=int, default=8); ap.add_argument('--target', type=float, default=None); ap.add_argument('--seed', type=int, default=0)
    a = ap.parse_args()
    if a.cmd == 'list':
        for k, v in TASKS.items(): print(f'{k:13} {v[0]}  [{", ".join(p[0] for p in v[2])}]')
        return
    from world import World, load_skills
    skf = GAME / 'skills.json'
    if a.cmd == 'restore':
        b = sorted(BACK.glob('skills_*.json'))
        if not b: print('no backup found'); return
        shutil.copy(b[-1], skf); print(f'restored {b[-1].name} -> skills.json'); return
    if a.cmd == 'promote':
        c = json.loads(Path(a.arg).read_text()); BACK.mkdir(exist_ok=True)
        shutil.copy(skf, BACK / f'skills_{time.strftime("%Y%m%d_%H%M%S")}.json')
        sk = load_skills()
        for path, v in c['settings'].items(): put(sk, path, v)
        sk.setdefault('sources', {})[c['task']] = f"trained {c['when']}: score {c['score']:.1f} {c['unit']}, task check {c.get('validated')}, full routine {c.get('routine')}"
        skf.write_text(json.dumps(sk, indent=1)); print(f"promoted {Path(a.arg).name} into skills.json (backup saved in skills_backups)"); return
    w = World()
    if a.cmd == 'validate':
        sk = load_skills()
        if a.arg:
            for path, v in json.loads(Path(a.arg).read_text())['settings'].items(): put(sk, path, v)
        res = routine_check(w, sk)
        for name, fell, landed, fwd in res: print(f'  {name:18} {"FELL at %.1f s" % fell if fell else "completed the routine"}  hops/jumps landed {landed}, forward {fwd / 1000:.2f} m', flush=True)
        print(f'RESULT: {sum(r[1] is None for r in res)}/{len(res)} conditions completed the full 45 s routine'); return
    if a.cmd != 'run' or a.arg not in TASKS:
        print(__doc__); return
    task = a.arg; desc, unit, params, _, tdef = TASKS[task]; target = a.target if a.target is not None else tdef
    rng = np.random.default_rng(a.seed); base = load_skills(); CAND.mkdir(exist_ok=True)
    lo = np.array([p[1] for p in params]); hi = np.array([p[2] for p in params])
    mu = np.clip(np.array([get(base, p[0]) for p in params]), lo, hi); sd = (hi - lo) * 0.2
    s0, f0, _ = evaluate(w, base, task, target, SEARCH)
    print(f'TASK {task}: {desc}' + (f' (target {target:g})' if target is not None else ''))
    print(f'start: score {s0:.1f} {unit}, falls {f0}/{len(SEARCH)}   settings ' + ', '.join(f'{p[0]}={m:.4g}' for p, m in zip(params, mu)), flush=True)
    best = (s0, mu.copy(), f0); stamp = time.strftime('%Y%m%d_%H%M%S'); out = CAND / f'{task}_{stamp}.json'; hist = [s0]
    for g in range(a.gens):
        X = np.clip(mu + sd * rng.standard_normal((a.pop, len(mu))), lo, hi); X[0] = best[1]; sc = []
        for x in X:
            sk = copy.deepcopy(base)
            for p, v in zip(params, x): put(sk, p[0], v)
            s, falls, _ = evaluate(w, sk, task, target, SEARCH); sc.append(s)
            if s > best[0]: best = (s, x.copy(), falls)
        order = np.argsort(sc)[::-1][:max(2, a.pop // 4)]
        mu = X[order].mean(axis=0); sd = np.maximum(X[order].std(axis=0), (hi - lo) * 0.02); hist.append(best[0])
        print(f'generation {g + 1}/{a.gens}: best score {best[0]:.1f} {unit} (falls {best[2]}/{len(SEARCH)}), this round best {max(sc):.1f}, average {np.mean(sc):.1f}', flush=True)
        out.write_text(json.dumps({'task': task, 'description': desc, 'unit': unit, 'target': target, 'when': time.strftime('%Y-%m-%d %H:%M'), 'score': best[0], 'start_score': s0,
                                   'settings': {p[0]: float(v) for p, v in zip(params, best[1])}, 'history': hist, 'generation': g + 1, 'validated': None, 'routine': None}, indent=1))
    sk = copy.deepcopy(base)
    for p, v in zip(params, best[1]): put(sk, p[0], v)
    s5, f5, vals = evaluate(w, sk, task, target, [c for _, c in CHECK])
    rc = routine_check(w, sk); nr = sum(r[1] is None for r in rc)
    c = json.loads(out.read_text()); c.update(validated=f'{len(CHECK) - f5}/{len(CHECK)} conditions without a fall, score {s5:.1f}', routine=f'{nr}/{len(rc)} conditions completed the full routine', done=True)
    out.write_text(json.dumps(c, indent=1))
    print(f'CHECK on 5 conditions: {len(CHECK) - f5}/{len(CHECK)} without a fall, score {s5:.1f} {unit} (was {s0:.1f} at the start on 3 conditions)')
    print(f'FULL ROUTINE with the new settings: {nr}/{len(rc)} conditions completed')
    print(f'saved candidate: {out.name}   (not in use until promoted)', flush=True)


if __name__ == '__main__':
    main()
