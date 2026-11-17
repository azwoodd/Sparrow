"""Judge a trained brain against the scripted controller, with FULL physics (air forces on) and conditions it has not
trained on (seeds 10000+).   python evaluate.py <task> [run_dir] [--episodes 20]
Prints success and fall rates for: scripted controller alone, and scripted controller + brain (or brain alone for getup).
Writes runs/<run>/evaluation.json."""
import argparse, json, sys
from pathlib import Path
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from sparrow_env import SparrowEnv
from policy import Brain, latest_run


def run_episodes(env, policy, n, seed0=10000):
    succ = fell = 0; rets = []
    for k in range(n):
        o, _ = env.reset(seed=seed0 + k); done = False; R = 0.0; info = {}
        while not done:
            a = policy(o); o, r, te, tr, info = env.step(a); R += r; done = te or tr
        succ += bool(info.get('success')); fell += bool(info.get('fell')); rets.append(R)
    return succ / n, fell / n, float(np.mean(rets))


def scripted_getup(env):
    """Baseline for getup: the hand-written GetUp controller, driven through the same environment."""
    from getup import GetUp
    state = {}
    def pol(o):
        lab = env.lab
        if 'g' not in state or env.steps == 0:
            state['g'] = GetUp(lab, env.sk.get('getup')); state['g'].start(lab, env.w.t)
        state['g'](lab, env.w.t)
        want = lab.d.ctrl[env.acts]
        return np.clip(2 * (want - env.lo) / (env.hi - env.lo) - 1, -1, 1).astype(np.float32)
    return pol, state


def main():
    ap = argparse.ArgumentParser(); ap.add_argument('task'); ap.add_argument('run', nargs='?'); ap.add_argument('--episodes', type=int, default=20)
    ap.add_argument('--no-aero', action='store_true')
    a = ap.parse_args()
    run = Path(a.run) if a.run else latest_run(a.task)
    env = SparrowEnv(a.task, aero=not a.no_aero)
    if a.task == 'getup':
        pol, st = scripted_getup(env)
        def base(o):
            if env.steps == 0: st.clear()
            return pol(o)
    else:
        base = lambda o: np.zeros(env.action_space.shape, np.float32) if a.task == 'recover' else np.r_[np.zeros(6), 1.0 if env.steps > 150 else -1.0].astype(np.float32)
    b = run_episodes(env, base, a.episodes)
    out = {'task': a.task, 'episodes': a.episodes, 'air_forces': not a.no_aero, 'scripted': dict(zip(('success', 'fell', 'return'), b))}
    print(f'{a.task}: scripted controller alone   success {b[0]:.0%}  fell {b[1]:.0%}  return {b[2]:8.1f}', flush=True)
    if run:
        brain = Brain(run); r = run_episodes(env, brain.act, a.episodes)
        out['brain'] = dict(zip(('success', 'fell', 'return'), r)); out['run'] = str(run)
        print(f'{a.task}: with the brain ({run.name})  success {r[0]:.0%}  fell {r[1]:.0%}  return {r[2]:8.1f}', flush=True)
        (run / 'evaluation.json').write_text(json.dumps(out, indent=1))
    else:
        print('no trained run found')


if __name__ == '__main__':
    main()
