"""Train a Sparrow brain with PPO (stable-baselines3). Runs on a laptop or a many-core cloud machine.

  python train_brain.py getup   --steps 2e6 --envs 2
  python train_brain.py recover --steps 5e6 --envs 32          # cloud
  python train_brain.py getup   --resume runs/getup_YYYYmmdd_HHMMSS   # continue a run
Workers default to env SPARROW_WORKERS (else 2). Each run writes runs/<task>_<stamp>/: progress.csv (one line per update:
steps, mean episode return, success rate, fall rate, steps/s, CPU and RAM use), checkpoints, final model.zip and
vecnormalize.pkl. evaluate.py judges a run with full physics. If the machine passes --guard percent CPU or RAM the
trainer pauses until it drops (default 90)."""
import argparse, csv, json, os, sys, time
from pathlib import Path
os.environ.setdefault('OMP_NUM_THREADS', '1')
import numpy as np, torch
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import BaseCallback, CheckpointCallback
from stable_baselines3.common.vec_env import SubprocVecEnv, DummyVecEnv, VecNormalize, VecMonitor

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from sparrow_env import make_env, TASKS


def machine_load():
    try:
        import psutil
        return psutil.cpu_percent(interval=None), psutil.virtual_memory().percent
    except ImportError:
        if sys.platform == 'win32':
            import ctypes
            class M(ctypes.Structure):
                _fields_ = [('l', ctypes.c_ulong), ('load', ctypes.c_ulong)] + [(n, ctypes.c_ulonglong) for n in 'abcdefg']
            m = M(); m.l = ctypes.sizeof(m); ctypes.windll.kernel32.GlobalMemoryStatusEx(ctypes.byref(m))
            return float('nan'), float(m.load)
        mem = dict(l.split(':') for l in open('/proc/meminfo').read().splitlines()[:3])
        tot = float(mem['MemTotal'].split()[0]); av = float(mem['MemAvailable'].split()[0])
        return os.getloadavg()[0] / os.cpu_count() * 100, 100 * (1 - av / tot)


class Progress(BaseCallback):
    def __init__(self, path, guard):
        super().__init__(); self.path = path; self.guard = guard; self.t0 = time.time(); self.n0 = 0
        self.ep = []; self.succ = []; self.fell = []
        self.f = open(path, 'a', newline=''); self.w = csv.writer(self.f)
        if path.stat().st_size == 0: self.w.writerow(['steps', 'episodes', 'mean_return', 'success_rate', 'fall_rate', 'steps_per_s', 'cpu_pct', 'ram_pct'])

    def _on_step(self):
        for info in self.locals['infos']:
            if 'episode' in info:
                self.ep.append(info['episode']['r']); self.succ.append(bool(info.get('success'))); self.fell.append(bool(info.get('fell')))
        return True

    def _on_rollout_end(self):
        cpu, ram = machine_load(); n = self.num_timesteps; sps = (n - self.n0) / max(time.time() - self.t0, 1e-6); self.t0 = time.time(); self.n0 = n
        k = self.ep[-100:]
        row = [n, len(self.ep), float(np.mean(k)) if k else float('nan'), float(np.mean(self.succ[-100:])) if k else float('nan'),
               float(np.mean(self.fell[-100:])) if k else float('nan'), round(sps), round(cpu, 1) if cpu == cpu else '', round(ram, 1)]
        self.w.writerow(row); self.f.flush()
        print(f'steps {n:>9,}  episodes {len(self.ep):>6}  return {row[2]:8.2f}  success {row[3]:.2f}  fell {row[4]:.2f}  {sps:5.0f} steps/s  RAM {ram:.0f}%', flush=True)
        while ram > self.guard:
            print(f'  memory {ram:.0f}% above the {self.guard:.0f}% guard - pausing 30 s', flush=True); time.sleep(30); cpu, ram = machine_load()


def main():
    ap = argparse.ArgumentParser(); ap.add_argument('task', choices=TASKS)
    ap.add_argument('--steps', type=float, default=1e6); ap.add_argument('--envs', type=int, default=int(os.environ.get('SPARROW_WORKERS', 2)))
    ap.add_argument('--resume', default=None); ap.add_argument('--seed', type=int, default=0); ap.add_argument('--guard', type=float, default=90.0)
    ap.add_argument('--aero', action='store_true', help='train with air forces on (slow; default off for ground tasks)')
    a = ap.parse_args()
    torch.set_num_threads(1)
    run = Path(a.resume) if a.resume else HERE / 'runs' / f'{a.task}_{time.strftime("%Y%m%d_%H%M%S")}'
    run.mkdir(parents=True, exist_ok=True)
    fns = [make_env(a.task, i, a.aero, a.seed) for i in range(a.envs)]
    venv = VecMonitor(SubprocVecEnv(fns) if a.envs > 1 else DummyVecEnv(fns), info_keywords=('success', 'fell'))
    if a.resume:
        venv = VecNormalize.load(str(run / 'vecnormalize.pkl'), venv); venv.training = True
        model = PPO.load(str(run / 'model.zip'), env=venv, device='cpu')
    else:
        venv = VecNormalize(venv, norm_obs=True, norm_reward=True, clip_obs=10.0)
        n_steps = max(256, 4096 // a.envs)
        model = PPO('MlpPolicy', venv, n_steps=n_steps, batch_size=min(4096, n_steps * a.envs) // 4, n_epochs=10, learning_rate=3e-4,
                    gamma=0.99, gae_lambda=0.95, clip_range=0.2, ent_coef=0.0, policy_kwargs=dict(net_arch=[256, 256], log_std_init=-0.5 if a.task == 'getup' else -1.6),
                    seed=a.seed, device='cpu', verbose=0)
        (run / 'config.json').write_text(json.dumps({'task': a.task, 'envs': a.envs, 'seed': a.seed, 'aero': a.aero, 'started': time.strftime('%Y-%m-%d %H:%M')}, indent=1))
    cb = [Progress(run / 'progress.csv', a.guard), CheckpointCallback(max(50_000 // a.envs, 1), str(run / 'checkpoints'), name_prefix='model', save_vecnormalize=True)]
    print(f'training {a.task} for {int(a.steps):,} steps on {a.envs} workers -> {run}', flush=True)
    try:
        model.learn(total_timesteps=int(a.steps), callback=cb, reset_num_timesteps=not a.resume, progress_bar=False)
    finally:
        model.save(str(run / 'model.zip')); venv.save(str(run / 'vecnormalize.pkl')); venv.close()
        print(f'saved {run / "model.zip"}', flush=True)


if __name__ == '__main__':
    main()
