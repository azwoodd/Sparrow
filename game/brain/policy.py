"""Load a trained brain and run it inside the game or an evaluation (same observations as training)."""
import sys
from pathlib import Path
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from sparrow_env import brain_obs, brain_joints


class Brain:
    def __init__(self, run_dir):
        from stable_baselines3 import PPO
        import pickle
        # model.zip and vecnormalize.pkl are pickle-based (stable-baselines3 format) and can run code when loaded.
        # Only load brains produced by this project's own training runs, never files from unknown sources.
        run = Path(run_dir)
        self.model = PPO.load(str(run / 'model.zip'), device='cpu')
        with open(run / 'vecnormalize.pkl', 'rb') as f:
            vn = pickle.load(f)
        self.mean, self.var, self.clip, self.eps = vn.obs_rms.mean, vn.obs_rms.var, vn.clip_obs, vn.epsilon
        self.na = self.model.action_space.shape[0]

    def act(self, obs):
        o = np.clip((obs - self.mean) / np.sqrt(self.var + self.eps), -self.clip, self.clip)
        a, _ = self.model.predict(o.astype(np.float32), deterministic=True)
        return np.asarray(a, np.float32)


def latest_run(task):
    runs = sorted((HERE / 'runs').glob(f'{task}_*/model.zip'))
    return runs[-1].parent if runs else None
