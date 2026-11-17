"""Sparrow Studio - one window to play, train and check the bird. Starts the other scripts in this folder as separate
processes; never more than MAX_JOBS training/check jobs at once and none while the PC is above the CPU/RAM guard."""
import ctypes, json, os, subprocess, sys, time, tkinter as tk
from pathlib import Path
from tkinter import ttk, messagebox

GAME = Path(__file__).resolve().parent; WS = GAME.parent
PY = sys.executable.replace('pythonw.exe', 'python.exe')
MAX_JOBS = 2; GUARD = 85.0
LOGS = GAME / 'logs'; LOGS.mkdir(exist_ok=True); CAND = GAME / 'candidates'; CAND.mkdir(exist_ok=True)
sys.path.insert(0, str(GAME))


class Load:
    """CPU and RAM use of the whole PC (Windows API, no extra packages)."""
    class _Mem(ctypes.Structure):
        _fields_ = [('len', ctypes.c_ulong), ('load', ctypes.c_ulong)] + [(n, ctypes.c_ulonglong) for n in ('tp', 'ap', 'tpf', 'apf', 'tv', 'av', 'aev')]

    def __init__(self):
        self.prev = self._times(); self.cpu = 0.0

    def _times(self):
        i, k, u = ctypes.c_ulonglong(), ctypes.c_ulonglong(), ctypes.c_ulonglong()
        ctypes.windll.kernel32.GetSystemTimes(ctypes.byref(i), ctypes.byref(k), ctypes.byref(u)); return i.value, k.value, u.value

    def read(self):
        now = self._times(); di, dk, du = (now[j] - self.prev[j] for j in range(3)); self.prev = now
        if dk + du > 0: self.cpu = 100.0 * (1 - di / (dk + du))
        m = Load._Mem(); m.len = ctypes.sizeof(m); ctypes.windll.kernel32.GlobalMemoryStatusEx(ctypes.byref(m))
        return self.cpu, float(m.load)


class Studio(tk.Tk):
    def __init__(self):
        super().__init__(); self.title('Sparrow Studio'); self.geometry('980x700'); self.minsize(820, 560)
        self.load = Load(); self.jobs = []; self.shown = None; self.pos = 0
        from train import TASKS
        self.tasks = TASKS
        top = ttk.Frame(self, padding=8); top.pack(fill='x')
        ttk.Label(top, text='Sparrow Studio', font=('Segoe UI', 16, 'bold')).pack(side='left')
        self.status = ttk.Label(top, text=''); self.status.pack(side='right')
        nb = ttk.Notebook(self); nb.pack(fill='both', expand=True, padx=8, pady=(0, 8))

        # ---------------- Play
        p = ttk.Frame(nb, padding=12); nb.add(p, text='Play')
        ttk.Label(p, text='Drive the bird yourself. Click the 3D window, then:', font=('Segoe UI', 11)).pack(anchor='w')
        ttk.Label(p, justify='left', font=('Consolas', 10), text=(
            '  UP walk      SHIFT+UP fast walk      LEFT / RIGHT steer\n  SPACE hop    SHIFT+SPACE far hop\n  ENTER jump   SHIFT+ENTER high jump\n'
            '  R reset      C camera                ESC quit\n  Xbox-style controller: left stick, A hop, X far hop, B jump, Y high jump, START reset')).pack(anchor='w', pady=8)
        row = ttk.Frame(p); row.pack(anchor='w', pady=6)
        ttk.Label(row, text='Floor grip').pack(side='left'); self.fr = tk.DoubleVar(value=1.0)
        ttk.Combobox(row, textvariable=self.fr, values=(0.5, 0.7, 1.0, 1.4), width=5).pack(side='left', padx=(4, 16))
        ttk.Label(row, text='Robot weight').pack(side='left'); self.ms = tk.DoubleVar(value=1.0)
        ttk.Combobox(row, textvariable=self.ms, values=(0.93, 1.0, 1.07, 1.15), width=5).pack(side='left', padx=4)
        prow = ttk.Frame(p); prow.pack(anchor='w', pady=10)
        ttk.Button(prow, text='PLAY', command=self.play).pack(side='left', ipadx=30, ipady=8)
        ttk.Button(prow, text='PLAY hip-twist prototype (turns properly)', command=lambda: self.play(GAME / 'skills_hipyaw.json')).pack(side='left', padx=12, ipadx=10, ipady=8)
        self.skill_txt = tk.Text(p, height=14, wrap='word', font=('Segoe UI', 9)); self.skill_txt.pack(fill='both', expand=True)

        # ---------------- Train
        t = ttk.Frame(nb, padding=12); nb.add(t, text='Train')
        r1 = ttk.Frame(t); r1.pack(fill='x')
        ttk.Label(r1, text='Skill to improve').pack(side='left'); self.task = tk.StringVar(value='fast_walk')
        cb = ttk.Combobox(r1, textvariable=self.task, values=list(TASKS), width=14, state='readonly'); cb.pack(side='left', padx=6); cb.bind('<<ComboboxSelected>>', lambda e: self.show_task())
        ttk.Label(r1, text='Rounds').pack(side='left', padx=(12, 0)); self.gens = tk.IntVar(value=6); ttk.Spinbox(r1, from_=1, to=40, textvariable=self.gens, width=4).pack(side='left', padx=4)
        ttk.Label(r1, text='Tries per round').pack(side='left', padx=(12, 0)); self.pop = tk.IntVar(value=8); ttk.Spinbox(r1, from_=4, to=24, textvariable=self.pop, width=4).pack(side='left', padx=4)
        ttk.Label(r1, text='Target (mm)').pack(side='left', padx=(12, 0)); self.target = tk.StringVar(value=''); ttk.Entry(r1, textvariable=self.target, width=6).pack(side='left', padx=4)
        ttk.Button(r1, text='Start training', command=self.train).pack(side='left', padx=12)
        self.task_lbl = ttk.Label(t, text='', wraplength=900, justify='left'); self.task_lbl.pack(anchor='w', pady=6)
        ttk.Label(t, text='Trained candidates (nothing changes in the game until you promote one):').pack(anchor='w', pady=(8, 2))
        self.cl = tk.Listbox(t, height=8, font=('Consolas', 9)); self.cl.pack(fill='x')
        r2 = ttk.Frame(t); r2.pack(fill='x', pady=6)
        for txt, fn in (('Refresh', self.refresh), ('Try it in the game', self.try_cand), ('Check it (5 conditions)', self.check_cand), ('Promote to the game', self.promote), ('Undo last promote', self.restore)):
            ttk.Button(r2, text=txt, command=fn).pack(side='left', padx=(0, 6))

        # ---------------- Checks
        c = ttk.Frame(nb, padding=12); nb.add(c, text='Checks')
        ttk.Button(c, text='Check the whole routine on 5 conditions (about 3 minutes)', command=lambda: self.start('check', ['train.py', 'validate'])).pack(anchor='w', pady=4)
        ttk.Button(c, text='Quick self-test (one condition, with details)', command=lambda: self.start('selftest', ['selftest.py'])).pack(anchor='w', pady=4)
        ttk.Button(c, text='Verify the original project files are untouched', command=self.verify).pack(anchor='w', pady=4)
        ttk.Label(c, wraplength=900, justify='left', text=(
            'The routine is 45 simulated seconds: walk, hop, walk left, walk right, jump, fast walk, high jump, hop while walking. '
            'A condition passes only if the bird never falls. Everything here is simulation with assumed servo behaviour; it is not yet a test of real hardware.')).pack(anchor='w', pady=10)

        # ---------------- log + jobs (always visible)
        bot = ttk.Frame(self, padding=(8, 0, 8, 8)); bot.pack(fill='both', expand=True)
        jr = ttk.Frame(bot); jr.pack(fill='x')
        self.job_lbl = ttk.Label(jr, text='No job running'); self.job_lbl.pack(side='left')
        ttk.Button(jr, text='Stop job', command=self.stop).pack(side='right')
        self.log = tk.Text(bot, height=12, font=('Consolas', 9), bg='#111', fg='#ddd'); self.log.pack(fill='both', expand=True, pady=4)
        self.show_task(); self.refresh(); self.show_skills(); self.after(500, self.poll)

    # ---------------------------------------------------------------- helpers
    def say(self, s):
        self.log.insert('end', s + '\n'); self.log.see('end')

    def running(self):
        self.jobs = [j for j in self.jobs if j['p'].poll() is None or not j.get('reported')]
        return [j for j in self.jobs if j['p'].poll() is None]

    def start(self, name, argv):
        cpu, ram = self.load.read()
        if len(self.running()) >= MAX_JOBS:
            messagebox.showinfo('Sparrow Studio', f'{MAX_JOBS} jobs are already running. Wait for one to finish or stop it.'); return
        if cpu > GUARD or ram > GUARD:
            messagebox.showinfo('Sparrow Studio', f'The PC is busy (CPU {cpu:.0f}%, memory {ram:.0f}%). Jobs only start below {GUARD:.0f}%.'); return
        lf = LOGS / f'{name}_{time.strftime("%Y%m%d_%H%M%S")}.log'; fh = open(lf, 'w')
        env = dict(os.environ, OMP_NUM_THREADS='1', PYTHONUNBUFFERED='1')
        p = subprocess.Popen([PY] + argv, cwd=GAME, stdout=fh, stderr=subprocess.STDOUT, env=env, creationflags=getattr(subprocess, 'CREATE_NO_WINDOW', 0))
        j = {'name': name, 'p': p, 'log': lf, 'fh': fh}; self.jobs.append(j); self.shown = j; self.pos = 0
        self.log.delete('1.0', 'end'); self.say(f'started: {name}   (log file: logs/{lf.name})')

    def stop(self):
        if self.shown and self.shown['p'].poll() is None:
            self.shown['p'].terminate(); self.say('stopped by you. Anything saved so far is kept.')

    def play(self, skills=None):
        argv = [PY, 'play.py', '--friction', str(self.fr.get()), '--mass', str(self.ms.get())] + (['--skills', str(skills)] if skills else [])
        subprocess.Popen(argv, cwd=GAME, creationflags=getattr(subprocess, 'CREATE_NEW_CONSOLE', 0))

    def train(self):
        argv = ['train.py', 'run', self.task.get(), '--gens', str(self.gens.get()), '--pop', str(self.pop.get())]
        if self.target.get().strip(): argv += ['--target', self.target.get().strip()]
        self.start('train_' + self.task.get(), argv)

    def show_task(self):
        d = self.tasks[self.task.get()]
        est = self.gens.get() * self.pop.get() * 0.2
        self.task_lbl.config(text=f'{d[0]}.  Score is in {d[1]} (higher is better).  Settings it may change: {", ".join(p[0] for p in d[2])}.' + (f'  Default target {d[4]:g} mm.' if d[4] else ''))

    def cands(self):
        out = []
        for f in sorted(CAND.glob('*.json'), reverse=True):
            try: out.append((f, json.loads(f.read_text())))
            except Exception: pass
        return out

    def refresh(self):
        self.cl.delete(0, 'end'); self._c = self.cands()
        for f, c in self._c:
            self.cl.insert('end', f"{c.get('when', ''):16}  {c['task']:12}  score {c['score']:7.1f} {c['unit']:8} (started {c.get('start_score', 0):.1f})  check: {c.get('validated') or 'not finished'}  |  {c.get('routine') or ''}")
        self.show_skills()

    def sel(self):
        s = self.cl.curselection()
        if not s: messagebox.showinfo('Sparrow Studio', 'Pick a candidate in the list first.'); return None
        return self._c[s[0]]

    def try_cand(self):
        x = self.sel()
        if not x: return
        from train import put
        sk = json.loads((GAME / 'skills.json').read_text())
        for path, v in x[1]['settings'].items(): put(sk, path, v)
        tmp = GAME / '_try_skills.json'; tmp.write_text(json.dumps(sk, indent=1)); self.play(tmp)

    def check_cand(self):
        x = self.sel()
        if x: self.start('check_' + x[1]['task'], ['train.py', 'validate', str(x[0])])

    def promote(self):
        x = self.sel()
        if x and messagebox.askyesno('Sparrow Studio', f"Use this {x[1]['task']} candidate in the game?\n\nScore {x[1]['score']:.1f} {x[1]['unit']}\nCheck: {x[1].get('validated')}\nFull routine: {x[1].get('routine')}\n\nThe current skills file is backed up first."):
            r = subprocess.run([PY, 'train.py', 'promote', str(x[0])], cwd=GAME, capture_output=True, text=True); self.say(r.stdout.strip() or r.stderr.strip()); self.show_skills()

    def restore(self):
        r = subprocess.run([PY, 'train.py', 'restore'], cwd=GAME, capture_output=True, text=True); self.say(r.stdout.strip() or r.stderr.strip()); self.show_skills()

    def verify(self):
        ps = WS / 'launchers' / 'Verify-Originals.ps1'
        r = subprocess.run(['powershell', '-NoProfile', '-ExecutionPolicy', 'Bypass', '-File', str(ps)], capture_output=True, text=True, creationflags=getattr(subprocess, 'CREATE_NO_WINDOW', 0))
        self.log.delete('1.0', 'end'); self.say(r.stdout[-3000:] or r.stderr[-2000:])
        self.say('Expected: 2 changed (the two Blender files saved on 12 Nov), 0 missing, 0 added.')

    def show_skills(self):
        sk = json.loads((GAME / 'skills.json').read_text()); w, r = sk['walk'], sk['run']
        lines = [f"Skills in use (skills.json) - model {sk.get('variant')}", '',
                 f"walk       {w['f']:.2f} steps/s per leg, stride {w['s'] * 1000:.0f} mm, lift {w['lift'] * 1000:.0f} mm",
                 f"fast walk  {r['f']:.2f} steps/s per leg, stride {r['s'] * 1000:.0f} mm, lift {r['lift'] * 1000:.0f} mm",
                 f"steering   strength {sk.get('turn_gain')}      turn on the spot: {'ON' if sk.get('pivot_stride', 0) > 0 else 'off (experimental)'}      backwards: {'ON' if sk.get('reverse_scale', 0) > 0 else 'off (experimental)'}",
                 'hops       ' + ', '.join(sk['hop_order']), '']
        lines += [f'{k}: {v}' for k, v in sk.get('sources', {}).items()]
        self.skill_txt.delete('1.0', 'end'); self.skill_txt.insert('end', '\n'.join(lines))

    def poll(self):
        cpu, ram = self.load.read(); run = self.running()
        self.status.config(text=f'CPU {cpu:3.0f}%   memory {ram:3.0f}%   jobs {len(run)}/{MAX_JOBS}')
        j = self.shown
        if j:
            try:
                with open(j['log'], 'r', errors='replace') as fh:
                    fh.seek(self.pos); new = fh.read(); self.pos = fh.tell()
                if new: self.log.insert('end', new); self.log.see('end')
            except OSError: pass
            if j['p'].poll() is not None and not j.get('reported'):
                j['reported'] = True; j['fh'].close(); self.say(f"finished: {j['name']} (exit code {j['p'].returncode})"); self.refresh()
            self.job_lbl.config(text=f"{j['name']}: {'running' if j['p'].poll() is None else 'finished'}")
        self.after(700, self.poll)


if __name__ == '__main__':
    Studio().mainloop()
