"""Sparrow Locomotion Engineering V1 - shared lab: isolated model loading + trustworthy contact instrumentation.

* Loads ONLY the copies inside this workspace (model/<variant>/ and src/vendor/), never the originals.
* vendor/control.py is a byte-identical copy of the owner's balance controller (Experiment class); it is imported unmodified.
* World convention (from the model): X = lateral (left foot at +X), bird faces -Y (forward = -Y), Z up.
"""
import csv, json, math, os, sys
from pathlib import Path
import numpy as np

WS = Path(__file__).resolve().parents[1]
sys.dont_write_bytecode = True
VARIANT = os.environ.get('SPARROW_VARIANT', 'v7e_baseline')
MODEL_DIR = WS / 'model' / VARIANT
os.environ['SPARROW_SOURCE'] = str(MODEL_DIR)
sys.path.insert(0, str(WS / 'src' / 'vendor'))
import mujoco                                   # noqa: E402
from control import Experiment                  # noqa: E402  (owner's controller, unmodified copy)

GAINS = tuple(json.loads((WS / 'src' / 'vendor' / 'gains.json').read_text()))
G = 9.81
FALL_TILT_DEG = 45.0
FALL_COM_Z = 0.09


class Lab:
    """One simulated robot + per-step measurement of everything needed to judge real steps."""

    def __init__(self):
        self.e = Experiment()
        self.s = self.e.s
        self.m, self.d = self.s.m, self.s.d
        m = self.m
        self.dt = float(m.opt.timestep)
        self.floor = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, 'floor')
        self.foot_geom = {S: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, 'col_foot_' + S) for S in 'LR'}
        self.geom_name = {g: mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g) for g in range(m.ngeom)}
        self.weight = float(self.s.total_mass) * G
        self.leg_names = list(self.e.legs)
        self.ids, self.qi, self.vi = self.e.ids, self.e.qi, self.e.vi
        self.frc_lim = np.max(np.abs(m.actuator_forcerange[self.ids]), axis=1)
        self.act = self.s.act
        self.rows = []
        self._corner_local = {}
        for S, g in self.foot_geom.items():
            hx, hy, hz = m.geom_size[g]
            self._corner_local[S] = np.array([[sx * hx, sy * hy, sz * hz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])

    # ------------------------------------------------------------------ setup
    def reset(self, case=None):
        self.e.reset(dict(case or {}))
        for n in ('jump_spring_L', 'jump_spring_R'):
            self.d.ctrl[self.act[n]] = 0.0
        self.weight = float(self.m.body_subtreemass[1]) * G
        self.rows = []
        self.start = self.s.com().copy()
        self.foot_start = {S: self.d.geom_xpos[g].copy() for S, g in self.foot_geom.items()}

    # ------------------------------------------------------------------ measurement
    def contact_state(self):
        """Per-geom ground reaction: normal force (N), tangential force (N), centre of pressure, contact count."""
        d, m = self.d, self.m
        out = {}
        f6 = np.zeros(6)
        for i in range(d.ncon):
            c = d.contact[i]
            if c.geom1 == self.floor:
                g = c.geom2
            elif c.geom2 == self.floor:
                g = c.geom1
            else:
                continue
            mujoco.mj_contactForce(m, d, i, f6)
            fn = float(f6[0])
            rec = out.setdefault(g, {'fn': 0.0, 'ft': np.zeros(2), 'mom': np.zeros(3), 'n': 0, 'pen': 0.0})
            rec['n'] += 1
            rec['pen'] = min(rec['pen'], float(c.dist))
            if fn > 0:
                fr = np.array(c.frame).reshape(3, 3)
                rec['fn'] += fn
                rec['ft'] += (f6[1] * fr[1] + f6[2] * fr[2])[:2]
                rec['mom'] += fn * np.array(c.pos)
        return out

    def foot_pose(self, S):
        g = self.foot_geom[S]
        R = self.d.geom_xmat[g].reshape(3, 3)
        p = self.d.geom_xpos[g]
        corners = p + self._corner_local[S] @ R.T
        yaw = math.atan2(-R[0, 1], R[1, 1])           # rotation of the foot long axis (local Y) about world Z
        tilt = math.degrees(math.acos(float(np.clip(R[2, 2], -1, 1))))
        return p.copy(), corners, yaw, tilt, R

    def measure(self):
        s, d = self.s, self.d
        p, r, v = self.e.state()
        com = s.com()
        cs = self.contact_state()
        tilt = math.degrees(math.acos(float(np.clip(s.torso_up(), -1, 1))))
        row = {'t': float(d.time), 'pitch_deg': math.degrees(p), 'roll_deg': math.degrees(r), 'tilt_deg': tilt,
               'com_x': com[0], 'com_y': com[1], 'com_z': com[2], 'root_x': d.qpos[0], 'root_y': d.qpos[1], 'root_z': d.qpos[2]}
        cv = np.zeros(6)
        row['com_vx'], row['com_vy'], row['com_vz'] = (d.subtree_linvel[1] if hasattr(d, 'subtree_linvel') else cv[:3])
        tot_f, tot_m, other_f, other = 0.0, np.zeros(3), 0.0, []
        for S, g in self.foot_geom.items():
            rec = cs.get(g)
            pos, corners, yaw, ftilt, R = self.foot_pose(S)
            fn = rec['fn'] if rec else 0.0
            row['fz_' + S] = fn
            row['ncon_' + S] = rec['n'] if rec else 0
            row['ft_' + S] = float(np.linalg.norm(rec['ft'])) if rec else 0.0
            cop = rec['mom'] / fn if rec and fn > 1e-9 else np.full(3, np.nan)
            row['cop_x_' + S], row['cop_y_' + S] = cop[0], cop[1]
            row['foot_x_' + S], row['foot_y_' + S], row['foot_z_' + S] = pos
            row['clear_min_' + S] = float(corners[:, 2].min())      # lowest point of the pad above ground (m)
            row['clear_max_' + S] = float(corners[:, 2].max())
            row['foot_yaw_deg_' + S] = math.degrees(yaw)
            row['foot_tilt_deg_' + S] = ftilt
            # COM ground projection expressed in the foot pad frame, and signed margin inside the pad rectangle (>0 = inside)
            loc = R.T @ (np.array([com[0], com[1], pos[2]]) - pos)
            hx, hy = self.m.geom_size[g][:2]
            row['com_in_pad_x_' + S], row['com_in_pad_y_' + S] = loc[0], loc[1]
            row['com_margin_' + S] = float(min(hx - abs(loc[0]), hy - abs(loc[1])))
            if rec:
                tot_f += fn; tot_m += rec['mom']
        for g, rec in cs.items():
            if g not in self.foot_geom.values() and rec['fn'] > 0:
                other_f += rec['fn']; other.append(self.geom_name[g]); tot_f += rec['fn']; tot_m += rec['mom']
        row['fz_other'] = other_f
        row['other_contacts'] = '+'.join(sorted(other))
        row['cop_x'], row['cop_y'] = (tot_m[:2] / tot_f) if tot_f > 1e-9 else (np.nan, np.nan)
        f = d.actuator_force[self.ids]
        for k, j in enumerate(self.leg_names):
            row[j + '_q'] = d.qpos[self.qi[k]]; row[j + '_tgt'] = d.ctrl[self.ids[k]]; row[j + '_Nm'] = f[k]
        row['n_saturated'] = int(np.sum(np.abs(f) >= 0.99 * self.frc_lim))
        row['spring_L_Nm'] = d.ctrl[self.act['jump_spring_L']]; row['spring_R_Nm'] = d.ctrl[self.act['jump_spring_R']]
        return row

    # ------------------------------------------------------------------ running
    def run(self, controller, duration, case=None, stop_after_fall=0.5, pre_reset=None, every=1):
        """controller(lab, t) is called at 100 Hz and writes d.ctrl through real actuators only. Records every `every` ms."""
        self.reset(case)
        if pre_reset:
            pre_reset(self)
        if hasattr(controller, 'start'):
            controller.start(self)
        fall = None
        n = round(duration / self.dt)
        sub = getattr(controller, 'substep', None)
        for i in range(n):
            if i % 10 == 0:
                controller(self, float(self.d.time))
            if sub:
                sub(self)                      # 1 kHz hook (e.g. passive spring law)
            self.e.step()
            if i % every == 0:
                row = self.measure()
                self.rows.append(row)
                if fall is None and (row['tilt_deg'] > FALL_TILT_DEG or row['com_z'] < FALL_COM_Z):
                    fall = row['t']
            if not np.isfinite(self.d.qpos).all():
                raise RuntimeError('non-finite state')
            if fall is not None and self.d.time > fall + stop_after_fall:
                break
        return fall

    def save_trace(self, path, decimate=5):
        rows = self.rows[::decimate]
        path = Path(path); path.parent.mkdir(parents=True, exist_ok=True)
        with open(path, 'w', newline='') as fh:
            w = csv.DictWriter(fh, fieldnames=list(rows[0])); w.writeheader()
            for r in rows:
                w.writerow({k: (f'{v:.6g}' if isinstance(v, (float, np.floating)) else v) for k, v in r.items()})


# ---------------------------------------------------------------------- owner's feedback controller wrapper
class FeedbackBase:
    """Owner's proven quiet-standing feedback (vendor control.Experiment.command, mode 'feedback', gains.json).
    Subclasses add deltas to the six leg targets; slew limit 0.03 rad per 10 ms tick is re-applied exactly as V1/V3/V4 did."""
    feedback = True

    def start(self, lab):
        self.last = lab.d.ctrl[lab.ids].copy()

    def delta(self, lab, t):
        return np.zeros(6)

    def extra(self, lab, t):
        pass

    def __call__(self, lab, t):
        e, d, m = lab.e, lab.d, lab.m
        if self.feedback:
            e.command('feedback', GAINS)
        else:
            e.command('baseline', GAINS)
        base = d.ctrl[lab.ids].copy()
        tgt = np.clip(base + self.delta(lab, t), m.actuator_ctrlrange[lab.ids, 0], m.actuator_ctrlrange[lab.ids, 1])
        tgt = np.clip(tgt, self.last - 0.03, self.last + 0.03)
        d.ctrl[lab.ids] = tgt
        self.last = tgt.copy()
        self.extra(lab, t)


# ---------------------------------------------------------------------- step / flight event analysis
def arr(rows, key):
    return np.array([r[key] for r in rows], float)


def detect_steps(rows, weight, clear_req=0.005, adv_req=0.010, reload_frac=0.20, reload_s=0.030,
                 stance_frac=0.50, recover_s=1.0, slip_lim=0.005):
    """Strict single-step detector (handoff Phase 1.2). A VALID step by foot F requires, in order:
       1 F fully unloaded (zero ground force)            2 lowest point of F's pad > 5 mm above ground at some instant
       3 while F is off: other foot carries >= 50% body weight on average and NO non-foot part touches the ground
       4 F's pad centre advances >= 10 mm forward (-Y) in the world between lift-off and touch-down
       5 F re-loaded to >= 20% body weight for >= 30 ms continuously
       6 no fall (tilt<45 deg, COM z>0.09 m) from lift-off until `recover_s` after re-load.
       'strict' additionally requires the stance foot to move < 5 mm during the swing (no hidden slide/pivot)."""
    t = arr(rows, 't'); dt = float(np.median(np.diff(t)))
    tilt = arr(rows, 'tilt_deg'); comz = arr(rows, 'com_z'); other = arr(rows, 'fz_other')
    fallen = (tilt > FALL_TILT_DEG) | (comz < FALL_COM_Z)
    first_fall = int(np.argmax(fallen)) if fallen.any() else len(t)
    events = []
    for S, O in (('L', 'R'), ('R', 'L')):
        fz = arr(rows, 'fz_' + S); fo = arr(rows, 'fz_' + O); cl = arr(rows, 'clear_min_' + S)
        fy = arr(rows, 'foot_y_' + S); fx = arr(rows, 'foot_x_' + S)
        oy = arr(rows, 'foot_y_' + O); ox = arr(rows, 'foot_x_' + O)
        off = fz <= 0.0
        i = 0
        while i < len(t):
            if not off[i] or (i > 0 and off[i - 1]):
                i += 1; continue
            j = i
            while j < len(t) and off[j]:
                j += 1
            a, b = i, j                       # off-ground interval [a, b)
            i = j
            if b - a < 3:
                continue
            ev = {'foot': S, 't_liftoff': float(t[a]), 't_touchdown': float(t[b]) if b < len(t) else None,
                  'swing_s': float((b - a) * dt), 'max_clear_mm': float(cl[a:b].max() * 1000),
                  'stance_mean_load_frac': float(fo[a:b].mean() / weight), 'stance_min_load_frac': float(fo[a:b].min() / weight),
                  'other_part_contact_N': float(other[a:b].max()),
                  'advance_mm': float(-(fy[min(b, len(t) - 1)] - fy[a]) * 1000),
                  'lateral_mm': float((fx[min(b, len(t) - 1)] - fx[a]) * 1000),
                  'stance_slip_mm': float(np.hypot(ox[a:b] - ox[a], oy[a:b] - oy[a]).max() * 1000)}
            need = int(round(reload_s / dt)); reload_ok = False; t_reload = None
            if b < len(t):
                k = b; run = 0
                while k < len(t) and t[k] < t[b] + 0.5:
                    run = run + 1 if fz[k] >= reload_frac * weight else 0
                    if run >= need:
                        reload_ok = True; t_reload = float(t[k]); break
                    k += 1
            ev['reloaded'] = reload_ok; ev['t_reload'] = t_reload
            end = (np.searchsorted(t, t_reload + recover_s) if reload_ok else len(t))
            ev['no_fall_through_recovery'] = bool(first_fall >= min(end, len(t)) and (not reload_ok or t[-1] >= t_reload + recover_s - 2 * dt))
            crit = {'unloaded': True, 'clear_gt_5mm': ev['max_clear_mm'] > clear_req * 1000,
                    'stance_loaded': ev['stance_mean_load_frac'] >= stance_frac and ev['other_part_contact_N'] <= 0.0,
                    'advance_ge_10mm': ev['advance_mm'] >= adv_req * 1000, 'reloaded_20pct_30ms': reload_ok,
                    'recovered': ev['no_fall_through_recovery']}
            ev['criteria'] = crit
            ev['valid'] = bool(all(crit.values()))
            ev['strict'] = bool(ev['valid'] and ev['stance_slip_mm'] < slip_lim * 1000)
            events.append(ev)
    events.sort(key=lambda e: e['t_liftoff'])
    valid = [e for e in events if e['valid']]
    alt = 0; last = None
    for e in valid:
        if e['foot'] != last:
            alt += 1; last = e['foot']
    return events, {'swing_intervals': len(events), 'valid_steps': len(valid), 'strict_steps': sum(e['strict'] for e in events),
                    'valid_alternating_steps': alt}


def summarize(rows, weight, fall):
    rows_all = rows
    rows = [r for r in rows if r['t'] >= 0.2] or rows      # ignore the 1 mm settling drop after reset
    t = arr(rows, 't'); sat = arr(rows, 'n_saturated')
    fzL, fzR = arr(rows, 'fz_L'), arr(rows, 'fz_R')
    out = {'duration_s': float(t[-1]), 'fell': fall is not None, 'first_fall_s': fall,
           'forward_com_mm': float(-(rows[-1]['com_y'] - rows_all[0]['com_y']) * 1000),
           'lateral_com_mm': float((rows[-1]['com_x'] - rows_all[0]['com_x']) * 1000),
           'max_tilt_deg': float(arr(rows, 'tilt_deg').max()), 'max_abs_roll_deg': float(np.abs(arr(rows, 'roll_deg')).max()),
           'max_abs_pitch_deg': float(np.abs(arr(rows, 'pitch_deg')).max()),
           'sat_fraction': float((sat > 0).mean()),
           'mean_load_frac_L': float(fzL.mean() / weight), 'mean_load_frac_R': float(fzR.mean() / weight),
           'min_load_frac_L': float(fzL.min() / weight), 'min_load_frac_R': float(fzR.min() / weight),
           'max_clear_L_mm': float(arr(rows, 'clear_min_L').max() * 1000), 'max_clear_R_mm': float(arr(rows, 'clear_min_R').max() * 1000),
           'single_support_L_frac': float(((fzL > 0) & (fzR <= 0)).mean()), 'single_support_R_frac': float(((fzR > 0) & (fzL <= 0)).mean()),
           'other_contact_frac': float((arr(rows, 'fz_other') > 0).mean())}
    ev, st = detect_steps(rows, weight)
    out.update(st)
    return out, ev
