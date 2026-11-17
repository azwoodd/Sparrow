"""Fast-gait probe:  python r1_probe.py key=val ... [dur=8] [save=name]   (Runner parameters, see run_gait.py)"""
import os, sys, json
os.environ.setdefault('SPARROW_VARIANT', 'v7e_stiffcontact')
import numpy as np
from lab import Lab, WS, VARIANT, summarize, arr
from run_gait import Runner

OUT = WS / 'results' / 'r1_probe'; OUT.mkdir(parents=True, exist_ok=True)


def run_metrics(lab, kw, dur=8.0, case=None):
    fall = lab.run(Runner(**kw), dur, case=case, stop_after_fall=0.2, every=2)
    rows = lab.rows; s, ev = summarize(rows, lab.weight, fall)
    r = [x for x in rows if x['t'] > 1.5] or rows
    fzL, fzR = arr(r, 'fz_L'), arr(r, 'fz_R')
    T = (r[-1]['t'] - r[0]['t']) or 1e-9
    out = {'fall': fall, 'T': rows[-1]['t'], 'forward_mm': s['forward_com_mm'], 'speed_mm_s': -(r[-1]['com_y'] - r[0]['com_y']) * 1000 / T,
           'lateral_mm': s['lateral_com_mm'], 'max_tilt': s['max_tilt_deg'], 'max_roll': s['max_abs_roll_deg'], 'max_pitch': s['max_abs_pitch_deg'],
           'valid_steps': s['valid_steps'], 'alt_steps': s['valid_alternating_steps'], 'swings': s['swing_intervals'],
           'flight_frac': float(((fzL <= 0) & (fzR <= 0)).mean()), 'single_frac': float(((fzL <= 0) ^ (fzR <= 0)).mean()),
           'sat': s['sat_fraction'], 'other': s['other_contact_frac'],
           'yaw_deg': float(r[-1]['foot_yaw_deg_L'] - rows[0]['foot_yaw_deg_L'])}
    return out, ev


if __name__ == '__main__':
    kw = {}; dur = 8.0; save = None
    for a in sys.argv[1:]:
        k, v = a.split('=')
        if k == 'dur': dur = float(v)
        elif k == 'save': save = v
        else: kw[k] = float(v)
    lab = Lab(); o, ev = run_metrics(lab, kw, dur)
    print(kw); print(' '.join(f'{k}={v:.3f}' if isinstance(v, float) else f'{k}={v}' for k, v in o.items()))
    if save: lab.save_trace(OUT / f'{save}.csv', decimate=2)
