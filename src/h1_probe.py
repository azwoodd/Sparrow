"""Hop probe: run one hop parameter set and print what happened.  python h1_probe.py key=val ...  [save=name]"""
import os, sys, json
os.environ.setdefault('SPARROW_VARIANT', 'v7e_stiffcontact')
import numpy as np
from lab import Lab, WS, VARIANT, summarize
from hop import Hopper, analyse_hops

OUT = WS / 'results' / 'h1_probe'; OUT.mkdir(parents=True, exist_ok=True)


def run(lab, p, dur=4.5, case=None, n=1, gap=1.6):
    ctl = Hopper([(0.5 + k * gap, p) for k in range(n)])
    fall = lab.run(ctl, dur + (n - 1) * gap, case=case, stop_after_fall=0.3, every=2)
    hops = analyse_hops(lab.rows, lab.weight)
    return fall, hops, ctl


def show(fall, hops, ctl):
    print(f"fall={fall} spring_work={ctl.spring_work:.3f}J phases={ctl.log}")
    for h in hops:
        print('  ' + ' '.join(f'{k}={v:.3f}' if isinstance(v, float) else f'{k}={v}' for k, v in h.items()))


if __name__ == '__main__':
    p = {}; save = None; n = 1
    for a in sys.argv[1:]:
        k, v = a.split('=')
        if k == 'save': save = v
        elif k == 'n': n = int(v)
        else: p[k] = float(v)
    lab = Lab()
    fall, hops, ctl = run(lab, p, n=n)
    print(VARIANT, p); show(fall, hops, ctl)
    if save:
        lab.save_trace(OUT / f'{save}.csv', decimate=2)
