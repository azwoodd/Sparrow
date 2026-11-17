"""Debug helper: run the candidate gait under one condition and save the trace.  python dbg_case.py name key=val ..."""
import sys, json
from lab import Lab, WS, summarize
from script_ctl import Script
from gait import walk_tracks
from p2_walk import CANDIDATE, line, run, OUT

name = sys.argv[1]; case = {}; kw = dict(CANDIDATE)
for a in sys.argv[2:]:
    k, v = a.split('=')
    try:
        v = float(v)
    except ValueError:
        pass
    (case if k in ('friction', 'mass', 'com_shift', 'lean', 'lean_axis') else kw)[k] = int(v) if k == 'n_steps' else v
lab = Lab()
s, ev = run(lab, name, kw, case=case, save=True)
print(line(s))
