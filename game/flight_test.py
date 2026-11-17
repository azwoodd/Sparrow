"""Headless flight experiments with free body dynamics.
  python flight_test.py glide|fly [key=value ...]     e.g.  python flight_test.py fly throttle=0.6 speed=7 k_pitch=1.5"""
import math, sys, os, json
from pathlib import Path
GAME = Path(__file__).resolve().parent
import world                                    # sets up paths / variant
from world import World
from flight import FlightRig, FlightPilot, air_start


def run(lab, rig, throttle=0.6, pitch_cmd=0.0, turn=0.0, T=8.0, height=3.0, speed=7.0, pitch0=5.0, case=None, verbose=True, turn_at=None):
    air_start(lab, rig, height, speed, pitch0, case); fp = FlightPilot(rig); fp.start(lab)
    fp.throttle, fp.pitch_cmd, fp.turn = throttle, pitch_cmd, 0.0 if turn_at else turn
    i = 0; out = None; pw = []; lf = []; maxp = 0.0
    while lab.d.time < T:
        if turn_at and lab.d.time >= turn_at: fp.turn = turn
        if i % 10 == 0: fp(lab, lab.d.time)
        lab.e.step(); i += 1
        pitch, roll, v = lab.e.state(); pw.append(rig.power); lf.append(rig.lift)
        z = float(lab.d.qpos[2]); vy = -float(lab.d.qvel[1]); vz = float(lab.d.qvel[2])
        maxp = max(maxp, abs(math.degrees(pitch)), abs(math.degrees(roll)))
        if verbose and i % 500 == 0:
            print(f't={lab.d.time:5.2f} height {z:5.2f} m  fwd {vy:5.2f} m/s  climb {vz:+5.2f} m/s  pitch(nose up) {-math.degrees(pitch):+6.1f}  roll {math.degrees(roll):+6.1f}  flap {fp.f:.1f} Hz  power {sum(pw[-500:]) / 500:5.1f} W  lift {sum(lf[-500:]) / 500:5.2f} N', flush=True)
        if z < 0.12 or abs(math.degrees(pitch)) > 80 or abs(math.degrees(roll)) > 80:
            out = 'ground' if z < 0.12 else 'tumbled'; break
    R = lab.d.xmat[1].reshape(3, 3)
    return {'end': out or 'flying', 't': float(lab.d.time), 'height': float(lab.d.qpos[2]), 'fwd': -float(lab.d.qvel[1]), 'climb': float(lab.d.qvel[2]), 'max_att': maxp,
            'heading': math.degrees(math.atan2(-R[0, 1], R[1, 1])), 'power': sum(pw) / max(len(pw), 1)}


if __name__ == '__main__':
    kw = {}; fl = {}
    for a in sys.argv[2:]:
        k, v = a.split('='); (kw if k in ('throttle', 'pitch_cmd', 'turn', 'T', 'height', 'speed', 'pitch0', 'turn_at') else fl)[k] = float(v)
    w = World(); rig = FlightRig(w.lab, fl)
    print(f'weight {w.lab.weight:.2f} N; flight settings: ' + ', '.join(f'{k}={v:g}' for k, v in rig.p.items()))
    if sys.argv[1] == 'glide': kw.setdefault('throttle', 0.0)
    r = run(w.lab, rig, **kw); rig.enable(False); print(r)
