"""Named demonstrations shared by the live viewer (view.py) and the offscreen renderer (render_frames.py)."""
from script_ctl import Script
from gait import walk_tracks
from p0_controls import Quiet, V3Gait, V3Lift
from p1_one_step import wing_step
from p2_walk import CANDIDATE


def _walk(**kw):
    tr, tend, _ = walk_tracks(**kw)
    return Script(tr), tend + 2.0


def _step(**kw):
    tr, tend = wing_step(**kw)
    return Script(tr), tend + 2.5


DEMOS = {
    'walk':        ('WingWalk-A candidate: 10 alternating steps (wing counterweight + back-lean)', lambda: _walk(**CANDIDATE)),
    'walk_long':   ('WingWalk-A, 30 steps', lambda: _walk(**dict(CANDIDATE, n_steps=30))),
    'one_step':    ('One genuine right-foot step, then re-stow the wing', lambda: _step(back=0.03, t_lift=0.6, t_adv=0.8, t_down=0.6)),
    'step_no_wing': ('NEGATIVE CONTROL: same step without the wing counterweight (falls)', lambda: _step(back=0.03, deploy=0.0)),
    'step_no_back': ('NEGATIVE CONTROL: wing out but no back-lean (pitch fall: one 0.2 Nm ankle saturates)', lambda: _step(back=0.0)),
    'quiet':       ('TRUE quiet standing - owner feedback controller only', lambda: (Quiet(), 20.0)),
    'v4_stationary': ("V4 'stationary_control' as archived (hidden 0.10 rad knee wiggle) - falls ~2.5 s", lambda: (V3Gait(0.0, 1.2), 5.0)),
    'shuffle':     ('V3/V4 sign-reversed shuffle (moves, but zero real steps)', lambda: (V3Gait(-0.085, 1.2), 12.0)),
    'original_gait': ('Original V1 gait (falls ~1.17 s)', lambda: (V3Gait(0.085, 1.0), 4.0)),
    'lift_L12':    ('V3/V4 12 deg left-leg lift (falls ~1.65 s)', lambda: (V3Lift('L', 12), 3.4)),
}


def _scn(name):
    """Hop scenarios (h4_scenarios.build) - need the optimised hop library of the current model variant."""
    def make():
        import h4_scenarios as h4
        from h2_opt import Library
        h4.lib = Library()
        ctl, dur, n, desc = h4.build(name)
        return ctl, dur
    return make


def _quick(**kw):
    from run_gait import Runner
    from r2_fast import QUICKSTEP
    return lambda: (Runner(**dict(QUICKSTEP, **kw)), kw.get('stop_t', 9.0) + 3.0)


def _bi(entry, n=1, gap=2.2):
    """Higher hops on the hip-and-knee spring model copy. Run the viewer with  --variant v7e_bispring  for these."""
    def make():
        import json
        from lab import WS, VARIANT
        from hop3 import BiHopper
        path = WS / 'results' / 'h2_opt' / VARIANT / 'library.json'
        if not path.exists() or entry not in json.loads(path.read_text()):
            raise SystemExit(f"demo needs the hip-and-knee spring model: add  --variant v7e_bispring  (no '{entry}' for {VARIANT})")
        p = json.loads(path.read_text())[entry]['params']
        return BiHopper([(0.5 + k * gap, p) for k in range(n)]), 0.5 + n * gap + 2.5
    return make


DEMOS.update({
    'bi_hop_60':  ('[--variant v7e_bispring] 60 mm hop, hip-and-knee spring, lands and stands', _bi('bi_E080')),
    'bi_hop_80':  ('[--variant v7e_bispring] 80 mm hop, hip-and-knee spring', _bi('bi_E100')),
    'bi_hop_100': ('[--variant v7e_bispring] 100 mm hop, hip-and-knee spring', _bi('bi_E110')),
    'bi_hop_60x3': ('[--variant v7e_bispring] three 60 mm hops in a row (untested combination)', _bi('bi_E080', n=3)),
    'quickstep':     ('Stowed-wing quick-step walk, 137 mm/s, stops and stands at 9 s', _quick()),
    'quickstep_fast': ('Stowed-wing quick-step at 4 Hz / 50 mm stride, ~250 mm/s', _quick(f=4.0, s=0.05)),
    'quickstep_left': ('Stowed-wing quick-step curving left (stride asymmetry 0.4)', _quick(turn=0.4)),
    'hop_25':         ('Standstill hop to a 25 mm target', _scn('standstill_25')),
    'hop_55':         ('Standstill hop to a 55 mm target', _scn('standstill_55')),
    'hop_triple':     ('Three 45 mm hops in a row', _scn('triple')),
    'hop_mixed':      ('Hops of 20, 58 and 40 mm in a row', _scn('triple_mixed')),
    'walk_hop':       ('Wing-counterweight walk, closing step, then a 50 mm hop', _scn('walk_hop')),
    'run_hop':        ('Stowed-wing fast gait, stop, then a 50 mm hop', _scn('quickstep_hop')),
})
ORDER = ['quickstep', 'hop_55', 'hop_triple', 'run_hop', 'walk_hop'] + [k for k in DEMOS if k not in ('quickstep', 'hop_55', 'hop_triple', 'run_hop', 'walk_hop')]
