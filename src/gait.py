"""Wing-counterweight quasi-static walk for the UNMODIFIED V7E (keyframe generator for script_ctl.Script).

Cycle (per step):
  SS  single support: stance-side wing deployed (COM over stance pad), body COM held `back` behind pad centre (near the
      ankle axis) so one 0.2 Nm ankle holds pitch; swing foot lifts, advances, lowers with a world-level pad.
  DS  double support: wings swap (lateral weight transfer) while both ankles move `s` backward under the body
      (body advances `s`), which puts the NEXT stance ankle back under the COM.
"""
from script_ctl import STOW


def walk_tracks(n_steps=10, s=0.030, back=0.030, lift=0.015, deploy=1.0, t0=0.5, t_prep=1.0, hold=0.3,
                t_lift=0.3, t_adv=0.4, t_down=0.3, t_ds=1.0, first='R', toe_up=0.0, finish=True, settle=0.15,
                t_unw=0.0, unw=0.003, neck=0.0, close=False):
    """t_unw/unw: gentle un-weighting - raise the swing ankle `unw` m over `t_unw` s before the real lift so the load moves
    to the stance pad quasi-statically (the model's ground contact is soft: pads sink 1-3 mm, and a sudden release rocks
    the body toward the pad's inner edge).  neck: head yaw (rad) toward the stance side during single support (+2.6 mm COM)."""
    out = STOW * (1 - deploy)
    tr = {k: [] for k in ('wing_L_sweep', 'wing_R_sweep', 'L_dy', 'R_dy', 'L_dz', 'R_dz', 'L_swing', 'R_swing', 'L_pitch', 'R_pitch', 'neck_yaw')}
    sgn = {'L': 1.0, 'R': -1.0}
    swing = first; stance = 'L' if swing == 'R' else 'R'
    pos = {'L': -back, 'R': -back}                      # ankle dy of each leg relative to standing pose
    t = t0
    tr[f'wing_{stance}_sweep'] += [(t, STOW), (t + t_prep, out)]
    tr[f'wing_{swing}_sweep'] += [(t, STOW)]
    for S in 'LR':
        tr[S + '_dy'] += [(t, 0.0), (t + t_prep, -back)]
        tr[S + '_dz'] += [(t, 0.0)]
    if neck:
        tr['neck_yaw'] += [(t, 0.0), (t + t_prep, neck * sgn[stance])]
    t += t_prep + hold
    plan = []
    for i in range(n_steps):
        u = t
        a = t + t_unw
        b, c, d = a + t_lift, a + t_lift + t_adv, a + t_lift + t_adv + t_down
        new = -back - s
        closing = close and i == n_steps - 1
        if closing:
            new = -back                      # bring the trailing foot level with the stance foot (feet side by side)
        if t_unw:
            tr[swing + '_dz'] += [(u, 0.0), (a, unw), (b, lift), (c, lift), (d, 0.0)]
        else:
            tr[swing + '_dz'] += [(a, 0.0), (b, lift), (c, lift), (d, 0.0)]
        a = u
        tr[swing + '_dy'] += [(b, pos[swing]), (c, new)]
        tr[swing + '_swing'] += [(a, 0.0), (a + 1e-3, 1.0), (d + settle, 1.0), (d + settle + 1e-3, 0.0)]
        if toe_up:
            tr[swing + '_pitch'] += [(a, 0.0), (b, toe_up), (c, toe_up), (d, 0.0)]
        plan.append({'step': i + 1, 'swing': swing, 't_lift': a, 't_down': d, 'world_advance_m': pos[swing] - new})
        pos[swing] = new
        t = d + settle
        last = (i == n_steps - 1)
        if last and not finish:
            break
        # double support: transfer weight to the foot that just landed and advance the body
        e = t + t_ds
        tr[f'wing_{stance}_sweep'] += [(t, out), (e, STOW)]
        if not last:
            tr[f'wing_{swing}_sweep'] += [(t, STOW), (e, out)]
        if neck:
            tr['neck_yaw'] += [(t, neck * sgn[stance]), (e, 0.0 if last else neck * sgn[swing])]
        shift = s if not last else s / 2          # final: square up with feet s apart, COM between
        if closing:
            shift = back                          # undo the back-lean: return to the exact standing pose, wings stowed
        for S in 'LR':
            tr[S + '_dy'] += [(t, pos[S]), (e, pos[S] + shift)]
            pos[S] += shift
        t = e + hold
        swing, stance = stance, swing
    return {k: v for k, v in tr.items() if v}, t, plan
