"""Phase 0b: corrected control comparison + instrumentation validation against the archived V3/V4 results.

Every case uses the owner's feedback controller (vendor copy) and identical reset; only the leg-target delta differs.
"""
import json, math
import numpy as np
from lab import Lab, FeedbackBase, WS, summarize, arr

OUT = WS / 'results' / 'p0_controls'
OUT.mkdir(parents=True, exist_ok=True)


class Quiet(FeedbackBase):
    """TRUE quiet standing: feedback only, zero added command."""


class NoFeedback(FeedbackBase):
    feedback = False


class V3Gait(FeedbackBase):
    """Exact V1/V3/V4 gait delta. Note k = 0.10*max(0,sin) is independent of amp (the V4 'stationary_control' confounder)."""
    def __init__(self, amp, hz, knee=0.10):
        self.amp, self.hz, self.knee = amp, hz, knee

    def delta(self, lab, t):
        d = np.zeros(6); ph = 2 * math.pi * self.hz * t
        for off, sp in ((0, 0.0), (3, math.pi)):
            x = math.sin(ph + sp); h = self.amp * x; k = self.knee * max(0.0, x)
            d[off] += h; d[off + 1] += k; d[off + 2] -= h + k
        return d


class V3Lift(FeedbackBase):
    def __init__(self, side, deg):
        self.idx = 0 if side == 'L' else 3; self.ang = math.radians(deg)

    def delta(self, lab, t):
        d = np.zeros(6)
        a = float(np.clip((t - .35) / .5, 0, 1) * np.clip((2.05 - t) / .45, 0, 1)); k = self.ang * a
        d[self.idx] -= .5 * k; d[self.idx + 1] += k; d[self.idx + 2] -= .5 * k
        return d


CASES = [
    ('A1_true_quiet_stand_feedback_4s', Quiet(), 4.0, 'NEW corrected control (same 4.0 s horizon as V4 stationary_control)'),
    ('A2_true_quiet_stand_feedback_20s', Quiet(), 20.0, 'NEW corrected control, long'),
    ('A3_hold_pose_no_feedback_20s', NoFeedback(), 20.0, 'position hold only'),
    ('B1_V4_stationary_control_repro_kneewiggle', V3Gait(0.0, 1.2), 4.0, 'reproduces V4 #013 (gait amp=0 -> knee wiggle 0.10 rad)'),
    ('B2_V4_shuffle_repro', V3Gait(-0.085, 1.2), 4.0, 'reproduces V4 #014'),
    ('B3_V4_original_gait_repro', V3Gait(0.085, 1.0), 4.0, 'reproduces V4 #015'),
    ('B4_V3_shuffle_12s', V3Gait(-0.085, 1.2), 12.0, 'reproduces V3 candidate_12s'),
    ('C1_lift_L8', V3Lift('L', 8), 3.4, 'reproduces V4 #002'),
    ('C2_lift_L12', V3Lift('L', 12), 3.4, 'reproduces V4 #005'),
    ('C3_lift_R12', V3Lift('R', 12), 3.4, 'reproduces V4 #011'),
]

if __name__ == '__main__':
    lab = Lab(); res = []
    for name, ctl, dur, note in CASES:
        fall = lab.run(ctl, dur, stop_after_fall=10.0)
        summ, ev = summarize(lab.rows, lab.weight, fall)
        knee_cmd = arr(lab.rows, 'leg_L_shin_tgt'); ank = arr(lab.rows, 'leg_L_ankle_tgt')
        summ.update(name=name, note=note, knee_L_target_range_rad=[float(knee_cmd.min()), float(knee_cmd.max())],
                    ankle_L_target_range_rad=[float(ank.min()), float(ank.max())],
                    rms_tilt_deg=float(np.sqrt(np.mean(arr(lab.rows, 'tilt_deg') ** 2))))
        lab.save_trace(OUT / f'{name}.csv')
        (OUT / f'{name}_events.json').write_text(json.dumps(ev, indent=1))
        res.append(summ)
        print(f"{name:44} fall={str(fall)[:5]:>5} fwd={summ['forward_com_mm']:7.1f}mm maxtilt={summ['max_tilt_deg']:5.1f} "
              f"kneeTgt=[{summ['knee_L_target_range_rad'][0]:+.3f},{summ['knee_L_target_range_rad'][1]:+.3f}] "
              f"minLoadL/R={summ['min_load_frac_L']:.2f}/{summ['min_load_frac_R']:.2f} clrL/R={summ['max_clear_L_mm']:.1f}/{summ['max_clear_R_mm']:.1f}mm "
              f"swings={summ['swing_intervals']} valid={summ['valid_steps']} sat={summ['sat_fraction']:.2f}", flush=True)
    (OUT / 'summary.json').write_text(json.dumps(res, indent=1))
