import json, os
os.environ["SPARROW_VARIANT"] = "v7e_bispring"
import v3_bispring as v3
from lab import WS
lib = json.loads((WS / "results" / "h2_opt" / "v7e_bispring" / "library.json").read_text())
CONDS = [("nominal", {}), ("friction0.6", {"friction": 0.6}), ("friction1.4", {"friction": 1.4}), ("mass1.07", {"mass": 1.07}), ("comY-3", {"com_shift": -0.003})]
res = []
for name in ("bi_E080", "bi_E110"):
    print("===", name, flush=True)
    for label, ch in (("drag x0.5", {"damp_scale": 0.5}), ("drag x1 (as validated)", {}), ("drag x1.5", {"damp_scale": 1.5}), ("drag x2", {"damp_scale": 2.0}), ("drag x3", {"damp_scale": 3.0}),
                      ("drag x2, energy +25%", {"damp_scale": 2.0, "E_scale": 1.25}), ("drag x3, energy +50%", {"damp_scale": 3.0, "E_scale": 1.5})):
        n = 0; cl = []; tl = []
        for cn, case in CONDS:
            r = v3.hop(dict(lib[name]["params"], **ch), case); n += r["good"]; cl.append(r.get("clear", 0)); tl.append(r.get("tilt_after", 99)); res.append(dict(r, entry=name, test=label, cond=cn))
        print(f"  {label:24} {n}/5 good  clearance {min(cl):5.1f}-{max(cl):5.1f} mm  max tilt after {max(tl):5.1f}", flush=True)
(WS / "results" / "v4_detail" / "drag_v7e_bispring.json").write_text(json.dumps(res, indent=1, default=float))
