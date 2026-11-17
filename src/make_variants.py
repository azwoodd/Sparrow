"""Build copied MuJoCo model variants under model/<name>/ from model/v7e_baseline (never from / into the originals).

Each variant is a self-contained directory (xml + json + sparrow_sim.py + meshes) plus VARIANT_MANIFEST.json listing the
exact text substitutions, so every difference from the owner's export is transparent. Select with env SPARROW_VARIANT.
"""
import difflib, hashlib, json, re, shutil, sys
from pathlib import Path

WS = Path(__file__).resolve().parents[1]
BASE = WS / 'model' / 'v7e_baseline'

VARIANTS = {
    # SIMULATION FIDELITY ONLY - no mechanical change. Default MuJoCo contact (solref 0.02 s) lets the 0.52 kg robot's pads
    # sink 1-3 mm into the floor; a real rubber sole on a hard floor compresses microns. 0.004 s is 25x stiffer and still
    # numerically safe at the model's 1 ms timestep (needs >= 2 timesteps).
    'v7e_stiffcontact': {
        'kind': 'simulation fidelity (contact stiffness), no mechanical change',
        'subs': [
            ('<mujoco model="sparrow_v7e">', '<mujoco model="sparrow_v7e_stiffcontact">'),
            ('<default class="col"><geom contype="1" conaffinity="0" group="3" density="0" rgba="1 0 0 0.3"/></default>',
             '<default class="col"><geom contype="1" conaffinity="0" group="3" density="0" rgba="1 0 0 0.3" solref="0.004 1"/></default>'),
            ('<geom name="floor" type="plane" size="6 6 0.1" material="grid" contype="0" conaffinity="1" friction="1.0 0.02 0.001"/>',
             '<geom name="floor" type="plane" size="6 6 0.1" material="grid" contype="0" conaffinity="1" friction="1.0 0.02 0.001" solref="0.004 1"/>'),
        ]},
    'v7e_stiffcontact_elliptic': {
        'kind': 'simulation fidelity (contact stiffness + elliptic friction cone), no mechanical change',
        'subs': [
            ('<mujoco model="sparrow_v7e">', '<mujoco model="sparrow_v7e_stiffcontact_elliptic">'),
            ('<option timestep="0.001" integrator="implicitfast"/>', '<option timestep="0.001" integrator="implicitfast" cone="elliptic" impratio="10"/>'),
            ('<default class="col"><geom contype="1" conaffinity="0" group="3" density="0" rgba="1 0 0 0.3"/></default>',
             '<default class="col"><geom contype="1" conaffinity="0" group="3" density="0" rgba="1 0 0 0.3" solref="0.004 1"/></default>'),
            ('<geom name="floor" type="plane" size="6 6 0.1" material="grid" contype="0" conaffinity="1" friction="1.0 0.02 0.001"/>',
             '<geom name="floor" type="plane" size="6 6 0.1" material="grid" contype="0" conaffinity="1" friction="1.0 0.02 0.001" solref="0.004 1"/>'),
        ]},
}


# ---------------------------------------------------------------------------------------------------------------------
# MECHANICAL COUNTERFACTUALS (copies only; nothing here is approved hardware). All start from v7e_stiffcontact.
_STIFF = VARIANTS['v7e_stiffcontact']['subs'][1:]
_HIP = '<position name="act_leg_{S}_thigh" joint="leg_{S}_thigh" kp="4" ctrlrange="-1.0472 1.0472" forcerange="-0.35 0.35"/>'
_TORSO = '<inertial pos="0.000570781 -0.00639425 0.0331198" mass="0.273197"'


def _hip(nm, extra_g):
    return {'kind': f'MECHANICAL counterfactual: hip servos {nm} Nm instead of 0.35 Nm, +{extra_g} g added to the torso (two heavier servos); speed not modelled',
            'subs': [('<mujoco model="sparrow_v7e">', f'<mujoco model="sparrow_v7e_hip{int(nm * 100):03d}">')] + _STIFF +
                    [(_HIP.format(S=S), _HIP.format(S=S).replace('forcerange="-0.35 0.35"', f'forcerange="-{nm} {nm}"')) for S in 'LR'] +
                    [(_TORSO, _TORSO.replace('0.273197', f'{0.273197 + extra_g / 1000:.6f}'))]}


def _ankle(shift_mm):
    s = shift_mm / 1000.0
    subs = [('<mujoco model="sparrow_v7e">', f'<mujoco model="sparrow_v7e_ankle_fwd{shift_mm}">')] + _STIFF
    for S, x in (('L', '0.003'), ('R', '-0.003')):
        subs.append((f'<geom name="col_foot_{S}" type="box" pos="{x} -0.043 -0.0219"', f'<geom name="col_foot_{S}" type="box" pos="{x} {-0.043 + s:.4f} -0.0219"'))
        for k in range(3):
            subs.append((f'<geom type="mesh" mesh="leg_{S}_ankle_{k}" ', f'<geom type="mesh" mesh="leg_{S}_ankle_{k}" pos="0 {s:.4f} 0" '))
        subs.append((f'<inertial pos="{x} -0.0392422 -0.0104108"', f'<inertial pos="{x} {-0.0392422 + s:.7f} -0.0104108"'))
    return {'kind': f'MECHANICAL counterfactual: ankle joint {shift_mm} mm further forward along the foot (foot shifted back under the ankle); same foot, same mass',
            'subs': subs}


def _combo(nm, extra_g, shift_mm):
    a, b = _hip(nm, extra_g), _ankle(shift_mm)
    return {'kind': a['kind'] + '  +  ' + b['kind'],
            'subs': [('<mujoco model="sparrow_v7e">', f'<mujoco model="sparrow_v7e_hip{int(nm * 100):03d}_ankle{shift_mm}">')] + a['subs'][1:] + b['subs'][1 + len(_STIFF):]}


VARIANTS.update({'v7e_hip055_ankle30': _combo(0.55, 12, 30), 'v7e_hip055': _hip(0.55, 12), 'v7e_hip070': _hip(0.70, 24), 'v7e_ankle_fwd30': _ankle(30), 'v7e_ankle_fwd45': _ankle(45)})


# Spring spanning hip AND knee (biarticular): two extra spring-torque channels on the hips. The split between knee and hip
# is set by the pulley radius ratio (parameter rho in the controller). +4 g on the torso for cable, pulleys and anchors.
_KS = '<motor name="jump_spring_R" joint="leg_R_shin" ctrlrange="-3 3"/>'
_HS = _KS + '\n    <motor name="jump_spring_hip_L" joint="leg_L_thigh" ctrlrange="-3 3"/>\n    <motor name="jump_spring_hip_R" joint="leg_R_thigh" ctrlrange="-3 3"/>'


def _bispring(ankle_mm=0):
    subs = [('<mujoco model="sparrow_v7e">', f'<mujoco model="sparrow_v7e_bispring{"_ankle%d" % ankle_mm if ankle_mm else ""}">')] + _STIFF
    subs += [(_KS, _HS), (_TORSO, _TORSO.replace('0.273197', f'{0.273197 + 0.004:.6f}'))]
    if ankle_mm:
        subs += _ankle(ankle_mm)['subs'][1 + len(_STIFF):]
    return {'kind': 'MECHANICAL counterfactual: jump spring spans hip and knee (biarticular), original 0.35 Nm servos, +4 g'
                    + (f'; ankle {ankle_mm} mm further forward along the foot' if ankle_mm else ''), 'subs': subs}


VARIANTS.update({'v7e_bispring': _bispring(0), 'v7e_bispring_ankle30': _bispring(30)})

def build(name):
    spec = VARIANTS[name]
    dst = WS / 'model' / name
    dst.mkdir(parents=True, exist_ok=True)
    for f in ('sparrow_v7e_model.json', 'sparrow_sim.py'):
        shutil.copy2(BASE / f, dst / f)
    if not (dst / 'meshes').exists():
        shutil.copytree(BASE / 'meshes', dst / 'meshes')
    src = (BASE / 'sparrow_v7e.xml').read_text()
    out = src
    for old, new in spec['subs']:
        if out.count(old) != 1:
            raise SystemExit(f'{name}: expected exactly one match for: {old[:70]}... (found {out.count(old)})')
        out = out.replace(old, new)
    for old, new in spec.get('regex', []):
        out, n = re.subn(old, new, out)
        if not n:
            raise SystemExit(f'{name}: regex matched nothing: {old}')
    (dst / 'sparrow_v7e.xml').write_text(out)
    diff = [l for l in difflib.unified_diff(src.splitlines(), out.splitlines(), 'v7e_baseline', name, lineterm='', n=0)]
    (dst / 'VARIANT_MANIFEST.json').write_text(json.dumps({
        'variant': name, 'kind': spec['kind'], 'built_from': str(BASE / 'sparrow_v7e.xml'),
        'baseline_sha256': hashlib.sha256(src.encode()).hexdigest(), 'variant_sha256': hashlib.sha256(out.encode()).hexdigest(),
        'changed_lines': sum(1 for l in diff if l.startswith('+') and not l.startswith('+++')), 'unified_diff': [l[:400] for l in diff]}, indent=1))
    print(f'built {name}: {sum(1 for l in diff if l.startswith("+") and not l.startswith("+++"))} changed lines')


if __name__ == '__main__':
    for n in (sys.argv[1:] or VARIANTS):
        build(n)



