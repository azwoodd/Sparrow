"""PROPOSED MECHANICAL CHANGE, simulation copy only: a hip-twist (yaw) servo on each leg, so the standing leg can rotate the
body - how birds and walking robots turn on the spot. Builds model/v7e_game_hipyaw from model/v7e_game; nothing else
is touched. Hardware assumed: one KST X08H Plus class servo per hip (0.35 Nm used, same as the other hip servos), 9.5 g
each, +-29 deg of twist. Their 19 g is added to the torso.   python make_hipyaw.py"""
import json, re, shutil
from pathlib import Path

WS = Path(__file__).resolve().parents[1]
SRC = WS / 'model' / 'v7e_game'; DST = WS / 'model' / 'v7e_game_hipyaw'
if DST.exists(): shutil.rmtree(DST)
shutil.copytree(SRC, DST)
xf = DST / 'sparrow_v7e.xml'; x = xf.read_text()
for S in 'LR':
    old = f'<joint name="leg_{S}_thigh" type="hinge" axis="1 0 0"'
    assert x.count(old) == 1
    x = x.replace(old, f'<joint name="leg_{S}_yaw" type="hinge" axis="0 0 1" range="-0.5 0.5" damping="0.02" armature="2e-05"/>\n        ' + old)
act = ''.join(f'<position name="act_leg_{S}_yaw" joint="leg_{S}_yaw" kp="4" ctrlrange="-0.5 0.5" forcerange="-0.35 0.35"/>' for S in 'LR')
x = x.replace('</actuator>', act + '</actuator>', 1)
# torso inertial: + 2 x 9.5 g
m = re.search(r'(<body name="torso"[^>]*>\s*<freejoint[^>]*>\s*<inertial [^>]*mass=")([0-9.]+)(")', x)
x = x[:m.start(2)] + f'{float(m.group(2)) + 0.019:.6f}' + x[m.end(2):]
# keyframes: insert 0 for each yaw joint just before its thigh angle (qpos 23 and, after that insert, 27)
def fix(mm):
    q = mm.group(2).split(); q.insert(23, '0'); q.insert(27, '0'); return mm.group(1) + ' '.join(q) + mm.group(3)
x, n = re.subn(r'(<key [^>]*qpos=")([^"]*)(")', fix, x)
x = re.sub(r'<mujoco model="[^"]*">', '<mujoco model="sparrow_v7e_game_hipyaw">', x, 1)
xf.write_text(x)
jf = DST / 'sparrow_v7e_model.json'; info = json.loads(jf.read_text())
for pose in info['poses'].values():
    for S in 'LR': pose['q'][f'leg_{S}_yaw'] = 0.0
jf.write_text(json.dumps(info, indent=1))
(DST / 'GAME_WORLD_MANIFEST.json').write_text(json.dumps({**json.loads((SRC / 'GAME_WORLD_MANIFEST.json').read_text()), 'built_from': 'model/v7e_game',
    'robot_changed': True, 'mechanical_change': 'hip yaw joint + 0.35 Nm position servo per leg, +-0.5 rad, +19 g on torso (PROPOSED, not approved)'}, indent=1))
import mujoco
mm = mujoco.MjModel.from_xml_path(str(xf))
print(f'built model/v7e_game_hipyaw: {n} keyframes fixed, mass {mm.body_subtreemass[1] * 1000:.0f} g, {mm.nu} actuators')
