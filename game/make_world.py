"""Build the game world: a copy of the hip-and-knee spring model (model/v7e_bispring -> model/v7e_game) with things in
it. Only the copy is written. The robot itself is not changed; a manifest lists what was added.
  python make_world.py
World layout (bird starts at the origin facing -Y; all sizes in metres):
  stairs      three 20 mm steps and a plateau straight ahead           hurdles    15 mm and 30 mm bars in the right lane
  balls       three light balls to push                                 ledge      a 0.30 m block to hop off, left
  tower       a 12 m launch tower behind the start (T key puts the bird on top; walk off the front edge to fly)
  gates       square hoops in the air to fly through, posts as landmarks, distance lines on the floor"""
import json, re, shutil
from pathlib import Path

WS = Path(__file__).resolve().parents[1]
SRC = WS / 'model' / 'v7e_bispring'; DST = WS / 'model' / 'v7e_game'
SPAWNS = {'start': (0.0, 0.0, 0.0), 'tower': (0.0, 5.75, 12.0), 'ledge': (-1.2, -1.0, 0.30), 'open': (3.0, 2.0, 0.0)}

S = 'contype="0" conaffinity="1" friction="1.0 0.02 0.001" solref="0.004 1"'
V = 'contype="0" conaffinity="0"'
g = []
box = lambda name, pos, size, rgba, extra=S: g.append(f'<geom name="env_{name}" type="box" pos="{pos[0]} {pos[1]} {pos[2]}" size="{size[0]} {size[1]} {size[2]}" rgba="{rgba}" {extra}/>')
# stairs: 20 mm rises, each tread 0.30 m deep, 0.5 m wide, then a plateau
for i, top in enumerate((0.02, 0.04, 0.06)):
    box(f'step{i + 1}', (0, -1.05 - 0.30 * i, top / 2), (0.25, 0.15, top / 2), '0.55 0.62 0.70 1')
box('plateau', (0, -2.35, 0.03), (0.25, 0.40, 0.03), '0.50 0.58 0.66 1')
# hurdles in the right lane (x = -0.7 is the bird's right; left foot is +X)
box('hurdle15', (-0.7, -0.9, 0.0075), (0.20, 0.006, 0.0075), '0.90 0.45 0.20 1')
box('hurdle30', (-0.7, -1.5, 0.015), (0.20, 0.006, 0.015), '0.90 0.30 0.20 1')
# ledge to hop off
box('ledge', (-1.2, -1.0, 0.15), (0.30, 0.30, 0.15), '0.60 0.55 0.45 1')
# launch tower
box('tower', (0, 6.0, 6.0), (0.6, 0.6, 6.0), '0.45 0.47 0.52 1')
box('tower_rim', (0, 6.62, 12.05), (0.6, 0.02, 0.05), '0.85 0.75 0.25 1')
# landmarks: posts
for i, (x, y, h) in enumerate(((4, -6, 1.5), (-5, -9, 2.0), (7, -14, 2.5), (-8, -18, 1.5), (3, -24, 3.0), (-3, -30, 2.0), (10, -32, 2.5), (-10, -4, 2.0))):
    g.append(f'<geom name="env_post{i}" type="cylinder" pos="{x} {y} {h / 2}" size="0.06 {h / 2}" rgba="0.35 0.55 0.35 1" {S}/>')
    g.append(f'<geom name="env_crown{i}" type="sphere" pos="{x} {y} {h + 0.25}" size="0.35" rgba="0.30 0.62 0.32 1" {V}/>')
# flying gates (visual only): 1.6 m square hoops
for i, (x, y, z) in enumerate(((0, -8, 4.5), (2, -18, 4.0), (-3, -28, 5.0), (0, -40, 3.5))):
    for j, (dx, dz, sx, sz) in enumerate(((0, 0.8, 0.85, 0.04), (0, -0.8, 0.85, 0.04), (0.8, 0, 0.04, 0.85), (-0.8, 0, 0.04, 0.85))):
        box(f'gate{i}_{j}', (x + dx, y, z + dz), (sx, 0.04, sz), '0.95 0.80 0.15 1', V)
# distance lines every metre for 10 m
for i in range(1, 11):
    box(f'line{i}', (0, -i, 0.0005), (1.5, 0.004, 0.0005), '0.30 0.30 0.35 1', V)
balls = [(0.35, -0.6, 0.03), (0.45, -0.9, 0.03), (0.30, -1.3, 0.03)]
ball_xml = ''.join(f'<body name="env_ball{i}" pos="{x} {y} {z}"><freejoint/><geom name="env_ball{i}" type="sphere" size="0.03" mass="0.004" rgba="{c} 1" contype="1" conaffinity="1" friction="0.6 0.01 0.001" solref="0.004 1"/></body>'
                   for i, ((x, y, z), c) in enumerate(zip(balls, ('0.9 0.2 0.2', '0.2 0.5 0.9', '0.95 0.8 0.1'))))

if DST.exists(): shutil.rmtree(DST)
shutil.copytree(SRC, DST)
xf = DST / 'sparrow_v7e.xml'; x = xf.read_text()
floor = re.search(r'<geom name="floor"[^>]*/>', x).group(0)
new_floor = floor.replace('size="6 6 0.1"', 'size="80 80 0.1"')
x = x.replace(floor, new_floor + '\n' + '\n'.join(g))
x = x.replace('texrepeat="8 8"', 'texrepeat="107 107"')
x = x.replace('</worldbody>', ball_xml + '</worldbody>', 1)
add = ''.join(f' {bx} {by} {bz} 1 0 0 0' for bx, by, bz in balls)
x, nkeys = re.subn(r'(<key [^>]*qpos=")([^"]*)(")', lambda m: m.group(1) + m.group(2).rstrip() + add + m.group(3), x)
x = re.sub(r'<mujoco model="[^"]*">', '<mujoco model="sparrow_v7e_game">', x, 1)
xf.write_text(x)
(DST / 'GAME_WORLD_MANIFEST.json').write_text(json.dumps({'built_from': 'model/v7e_bispring', 'robot_changed': False, 'static_objects': len(g), 'balls': len(balls), 'keyframes_extended': nkeys,
                                                         'spawns': SPAWNS, 'note': 'floor enlarged for flying; every added geom is named env_*'}, indent=1))
import mujoco
m = mujoco.MjModel.from_xml_path(str(xf))
print(f'built model/v7e_game: {len(g)} static objects, {len(balls)} balls, {nkeys} keyframes extended; model loads: {m.nbody} bodies, {m.ngeom} geoms, torso is body 1: {mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, 1)}')
