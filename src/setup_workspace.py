"""Phase 0: hash every original Sparrow source (read-only) and build isolated copies.

Usage:  python setup_workspace.py snapshot   -> manifests/originals_sha256.json + copies into model/ and src/vendor/
        python setup_workspace.py verify     -> re-hash originals and report any difference
Nothing is ever written outside D:\\Downloads\\Sparrow_Locomotion_Engineering_V1.
"""
import hashlib, json, shutil, sys, time
from pathlib import Path

WS = Path(__file__).resolve().parents[1]
MASTER = Path(r'C:\Users\Work\Documents\Codex\2026-11-11\blender-mcp-c-users-work-agents\outputs\Sparrow_V7D_Style')
CONTROL = Path(r'C:\Users\Work\Documents\Codex\2026-11-12\new-chat-2\Sparrow_Control_Development')
ROOTS = {
    'blender_and_mujoco_master': MASTER,
    'balance_control_and_rl': CONTROL,
    'walk_hop_v1': Path(r'D:\Downloads\Sparrow_Walk_Hop'),
    'diagnostics_v2': Path(r'D:\Downloads\Sparrow_Walk_Diagnostics_V2'),
    'feasibility_v3': Path(r'D:\Downloads\Sparrow_Feasibility_V3'),
    'verification_v4': Path(r'D:\Downloads\Sparrow_Verification_V4'),
    'handoff_package': Path(r'D:\Downloads\Sparrow_Claude_FINAL_12Nov2026'),
}
SKIP_DIRS = {'.venv', '.uv-cache', '__pycache__'}
MANIFEST = WS / 'manifests' / 'originals_sha256.json'


def sha(p):
    h = hashlib.sha256()
    with open(p, 'rb') as f:
        for chunk in iter(lambda: f.read(1 << 20), b''):
            h.update(chunk)
    return h.hexdigest()


def walk(root):
    out = {}
    for p in sorted(root.rglob('*')):
        if p.is_file() and not (set(p.relative_to(root).parts[:-1]) & SKIP_DIRS):
            out[str(p)] = {'sha256': sha(p), 'bytes': p.stat().st_size, 'mtime': p.stat().st_mtime}
    return out


def snapshot():
    data = {'created': time.strftime('%Y-%m-%d %H:%M:%S'), 'skip_dirs': sorted(SKIP_DIRS), 'roots': {}}
    for name, root in ROOTS.items():
        files = walk(root)
        data['roots'][name] = {'path': str(root), 'file_count': len(files), 'files': files}
        print(f'{name:28} {len(files):6d} files  {root}')
    MANIFEST.write_text(json.dumps(data, indent=1))
    # isolated copies (byte-identical; provenance recorded)
    prov = {}
    base = WS / 'model' / 'v7e_baseline'
    base.mkdir(parents=True, exist_ok=True)
    for f in ('sparrow_v7e.xml', 'sparrow_v7e_model.json', 'sparrow_sim.py'):
        shutil.copy2(MASTER / 'mujoco' / f, base / f)
        prov[str(base / f)] = {'from': str(MASTER / 'mujoco' / f), 'sha256': sha(base / f)}
    if not (base / 'meshes').exists():
        shutil.copytree(MASTER / 'mujoco' / 'meshes', base / 'meshes')
    prov[str(base / 'meshes')] = {'from': str(MASTER / 'mujoco' / 'meshes'), 'files': len(list((base / 'meshes').iterdir()))}
    vend = WS / 'src' / 'vendor'
    vend.mkdir(parents=True, exist_ok=True)
    for f in ('control.py', 'fast_aero.py', 'gains.json', 'torque_gains.json'):
        shutil.copy2(CONTROL / f, vend / f)
        prov[str(vend / f)] = {'from': str(CONTROL / f), 'sha256': sha(vend / f)}
    (WS / 'manifests' / 'copy_provenance.json').write_text(json.dumps(prov, indent=1))
    print('copies written; provenance in manifests/copy_provenance.json')


def verify():
    data = json.loads(MANIFEST.read_text())
    changed, missing, added = [], [], []
    for name, root in data['roots'].items():
        now = walk(Path(root['path']))
        for p, rec in root['files'].items():
            if p not in now:
                missing.append(p)
            elif now[p]['sha256'] != rec['sha256']:
                changed.append(p)
        added += [p for p in now if p not in root['files']]
    res = {'checked': sum(r['file_count'] for r in data['roots'].values()), 'changed': changed, 'missing': missing, 'added': added,
           'verified': time.strftime('%Y-%m-%d %H:%M:%S')}
    (WS / 'manifests' / 'verify_latest.json').write_text(json.dumps(res, indent=1))
    print(json.dumps({k: (v if not isinstance(v, list) else {'count': len(v), 'first': v[:10]}) for k, v in res.items()}, indent=1))


if __name__ == '__main__':
    {'snapshot': snapshot, 'verify': verify}[sys.argv[1]]()
