"""Print the CAD mass register that came with the V7E model export (read-only)."""
import json, collections
from pathlib import Path
d = json.load(open(Path(__file__).resolve().parents[1] / 'model' / 'v7e_stiffcontact' / 'sparrow_v7e_model.json'))
r = d['mass_register']
print(len(r), 'rows, total', round(sum(x['mass_g'] for x in r), 1), 'g')
for k, v in collections.Counter(x['basis'][:70] for x in r).most_common():
    print(f'  {v:3d} x  {k}')
for x in r:
    if 'CAD volume' not in x['basis'] or x['mass_g'] >= 5:
        print(f"{x['mass_g']:6.2f} g | {x['group']:22} | {x['body']:16} | {x['object'][:58]:58} | {x['basis'][:80]}")
print(d['notes'])
