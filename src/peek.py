"""Print selected columns of a trace CSV at a coarse time step:  python peek.py <csv> <dt> <t0> <t1> col1 col2 ..."""
import csv, sys
path, dt, t0, t1 = sys.argv[1], float(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4]); cols = sys.argv[5:]
rows = list(csv.DictReader(open(path)))
print('t      ' + ' '.join(f'{c[-12:]:>12}' for c in cols))
nxt = t0
for r in rows:
    t = float(r['t'])
    if t >= nxt and t <= t1:
        nxt = t + dt
        print(f'{t:6.3f} ' + ' '.join(f'{float(r[c]):12.4f}' if r[c] not in ('', 'nan') and c != 'other_contacts' else f'{r[c]:>12}' for c in cols))
