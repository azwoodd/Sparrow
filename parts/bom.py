"""Whole-robot parts list and mass roll-up for Sparrow V7E, then the flight numbers at the resulting mass.
Reads the CAD mass register shipped with the model export (read-only). Every UNSELECTED slot gets a candidate real part.
Confidence of each real mass:  S = retailer/manufacturer listing read on 14 Nov 2026,  K = typical figure from general
knowledge, NOT yet checked against a listing,  T = no part chosen, the CAD target is kept,  C = calculated.
Writes parts/BOM.md and parts/bom.csv. Nothing is bought, nothing in CAD is changed."""
import json, csv, math
from pathlib import Path

HERE = Path(__file__).resolve().parent
info = json.load(open(HERE.parent / 'model' / 'v7e_stiffcontact' / 'sparrow_v7e_model.json'))
reg = info['mass_register']

# slot key (text contained in the register object name) -> (candidate, real mass g each, conf, price low, price high, note)
SEL = {
 'Hip servo':            ('KST X08H Plus (0.52 Nm at 8.4 V, 11.6 rad/s)', 9.5, 'S', 40, 55, 'needs 2S supply; PWM only, no readback'),
 'Knee servo':           ('KST X08H Plus', 9.5, 'S', 40, 55, 'same part as hip'),
 'Ankle servo':          ('Feetech SCS0009 (0.23 Nm at 6 V, serial bus)', 9.0, 'S', 15, 19, 'speed not listed; needs 6 V rail'),
 'Jump winder gearmotor':('N20-size metal gearmotor, 6 V', 10.0, 'K', 5, 15, 'ratio depends on spring energy and winding time'),
 'Knee jump spring':     ('NOT SELECTABLE YET - see spring note', 4.0, 'T', 2, 10, '2.2-3.3 J total needed'),
 'Flap motor':           ('1806-size brushless outrunner, ~2300 KV', 21.0, 'S', 12, 25, 'listings give 18-22 g and 80-170 W'),
 'Fold servo':           ('KST X08H V6 (about 0.27 Nm at 8.4 V)', 8.0, 'K', 35, 45, 'model asks 0.30 Nm; 2 g over target each'),
 'Tail servo':           ('4-5 g digital sub-micro servo (EMAX ES9051 class)', 4.3, 'K', 5, 10, 'about 0.08 Nm, model asks 0.10 Nm'),
 'Neck yaw servo':       ('Feetech SCS0009', 9.0, 'S', 15, 19, ''),
 'Head pitch servo':     ('Feetech SCS0009', 9.0, 'S', 15, 19, ''),
 'Beak servo':           ('3.7 g sub-micro servo', 3.7, 'K', 5, 12, ''),
 'Battery UNSELECTED':   ('Tattu 2S 650 mAh 75C LiPo (4.8 Wh)', 43.0, 'S', 9, 15, '57 x 31 x 12 mm'),
 'Flight controller':    ('ESP32-S3 module + 9-axis IMU breakout', 6.0, 'K', 20, 40, 'must also drive 13 servos + bus'),
 'Motor controller':     ('12-20 A brushless ESC', 6.0, 'K', 10, 20, ''),
 'Vision and audio computer': ('Raspberry Pi Zero 2 W', 11.0, 'K', 15, 20, ''),
 'Radio receiver':       ('ExpressLRS nano receiver', 1.0, 'K', 10, 20, ''),
 'Speaker and amp':      ('I2S amp board + 20 mm speaker', 6.0, 'K', 8, 15, ''),
 'Microphone':           ('I2S MEMS microphone breakout', 1.0, 'K', 4, 8, ''),
 'Wiring allowance':     ('wiring, connectors', 12.0, 'T', 10, 20, 'allowance kept'),
 'Charger and battery protection': ('2S protection/balance board', 2.5, 'T', 3, 8, ''),
 'Camera':               ('Pi Zero camera module', 3.0, 'K', 10, 25, ''),
 'Forward distance sensor': ('VL53L1X time-of-flight breakout', 1.0, 'K', 8, 15, ''),
 'Upward distance sensor':  ('VL53L1X time-of-flight breakout', 1.5, 'K', 8, 15, ''),
 'light ring':           ('addressable LED ring', None, 'T', 3, 6, 'target kept'),
 'Jaw return spring':    ('small extension spring', 0.3, 'T', 1, 2, ''),
 'Charge contacts':      ('pogo contacts', 0.8, 'T', 2, 5, ''),
}
# things the hop work or a real build needs that are NOT in the CAD register at all
MISSING = [
 ('Voltage regulators (6 V servo rail + 5 V logic)', 5.0, 'K', 8, 15, 'SCS0009 and Pi cannot run from 2S directly'),
 ('Fasteners, glue, heat-set inserts', 15.0, 'K', 5, 10, 'register notes say these are not counted; 3% allowance'),
 ('Hip pulleys and cables for the hip-and-knee spring', 4.0, 'C', 3, 8, 'as modelled in v7e_bispring'),
 ('Spring latch and release actuator', 6.0, 'K', 5, 15, 'not designed; a sub-micro servo plus latch parts'),
 ('Compliant servo couplings, 6 leg joints', 6.0, 'K', 5, 15, 'only if the compliance tests confirm them; 1 g each assumed'),
]

rows = []; slot_target = 0.0; slot_real = 0.0
for x in reg:
    if 'UNSELECTED' not in x['object'] and 'Wiring allowance' not in x['object']:
        continue
    if x['group'] == 'dock':
        continue
    key = next((k for k in SEL if k in x['object']), None)
    name, m, conf, plo, phi, note = SEL[key] if key else ('NO CANDIDATE', None, 'T', 0, 0, '')
    m = x['mass_g'] if m is None else m
    rows.append({'slot': x['object'].replace('V7 | ', '').replace(' UNSELECTED', ''), 'group': x['group'], 'target_g': x['mass_g'], 'part': name, 'real_g': m,
                 'conf': conf, 'price_lo': plo, 'price_hi': phi, 'note': note})
    slot_target += x['mass_g']; slot_real += m
structure = sum(x['mass_g'] for x in reg) - slot_target
missing = sum(m for _, m, *_ in MISSING)
total_target = sum(x['mass_g'] for x in reg)
total_real = structure + slot_real + missing
# heavier-leg option: Dynamixel XL330-M288-T on hips and knees (18 g, S)
total_optB = total_real + 4 * (18.0 - 9.5)

# ---------------------------------------------------------------- flight numbers at each mass
S = info['wing_area_cm2'] / 1e4 if info['wing_area_cm2'] > 1 else 0.166; b = info['wing_span_mm'] / 1000; rho = 1.225; g = 9.81
def flight(m_kg):
    W = m_kg * g; out = {'mass_g': m_kg * 1000, 'wing_loading': W / S}
    out['stall'] = {cl: math.sqrt(2 * W / (rho * S * cl)) for cl in (1.0, 1.2, 1.4)}
    AR = b * b / S; e = 0.8
    pw = {}
    for cd0 in (0.03, 0.08):
        best = min(((0.5 * rho * v ** 3 * S * cd0 + 2 * W ** 2 / (rho * v * math.pi * e * b * b)), v) for v in [x / 10 for x in range(40, 140)])
        pw[cd0] = best
    out['aero_power'] = pw
    out['elec_lo'] = pw[0.03][0] / (0.6 * 0.7); out['elec_hi'] = pw[0.08][0] / (0.25 * 0.5)
    return out
F = {k: flight(m / 1000) for k, m in (('CAD targets', total_target), ('real parts, light legs', total_real), ('real parts, Dynamixel hips/knees', total_optB))}
batt_wh = 7.4 * 0.65; usable = 0.8 * batt_wh

# ---------------------------------------------------------------- jump -> wings -> flap timing
hops = {'60 mm hop': 1.20, '100 mm hop': 1.47}          # take-off vertical speed, m/s, from the validated hip-and-knee spring hops
timing = {k: {'air_time_s': 2 * v / g, 'apex_rise_mm': v * v / (2 * g) * 1000} for k, v in hops.items()}
unfold_s = 1.518 / 11.6                                  # fold servo sweep 87 deg at the KST's unloaded speed; loaded will be slower

md = ['# Sparrow V7E - whole-robot parts list and mass roll-up (14 Nov 2026)', '',
      'Built by `parts/bom.py` from the CAD mass register in the model export. Desk study: nothing bought or weighed, no CAD change.',
      'Confidence of each real mass: **S** listing read today, **K** typical figure from general knowledge and not yet checked against a listing,',
      '**T** CAD target kept (no part chosen), **C** calculated. Prices are rough USD per item.', '',
      '## 1. Component slots', '', '| Slot | CAD target | Candidate | Real mass | Conf. | Price each | Note |', '|---|---:|---|---:|:-:|---:|---|']
for r in rows:
    md.append(f"| {r['slot']} | {r['target_g']:.1f} g | {r['part']} | {r['real_g']:.1f} g | {r['conf']} | ${r['price_lo']}-{r['price_hi']} | {r['note']} |")
md += ['', '## 2. Needed but missing from the CAD register', '', '| Item | Estimated mass | Conf. | Price | Note |', '|---|---:|:-:|---:|---|']
for n, m, c, lo, hi, note in MISSING:
    md.append(f'| {n} | {m:.1f} g | {c} | ${lo}-{hi} | {note} |')
plo = sum(r['price_lo'] for r in rows) + sum(x[3] for x in MISSING); phi = sum(r['price_hi'] for r in rows) + sum(x[4] for x in MISSING)
nS = sum(r['real_g'] for r in rows if r['conf'] == 'S'); nK = sum(r['real_g'] for r in rows if r['conf'] == 'K') + sum(m for _, m, c, *_ in MISSING if c == 'K')
md += ['', '## 3. Mass roll-up', '', '| | Mass |', '|---|---:|',
       f'| Printed / carbon / film structure (CAD volume estimates, unchanged) | {structure:.0f} g |',
       f'| Component slots at CAD targets | {slot_target:.0f} g |', f'| Component slots with candidate parts | {slot_real:.0f} g |',
       f'| Items missing from the register | {missing:.0f} g |', f'| **Total, CAD targets (what every simulation so far used)** | **{total_target:.0f} g** |',
       f'| **Total, candidate parts, light legs (KST)** | **{total_real:.0f} g** |', f'| Total with Dynamixel XL330 on hips and knees | {total_optB:.0f} g |', '',
       f'Of the purchased-part mass, {nS:.0f} g rests on listings read today and {nK:.0f} g on unchecked typical figures.',
       f'Rough purchased-parts cost, one robot, no spares, no printing or carbon: ${plo}-{phi}.', '',
       '## 4. Flight numbers at each mass', '', '| Case | Mass | Wing loading | Stall speed (max lift coeff. 1.0 / 1.2 / 1.4) | Least aerodynamic power | Electrical power, optimistic to pessimistic |', '|---|---:|---:|---|---|---|']
for k, f in F.items():
    md.append(f"| {k} | {f['mass_g']:.0f} g | {f['wing_loading']:.1f} N/m2 | {f['stall'][1.0]:.1f} / {f['stall'][1.2]:.1f} / {f['stall'][1.4]:.1f} m/s | "
              f"{f['aero_power'][0.03][0]:.1f}-{f['aero_power'][0.08][0]:.1f} W at {f['aero_power'][0.03][1]:.1f}-{f['aero_power'][0.08][1]:.1f} m/s | {f['elec_lo']:.0f}-{f['elec_hi']:.0f} W |")
fr = F['real parts, light legs']
md += ['', 'Same fixed-wing relations and assumptions as `flight_research/F0_FEASIBILITY_AUDIT.md` (Oswald 0.8, zero-lift drag 0.03-0.08,',
       'propulsive efficiency 0.25-0.6, drive efficiency 0.5-0.7). These are bounds, not a flight simulation.', '',
       f'- Battery: {batt_wh:.1f} Wh, {usable:.1f} Wh usable at 80%. Endurance {usable / fr["elec_hi"] * 60:.0f}-{usable / fr["elec_lo"] * 60:.0f} min across that power range, flap drive only.',
       '- Flap motor: an 1806-size motor is listed at 80-170 W, several times the cruise estimate, so motor power is not the limit.',
       '- Reference point: Festo SmartBird, 450 g with a 2.0 m span, is reported at 23 W for take-off and 12 W in flight. Sparrow carries',
       f'  more weight on a 1.42 m span, so its span loading is {(fr["mass_g"] / 1000 * g / b) ** 2 / (0.45 * g / 2.0) ** 2:.1f}x higher in the term that sets low-speed power.',
       '- Still unresolved and not fixed by any part choice: there is no wing-twist or other thrust mechanism in the design, and the',
       '  flap gearbox ratio is undefined. Until those exist, flight is not established.', '',
       '## 5. Jump, then wings, then flapping: the time available', '', '| | Air time | Rise of the body |', '|---|---:|---:|']
for k, t in timing.items():
    md.append(f"| {k} | {t['air_time_s']:.2f} s | {t['apex_rise_mm']:.0f} mm |")
md += ['', f'- Unfolding the wing is an 87 degree sweep. At the fold servo\'s unloaded speed that is {unfold_s:.2f} s; under load it will be longer.',
       '- One wingbeat at 2-4 Hz takes 0.25-0.50 s. So a 100 mm hop gives time for the wings to open and for roughly one downstroke, not more.',
       f'- The hop leaves the ground at 1.2-1.5 m/s. Wing-borne flight needs about {fr["stall"][1.2]:.1f} m/s. The hop supplies height and time, not flying speed.',
       '- So the sequence only works if the first wingbeats themselves carry most of the weight (near-hover power, estimated earlier at',
       '  40-130 W electrical for a few seconds), or if the wings are already open before the jump. Both need the thrust mechanism first.', '',
       '## 6. Spring note', '',
       'The CAD allows 4 g per knee spring. The hops need 2.2 J stored in total with ideal servos and 3.3 J with typical servo drag.',
       'At about 100 J/kg a steel spring for that is 22-33 g; a composite leaf at about 500 J/kg is 4-7 g; natural rubber at about 2000 J/kg',
       'is 1-2 g. So the 8 g spring budget only closes with an elastomer or composite spring, not steel.', '',
       '## 7. What to do before buying a full set', '',
       '1. Bench-test two KST X08H Plus and two Dynamixel XL330-M288-T (see `LEG_SERVO_SHORTLIST.md`).',
       '2. Replace every K row above with a real listing and, once bought, a weighed mass.',
       '3. Decide the supply: 2S direct for the KST servos and motor, regulated 6 V for bus servos, 5 V for the computer.',
       '4. Design the thrust mechanism and gearbox ratio; they decide whether section 4 means anything.', '',
       '## Sources read on 14 Nov 2026', '',
       '- https://www.buddyrc.com/products/kst-x08h-plus-digital-servo-8-4v-0-09s-5-3kg-cm-73oz-in', '- https://emanual.robotis.com/docs/en/dxl/x/xl330-m288/',
       '- https://openelab.io/products/feetech-scs0009-ultra-compact-servo', '- https://progressiverc.com/products/tattu-2s-650mah-75c',
       '- https://www.flyingtech.co.uk/product/dys-be1806-2300kv-brushless-motor-for-racing-drone/', '- https://www.buddyrc.com/products/sunnysky-x1806s-2300kv-brushless-motor-cw-on-sale',
       '- https://www.theengineer.co.uk/content/news/festos-biomechatronic-bird-flies-and-lands-autonomously', '- https://www.festo.com/smartbird']
(HERE / 'BOM.md').write_text('\n'.join(md) + '\n', encoding='utf-8')
with open(HERE / 'bom.csv', 'w', newline='', encoding='utf-8') as fh:
    w = csv.DictWriter(fh, fieldnames=list(rows[0])); w.writeheader(); w.writerows(rows)
print(f'slots {len(rows)}  structure {structure:.1f} g  slots target {slot_target:.1f} -> real {slot_real:.1f} g  missing {missing:.1f} g')
print(f'TOTAL target {total_target:.1f} g  real {total_real:.1f} g  optB {total_optB:.1f} g   cost ${plo}-{phi}')
for k, f in F.items():
    print(k, {a: (round(v, 2) if isinstance(v, float) else v) for a, v in f.items() if a in ('mass_g', 'wing_loading', 'elec_lo', 'elec_hi')}, {c: round(v, 2) for c, v in f['stall'].items()})
print('no candidate:', [r['slot'] for r in rows if r['part'] == 'NO CANDIDATE'])
print('timing', timing, 'unfold', round(unfold_s, 2))
