# Sparrow V7E - whole-robot parts list and mass roll-up (14 Nov 2026)

Built by `parts/bom.py` from the CAD mass register in the model export. Desk study: nothing bought or weighed, no CAD change.
Confidence of each real mass: **S** listing read today, **K** typical figure from general knowledge and not yet checked against a listing,
**T** CAD target kept (no part chosen), **C** calculated. Prices are rough USD per item.

## 1. Component slots

| Slot | CAD target | Candidate | Real mass | Conf. | Price each | Note |
|---|---:|---|---:|:-:|---:|---|
| G03 | Flap motor | 22.0 g | 1806-size brushless outrunner, ~2300 KV | 21.0 g | S | $12-25 | listings give 18-22 g and 80-170 W |
| E01 | Battery | 45.0 g | Tattu 2S 650 mAh 75C LiPo (4.8 Wh) | 43.0 g | S | $9-15 | 57 x 31 x 12 mm |
| E02 | Flight controller | 8.0 g | ESP32-S3 module + 9-axis IMU breakout | 6.0 g | K | $20-40 | must also drive 13 servos + bus |
| E03 | Motor controller | 8.0 g | 12-20 A brushless ESC | 6.0 g | K | $10-20 |  |
| E04 | Vision and audio computer | 12.0 g | Raspberry Pi Zero 2 W | 11.0 g | K | $15-20 |  |
| E05 | Radio receiver | 3.0 g | ExpressLRS nano receiver | 1.0 g | K | $10-20 |  |
| E06 | Speaker and amp | 8.0 g | I2S amp board + 20 mm speaker | 6.0 g | K | $8-15 |  |
| E07 | Microphone | 1.0 g | I2S MEMS microphone breakout | 1.0 g | K | $4-8 |  |
| E08 | Wiring allowance | 12.0 g | wiring, connectors | 12.0 g | T | $10-20 | allowance kept |
| E10 | Charger and battery protection | 2.5 g | 2S protection/balance board | 2.5 g | T | $3-8 |  |
| A13b | Neck light ring | 0.5 g | addressable LED ring | 0.5 g | T | $3-6 | target kept |
| T01 | Tail servo L | 4.5 g | 4-5 g digital sub-micro servo (EMAX ES9051 class) | 4.3 g | K | $5-10 | about 0.08 Nm, model asks 0.10 Nm |
| T01 | Tail servo R | 4.5 g | 4-5 g digital sub-micro servo (EMAX ES9051 class) | 4.3 g | K | $5-10 | about 0.08 Nm, model asks 0.10 Nm |
| W05 | Fold servo L | 6.0 g | KST X08H V6 (about 0.27 Nm at 8.4 V) | 8.0 g | K | $35-45 | model asks 0.30 Nm; 2 g over target each |
| W05 | Fold servo R | 6.0 g | KST X08H V6 (about 0.27 Nm at 8.4 V) | 8.0 g | K | $35-45 | model asks 0.30 Nm; 2 g over target each |
| L00 | Jump winder gearmotor | 10.0 g | N20-size metal gearmotor, 6 V | 10.0 g | K | $5-15 | ratio depends on spring energy and winding time |
| L01 | Hip servo L | 14.0 g | KST X08H Plus (0.52 Nm at 8.4 V, 11.6 rad/s) | 9.5 g | S | $40-55 | needs 2S supply; PWM only, no readback |
| L04 | Knee servo L | 14.0 g | KST X08H Plus | 9.5 g | S | $40-55 | same part as hip |
| L08 | Ankle servo L | 9.0 g | Feetech SCS0009 (0.23 Nm at 6 V, serial bus) | 9.0 g | S | $15-19 | speed not listed; needs 6 V rail |
| L09 | Knee jump spring L | 4.0 g | NOT SELECTABLE YET - see spring note | 4.0 g | T | $2-10 | 2.2-3.3 J total needed |
| L01 | Hip servo R | 14.0 g | KST X08H Plus (0.52 Nm at 8.4 V, 11.6 rad/s) | 9.5 g | S | $40-55 | needs 2S supply; PWM only, no readback |
| L04 | Knee servo R | 14.0 g | KST X08H Plus | 9.5 g | S | $40-55 | same part as hip |
| L08 | Ankle servo R | 9.0 g | Feetech SCS0009 (0.23 Nm at 6 V, serial bus) | 9.0 g | S | $15-19 | speed not listed; needs 6 V rail |
| L09 | Knee jump spring R | 4.0 g | NOT SELECTABLE YET - see spring note | 4.0 g | T | $2-10 | 2.2-3.3 J total needed |
| H01 | Neck yaw servo | 9.0 g | Feetech SCS0009 | 9.0 g | S | $15-19 |  |
| H05 | Head pitch servo | 9.0 g | Feetech SCS0009 | 9.0 g | S | $15-19 |  |
| H27 | Upward distance sensor | 1.5 g | VL53L1X time-of-flight breakout | 1.5 g | K | $8-15 |  |
| H28 | Crown light ring | 0.4 g | addressable LED ring | 0.4 g | T | $3-6 | target kept |
| H16 | Beak servo | 3.5 g | 3.7 g sub-micro servo | 3.7 g | K | $5-12 |  |
| H19 | Jaw return spring | 0.3 g | small extension spring | 0.3 g | T | $1-2 |  |
| H10 | Camera | 3.0 g | Pi Zero camera module | 3.0 g | K | $10-25 |  |
| H11 | Forward distance sensor | 1.0 g | VL53L1X time-of-flight breakout | 1.0 g | K | $8-15 |  |
| H09 | Eye light ring L | 1.0 g | addressable LED ring | 1.0 g | T | $3-6 | target kept |
| H09 | Eye light ring R | 1.0 g | addressable LED ring | 1.0 g | T | $3-6 | target kept |
| E09 | Charge contacts | 0.8 g | pogo contacts | 0.8 g | T | $2-5 |  |

## 2. Needed but missing from the CAD register

| Item | Estimated mass | Conf. | Price | Note |
|---|---:|:-:|---:|---|
| Voltage regulators (6 V servo rail + 5 V logic) | 5.0 g | K | $8-15 | SCS0009 and Pi cannot run from 2S directly |
| Fasteners, glue, heat-set inserts | 15.0 g | K | $5-10 | register notes say these are not counted; 3% allowance |
| Hip pulleys and cables for the hip-and-knee spring | 4.0 g | C | $3-8 | as modelled in v7e_bispring |
| Spring latch and release actuator | 6.0 g | K | $5-15 | not designed; a sub-micro servo plus latch parts |
| Compliant servo couplings, 6 leg joints | 6.0 g | K | $5-15 | only if the compliance tests confirm them; 1 g each assumed |

## 3. Mass roll-up

| | Mass |
|---|---:|
| Printed / carbon / film structure (CAD volume estimates, unchanged) | 254 g |
| Component slots at CAD targets | 266 g |
| Component slots with candidate parts | 239 g |
| Items missing from the register | 36 g |
| **Total, CAD targets (what every simulation so far used)** | **519 g** |
| **Total, candidate parts, light legs (KST)** | **529 g** |
| Total with Dynamixel XL330 on hips and knees | 563 g |

Of the purchased-part mass, 138 g rests on listings read today and 107 g on unchecked typical figures.
Rough purchased-parts cost, one robot, no spares, no printing or carbon: $482-793.

## 4. Flight numbers at each mass

| Case | Mass | Wing loading | Stall speed (max lift coeff. 1.0 / 1.2 / 1.4) | Least aerodynamic power | Electrical power, optimistic to pessimistic |
|---|---:|---:|---|---|---|
| CAD targets | 519 g | 30.7 N/m2 | 7.1 / 6.5 / 6.0 m/s | 2.0-2.6 W at 5.5-4.3 m/s | 5-21 W |
| real parts, light legs | 529 g | 31.3 N/m2 | 7.1 / 6.5 / 6.0 m/s | 2.1-2.7 W at 5.5-4.3 m/s | 5-21 W |
| real parts, Dynamixel hips/knees | 563 g | 33.3 N/m2 | 7.4 / 6.7 / 6.2 m/s | 2.3-2.9 W at 5.7-4.5 m/s | 5-23 W |

Same fixed-wing relations and assumptions as `flight_research/F0_FEASIBILITY_AUDIT.md` (Oswald 0.8, zero-lift drag 0.03-0.08,
propulsive efficiency 0.25-0.6, drive efficiency 0.5-0.7). These are bounds, not a flight simulation.

- Battery: 4.8 Wh, 3.8 Wh usable at 80%. Endurance 11-46 min across that power range, flap drive only.
- Flap motor: an 1806-size motor is listed at 80-170 W, several times the cruise estimate, so motor power is not the limit.
- Reference point: Festo SmartBird, 450 g with a 2.0 m span, is reported at 23 W for take-off and 12 W in flight. Sparrow carries
  more weight on a 1.42 m span, so its span loading is 2.7x higher in the term that sets low-speed power.
- Still unresolved and not fixed by any part choice: there is no wing-twist or other thrust mechanism in the design, and the
  flap gearbox ratio is undefined. Until those exist, flight is not established.

## 5. Jump, then wings, then flapping: the time available

| | Air time | Rise of the body |
|---|---:|---:|
| 60 mm hop | 0.24 s | 73 mm |
| 100 mm hop | 0.30 s | 110 mm |

- Unfolding the wing is an 87 degree sweep. At the fold servo's unloaded speed that is 0.13 s; under load it will be longer.
- One wingbeat at 2-4 Hz takes 0.25-0.50 s. So a 100 mm hop gives time for the wings to open and for roughly one downstroke, not more.
- The hop leaves the ground at 1.2-1.5 m/s. Wing-borne flight needs about 6.5 m/s. The hop supplies height and time, not flying speed.
- So the sequence only works if the first wingbeats themselves carry most of the weight (near-hover power, estimated earlier at
  40-130 W electrical for a few seconds), or if the wings are already open before the jump. Both need the thrust mechanism first.

## 6. Spring note

The CAD allows 4 g per knee spring. The hops need 2.2 J stored in total with ideal servos and 3.3 J with typical servo drag.
At about 100 J/kg a steel spring for that is 22-33 g; a composite leaf at about 500 J/kg is 4-7 g; natural rubber at about 2000 J/kg
is 1-2 g. So the 8 g spring budget only closes with an elastomer or composite spring, not steel.

## 7. What to do before buying a full set

1. Bench-test two KST X08H Plus and two Dynamixel XL330-M288-T (see `LEG_SERVO_SHORTLIST.md`).
2. Replace every K row above with a real listing and, once bought, a weighed mass.
3. Decide the supply: 2S direct for the KST servos and motor, regulated 6 V for bus servos, 5 V for the computer.
4. Design the thrust mechanism and gearbox ratio; they decide whether section 4 means anything.

## Sources read on 14 Nov 2026

- https://www.buddyrc.com/products/kst-x08h-plus-digital-servo-8-4v-0-09s-5-3kg-cm-73oz-in
- https://emanual.robotis.com/docs/en/dxl/x/xl330-m288/
- https://openelab.io/products/feetech-scs0009-ultra-compact-servo
- https://progressiverc.com/products/tattu-2s-650mah-75c
- https://www.flyingtech.co.uk/product/dys-be1806-2300kv-brushless-motor-for-racing-drone/
- https://www.buddyrc.com/products/sunnysky-x1806s-2300kv-brushless-motor-cw-on-sale
- https://www.theengineer.co.uk/content/news/festos-biomechatronic-bird-flies-and-lands-autonomously
- https://www.festo.com/smartbird
