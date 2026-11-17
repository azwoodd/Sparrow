# Sparrow leg servos - first real-parts shortlist (14 Nov 2026)

Status: desk research from retailer listings and one manufacturer manual. Nothing bought, nothing bench-tested, no CAD
changed. Prices are listing prices seen on 14 Nov 2026 and vary a lot between shops. Scope is the six leg servos only,
because they decide whether the hop results carry over to hardware. Wings, tail, head, battery, spring, latch and
winder are not covered yet.

## What the simulation asks of a leg servo

| Requirement | Hip / knee | Ankle | Where it comes from |
|---|---|---|---|
| Stall torque | 0.35 Nm (3.6 kg cm) | 0.20 Nm (2.0 kg cm) | model actuator limits |
| No-load speed | about 10 rad/s (0.10 s/60 deg) | same | hop controller slew limit |
| Mass | thigh link is 17 g, shin 14.6 g, foot 9 g in the model, servo included | | model link masses |
| Survives being back-driven at ~30 rad/s for ~20 ms | yes | yes | the spring push |
| Gear train survives 1-2.5 Nm peaks (rigid coupling) | | | sections S and T of REPORT.md |
| Position readback, ability to go limp or follow | wanted | wanted | hop controller "follow" mode |

The last three rows are not on any datasheet. They need a bench test (below).

## Candidates

| Servo | Type | Stall torque | No-load speed | Mass | Size (mm) | Price seen | Fits |
|---|---|---|---|---|---|---|---|
| KST X08H Plus | PWM, coreless, steel gears | 0.52 Nm at 8.4 V; 0.38 Nm at 6 V; 0.24 Nm at 3.8 V | 0.09 s/60 deg at 8.4 V (11.6 rad/s); 0.15 s at 6 V (7.0 rad/s) | 9.5 g | 23.5 x 8 x 17 | $40-55 | hip/knee at 6-8.4 V |
| ROBOTIS Dynamixel XL330-M288-T | serial bus, encoder, torque/position modes | 0.52 Nm at 5 V | 103 rpm at 5 V (10.8 rad/s) | 18 g | 20 x 34 x 26 | $27.49 (US, sold out when checked); EUR 40.20 inc. VAT | hip/knee, heavy for the links |
| Feetech STS3032 | serial bus, feedback | 0.44 Nm at 6 V (rated 0.15 Nm) | 0.09 s/60 deg at 6 V (11.6 rad/s) | 20 g | 23.2 x 12.1 x 28.5 | not confirmed | hip/knee, heavy for the links |
| Feetech SCS0009 | serial bus, 10-bit feedback, metal gears | 0.23 Nm at 6 V (one listing says 0.15) | not listed | 9 g | 23.2 x 12.1 x 25.3 | $15-19; GBP 19.50 | ankle |
| EMAX ES08MA II | PWM analogue, metal gears | 0.20 Nm at 6 V; 0.16 Nm at 4.8 V | 0.10 s/60 deg at 6 V (10.5 rad/s) | 12 g | 23 x 11.5 x 24 | not confirmed (low-cost class) | ankle, no margin |

Listings disagree in places (SCS0009 torque, STS3032 speed), so treat each number as "to be confirmed on the bench".

## What each choice means for Sparrow

- **KST X08H Plus (hip and knee).** The only candidate that meets torque and speed inside the mass the model already
  assumes. Needs a 2S supply (7.4-8.4 V) to reach both; at 6 V it has the torque but is 30% slower than assumed.
  Plain PWM: no position readback and no commanded "limp" state, so the hop's follow mode would have to be done by
  cutting the signal, and behaviour with no signal is not documented.
- **Dynamixel XL330-M288-T (hip and knee).** Meets torque and speed at 5 V, reports position and has current/PWM
  modes, which is what the hop controller wants. Costs about +8 to +9 g per joint over the model (about +50 g for six,
  10% of the robot), and that added leg mass has not been simulated.
- **Feetech STS3032.** Same role and mass penalty as the XL330; price and real speed unconfirmed.
- **Ankle.** SCS0009 gives 15% torque margin with feedback at 9 g. The ES08MA II has no margin at 6 V.

## Budget, legs only (six servos, listing prices)

| Option | Parts | Rough cost |
|---|---|---|
| A: light, PWM | 4 x KST X08H Plus + 2 x SCS0009 | $190-260 |
| B: feedback everywhere | 4 x XL330-M288-T + 2 x SCS0009 | $140-150, plus a bus adapter |
| C: lowest cost check | 4 x STS3032 + 2 x SCS0009 | not priced (STS3032 price unconfirmed) |

Add spares: the simulated gear loads are several times stall torque, so expect to break some.

## What no datasheet gives, and the bench test that does

The hop simulation is sensitive to three servo properties that are currently assumed ranges:

| Property | Assumed range in simulation | Bench measurement |
|---|---|---|
| Rotor inertia reflected at the output | 1e-4 to 1e-3 kg m^2 | hang a known mass on a horn, release, time the swing with the servo unpowered |
| Back-drive friction (dry) | 0.02-0.07 Nm | torque needed to start turning the unpowered horn (small spring scale) |
| Drag when driven faster than no-load speed | stall / no-load speed | spin the horn with a drill at known rpm, measure torque; repeat powered, limp and signal-off |
| Gear strength under impact | unknown | drop a known mass on a horn-length arm, step the height until it strips |

One more thing to check on the bench before a spring is fitted: driving a geared DC servo at about three times its
no-load speed makes it act as a generator at well above its supply voltage. That is my engineering expectation, not a
measured fact for these parts; it could stress the servo electronics or the supply rail.

## Recommendation

Buy two servos of each of two types for the bench before committing to six: KST X08H Plus (fits the mass) and
Dynamixel XL330-M288-T (fits the control). About $135-165 at listing prices. Measure the four properties above, put
the measured numbers into `src/v9_backdrive.py` in place of the assumed "typical" case, and re-run the hop validation.
That is the step that turns the hop results from "works with assumed parts" into "works with these parts", or shows
which mechanical change (compliant coupling, hip spring split) is needed.

## Sources

- https://emanual.robotis.com/docs/en/dxl/x/xl330-m288/
- https://robotis.us/dynamixel-xl330-m288-t
- https://www.generationrobots.com/en/403817-dynamixel-xl330-m288-t-servo.html
- https://www.buddyrc.com/products/kst-x08h-plus-digital-servo-8-4v-0-09s-5-3kg-cm-73oz-in
- https://www.flashrc.com/en/rc-servos-precision-reliability-for-modeling/26825-kst-x08h-plus-mg-hv-micro-servo-95g-53kgcm-009s-60-4897073541273.html
- https://www.espritmodel.com/kst-x08-h-plus-high-torque-coreless-micro-thin-wing-lv-hv-digital-servo-8mm.aspx
- https://www.robotshop.com/products/feetech-6v-45-kg-serial-bus-servo
- https://openelab.io/products/feetech-scs0009-ultra-compact-servo
- https://shop.sb-components.co.uk/products/feetech-scs009-6v-2-3kg-serial-bus-servo-metal-gearbox-servos-for-rc-h
- https://www.racer.lt/item/emax-es08ma-ii-12g-micro-metal-gear-pwm-reverse-analog-servo-for-rc-models-24303.html
