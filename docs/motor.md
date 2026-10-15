# Motor and drive — measurements

Evidence tags: [MEAS] measured, [DS] datasheet, [SPEC] vendor spec sheet, [HYP] hypothesis.

## Summary (servo_01, 2026-10-15)
| Parameter | Value | How obtained |
|---|---|---|
| Pole pairs | **1** | speed/voltage with and without gearbox, consistent with the spec sheet 0.18 s/60° at 12 V [MEAS] |
| Kv | **1466–1481 rpm/V** | free rotor, self-test [MEAS] |
| R phase-to-phase | **505 mΩ** at ~24 °C | locked shaft, 4.6 A, both directions [MEAS] |
| Gear ratio N | ≈ 210–223 | from Hall steps per ADC unit and 120° travel [HYP] |
| Hall steps per ADC unit | **0.206** (steady-state travel) | self-test; 0.22 including backlash |
| No-load friction | 0.85–0.89 A + 0.32–0.36 mA/(step/s) | self-test: Coulomb + viscous [MEAS] |
| Breakaway duty | 54–57 | self-test [MEAS] |
| Backlash on reversal | **38–48 Hall steps ≈ 12° at the output** | pushes after reversal, three points of travel [MEAS] |
| Rotor Kt | 6.5 mN·m/A | 60/(2π·Kv) [calc] |
| Winding L, thermal constants | — | not measured |

## No-load run on stock firmware (servo_01, 2026-10-15)
Conditions: potentiometer removed (position loop open), `tools/spin_native.py 10 300 2100` —
Torque Limit 300/1023, Goal 2100 µs, output unloaded, bench PSU.

| Parameter | Value | Confidence |
|---|---|---|
| Bus voltage (servo sensor) | 20.7 V without torque → 20.3–20.5 V running | [MEAS] |
| PSU voltage | **24.0 V** (servo sensor reads 20.7 meanwhile → under-reads by ≈1.16×) | [MEAS] |
| PSU current while running | **330–340 mA** (10 s and 20 s runs) | [MEAS] |
| Present Load without torque | ~126–135 (current sensor zero offset) | [MEAS] |
| Present Load while running | ~270–330, briefly up to ~400 | [MEAS] |
| No-load power | ≈ 8.1 W (24.0 V × 0.34 A) at TL=300 | calc |
| Current fluctuations | waves ~250 → ~400 → ~250 over a few seconds (repeated in 10 s and 20 s runs) | [MEAS] — mechanics? |

## Current sensor scale [HYP]
- Firmware: Present Load = ADC11 × 3.2327 (double), overload threshold 9500 > 1 s.
- Logic consumption (torque off): **12 mA** at 24.0 V [MEAS] → motor at no load ≈ 328 mA from the bus.
- Present Load: zero offset ~130–140, running ~250–330 (average ~290) → increase ~155 units ↔ 328 mA average bus current.
  The "Load ≈ mA" hypothesis **was not confirmed**. Probably the ADC samples synchronously with PWM (in the "switch on" phase) —
  then Load ∝ phase current, not average bus current (phase current ≈ bus current / duty). Points at different
  Torque Limit (duty) and under load are needed to separate the two options.
- Overload threshold 9500 units > 1 s — in amps still unknown.

### Calibration from three runs (24.0 V, 12 mA logic subtracted) [MEAS]
| Torque Limit | Motor current (PSU) | Present Load avg | Load − 135 | units/mA |
|---|---|---|---|---|
| 150 | ~144 mA | 198 | ~63 | 0.44 |
| 300 | ~324 mA | 284 | ~149 | 0.46 |
| 600 | ~488 mA steady-state (PSU 500 mA); **start — peak >1 A** (with 1 A PSU limit → sag → MCU reset) | ~390 | ~255 | 0.52 |

→ Present Load is ≈ linear in **average bus current**: ≈ **2.0 mA/unit** (0.44–0.52 units/mA) above the ~135 offset [HYP, 3 points].
Overload threshold 9500 ≈ 19 A (on this scale). No-load current: 144 → 324 → 488 mA at TL 150/300/600 (≈12 W at 600).
Inrush current >2× steady-state — set the PSU limit with margin (≥2 A), otherwise the MCU resets.
After the reset the MCU came up on its own, Torque Enable = 0, no errors.

## Halls, Kv, pole pairs — firmware v1 on servo_01 (2026-10-15, 24.0 V, potentiometer in place) [MEAS]
Logs `dumps/servo_01/tests/pos_clone_tl{300,600}_20261015_08*.csv`, steps of 1.5 s, 20 ms sampling.

| Duty | Winding U (Vbus·D − I·R) | Rotor speed | Winding current (estimate) |
|---|---|---|---|
| 29 % (TL 300) | ≈ 6.3–6.4 V | 915–970 Hall steps/s | 1.0–1.2 A |
| 58 % (TL 600) | ≈ 13 V | 1830–1930 steps/s | 0.9–1.5 A |

- Speed ∝ voltage (×2.0 at ×2 U): motion is limited by Kv, not by the controller.
- **Hall steps per potentiometer ADC unit = 0.222** (373 steps per 1677 units, repeatable in both directions).
- Kv = speed / (6·pp) · 60 / U: **pp = 1 → 1420–1520 rpm/V; pp = 2 → 710–760**.

Which hypothesis holds is decided by the physical travel angle. If 1000→2000 µs = ~100° (120° full travel, as in the spec sheet),
then ADC units per degree 16.8 → steps per output revolution 1340 → **pp = 1, N ≈ 223, Kv ≈ 1470** [HYP, strong]:
- consistent with the spec sheet 0.18 s/60° at 12 V (we have 0.11 s/60° at 13 V without load),
- consistent with stall torque: Kt = 6.5 mN·m/A × 15 A × 223 × 0.75 ≈ 165 kg·cm (spec sheet 200),
- consistent with tooth counts from photos with 17T small gears: 55/18 · 60/17 · 85/17 · 70/17 ≈ 222.
If instead 1000→2000 travel = ~50°, then pp = 2, N ≈ 274. (The number 274 from an early journal analysis was the bytes
"version 0x12, ID 0x01" read as u16, unrelated to N.)

**To check:** rocker arm angle between 1000 and 2000 µs with a protractor (`tools/pos_test.py --tl 300 1000 2000`).

### Bench test without gearbox (2026-10-15, firmware v1.5, wheel mode) [MEAS]
| duty | Winding U | Rotor speed | Bus current | Kv (pp = 1) |
|---|---|---|---|---|
| 150 | 3.5 V | 440 steps/s (4400 rpm) | 200 mA | ~1330 |
| 300 | 7.0 V | 915 steps/s (9150 rpm) | 450 mA | ~1475 (accounting for I·R at R 0.5 Ω) |
Free rotor: Kv ≈ 1475 rpm/V matches the measurement with gearbox → **pp = 1** [MEAS, two independent bench tests].
Sinusoidal commutation at the same duty: 97 % speed, +10 % bus current; Hall offset +6° el.

### Locked rotor (2026-10-15, `tools/lockrotor_test.py`) [MEAS]
duty ±100: U = 2.32 V, I_wind = 4.59–4.60 A, ω = 0 → **R phase-to-phase = 504–505 mΩ** both directions at ~24 °C
(online estimate while running 503–512; one point at duty 80 in another sector gave 534 — a different phase pair or sensor nonlinearity).
At duty 120 (≈5.6 A, ~36 mN·m at the rotor) the rotor can no longer be held by hand. R_REF in the journal = 505.

### Gearbox backlash (2026-10-15, after two rebuilds) [MEAS]
Push at duty 150 after reversal: the rotor travels **38–48 Hall steps (≈7.5 revolutions) before the potentiometer moves by 10 units**, the same in both
directions and at points 1500/2050/2600; without reversal — 3–5 steps. At N ≈ 223 this is ≈ 12° at the output, taken up in ~100 ms.
Self-test gives the same number (41–43). Steps/unit on steady-state travel 195–206 (the earlier 222 included backlash).
Where to look: potentiometer gear mesh and gearbox seating (it came apart twice that day); compare with another servo.
At the output this is backlash whenever the load direction changes — it can only be compensated with a second sensor (encoder on the output).

### Implications for speed (at pp = 1, N = 223)
| Vbus | Rotor no-load | Output | s/60° |
|---|---|---|---|
| 12 V | 17.6k rpm | 79 rpm | 0.13 |
| 24 V | 35k | 158 | 0.063 |
| 48 V | 70k — the rotor will not survive that, limit ~30–35k needed | ~150 | ~0.065 |
Conclusion: 48 V does not add output speed beyond 24 V, rotor speed is the limit; it adds torque at speed.
For output speed above ~150 rpm a different gearbox is needed, not voltage.
