# HB200T servo model for calculations (summary, 2026-10-18)

Evidence tags: [MEAS] measured, [calc] derived from measurements, [HYP] estimate, [SPEC] vendor spec sheet, [DS] datasheet.
"Output" means the position-sensor gear (1:1 with the output shaft by eye, not measured). Test units: servo_01, servo_02,
custom firmware, unloaded output, bench PSU unless noted.

## Motor (rotor)
| Parameter | Value | |
|---|---|---|
| Pole pairs | 1 | [MEAS] |
| Kv | 1340 rpm/V at high speed (servo_02); 1466–1481 at low speed (servo_01, self-test) | [MEAS] |
| Kt = 60/(2π·Kv) | 7.1 mN·m/A (Kv 1340); 6.5 (Kv 1475) | [calc] |
| R phase-to-phase | 0.505 Ω at 24 °C; copper +0.39 %/K → ~0.66 Ω at 100 °C | [MEAS] / [calc] |
| L | not measured | |
| J of rotor + gearbox, referred to the rotor | ≈ 1.0·10⁻⁶ kg·m² (±25 %) — from acceleration: 0 → 90 % speed in 65–69 ms at 8 A and in 103 ms at 5 A | [calc] |
| No-load current (motor + gearbox) | 0.85–0.89 A + 0.32–0.36 mA per step/s (1 step/s = 10 rpm of the rotor) | [MEAS, servo_01] |
| Breakaway | duty 54–57 of 1023 at 24 V (≈ 2.5 A at standstill) | [MEAS] |

## Gearbox
| Parameter | Value | |
|---|---|---|
| N | **206.0** rotor turns per output turn (Hall steps vs AS5600, 6 turns both ways) | [MEAS] |
| Efficiency | not measured; 0.7–0.9 (the 200 kg·cm stall at 15 A spec implies 0.89) | [HYP] |
| Dead band rotor → sensor | 36–48 Hall steps ≈ 10–14° at the output (may partly be the sensor gear) | [MEAS] |
| Back-drivable | turns by hand from the output | [MEAS] |

## Output side (derived)
| Parameter | Formula / value |
|---|---|
| No-load speed | ω₀ ≈ 39.0 °/s per volt across the winding (6·Kv/N); 24 V → 925 °/s [MEAS] |
| Torque per winding amp | k = Kt·N·η = 1.47·η N·m/A → 1.03 (η 0.7) … 1.31 (η 0.89) N·m/A; minus ~0.85 A no-load current |
| Motor inertia referred to the output | J·N² ≈ 0.042 kg·m² (like 1 kg at a 20 cm radius) |
| Torque–speed (full duty) | I(ω) = (V − ω·N/(6·Kv))/R, ω in °/s at the output; T = k·(min(I, I_lim) − I₀) |
| Equation of motion (output side, rad/s²) | Kt·N·(I − I₀) = J_rotor·N²·α + (J_load·α + T_load)/η — the rotor inertia sits BEFORE the gearbox, η does not apply to it; with a light load most of the current accelerates the rotor (0.042 kg·m²) |
| Voltage needed for current I at speed ω | V ≥ I·R + ω/39.0 (ω in °/s at the output): 25 A at 800 °/s → ≥ 33 V (hot winding ~37 V); at 24 V, 25 A is available only up to ~440 °/s |

Output speed up to which the full current I_lim is available: ω_c = 39.0·(V − I_lim·R) °/s (R = 0.505; hot winding ~5–10 % lower) [calc]:

| Vbus | ω₀ | ω_c at 8 A | ω_c at 15 A |
|---|---|---|---|
| 22.2 V | 866 | 709 | 571 |
| 25.2 V | 984 | 826 | 688 |
| 36 V | 1405 | 1247 | 1110 |
| 42 V | 1639 | 1482 | 1343 |
| 44.4 V | 1733 | 1575 | 1437 |
| 50.4 V | 1967 | 1809 | 1671 |

## Heat
| Parameter | Value |
|---|---|
| Copper loss | I²·R: 8 A → 32 W, 15 A → 114 W |
| Continuous current / thermal time constants | not measured; continuous torque estimated at 15–50 kg·cm depending on cooling [HYP] |

## Bench results (servo_02, unloaded; PSU: 8 A current cap, braking without regeneration) [MEAS]
| What | 24 V | 31.5 V |
|---|---|---|
| Max output speed | 154 rpm (925 °/s) | 205 rpm (1230 °/s), sine commutation with +24° advance |
| Acceleration to 90 % | 65–69 ms (8 A), 103 ms (5 A), 137–159 ms (3 A) | 67–75 ms (8 A) |
| Step to ±1°, Kp 15 / Kd 1.0 | 90° 208 ms, 180° 301, 270° 400, 360° ~500 | 90° 197, 180° 273 |
| Braking by shorting the low-side switches from full speed | 43–51 ms | 58 ms (from 41.7k rpm) |

The steps are limited by the bench (8 A cap, regeneration limiter because the PSU cannot sink energy, gains in duty units) —
do not use them to judge the effect of bus voltage. Physically, voltage buys torque at speed: the current available at speed ω is
(V − back-EMF)/R.

## Extrapolation to 36 V [calc, linear model, not measured]
| What | 36 V | 42 V (10S full) |
|---|---|---|
| No-load output speed | ~1350 °/s six-step, ~1400 sine (225–233 rpm; rotor 46–48k rpm) | ~1580–1640 °/s (rotor 54–56k rpm — above the tested 45k) |
| Full 8 A available up to | 1250 °/s (hot winding 1200) | 1480 |
| Full 15 A available up to | 1110 °/s (hot 1020) | 1340 |
| Torque at 8 A / 15 A | ~8.0 / ~15.8 N·m (82 / 161 kg·cm) at η 0.8; 6.7–9.4 / 13.2–18.7 over the η range — independent of voltage | same |
| Stall current without a limit | ≈ 71 A → software current cap required | ≈ 83 A |
| Unloaded output acceleration | 8 A: ~14 000 °/s², to 1250 °/s in ~90 ms; 15 A: ~28 000 °/s², to 1110 °/s in ~40 ms | |
| Braking by shorting the phases from full speed | ~60–70 ms (43–58 ms measured at 24–31.5 V) | |
| Margin of the 60 V switches | 1.67× | 1.43× |

Board for 36–42 V: input electrolytic ≥ 63 V, logic buck catch diode a Schottky ≥ 60 V (100 V preferred; a slow M7/1N4007 does not work),
bus ceramics ≥ 63 V. Gains are in duty units — scale Kp/Kd by 24/V to keep the same stiffness (×0.67 at 36 V). The model ignores
inductance — at high speed the real current and torque are lower than calculated.

## Electrical limits of the board
| What | Value |
|---|---|
| Switches | NCEP6090AGU 60 V / 90 A / 2.8 mΩ, 2 in parallel per bridge arm [DS] |
| Shunt | 2.5 mΩ in the common bus [MEAS] |
| Spec sheet | stall 15 A, 200 kg·cm; 12–24 V; 0.18 s/60° [SPEC] |
| Stock parts | SS34 40 V, 35 V electrolytic → ≤ ~30 V without rework; Vbus is measured up to 53.7 V [MEAS] |

## Sensors and control loop
| What | Value |
|---|---|
| Hall sensors | 6 steps per rotor turn → 1236 per output turn (0.29° of output per step) |
| AS5600 | 4096 per turn (0.088°), I2C, read at 1 kHz |
| Loop | 1 kHz; PWM duty 0..1023 of Vbus (Kp/Kd in duty units — stiffness grows with Vbus) |
| Mass / size | 435 g, 64×33×81 mm [SPEC] |

## Not measured (needed for an honest model)
Gearbox efficiency, winding inductance, thermal time constants and continuous current, dead band at the output shaft itself,
acceleration and braking without the bench limits (battery supply, 15 A, regeneration into the battery).

Fast moves (rotor inertia, RMS current, voltage needed for current at speed, trajectory shapes) — [dynamics.md](dynamics.md).
